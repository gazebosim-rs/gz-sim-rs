// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! CPU IMU leases and declarations owned exclusively by the publication IO thread.
use gz_sim_proto::{msgs::Imu, prost::Name};
use std::{
    collections::BTreeMap,
    sync::{
        Arc, Weak,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
};
use zenoh::{
    Wait, liveliness::LivelinessToken, matching::MatchingListener, pubsub::Publisher,
    qos::CongestionControl,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct ImuId(uuid::Uuid);

struct Interest {
    matching: AtomicBool,
    usable: AtomicBool,
}

/// Contains no Session, publisher, listener or liveliness declaration.
pub(super) struct ImuOutput {
    id: ImuId,
    interest: Arc<Interest>,
    retired: mpsc::SyncSender<ImuId>,
    dirty: Arc<AtomicBool>,
}
impl ImuOutput {
    /// The caller also checks its world Session for immediate closed-session errors.
    pub(super) fn matching(&self) -> Result<bool, &'static str> {
        if !self.interest.usable.load(Ordering::Acquire) {
            return Err("IMU declaration is unavailable");
        }
        Ok(self.interest.matching.load(Ordering::Acquire))
    }
}
impl Drop for ImuOutput {
    fn drop(&mut self) {
        // A full wake channel already has work queued. IO also reconciles Weak
        // leases while declarations exist; notification loss cannot retain them.
        self.dirty.store(true, Ordering::Release);
        let _ = self.retired.try_send(self.id);
    }
}

/// Queueing a frame cannot extend the CPU lease's lifetime.
pub(super) struct ImuJob {
    id: ImuId,
    lease: Weak<ImuOutput>,
    bytes: Vec<u8>,
}
impl ImuJob {
    pub(super) fn new(output: &Arc<ImuOutput>, bytes: Vec<u8>) -> Self {
        Self {
            id: output.id,
            lease: Arc::downgrade(output),
            bytes,
        }
    }
}

struct Declaration {
    key: String,
    lease: Weak<ImuOutput>,
    interest: Arc<Interest>,
    _listener: MatchingListener<()>,
    publisher: Publisher<'static>,
    _presence: LivelinessToken,
}
impl Drop for Declaration {
    fn drop(&mut self) {
        self.interest.usable.store(false, Ordering::Release);
    }
}

/// Construct, reconcile, publish and drop only on the world's publication IO owner.
pub(super) struct ImuDeclarations {
    entries: BTreeMap<ImuId, Declaration>,
    dirty: Option<Arc<AtomicBool>>,
}
impl ImuDeclarations {
    pub(super) fn new() -> Self {
        Self {
            entries: BTreeMap::new(),
            dirty: None,
        }
    }

    pub(super) fn get_or_create(
        &mut self,
        session: &zenoh::Session,
        key: String,
        process: &str,
        node: &str,
        retired: mpsc::SyncSender<ImuId>,
    ) -> zenoh::Result<Arc<ImuOutput>> {
        self.reconcile(session.is_closed());
        if session.is_closed() {
            return Err("world session is closed".into());
        }
        // Topic sharing is resolved at setup, not by retaining a second CPU registry.
        if let Some((&id, entry)) = self.entries.iter().find(|(_, entry)| entry.key == key) {
            if let Some(output) = entry.lease.upgrade() {
                return Ok(output);
            }
            // Arc's strong count reaches zero before the CPU destructor sets dirty.
            self.entries.remove(&id);
            if self.entries.is_empty() {
                self.dirty = None;
            }
        }
        let token = crate::world_io::presence(&key, process, node, "MP", &Imu::full_name())?;
        let publisher = session
            .declare_publisher(key.clone())
            .congestion_control(CongestionControl::Drop)
            .wait()?;
        let interest = Arc::new(Interest {
            matching: AtomicBool::new(false),
            usable: AtomicBool::new(false),
        });
        let observed = Arc::clone(&interest);
        // Zenoh installs this listener and reports an existing true match while
        // holding its listener state lock; a separate snapshot could overwrite it.
        let listener = publisher
            .matching_listener()
            .callback(move |status| {
                observed
                    .matching
                    .store(status.matching(), Ordering::Release);
            })
            .wait()?;
        let presence = session.liveliness().declare_token(token).wait()?;
        if session.is_closed() {
            return Err("world session closed during IMU setup".into());
        }
        let dirty = self
            .dirty
            .get_or_insert_with(|| Arc::new(AtomicBool::new(false)));
        let output = Arc::new(ImuOutput {
            id: ImuId(uuid::Uuid::new_v4()),
            interest: Arc::clone(&interest),
            retired,
            dirty: Arc::clone(dirty),
        });
        interest.usable.store(true, Ordering::Release);
        self.entries.insert(
            output.id,
            Declaration {
                key,
                lease: Arc::downgrade(&output),
                interest,
                _listener: listener,
                publisher,
                _presence: presence,
            },
        );
        Ok(output)
    }

    pub(super) fn reconcile(&mut self, session_closed: bool) {
        if session_closed {
            self.entries = BTreeMap::new();
            self.dirty = None;
        } else if self
            .dirty
            .as_ref()
            .is_some_and(|dirty| dirty.swap(false, Ordering::AcqRel))
        {
            self.entries
                .retain(|_, entry| entry.lease.strong_count() != 0);
            if self.entries.is_empty() {
                self.entries = BTreeMap::new();
                self.dirty = None;
            }
        }
    }

    pub(super) fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// False means an unsent retired/stale frame was cancelled. A put admitted
    /// before the final CPU drop may finish; no strong CPU lease is held by IO.
    pub(super) fn publish(&self, job: ImuJob) -> zenoh::Result<bool> {
        let Some(entry) = self.entries.get(&job.id) else {
            return Ok(false);
        };
        if !Weak::ptr_eq(&entry.lease, &job.lease) || job.lease.strong_count() == 0 {
            return Ok(false);
        }
        entry
            .publisher
            .put(job.bytes)
            .attachment(Imu::full_name())
            .wait()?;
        Ok(true)
    }
}

#[cfg(test)]
#[path = "../tests/private/imu_io.rs"]
mod tests;
