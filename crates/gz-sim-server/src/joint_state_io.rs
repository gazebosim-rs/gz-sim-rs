// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! CPU joint-state leases and declarations owned exclusively by the publication IO thread.
use gz_sim_proto::{msgs::Model, prost::Name};
use std::{
    collections::BTreeMap,
    sync::{
        Arc, Weak,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
};
use zenoh::{Wait, liveliness::LivelinessToken, pubsub::Publisher, qos::CongestionControl};

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct JointStateId(uuid::Uuid);

/// Contains no Session, publisher, listener or liveliness declaration.
pub(super) struct JointStateOutput {
    id: JointStateId,
    retired: mpsc::SyncSender<JointStateId>,
    dirty: Arc<AtomicBool>,
}
impl Drop for JointStateOutput {
    fn drop(&mut self) {
        // A full wake channel already has work queued. IO also reconciles Weak
        // leases while declarations exist; notification loss cannot retain them.
        self.dirty.store(true, Ordering::Release);
        let _ = self.retired.try_send(self.id);
    }
}

/// Queueing a frame cannot extend the CPU lease's lifetime.
pub(super) struct JointStateJob {
    id: JointStateId,
    lease: Weak<JointStateOutput>,
    bytes: Vec<u8>,
}
impl JointStateJob {
    pub(super) fn new(output: &Arc<JointStateOutput>, bytes: Vec<u8>) -> Self {
        Self {
            id: output.id,
            lease: Arc::downgrade(output),
            bytes,
        }
    }
}

struct Declaration {
    key: String,
    lease: Weak<JointStateOutput>,
    publisher: Publisher<'static>,
    _presence: LivelinessToken,
}
/// Construct, reconcile, publish and drop only on the world's publication IO owner.
pub(super) struct JointStateDeclarations {
    entries: BTreeMap<JointStateId, Declaration>,
    dirty: Option<Arc<AtomicBool>>,
}
impl JointStateDeclarations {
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
        retired: mpsc::SyncSender<JointStateId>,
    ) -> zenoh::Result<Arc<JointStateOutput>> {
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
        let token = crate::world_io::presence(&key, process, node, "MP", &Model::full_name())?;
        let publisher = session
            .declare_publisher(key.clone())
            .congestion_control(CongestionControl::Drop)
            .wait()?;
        let presence = session.liveliness().declare_token(token).wait()?;
        if session.is_closed() {
            return Err("world session closed during joint-state setup".into());
        }
        let dirty = self
            .dirty
            .get_or_insert_with(|| Arc::new(AtomicBool::new(false)));
        let output = Arc::new(JointStateOutput {
            id: JointStateId(uuid::Uuid::new_v4()),
            retired,
            dirty: Arc::clone(dirty),
        });
        self.entries.insert(
            output.id,
            Declaration {
                key,
                lease: Arc::downgrade(&output),
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
    pub(super) fn publish(&self, job: JointStateJob) -> zenoh::Result<bool> {
        let Some(entry) = self.entries.get(&job.id) else {
            return Ok(false);
        };
        if !Weak::ptr_eq(&entry.lease, &job.lease) || job.lease.strong_count() == 0 {
            return Ok(false);
        }
        entry
            .publisher
            .put(job.bytes)
            .attachment(Model::full_name())
            .wait()?;
        Ok(true)
    }
}

#[cfg(test)]
#[path = "../tests/private/joint_state_io.rs"]
mod tests;
