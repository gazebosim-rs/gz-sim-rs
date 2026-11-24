// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! CPU lidar leases and declarations owned exclusively by the publication IO thread.
use gz_sim_proto::{
    msgs::{LaserScan, PointCloudPacked},
    prost::Name,
};
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
pub(super) struct LidarId(uuid::Uuid);

pub(super) const POINTS_MAX_MESSAGE_BYTES: usize = 4096;
pub(super) const POINTS_MAX_NAME_BYTES: usize = 1024;
const MAX_DECLARATIONS: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Kind {
    Scan,
    Points,
}

impl Kind {
    fn message_name(self) -> String {
        match self {
            Self::Scan => LaserScan::full_name(),
            Self::Points => PointCloudPacked::full_name(),
        }
    }
}

struct Interest {
    matching: AtomicBool,
    usable: AtomicBool,
}

/// Contains no Session, publisher, listener or liveliness declaration.
pub(super) struct LidarOutput {
    id: LidarId,
    key: String,
    kind: Kind,
    interest: Arc<Interest>,
    retired: mpsc::SyncSender<LidarId>,
    dirty: Arc<AtomicBool>,
}
impl LidarOutput {
    pub(super) fn matches(&self, key: &str, kind: Kind) -> bool {
        self.key == key && self.kind == kind
    }

    pub(super) fn is_points(&self) -> bool {
        self.kind == Kind::Points
    }
    /// The caller also checks its world Session for immediate closed-session errors.
    pub(super) fn matching(&self) -> Result<bool, &'static str> {
        if !self.interest.usable.load(Ordering::Acquire) {
            return Err("lidar declaration is unavailable");
        }
        Ok(self.interest.matching.load(Ordering::Acquire))
    }
}
impl Drop for LidarOutput {
    fn drop(&mut self) {
        // A full wake channel already has work queued. IO also reconciles Weak
        // leases while declarations exist; notification loss cannot retain them.
        self.dirty.store(true, Ordering::Release);
        let _ = self.retired.try_send(self.id);
    }
}

/// Queueing a frame cannot extend the CPU lease's lifetime.
pub(super) struct LidarJob {
    id: LidarId,
    lease: Weak<LidarOutput>,
    bytes: Vec<u8>,
}
impl LidarJob {
    pub(super) fn new(output: &Arc<LidarOutput>, bytes: Vec<u8>) -> Self {
        Self {
            id: output.id,
            lease: Arc::downgrade(output),
            bytes,
        }
    }
}

struct Declaration {
    key: String,
    kind: Kind,
    lease: Weak<LidarOutput>,
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
pub(super) struct LidarDeclarations {
    entries: BTreeMap<LidarId, Declaration>,
    dirty: Option<Arc<AtomicBool>>,
}
impl LidarDeclarations {
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
        retired: mpsc::SyncSender<LidarId>,
    ) -> zenoh::Result<Arc<LidarOutput>> {
        self.get_or_create_kind(session, key, process, node, retired, Kind::Scan)
    }

    pub(super) fn get_or_create_kind(
        &mut self,
        session: &zenoh::Session,
        key: String,
        process: &str,
        node: &str,
        retired: mpsc::SyncSender<LidarId>,
        kind: Kind,
    ) -> zenoh::Result<Arc<LidarOutput>> {
        self.reconcile(session.is_closed());
        if session.is_closed() {
            return Err("world session is closed".into());
        }
        // Topic sharing is resolved at setup, not by retaining a second CPU registry.
        if let Some((&id, entry)) = self.entries.iter().find(|(_, entry)| entry.key == key) {
            if let Some(output) = entry.lease.upgrade() {
                if entry.kind != kind {
                    return Err("lidar key is reserved for another message type".into());
                }
                return Ok(output);
            }
            // Arc's strong count reaches zero before the CPU destructor sets dirty.
            self.entries.remove(&id);
            if self.entries.is_empty() {
                self.dirty = None;
            }
        }
        if self.entries.len() >= MAX_DECLARATIONS
            || self
                .entries
                .values()
                .filter(|entry| entry.kind == kind)
                .count()
                >= 8
            || kind == Kind::Points && key.len() > POINTS_MAX_NAME_BYTES
        {
            return Err("lidar declaration bound exceeded".into());
        }
        let token = crate::world_io::presence(&key, process, node, "MP", &kind.message_name())?;
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
            return Err("world session closed during lidar setup".into());
        }
        let dirty = self
            .dirty
            .get_or_insert_with(|| Arc::new(AtomicBool::new(false)));
        let output = Arc::new(LidarOutput {
            id: LidarId(uuid::Uuid::new_v4()),
            key: key.clone(),
            kind,
            interest: Arc::clone(&interest),
            retired,
            dirty: Arc::clone(dirty),
        });
        interest.usable.store(true, Ordering::Release);
        self.entries.insert(
            output.id,
            Declaration {
                key,
                kind,
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
    pub(super) fn publish(&self, job: LidarJob) -> zenoh::Result<bool> {
        let Some(entry) = self.entries.get(&job.id) else {
            return Ok(false);
        };
        if !Weak::ptr_eq(&entry.lease, &job.lease) || job.lease.strong_count() == 0 {
            return Ok(false);
        }
        if entry.kind == Kind::Points && job.bytes.len() > POINTS_MAX_MESSAGE_BYTES {
            return Err("lidar point cloud exceeds its message bound".into());
        }
        entry
            .publisher
            .put(job.bytes)
            .attachment(entry.kind.message_name())
            .wait()?;
        Ok(true)
    }

    /// Queue admission is paired; transport puts keep their existing independent
    /// completion semantics if a subscriber or CPU owner retires meanwhile.
    pub(super) fn publish_pair(&self, scan: LidarJob, points: LidarJob) -> zenoh::Result<bool> {
        let valid = |job: &LidarJob, kind| {
            self.entries.get(&job.id).is_some_and(|entry| {
                entry.kind == kind
                    && Weak::ptr_eq(&entry.lease, &job.lease)
                    && job.lease.strong_count() != 0
            })
        };
        if scan.id == points.id || !valid(&scan, Kind::Scan) || !valid(&points, Kind::Points) {
            return Ok(false);
        }
        if points.bytes.len() > POINTS_MAX_MESSAGE_BYTES {
            return Err("lidar point cloud exceeds its message bound".into());
        }
        if !self.publish(scan)? {
            return Ok(false);
        }
        self.publish(points)
    }
}

#[cfg(test)]
#[path = "../tests/private/lidar_io.rs"]
mod tests;
