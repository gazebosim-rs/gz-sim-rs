// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Three typed ground-truth channels; only the world IO owner holds declarations.
use gz_sim_proto::{
    msgs::{Odometry, OdometryWithCovariance, PoseV},
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
use zenoh::{Wait, liveliness::LivelinessToken, pubsub::Publisher, qos::CongestionControl};
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) enum GroundTruthOdometryChannel {
    Odometry,
    Covariance,
    Transform,
}
impl GroundTruthOdometryChannel {
    fn schema(self) -> String {
        match self {
            Self::Odometry => Odometry::full_name(),
            Self::Covariance => OdometryWithCovariance::full_name(),
            Self::Transform => PoseV::full_name(),
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct GroundTruthOdometryId(uuid::Uuid);
pub(super) struct GroundTruthOdometryOutput {
    id: GroundTruthOdometryId,
    pub(super) channel: GroundTruthOdometryChannel,
    retired: mpsc::SyncSender<GroundTruthOdometryId>,
    dirty: Arc<AtomicBool>,
}
impl Drop for GroundTruthOdometryOutput {
    fn drop(&mut self) {
        self.dirty.store(true, Ordering::Release);
        let _ = self.retired.try_send(self.id);
    }
}
pub(super) struct GroundTruthOdometryJob {
    id: GroundTruthOdometryId,
    pub(super) channel: GroundTruthOdometryChannel,
    lease: Weak<GroundTruthOdometryOutput>,
    bytes: Vec<u8>,
}
impl GroundTruthOdometryJob {
    pub(super) fn new(output: &Arc<GroundTruthOdometryOutput>, bytes: Vec<u8>) -> Self {
        Self {
            id: output.id,
            channel: output.channel,
            lease: Arc::downgrade(output),
            bytes,
        }
    }
}
struct Declaration {
    key: String,
    channel: GroundTruthOdometryChannel,
    lease: Weak<GroundTruthOdometryOutput>,
    publisher: Publisher<'static>,
    _presence: LivelinessToken,
}
pub(super) struct GroundTruthOdometryDeclarations {
    entries: BTreeMap<GroundTruthOdometryId, Declaration>,
    dirty: Option<Arc<AtomicBool>>,
}
impl GroundTruthOdometryDeclarations {
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
        channel: GroundTruthOdometryChannel,
        process: &str,
        node: &str,
        retired: mpsc::SyncSender<GroundTruthOdometryId>,
    ) -> zenoh::Result<Arc<GroundTruthOdometryOutput>> {
        self.reconcile(session.is_closed());
        if session.is_closed() {
            return Err("world session is closed".into());
        }
        if let Some((&id, entry)) = self.entries.iter().find(|(_, e)| e.key == key) {
            if entry.channel != channel {
                return Err("ground-truth odometry channel collision".into());
            }
            if let Some(output) = entry.lease.upgrade() {
                return Ok(output);
            }
            self.entries.remove(&id);
            if self.entries.is_empty() {
                self.dirty = None;
            }
        }
        let token = crate::world_io::presence(&key, process, node, "MP", &channel.schema())?;
        let publisher = session
            .declare_publisher(key.clone())
            .congestion_control(CongestionControl::Drop)
            .wait()?;
        let presence = session.liveliness().declare_token(token).wait()?;
        if session.is_closed() {
            return Err("world session closed during ground-truth odometry setup".into());
        }
        let dirty = self
            .dirty
            .get_or_insert_with(|| Arc::new(AtomicBool::new(false)));
        let output = Arc::new(GroundTruthOdometryOutput {
            id: GroundTruthOdometryId(uuid::Uuid::new_v4()),
            channel,
            retired,
            dirty: Arc::clone(dirty),
        });
        self.entries.insert(
            output.id,
            Declaration {
                key,
                channel,
                lease: Arc::downgrade(&output),
                publisher,
                _presence: presence,
            },
        );
        Ok(output)
    }
    pub(super) fn reconcile(&mut self, closed: bool) {
        if closed {
            self.entries = BTreeMap::new();
            self.dirty = None;
        } else if self
            .dirty
            .as_ref()
            .is_some_and(|d| d.swap(false, Ordering::AcqRel))
        {
            self.entries.retain(|_, e| e.lease.strong_count() != 0);
            if self.entries.is_empty() {
                self.entries = BTreeMap::new();
                self.dirty = None;
            }
        }
    }
    pub(super) fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
    pub(super) fn publish(&self, job: GroundTruthOdometryJob) -> zenoh::Result<bool> {
        let Some(entry) = self.entries.get(&job.id) else {
            return Ok(false);
        };
        if entry.channel != job.channel
            || !Weak::ptr_eq(&entry.lease, &job.lease)
            || job.lease.strong_count() == 0
        {
            return Ok(false);
        }
        entry
            .publisher
            .put(job.bytes)
            .attachment(entry.channel.schema())
            .wait()?;
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn session() -> zenoh::Session {
        let config=zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap();
        zenoh::open(config).wait().unwrap()
    }
    #[test]
    fn typed_leases_reconcile_and_queued_jobs_never_keep_an_output_alive() {
        let session = session();
        let mut d = GroundTruthOdometryDeclarations::new();
        let (tx, _rx) = mpsc::sync_channel(1);
        assert!(d.is_empty());
        for (i, channel) in [
            GroundTruthOdometryChannel::Odometry,
            GroundTruthOdometryChannel::Covariance,
            GroundTruthOdometryChannel::Transform,
        ]
        .into_iter()
        .enumerate()
        {
            let key = format!("@/m34@/odometry/{i}");
            let out = d
                .get_or_create(
                    &session,
                    key.clone(),
                    channel,
                    "process",
                    "node",
                    tx.clone(),
                )
                .unwrap();
            assert!(Arc::ptr_eq(
                &out,
                &d.get_or_create(
                    &session,
                    key.clone(),
                    channel,
                    "process",
                    "node",
                    tx.clone()
                )
                .unwrap()
            ));
            let other = if channel == GroundTruthOdometryChannel::Odometry {
                GroundTruthOdometryChannel::Transform
            } else {
                GroundTruthOdometryChannel::Odometry
            };
            assert!(
                d.get_or_create(&session, key, other, "process", "node", tx.clone())
                    .is_err()
            );
            assert!(
                d.publish(GroundTruthOdometryJob::new(&out, Vec::new()))
                    .unwrap()
            );
            let stale = GroundTruthOdometryJob::new(&out, Vec::new());
            let missing = GroundTruthOdometryJob::new(&out, Vec::new());
            drop(out);
            assert!(!d.publish(stale).unwrap());
            d.reconcile(false);
            assert!(d.is_empty());
            assert!(!d.publish(missing).unwrap());
        }
        session.close().wait().unwrap();
        d.reconcile(true);
        assert!(d.is_empty());
        assert!(
            d.get_or_create(
                &session,
                "@/m34@/odometry/closed".into(),
                GroundTruthOdometryChannel::Odometry,
                "process",
                "node",
                tx
            )
            .is_err()
        );
    }
}
