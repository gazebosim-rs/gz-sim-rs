// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One canonical Twist subscription on the existing world publication IO owner.
use gz_sim_proto::{
    msgs::Twist,
    prost::{Message, Name},
};
use std::sync::{
    Arc, Weak,
    atomic::{AtomicBool, AtomicU64, Ordering},
    mpsc,
};
use zenoh::{Wait, liveliness::LivelinessToken, pubsub::Subscriber, sample::Sample};

pub(super) type Command = (u64, Option<([f64; 3], [f64; 3])>);
pub(super) type Receipt = mpsc::SyncSender<Result<Arc<Lease>, String>>;
pub(super) struct Setup {
    pub key: String,
    pub input: mpsc::SyncSender<Command>,
    pub max_bytes: usize,
    pub drops: Arc<AtomicU64>,
    pub epoch: Arc<AtomicU64>,
}
pub(super) struct Lease {
    id: uuid::Uuid,
    key: String,
    active: Arc<AtomicBool>,
}
impl Lease {
    pub(super) fn retire(&self) {
        self.active.store(false, Ordering::Release);
    }
    pub(super) fn identity(&self) -> uuid::Uuid {
        self.id
    }
    pub(super) fn matches(&self, id: uuid::Uuid, key: &str) -> bool {
        self.id == id && self.key == key && self.active.load(Ordering::Acquire)
    }
}
impl Drop for Lease {
    fn drop(&mut self) {
        self.retire();
    }
}
fn command(message: Twist) -> Option<([f64; 3], [f64; 3])> {
    let linear = message.linear.unwrap_or_default();
    let angular = message.angular.unwrap_or_default();
    let linear = [linear.x, linear.y, linear.z];
    let angular = [angular.x, angular.y, angular.z];
    linear
        .into_iter()
        .chain(angular)
        .all(f64::is_finite)
        .then_some((linear, angular))
}
struct Callback {
    active: Arc<AtomicBool>,
    input: mpsc::SyncSender<Command>,
    drops: Arc<AtomicU64>,
    epoch: Arc<AtomicU64>,
    max_bytes: usize,
    message_type: String,
}
impl Callback {
    fn receive(&self, sample: Sample) {
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        let epoch = self.epoch.load(Ordering::Acquire);
        let valid = sample.payload().len() <= self.max_bytes
            && sample.attachment().is_some_and(|a| {
                a.len() == self.message_type.len()
                    && a.to_bytes().as_ref() == self.message_type.as_bytes()
            });
        let command = valid
            .then(|| Twist::decode(sample.payload().to_bytes().as_ref()).ok())
            .flatten()
            .and_then(command);
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        if matches!(
            self.input.try_send((epoch, command)),
            Err(mpsc::TrySendError::Full(_))
        ) {
            self.drops.fetch_add(1, Ordering::Relaxed);
        }
    }
}
pub(super) struct Declaration {
    lease: Weak<Lease>,
    state: Arc<Callback>,
    subscriber: Option<Subscriber<()>>,
    presence: Option<LivelinessToken>,
}
impl Declaration {
    pub(super) fn new(
        session: &zenoh::Session,
        setup: Setup,
        process: &str,
        node: &str,
    ) -> zenoh::Result<(Self, Arc<Lease>)> {
        if session.is_closed() || setup.max_bytes == 0 || setup.max_bytes > i64::MAX as usize {
            return Err("invalid velocity-control setup or closed world session".into());
        }
        let key = zenoh::key_expr::OwnedKeyExpr::try_from(setup.key.as_str())?;
        let token =
            crate::world_io::presence(&setup.key, process, node, "MS", &Twist::full_name())?;
        if key.is_wild() || zenoh::key_expr::OwnedKeyExpr::try_from(token.as_str())?.is_wild() {
            return Err("velocity control requires a concrete key".into());
        }
        let lease = Arc::new(Lease {
            id: uuid::Uuid::new_v4(),
            key: setup.key.clone(),
            active: Arc::new(AtomicBool::new(true)),
        });
        let state = Arc::new(Callback {
            active: Arc::clone(&lease.active),
            input: setup.input,
            drops: setup.drops,
            epoch: setup.epoch,
            max_bytes: setup.max_bytes,
            message_type: Twist::full_name(),
        });
        let mut result = Self {
            lease: Arc::downgrade(&lease),
            state,
            subscriber: None,
            presence: None,
        };
        let callback = Arc::clone(&result.state);
        result.subscriber = Some(
            session
                .declare_subscriber(setup.key)
                .callback(move |sample| callback.receive(sample))
                .wait()?,
        );
        result.presence = Some(session.liveliness().declare_token(token).wait()?);
        if session.is_closed() {
            return Err("world session closed during velocity setup".into());
        }
        Ok((result, lease))
    }
    pub(super) fn begin_shutdown(&self) {
        self.state.active.store(false, Ordering::Release);
    }
    pub(super) fn reconcile(&mut self, closed: bool) -> bool {
        if closed || self.lease.strong_count() == 0 {
            self.begin_shutdown();
        }
        if self.state.active.load(Ordering::Acquire) {
            return false;
        }
        self.subscriber = None;
        self.presence = None;
        // Undeclaration alone is not callback quiescence. Retain the capture witness.
        Arc::strong_count(&self.state) == 1
    }
}
impl Drop for Declaration {
    fn drop(&mut self) {
        self.begin_shutdown();
    }
}

#[cfg(test)]
#[path = "../tests/private/velocity_control_io.rs"]
mod tests;
