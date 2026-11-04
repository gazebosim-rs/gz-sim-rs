// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One built-in drive's canonical declarations on the existing world IO owner.
use gz_sim_proto::{
    msgs::{Odometry, Twist},
    prost::{Message, Name},
};
use std::sync::{
    Arc, Weak,
    atomic::{AtomicBool, AtomicU64, Ordering},
    mpsc,
};
use zenoh::{
    Wait,
    liveliness::LivelinessToken,
    matching::MatchingListener,
    pubsub::{Publisher, Subscriber},
    qos::CongestionControl,
    sample::Sample,
};

pub(super) type Command = (u64, Option<[f64; 2]>);
pub(super) type Receipt = mpsc::SyncSender<Result<Arc<Output>, String>>;

pub(super) struct Setup {
    pub(super) command_key: String,
    pub(super) odometry_key: String,
    pub(super) input: mpsc::SyncSender<Command>,
    pub(super) max_bytes: usize,
    pub(super) drops: Arc<AtomicU64>,
    pub(super) epoch: Arc<AtomicU64>,
}

pub(super) struct Output {
    id: uuid::Uuid,
    active: Arc<AtomicBool>,
    matching: Arc<AtomicBool>,
}
impl Output {
    pub(super) fn matching(&self) -> Result<bool, &'static str> {
        if !self.active.load(Ordering::Acquire) {
            return Err("drive declaration is unavailable");
        }
        Ok(self.matching.load(Ordering::Acquire))
    }
}
impl Drop for Output {
    fn drop(&mut self) {
        self.active.store(false, Ordering::Release);
    }
}

pub(super) struct Job {
    id: uuid::Uuid,
    lease: Weak<Output>,
    bytes: Vec<u8>,
}
impl Job {
    pub(super) fn new(output: &Arc<Output>, bytes: Vec<u8>) -> Self {
        Self {
            id: output.id,
            lease: Arc::downgrade(output),
            bytes,
        }
    }
}

fn command(message: Twist) -> Option<[f64; 2]> {
    let linear = message.linear.unwrap_or_default();
    let angular = message.angular.unwrap_or_default();
    ([
        linear.x, linear.y, linear.z, angular.x, angular.y, angular.z,
    ]
    .into_iter()
    .all(f64::is_finite)
        && linear.y == 0.0
        && linear.z == 0.0
        && angular.x == 0.0
        && angular.y == 0.0)
        .then_some([linear.x, angular.z])
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
        let admitted = sample.payload().len() <= self.max_bytes
            && sample.attachment().is_some_and(|a| {
                a.len() == self.message_type.len()
                    && a.to_bytes().as_ref() == self.message_type.as_bytes()
            });
        let command = admitted
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

pub(super) struct Declarations {
    lease: Weak<Output>,
    state: Arc<Callback>,
    publisher: Option<Publisher<'static>>,
    subscriber: Option<Subscriber<()>>,
    listener: Option<MatchingListener<()>>,
    presence: Vec<LivelinessToken>,
}
impl Declarations {
    pub(super) fn new(
        session: &zenoh::Session,
        setup: Setup,
        process: &str,
        node: &str,
    ) -> zenoh::Result<(Self, Arc<Output>)> {
        if session.is_closed()
            || setup.max_bytes == 0
            || setup.max_bytes > i64::MAX as usize
            || setup.command_key == setup.odometry_key
        {
            return Err("invalid drive setup or closed world session".into());
        }
        let output = Arc::new(Output {
            id: uuid::Uuid::new_v4(),
            active: Arc::new(AtomicBool::new(true)),
            matching: Arc::new(AtomicBool::new(false)),
        });
        let state = Arc::new(Callback {
            active: Arc::clone(&output.active),
            input: setup.input,
            drops: setup.drops,
            epoch: setup.epoch,
            max_bytes: setup.max_bytes,
            message_type: Twist::full_name(),
        });
        let mut result = Self {
            lease: Arc::downgrade(&output),
            state,
            publisher: None,
            subscriber: None,
            listener: None,
            presence: Vec::new(),
        };
        let input_token = crate::world_io::presence(
            &setup.command_key,
            process,
            node,
            "MS",
            &Twist::full_name(),
        )?;
        let output_token = crate::world_io::presence(
            &setup.odometry_key,
            process,
            node,
            "MP",
            &Odometry::full_name(),
        )?;
        let callback = Arc::clone(&result.state);
        result.subscriber = Some(
            session
                .declare_subscriber(setup.command_key)
                .callback(move |sample| callback.receive(sample))
                .wait()?,
        );
        let publisher = session
            .declare_publisher(setup.odometry_key)
            .congestion_control(CongestionControl::Drop)
            .wait()?;
        let matching = Arc::clone(&output.matching);
        result.listener = Some(
            publisher
                .matching_listener()
                .callback(move |status| {
                    matching.store(status.matching(), Ordering::Release);
                })
                .wait()?,
        );
        result.publisher = Some(publisher);
        result
            .presence
            .push(session.liveliness().declare_token(input_token).wait()?);
        result
            .presence
            .push(session.liveliness().declare_token(output_token).wait()?);
        if session.is_closed() {
            return Err("world session closed during drive setup".into());
        }
        Ok((result, output))
    }
    pub(super) fn begin_shutdown(&self) {
        self.state.active.store(false, Ordering::Release);
    }
    /// Keep the upstream callback capture witness until no retired callback owns it.
    pub(super) fn reconcile(&mut self, closed: bool) -> bool {
        if closed || self.lease.strong_count() == 0 {
            self.begin_shutdown();
        }
        if self.state.active.load(Ordering::Acquire) {
            return false;
        }
        self.subscriber = None;
        self.listener = None;
        self.publisher = None;
        self.presence = Vec::new();
        Arc::strong_count(&self.state) == 1
    }
    pub(super) fn publish(&self, job: Job) -> zenoh::Result<bool> {
        if !self.state.active.load(Ordering::Acquire)
            || !Weak::ptr_eq(&self.lease, &job.lease)
            || job.lease.strong_count() == 0
        {
            return Ok(false);
        }
        let Some(output) = self.lease.upgrade() else {
            return Ok(false);
        };
        if output.id != job.id {
            return Ok(false);
        }
        // The temporary identity check must not retain the CPU lease during put.
        drop(output);
        let Some(publisher) = &self.publisher else {
            return Ok(false);
        };
        publisher
            .put(job.bytes)
            .attachment(Odometry::full_name())
            .wait()?;
        Ok(true)
    }
}
impl Drop for Declarations {
    fn drop(&mut self) {
        self.begin_shutdown();
    }
}

#[cfg(test)]
#[path = "../tests/private/diff_drive_io.rs"]
mod tests;
