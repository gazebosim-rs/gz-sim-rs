// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical Double ingress on the existing world IO owner; no actuation here.
use gz_sim_proto::{
    msgs::Double,
    prost::{Message, Name},
};
use std::{
    collections::BTreeSet,
    sync::{
        Arc, Weak,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
};
use zenoh::{Wait, liveliness::LivelinessToken, pubsub::Subscriber, sample::Sample};

pub(super) type Command = (u64, Option<f64>);
pub(super) type Receipt = mpsc::SyncSender<Result<Vec<Arc<Lease>>, String>>;
pub(super) struct Endpoint {
    pub(super) key: String,
    pub(super) input: mpsc::SyncSender<Command>,
}
pub(super) struct Setup {
    pub(super) endpoints: Vec<Endpoint>,
    pub(super) max_bytes: usize,
    pub(super) drops: Arc<AtomicU64>,
    pub(super) epoch: Arc<AtomicU64>,
}
pub(super) struct Lease {
    active: Arc<AtomicBool>,
}
impl Drop for Lease {
    fn drop(&mut self) {
        self.active.store(false, Ordering::Release);
    }
}
struct Callback {
    input: mpsc::SyncSender<Command>,
    active: Arc<AtomicBool>,
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
            && sample.attachment().is_some_and(|attachment| {
                attachment.len() == self.message_type.len()
                    && attachment.to_bytes().as_ref() == self.message_type.as_bytes()
            });
        let value = admitted
            .then(|| Double::decode(sample.payload().to_bytes().as_ref()).ok())
            .flatten()
            .map(|message| message.data)
            .filter(|value| value.is_finite());
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        // Invalid packets consume one bounded command slot, never a transport buffer.
        if matches!(
            self.input.try_send((epoch, value)),
            Err(mpsc::TrySendError::Full(_))
        ) {
            self.drops.fetch_add(1, Ordering::Relaxed);
        }
    }
}
struct Declaration {
    lease: Weak<Lease>,
    state: Arc<Callback>,
    subscriber: Option<Subscriber<()>>,
    presence: Option<LivelinessToken>,
}
impl Drop for Declaration {
    fn drop(&mut self) {
        self.state.active.store(false, Ordering::Release);
    }
}
pub(super) struct Declarations {
    endpoints: Vec<Declaration>,
}
impl Declarations {
    pub(super) fn new(
        session: &zenoh::Session,
        setup: Setup,
        process: &str,
        node: &str,
    ) -> zenoh::Result<(Self, Vec<Arc<Lease>>)> {
        if session.is_closed()
            || setup.max_bytes == 0
            || setup.max_bytes > i64::MAX as usize
            || setup.endpoints.is_empty()
            || setup.endpoints.len() > 2
        {
            return Err("invalid joint controller setup or closed world session".into());
        }
        let mut keys = BTreeSet::new();
        if setup
            .endpoints
            .iter()
            .any(|endpoint| !keys.insert(endpoint.key.as_str()))
        {
            return Err("duplicate joint controller endpoint".into());
        }
        let mut result = Self {
            endpoints: Vec::new(),
        };
        let mut leases = Vec::new();
        for endpoint in setup.endpoints {
            let lease = Arc::new(Lease {
                active: Arc::new(AtomicBool::new(true)),
            });
            let state = Arc::new(Callback {
                input: endpoint.input,
                active: Arc::clone(&lease.active),
                drops: Arc::clone(&setup.drops),
                epoch: Arc::clone(&setup.epoch),
                max_bytes: setup.max_bytes,
                message_type: Double::full_name(),
            });
            let mut declaration = Declaration {
                lease: Arc::downgrade(&lease),
                state,
                subscriber: None,
                presence: None,
            };
            let token = crate::world_io::presence(
                &endpoint.key,
                process,
                node,
                "MS",
                &Double::full_name(),
            )?;
            let callback = Arc::clone(&declaration.state);
            declaration.subscriber = Some(
                session
                    .declare_subscriber(endpoint.key)
                    .callback(move |sample| callback.receive(sample))
                    .wait()?,
            );
            declaration.presence = Some(session.liveliness().declare_token(token).wait()?);
            result.endpoints.push(declaration);
            leases.push(lease);
        }
        if session.is_closed() {
            return Err("world session closed during joint controller setup".into());
        }
        Ok((result, leases))
    }
    pub(super) fn begin_shutdown(&self) {
        for endpoint in &self.endpoints {
            endpoint.state.active.store(false, Ordering::Release);
        }
    }
    /// A replacement generation waits until every retired upstream capture is gone.
    pub(super) fn reconcile(&mut self, closed: bool) -> bool {
        for endpoint in &mut self.endpoints {
            if closed || endpoint.lease.strong_count() == 0 {
                endpoint.state.active.store(false, Ordering::Release);
            }
            if !endpoint.state.active.load(Ordering::Acquire) {
                endpoint.subscriber = None;
                endpoint.presence = None;
            }
        }
        self.endpoints.retain(|endpoint| {
            endpoint.state.active.load(Ordering::Acquire) || Arc::strong_count(&endpoint.state) != 1
        });
        if self.endpoints.is_empty() {
            self.endpoints = Vec::new();
        }
        self.endpoints.is_empty()
    }
}
impl Drop for Declarations {
    fn drop(&mut self) {
        self.begin_shutdown();
    }
}

#[cfg(test)]
#[path = "../tests/private/joint_controller_io.rs"]
mod tests;
