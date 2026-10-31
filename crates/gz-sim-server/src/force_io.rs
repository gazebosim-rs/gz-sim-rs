// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Force endpoint declarations and scalar decoding on the world's IO owner.
use gz_sim_proto::{
    msgs::Double,
    prost::{Message, Name},
};
use std::{
    collections::BTreeSet,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
};
use zenoh::{Wait, liveliness::LivelinessToken, pubsub::Subscriber, sample::Sample};

/// Plain setup only; CPU receivers and entity/fanout identities stay with Forces.
pub(super) struct ForceEndpoint {
    pub(super) key: String,
    pub(super) input: mpsc::SyncSender<Option<f64>>,
    pub(super) active: Arc<AtomicBool>,
}
pub(super) struct ForceSetup {
    pub(super) endpoints: Vec<ForceEndpoint>,
    pub(super) max_bytes: usize,
    pub(super) ingress_drops: Arc<AtomicU64>,
}
impl ForceSetup {
    pub(super) fn is_empty(&self) -> bool {
        self.endpoints.is_empty()
    }
}

struct CallbackState {
    input: mpsc::SyncSender<Option<f64>>,
    active: Arc<AtomicBool>,
    ingress_drops: Arc<AtomicU64>,
    max_bytes: usize,
    message_type: String,
}
impl CallbackState {
    fn receive(&self, sample: Sample) {
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        // Flatten only admitted bounded buffers. Even invalid packets consume one
        // scalar queue/budget slot; no Sample or unbounded payload reaches CPU state.
        let admitted = sample.payload().len() <= self.max_bytes
            && sample.attachment().is_some_and(|attachment| {
                attachment.len() == self.message_type.len()
                    && attachment.to_bytes().as_ref() == self.message_type.as_bytes()
            });
        let value = admitted
            .then(|| Double::decode(sample.payload().to_bytes().as_ref()).ok())
            .flatten()
            .map(|message| message.data);
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        if matches!(self.input.try_send(value), Err(mpsc::TrySendError::Full(_))) {
            // This is the caller's world-lifetime metric, never a retired snapshot.
            self.ingress_drops.fetch_add(1, Ordering::Relaxed);
        }
    }
}

struct Declaration {
    subscriber: Option<Subscriber<()>>,
    presence: Option<LivelinessToken>,
    state: Arc<CallbackState>,
}
impl Drop for Declaration {
    fn drop(&mut self) {
        self.state.active.store(false, Ordering::Release);
    }
}

/// Native declarations and capture-release witnesses belong only on the IO owner.
/// The vector stays bounded by the installed endpoint list until captures drain.
pub(super) struct ForceDeclarations {
    endpoints: Vec<Declaration>,
}
impl ForceDeclarations {
    pub(super) fn new(session: &zenoh::Session, setup: ForceSetup) -> zenoh::Result<Self> {
        if session.is_closed() {
            return Err("world session is closed".into());
        }
        if setup.max_bytes == 0 || setup.max_bytes > i64::MAX as usize {
            return Err("invalid force message bound".into());
        }
        if setup.is_empty() {
            return Ok(Self {
                endpoints: Vec::new(),
            });
        }
        let mut keys = BTreeSet::new();
        for endpoint in &setup.endpoints {
            if !keys.insert(endpoint.key.as_str()) {
                return Err("duplicate force endpoint".into());
            }
        }
        let process = session.zid().to_string();
        let mut result = Self {
            endpoints: Vec::new(),
        };
        for endpoint in setup.endpoints {
            if !endpoint.active.load(Ordering::Acquire) {
                continue;
            }
            let state = Arc::new(CallbackState {
                input: endpoint.input,
                active: endpoint.active,
                ingress_drops: Arc::clone(&setup.ingress_drops),
                max_bytes: setup.max_bytes,
                message_type: Double::full_name(),
            });
            let mut declaration = Declaration {
                subscriber: None,
                presence: None,
                state,
            };
            let node = uuid::Uuid::new_v4().to_string();
            let token = crate::world_io::presence(
                &endpoint.key,
                &process,
                &node,
                "MS",
                &Double::full_name(),
            )?;
            let callback = Arc::clone(&declaration.state);
            let subscriber = session
                .declare_subscriber(endpoint.key)
                .callback(move |sample| callback.receive(sample))
                .wait()?;
            declaration.subscriber = Some(subscriber);
            declaration.presence = Some(session.liveliness().declare_token(token).wait()?);
            result.endpoints.push(declaration);
        }
        Ok(result)
    }

    /// True means no retired endpoint has an outstanding upstream callback capture.
    /// The caller stages any next generation until this witness is complete.
    pub(super) fn reconcile(&mut self) -> bool {
        for endpoint in &mut self.endpoints {
            if !endpoint.state.active.load(Ordering::Acquire) {
                drop(endpoint.subscriber.take());
                drop(endpoint.presence.take());
            }
        }
        self.endpoints.retain(|endpoint| {
            endpoint.state.active.load(Ordering::Acquire) || Arc::strong_count(&endpoint.state) != 1
        });
        if self.endpoints.is_empty() {
            self.endpoints = Vec::new();
        }
        self.endpoints
            .iter()
            .all(|endpoint| endpoint.state.active.load(Ordering::Acquire))
    }
    pub(super) fn is_empty(&self) -> bool {
        self.endpoints.is_empty()
    }
    pub(super) fn begin_shutdown(&self) {
        for endpoint in &self.endpoints {
            endpoint.state.active.store(false, Ordering::Release);
        }
    }
}
impl Drop for ForceDeclarations {
    fn drop(&mut self) {
        self.begin_shutdown();
    }
}

#[cfg(test)]
#[path = "../tests/private/force_io.rs"]
mod tests;
