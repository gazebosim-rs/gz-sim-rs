// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded canonical input declarations, owned only by the existing WorldIO worker.
use gz_sim_proto::{
    msgs::{Entity, EntityWrench},
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Channel {
    Once,
    Persistent,
    Clear,
}
impl Channel {
    pub(super) fn index(self) -> usize {
        match self {
            Self::Once => 0,
            Self::Persistent => 1,
            Self::Clear => 2,
        }
    }
    fn message_type(self) -> String {
        match self {
            Self::Once | Self::Persistent => EntityWrench::full_name(),
            Self::Clear => Entity::full_name(),
        }
    }
}
pub(super) enum Command {
    Wrench(Box<EntityWrench>),
    Clear(Entity),
}
pub(super) struct Envelope {
    pub(super) epoch: u64,
    pub(super) command: Option<Command>,
}
pub(super) struct WrenchEndpoint {
    pub(super) key: String,
    pub(super) channel: Channel,
    pub(super) input: mpsc::SyncSender<Envelope>,
    pub(super) active: Arc<AtomicBool>,
    pub(super) epoch: u64,
}
pub(super) struct WrenchSetup {
    pub(super) endpoints: Vec<WrenchEndpoint>,
    pub(super) max_bytes: usize,
    pub(super) ingress_drops: Arc<AtomicU64>,
}
impl WrenchSetup {
    pub(super) fn is_empty(&self) -> bool {
        self.endpoints.is_empty()
    }
}
struct CallbackState {
    channel: Channel,
    input: mpsc::SyncSender<Envelope>,
    active: Arc<AtomicBool>,
    epoch: u64,
    max_bytes: usize,
    ingress_drops: Arc<AtomicU64>,
}
impl CallbackState {
    fn receive(&self, sample: Sample) {
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        let expected = self.channel.message_type();
        let admitted = sample.payload().len() <= self.max_bytes
            && sample.attachment().is_some_and(|a| {
                a.len() == expected.len() && a.to_bytes().as_ref() == expected.as_bytes()
            });
        let command = if admitted {
            let bytes = sample.payload().to_bytes();
            match self.channel {
                Channel::Once | Channel::Persistent => EntityWrench::decode(bytes.as_ref())
                    .ok()
                    .map(Box::new)
                    .map(Command::Wrench),
                Channel::Clear => Entity::decode(bytes.as_ref()).ok().map(Command::Clear),
            }
        } else {
            None
        };
        if !self.active.load(Ordering::Acquire) {
            return;
        }
        if matches!(
            self.input.try_send(Envelope {
                epoch: self.epoch,
                command
            }),
            Err(mpsc::TrySendError::Full(_))
        ) {
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
pub(super) struct WrenchDeclarations {
    entries: Vec<Declaration>,
}
impl WrenchDeclarations {
    pub(super) fn new(session: &zenoh::Session, setup: WrenchSetup) -> zenoh::Result<Self> {
        if session.is_closed() || setup.max_bytes == 0 || setup.max_bytes > i64::MAX as usize {
            return Err("invalid wrench input setup".into());
        }
        let mut keys = BTreeSet::new();
        let mut channels = BTreeSet::new();
        if !setup.is_empty()
            && (setup.endpoints.len() != 3
                || setup
                    .endpoints
                    .iter()
                    .any(|e| !keys.insert(e.key.clone()) || !channels.insert(e.channel.index())))
        {
            return Err("wrench inputs require three distinct channels and keys".into());
        }
        let process = session.zid().to_string();
        let mut result = Self {
            entries: Vec::new(),
        };
        for endpoint in setup.endpoints {
            if !endpoint.active.load(Ordering::Acquire) {
                continue;
            }
            let state = Arc::new(CallbackState {
                channel: endpoint.channel,
                input: endpoint.input,
                active: endpoint.active,
                epoch: endpoint.epoch,
                max_bytes: setup.max_bytes,
                ingress_drops: Arc::clone(&setup.ingress_drops),
            });
            let node = uuid::Uuid::new_v4().to_string();
            let token = crate::world_io::presence(
                &endpoint.key,
                &process,
                &node,
                "MS",
                &state.channel.message_type(),
            )?;
            let mut declaration = Declaration {
                subscriber: None,
                presence: None,
                state: Arc::clone(&state),
            };
            declaration.subscriber = Some(
                session
                    .declare_subscriber(endpoint.key)
                    .callback(move |sample| state.receive(sample))
                    .wait()?,
            );
            declaration.presence = Some(session.liveliness().declare_token(token).wait()?);
            result.entries.push(declaration);
        }
        Ok(result)
    }
    /// Retain the callback Arc witness until every retired upstream capture ends.
    pub(super) fn reconcile(&mut self) -> bool {
        for entry in &mut self.entries {
            if !entry.state.active.load(Ordering::Acquire) {
                drop(entry.subscriber.take());
                drop(entry.presence.take());
            }
        }
        self.entries.retain(|entry| {
            entry.state.active.load(Ordering::Acquire) || Arc::strong_count(&entry.state) != 1
        });
        if self.entries.is_empty() {
            self.entries = Vec::new();
        }
        self.entries
            .iter()
            .all(|entry| entry.state.active.load(Ordering::Acquire))
    }
    pub(super) fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
    pub(super) fn begin_shutdown(&self) {
        for entry in &self.entries {
            entry.state.active.store(false, Ordering::Release);
        }
    }
}
impl Drop for WrenchDeclarations {
    fn drop(&mut self) {
        self.begin_shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};
    fn session() -> zenoh::Session {
        let mut config = zenoh::Config::default();
        for (key, value) in [
            ("mode", "\"peer\""),
            ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
            ("connect/endpoints", "[]"),
            ("scouting/multicast/enabled", "false"),
            ("scouting/gossip/enabled", "false"),
        ] {
            config.insert_json5(key, value).unwrap();
        }
        zenoh::open(config).wait().unwrap()
    }
    fn make_setup(
        capacity: usize,
        max_bytes: usize,
        drops: &Arc<AtomicU64>,
    ) -> (WrenchSetup, Vec<String>, Vec<mpsc::Receiver<Envelope>>) {
        let active = Arc::new(AtomicBool::new(true));
        let mut endpoints = Vec::new();
        let mut keys = Vec::new();
        let mut receivers = Vec::new();
        for channel in [Channel::Once, Channel::Persistent, Channel::Clear] {
            let key = format!("@/m35-{}@/input/{}", uuid::Uuid::new_v4(), channel.index());
            let (input, receiver) = mpsc::sync_channel(capacity);
            keys.push(key.clone());
            receivers.push(receiver);
            endpoints.push(WrenchEndpoint {
                key,
                channel,
                input,
                active: Arc::clone(&active),
                epoch: 7,
            });
        }
        (
            WrenchSetup {
                endpoints,
                max_bytes,
                ingress_drops: Arc::clone(drops),
            },
            keys,
            receivers,
        )
    }
    fn put(session: &zenoh::Session, key: &str, bytes: Vec<u8>, attachment: Option<Vec<u8>>) {
        let put = session.put(key.to_owned(), bytes);
        match attachment {
            Some(value) => put.attachment(value).wait().unwrap(),
            None => put.wait().unwrap(),
        };
    }
    fn receive(input: &mpsc::Receiver<Envelope>) -> Envelope {
        input.recv_timeout(Duration::from_secs(3)).unwrap()
    }
    #[test]
    fn three_actual_channel_types_and_invalid_budget_markers_are_bounded() {
        let session = session();
        let drops = Arc::new(AtomicU64::new(0));
        let (setup, keys, inputs) = make_setup(1, 16, &drops);
        let _declarations = WrenchDeclarations::new(&session, setup).unwrap();
        for channel in [Channel::Once, Channel::Persistent, Channel::Clear] {
            let index = channel.index();
            put(
                &session,
                &keys[index],
                Vec::new(),
                Some(channel.message_type().into_bytes()),
            );
            let message = receive(&inputs[index]);
            assert_eq!(message.epoch, 7);
            match (channel, message.command) {
                (Channel::Clear, Some(Command::Clear(_)))
                | (Channel::Once | Channel::Persistent, Some(Command::Wrench(_))) => {}
                _ => panic!("wrong canonical decoded channel"),
            }
            for (payload, attachment) in [
                (vec![0xff], Some(channel.message_type().into_bytes())),
                (Vec::new(), None),
                (Vec::new(), Some(b"gz.msgs.Double".to_vec())),
                (vec![0; 17], Some(channel.message_type().into_bytes())),
                (Vec::new(), Some(vec![0; 1024])),
            ] {
                put(&session, &keys[index], payload, attachment);
                assert!(receive(&inputs[index]).command.is_none());
                assert!(inputs[index].try_recv().is_err());
            }
        }
        assert_eq!(drops.load(Ordering::Relaxed), 0);
    }
    #[test]
    fn real_full_queue_uses_world_metric_and_retirement_waits_for_callback_capture() {
        let session = session();
        let drops = Arc::new(AtomicU64::new(3));
        let (setup, keys, mut inputs) = make_setup(1, 64, &drops);
        let mut declarations = WrenchDeclarations::new(&session, setup).unwrap();
        for _ in 0..3 {
            put(
                &session,
                &keys[0],
                Vec::new(),
                Some(EntityWrench::full_name().into_bytes()),
            );
        }
        assert_eq!(drops.load(Ordering::Relaxed), 5);
        assert!(receive(&inputs[0]).command.is_some());
        drop(inputs.remove(0));
        put(
            &session,
            &keys[0],
            Vec::new(),
            Some(EntityWrench::full_name().into_bytes()),
        );
        assert_eq!(drops.load(Ordering::Relaxed), 5);
        let capture = Arc::clone(&declarations.entries[0].state);
        let weak = Arc::downgrade(&capture);
        declarations.begin_shutdown();
        assert!(!declarations.reconcile());
        assert!(!declarations.is_empty());
        put(
            &session,
            &keys[1],
            Vec::new(),
            Some(EntityWrench::full_name().into_bytes()),
        );
        assert!(inputs[0].try_recv().is_err());
        drop(capture);
        let deadline = Instant::now() + Duration::from_secs(3);
        while !declarations.reconcile() {
            assert!(Instant::now() < deadline);
            std::thread::yield_now();
        }
        assert!(declarations.is_empty());
        assert_eq!(declarations.entries.capacity(), 0);
        assert!(weak.upgrade().is_none());
    }
    #[test]
    fn invalid_duplicate_closed_and_inactive_setups_never_leave_live_inputs() {
        let session = session();
        let drops = Arc::new(AtomicU64::new(0));
        for kind in ["zero", "missing", "key", "channel"] {
            let (mut setup, _, _) = make_setup(1, 64, &drops);
            match kind {
                "zero" => setup.max_bytes = 0,
                "missing" => {
                    setup.endpoints.pop();
                }
                "key" => setup.endpoints[1].key = setup.endpoints[0].key.clone(),
                "channel" => setup.endpoints[1].channel = Channel::Once,
                _ => unreachable!(),
            }
            assert!(WrenchDeclarations::new(&session, setup).is_err());
        }
        let (setup, _, inputs) = make_setup(1, 64, &drops);
        setup.endpoints[0].active.store(false, Ordering::Release);
        let mut declarations = WrenchDeclarations::new(&session, setup).unwrap();
        assert!(declarations.is_empty());
        assert!(declarations.reconcile());
        for input in inputs {
            assert!(input.try_recv().is_err());
        }
        let empty = WrenchSetup {
            endpoints: Vec::new(),
            max_bytes: 1,
            ingress_drops: Arc::clone(&drops),
        };
        assert!(empty.is_empty());
        assert!(WrenchDeclarations::new(&session, empty).unwrap().is_empty());
        session.close().wait().unwrap();
        let (setup, _, _) = make_setup(1, 64, &drops);
        assert!(WrenchDeclarations::new(&session, setup).is_err());
    }
}
