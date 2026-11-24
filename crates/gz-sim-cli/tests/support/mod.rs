// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_proto::{msgs::Boolean, prost::Message};
use std::{
    process::{Child, Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc,
    },
    time::Duration,
};
use zenoh::{Wait, pubsub::Subscriber, sample::Sample};

pub struct Network {
    directory: tempfile::TempDir,
    listener: zenoh::Session,
    pub partition: String,
}
impl Network {
    pub fn new() -> Self {
        let directory = tempfile::tempdir().unwrap();
        let partition = format!(
            "cli-{}-{}",
            std::process::id(),
            directory.path().file_name().unwrap().to_str().unwrap()
        );
        let scouting =
            serde_json::json!({"multicast":{"enabled":false},"gossip":{"enabled":false}});
        let config = serde_json::json!({
            "mode":"peer", "listen":{"endpoints":["tcp/127.0.0.1:0"]},
            "connect":{"endpoints":[]}, "scouting":scouting,
            "transport":{"shared_memory":{"enabled":false}}
        });
        let listener = zenoh::open(zenoh::Config::from_json5(&config.to_string()).unwrap())
            .wait()
            .unwrap();
        let locators = listener.info().locators().wait();
        assert_eq!(locators.len(), 1);
        let endpoint = locators[0].to_string();
        assert!(endpoint.starts_with("tcp/127.0.0.1:") && !endpoint.ends_with(":0"));
        let client = serde_json::json!({
            "mode":"client", "listen":{"endpoints":[]},
            "connect":{"endpoints":[endpoint],"timeout_ms":2000}, "scouting":scouting,
            "transport":{"shared_memory":{"enabled":false}}
        });
        std::fs::write(
            directory.path().join("client.json"),
            serde_json::to_vec(&client).unwrap(),
        )
        .unwrap();
        Self {
            directory,
            listener,
            partition,
        }
    }
    fn configure(&self, command: &mut Command) {
        command
            .env("GZ_IP", "127.0.0.1")
            .env("GZ_PARTITION", &self.partition)
            .env("ZENOH_CONFIG", self.directory.path().join("client.json"))
            .env_remove("GZ_RELAY")
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT")
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0");
    }
    pub fn spawn_server(&mut self, command: &mut Command) -> Child {
        self.configure(command);
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
        command.spawn().unwrap()
    }
    pub fn configure_native(&self, command: &mut Command, peer: &crate::zenoh_peer::NativePeer) {
        self.configure(command);
        peer.configure_runtime(command);
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
    }
    pub fn session(&self) -> zenoh::Session {
        self.listener.clone()
    }
    pub fn key(&self, topic: &str) -> String {
        format!("@/{}@{topic}", self.partition)
    }
    pub fn control(&self, session: &zenoh::Session, bytes: Vec<u8>) {
        let key = self.key("/world/default/control");
        let control = session
            .declare_querier(key.clone())
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap();
        let changes = control.matching_listener().wait().unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while !control.matching_status().wait().unwrap().matching() {
            assert!(
                changes
                    .recv_timeout(deadline.saturating_duration_since(std::time::Instant::now()))
                    .unwrap()
                    .is_some(),
                "world control service never matched"
            );
        }
        let replies = control.get().payload(bytes).wait().unwrap();
        let sample = replies
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .expect("missing control reply")
            .into_result()
            .unwrap();
        assert_eq!(sample.key_expr().as_str(), key);
        assert!(
            Boolean::decode(sample.payload().to_bytes().as_ref())
                .unwrap()
                .data
        );
    }
}

pub struct Samples {
    _subscriber: Subscriber<()>,
    receiver: mpsc::Receiver<Sample>,
    drops: Arc<AtomicU64>,
}
impl Samples {
    pub fn new(session: &zenoh::Session, key: String) -> Self {
        // Retain the old client default bound, while callbacks never block IO.
        let (send, receiver) = mpsc::sync_channel(1000);
        let drops = Arc::new(AtomicU64::new(0));
        let overflow = Arc::clone(&drops);
        let subscriber = session
            .declare_subscriber(key)
            .callback(move |sample| {
                if let Err(mpsc::TrySendError::Full(_)) = send.try_send(sample) {
                    overflow.fetch_add(1, Ordering::Relaxed);
                }
            })
            .wait()
            .unwrap();
        Self {
            _subscriber: subscriber,
            receiver,
            drops,
        }
    }
    pub fn recv_timeout(&self, timeout: Duration) -> Sample {
        let sample = self.receiver.recv_timeout(timeout).unwrap();
        assert_eq!(
            self.drops.load(Ordering::Relaxed),
            0,
            "clock client queue overflowed"
        );
        assert_eq!(
            sample.attachment().unwrap().to_bytes().as_ref(),
            b"gz.msgs.Clock"
        );
        sample
    }
}
