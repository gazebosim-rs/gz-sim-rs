// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_proto::{msgs::Boolean, prost::Message};
use std::{
    net::TcpListener,
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
    reservation: Option<TcpListener>,
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
        let reservation = TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", reservation.local_addr().unwrap().port());
        let scouting =
            serde_json::json!({"multicast":{"enabled":false},"gossip":{"enabled":false}});
        for (name, config) in [
            (
                "server",
                serde_json::json!({"mode":"peer","listen":{"endpoints":[endpoint]},"scouting":scouting}),
            ),
            (
                "client",
                serde_json::json!({"mode":"peer","listen":{"endpoints":["tcp/127.0.0.1:0"]},"connect":{"endpoints":[endpoint],"timeout_ms":2000},"scouting":scouting,"transport":{"shared_memory":{"enabled":false}}}),
            ),
        ] {
            std::fs::write(
                directory.path().join(format!("{name}.json")),
                serde_json::to_vec(&config).unwrap(),
            )
            .unwrap();
        }
        Self {
            directory,
            reservation: Some(reservation),
            partition,
        }
    }
    fn configure(&self, command: &mut Command, name: &str) {
        command
            .env("GZ_IP", "127.0.0.1")
            .env("GZ_PARTITION", &self.partition)
            .env(
                "ZENOH_CONFIG",
                self.directory.path().join(format!("{name}.json")),
            )
            .env_remove("GZ_RELAY")
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT")
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0");
    }
    pub fn spawn_server(&mut self, command: &mut Command) -> Child {
        self.configure(command, "server");
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
        // A stolen port fails the child's bind; it cannot become a passing peer.
        self.reservation.take();
        command.spawn().unwrap()
    }
    pub fn configure_native(&self, command: &mut Command, peer: &crate::zenoh_peer::NativePeer) {
        self.configure(command, "client");
        peer.configure_runtime(command);
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
    }
    pub fn session(&self) -> zenoh::Session {
        zenoh::open(zenoh::Config::from_file(self.directory.path().join("client.json")).unwrap())
            .wait()
            .unwrap()
    }
    pub fn key(&self, topic: &str) -> String {
        format!("@/{}@{topic}", self.partition)
    }
    pub fn control(&self, session: &zenoh::Session, bytes: Vec<u8>) {
        let key = self.key("/world/default/control");
        let replies = session
            .get(key.clone())
            .payload(bytes)
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap();
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
