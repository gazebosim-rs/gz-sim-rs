// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_proto::msgs::WorldControl;
use gz_sim_scene::{Name, PhysicsEngine, Velocity, load_world_with_sdf_step};
use gz_sim_server::{Server, ServerOptions};
use std::{
    io::{Read, Write},
    net::TcpListener,
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::OnceLock,
    time::{Duration, Instant},
};
use zenoh::Wait;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;
struct Peer(Child);
impl Drop for Peer {
    fn drop(&mut self) {
        if self.0.try_wait().unwrap().is_none() {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}
fn peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        zenoh_peer::NativePeer::build(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            "velocity_control",
        )
    })
}
#[test]
fn actual_cpp_twist_preserves_all_six_axes_and_invalid_message_cannot_replace_pending_target() {
    let reservation = TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/{}", reservation.local_addr().unwrap());
    let scouting = serde_json::json!({"multicast":{"enabled":false},"gossip":{"enabled":false}});
    let router_config =
        serde_json::json!({"mode":"router","listen":{"endpoints":[endpoint]},"scouting":scouting});
    let client = serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":[endpoint],"timeout_ms":2000},"scouting":scouting,"transport":{"shared_memory":{"enabled":false}}});
    let directory = tempfile::tempdir().unwrap();
    std::fs::write(
        directory.path().join("client.json"),
        serde_json::to_vec(&client).unwrap(),
    )
    .unwrap();
    drop(reservation);
    let router = zenoh::open(zenoh::Config::from_json5(&router_config.to_string()).unwrap())
        .wait()
        .unwrap();
    let sim = load_world_with_sdf_step(
        include_str!("fixtures/m79_velocity_control.sdf"),
        "m79-cpp.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let options = ServerOptions {
        partition: format!("m79-cpp-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        options.clone(),
        zenoh::Config::from_json5(&client.to_string()).unwrap(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    let native = peer();
    let mut command = Command::new(&native.binary);
    native.configure_runtime(&mut command);
    command
        .env("GZ_PARTITION", &options.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("ZENOH_CONFIG", directory.path().join("client.json"))
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_DISCOVERY_MSG_PORT")
        .env_remove("GZ_DISCOVERY_SRV_PORT")
        .env_remove("GZ_DISCOVERY_MULTICAST_IP")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = Peer(command.spawn().unwrap());
    let deadline = Instant::now() + Duration::from_secs(35);
    while server.counters().velocity_control_messages_received < 2 {
        assert!(
            Instant::now() < deadline,
            "two real canonical callback receipts deadline"
        );
        assert!(
            child.0.try_wait().unwrap().is_none(),
            "peer exited before owned completion"
        );
        server.update().unwrap();
        std::thread::yield_now();
    }
    assert_eq!(server.counters().velocity_control_messages_received, 2);
    assert_eq!(server.counters().velocity_control_messages_rejected, 1);
    assert_eq!(server.counters().velocity_control_commands_pending, 1);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    server.update().unwrap();
    server.update().unwrap();
    let model = server
        .simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig::body"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let motion = server
        .simulation()
        .world()
        .get::<Velocity>(entity_by_id(server.simulation().world(), model).unwrap())
        .unwrap();
    for (actual, expected) in motion
        .linear
        .into_iter()
        .chain(motion.angular)
        .zip([1.0, -2.0, 3.0, 0.5, -0.75, 1.25])
    {
        assert!((actual - expected).abs() < 1e-12, "{actual} vs {expected}");
    }
    child
        .0
        .stdin
        .take()
        .unwrap()
        .write_all(b"finish\n")
        .unwrap();
    let status = loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            break status;
        }
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    };
    let mut stdout = Vec::new();
    let mut stderr = Vec::new();
    child
        .0
        .stdout
        .take()
        .unwrap()
        .take(4097)
        .read_to_end(&mut stdout)
        .unwrap();
    child
        .0
        .stderr
        .take()
        .unwrap()
        .take(4097)
        .read_to_end(&mut stderr)
        .unwrap();
    assert!(
        status.success(),
        "{status}: {}",
        String::from_utf8_lossy(&stderr)
    );
    assert!(stdout.len() <= 4096 && stderr.is_empty());
    assert_eq!(
        stdout,
        b"SENT six-axis Twist and invalid control\nPASS canonical six-axis VelocityControl\n"
    );
    server.shutdown();
    router.close().wait().unwrap();
}
