// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{msgs::Clock, prost::Message};

#[test]
fn shared_session_clone_uses_actual_world_routing_and_server_shutdown() {
    let mut transport = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        transport.insert_json5(key, value).unwrap();
    }
    let mut server = Server::with_zenoh_config(
        Simulation::new(Duration::from_millis(1)).unwrap(),
        "default",
        ServerOptions {
            partition: format!("shared-world-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        transport,
        NonZeroUsize::new(1).unwrap(),
    )
    .unwrap();
    let shared = server.zenoh_session();
    assert_eq!(shared.zid(), server.world_io.session.zid());
    let observer = shared
        .declare_subscriber(server.options().topic_key("/world/default/clock").unwrap())
        .wait()
        .unwrap();
    drop(server.zenoh_session());
    assert!(
        !shared.is_closed(),
        "dropping a clone must not close world IO"
    );
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    let info = server.update().unwrap();
    assert_eq!(info.sim_time, Duration::from_millis(1));
    let sample = observer
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .unwrap();
    let clock = Clock::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(clock.sim.unwrap().nsec, 1_000_000);
    server.shutdown();
    assert!(
        shared.is_closed(),
        "explicit world shutdown must close every clone"
    );
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}
