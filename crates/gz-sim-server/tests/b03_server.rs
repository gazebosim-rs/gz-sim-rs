// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Server integration only; the separate B03 reference checks native trajectories.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    EntityKind, JointSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
};
use gz_sim_server::{Server, ServerOptions};
use sdformat::NoResources;
use std::{collections::BTreeMap, num::NonZeroUsize, time::Duration};

const XML: &str = include_str!("../../gz-sim-scene/tests/fixtures/b03_double_pendulum.sdf");
const BODIES: [&str; 7] = [
    "default::ground_plane::link",
    "default::double_pendulum_with_base0::base",
    "default::double_pendulum_with_base0::upper_link",
    "default::double_pendulum_with_base0::lower_link",
    "default::double_pendulum_with_base1::base",
    "default::double_pendulum_with_base1::upper_link",
    "default::double_pendulum_with_base1::lower_link",
];
const JOINTS: [&str; 4] = [
    "default::double_pendulum_with_base0::upper_joint",
    "default::double_pendulum_with_base0::lower_joint",
    "default::double_pendulum_with_base1::upper_joint",
    "default::double_pendulum_with_base1::lower_joint",
];

struct Snapshot {
    identities: BTreeMap<String, u64>,
    values: BTreeMap<String, Vec<f64>>,
}

fn snapshot(simulation: &Simulation, tick: u64) -> Snapshot {
    assert_eq!(simulation.elapsed(), Duration::from_millis(tick));
    let mut identities = BTreeMap::new();
    let mut values = BTreeMap::new();
    let mut bodies = 0;
    let mut joints = 0;
    for entity in simulation.world().iter_entities() {
        let Some(kind) = entity.get::<EntityKind>() else {
            continue;
        };
        assert_ne!(*kind, EntityKind::Sensor);
        if !matches!(kind, EntityKind::Link | EntityKind::Joint) {
            continue;
        }
        let name = &entity.get::<Name>().unwrap().0;
        assert!(
            identities
                .insert(name.clone(), entity.get::<EntityId>().unwrap().get())
                .is_none()
        );
        let row = if *kind == EntityKind::Link {
            assert!(BODIES.contains(&name.as_str()));
            bodies += 1;
            let pose = entity.get::<WorldPose>().unwrap();
            let velocity = entity.get::<Velocity>().unwrap();
            pose.translation
                .into_iter()
                .chain(pose.rotation)
                .chain(velocity.linear)
                .chain(velocity.angular)
                .collect::<Vec<_>>()
        } else {
            assert!(JOINTS.contains(&name.as_str()));
            joints += 1;
            let sample = entity.get::<JointSample>().unwrap();
            assert_eq!(sample.time, simulation.elapsed());
            assert_eq!(sample.epoch, 0);
            vec![sample.state.position, sample.state.velocity]
        };
        assert!(row.iter().all(|value| value.is_finite()));
        assert!(values.insert(name.clone(), row).is_none());
    }
    assert_eq!((bodies, joints), (7, 4));
    Snapshot { identities, values }
}

fn same_physics(server: &Snapshot, scene: &Snapshot, tick: u64) {
    assert_eq!(
        server.values.keys().collect::<Vec<_>>(),
        scene.values.keys().collect::<Vec<_>>()
    );
    for (name, actual) in &server.values {
        let expected = &scene.values[name];
        assert_eq!(actual.len(), expected.len());
        for (field, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (actual - expected).abs() <= 1e-12,
                "tick{tick} {name} field{field}: Server {actual:.17e}, Scene {expected:.17e}"
            );
        }
    }
}

#[test]
fn loaded_b03_server_preserves_scene_motion_and_identity_through_shutdown() {
    let load = || {
        let mut simulation =
            load_world_with_sdf_step(XML, "b03.sdf", "default", &NoResources, PhysicsEngine::Dart)
                .unwrap();
        simulation.set_paused(false);
        simulation
    };
    let mut scene = load();
    let mut network = zenoh::Config::default();
    for (path, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        network.insert_json5(path, value).unwrap();
    }
    let mut server = Server::with_zenoh_config(
        load(),
        "default",
        ServerOptions {
            partition: format!("b03-server-{}", uuid::Uuid::new_v4()),
            ..ServerOptions::default()
        },
        network,
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let server_initial = snapshot(server.simulation(), 0);
    let scene_initial = snapshot(&scene, 0);
    same_physics(&server_initial, &scene_initial, 0);
    for tick in 1..=100 {
        scene.step_exact(1).unwrap();
        let info = server.update().unwrap();
        assert_eq!(info.sim_time, Duration::from_millis(tick));
        assert_eq!(info.dt, Duration::from_millis(1));
        assert_eq!(info.iteration, tick);
        assert_eq!(info.epoch, 0);
        assert!(!info.paused);
        let actual = snapshot(server.simulation(), tick);
        let expected = snapshot(&scene, tick);
        assert_eq!(actual.identities, server_initial.identities);
        assert_eq!(expected.identities, scene_initial.identities);
        same_physics(&actual, &expected, tick);
    }
    let before = snapshot(server.simulation(), 100);
    server.shutdown();
    assert!(server.is_shutdown());
    assert!(matches!(
        server.update(),
        Err(gz_sim_server::Error::WorldClosed)
    ));
    let after = snapshot(server.simulation(), 100);
    assert_eq!(after.identities, before.identities);
    same_physics(&after, &before, 100);
}
