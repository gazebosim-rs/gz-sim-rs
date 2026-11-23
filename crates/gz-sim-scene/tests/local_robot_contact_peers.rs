// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent material/contact cross-check in a scene with an active welded carrier.
#![allow(deprecated)]
use gz_sim_scene::{Name, PhysicsEngine, Velocity, load_world_with_sdf_step, reset_loaded_world};

#[test]
fn peer_to_peer_rebound_and_momentum_match_with_a_weld_in_the_same_native_world() {
    let bounce = include_str!("../../../examples/local_robot_bounce.sdf");
    let start = bounce.find("<model name=\"ball\">").unwrap();
    let end = bounce[start..].find("</model>").unwrap() + start + "</model>".len();
    let ball = &bounce[start..end];
    let first = ball
        .replace("\"ball\"", "\"first\"")
        .replace("0 0 0.23", "3 0 2")
        .replace("0 0 -2", "1 0 0");
    let second = ball
        .replace("\"ball\"", "\"second\"")
        .replace("0 0 0.23", "3.2 0 2")
        .replace("0 0 -2", "-1 0 0")
        .replace(">0.65<", ">0.2<");
    let scene = include_str!("../../../examples/local_robot_contact.sdf")
        .replace("</world>", &format!("{first}{second}</world>"));
    let mut sim = load_world_with_sdf_step(
        &scene,
        "peers.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    for _ in 0..2 {
        sim.step_exact(100).unwrap();
        let velocity = |name| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<Velocity>()
                .unwrap()
                .linear
        };
        let a = velocity("default::first::body");
        let b = velocity("default::second::body");
        assert!(
            (a[0] + 0.65).abs() < 1e-8 && (b[0] - 0.65).abs() < 1e-8,
            "{a:?}, {b:?}"
        );
        assert!((a[0] + b[0]).abs() < 1e-12);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}
