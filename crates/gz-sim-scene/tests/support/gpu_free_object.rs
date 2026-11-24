// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{Velocity, gpu_scene::planar_tool::observe_object};

const SCENE: &str = include_str!("../../../../examples/gpu_planar_free_object.sdf");

#[test]
fn unsupported_objects_are_rejected_before_device_selection() {
    for (old, new, reason) in [
        (
            "object=\"ball\"",
            "object=\"unknown\"",
            "unknown free sphere",
        ),
        (
            "object=\"ball\"",
            "object=\"floor\"",
            "dynamic solid sphere",
        ),
        ("0.503 0 0.672", "0.503 0.01 0.672", "coplanar"),
        ("<mass>0.08", "<mass>0.25", "solid-sphere inertia"),
        (
            "<effort>8</effort>",
            "<effort>8</effort><upper>1</upper>",
            "unlimited hinges",
        ),
        (
            "<link name=\"body\">",
            "<link name=\"body\"><kinematic>true</kinematic>",
            "coplanar",
        ),
        (
            "<link name=\"body\">",
            "<link name=\"body\"><gravity>false</gravity>",
            "coplanar",
        ),
        ("<mu>0</mu><mu2>0", "<mu>0.2</mu><mu2>0.2", "frictionless"),
    ] {
        let reason_seen = load(&SCENE.replace(old, new), Some("missing device"))
            .err()
            .unwrap()
            .to_string();
        assert!(reason_seen.contains(reason), "{reason}: {reason_seen}");
    }
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn loaded_free_sphere_reacts_publishes_and_repeats_after_reset() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut sim = load(SCENE, Some(&adapter)).unwrap();
    let find_ball = |sim: &Simulation| {
        sim.world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::ball::body")
            })
            .unwrap()
            .id()
    };
    assert_eq!(execution_counts(&sim), Some((0, 1)));
    assert_eq!(
        environment_placement(&sim, 7).unwrap(),
        ExecutionPlacement::Gpu
    );
    assert!(matches!(
        observe_object(&sim, 999),
        Err(GpuError::UnknownEnvironment(999))
    ));
    let mut trace = Vec::new();
    for episode in 0..2 {
        if episode == 1 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        assert_eq!(observe_object(&sim, 7).unwrap().unwrap().normal_impulse, 0.);
        let ball = find_ball(&sim);
        set_targets(&mut sim, 7, [-0.84, 0.12]).unwrap();
        let mut first = None;
        for tick in 1..=400 {
            sim.step_exact(1).unwrap();
            let object = observe_object(&sim, 7).unwrap().unwrap();
            let pose = sim.world().get::<WorldPose>(ball).unwrap();
            let velocity = sim.world().get::<Velocity>(ball).unwrap();
            assert_eq!(
                pose.translation,
                [object.position[0], 0., 0.60 + object.position[1]]
            );
            assert_eq!(
                velocity.linear,
                [object.velocity[0], 0., object.velocity[1]]
            );
            assert_eq!(velocity.angular, [0.; 3]);
            if object.normal_impulse > 0. {
                first.get_or_insert(tick);
            }
            let row = (
                object.position,
                object.velocity,
                object.normal_impulse,
                object.floor_impulse,
                observe(&sim, 7).unwrap().arm.speed,
            );
            if episode == 0 {
                trace.push(row);
            } else {
                assert_eq!(trace[tick - 1], row);
            }
        }
        assert_eq!(first, Some(21));
        let object = observe_object(&sim, 7).unwrap().unwrap();
        assert!((object.position[0] - 0.903122).abs() < 1e-5);
        assert_eq!(object.normal_impulse, 0.);
        assert!((object.floor_impulse - 0.001 * 0.08 * 9.81).abs() < 1e-12);
    }
    let ball = find_ball(&sim);
    sim.world_mut().entity_mut(ball).remove::<Velocity>();
    assert!(sim.step_exact(1).is_err());
    assert!(matches!(
        set_targets(&mut sim, 7, [0.; 2]),
        Err(GpuError::StaleBoundary)
    ));
    let original = load(WORLD, Some(&adapter)).unwrap();
    assert!(observe_object(&original, 7).unwrap().is_none());
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn ordinary_motor_commands_reach_the_free_body_and_validate_axis_mode() {
    let ball =
        &SCENE[SCENE.find("<model name=\"ball\">").unwrap()..SCENE.find("  </world>").unwrap()];
    let scene = COMMAND_WORLD
        .replace(
            "plane=\"floor::base\"",
            "plane=\"floor::base\" object=\"ball\"",
        )
        .replace("<mu>0.30</mu><mu2>0.30", "<mu>0</mu><mu2>0")
        .replace("</world>", &format!("{ball}</world>"));
    let mut sim = load(&scene, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    let ids: Vec<_> = gz_sim_scene::initial_joint_controller_configurations(&sim)
        .unwrap()
        .map(|(id, _)| id)
        .collect();
    assert_eq!(ids.len(), 2);
    {
        let mut batch = sim
            .world_mut()
            .resource_mut::<gz_physics_gpu::PlanarObjectBatch>();
        assert!(batch.set_target_angles(4, [0.; 2]).is_err());
        assert!(batch.set_motor_speed(4, 0, 0.).is_err());
        assert!(batch.set_motor_speed(0, 2, 0.).is_err());
        assert!(batch.set_motor_speed(0, 0, f64::NAN).is_err());
    }
    gz_sim_scene::set_joint_controller_command(&mut sim, ids[0], -8.).unwrap();
    gz_sim_scene::set_joint_controller_command(&mut sim, ids[1], 0.12).unwrap();
    let mut total = 0.;
    for _ in 0..200 {
        sim.step_exact(1).unwrap();
        total += observe_object(&sim, 7).unwrap().unwrap().normal_impulse;
    }
    assert!(total > 0.01);
    assert!(observe_object(&sim, 7).unwrap().unwrap().position[0] > 0.52);
}
