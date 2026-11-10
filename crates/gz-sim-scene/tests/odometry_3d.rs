// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, StepError, StepFault};
use gz_sim_scene::{
    GroundTruthOdometryDimensions as Dimensions, GroundTruthOdometryError,
    GroundTruthOdometryPublication, GroundTruthOdometrySample, GroundTruthOdometryScheduleFault,
    Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step, queue_link_world_wrench,
    remove_model, reset_loaded_world,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m38_odometry_3d.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m38.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn owner(sim: &Simulation) -> bevy_ecs::entity::Entity {
    entity(sim, "default::rig")
}
fn link(sim: &Simulation) -> bevy_ecs::entity::Entity {
    entity(sim, "default::rig::body")
}
fn body_id(sim: &Simulation) -> u64 {
    sim.world().get::<EntityId>(link(sim)).unwrap().get()
}
fn sample(sim: &Simulation) -> GroundTruthOdometrySample {
    *sim.world()
        .get::<GroundTruthOdometrySample>(owner(sim))
        .unwrap()
}
fn bits(sample: GroundTruthOdometrySample) -> ([u64; 3], [u64; 4], [u64; 3], [u64; 3]) {
    (
        sample.pose.translation.map(f64::to_bits),
        sample.pose.rotation.map(f64::to_bits),
        sample.linear.map(f64::to_bits),
        sample.angular.map(f64::to_bits),
    )
}
#[test]
fn dimensions_three_publishes_full_pose_and_body_twist_under_actual_rapier_motion() {
    let mut sim = load(XML);
    let metadata = sim
        .world()
        .get::<GroundTruthOdometryPublication>(owner(&sim))
        .unwrap();
    assert_eq!(metadata.dimensions, Dimensions::Three);
    assert_eq!(
        (
            metadata.odom_frame.as_str(),
            metadata.robot_base_frame.as_str()
        ),
        ("m38/odom", "m38/base")
    );
    assert_eq!(
        (
            metadata.odom_topic.as_str(),
            metadata.covariance_topic.as_str(),
            metadata.tf_topic.as_str()
        ),
        (
            "/m38/odometry",
            "/m38/odometry_covariance",
            "/m38/transform"
        )
    );
    sim.step_exact(20).unwrap();
    let stationary = sample(&sim);
    assert_eq!(stationary.pose.translation, [2.0, -1.0, 3.0]);
    assert_eq!(stationary.linear, [0.0; 3]);
    assert_eq!(stationary.angular, [0.0; 3]);
    let id = body_id(&sim);
    queue_link_world_wrench(&mut sim, id, [4.0, -2.0, 6.0], [2.0, 4.0, 6.0]).unwrap();
    sim.step_exact(1).unwrap();
    let velocity = sim.world().get::<Velocity>(link(&sim)).unwrap();
    // Independent mass/inertia integration check, not a filtered-Twist oracle.
    for (actual, expected) in velocity.linear.into_iter().zip([0.002, -0.001, 0.003]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    for (actual, expected) in velocity.angular.into_iter().zip([0.002, 0.004, 0.006]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    sim.step_exact(19).unwrap();
    let moved = sample(&sim);
    assert_eq!(moved.time, Duration::from_millis(40));
    assert!(moved.pose.translation[2] > stationary.pose.translation[2]);
    assert_ne!(moved.pose.rotation, stationary.pose.rotation);
    assert!(moved.linear.into_iter().all(|v| v.is_finite() && v != 0.0));
    assert!(moved.angular.into_iter().all(|v| v.is_finite() && v != 0.0));
    let world = sim.world().get::<WorldPose>(owner(&sim)).unwrap();
    assert_eq!(
        moved.pose.translation.map(f64::to_bits),
        world.translation.map(f64::to_bits)
    );
    assert_eq!(
        moved.pose.rotation.map(f64::to_bits),
        world.rotation.map(f64::to_bits)
    );
}
#[test]
fn three_dimensional_history_updates_before_throttle_and_freezes_when_paused() {
    let every_xml = XML.replace(
        "<odom_publish_frequency>50</odom_publish_frequency>",
        "<odom_publish_frequency>0</odom_publish_frequency>",
    );
    let mut every = load(&every_xml);
    let mut limited = load(XML);
    every.step_exact(20).unwrap();
    limited.step_exact(20).unwrap();
    let a = body_id(&every);
    let b = body_id(&limited);
    queue_link_world_wrench(&mut every, a, [2.0, 4.0, 6.0], [1.0, 2.0, 3.0]).unwrap();
    queue_link_world_wrench(&mut limited, b, [2.0, 4.0, 6.0], [1.0, 2.0, 3.0]).unwrap();
    for tick in 21..=40 {
        every.step_exact(1).unwrap();
        limited.step_exact(1).unwrap();
        if tick < 40 {
            assert_eq!(sample(&limited).time, Duration::from_millis(20));
        }
    }
    assert_eq!(bits(sample(&limited)), bits(sample(&every)));
    assert_ne!(sample(&limited).revision, sample(&every).revision);
    limited.set_paused(true);
    let held = sample(&limited);
    let time = limited.elapsed();
    for _ in 0..3 {
        limited.tick().unwrap();
    }
    assert_eq!(limited.elapsed(), time);
    assert_eq!(bits(sample(&limited)), bits(held));
    assert_eq!(
        (sample(&limited).time, sample(&limited).revision),
        (held.time, held.revision)
    );
}
#[test]
fn dimensions_identity_survives_loaded_reset_and_stale_mutation_faults() {
    let mut sim = load(XML);
    let id = sim.world().get::<EntityId>(owner(&sim)).unwrap().get();
    sim.step_exact(20).unwrap();
    let initial = sample(&sim);
    let body = body_id(&sim);
    queue_link_world_wrench(&mut sim, body, [0.0, 0.0, 4.0], [2.0, 0.0, 0.0]).unwrap();
    sim.step_exact(20).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let model = owner(&sim);
    assert_eq!(sim.world().get::<EntityId>(model).unwrap().get(), id);
    assert_eq!(
        sim.world()
            .get::<GroundTruthOdometryPublication>(model)
            .unwrap()
            .dimensions,
        Dimensions::Three
    );
    assert!(
        sim.world()
            .get::<GroundTruthOdometrySample>(model)
            .is_none()
    );
    sim.step_exact(20).unwrap();
    assert_eq!(bits(sample(&sim)), bits(initial));
    assert_eq!(
        (sample(&sim).epoch, sample(&sim).revision),
        (1, initial.revision)
    );
    sim.world_mut()
        .get_mut::<GroundTruthOdometryPublication>(model)
        .unwrap()
        .dimensions = Dimensions::Two;
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    assert!(
        sim.world()
            .get::<GroundTruthOdometrySample>(model)
            .is_none()
    );
    let fault = sim
        .world()
        .resource::<StepFault>()
        .error()
        .unwrap()
        .downcast_ref::<GroundTruthOdometryScheduleFault>()
        .unwrap();
    assert_eq!(
        (fault.entity, fault.source),
        (id, GroundTruthOdometryError::Identity)
    );
    // Core faults are terminal: loaded reset preflights the clock before
    // invoking observers or changing scene identity, physics or metadata.
    let clock_before = (sim.epoch(), sim.elapsed(), sim.is_paused());
    let metadata_before = sim
        .world()
        .get::<GroundTruthOdometryPublication>(model)
        .unwrap()
        .clone();
    let pose_before = *sim.world().get::<WorldPose>(model).unwrap();
    let velocity_before = sim.world().get::<Velocity>(link(&sim)).unwrap();
    let velocity_bits = (
        velocity_before.linear.map(f64::to_bits),
        velocity_before.angular.map(f64::to_bits),
    );
    let mut observed = false;
    let result = reset_loaded_world(&mut sim, |_| observed = true);
    assert!(matches!(
        result,
        Err(gz_sim_scene::SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!observed);
    assert!(sim.is_faulted());
    assert_eq!((sim.epoch(), sim.elapsed(), sim.is_paused()), clock_before);
    assert_eq!(owner(&sim), model);
    assert_eq!(sim.world().get::<EntityId>(model).unwrap().get(), id);
    assert_eq!(
        sim.world()
            .get::<GroundTruthOdometryPublication>(model)
            .unwrap(),
        &metadata_before
    );
    assert_eq!(metadata_before.dimensions, Dimensions::Two);
    let pose_after = sim.world().get::<WorldPose>(model).unwrap();
    assert_eq!(
        pose_after.translation.map(f64::to_bits),
        pose_before.translation.map(f64::to_bits)
    );
    assert_eq!(
        pose_after.rotation.map(f64::to_bits),
        pose_before.rotation.map(f64::to_bits)
    );
    let velocity_after = sim.world().get::<Velocity>(link(&sim)).unwrap();
    assert_eq!(
        (
            velocity_after.linear.map(f64::to_bits),
            velocity_after.angular.map(f64::to_bits)
        ),
        velocity_bits
    );
    assert!(
        sim.world()
            .get::<GroundTruthOdometrySample>(model)
            .is_none()
    );
    let fault = sim
        .world()
        .resource::<StepFault>()
        .error()
        .unwrap()
        .downcast_ref::<GroundTruthOdometryScheduleFault>()
        .unwrap();
    assert_eq!(
        (fault.entity, fault.source),
        (id, GroundTruthOdometryError::Identity)
    );
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    assert_eq!((sim.epoch(), sim.elapsed(), sim.is_paused()), clock_before);
}
#[test]
fn three_dimensional_removal_retires_without_retargeting_commands() {
    let mut sim = load(XML);
    sim.step_exact(20).unwrap();
    let model = sim.world().get::<EntityId>(owner(&sim)).unwrap().get();
    let body = body_id(&sim);
    queue_link_world_wrench(&mut sim, body, [0.0, 0.0, 4.0], [1.0, 2.0, 3.0]).unwrap();
    remove_model(&mut sim, model).unwrap();
    sim.step_exact(20).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<GroundTruthOdometryPublication>()
                || e.contains::<GroundTruthOdometrySample>())
    );
    assert!(queue_link_world_wrench(&mut sim, body, [1.0; 3], [0.0; 3]).is_err());
    // The combined model has a plugin: reconstruction is loaded reset, not cloning.
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.world().get::<EntityId>(owner(&sim)).unwrap().get(),
        model
    );
    assert_eq!(body_id(&sim), body);
    sim.step_exact(20).unwrap();
    assert_eq!(sample(&sim).epoch, 1);
    assert_eq!(sample(&sim).pose.translation, [2.0, -1.0, 3.0]);
}
#[test]
fn dimensions_admission_preserves_planar_defaults_and_refuses_other_metadata() {
    for replacement in [
        "",
        "<dimensions>2</dimensions>",
        "<dimensions>3</dimensions>",
    ] {
        let mut sim = load(&XML.replace("<dimensions>3</dimensions>", replacement));
        let dimension = sim
            .world()
            .get::<GroundTruthOdometryPublication>(owner(&sim))
            .unwrap()
            .dimensions;
        assert_eq!(
            dimension,
            if replacement.contains('3') {
                Dimensions::Three
            } else {
                Dimensions::Two
            }
        );
        sim.step_exact(20).unwrap();
        assert_eq!(
            sample(&sim).pose.translation[2],
            if dimension == Dimensions::Three {
                3.0
            } else {
                0.0
            }
        );
    }
    for fields in [
        "<dimensions>0</dimensions>",
        "<dimensions>1</dimensions>",
        "<dimensions>4</dimensions>",
        "<dimensions>-1</dimensions>",
        "<dimensions>bad</dimensions>",
        "<dimensions>3</dimensions><dimensions>3</dimensions>",
        "<dimensions>3<!-- gap -->garbage</dimensions>",
        "<dimensions><![CDATA[3]]></dimensions>",
        "<dimensions><nested/></dimensions>",
        "<dimensions value='3'>3</dimensions>",
        "<dimensions>3</dimensions><gaussian_noise>0.01</gaussian_noise>",
        "<dimensions>3</dimensions><xyz_offset>0 0 1</xyz_offset>",
    ] {
        assert!(
            load_world_with_sdf_step(
                &XML.replace("<dimensions>3</dimensions>", fields),
                "bad.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{fields}"
        );
    }
    assert!(
        load_world_with_sdf_step(
            XML,
            "bad.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
    let plain = XML
        .split("      <plugin filename=")
        .next()
        .unwrap()
        .to_owned()
        + "    </model></world></sdf>";
    let mut absent = load(&plain);
    absent.step_exact(20).unwrap();
    assert!(
        !absent
            .world()
            .iter_entities()
            .any(|e| e.contains::<GroundTruthOdometryPublication>()
                || e.contains::<GroundTruthOdometrySample>())
    );
}
