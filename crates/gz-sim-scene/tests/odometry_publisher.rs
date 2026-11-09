// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    GroundTruthOdometryPublication, GroundTruthOdometrySample, Name, PhysicsEngine,
    load_world_with_engine, queue_link_world_velocity, remove_model, reset_loaded_world,
};
use std::time::Duration;
const XML: &str = include_str!("../../../examples/m34_odometry_publisher.sdf");
fn load(xml: &str, step: Duration) -> Simulation {
    load_world_with_engine(
        xml,
        "m34.sdf",
        "default",
        step,
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
fn sample(sim: &Simulation) -> GroundTruthOdometrySample {
    *sim.world()
        .get::<GroundTruthOdometrySample>(entity(sim, "default::rig"))
        .unwrap()
}
fn values(s: GroundTruthOdometrySample) -> ([f64; 3], [f64; 4], [f64; 3], [f64; 3]) {
    (s.pose.translation, s.pose.rotation, s.linear, s.angular)
}
#[test]
fn actual_model_motion_history_throttle_pause_and_loaded_reset_preserve_native_transient() {
    let mut sim = load(XML, Duration::from_millis(1));
    let model = entity(&sim, "default::rig");
    let id = sim.world().get::<EntityId>(model).unwrap().get();
    let publication = sim
        .world()
        .get::<GroundTruthOdometryPublication>(model)
        .unwrap();
    assert_eq!(
        (&publication.odom_frame, &publication.robot_base_frame),
        (&"m34/odom".to_owned(), &"m34/base".to_owned())
    );
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<GroundTruthOdometrySample>(model)
            .is_none()
    );
    sim.step_exact(19).unwrap();
    let stationary = sample(&sim);
    assert_eq!(stationary.time, Duration::from_millis(20));
    assert_eq!(stationary.pose.translation, [2.0, -1.0, 0.0]);
    assert_eq!(stationary.linear, [0.0; 3]);
    assert_eq!(stationary.angular, [0.0; 3]);
    let link = sim
        .world()
        .get::<EntityId>(entity(&sim, "default::rig::body"))
        .unwrap()
        .get();
    queue_link_world_velocity(&mut sim, link, [1.0, 0.5, 0.0], [0.0, 0.0, 0.4]).unwrap();
    sim.step_exact(20).unwrap();
    let moved = sample(&sim);
    assert!(moved.pose.translation[0] > stationary.pose.translation[0]);
    assert!(moved.angular[2] > 0.39 && moved.angular[2] < 0.41);
    assert_ne!(moved.pose.rotation, stationary.pose.rotation);
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(values(sample(&sim)), values(moved));
    assert_eq!(sample(&sim).time, moved.time);
    sim.set_paused(false);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let model = entity(&sim, "default::rig");
    assert_eq!(sim.world().get::<EntityId>(model).unwrap().get(), id);
    assert!(
        sim.world()
            .get::<GroundTruthOdometrySample>(model)
            .is_none()
    );
    sim.step_exact(20).unwrap();
    assert_eq!(values(sample(&sim)), values(stationary));
    assert_eq!(sample(&sim).epoch, 1);
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<GroundTruthOdometryPublication>())
    );
}
#[test]
fn unthrottled_nonzero_pose_transient_fills_and_exits_ten_sample_window() {
    let xml = XML.replace(
        "<odom_publish_frequency>50</odom_publish_frequency>",
        "<odom_publish_frequency>0</odom_publish_frequency>",
    );
    let mut sim = load(&xml, Duration::from_millis(1));
    sim.step_exact(2).unwrap();
    let first = sample(&sim);
    assert_eq!(first.time, Duration::from_millis(2));
    assert!(first.linear[0].abs() > 100.0 && first.angular[2] > 100.0);
    for count in 2..=10 {
        sim.step_exact(1).unwrap();
        assert!((sample(&sim).linear[0] - first.linear[0] / count as f64).abs() < 1e-10);
    }
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim).linear, [0.0; 3]);
    assert_eq!(sample(&sim).angular, [0.0; 3]);
}
#[test]
fn admission_rejects_unsupported_or_ambiguous_metadata_without_losing_old_loader_domain() {
    let plugin = "<dimensions>2</dimensions>";
    for fields in [
        "<dimensions>3</dimensions>",
        "<dimensions>2</dimensions><dimensions>2</dimensions>",
        "<dimensions>2</dimensions><gaussian_noise>0.01</gaussian_noise>",
        "<dimensions>2</dimensions><xyz_offset>0 1 0</xyz_offset>",
        "<dimensions>2</dimensions><rpy_global_offset>0 0 NaN</rpy_global_offset>",
        "<dimensions>2</dimensions><unknown>0</unknown>",
        "<dimensions>2<!-- gap -->garbage</dimensions>",
        "<dimensions><![CDATA[2]]></dimensions>",
        "<dimensions value='2'>2</dimensions>",
        "<dimensions>2</dimensions><odom_frame> </odom_frame>",
    ] {
        assert!(
            load_world_with_engine(
                &XML.replace(plugin, fields),
                "bad.sdf",
                "default",
                Duration::from_millis(1),
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{fields}"
        );
    }
    let xml = XML.replace(
        "<odom_topic>/m34/odometry</odom_topic>",
        "<odom_topic>/m34/transform</odom_topic>",
    );
    assert!(
        load_world_with_engine(
            &xml,
            "bad.sdf",
            "default",
            Duration::from_millis(1),
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    assert!(
        load_world_with_engine(
            XML,
            "bad.sdf",
            "default",
            Duration::from_millis(1),
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
    let plain =
        XML.split("      <plugin").next().unwrap().to_owned() + "    </model></world></sdf>";
    let mut sim = load(&plain, Duration::from_millis(1));
    sim.step_exact(20).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<GroundTruthOdometrySample>())
    );
}
