// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointHandle, LocalPose, Name, PhysicsEngine, PosePublication, PoseSample, WorldPose,
    load_world_with_sdf_step, reset_loaded_world,
};
const XML: &str = include_str!("fixtures/m27_pose_publisher.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m27-public.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
#[test]
fn public_pose_frames_are_actual_parent_relative_transforms_while_real_joint_moves() {
    let xml = XML.replace(
        "<update_frequency>100</update_frequency>",
        "<update_frequency>0</update_frequency>",
    );
    let mut sim = load(&xml);
    let model = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<PosePublication>())
        .unwrap()
        .id();
    let config = sim.world().get::<PosePublication>(model).unwrap().clone();
    assert_eq!(
        config
            .poses
            .iter()
            .map(|p| p.child_frame_id.as_str())
            .collect::<Vec<_>>(),
        ["rig", "rig::base", "rig::tip"]
    );
    assert_eq!(config.poses[0].frame_id, "default");
    assert!(config.poses[1..].iter().all(|p| p.frame_id == "rig"));
    let joint = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rig::tip_hinge")
        })
        .unwrap();
    let handle = joint.get::<JointHandle>().unwrap().id();
    let before = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_state(handle)
        .unwrap();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_motor(
            handle,
            Some(JointVelocityMotor {
                target_velocity: 0.5,
                max_force: 0.5,
            }),
        )
        .unwrap();
    assert_eq!(
        sim.world().resource::<PhysicsBackend>().joint_state(handle),
        Some(before)
    );
    sim.step_exact(100).unwrap();
    let measured = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_state(handle)
        .unwrap();
    assert!(measured.position.abs() > 0.0 && measured.velocity.abs() > 0.0);
    let sample = sim.world().get::<PoseSample>(model).unwrap();
    assert_eq!(
        (sample.time, sample.epoch, sample.revision),
        (std::time::Duration::from_millis(100), 0, 100)
    );
    for frame in &sample.poses {
        let entity = entity_by_id(sim.world(), frame.identity).unwrap();
        let local = sim.world().get::<LocalPose>(entity).unwrap().0;
        assert_eq!(frame.pose.translation, local.translation);
        assert_eq!(frame.pose.rotation, local.rotation);
        if entity != model {
            assert_ne!(
                frame.pose.translation,
                sim.world().get::<WorldPose>(entity).unwrap().translation
            );
        }
    }
    assert_eq!(sim.world().get::<PosePublication>(model).unwrap(), &config);
    let id = sim.world().get::<EntityId>(model).unwrap().get();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), id).unwrap();
    assert_ne!(restored, model);
    assert_eq!(
        sim.world().get::<PosePublication>(restored).unwrap(),
        &config
    );
    assert!(sim.world().get::<PoseSample>(restored).is_none());
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(
                sim.world()
                    .iter_entities()
                    .find(|e| e
                        .get::<Name>()
                        .is_some_and(|n| n.0 == "default::rig::tip_hinge"))
                    .unwrap()
                    .get::<JointHandle>()
                    .unwrap()
                    .id()
            )
            .unwrap()
            .position,
        0.0
    );
}
#[test]
fn public_pose_default_links_and_model_only_are_selections_not_scene_filters() {
    for (extra, expected) in [
        ("", vec!["rig::base", "rig::tip"]),
        (
            "<publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose>",
            vec!["rig"],
        ),
    ] {
        let xml = XML.replace("<publish_model_pose>true</publish_model_pose>", extra);
        let mut sim = load(&xml);
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<PosePublication>())
            .unwrap()
            .id();
        sim.step_exact(10).unwrap();
        let config = sim.world().get::<PosePublication>(entity).unwrap();
        assert_eq!(
            config
                .poses
                .iter()
                .map(|p| p.child_frame_id.as_str())
                .collect::<Vec<_>>(),
            expected
        );
        assert!(
            sim.world()
                .iter_entities()
                .any(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig::tip"))
        );
        assert_eq!(
            sim.world().get::<PoseSample>(entity).unwrap().poses.len(),
            config.poses.len()
        );
    }
}
