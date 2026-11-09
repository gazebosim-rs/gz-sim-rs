// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    JointHandle, LocalPose, Name, PhysicsEngine, PosePublication, PoseSample, StaticPoseSample,
    load_world_with_sdf_step,
};
const XML: &str = include_str!("fixtures/m31_pose_static_publisher.sdf");
#[test]
fn split_samples_retain_canonical_model_and_links_while_measured_noncanonical_joint_moves() {
    let mut sim = load_world_with_sdf_step(
        XML,
        "m31.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let owner = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<PosePublication>())
        .unwrap()
        .id();
    let handle = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::rig::tip_hinge")
        })
        .unwrap()
        .get::<JointHandle>()
        .unwrap()
        .id();
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
    sim.step_exact(80).unwrap();
    let measured = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_state(handle)
        .unwrap();
    assert!(measured.position.abs() > 0.0 && measured.velocity.abs() > 0.0);
    let config = sim.world().get::<PosePublication>(owner).unwrap();
    let dynamic = sim.world().get::<PoseSample>(owner).unwrap();
    let fixed = &sim.world().get::<StaticPoseSample>(owner).unwrap().0;
    assert_eq!((dynamic.revision, fixed.revision), (8, 2));
    assert_eq!(dynamic.time, fixed.time);
    for (frames, sample) in [
        (&config.poses, dynamic),
        (&config.static_publication.as_ref().unwrap().poses, fixed),
    ] {
        for (frame, value) in frames.iter().zip(&sample.poses) {
            assert_eq!(frame.identity, value.identity);
            let local = sim
                .world()
                .get::<LocalPose>(entity_by_id(sim.world(), value.identity).unwrap())
                .unwrap()
                .0;
            assert_eq!(value.pose.translation, local.translation);
            assert_eq!(value.pose.rotation, local.rotation);
        }
    }
}
#[test]
fn jointless_moving_model_refreshes_static_local_pose_without_creating_empty_dynamic_sample() {
    let start = XML.find("    <joint ").unwrap();
    let end = XML.find("    <plugin ").unwrap();
    let xml = format!("{}{}", &XML[..start], &XML[end..]);
    let mut sim = load_world_with_sdf_step(
        &xml,
        "jointless.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let owner = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<PosePublication>())
        .unwrap()
        .id();
    assert!(
        sim.world()
            .get::<PosePublication>(owner)
            .unwrap()
            .poses
            .is_empty()
    );
    let bodies: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<gz_sim_scene::BodyHandle>().map(|h| h.id()))
        .collect();
    for body in bodies {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(body, [1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
    }
    sim.step_exact(40).unwrap();
    let first = sim
        .world()
        .get::<StaticPoseSample>(owner)
        .unwrap()
        .0
        .clone();
    sim.step_exact(40).unwrap();
    let next = &sim.world().get::<StaticPoseSample>(owner).unwrap().0;
    assert_eq!(next.revision, first.revision + 1);
    assert!(next.poses[0].pose.translation[0] > first.poses[0].pose.translation[0]);
    let model_value = next
        .poses
        .iter()
        .find(|p| {
            p.identity
                == sim
                    .world()
                    .get::<gz_sim_core::EntityId>(owner)
                    .unwrap()
                    .get()
        })
        .unwrap();
    assert_eq!(
        model_value.pose.translation,
        sim.world().get::<LocalPose>(owner).unwrap().0.translation
    );
    assert!(sim.world().get::<PoseSample>(owner).is_none());
}
