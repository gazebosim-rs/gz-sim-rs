// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{JointVelocityMotor, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    InputError, JointHandle, JointSample, JointStatePublication, JointStateSample, Name,
    PhysicsEngine, load_world_with_sdf_step, queue_joint_effort,
};
const XML: &str = include_str!("fixtures/m18_joint_state.sdf");
fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "m18-public.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
}

#[test]
fn public_filtered_values_follow_real_joint_without_changing_physics_or_joint_order() {
    let filtered=XML.replace("<update_rate>100</update_rate>",
        "<joint_name>tip_hinge</joint_name><joint_name>tip_hinge</joint_name><update_rate>0</update_rate>");
    let mut sim = load(&filtered).expect("one filtered system");
    let model = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::arm"))
        .expect("model")
        .id();
    let joint = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::arm::tip_hinge")
        })
        .expect("joint");
    let entity = joint.id();
    let identity = joint.get::<EntityId>().unwrap().get();
    let config = sim.world().get::<JointStatePublication>(model).unwrap();
    assert_eq!(config.joints.len(), 1);
    assert_eq!(config.joints[0].id, identity);
    assert_eq!(config.joints[0].name, "tip_hinge");
    let original_config = config.clone();
    let solver_joint = sim.world().get::<JointHandle>(entity).unwrap().id();
    let before = (
        sim.elapsed(),
        sim.epoch(),
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(solver_joint)
            .unwrap(),
    );
    // Generalized effort on a moving-parent Rapier joint stays unsupported.
    assert!(matches!(
        queue_joint_effort(&mut sim, identity, 0.5),
        Err(InputError::Physics(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "moving-parent joint effort commands",
        }))
    ));
    assert_eq!(
        (
            sim.elapsed(),
            sim.epoch(),
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(solver_joint)
                .unwrap()
        ),
        before
    );
    assert!(sim.world().get::<JointStateSample>(model).is_none());
    {
        // Use the already-admitted bounded velocity motor on this exact joint;
        // configuring it changes neither measured state nor the simulation clock.
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        backend
            .set_joint_velocity_motor(
                solver_joint,
                Some(JointVelocityMotor {
                    target_velocity: 0.5,
                    max_force: 0.5,
                }),
            )
            .expect("existing moving-parent velocity motor");
        assert_eq!(backend.joint_state(solver_joint), Some(before.2));
    }
    assert_eq!((sim.elapsed(), sim.epoch()), (before.0, before.1));
    assert_eq!(
        sim.world().get::<JointStatePublication>(model),
        Some(&original_config)
    );
    assert!(sim.world().get::<JointStateSample>(model).is_none());
    sim.step_exact(1).unwrap();
    let actual = sim.world().get::<JointSample>(entity).unwrap();
    let message = sim.world().get::<JointStateSample>(model).unwrap();
    assert_eq!(message.joints.len(), 1);
    assert_eq!(
        (message.joints[0].position, message.joints[0].velocity),
        (actual.state.position, actual.state.velocity)
    );
    assert!(message.joints[0].velocity.abs() > 0.0);
    assert_eq!(message.joints[0].id, identity);
}

#[test]
fn unsupported_configuration_does_not_silently_degrade_to_a_publisher() {
    for change in [
        "<joint_name>missing</joint_name>",
        "<force>true</force>",
        "<update_rate>NaN</update_rate>",
        "<topic>/bad topic</topic>",
    ] {
        let xml = XML.replace(
            "<topic>/m18/joint_state</topic><update_rate>100</update_rate>",
            change,
        );
        assert!(load(&xml).is_err(), "{change}");
    }
    let multi = XML.replace(
        "</axis></joint>",
        "</axis><axis2><xyz>0 1 0</xyz></axis2></joint>",
    );
    assert!(load(&multi).is_err());
}

#[test]
fn the_local_transfer_robot_publishes_its_sliders_and_spindle_and_repeats_after_reset() {
    use gz_sim_scene::{JointForceCommand, JointStateKind};
    use std::time::Duration;
    const TRANSFER: &str = include_str!("../../../examples/worlds/local_robot_serial_transfer.sdf");
    let make = |xml: &str| {
        load_world_with_sdf_step(
            xml,
            "transfer.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    let filtered = TRANSFER.replace("        <joint_name>wrist</joint_name>\n", "");
    let mut single_axes = make(&filtered).unwrap();
    single_axes.step_exact(10).unwrap();
    assert!(
        single_axes
            .world()
            .iter_entities()
            .find_map(|n| n.get::<JointStateSample>())
            .unwrap()
            .second_axes
            .is_empty()
    );
    let mut sim = make(TRANSFER).unwrap();
    let model = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::gripper"))
        .unwrap()
        .id();
    let config = sim
        .world()
        .get::<JointStatePublication>(model)
        .unwrap()
        .clone();
    assert_eq!(config.topic, "/world/default/model/gripper/joint_state");
    assert_eq!(
        config
            .joints
            .iter()
            .map(|j| j.name.as_str())
            .collect::<Vec<_>>(),
        ["travel", "spindle", "wrist", "close_left", "close_right"]
    );
    assert_eq!(
        config.joints.iter().map(|j| j.kind).collect::<Vec<_>>(),
        [
            JointStateKind::Prismatic,
            JointStateKind::Screw,
            JointStateKind::Universal,
            JointStateKind::Prismatic,
            JointStateKind::Prismatic
        ]
    );
    let gap = |sim: &Simulation| {
        let z = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<gz_sim_scene::WorldPose>()
                .unwrap()
                .translation[2]
        };
        z("default::gripper::nut") - z("default::gripper::carriage")
    };
    let initial_gap = gap(&sim);
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    let mut passes = Vec::new();
    for pass in 0..2 {
        if pass == 1 {
            gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        let model = gz_sim_core::entity_by_id(sim.world(), model_id).unwrap();
        assert!(sim.world().get::<JointStateSample>(model).is_none());
        JointForceCommand::set_batch(&mut sim, [(config.joints[0].id, 2.)]).unwrap();
        sim.step_exact(9).unwrap();
        assert!(sim.world().get::<JointStateSample>(model).is_none());
        let mut readings = Vec::new();
        for tick in [10, 20, 30] {
            sim.step_exact(if tick == 10 { 1 } else { 10 }).unwrap();
            let sample = sim.world().get::<JointStateSample>(model).unwrap();
            assert_eq!(sample.time, Duration::from_millis(tick));
            assert_eq!(sample.epoch, sim.epoch());
            assert_eq!(sample.joints.len(), 5);
            assert_eq!(sample.second_axes.len(), 1);
            for (reading, joint) in sample.joints.iter().zip(&config.joints) {
                assert_eq!(reading.id, joint.id);
                let entity = gz_sim_core::entity_by_id(sim.world(), joint.id).unwrap();
                if joint.kind.has_second_axis() {
                    let wrist = sim
                        .world()
                        .get::<gz_sim_scene::UniversalJointSample>(entity)
                        .unwrap();
                    let second = sample.second_axes[0];
                    assert_eq!(second.id, reading.id);
                    assert_eq!([reading.position, second.position], wrist.state.position);
                    assert_eq!([reading.velocity, second.velocity], wrist.state.velocity);
                    continue;
                }
                let native = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .joint_state(sim.world().get::<JointHandle>(entity).unwrap().id())
                    .unwrap();
                assert_eq!(
                    (reading.position, reading.velocity),
                    (native.position, native.velocity)
                );
            }
            let screw_travel = 0.50 / std::f64::consts::TAU * sample.joints[1].position;
            assert!((gap(&sim) - initial_gap - screw_travel).abs() < 2e-5);
            readings.push((sample.joints.clone(), sample.second_axes.clone()));
        }
        assert!(readings[2].0[0].position > 1e-5);
        assert!(readings[2].0[1].position > 0.001);
        assert!(readings[2].1[0].velocity.abs() > 0.001);
        passes.push(readings);
    }
    assert_eq!(passes[0], passes[1]);
}
