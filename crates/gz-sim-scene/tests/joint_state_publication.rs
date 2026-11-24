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
fn the_local_transfer_robot_publishes_only_its_three_sliders_and_repeats_after_reset() {
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
    // An unrequested universal wrist is valid in this robot; a scalar
    // publication cannot silently substitute one of its axes if it is requested.
    let unsupported = TRANSFER.replace(
        "<joint_name>travel</joint_name>",
        "<joint_name>wrist</joint_name>",
    );
    assert!(matches!(
        make(&unsupported),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "joint-state publisher requires supported single-axis joints",
            ..
        })
    ));
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
        ["travel", "close_left", "close_right"]
    );
    assert!(
        config
            .joints
            .iter()
            .all(|j| j.kind == JointStateKind::Prismatic)
    );
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
            assert_eq!(sample.joints.len(), 3);
            for (reading, joint) in sample.joints.iter().zip(&config.joints) {
                assert_eq!(reading.id, joint.id);
                let entity = gz_sim_core::entity_by_id(sim.world(), joint.id).unwrap();
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
            readings.push(sample.joints.clone());
        }
        assert!(readings[2][0].position > 1e-5);
        passes.push(readings);
    }
    assert_eq!(passes[0], passes[1]);
}
