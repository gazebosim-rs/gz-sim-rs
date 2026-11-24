// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use glam::{DQuat, DVec3};
use gz_physics::{PhysicsBackend, joint_drive::SphericalDrive};
use gz_physics_multifidelity::{BallJointState, MultiFidelityBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    InputError, JointHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_ball_joint_drive,
    queue_ball_joint_effort,
};

fn load() -> Simulation {
    load_world_with_sdf_step(
        include_str!("../../../examples/worlds/local_hanging_ball_wrist.sdf"),
        "wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, part: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == format!("default::wrist::{part}"))
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn pending(sim: &Simulation, id: u64) -> Option<[f64; 3]> {
    let e = gz_sim_core::entity_by_id(sim.world(), id)?;
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()?
        .pending_ball_effort(sim.world().get::<JointHandle>(e)?.id())
}

#[test]
fn a_feedback_torque_uses_the_committed_encoder_and_survives_pause_once() {
    let mut sim = load();
    let joint = id(&sim, "bearing");
    let drive = SphericalDrive::new(4., 0.5, 1.4).unwrap();
    let axis = DVec3::new(1., 2., -2.).normalize();
    let target = BallJointState {
        orientation: DQuat::from_axis_angle(axis, 0.72).to_array(),
        angular_velocity: [0.; 3],
    };
    let t = queue_ball_joint_drive(&mut sim, joint, &drive, target, [0.; 3]).unwrap();
    assert!(DVec3::from(t).distance(1.4 * axis) < 1e-14);
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(pending(&sim, joint), Some(t));
    sim.step_exact(1).unwrap();
    assert_eq!(pending(&sim, joint), None);
    let desired = BallJointState {
        orientation: [0., 0., 0., 1.],
        angular_velocity: [0.4, -0.2, 0.1],
    };
    let t = queue_ball_joint_drive(&mut sim, joint, &drive, desired, [0.2, 0.1, -0.3]).unwrap();
    assert_ne!(t, [0.4, 0., -0.25]);
}

#[test]
fn rejected_commands_leave_any_prior_load_intact() {
    let mut sim = load();
    let joint = id(&sim, "bearing");
    let tool = id(&sim, "tool");
    let target = BallJointState {
        orientation: [0., 0., 0., 1.],
        angular_velocity: [0.; 3],
    };
    let drive = SphericalDrive::new(0., 0., f64::MAX).unwrap();
    for (id, reference, feed) in [
        (tool, target, [0.; 3]),
        (u64::MAX, target, [0.; 3]),
        (
            joint,
            BallJointState {
                orientation: [0.; 4],
                ..target
            },
            [0.; 3],
        ),
        (joint, target, [f64::NAN; 3]),
    ] {
        assert!(queue_ball_joint_drive(&mut sim, id, &drive, reference, feed).is_err());
        assert_eq!(pending(&sim, joint), None);
    }
    let mount = id(&sim, "mount");
    assert!(matches!(
        queue_ball_joint_drive(&mut sim, mount, &drive, target, [0.; 3]),
        Err(InputError::Unsupported(_))
    ));
    queue_ball_joint_effort(&mut sim, joint, [f64::MAX, 0., 0.]).unwrap();
    assert!(matches!(
        queue_ball_joint_drive(&mut sim, joint, &drive, target, [f64::MAX, 0., 0.]),
        Err(InputError::InvalidValue)
    ));
    assert_eq!(pending(&sim, joint), Some([f64::MAX, 0., 0.]));
    sim.world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("injected failure"));
    assert!(matches!(
        queue_ball_joint_drive(&mut sim, joint, &drive, target, [0.; 3]),
        Err(InputError::Clock(_))
    ));
}
