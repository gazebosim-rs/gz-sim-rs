// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, JointId, RotationSpec, Shape};
use gz_physics_multifidelity::{
    ArticulationTier::{Dynamic, Prescribed},
    MultiFidelityBackend,
};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);

fn link(x: f64, speed: f64) -> BodySpec {
    BodySpec {
        position: [x, 0.0, 0.0],
        velocity: [speed, 0.0, 0.0],
        shape: Shape::Box {
            half_extents: [0.24, 0.2, 0.2],
        },
        mass: Some(1.0),
    }
}

fn assembly(world: &mut MultiFidelityBackend, speed: f64) -> (BodyId, BodyId, JointId) {
    let mut spawn = |x, offset| {
        world
            .spawn_articulation_link(
                link(x, speed),
                RotationSpec::default(),
                Some(Inertia {
                    center: [offset, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [0.03, 0.03, 0.03, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap()
    };
    let root = spawn(0.0, 0.07);
    let child = spawn(0.6, -0.04);
    let joint = world
        .add_reduced_revolute(Some(root), child, [0.3, 0.0, 0.0], [0.0, 0.0, 1.0])
        .unwrap();
    (root, child, joint)
}

#[test]
fn prescribed_articulation_drives_dynamic_contact_and_transfers_one_queued_force() {
    let mut world = MultiFidelityBackend::new([0.0; 3]).unwrap();
    let (root, child, joint) = assembly(&mut world, 1.0);
    let payload = world
        .spawn_link(link(1.2, 0.0), Default::default(), None)
        .unwrap();
    world.step(DT).unwrap();
    let before = (
        world.state(root),
        world.state(child),
        world.joint_state(joint),
    );
    assert!(
        world
            .switch_articulation(root, Prescribed, Duration::ZERO)
            .is_err()
    );
    world.switch_articulation(root, Prescribed, DT).unwrap();
    world.switch_articulation(root, Prescribed, DT).unwrap();
    assert_eq!(
        (
            world.state(root),
            world.state(child),
            world.joint_state(joint)
        ),
        before
    );
    assert!(world.switch_articulation(child, Dynamic, DT).is_err());
    assert!(world.add_joint_effort(joint, 1.0).is_err());
    assert_eq!(world.is_kinematic(child), Some(true));
    // An overflowing addition must leave the original pending force intact.
    world
        .add_world_wrench(root, [f64::MAX, 0., 0.], [0.; 3])
        .unwrap();
    assert!(
        world
            .add_world_wrench(root, [f64::MAX, 0., 0.], [0.; 3])
            .is_err()
    );
    world
        .add_world_wrench(root, [-f64::MAX, 0., 0.], [0.; 3])
        .unwrap();
    world
        .add_world_wrench(root, [0.0, 0.0, 20.0], [0.0; 3])
        .unwrap();
    assert_eq!(
        world.pending_world_wrench(root),
        Some(([0., 0., 20.], [0.; 3]))
    );
    for _ in 0..400 {
        world.step(DT).unwrap();
    }
    let prescribed = world.state(root).unwrap();
    assert!(
        (prescribed.position[0] - 0.401).abs() < 1e-9,
        "{prescribed:?}"
    );
    assert_eq!(prescribed.velocity, [1.0, 0.0, 0.0]);
    assert!(world.state(payload).unwrap().position[0] > 1.35);
    assert!(world.state(payload).unwrap().velocity[0] > 0.8);
    assert_eq!(world.joint_child(joint), Some(child));
    assert!((world.state(child).unwrap().position[0] - prescribed.position[0] - 0.6).abs() < 1e-10);
    world.remove(payload);
    world.switch_articulation(root, Dynamic, DT).unwrap();
    for _ in 0..2 {
        world.step(DT).unwrap();
        let vz = world.state(root).unwrap().velocity[2] + world.state(child).unwrap().velocity[2];
        assert!((vz - 0.02).abs() < 1e-9, "{vz}");
    }
}

#[test]
fn freefall_and_pending_wrenches_refuse_demotion_without_changing_source() {
    for gravity in [[0.0; 3], [0.0, 0.0, -9.81]] {
        let mut world = MultiFidelityBackend::new(gravity).unwrap();
        let (root, child, joint) = assembly(&mut world, 0.0);
        if gravity != [0.0; 3] {
            // Zero initial velocities do not establish contact support.
            assert!(matches!(
                world.switch_articulation(root, Prescribed, DT),
                Err(gz_physics::PhysicsError::TransitionDeferred { .. })
            ));
        }
        world.step(DT).unwrap();
        if gravity == [0.0; 3] {
            world
                .add_world_wrench(child, [1.0, 0.0, 0.0], [0.0; 3])
                .unwrap();
        }
        let before = (
            world.state(root),
            world.state(child),
            world.joint_state(joint),
        );
        assert!(world.switch_articulation(root, Prescribed, DT).is_err());
        assert_eq!(world.articulation_tier(root).unwrap(), Dynamic);
        assert_eq!(
            (
                world.state(root),
                world.state(child),
                world.joint_state(joint)
            ),
            before
        );
        world.step(DT).unwrap();
        if gravity == [0.0; 3] {
            assert!(
                (world.state(root).unwrap().velocity[0] + world.state(child).unwrap().velocity[0]
                    - 0.001)
                    .abs()
                    < 1e-10
            );
        }
    }
}

#[test]
fn nonprincipal_spin_is_deferred_without_losing_link_or_joint_state() {
    let mut world = MultiFidelityBackend::new([0.0; 3]).unwrap();
    let omega = [1.0, 0.0, 1.0];
    let mut spawn = |x: f64| {
        world
            .spawn_articulation_link(
                BodySpec {
                    velocity: [0.0, x, 0.0],
                    ..link(x, 0.0)
                },
                RotationSpec {
                    angular_velocity: omega,
                    ..Default::default()
                },
                None,
            )
            .unwrap()
    };
    let root = spawn(0.0);
    let child = spawn(0.6);
    let joint = world
        .add_reduced_revolute(Some(root), child, [0.3, 0.0, 0.0], [0.0, 1.0, 0.0])
        .unwrap();
    let before = (
        world.state(root),
        world.state(child),
        world.joint_state(joint),
    );
    assert_eq!(world.articulation_link_count(root).unwrap(), 2);
    assert!(world.articulation_contains(root, child));
    assert!(!world.articulation_contains(child, root));
    assert!(matches!(
        world.switch_articulation(root, Prescribed, DT),
        Err(gz_physics::PhysicsError::TransitionDeferred {
            reason: "torque-free motion requires a principal-axis spin"
        })
    ));
    assert_eq!(world.articulation_tier(root).unwrap(), Dynamic);
    assert_eq!(
        (
            world.state(root),
            world.state(child),
            world.joint_state(joint)
        ),
        before
    );
}

#[test]
fn prescribed_topology_edits_refuse_and_retirement_releases_surviving_links() {
    let mut world = MultiFidelityBackend::new([0.0; 3]).unwrap();
    let (root, child, joint) = assembly(&mut world, 0.0);
    let other = world
        .spawn_articulation_link(link(5.0, 0.0), Default::default(), None)
        .unwrap();
    let snapshot = world.initial_state();
    world.switch_articulation(root, Prescribed, DT).unwrap();
    assert!(
        world
            .add_reduced_revolute(Some(child), other, [4.0, 0.0, 0.0], [0.0, 0.0, 1.0])
            .is_err()
    );
    assert_eq!(world.articulation_link_count(root).unwrap(), 2);
    assert!(!world.articulation_contains(root, other));
    world
        .add_world_wrench(root, [3.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert!(world.remove_joint(joint));
    assert!(!world.is_kinematic(root).unwrap());
    world.step(DT).unwrap();
    assert!((world.state(root).unwrap().velocity[0] - 0.003).abs() < 1e-11);
    let prepared = world.prepare_initial(&snapshot).unwrap();
    world.apply_initial(prepared).unwrap();
    assert_eq!(world.articulation_tier(root).unwrap(), Dynamic);
    assert_eq!(world.joint_child(joint), Some(child));
    world.switch_articulation(root, Prescribed, DT).unwrap();
    assert!(world.remove(root));
    assert!(world.state(root).is_none());
    assert_eq!(world.is_kinematic(child), Some(false));
    world.step(DT).unwrap();
}

#[test]
fn limited_joint_validation_is_atomic_and_tiny_persistent_motion_refuses_a_hold() {
    use gz_physics::PhysicsError;
    let mut world = MultiFidelityBackend::new([0.; 3]).unwrap();
    let mut spawn = |x, spin| {
        world
            .spawn_articulation_link(
                link(x, 0.0),
                RotationSpec {
                    angular_velocity: [0., 0., spin],
                    ..RotationSpec::default()
                },
                None,
            )
            .unwrap()
    };
    let root = spawn(0.0, 0.0);
    let child = spawn(0.6, 5e-9);
    let axis = [0., 0., 1.];
    let anchor = [0.6, 0., 0.];
    for limits in [[f64::NAN, 1.0], [-1., f64::INFINITY], [-2., -1.], [1., 2.]] {
        assert_eq!(
            world.add_limited_revolute(Some(root), child, anchor, axis, limits),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(world.articulation_link_count(root).ok(), None);
    }
    let hinge = world
        .add_limited_revolute(Some(root), child, anchor, axis, [-0.25, 0.5])
        .unwrap();
    assert_eq!(hinge, JointId::from_raw(0));
    for _ in 0..2 {
        let before = (world.state(child), world.joint_state(hinge));
        assert!(matches!(
            world.switch_articulation(root, Prescribed, DT),
            Err(PhysicsError::TransitionDeferred { .. })
        ));
        assert_eq!(before, (world.state(child), world.joint_state(hinge)));
        assert_eq!(world.articulation_tier(root).unwrap(), Dynamic);
        world.step(DT).unwrap();
    }
    assert!(world.joint_state(hinge).unwrap().position > 0.0);
}

#[test]
fn zero_volume_gates_keep_the_link_identity_across_both_fidelity_switches() {
    let mut world = MultiFidelityBackend::new([0.; 3]).unwrap();
    let (root, child, _) = assembly(&mut world, 0.7);
    for tier in [Dynamic, Prescribed, Dynamic] {
        world.switch_articulation(root, tier, DT).unwrap();
        world.step(DT).unwrap();
        for id in [root, child] {
            let mut at = world.state(id).unwrap().position;
            assert_eq!(world.bodies_overlapping_aabb(at, at).unwrap(), [id]);
            at[1] += 0.201;
            assert!(world.bodies_overlapping_aabb(at, at).unwrap().is_empty());
        }
    }
}
