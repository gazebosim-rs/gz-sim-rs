// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, PhysicsError, RapierBackend, RotationSpec, Shape,
};
use std::time::Duration;
const AXIS: [f64; 3] = [0.0, 1.0, 0.0];
fn body(backend: &mut RapierBackend, speed: f64) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                angular_velocity: [0.0, speed, 0.0],
                ..Default::default()
            },
            None,
        )
        .unwrap()
}
fn steps(backend: &mut RapierBackend, count: usize) {
    for _ in 0..count {
        backend.step(Duration::from_millis(1)).unwrap();
    }
}
#[test]
fn moving_parent_preserves_spin_and_continuous_relative_coordinate() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let parent = body(&mut backend, 1.0);
    let child = body(&mut backend, 13.0);
    let joint = backend
        .add_reduced_revolute(Some(parent), child, [0.0; 3], AXIS)
        .unwrap();
    steps(&mut backend, 1000);
    let sample = backend.joint_state(joint).unwrap();
    assert!((sample.position - 12.0).abs() < 1e-8, "{sample:?}");
    assert!((sample.velocity - 12.0).abs() < 1e-8);
    for (id, speed) in [(parent, 1.0), (child, 13.0)] {
        assert!((backend.state(id).unwrap().angular_velocity[1] - speed).abs() < 1e-8);
    }
}
#[test]
fn world_joint_receives_actual_torque_without_hidden_damping() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let child = body(&mut backend, 0.0);
    let joint = backend
        .add_reduced_revolute(None, child, [0.0; 3], AXIS)
        .unwrap();
    // I=2mr^2/5=.016; torque1 for .1s gives omega6.25 and angle.3125.
    for _ in 0..100 {
        backend.add_world_wrench(child, [0.0; 3], AXIS).unwrap();
        steps(&mut backend, 1);
    }
    let sample = backend.joint_state(joint).unwrap();
    assert!((sample.velocity - 6.25).abs() < 1e-8, "{sample:?}");
    assert!((backend.state(child).unwrap().angular_velocity[1] - 6.25).abs() < 1e-8);
    assert!((sample.position - 0.3125).abs() < 1e-3);
    steps(&mut backend, 100);
    assert!((backend.joint_state(joint).unwrap().velocity - 6.25).abs() < 1e-8);
}
#[test]
fn nonidentity_frame_offset_anchor_and_reset_keep_the_public_axis() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let child = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [1.0, 0.0, 0.0],
                velocity: [0.0, 1.0, 0.0],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation: [half, 0.0, 0.0, half],
                angular_velocity: [0.0, 0.0, 1.0],
            },
            None,
        )
        .unwrap();
    let joint = backend
        .add_reduced_revolute(None, child, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    for _ in 0..2 {
        steps(&mut backend, 500);
        let sample = backend.joint_state(joint).unwrap();
        let pose = backend.state(child).unwrap();
        assert!((sample.position - 0.5).abs() < 1e-6, "{sample:?}");
        assert!((sample.velocity - 1.0).abs() < 1e-6);
        assert!((pose.position[0] - 0.5_f64.cos()).abs() < 1e-6);
        assert!((pose.position[1] - 0.5_f64.sin()).abs() < 1e-6);
        backend.reset().unwrap();
    }
}
#[test]
fn invalid_topology_mixed_constraints_and_initial_velocities_are_atomic() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let a = body(&mut backend, 0.0);
    let b = body(&mut backend, 0.0);
    let c = body(&mut backend, 0.0);
    let d = body(&mut backend, 0.0);
    let bc = backend
        .add_reduced_revolute(Some(b), c, [0.0; 3], AXIS)
        .unwrap();
    assert_eq!(bc.raw(), 0);
    let before = backend.state(c);
    assert_eq!(
        backend.add_reduced_revolute(Some(d), c, [0.0; 3], AXIS),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(
        backend.add_reduced_revolute(Some(c), b, [0.0; 3], AXIS),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(backend.state(c), before);
    assert!(matches!(
        backend.add_revolute(a, b, [0.0; 3], AXIS),
        Err(PhysicsError::Unsupported { .. })
    ));
    let ab = backend
        .add_reduced_revolute(Some(a), b, [0.0; 3], AXIS)
        .unwrap();
    assert_eq!(ab.raw(), 1);
    let de = body(&mut backend, 0.0);
    let _impulse = backend.add_revolute(d, de, [0.0; 3], AXIS).unwrap();
    assert!(matches!(
        backend.add_reduced_revolute(Some(a), d, [0.0; 3], AXIS),
        Err(PhysicsError::Unsupported { .. })
    ));
    let incompatible = body(&mut backend, 2.0);
    let before = backend.state(incompatible);
    assert!(matches!(
        backend.add_reduced_revolute(None, incompatible, [0.0; 3], [1.0, 0.0, 0.0]),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(backend.state(incompatible), before);
    assert_eq!(backend.is_jointed(incompatible), Some(false));
}
#[test]
fn direct_velocity_and_impulse_edits_reject_before_articulated_state_changes() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let child = body(&mut backend, 2.0);
    let joint = backend
        .add_reduced_revolute(None, child, [0.0; 3], AXIS)
        .unwrap();
    for _ in 0..2 {
        let before = backend.state(child);
        let sample = backend.joint_state(joint);
        assert!(matches!(
            backend.set_world_velocity(child, [0.0; 3], AXIS),
            Err(PhysicsError::Unsupported { .. })
        ));
        assert!(matches!(
            backend.apply_impulse(child, [0.0; 3], AXIS),
            Err(PhysicsError::Unsupported { .. })
        ));
        assert_eq!(backend.state(child), before);
        assert_eq!(backend.joint_state(joint), sample);
        steps(&mut backend, 1);
    }
}
#[test]
fn subtree_removal_resets_and_prepared_restore_keep_ids_and_survivors() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let a = body(&mut backend, 1.0);
    let b = body(&mut backend, 2.0);
    let c = body(&mut backend, 3.0);
    // Build the child subtree before attaching its root.
    let bc = backend
        .add_reduced_revolute(Some(b), c, [0.0; 3], AXIS)
        .unwrap();
    let ab = backend
        .add_reduced_revolute(Some(a), b, [0.0; 3], AXIS)
        .unwrap();
    let initial = backend.initial_state();
    backend.step(Duration::ZERO).unwrap();
    steps(&mut backend, 100);
    assert!(matches!(
        backend.add_reduced_revolute(None, a, [0.0; 3], AXIS),
        Err(PhysicsError::JointConstructionClosed)
    ));
    let a_before = backend.state(a).unwrap();
    let c_before = backend.state(c).unwrap();
    assert!(backend.remove(b));
    assert!(backend.joint_state(ab).is_none());
    assert!(backend.joint_state(bc).is_none());
    assert_eq!(backend.state(a).unwrap(), a_before);
    assert_eq!(backend.state(c).unwrap(), c_before);
    assert_eq!(backend.is_jointed(a), Some(false));
    assert_eq!(backend.is_jointed(c), Some(false));
    steps(&mut backend, 10);
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    for _ in 0..2 {
        steps(&mut backend, 100);
        for joint in [ab, bc] {
            let sample = backend.joint_state(joint).unwrap();
            assert!((sample.position - 0.1).abs() < 1e-8);
            assert!((sample.velocity - 1.0).abs() < 1e-8);
        }
        backend.reset().unwrap();
    }
    assert!(backend.remove_joint(bc));
    let sample = backend.joint_state(ab).unwrap();
    assert_eq!(sample.position, 0.0);
    let later = backend
        .add_reduced_revolute(Some(b), c, [0.0; 3], AXIS)
        .unwrap();
    assert!(later.raw() > bc.raw());
}

#[test]
fn splitting_a_live_tree_preserves_surviving_handles_and_world_state() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let a = body(&mut backend, 1.0);
    let b = body(&mut backend, 2.0);
    let c = body(&mut backend, 3.0);
    let d = body(&mut backend, 4.0);
    // Splitting changes internal contact filtering; isolate retirement dynamics.
    for id in [a, b, c, d] {
        backend.set_self_collision_group(id, Some(17)).unwrap();
    }
    let ab = backend
        .add_reduced_revolute(Some(a), b, [0.0; 3], AXIS)
        .unwrap();
    let bc = backend
        .add_reduced_revolute(Some(b), c, [0.0; 3], AXIS)
        .unwrap();
    let cd = backend
        .add_reduced_revolute(Some(c), d, [0.0; 3], AXIS)
        .unwrap();
    steps(&mut backend, 500);
    let before = [a, b, c, d].map(|id| backend.state(id).unwrap());
    assert!(backend.remove_joint(bc));
    assert_eq!([a, b, c, d].map(|id| backend.state(id).unwrap()), before);
    assert!(backend.joint_state(bc).is_none());
    steps(&mut backend, 100);
    for (id, speed) in [(a, 1.0), (b, 2.0), (c, 3.0), (d, 4.0)] {
        let state = backend.state(id).unwrap();
        assert!(
            (state.angular_velocity[1] - speed).abs() < 1e-8,
            "{state:?}"
        );
    }
    for joint in [ab, cd] {
        let state = backend.joint_state(joint).unwrap();
        assert!((state.position - 0.6).abs() < 1e-8, "{state:?}");
        assert!((state.velocity - 1.0).abs() < 1e-8);
    }
    assert!(backend.remove(b));
    assert!(backend.joint_state(ab).is_none());
    let sample = backend.joint_state(cd).unwrap();
    steps(&mut backend, 100);
    assert!((backend.joint_state(cd).unwrap().position - sample.position - 0.1).abs() < 1e-8);
    for (id, speed) in [(a, 1.0), (c, 3.0), (d, 4.0)] {
        assert!((backend.state(id).unwrap().angular_velocity[1] - speed).abs() < 1e-8);
    }
    backend.reset().unwrap();
    steps(&mut backend, 100);
    assert!((backend.joint_state(cd).unwrap().position - 0.1).abs() < 1e-8);
}

#[test]
fn nonidentity_free_parent_preserves_world_linear_and_angular_velocity() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let mut spawn = |speed| {
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.0; 3],
                    velocity: [0.1, 0.2, 0.3],
                    mass: Some(1.0),
                },
                RotationSpec {
                    orientation: [0.0, 0.0, half, half],
                    angular_velocity: [0.0, speed, 0.0],
                },
                None,
            )
            .unwrap()
    };
    let parent = spawn(1.0);
    let child = spawn(13.0);
    let joint = backend
        .add_reduced_revolute(Some(parent), child, [0.0; 3], AXIS)
        .unwrap();
    steps(&mut backend, 1000);
    assert!((backend.joint_state(joint).unwrap().position - 12.0).abs() < 1e-8);
    for (id, speed) in [(parent, 1.0), (child, 13.0)] {
        let sample = backend.state(id).unwrap();
        for (actual, expected) in sample.position.into_iter().zip([0.1, 0.2, 0.3]) {
            assert!((actual - expected).abs() < 1e-8, "{sample:?}");
        }
        for (actual, expected) in sample.velocity.into_iter().zip([0.1, 0.2, 0.3]) {
            assert!((actual - expected).abs() < 1e-8);
        }
        assert!((sample.angular_velocity[1] - speed).abs() < 1e-8);
    }
}

#[test]
fn contact_between_separate_reduced_components_remains_physical() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut roots = Vec::new();
    for sign in [-1.0, 1.0] {
        let mut spawn = || {
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.2 },
                        position: [sign * 0.3, 0.0, 0.0],
                        velocity: [-sign, 0.0, 0.0],
                        mass: Some(1.0),
                    },
                    RotationSpec::default(),
                    None,
                )
                .unwrap()
        };
        let parent = spawn();
        let child = spawn();
        backend
            .add_reduced_revolute(Some(parent), child, [sign * 0.3, 0.0, 0.0], AXIS)
            .unwrap();
        roots.push(parent);
    }
    steps(&mut backend, 500);
    let left = backend.state(roots[0]).unwrap();
    let right = backend.state(roots[1]).unwrap();
    assert!(
        right.position[0] - left.position[0] >= 0.399,
        "{left:?} {right:?}"
    );
    assert!(left.velocity[0].abs() < 1e-3, "{left:?}");
    assert!(right.velocity[0].abs() < 1e-3, "{right:?}");
}

#[test]
fn impulse_offset_center_of_mass_reference_preserves_existing_physics() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut spawn = || {
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                },
                RotationSpec {
                    angular_velocity: AXIS,
                    ..Default::default()
                },
                Some(gz_physics_rapier::Inertia {
                    center: [0.1, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [0.016, 0.016, 0.016, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap()
    };
    let parent = spawn();
    let child = spawn();
    backend.add_revolute(parent, child, [0.0; 3], AXIS).unwrap();
    steps(&mut backend, 1000);
    let state = backend.state(parent).unwrap();
    let x = state.position[0] + 0.1 * 1.0_f64.cos();
    let z = state.position[2] - 0.1 * 1.0_f64.sin();
    assert!(
        (x - 0.1).abs() < 1e-6 && z.abs() < 1e-6,
        "{state:?} comx={x} comz={z}"
    );
}

#[test]
fn offset_centers_reject_atomically_on_either_endpoint_without_changing_impulse_support() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let parent = body(&mut backend, 1.0);
    let child = body(&mut backend, 1.0);
    let offset = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                angular_velocity: AXIS,
                ..Default::default()
            },
            Some(gz_physics_rapier::Inertia {
                center: [0.1, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.016, 0.016, 0.016, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let before = [parent, child, offset].map(|id| backend.state(id));
    for (a, b) in [
        (Some(parent), offset),
        (Some(offset), child),
        (None, offset),
    ] {
        assert_eq!(
            backend.add_reduced_revolute(a, b, [0.0; 3], AXIS),
            Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "offset articulation centers of mass"
            })
        );
        assert_eq!([parent, child, offset].map(|id| backend.state(id)), before);
        for id in [parent, child, offset] {
            assert_eq!(backend.is_jointed(id), Some(false));
        }
    }
    let joint = backend
        .add_reduced_revolute(Some(parent), child, [0.0; 3], AXIS)
        .unwrap();
    assert_eq!(joint.raw(), 0);
    assert_eq!(
        backend.add_reduced_revolute(Some(child), offset, [0.0; 3], AXIS),
        Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "offset articulation centers of mass"
        })
    );
    assert_eq!(backend.joint_child(joint), Some(child));
    assert!(backend.remove_joint(joint));
    backend
        .add_revolute(parent, offset, [0.0; 3], AXIS)
        .unwrap();
}

#[test]
fn centered_rotated_inertia_and_signed_zero_keep_actual_torque_response() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let child = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(gz_physics_rapier::Inertia {
                center: [-0.0, 0.0, -0.0],
                orientation: [half, 0.0, 0.0, half],
                tensor: [0.016, 0.032, 0.04, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = backend
        .add_reduced_revolute(None, child, [0.0; 3], AXIS)
        .unwrap();
    // Rotating the principal inertia by pi/2 around X gives world Iyy=.04.
    for _ in 0..100 {
        backend.add_world_wrench(child, [0.0; 3], AXIS).unwrap();
        steps(&mut backend, 1);
    }
    assert!((backend.joint_state(joint).unwrap().velocity - 2.5).abs() < 1e-8);
    assert!((backend.state(child).unwrap().angular_velocity[1] - 2.5).abs() < 1e-8);
    steps(&mut backend, 100);
    assert!((backend.joint_state(joint).unwrap().velocity - 2.5).abs() < 1e-8);
}

#[test]
fn last_articulation_removal_leaves_free_body_velocities_and_edits_intact() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let parent = body(&mut backend, 1.0);
    let child = body(&mut backend, 3.0);
    for id in [parent, child] {
        backend.set_self_collision_group(id, Some(71)).unwrap();
    }
    let joint = backend
        .add_reduced_revolute(Some(parent), child, [0.0; 3], AXIS)
        .unwrap();
    steps(&mut backend, 100);
    let before = [parent, child].map(|id| backend.state(id));
    assert!(backend.remove_joint(joint));
    assert_eq!([parent, child].map(|id| backend.state(id)), before);
    steps(&mut backend, 100);
    for (id, speed) in [(parent, 1.0), (child, 3.0)] {
        assert_eq!(backend.is_jointed(id), Some(false));
        assert!((backend.state(id).unwrap().angular_velocity[1] - speed).abs() < 1e-8);
    }
    backend
        .set_world_velocity(parent, [0.0; 3], [0.0, 2.0, 0.0])
        .unwrap();
    backend
        .apply_impulse(child, [0.0; 3], [0.0, 0.016, 0.0])
        .unwrap();
    steps(&mut backend, 1);
    assert!((backend.state(parent).unwrap().angular_velocity[1] - 2.0).abs() < 1e-8);
    assert!((backend.state(child).unwrap().angular_velocity[1] - 4.0).abs() < 1e-8);
}

#[test]
fn velocity_admission_covers_both_current_and_reset_construction_values() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let child = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                angular_velocity: [1.0, 0.0, 0.0],
                ..Default::default()
            },
            None,
        )
        .unwrap();
    backend
        .set_world_velocity(child, [0.0; 3], [0.0, 2.0, 0.0])
        .unwrap();
    let before = backend.state(child);
    assert!(matches!(
        backend.add_reduced_revolute(None, child, [0.0; 3], AXIS),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(backend.state(child), before);
    assert_eq!(backend.is_jointed(child), Some(false));
    let valid = body(&mut backend, 2.0);
    backend
        .set_world_velocity(valid, [0.0; 3], [0.0, 4.0, 0.0])
        .unwrap();
    let joint = backend
        .add_reduced_revolute(None, valid, [0.0; 3], AXIS)
        .unwrap();
    assert_eq!(joint.raw(), 0);
    steps(&mut backend, 100);
    assert!((backend.joint_state(joint).unwrap().position - 0.4).abs() < 1e-8);
    backend.reset().unwrap();
    steps(&mut backend, 100);
    assert!((backend.joint_state(joint).unwrap().position - 0.2).abs() < 1e-8);
}
