// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, JointId, PhysicsError, RotationSpec, Shape};
use gz_physics_rapier::RapierBackend;
use std::time::Duration;

fn body(b: &mut RapierBackend, mass: Option<f64>) -> BodyId {
    b.spawn_link(
        BodySpec {
            shape: Shape::Sphere { radius: 0.2 },
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass,
        },
        RotationSpec::default(),
        None,
    )
    .unwrap()
}
fn joint(b: &mut RapierBackend, reduced: bool, parent: Option<BodyId>, child: BodyId) -> JointId {
    if reduced {
        b.add_reduced_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    } else if let Some(parent) = parent {
        b.add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    } else {
        b.add_world_revolute(child, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap()
    }
}
fn tick(b: &mut RapierBackend) {
    b.step(Duration::from_millis(1)).unwrap();
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}
#[test]
fn world_hinge_effort_is_additive_and_consumed_once_after_zero_step() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let child = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, None, child);
        b.add_joint_effort(id, 2.0).unwrap();
        b.add_joint_effort(id, -0.5).unwrap();
        b.step(Duration::ZERO).unwrap();
        close(b.joint_state(id).unwrap().velocity, 0.0);
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.09375);
        close(b.state(child).unwrap().angular_velocity[1], 0.09375);
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.09375);
        b.add_joint_effort(id, -3.0).unwrap();
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, -0.09375);
    }
}
#[test]
fn moving_parent_effort_is_explicitly_unsupported_before_numeric_validation() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let parent = body(&mut b, Some(2.0));
        let child = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, Some(parent), child);
        let before = (b.state(parent), b.state(child), b.joint_state(id));
        for value in [0.0, 1.0, f64::NAN] {
            assert!(matches!(
                b.add_joint_effort(id, value),
                Err(PhysicsError::Unsupported {
                    engine: "rapier",
                    capability: "moving-parent joint effort commands"
                })
            ));
            assert_eq!((b.state(parent), b.state(child), b.joint_state(id)), before);
        }
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.0);
    }
}
#[test]
fn invalid_scalar_and_combined_world_torque_keep_admitted_commands() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let child = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, None, child);
        b.add_joint_effort(id, 2.0).unwrap();
        let before = (b.state(child), b.joint_state(id));
        for value in [f64::NAN, f64::INFINITY, -f64::INFINITY] {
            assert_eq!(
                b.add_joint_effort(id, value),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!((b.state(child), b.joint_state(id)), before);
        }
        assert_eq!(
            b.add_joint_effort(JointId::from_raw(u64::MAX), 0.0),
            Err(PhysicsError::MissingJoint)
        );
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.125);
        b.add_joint_effort(id, f64::MAX).unwrap();
        assert_eq!(
            b.add_joint_effort(id, f64::MAX),
            Err(PhysicsError::InvalidValue)
        );
        b.add_joint_effort(id, -f64::MAX).unwrap();
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.125);
        // Each projected torque is finite; only their combination overflows.
        b.add_joint_effort(id, 1e308).unwrap();
        b.add_world_wrench(child, [0.0; 3], [0.0, 1e308, 0.0])
            .unwrap();
        let before = (b.state(child), b.joint_state(id));
        assert_eq!(
            b.step(Duration::from_millis(1)),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!((b.state(child), b.joint_state(id)), before);
        b.add_joint_effort(id, -1e308).unwrap();
        b.add_world_wrench(child, [0.0; 3], [0.0, -1e308, 0.0])
            .unwrap();
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.125);
    }
}
#[test]
fn reset_and_retirement_discard_effort_but_preparation_does_not() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let parent = body(&mut b, None);
        let child = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, Some(parent), child);
        b.add_joint_effort(id, 1.0).unwrap();
        let initial = b.initial_state();
        b.reset().unwrap();
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.0);
        b.add_joint_effort(id, 1.0).unwrap();
        let prepared = b.prepare_initial(&initial).unwrap();
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.0625);
        b.add_joint_effort(id, 1.0).unwrap();
        b.apply_initial(prepared).unwrap();
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.0);
        b.add_joint_effort(id, 1.0).unwrap();
        assert!(b.remove_joint(id));
        tick(&mut b);
        close(b.state(child).unwrap().angular_velocity[1], 0.0);
        assert_eq!(b.add_joint_effort(id, 1.0), Err(PhysicsError::MissingJoint));
        b.reset().unwrap();
        let id = joint(&mut b, reduced, Some(parent), child);
        b.add_joint_effort(id, 1.0).unwrap();
        assert!(b.remove(parent));
        tick(&mut b);
        assert_eq!(b.joint_state(id), None);
        close(b.state(child).unwrap().angular_velocity[1], 0.0);
    }
}

#[test]
fn rotated_fixed_parent_and_child_keep_the_world_axis_and_static_reaction() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let mut spawn = |mass, orientation| {
            b.spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass,
                },
                RotationSpec {
                    orientation,
                    angular_velocity: [0.0; 3],
                },
                None,
            )
            .unwrap()
        };
        let parent = spawn(None, [0.0, 0.0, h, h]);
        let child = spawn(Some(1.0), [h, 0.0, 0.0, h]);
        let parent_before = b.state(parent);
        let id = joint(&mut b, reduced, Some(parent), child);
        b.add_joint_effort(id, 1.0).unwrap();
        tick(&mut b);
        let state = b.state(child).unwrap();
        close(state.angular_velocity[0], 0.0);
        close(state.angular_velocity[1], 0.0625);
        close(state.angular_velocity[2], 0.0);
        close(b.joint_state(id).unwrap().velocity, 0.0625);
        assert_eq!(b.state(parent), parent_before);
    }
}
#[test]
fn retiring_other_joints_and_bodies_keeps_the_surviving_effort() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let child = body(&mut b, Some(1.0));
        let other = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, None, child);
        let retired = joint(&mut b, reduced, None, other);
        b.add_joint_effort(id, 1.0).unwrap();
        b.add_joint_effort(retired, 5.0).unwrap();
        assert!(b.remove(other));
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.0625);
        let other = body(&mut b, Some(1.0));
        b.add_joint_effort(id, 1.0).unwrap();
        assert!(b.remove(other));
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.125);
    }
}
#[test]
fn overlapping_efforts_refuse_before_any_joint_or_body_changes() {
    let mut b = RapierBackend::new([0.0; 3]).unwrap();
    let child = body(&mut b, Some(1.0));
    let first = joint(&mut b, false, None, child);
    let second = joint(&mut b, false, None, child);
    b.add_joint_effort(first, 1e308).unwrap();
    b.add_joint_effort(second, 1e308).unwrap();
    let before = (b.state(child), b.joint_state(first), b.joint_state(second));
    assert_eq!(
        b.step(Duration::from_millis(1)),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        (b.state(child), b.joint_state(first), b.joint_state(second)),
        before
    );
    b.add_joint_effort(first, -1e308).unwrap();
    b.add_joint_effort(second, -1e308).unwrap();
    let spare = body(&mut b, Some(1.0));
    let spare_joint = joint(&mut b, false, None, spare);
    assert_eq!(spare_joint.raw(), 2);
    assert!(b.remove(spare));
    assert!(b.remove_joint(second));
    b.add_joint_effort(first, 1.0).unwrap();
    tick(&mut b);
    close(b.joint_state(first).unwrap().velocity, 0.0625);
}

#[test]
fn foreign_initial_states_do_not_consume_queued_effort() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let child = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, None, child);
        let other = RapierBackend::new([0.0; 3]).unwrap();
        let foreign = other.initial_state();
        b.add_joint_effort(id, 1.0).unwrap();
        assert!(matches!(
            b.prepare_initial(&foreign),
            Err(PhysicsError::ForeignInitialState)
        ));
        let prepared = other.prepare_initial(&foreign).unwrap();
        assert_eq!(
            b.apply_initial(prepared),
            Err(PhysicsError::ForeignInitialState)
        );
        tick(&mut b);
        close(b.joint_state(id).unwrap().velocity, 0.0625);
    }
}

#[test]
fn native_cpp_force_pulses_match_rates_for_both_solver_representations() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/joint_effort.json")).unwrap();
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let child = body(&mut b, Some(1.0));
        let id = joint(&mut b, reduced, None, child);
        for row in fixture["observations"].as_array().unwrap() {
            b.add_joint_effort(id, row["effort"].as_f64().unwrap())
                .unwrap();
            tick(&mut b);
            close(
                b.joint_state(id).unwrap().velocity,
                row["dq"].as_f64().unwrap(),
            );
        }
    }
}

#[test]
fn tilted_offset_anchor_has_the_work_conjugate_inertia_and_com_twist() {
    use rapier3d_f64::prelude::Vector;
    let position = Vector::new(0.3, 0.2, -0.1);
    let anchor = Vector::new(0.1, -0.2, 0.4);
    let axis = Vector::new(0.3, -0.6, 0.7).normalize();
    let lever = position - anchor;
    let moment = 0.016 + lever.length_squared() - lever.dot(axis).powi(2);
    let rate = 0.001 / moment;
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.0; 3]).unwrap();
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let child = b
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: position.to_array(),
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                },
                RotationSpec {
                    orientation: [0.0, 0.0, h, h],
                    angular_velocity: [0.0; 3],
                },
                None,
            )
            .unwrap();
        let id = if reduced {
            b.add_reduced_revolute(None, child, anchor.to_array(), axis.to_array())
        } else {
            b.add_world_revolute(child, anchor.to_array(), axis.to_array())
        }
        .unwrap();
        b.add_joint_effort(id, 1.0).unwrap();
        tick(&mut b);
        let state = b.state(child).unwrap();
        assert!((b.joint_state(id).unwrap().velocity - rate).abs() < 1e-8);
        assert!((Vector::from(state.angular_velocity) - axis * rate).length() < 1e-8);
        assert!((Vector::from(state.velocity) - (axis * rate).cross(lever)).length() < 1e-8);
    }
}
