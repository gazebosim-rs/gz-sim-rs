// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real native candidate ownership and final guard controls, not allocator injection.
use super::*;
type PhysicalSnapshot = ([f64; 3], [f64; 4], Twist, f64, f64);
fn sphere(position: [f64; 3]) -> BodyConfig {
    crate::body::config(
        gz_physics::BodySpec {
            shape: gz_physics::Shape::Sphere { radius: 0.2 },
            position,
            velocity: [0.; 3],
            mass: Some(1.),
        },
        gz_physics::RotationSpec::default(),
        Some(gz_physics::Inertia {
            center: [0.; 3],
            orientation: [0., 0., 0., 1.],
            tensor: [1., 1., 1., 0., 0., 0.],
        }),
    )
    .unwrap()
}
fn edges() -> [Twist; 2] {
    [
        Twist {
            linear: [1., 0., 0.],
            angular: [0., 0., 1.],
        },
        Twist {
            linear: [3., 0., 0.],
            angular: [0., 0., 1.],
        },
    ]
}
fn snapshots(world: &mut NativeWorld, slots: [usize; 3]) -> Vec<PhysicalSnapshot> {
    slots
        .into_iter()
        .map(|slot| {
            let r = world.read(slot).unwrap();
            (
                r.position,
                r.orientation,
                r.velocity,
                r.joint_position,
                r.joint_velocity,
            )
        })
        .collect()
}
#[test]
fn second_edge_native_refusal_drops_partial_candidate_and_preserves_original_future_work() {
    let mut world = NativeWorld::new([0.; 3]).unwrap();
    let mut control = NativeWorld::new([0.; 3]).unwrap();
    let slots = std::array::from_fn(|i| world.add_body(&sphere([i as f64 * 2., 0., 0.])).unwrap());
    let controls =
        std::array::from_fn(|i| control.add_body(&sphere([i as f64 * 2., 0., 0.])).unwrap());
    let before = snapshots(&mut world, slots);
    let mut bad = edges();
    bad[1].linear[0] = f64::NAN;
    assert!(
        world
            .prepare_serial_revolutes(slots, bad)
            .unwrap()
            .is_none()
    );
    assert_eq!(snapshots(&mut world, slots), before);
    assert_eq!(world.skeleton_count().unwrap(), 3);
    assert!(world.contacts_conditioned().unwrap());
    let command = |slot| Wrench {
        slot,
        force: [1., 2., 3.],
        torque: [0.2, -0.1, 0.3],
    };
    assert_eq!(world.step(0.001, &[command(slots[0])]).unwrap(), 0);
    assert_eq!(control.step(0.001, &[command(controls[0])]).unwrap(), 0);
    assert_eq!(
        snapshots(&mut world, slots),
        snapshots(&mut control, controls)
    );
}
#[test]
fn candidate_outlives_original_and_keeps_slot_holes_and_static_shape_owners() {
    let mut world = NativeWorld::new([0.; 3]).unwrap();
    let slots = std::array::from_fn(|i| world.add_body(&sphere([i as f64 * 2., 0., 0.])).unwrap());
    let hole = world.add_body(&sphere([20., 0., 0.])).unwrap();
    world.remove_body(hole).unwrap();
    let mut fixed = sphere([30., 0., 0.]);
    fixed.mobile = false;
    let static_slot = world.add_body(&fixed).unwrap();
    let before = world.read(static_slot).unwrap();
    let free_contact = world.add_body(&sphere([30.3, 0., 0.])).unwrap();
    assert!(world.contacts_conditioned().unwrap());
    let mut prepared = world
        .prepare_serial_revolutes(slots, edges())
        .unwrap()
        .unwrap();
    assert_eq!(world.skeleton_count().unwrap(), 5);
    assert_eq!(prepared.skeleton_count().unwrap(), 3);
    let fixed_after = prepared.read(static_slot).unwrap();
    assert_eq!(fixed_after.position, before.position);
    assert_eq!(fixed_after.orientation, before.orientation);
    assert_eq!(fixed_after.velocity, before.velocity);
    assert!(prepared.read(hole).is_err());
    drop(world);
    assert_eq!(prepared.step(0.001, &[]).unwrap(), 0);
    assert!(prepared.contacts_conditioned().unwrap());
    for slot in slots.into_iter().rev() {
        prepared.remove_body(slot).unwrap();
    }
    prepared.remove_body(static_slot).unwrap();
    prepared.remove_body(free_contact).unwrap();
    assert_eq!(prepared.skeleton_count().unwrap(), 0);
}
#[test]
fn final_grid_and_generalized_velocity_guards_reject_raw_candidates_without_touching_source() {
    // Deliberately raw private FFI records, never claimed as admitted public
    // BodySpecs. No collision is requested on the off-grid original world.
    for grid in [false, true] {
        let mut world = NativeWorld::new([0.; 3]).unwrap();
        let slots = std::array::from_fn(|i| {
            let mut config = sphere([i as f64 * 2., 0., 0.]);
            if grid {
                config.position[0] += 1e9;
            }
            if !grid {
                config.velocity.linear[0] = 2e60;
            }
            world.add_body(&config).unwrap()
        });
        let before = snapshots(&mut world, slots);
        let mut e = edges();
        if grid {
            e[0].linear[0] += 1e9;
            e[1].linear[0] += 1e9;
        }
        assert!(world.prepare_serial_revolutes(slots, e).unwrap().is_none());
        assert_eq!(snapshots(&mut world, slots), before);
        assert_eq!(world.skeleton_count().unwrap(), 3);
    }
}

#[test]
fn child_frame_candidate_second_failure_is_atomic_and_success_outlives_original() {
    use std::time::Duration;
    let mut original = NativeWorld::new([0.; 3]).unwrap();
    let slots = [0., 2., 4.].map(|x| original.add_body(&sphere([x, 0., 0.])).unwrap());
    let before = snapshots(&mut original, slots);
    let mut local = [Twist {
        linear: [-1., 0., 0.],
        angular: [0., 0., 1.],
    }; 2];
    local[1].linear[0] = f64::NAN;
    assert!(
        original
            .prepare_serial_revolutes_in_child_frames(slots, local)
            .unwrap()
            .is_none()
    );
    assert_eq!(snapshots(&mut original, slots), before);
    local[1].linear[0] = -1.;
    let (mut candidate, resolved) = original
        .prepare_serial_revolutes_in_child_frames(slots, local)
        .unwrap()
        .unwrap();
    assert_eq!(resolved[0], edges()[0]);
    assert_eq!(resolved[1], edges()[1]);
    assert_eq!(snapshots(&mut original, slots), before);
    drop(original);
    assert_eq!(
        candidate
            .step(Duration::from_millis(1).as_secs_f64(), &[])
            .unwrap(),
        0
    );
    for (actual, expected) in snapshots(&mut candidate, slots).into_iter().zip(before) {
        assert_eq!(actual, expected);
    }
    for slot in slots {
        candidate.remove_body(slot).unwrap();
    }
    assert_eq!(candidate.skeleton_count().unwrap(), 0);
}

#[test]
fn child_frame_joint_id_exhaustion_preserves_real_pending_work_and_all_state() {
    use crate::DartBackend;
    use gz_physics::{BodyId, DynamicBackend};
    use std::time::Duration;
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let ids: [BodyId; 3] = std::array::from_fn(|i| {
        backend
            .spawn_link(
                gz_physics::BodySpec {
                    shape: gz_physics::Shape::Sphere { radius: 0.2 },
                    position: [i as f64 * 2., 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(1.),
                },
                gz_physics::RotationSpec::default(),
                Some(gz_physics::Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [1., 1., 1., 0., 0., 0.],
                }),
            )
            .unwrap()
    });
    backend
        .add_world_wrench(ids[0], [1., 2., 3.], [0.2, -0.1, 0.3])
        .unwrap();
    let pending = backend.pending.clone();
    let before = ids.map(|id| backend.state(id));
    backend.next_joint = u64::MAX - 1;
    assert_eq!(
        backend.add_serial_revolutes_in_child_frames(ids, [[-1., 0., 0.]; 2], [[0., 0., 1.]; 2]),
        Err(PhysicsError::JointIdOverflow)
    );
    assert_eq!(backend.next_joint, u64::MAX - 1);
    assert_eq!(backend.pending, pending);
    assert_eq!(ids.map(|id| backend.state(id)), before);
    assert!(backend.world.joints.is_empty() && backend.world.serial_assemblies.is_empty());
    backend.step(Duration::from_millis(1)).unwrap();
    assert!(backend.pending.is_empty());
    assert!(
        backend
            .state(ids[0])
            .unwrap()
            .velocity
            .iter()
            .any(|v| *v != 0.)
    );
}
