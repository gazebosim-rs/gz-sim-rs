// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded two-edge cold construction; no B03-cause or performance claim.
use gz_physics::{
    BodyId, BodySpec, BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia, JointId,
    PhysicsError, RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use gz_physics_rapier::RapierBackend;
use std::time::Duration;
const DT: Duration = Duration::from_millis(1);
const AXES: [[f64; 3]; 2] = [[0., 0., 1.]; 2];
const ANCHORS: [[f64; 3]; 2] = [[0.; 3]; 2];
fn body(backend: &mut DartBackend, position: [f64; 3], fixed: bool) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position,
                velocity: [0.; 3],
                mass: (!fixed).then_some(1.),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [1., 1., 1., 0., 0., 0.],
            }),
        )
        .unwrap()
}
fn three(backend: &mut DartBackend, overlap: bool) -> [BodyId; 3] {
    std::array::from_fn(|i| {
        body(
            backend,
            [if overlap { 0. } else { i as f64 * 2. }, 0., 0.],
            false,
        )
    })
}
fn near(a: BodyState, b: BodyState) {
    for (x, y) in a
        .position
        .into_iter()
        .chain(a.orientation)
        .chain(a.velocity)
        .chain(a.angular_velocity)
        .zip(
            b.position
                .into_iter()
                .chain(b.orientation)
                .chain(b.velocity)
                .chain(b.angular_velocity),
        )
    {
        assert!(
            x.is_finite() && y.is_finite() && (x - y).abs() <= 1e-12,
            "{a:?} != {b:?}"
        );
    }
}
fn attach(backend: &mut DartBackend, bodies: [BodyId; 3]) -> [JointId; 2] {
    backend.add_serial_revolutes(bodies, ANCHORS, AXES).unwrap()
}
#[test]
fn overlapping_intended_children_commit_together_without_changing_the_old_single_api() {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let bodies = three(&mut backend, true);
    let before = bodies.map(|id| backend.state(id));
    assert_eq!(
        backend.add_revolute(bodies[0], bodies[1], ANCHORS[0], AXES[0]),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(bodies.map(|id| backend.state(id)), before);
    let joints = attach(&mut backend, bodies);
    assert_eq!(joints.map(JointId::raw), [0, 1]);
    assert_eq!(bodies.map(|id| backend.state(id)), before);
    assert_eq!(
        joints.map(|id| backend.joint_child(id)),
        [Some(bodies[1]), Some(bodies[2])]
    );
    assert_eq!(
        joints.map(|id| backend.joint_state(id).unwrap().position),
        [0.; 2]
    );
    backend.step(Duration::ZERO).unwrap();
    assert_eq!(bodies.map(|id| backend.state(id)), before);
    let captured = backend.initial_state();
    backend.step(DT).unwrap();
    assert_eq!(
        backend.add_serial_revolutes(bodies, ANCHORS, AXES),
        Err(PhysicsError::JointConstructionClosed)
    );
    backend.reset().unwrap();
    assert_eq!(bodies.map(|id| backend.state(id)), before);
    let prepared = backend.prepare_initial(&captured).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(bodies.map(|id| backend.state(id)), before);
    assert_eq!(
        joints.map(|id| backend.joint_child(id)),
        [Some(bodies[1]), Some(bodies[2])]
    );
}
#[test]
fn final_external_dynamic_contact_refuses_both_edges_and_preserves_pending_wrench_and_ids() {
    let mut candidate = DartBackend::new([0.; 3]).unwrap();
    let mut clean = DartBackend::new([0.; 3]).unwrap();
    let ids = three(&mut candidate, true);
    let controls = three(&mut clean, true);
    let outsider = body(&mut candidate, [0.; 3], false);
    let other = body(&mut clean, [0.; 3], false);
    candidate
        .add_world_wrench(ids[0], [1., 2., 3.], [0.2, -0.1, 0.3])
        .unwrap();
    let before = [ids[0], ids[1], ids[2], outsider].map(|id| candidate.state(id));
    assert_eq!(
        candidate.add_serial_revolutes(ids, ANCHORS, AXES),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(
        [ids[0], ids[1], ids[2], outsider].map(|id| candidate.state(id)),
        before
    );
    assert!(
        ids.iter()
            .all(|id| candidate.is_jointed(*id) == Some(false))
    );
    assert!(candidate.remove(outsider));
    assert!(clean.remove(other));
    let joints = attach(&mut candidate, ids);
    let expected = attach(&mut clean, controls);
    clean
        .add_world_wrench(controls[0], [1., 2., 3.], [0.2, -0.1, 0.3])
        .unwrap();
    assert_eq!(joints.map(JointId::raw), [0, 1]);
    for _ in 0..3 {
        candidate.step(DT).unwrap();
        clean.step(DT).unwrap();
        for (a, b) in ids.into_iter().zip(controls) {
            near(candidate.state(a).unwrap(), clean.state(b).unwrap());
        }
        for (a, b) in joints.into_iter().zip(expected) {
            assert_eq!(candidate.joint_state(a), clean.joint_state(b));
        }
    }
    let next = body(&mut candidate, [20., 0., 0.], false);
    assert_eq!(next.raw(), outsider.raw() + 1);
}
#[test]
fn invalid_final_edge_and_nonisolated_or_missing_endpoints_are_atomic() {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let ids = three(&mut backend, false);
    let before = ids.map(|id| backend.state(id));
    for (anchors, axes) in [
        (ANCHORS, [[0., 0., 1.], [0.; 3]]),
        ([[0.; 3], [f64::NAN, 0., 0.]], AXES),
        (ANCHORS, [[0., 0., 1.], [0., f64::INFINITY, 0.]]),
        ([[0.; 3], [1e9, 0., 0.]], AXES),
    ] {
        assert_eq!(
            backend.add_serial_revolutes(ids, anchors, axes),
            Err(PhysicsError::InvalidJoint)
        );
        assert_eq!(ids.map(|id| backend.state(id)), before);
    }
    assert_eq!(
        backend.add_serial_revolutes([ids[0], ids[1], BodyId::from_raw(999)], ANCHORS, AXES),
        Err(PhysicsError::MissingBody)
    );
    assert!(matches!(
        backend.add_serial_revolutes([ids[0], ids[1], ids[1]], ANCHORS, AXES),
        Err(PhysicsError::Unsupported { .. })
    ));
    let fixed = body(&mut backend, [20., 0., 0.], true);
    assert!(matches!(
        backend.add_serial_revolutes([ids[0], ids[1], fixed], ANCHORS, AXES),
        Err(PhysicsError::Unsupported { .. })
    ));
    let first = backend
        .add_revolute(ids[0], ids[1], ANCHORS[0], AXES[0])
        .unwrap();
    let joint_before = backend.joint_state(first);
    assert!(matches!(
        backend.add_serial_revolutes(ids, ANCHORS, AXES),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(backend.joint_state(first), joint_before);
    assert_eq!(ids.map(|id| backend.state(id)), before);
}
#[test]
fn unrelated_warm_articulation_velocity_effort_and_pending_world_wrench_survive_clone_commit() {
    let mut candidate = DartBackend::new([0.; 3]).unwrap();
    let mut clean = DartBackend::new([0.; 3]).unwrap();
    let make = |b: &mut DartBackend| {
        let root = body(b, [10., 0., 0.], false);
        let child = body(b, [12., 0., 0.], false);
        let joint = b
            .add_revolute(root, child, [11., 0., 0.], [0., 1., 0.])
            .unwrap();
        b.set_world_velocity(child, [0., 0., 0.2], [0., 0.3, 0.])
            .unwrap();
        b.add_joint_effort(joint, 0.7).unwrap();
        b.add_world_wrench(root, [0.2, -0.1, 0.3], [0., 0.4, 0.])
            .unwrap();
        ([root, child], joint)
    };
    let (old, joint) = make(&mut candidate);
    let (references, control_joint) = make(&mut clean);
    let before = old.map(|id| candidate.state(id));
    let rate = candidate.joint_state(joint);
    let fresh = three(&mut candidate, true);
    let controls = three(&mut clean, true);
    attach(&mut candidate, fresh);
    assert_eq!(old.map(|id| candidate.state(id)), before);
    assert_eq!(candidate.joint_state(joint), rate);
    // Retire all newly assembled bodies; the cloned unrelated component must
    // remain live and apply the actual pending commands to its original IDs.
    for id in fresh.into_iter().rev() {
        assert!(candidate.remove(id));
    }
    for id in controls.into_iter().rev() {
        assert!(clean.remove(id));
    }
    for _ in 0..3 {
        candidate.step(DT).unwrap();
        clean.step(DT).unwrap();
        for (a, b) in old.into_iter().zip(references) {
            near(candidate.state(a).unwrap(), clean.state(b).unwrap());
        }
        assert_eq!(
            candidate.joint_state(joint),
            clean.joint_state(control_joint)
        );
    }
}
#[test]
fn compound_shapes_offset_com_reset_removal_and_prepared_initial_state_keep_batch_identity() {
    for removed in 0..3 {
        let mut backend = DartBackend::new([0.; 3]).unwrap();
        let shapes = [-0.3, 0., 0.3].map(|x| CollisionSpec {
            shape: Shape::Sphere { radius: 0.2 },
            position: [x, 0., 0.],
            orientation: [0., 0., 0., 1.],
        });
        let ids = std::array::from_fn(|i| {
            backend
                .spawn_compound_link(
                    CompoundBodySpec {
                        collisions: &shapes,
                        position: [i as f64 * 3., 0., 0.],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec::default(),
                    Inertia {
                        center: [0., 0., 0.1],
                        orientation: [0., 0., 0., 1.],
                        tensor: [1., 1., 1., 0., 0., 0.],
                    },
                )
                .unwrap()
        });
        let joints = attach(&mut backend, ids);
        let initial = backend.initial_state();
        let before = ids.map(|id| backend.state(id));
        backend
            .add_world_wrench(ids[0], [0.2, 0., 0.], [0., 0., 0.1])
            .unwrap();
        backend.add_joint_effort(joints[1], 0.7).unwrap();
        backend.step(DT).unwrap();
        backend
            .add_world_wrench(ids[0], [0.2, 0., 0.], [0., 0., 0.1])
            .unwrap();
        backend.add_joint_effort(joints[1], 0.7).unwrap();
        backend.reset().unwrap();
        assert_eq!(ids.map(|id| backend.state(id)), before);
        assert_eq!(
            joints.map(|id| backend.joint_child(id)),
            [Some(ids[1]), Some(ids[2])]
        );
        backend.step(DT).unwrap();
        assert_eq!(
            ids.map(|id| backend.state(id)),
            before,
            "reset must discard actual pending wrench/effort"
        );
        assert!(backend.remove(ids[removed]));
        assert_eq!(backend.state(ids[removed]), None);
        let survivor = ids[if removed == 0 { 1 } else { 0 }];
        backend
            .add_world_wrench(survivor, [0.2, 0., 0.], [0., 0., 0.1])
            .unwrap();
        for joint in joints {
            if backend.joint_state(joint).is_some() {
                backend.add_joint_effort(joint, 0.7).unwrap();
            }
        }
        let prepared = backend.prepare_initial(&initial).unwrap();
        backend.apply_initial(prepared).unwrap();
        assert_eq!(ids.map(|id| backend.state(id)), before);
        assert_eq!(
            joints.map(|id| backend.joint_child(id)),
            [Some(ids[1]), Some(ids[2])]
        );
        backend.step(DT).unwrap();
        assert_eq!(
            ids.map(|id| backend.state(id)),
            before,
            "prepared reset must discard actual pending commands"
        );
    }
}
#[test]
fn a_partially_retired_separated_batch_resets_its_surviving_single_edge() {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let ids = three(&mut backend, false);
    let joints = attach(&mut backend, ids);
    assert!(backend.remove_joint(joints[1]));
    assert_eq!(backend.joint_state(joints[1]), None);
    let before = ids.map(|id| backend.state(id));
    backend.reset().unwrap();
    assert_eq!(ids.map(|id| backend.state(id)), before);
    assert_eq!(backend.joint_child(joints[0]), Some(ids[1]));
    assert_eq!(backend.joint_state(joints[1]), None);
    backend.step(DT).unwrap();
}
#[test]
fn static_contact_conditioning_remains_a_final_atomic_refusal() {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let ids = std::array::from_fn(|i| {
        let moment = 1e-4 * if i == 2 { 74282549781.30127 } else { 1. };
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: if i == 0 {
                            [0.25, 0.25, 0.005]
                        } else {
                            [0.05; 3]
                        },
                    },
                    position: [0., 0., 0.0051 + 0.5 * i as f64],
                    velocity: [0.; 3],
                    mass: Some(if i == 0 { 120. } else { 0.6 }),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [moment, moment, moment, 0., 0., 0.],
                }),
            )
            .unwrap()
    });
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [50., 50., 0.05],
                },
                position: [0., 0., -0.0498],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let before = ids.map(|id| backend.state(id));
    assert_eq!(
        backend.add_serial_revolutes(
            ids,
            [[0., 0., 0.5051], [0., 0., 1.0051]],
            [[0., 1., 0.], [1., 0., 0.]]
        ),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(ids.map(|id| backend.state(id)), before);
    assert!(ids.iter().all(|id| backend.is_jointed(*id) == Some(false)));
}
#[test]
fn other_engines_keep_default_unsupported_before_argument_validation() {
    let mut backend = RapierBackend::new([0.; 3]).unwrap();
    let error = backend.add_serial_revolutes(
        [BodyId::from_raw(u64::MAX); 3],
        [[f64::NAN; 3]; 2],
        [[0.; 3]; 2],
    );
    assert!(matches!(
        error,
        Err(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "atomic two-revolute serial assembly"
        })
    ));
}

#[test]
fn child_frames_refuse_invalid_inputs_atomically_and_other_engines_before_validation() {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let ids = three(&mut backend, false);
    let before = ids.map(|id| backend.state(id));
    let local = [[-1., 0., 0.]; 2];
    for (anchors, axes) in [
        (local, [[0.; 3]; 2]),
        ([[f64::NAN, 0., 0.], local[1]], AXES),
        (local, [[0., 0., 1.], [f64::INFINITY, 0., 0.]]),
        ([[1e9, 0., 0.], local[1]], AXES),
    ] {
        assert_eq!(
            backend.add_serial_revolutes_in_child_frames(ids, anchors, axes),
            Err(PhysicsError::InvalidJoint)
        );
        assert_eq!(ids.map(|id| backend.state(id)), before);
        assert!(
            ids.into_iter()
                .all(|id| backend.is_jointed(id) == Some(false))
        );
    }
    assert_eq!(
        backend.add_serial_revolutes_in_child_frames(
            [ids[0], ids[1], BodyId::from_raw(999)],
            local,
            AXES
        ),
        Err(PhysicsError::MissingBody)
    );
    assert!(matches!(
        backend.add_serial_revolutes_in_child_frames([ids[0], ids[1], ids[1]], local, AXES),
        Err(PhysicsError::Unsupported { .. })
    ));
    let mut other = RapierBackend::new([0.; 3]).unwrap();
    assert!(matches!(
        other.add_serial_revolutes_in_child_frames(
            [BodyId::from_raw(999); 3],
            [[f64::NAN; 3]; 2],
            [[f64::NAN; 3]; 2]
        ),
        Err(PhysicsError::Unsupported { .. })
    ));
    let joints = backend
        .add_serial_revolutes_in_child_frames(ids, local, AXES)
        .unwrap();
    assert_eq!(joints.map(JointId::raw), [0, 1]);
    backend.step(DT).unwrap();
    let before = ids.map(|id| backend.state(id));
    assert_eq!(
        backend.add_serial_revolutes_in_child_frames(
            [BodyId::from_raw(999); 3],
            [[f64::NAN; 3]; 2],
            AXES
        ),
        Err(PhysicsError::JointConstructionClosed)
    );
    assert_eq!(ids.map(|id| backend.state(id)), before);
    let mut guarded = DartBackend::new([0.; 3]).unwrap();
    let ids = three(&mut guarded, false);
    let fixed = body(&mut guarded, [20., 0., 0.], true);
    let before = [ids[0], ids[1], ids[2], fixed].map(|id| guarded.state(id));
    assert!(matches!(
        guarded.add_serial_revolutes_in_child_frames([ids[0], ids[1], fixed], local, AXES),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(
        [ids[0], ids[1], ids[2], fixed].map(|id| guarded.state(id)),
        before
    );
    let first = guarded
        .add_revolute(ids[0], ids[1], [1., 0., 0.], AXES[0])
        .unwrap();
    let before = ids.map(|id| guarded.state(id));
    let joint = guarded.joint_state(first);
    assert!(matches!(
        guarded.add_serial_revolutes_in_child_frames(ids, local, AXES),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(ids.map(|id| guarded.state(id)), before);
    assert_eq!(guarded.joint_state(first), joint);
}

#[test]
fn child_frame_contact_refusal_preserves_work_and_partial_reset() {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let mut control = DartBackend::new([0.; 3]).unwrap();
    let ids = three(&mut backend, true);
    let references = three(&mut control, true);
    let obstacle = body(&mut backend, [0.; 3], false);
    let other = body(&mut control, [0.; 3], false);
    backend
        .add_world_wrench(ids[0], [1., 2., 3.], [0.2, -0.1, 0.3])
        .unwrap();
    let before = [ids[0], ids[1], ids[2], obstacle].map(|id| backend.state(id));
    assert_eq!(
        backend.add_serial_revolutes_in_child_frames(ids, [[0.; 3]; 2], AXES),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!(
        [ids[0], ids[1], ids[2], obstacle].map(|id| backend.state(id)),
        before
    );
    assert!(backend.remove(obstacle));
    assert!(control.remove(other));
    let joints = backend
        .add_serial_revolutes_in_child_frames(ids, [[0.; 3]; 2], AXES)
        .unwrap();
    let controls = control
        .add_serial_revolutes_in_child_frames(references, [[0.; 3]; 2], AXES)
        .unwrap();
    assert_eq!(joints.map(JointId::raw), [0, 1]);
    control
        .add_world_wrench(references[0], [1., 2., 3.], [0.2, -0.1, 0.3])
        .unwrap();
    backend.step(DT).unwrap();
    control.step(DT).unwrap();
    for (id, expected) in ids.into_iter().zip(references) {
        near(backend.state(id).unwrap(), control.state(expected).unwrap());
    }
    for (joint, expected) in joints.into_iter().zip(controls) {
        assert_eq!(backend.joint_state(joint), control.joint_state(expected));
    }
    // Removing either edge destroys only that accepted two-edge reset group.
    for removed in 0..2 {
        let mut b = DartBackend::new([0.; 3]).unwrap();
        let bodies = three(&mut b, false);
        let edges = b
            .add_serial_revolutes_in_child_frames(bodies, [[-1., 0., 0.]; 2], AXES)
            .unwrap();
        let kept = 1 - removed;
        assert!(b.remove_joint(edges[removed]));
        let initial = b.initial_state();
        let before = bodies.map(|id| b.state(id).unwrap());
        b.add_world_wrench(bodies[0], [1., 0., 0.], [0., 0., 0.1])
            .unwrap();
        b.step(DT).unwrap();
        let prepared = b.prepare_initial(&initial).unwrap();
        b.apply_initial(prepared).unwrap();
        assert!(b.joint_state(edges[removed]).is_none());
        assert!(b.joint_state(edges[kept]).is_some());
        for (id, expected) in bodies.into_iter().zip(before) {
            near(b.state(id).unwrap(), expected);
        }
        for id in bodies {
            assert!(b.remove(id));
            assert!(b.state(id).is_none());
        }
        assert!(b.joint_state(edges[kept]).is_none());
    }
}
