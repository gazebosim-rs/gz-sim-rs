// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact contact literals; plain public backend APIs only.
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use std::time::Duration;

pub const DT: Duration = Duration::from_millis(1);

pub fn near(a: BodyState, b: BodyState) {
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
        assert!((x - y).abs() < 1e-12, "{a:?} != {b:?}");
    }
}
pub fn pair() -> (DartBackend, [BodyId; 3], JointId) {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let mut ids = Vec::new();
    for position in [[0.; 3], [0., 0., -2.], [10., 0., 0.]] {
        ids.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position,
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec::default(),
                    Some(Inertia {
                        center: [0.; 3],
                        orientation: [0., 0., 0., 1.],
                        tensor: [0.004, 0.004, 0.004, 0., 0., 0.],
                    }),
                )
                .unwrap(),
        );
    }
    let ids: [BodyId; 3] = ids.try_into().unwrap();
    let first = backend
        .add_revolute(ids[0], ids[1], [0., 0., -1.], [0., 1., 0.])
        .unwrap();
    (backend, ids, first)
}
pub fn finish(backend: &mut DartBackend, ids: [BodyId; 3]) -> JointId {
    backend
        .add_revolute(ids[1], ids[2], [0.; 3], [0., 1., 0.])
        .unwrap()
}
pub fn obstacle(backend: &mut DartBackend, approaching: bool, dynamic: bool) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: if approaching {
                        [20., 20., 0.1]
                    } else {
                        [0.25, 0.25, 0.1]
                    },
                },
                position: if approaching {
                    [0., 0., -3.1]
                } else {
                    [10., 0., -0.15]
                },
                velocity: [0.; 3],
                mass: if dynamic { Some(1.) } else { None },
            },
            RotationSpec::default(),
            None,
        )
        .unwrap()
}
pub fn warm_commands(backend: &mut DartBackend, ids: [BodyId; 3], first: JointId) {
    backend.apply_impulse(ids[1], [0.; 3], [0.; 3]).unwrap();
    backend
        .add_world_wrench(ids[0], [0.2, 0., 0.], [0., 0.1, 0.])
        .unwrap();
    backend.add_joint_effort(first, 0.7).unwrap();
}
pub fn approaching(dynamic: bool) -> (DartBackend, [BodyId; 3], [JointId; 2], BodyId) {
    let (mut backend, ids, first) = pair();
    let second = finish(&mut backend, ids);
    let floor = obstacle(&mut backend, true, dynamic);
    backend
        .set_world_velocity(ids[0], [0., 0., -3.], [0.; 3])
        .unwrap();
    (backend, ids, [first, second], floor)
}
pub fn approach_commands(backend: &mut DartBackend, ids: [BodyId; 3], second: JointId) {
    backend
        .add_world_wrench(ids[0], [0.01, 0., 0.], [0.; 3])
        .unwrap();
    backend.add_joint_effort(second, 0.001).unwrap();
}
pub fn compare(
    candidate: &DartBackend,
    ids: [BodyId; 3],
    joints: [JointId; 2],
    clean: &DartBackend,
    controls: [BodyId; 3],
    control_joints: [JointId; 2],
) {
    for (a, b) in ids.into_iter().zip(controls) {
        near(candidate.state(a).unwrap(), clean.state(b).unwrap());
    }
    for (a, b) in joints.into_iter().zip(control_joints) {
        assert_eq!(candidate.joint_state(a), clean.joint_state(b));
    }
}
pub fn fixture() -> serde_json::Value {
    let value: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/three_body_static_contacts.json")).unwrap();
    assert_eq!(value["dart_version"], "6.13.2");
    assert_eq!(value["dt_ns"], 1_000_000);
    value
}
pub fn native_row(
    backend: &DartBackend,
    ids: [BodyId; 3],
    joints: [JointId; 2],
    row: &serde_json::Value,
) {
    for (i, id) in ids.into_iter().enumerate() {
        let v = &row["bodies"][i];
        let values = |start: usize| std::array::from_fn(|index| v[start + index].as_f64().unwrap());
        let orientation = std::array::from_fn(|index| v[3 + index].as_f64().unwrap());
        near(
            backend.state(id).unwrap(),
            BodyState {
                position: values(0),
                orientation,
                velocity: values(7),
                angular_velocity: values(10),
            },
        );
    }
    for (i, id) in joints.into_iter().enumerate() {
        let actual = backend.joint_state(id).unwrap();
        assert!((actual.position - row["joints"][i][0].as_f64().unwrap()).abs() < 1e-12);
        assert!((actual.velocity - row["joints"][i][1].as_f64().unwrap()).abs() < 1e-12);
    }
}
