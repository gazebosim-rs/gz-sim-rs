// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, BodyState, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::UnitQuaternion;
use std::time::Duration;
#[test]
fn offset_rotated_impulses_preserve_total_world_momentum() {
    use nalgebra::{Matrix3, Quaternion, Vector3};
    for target in 0..2 {
        let mut b = DartBackend::new([0.0; 3]).unwrap();
        let mut ids = vec![];
        let centers = [[0.03, -0.02, 0.04], [-0.02, 0.01, 0.04]];
        let masses = [2.0, 3.0];
        let moments = [[0.03, 0.04, 0.05], [0.04, 0.05, 0.06]];
        for i in 0..2 {
            let q = UnitQuaternion::from_euler_angles(0.3 + 0.2 * i as f64, -0.4, 0.7);
            let q = q.quaternion();
            ids.push(
                b.spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position: [i as f64 * 2., 0.1, 0.3],
                        velocity: [0.1, -0.2, 0.3],
                        mass: Some(masses[i]),
                    },
                    RotationSpec {
                        orientation: [q.i, q.j, q.k, q.w],
                        angular_velocity: [0.2, 0.3, -0.1],
                    },
                    Some(Inertia {
                        center: centers[i],
                        orientation: [0., 0., 0., 1.],
                        tensor: [moments[i][0], moments[i][1], moments[i][2], 0., 0., 0.],
                    }),
                )
                .unwrap(),
            );
        }
        b.add_revolute(ids[0], ids[1], [0.4, 0.3, 0.2], [0.3, 0.8, 0.5])
            .unwrap();
        let momentum = |b: &DartBackend| {
            let mut p = Vector3::zeros();
            let mut l = Vector3::zeros();
            for i in 0..2 {
                let s = b.state(ids[i]).unwrap();
                let [x, y, z, w] = s.orientation;
                let r = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
                let c = Vector3::from(s.position) + r * Vector3::from(centers[i]);
                let v = Vector3::from(s.velocity) * masses[i];
                p += v;
                l += c.cross(&v)
                    + r * (Matrix3::from_diagonal(&Vector3::from(moments[i]))
                        * (r.inverse() * Vector3::from(s.angular_velocity)));
            }
            (p, l)
        };
        let (p0, l0) = momentum(&b);
        let state = b.state(ids[target]).unwrap();
        let [x, y, z, w] = state.orientation;
        let r = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
        let c = Vector3::from(state.position) + r * Vector3::from(centers[target]);
        let impulse = Vector3::new(0.2, -0.3, 0.4);
        let torque = Vector3::new(-0.1, 0.2, 0.3);
        b.apply_impulse(ids[target], impulse.into(), torque.into())
            .unwrap();
        let (p1, l1) = momentum(&b);
        assert!((p1 - p0 - impulse).norm() < 1e-12);
        assert!((l1 - l0 - c.cross(&impulse) - torque).norm() < 1e-12);
    }
}

fn offset_pair() -> (DartBackend, [gz_physics::BodyId; 2], gz_physics::JointId) {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let bodies = [0.0, -2.0].map(|z| {
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.1 },
                    position: [0.0, 0.0, z],
                    velocity: [0.1, 0.2, 0.3],
                    mass: Some(1.0),
                },
                RotationSpec {
                    angular_velocity: [0.0, 0.2, 0.0],
                    ..Default::default()
                },
                Some(Inertia {
                    center: [0.0, 0.0, 0.1],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap()
    });
    let joint = backend
        .add_revolute(bodies[0], bodies[1], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
        .unwrap();
    (backend, bodies, joint)
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
        assert!((x - y).abs() < 1e-12, "{a:?} vs {b:?}");
    }
}
#[test]
fn native_origin_bound_restores_all_seven_rates_and_future_motion() {
    let (mut candidate, bodies, joint) = offset_pair();
    let (mut clean, controls, clean_joint) = offset_pair();
    // Warm coupled matrices and Jacobians, without changing physical rates.
    candidate
        .apply_impulse(bodies[0], [0.0; 3], [0.0; 3])
        .unwrap();
    clean
        .apply_impulse(controls[0], [0.0; 3], [0.0; 3])
        .unwrap();
    let before = bodies.map(|id| candidate.state(id).unwrap());
    let rate = candidate.joint_state(joint).unwrap();
    // Both COM vectors fit the Rust bound; origin conversion reaches 1.045e60.
    assert_eq!(
        candidate.set_world_velocity(bodies[0], [-9.5e59, 0.0, 0.0], [0.0, 9.5e59, 0.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(bodies.map(|id| candidate.state(id).unwrap()), before);
    assert_eq!(candidate.joint_state(joint).unwrap(), rate);
    for _ in 0..3 {
        candidate.add_joint_effort(joint, 0.3).unwrap();
        clean.add_joint_effort(clean_joint, 0.3).unwrap();
        candidate.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near(candidate.state(a).unwrap(), clean.state(b).unwrap());
        }
    }
}
