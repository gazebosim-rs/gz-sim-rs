// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn make() -> (FrictionlessEdgeSupport, EdgeSupportState) {
    let q = DQuat::from_rotation_z(0.47) * DQuat::from_rotation_x(0.18);
    let arms = [
        DVec3::new(-0.13, -0.065, -0.1),
        DVec3::new(0.19, -0.065, -0.1),
    ];
    let r = q * arms[0];
    let w = (q * DVec3::X) * 0.52 + DVec3::Z * 1.27;
    (
        FrictionlessEdgeSupport {
            mass: 1.74,
            inertia_body: DMat3::from_cols(
                DVec3::new(0.026, 0.002, -0.003),
                DVec3::new(0.002, 0.043, 0.004),
                DVec3::new(-0.003, 0.004, 0.058),
            ),
            sites_body: arms,
            normal: DVec3::Z,
            plane_offset: 0.48,
            gravity: DVec3::new(0.2, -0.3, -9.81),
        },
        EdgeSupportState {
            center: DVec3::new(0.75, -0.28, 0.48 - r.z),
            orientation: q,
            velocity: DVec3::new(-0.32, 0.14, -w.cross(r).z),
            angular_velocity: w,
        },
    )
}
#[test]
fn triaxial_momentum_reduction_matches_resolved_newton_euler_reactions() {
    let (b, s) = make();
    let arc = b.bind(s).unwrap().unwrap();
    let base = arc.sample(0., 1.).unwrap();
    let mut cursor = arc.trajectory();
    let (mut first, mut moment) = (DVec3::ZERO, DVec3::ZERO);
    let mut forces = base.signed_force;
    let mut prev = s;
    let mut energy = 0f64;
    for _ in 0..500 {
        let dt = 0.00005;
        let sample = cursor.advance(dt, dt).unwrap();
        let x = sample.state;
        let w = x.angular_velocity;
        let q = x.orientation;
        let i = DMat3::from_quat(q) * b.inertia_body * DMat3::from_quat(q).transpose();
        let arms = b.sites_body.map(|p| q * p);
        let t = arms.map(|r| r.cross(b.normal));
        let gyro = i.inverse() * (-w.cross(i * w));
        let normal = crate::TwoNormalContact {
            inverse_mass: t.map(|u| t.map(|v| 1. / b.mass + u.dot(i.inverse() * v))),
            free_normal: arms.map(|r| {
                b.normal
                    .dot(b.gravity + gyro.cross(r) + w.cross(w.cross(r)))
            }),
        }
        .solve()
        .unwrap();
        for (k, &v) in normal.load.iter().enumerate() {
            assert!((v - sample.signed_force[k]).abs() < 5e-13);
            assert!(b.normal.dot(x.velocity + w.cross(arms[k])).abs() < 1e-15);
            assert!((b.normal.dot(x.center + arms[k]) - b.plane_offset).abs() < 3e-15);
            first += b.normal * (0.5 * dt * (v + forces[k]));
            moment += 0.5
                * dt
                * (t[k] * v + (prev.orientation * b.sites_body[k]).cross(b.normal) * forces[k]);
        }
        let now = sample.signed_impulse.iter().sum::<f64>();
        let integrated = b.mass * (x.velocity - s.velocity - b.gravity * cursor.time);
        assert!(integrated.distance(b.normal * now) < 2e-13);
        energy = energy.max((sample.mechanical_energy - base.mechanical_energy).abs());
        assert!((sample.yaw_momentum - base.yaw_momentum).abs() < 5e-15);
        prev = x;
        forces = sample.signed_force;
    }
    assert!(energy < 1e-12);
    let h = |s: EdgeSupportState| {
        s.orientation * (b.inertia_body * (s.orientation.conjugate() * s.angular_velocity))
    };
    assert!(first.distance(b.mass * (prev.velocity - s.velocity - b.gravity * cursor.time)) < 2e-8);
    assert!(moment.distance(h(prev) - h(s)) < 2e-9);
    let back = cursor.advance(-cursor.time, 0.00005).unwrap();
    assert!(back.state.angular_velocity.distance(s.angular_velocity) < 5e-13);
    assert!(back.state.center.distance(s.center) < 1e-13);
    assert!(back.signed_impulse.iter().all(|v| v.abs() < 2e-12));
}
#[test]
fn reflected_site_order_and_rotated_plane_preserve_trajectory_and_reverse_loads() {
    let (b, s) = make();
    let transform = DQuat::from_rotation_y(-0.52) * DQuat::from_rotation_z(0.23);
    let shift = DVec3::new(1., -0.8, 0.32);
    let nn = transform * b.normal;
    let transformed = FrictionlessEdgeSupport {
        normal: nn,
        plane_offset: b.plane_offset + nn.dot(shift),
        gravity: transform * b.gravity,
        sites_body: [b.sites_body[1], b.sites_body[0]],
        ..b
    };
    let start = EdgeSupportState {
        center: transform * s.center + shift,
        orientation: transform * s.orientation,
        velocity: transform * s.velocity,
        angular_velocity: transform * s.angular_velocity,
    };
    let a = b.bind(s).unwrap().unwrap().sample(0.043, 0.0001).unwrap();
    let c = transformed
        .bind(start)
        .unwrap()
        .unwrap()
        .sample(0.043, 0.0001)
        .unwrap();
    assert!((transform * a.state.center + shift).distance(c.state.center) < 2e-14);
    assert!((transform * a.state.angular_velocity).distance(c.state.angular_velocity) < 2e-13);
    for k in 0..2 {
        assert!((a.signed_impulse[k] - c.signed_impulse[1 - k]).abs() < 5e-14);
    }
    assert!(
        b.bind(EdgeSupportState {
            velocity: s.velocity + 1e-4 * b.normal,
            ..s
        })
        .unwrap()
        .is_none()
    );
    assert!(
        b.bind(EdgeSupportState {
            center: s.center + 1e-6 * b.normal,
            ..s
        })
        .unwrap()
        .is_none()
    );
    assert!(
        FrictionlessEdgeSupport {
            gravity: -b.gravity,
            ..b
        }
        .bind(s)
        .unwrap()
        .is_none()
    );
    assert!(
        FrictionlessEdgeSupport {
            inertia_body: -b.inertia_body,
            ..b
        }
        .bind(s)
        .is_err()
    );
    let mut cursor = b.bind(s).unwrap().unwrap().trajectory();
    assert!(cursor.advance(1., 0.).is_err());
    assert_eq!(cursor.time, 0.);
}

#[test]
fn short_segment_bounds_contain_signed_loads_vertex_gaps_and_speeds() {
    let (b, s) = make();
    let arc = b.bind(s).unwrap().unwrap();
    for h in [1e-9, 0.0000625, 0.0005, 0.002] {
        let guard = arc.enclose(h).unwrap().unwrap();
        let vertex = DVec3::new(0.18, 0.11, -0.1);
        let [lo, hi] = guard.height(vertex);
        for step in 0..=32 {
            let v = arc.sample(h * f64::from(step) / 32., 0.0000625).unwrap();
            let at = b.normal.dot(v.state.center + v.state.orientation * vertex) - b.plane_offset;
            assert!(lo <= at && at <= hi);
            for (bounds, load) in guard.force_bounds.into_iter().zip(v.signed_force) {
                assert!(bounds[0] <= load && load <= bounds[1]);
            }
            assert!(v.state.velocity.length() <= guard.center_speed_bound);
            assert!(v.state.angular_velocity.length() <= guard.angular_speed_bound);
        }
        if h < 0.001 {
            assert!(guard.force_bounds.into_iter().all(|bound| bound[0] > 0.));
        }
    }
    assert!(arc.enclose(10.).unwrap().is_none());
    assert!(arc.enclose(f64::NAN).is_err());
}
