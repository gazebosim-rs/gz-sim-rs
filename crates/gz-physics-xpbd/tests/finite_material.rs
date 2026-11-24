// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, MaterialFeature, Particle, PrincipalRigidMotion, RigidMobility, SoftBody,
    SolverConfig, step_coupled_rigid,
};
use std::time::Duration;
const I: DVec3 = DVec3::new(0.045, 0.067, 0.09);
const MASS: f64 = 0.4;
const R: f64 = 0.08;
const H: f64 = 0.2;
fn config(steps: u32) -> SolverConfig {
    SolverConfig {
        gravity: [0.; 3],
        particle_radius: 0.02,
        substeps: steps,
        iterations: 4,
        friction: 0.,
        self_contact: false,
    }
}
fn point(x: DVec3, v: DVec3, m: f64) -> Particle {
    Particle {
        position: x.to_array(),
        velocity: v.to_array(),
        inverse_mass: 1. / m,
    }
}
fn spec(q: DQuat, axes: DQuat, w: DVec3, dt: f64) -> RigidMobility {
    RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: 1. / MASS,
        inverse_principal_inertia: I.recip().to_array(),
        principal_orientation: axes.to_array(),
        start_orientation: q.to_array(),
        end_orientation: (DQuat::from_scaled_axis(w * dt) * q).to_array(),
        start_angular_velocity: w.to_array(),
    })
}
fn capsule(pos: DVec3, v: DVec3, w: DVec3) -> Boundary {
    Boundary::DynamicCapsule {
        center: pos.to_array(),
        radius: R,
        half_length: H,
        velocity: v.to_array(),
        angular_velocity: w.to_array(),
    }
}
fn close(a: DVec3, b: DVec3, tol: f64) {
    assert!(a.distance(b) < tol, "{a:?} vs {b:?} tolerance {tol}");
}
#[test]
fn centered_barrel_impact_has_reduced_mass_velocity_and_post_impact_trajectory() {
    let q = DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    let dt = Duration::from_millis(4);
    for n in [1, 2, 4, 8] {
        let ps = [(-0.6, 0.25), (0.2, 0.75)]
            .map(|(x, m)| point(DVec3::new(x, 0., 0.105), -DVec3::Z * 2., m))
            .to_vec();
        let mut soft = SoftBody::cable(ps, 0., 0., config(n))
            .unwrap()
            .with_capsule_material_contact()
            .unwrap();
        let g = capsule(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
        let m = step_coupled_rigid(
            &mut [&mut soft],
            dt,
            &[g],
            &[Some(spec(q, DQuat::IDENTITY, DVec3::ZERO, 0.004))],
        )
        .unwrap()[0]
            .unwrap();
        let velocity = -DVec3::Z * (2. / (1. + MASS));
        close(m.linear_velocity_correction.into(), velocity, 1e-11);
        close(m.angular_velocity_correction.into(), DVec3::ZERO, 1e-11);
        close(m.translation_correction.into(), velocity * 0.0015, 1e-12);
        let [load] = soft.last_capsule_material_loads() else {
            panic!()
        };
        assert!(load.two_way);
        close(load.linear.into(), MASS * velocity, 1e-12);
        assert!((load.first_time - 0.0025).abs() < 1e-11);
        for p in soft.particles() {
            close(p.velocity.into(), velocity, 1e-11);
            assert!((p.position[2] - (0.1 + velocity.z * 0.0015)).abs() < 1e-12);
        }
    }
}
#[test]
fn tilted_face_at_endcap_gives_the_full_principal_frame_angular_impulse() {
    for n in [1, 2, 4, 8] {
        let dt = Duration::from_millis(5);
        let h = dt.as_secs_f64();
        let q = DQuat::from_rotation_y(-0.37) * DQuat::from_rotation_z(0.4);
        let axes = DQuat::from_rotation_y(0.21) * DQuat::from_rotation_x(-0.33);
        let normal = q * DVec3::new(0.4, 0., (1f64 - 0.4 * 0.4).sqrt());
        let tangent = q * DVec3::Y;
        let side = normal.cross(tangent);
        let arm = q * DVec3::Z * H + normal * (R + 0.02);
        let a = arm + normal * (2. * h) - 0.75 * 0.8 * tangent - 0.25 * 0.6 * side;
        let pts = [
            (a, 0.25),
            (a + 0.8 * tangent, 0.5),
            (a + 0.6 * side, 0.15),
            (a + 0.8 * tangent + 0.6 * side, 0.25),
        ];
        let center = DVec3::new(1.5, 0.34, -0.2);
        let drift = DVec3::new(0.15, 0.37, -0.22);
        let ps = pts
            .map(|(p, m)| point(center + p, drift - 2. * normal, m))
            .to_vec();
        let mut soft = SoftBody::cloth(ps, 2, 1e-3, 1e-3, 1e-3, config(n))
            .unwrap()
            .with_capsule_material_contact()
            .unwrap();
        let inertia = q * axes;
        let inv = |v: DVec3| inertia * ((inertia.inverse() * v) / I);
        let r_cross_n = arm.cross(normal);
        let expected = -normal * 2. / (1. + MASS.recip() + r_cross_n.dot(inv(r_cross_n)));
        let native = step_coupled_rigid(
            &mut [&mut soft],
            dt,
            &[capsule(center, drift, DVec3::ZERO)],
            &[Some(spec(q, axes, DVec3::ZERO, h))],
        )
        .unwrap()[0]
            .unwrap();
        let [load] = soft.last_capsule_material_loads() else {
            panic!("{:?}", soft.last_capsule_material_loads());
        };
        assert_eq!(load.feature, MaterialFeature::Triangle([0, 1, 3]));
        assert!(load.two_way);
        close(load.linear.into(), expected, 1e-11);
        close(
            load.angular.into(),
            (arm + drift * h).cross(expected),
            1e-11,
        );
        close(
            native.linear_velocity_correction.into(),
            expected / MASS,
            1e-11,
        );
        close(
            native.angular_velocity_correction.into(),
            inv(arm.cross(expected)),
            1e-11,
        );
        for (j, f) in load.nodal.iter().zip([0.25, 0.5, 0.25]) {
            close((*j).into(), -f * expected, 1e-11);
        }
        println!(
            "finite tilted cap {n} substeps: pure impulse at {:.9} s, rigid linear {:?} angular {:?}, tensor spin {:?}",
            load.first_time, load.linear, load.angular, native.angular_velocity_correction
        );
    }
}

#[test]
fn a_fast_material_crossing_refines_the_shared_clock_and_refuses_atomically_beyond_budget() {
    let q = DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    let dt = Duration::from_millis(20);
    for speed in [20., 1e6] {
        let ps = [-0.4, 0.4]
            .map(|x| point(DVec3::new(x, 0., 0.3), -DVec3::Z * speed, 0.5))
            .to_vec();
        let mut b = SoftBody::cable(ps, 0., 0., config(1))
            .unwrap()
            .with_capsule_material_contact()
            .unwrap();
        let initial = b.particles().to_vec();
        let m = step_coupled_rigid(
            &mut [&mut b],
            dt,
            &[capsule(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO)],
            &[Some(spec(q, DQuat::IDENTITY, DVec3::ZERO, 0.02))],
        );
        if speed > 1e4 {
            assert!(m.is_err());
            assert_eq!(initial, b.particles());
            assert!(b.last_capsule_material_loads().is_empty());
            continue;
        }
        let state = m.unwrap()[0].unwrap();
        let v = -DVec3::Z * speed / (1. + MASS);
        close(state.linear_velocity_correction.into(), v, 1e-10);
        close(state.translation_correction.into(), v * 0.010, 1e-10);
        assert!(b.last_sweep_refinements() > 0);
        println!(
            "20 m/s swept segment in a single 20 ms public tick: {} retries, receiver velocity {:?}, displacement {:?}",
            b.last_sweep_refinements(),
            state.linear_velocity_correction,
            state.translation_correction
        );
    }
}
#[test]
fn simultaneous_faces_share_impulse_with_two_separate_finite_receivers() {
    let dt = Duration::from_millis(5);
    let d = 0.005;
    let positions = [(-0.30, -0.27), (0.30, -0.27), (-0.30, 0.27), (0.30, 0.27)];
    let ps = positions
        .map(|(x, y)| point(DVec3::new(x, y, 0.1 + 1.8 * d), -DVec3::Z * 1.8, 0.05))
        .to_vec();
    let mut body = SoftBody::cloth(ps, 2, 1e-3, 1e-3, 1e-3, config(1))
        .unwrap()
        .with_capsule_material_contact()
        .unwrap();
    let caps = [(0.15, -0.135), (-0.15, 0.135)]
        .map(|(x, y)| capsule(DVec3::new(x, y, -H), DVec3::ZERO, DVec3::ZERO));
    let motion = Some(spec(DQuat::IDENTITY, DQuat::IDENTITY, DVec3::ZERO, d));
    let r = step_coupled_rigid(&mut [&mut body], dt, &caps, &[motion; 2]).unwrap();
    for m in r {
        let m = m.unwrap();
        close(m.linear_velocity_correction.into(), -0.36 * DVec3::Z, 1e-10);
        close(m.angular_velocity_correction.into(), DVec3::ZERO, 1e-10);
    }
    for p in body.particles() {
        close(p.velocity.into(), -0.36 * DVec3::Z, 1e-10);
    }
    assert_eq!(body.last_capsule_material_loads().len(), 2);
    for l in body.last_capsule_material_loads() {
        close(l.linear.into(), -0.144 * DVec3::Z, 1e-10);
        assert!(l.two_way);
    }
}
