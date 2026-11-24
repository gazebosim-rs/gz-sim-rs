// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, RigidMobility, SoftBody, SolverConfig, SphereMobility, XpbdError,
    step_coupled_rigid,
};
use std::time::Duration;

const R: f64 = 0.2;
const H: f64 = 0.5;
const SKIN: f64 = 0.025;
const MASS: f64 = 0.6;
const STEP: Duration = Duration::from_millis(20);

fn particle(p: DVec3, v: DVec3, substeps: u32, mu: f64) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: p.to_array(),
            velocity: v.to_array(),
            inverse_mass: 1. / MASS,
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            particle_radius: SKIN,
            substeps,
            iterations: 4,
            self_contact: false,
            friction: mu,
        },
    )
    .unwrap()
}
fn guide(center: DVec3, axis: DVec3, velocity: DVec3, angular_speed: f64) -> Boundary {
    Boundary::Capsule {
        center: center.to_array(),
        axis: axis.to_array(),
        radius: R,
        half_length: H,
        velocity: velocity.to_array(),
        angular_speed,
    }
}
#[test]
fn swept_sides_endcaps_and_seams_obey_incident_plane_and_coulomb_impulses() {
    // A segment through the centerline has a known entry time and normal. Its
    // material velocity is omega × (R n) even on an offset spherical cap.
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.71, -0.53, 1.27);
    let axis = q * DVec3::Z;
    let origin = DVec3::new(1.3, -0.7, 0.6);
    let translation = DVec3::new(0.35, 0.2, -0.12);
    let spin = 10.;
    for (case, z, local) in [
        ("barrel", 0.17, DVec3::X),
        ("seam", H, DVec3::X),
        ("cap", H, DVec3::new(0.8, 0., 0.6)),
        ("lower cap", -H, DVec3::new(0.8, 0., -0.6)),
        ("tip", H, DVec3::Z),
    ] {
        let n = q * local;
        let point = origin + z * axis + 0.47 * n;
        let incoming = translation - 45. * n;
        for mu in [0., 0.025, 0.25] {
            let vt = spin * R * axis.cross(n);
            let tangential = vt.clamp_length_max(mu * 45.);
            let expected = translation + tangential;
            for count in [1, 2, 4, 16] {
                let mut soft = particle(point, incoming, count, mu);
                soft.step(STEP, &[guide(origin, axis, translation, spin)])
                    .unwrap();
                let out = soft.particles()[0];
                let v = DVec3::from(out.velocity);
                let reaction = soft.last_boundary_impulses()[0];
                let linear = (v - incoming) * MASS;
                let angular = (DVec3::from(out.position) - origin).cross(v * MASS)
                    - (point - origin).cross(incoming * MASS);
                assert!(
                    v.distance(expected) < 2e-10,
                    "{case} mu={mu} n={count} {v:?} vs {expected:?}"
                );
                assert!((DVec3::from(reaction.linear) + linear).length() < 2e-10);
                assert!((DVec3::from(reaction.angular) + angular).length() < 2e-10);
                let min_gap =
                    (DVec3::from(out.position) - origin - translation * STEP.as_secs_f64()).dot(n)
                        - (R + SKIN + H * axis.dot(n).abs());
                assert!(min_gap >= -1e-11, "{case}: {min_gap}");
            }
        }
    }
}
#[test]
fn cap_tangency_clearance_and_finite_length_controls() {
    let boundary = guide(DVec3::ZERO, DVec3::Z, DVec3::ZERO, 10.);
    for z in [H + R + SKIN, H + R + SKIN + 1e-8, H + R + SKIN + 1.] {
        let p = DVec3::new(-0.45, 0., z);
        let v = DVec3::X * 45.;
        let mut soft = particle(p, v, 1, 0.5);
        soft.step(STEP, &[boundary]).unwrap();
        assert_eq!(soft.last_boundary_impulses()[0].linear, [0.; 3]);
        assert_eq!(soft.particles()[0].velocity, v.to_array());
    }
    let mut inward = particle(
        DVec3::new(-0.45, 0., H + R + SKIN - 1e-8),
        DVec3::X * 45.,
        1,
        0.,
    );
    inward.step(STEP, &[boundary]).unwrap();
    assert!(inward.particles()[0].velocity[2] > 0.001);
    let mut tangent = particle(
        DVec3::new(-0.45, R + SKIN, H * 0.95),
        DVec3::X * 45.,
        1,
        0.1,
    );
    tangent.step(STEP, &[boundary]).unwrap();
    assert_eq!(tangent.last_boundary_impulses()[0].linear, [0.; 3]);
}
#[test]
fn embedded_centerline_geometry_and_invalid_boundary_admission() {
    let mut soft = particle(DVec3::ZERO, DVec3::ZERO, 1, 0.);
    soft.step(STEP, &[guide(DVec3::ZERO, DVec3::Z, DVec3::ZERO, 0.)])
        .unwrap();
    let x = DVec3::from(soft.particles()[0].position);
    assert!(x.truncate().length() >= R + SKIN - 1e-12);
    assert!(DVec3::from(soft.particles()[0].velocity).length() < 1e-11);
    let b = guide(DVec3::ZERO, DVec3::Z, DVec3::ZERO, 0.);
    let p = DVec3::new(R + SKIN - 0.001, 0., 0.12);
    let mut tunneled = particle(p, -45. * DVec3::X, 1, 0.);
    tunneled.step(STEP, &[b]).unwrap();
    assert!((tunneled.particles()[0].position[0] - R - SKIN).abs() < 1e-12);
    assert!(DVec3::from(tunneled.particles()[0].velocity).length() < 1e-12);
    let mut departing = particle(p, 45. * DVec3::X, 1, 0.);
    departing.step(STEP, &[b]).unwrap();
    assert_eq!(departing.last_boundary_impulses()[0].linear, [0.; 3]);
    for b in [
        guide(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO, 0.),
        guide(DVec3::ZERO, DVec3::Z, DVec3::ZERO, f64::NAN),
        Boundary::Capsule {
            center: [0.; 3],
            axis: [0., 0., 1.],
            radius: 0.,
            half_length: H,
            velocity: [0.; 3],
            angular_speed: 0.,
        },
    ] {
        assert_eq!(soft.step(STEP, &[b]), Err(XpbdError::InvalidBoundary));
    }
    let mut bodies = vec![&mut soft];
    let m = RigidMobility::Isotropic(SphereMobility {
        inverse_mass: 1.,
        inverse_inertia: 1.,
    });
    assert_eq!(
        step_coupled_rigid(
            &mut bodies,
            STEP,
            &[guide(DVec3::ZERO, DVec3::Z, DVec3::ZERO, 0.)],
            &[Some(m)]
        )
        .unwrap_err(),
        XpbdError::InvalidBoundary
    );
}

#[test]
fn oblique_guide_sweeps_agree_with_parry_raycast_on_the_inflated_capsule() {
    use parry3d_f64::{
        math::Vector,
        query::{Ray, RayCast},
        shape::Capsule,
    };
    let shape = Capsule::new_z(H, R + SKIN);
    let boundary = guide(DVec3::ZERO, DVec3::Z, DVec3::ZERO, 0.);
    let (mut hits, mut misses) = (0, 0);
    let mut max_error = 0f64;
    for yi in -9..=9 {
        for zi in -13..=13 {
            for dir in [-1., 0., 1.] {
                let a = DVec3::new(-0.6, yi as f64 * 0.03, zi as f64 * 0.06);
                let delta = DVec3::new(1.2, dir * 0.12, -dir * 0.31);
                let v = delta / STEP.as_secs_f64();
                let ray = Ray::new(
                    Vector::from_array(a.to_array()),
                    Vector::from_array(delta.to_array()),
                );
                let expected =
                    if let Some(hit) = shape.cast_local_ray_and_get_normal(&ray, 1., true) {
                        hits += 1;
                        let n = DVec3::from(hit.normal.to_array());
                        v - n * v.dot(n)
                    } else {
                        misses += 1;
                        v
                    };
                let mut body = particle(a, v, 1, 0.);
                body.step(STEP, &[boundary]).unwrap();
                let err = DVec3::from(body.particles()[0].velocity).distance(expected);
                max_error = max_error.max(err);
                assert!(
                    err < 1e-8,
                    "a {a:?} delta {delta:?} error {err} expected {expected:?} got {:?}",
                    body.particles()[0].velocity
                );
            }
        }
    }
    assert!(hits > 600 && misses > 200);
    println!(
        "independent Parry segment ray oracle: {hits} impacts, {misses} misses, max outbound velocity difference {max_error:e} m/s"
    );
}
