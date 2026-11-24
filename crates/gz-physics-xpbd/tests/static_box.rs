// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Boundary, Particle, SoftBody, SolverConfig, XpbdError};
use std::time::Duration;

const DT: Duration = Duration::from_millis(10);
const HALF: [f64; 3] = [0.4, 0.2, 0.1];
const CENTER: DVec3 = DVec3::new(-0.2, 0.6, 0.5);
const R: f64 = 0.025;
const MASS: f64 = 0.12;

fn box_at(rotation: DQuat) -> Boundary {
    Boundary::StaticCuboid {
        center: CENTER.to_array(),
        half_extents: HALF,
        orientation: rotation.to_array(),
    }
}
fn particle(x: DVec3, v: DVec3) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: x.to_array(),
            velocity: v.to_array(),
            inverse_mass: 1. / MASS,
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            iterations: 4,
            substeps: 1,
            friction: 0.2,
            particle_radius: R,
            self_contact: false,
        },
    )
    .unwrap()
}

#[test]
fn swept_faces_edges_corners_and_underside_transfer_measured_momentum() {
    let q = DQuat::from_rotation_z(0.78) * DQuat::from_rotation_x(0.25);
    for (witness, n) in [
        (DVec3::new(0.1, 0., 0.1), DVec3::Z),
        (DVec3::new(0.1, 0., -0.1), -DVec3::Z),
        (DVec3::new(0.4, 0.02, 0.), DVec3::X),
        (
            DVec3::new(0.4, 0.03, 0.1),
            DVec3::new(1., 0., 1.).normalize(),
        ),
        (DVec3::from_array(HALF), DVec3::ONE.normalize()),
    ] {
        let skin = CENTER + q * (witness + R * n);
        let normal = q * n;
        let velocity = -80. * normal;
        let mut body = particle(skin + 0.004 * normal, velocity);
        body.step(DT, &[box_at(q)]).unwrap();
        let point = body.particles()[0];
        assert!(DVec3::from(point.position).distance(skin) < 2e-10);
        let impulse = DVec3::from(body.last_boundary_impulses()[0].linear);
        let moment = DVec3::from(body.last_boundary_impulses()[0].angular);
        assert!((impulse - MASS * (velocity - DVec3::from(point.velocity))).length() < 2e-10);
        assert!((impulse + 80. * MASS * normal).length() < 2e-10);
        assert!((moment - (skin - CENTER).cross(impulse)).length() < 2e-10);
    }
}
#[test]
fn coulomb_cap_and_corner_clearance() {
    let p = CENTER + DVec3::new(0., 0., 0.1 + R + 0.01);
    let mut body = particle(p, DVec3::new(4., 0., -2.));
    body.step(DT, &[box_at(DQuat::IDENTITY)]).unwrap();
    let j = body.last_boundary_impulses()[0].linear;
    assert!((j[2] + 2. * MASS).abs() < 1e-12);
    assert!((j[0] + 0.2 * j[2]).abs() < 1e-12);
    let v = DVec3::new(0., 0., -25.);
    // Within the expanded box AABB but beyond the rounded edge at x/y.
    let p = CENTER + DVec3::new(0.4 + 0.02, 0.2 + 0.02, 0.16);
    let mut outside = particle(p, v);
    outside.step(DT, &[box_at(DQuat::IDENTITY)]).unwrap();
    assert!(DVec3::from(outside.particles()[0].position).distance(p + 0.01 * v) < 1e-14);
    assert_eq!(outside.last_boundary_impulses()[0].linear, [0.; 3]);
}

#[test]
fn validates_dimensions_and_orientation_before_advancing() {
    let mut body = particle(CENTER + DVec3::Z, DVec3::ZERO);
    for (half, quaternion) in [
        ([f64::NAN; 3], DQuat::IDENTITY),
        ([0.; 3], DQuat::IDENTITY),
        ([f64::MAX; 3], DQuat::IDENTITY),
        (HALF, DQuat::from_array([f64::NAN; 4])),
        (HALF, DQuat::from_array([1.; 4])),
    ] {
        assert_eq!(
            body.step(
                DT,
                &[Boundary::StaticCuboid {
                    center: CENTER.to_array(),
                    half_extents: half,
                    orientation: quaternion.to_array()
                }]
            ),
            Err(XpbdError::InvalidBoundary)
        );
    }
    assert_eq!(body.particles()[0].position, (CENTER + DVec3::Z).to_array());
}

#[test]
fn exact_endpoint_stops_now_and_tangency_transfers_nothing() {
    let b = box_at(DQuat::IDENTITY);
    let start = CENTER + DVec3::Z * (HALF[2] + R + 0.01);
    let mut head_on = particle(start, -DVec3::Z);
    head_on.step(DT, &[b]).unwrap();
    assert!((head_on.last_boundary_impulses()[0].linear[2] + MASS).abs() < 1e-12);
    let start = CENTER + DVec3::Z * (HALF[2] + R);
    let mut tangent = particle(start, DVec3::X);
    tangent.step(DT, &[b]).unwrap();
    assert_eq!(tangent.last_boundary_impulses()[0].linear, [0.; 3]);
    let mut enormous = particle(DVec3::ZERO, DVec3::X * 1e160);
    assert_eq!(enormous.step(DT, &[b]), Err(XpbdError::Numerical));
}
#[test]
fn broad_phase_contains_the_rotated_long_axis_and_culls_distant_boxes() {
    let q = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let shapes: Vec<_> = (0..8)
        .map(|i| Boundary::StaticCuboid {
            center: (CENTER + 3. * f64::from(i) * DVec3::X).to_array(),
            half_extents: HALF,
            orientation: q.to_array(),
        })
        .collect();
    let mut points = Vec::new();
    for i in 0..64 {
        let x = CENTER
            + DVec3::new(0., 0.32 + f64::from(i % 8) * 0.001, HALF[2] + R + 0.001)
            + if i < 32 { DVec3::ZERO } else { 10. * DVec3::Y };
        points.push(Particle {
            position: x.to_array(),
            velocity: (-DVec3::Z).to_array(),
            inverse_mass: 1. / MASS,
        });
    }
    let cfg = particle(CENTER, DVec3::ZERO).config();
    let mut batch = SoftBody::new(points, vec![], cfg).unwrap();
    batch.step(DT, &shapes).unwrap();
    assert!((batch.last_boundary_impulses()[0].linear[2] + 32. * MASS).abs() < 1e-10);
    assert!(
        batch.last_boundary_impulses()[1..]
            .iter()
            .all(|j| j.linear == [0.; 3])
    );
}
