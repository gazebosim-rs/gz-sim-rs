// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{
    FiniteBoxFlight, FiniteFeatureArc, FinitePointSite, FlightBox, RectangularNormalContact,
    SeparatingFeature, SphericalPointMaterial,
};

#[test]
fn a_touching_unloaded_sole_has_no_pressure_center_until_a_load_is_applied() {
    let rows = [(-1., -1.), (-1., 1.), (1., -1.), (1., 1.)]
        .map(|(x, y)| DVec3::new(1., y * 0.2, -x * 0.1));
    let mass = 2.;
    let mobility = DVec3::new(1. / mass, 1. / 0.03, 1. / 0.05);
    let unloaded = RectangularNormalContact {
        inverse_mass: rows.map(|a| rows.map(|b| a.dot(mobility * b))),
        free_normal: [0.; 4],
    };
    let lift_off = unloaded.solve().unwrap();
    assert_eq!(lift_off.resultant, 0.);
    assert_eq!(lift_off.normalized_center, None);
    assert_eq!(lift_off.equivalents.at(0.), Some([0.; 4]));
    assert_eq!(lift_off.equivalents.at(1e-10), None);
    let loaded = RectangularNormalContact {
        free_normal: [-9.81; 4],
        ..unloaded
    }
    .solve()
    .unwrap();
    assert!((loaded.resultant - mass * 9.81).abs() < 1e-13);
    assert!(
        loaded
            .normalized_center
            .unwrap()
            .iter()
            .all(|v| v.abs() < 1e-13)
    );
    // Two fixed fixtures have no mobility to respond to a closing acceleration.
    assert!(
        RectangularNormalContact {
            inverse_mass: [[0.; 4]; 4],
            free_normal: [-9.81; 4]
        }
        .solve()
        .is_err()
    );
}

#[test]
fn a_single_sliding_corner_approaches_the_support_rim_without_carrying_pressure_across_it() {
    let point = DVec3::new(0.299, 0., 0.);
    let q = DQuat::from_rotation_y(0.34) * DQuat::from_rotation_x(-0.12);
    let half = DVec3::new(0.07, 0.03, 0.04);
    let contact_arm = q * (half * (q.inverse() * -DVec3::Z).signum());
    let flight = FiniteBoxFlight {
        moving: FlightBox {
            center: point - contact_arm,
            orientation: q,
            half_extent: half,
        },
        support: FlightBox {
            center: -0.1 * DVec3::Z,
            orientation: DQuat::IDENTITY,
            half_extent: DVec3::new(0.3, 0.2, 0.1),
        },
        center_of_mass: point,
        relative_velocity: DVec3::X,
        angular_velocity: DVec3::ZERO,
        acceleration: -9.81 * DVec3::Z,
    };
    let site = FinitePointSite {
        feature: SeparatingFeature::SupportFace(2),
        point,
        normal: DVec3::Z,
    };
    let mat = SphericalPointMaterial {
        mass: 2.,
        inertia: 0.1,
        friction: 0.2,
    };
    let arc = FiniteFeatureArc::new(flight, site, mat, false, 1e-13)
        .unwrap()
        .unwrap();
    let t = 0.0005;
    let supported = arc.sliding_vertex_approach(t).unwrap().unwrap();
    let expected = DVec3::new(-mat.friction, 0., 1.) * (mat.mass * 9.81 * t);
    assert!(supported.motion.impulse.distance(expected) < 1e-13);
    let x = point.x + t - 0.5 * mat.friction * 9.81 * t * t;
    assert!((supported.motion.center.x - x).abs() < 1e-13);
    assert!(arc.resolved_spatial_transfers(t).unwrap().is_none());
    assert!(arc.sliding_vertex_approach(0.003).unwrap().is_none());
    // Only one corner is involved. A handoff to two body vertices would invent a support.
    assert!(arc.vertex_handoff_hint(0.003).unwrap().is_none());
}

#[test]
fn a_tilted_elastic_edge_returns_the_force_and_moment_derived_from_its_stored_energy() {
    let layer = gz_physics::ElasticLineFoundation::new(2.4e6, 1.2e6).unwrap();
    let length = 0.24;
    let contact = |translation: f64, tilt: f64| {
        layer
            .integrate_segment(
                length,
                [
                    0.002 + translation - 0.5 * length * tilt,
                    -0.004 + translation + 0.5 * length * tilt,
                ],
            )
            .unwrap()
    };
    let eps = 1e-7;
    let at_rest = contact(0., 0.);
    let d_vertical = (contact(eps, 0.).energy_j - contact(-eps, 0.).energy_j) / (2. * eps);
    let d_pitch = (contact(0., eps).energy_j - contact(0., -eps).energy_j) / (2. * eps);
    let center = length * (at_rest.center_of_pressure_fraction.unwrap() - 0.5);
    assert!((at_rest.loaded_length_m - length / 3.).abs() < 1e-14);
    assert!((at_rest.normal_force_n - d_vertical).abs() < 1e-7);
    assert!((at_rest.normal_force_n * center - d_pitch).abs() < 1e-7);
}
