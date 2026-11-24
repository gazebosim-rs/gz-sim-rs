// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn top_ridge() -> Ridge {
    Ridge::all(DVec3::ZERO, DVec3::ONE, DQuat::IDENTITY)
        .nth(3)
        .unwrap()
}

#[test]
fn only_outward_normals_belong_to_ridges_and_vertices() {
    let r = top_ridge();
    assert_eq!(r.feature(0.5, DVec3::Z), Some(11));
    assert_eq!(r.feature(0.5, DVec3::Y), Some(11));
    assert_eq!(r.feature(0.5, DVec3::X), None);
    assert_eq!(r.feature(0.5, -DVec3::Z), None);
    assert_eq!(r.feature(0., -DVec3::X), Some(6));
    assert_eq!(r.feature(1., DVec3::X), Some(7));
    assert_eq!(r.feature(0., DVec3::X), None);
}

fn sweep(
    span: [DVec3; 2],
    wing: DVec3,
    other: Option<DVec3>,
    motion: DVec3,
    radius: f64,
    inverse_mass: f64,
) -> Result<Option<Row>, XpbdError> {
    let points = [span[0], span[1], wing, other.unwrap_or(wing)];
    let p = points.map(|pos| Particle {
        position: pos.to_array(),
        velocity: motion.to_array(),
        inverse_mass,
    });
    let edge = Edge {
        ends: [0, 1],
        wing: 2,
        other: other.map(|_| 3),
    };
    Ok(top_ridge()
        .sweep(
            edge,
            DQuat::IDENTITY,
            &p,
            &points.map(|pos| pos + motion),
            radius,
            &mut 0,
        )?
        .map(|(_, r)| r))
}

#[test]
fn coincident_parallel_edges_require_a_resolved_transverse_normal() {
    let middle = DVec3::Y + DVec3::Z;
    let span = [middle - 2. * DVec3::X, middle + 2. * DVec3::X];
    let wing = middle + DVec3::Z;
    let cast = |motion, radius| sweep(span, wing, None, motion, radius, 1.);
    assert_eq!(cast(-DVec3::Z, 0.).unwrap().unwrap().normal, DVec3::Z);
    assert!(cast(DVec3::Z, 0.).unwrap().is_none());
    assert!(cast(DVec3::X, 0.).unwrap().is_none());
    assert!(matches!(
        cast(-DVec3::Z, 0.01),
        Err(XpbdError::UnresolvedSurface)
    ));
    assert!(
        sweep(span, wing, None, -DVec3::Z, 0.01, 0.)
            .unwrap()
            .is_none()
    );
    let shorter = [middle - 0.5 * DVec3::X, middle + 0.5 * DVec3::X];
    assert!(
        sweep(shorter, wing, None, -DVec3::Z, 0.01, 1.)
            .unwrap()
            .is_none()
    );
}

#[test]
fn outward_normal_is_winding_independent_and_visible_to_both_wings() {
    let middle = DVec3::Y + DVec3::Z;
    let [a, b] = [middle - 0.4 * DVec3::Y, middle + 0.4 * DVec3::Y];
    let wing = middle + DVec3::Z;
    for span in [[a, b], [b, a]] {
        let row = sweep(span, wing, None, -DVec3::Z, 0., 1.).unwrap().unwrap();
        assert!(row.normal.distance(DVec3::Z) < 1e-14);
        assert!(DVec3::from(row.weights).distance(DVec3::new(0.5, 0.5, 0.)) < 1e-14);
        assert!(
            sweep(span, wing, Some(middle - DVec3::Z), -DVec3::Z, 0., 1.)
                .unwrap()
                .is_none()
        );
        assert!(
            sweep(span, wing, Some(wing), -DVec3::Z, 0., 1.)
                .unwrap()
                .is_some()
        );
    }
}
