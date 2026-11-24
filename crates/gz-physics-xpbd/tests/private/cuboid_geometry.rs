// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use parry3d_f64::{
    math::{Pose, Rotation, Vector},
    query::{NonlinearRigidMotion, cast_shapes_nonlinear},
    shape::{Ball, Cuboid, SharedShape},
};

fn native(s: &Sweep) -> Option<f64> {
    // Give both reference shapes the same additional world translation:
    // TOI depends on relative travel, not the absolute observer frame.
    let translation = Vector::new(3., -2., 1.);
    let common = Vector::new(0.17, 0.25, -0.6);
    let cube = NonlinearRigidMotion::new(
        Pose::from_parts(translation, Rotation::from_array(s.q0.to_array())),
        Vector::ZERO,
        common,
        Vector::from_array(s.angular_travel().to_array()),
    );
    let point = NonlinearRigidMotion::new(
        Pose::from_parts(
            translation + Vector::from_array(s.start.to_array()),
            Rotation::IDENTITY,
        ),
        Vector::ZERO,
        common + Vector::from_array((s.end - s.start).to_array()),
        Vector::ZERO,
    );
    let shape = match s.geometry {
        OrientedShape::Cuboid(h) => SharedShape::new(Cuboid::new(Vector::from_array(h.to_array()))),
        OrientedShape::Capsule(c) => SharedShape::capsule_z(c.half_length, c.radius),
    };
    cast_shapes_nonlinear(
        &cube,
        shape.as_ref(),
        &point,
        &Ball::new(s.radius),
        0.,
        1.,
        true,
    )
    .unwrap()
    .map(|h| h.time_of_impact)
}

#[test]
fn exact_face_edge_corner_and_near_miss_for_a_translating_tilted_box() {
    let q = DQuat::from_rotation_y(0.49) * DQuat::from_rotation_z(-0.37);
    let half = DVec3::new(0.5, 0.25, 0.125);
    let radius = 0.1;
    for (dy, dz) in [(0., 0.), (0.061, 0.), (0.031, 0.049), (0.079, 0.079)] {
        let y = if dy == 0. { 0.1 } else { half.y + dy };
        let z = if dz == 0. { -0.03 } else { half.z + dz };
        let s = Sweep {
            geometry: OrientedShape::Cuboid(half),
            radius,
            start: q * DVec3::new(-2., y, z),
            end: q * DVec3::new(2., y, z),
            q0: q,
            q1: q,
        };
        let hit = s.cast().unwrap();
        let oracle = native(&s);
        if dy * dy + dz * dz > radius * radius {
            assert!(hit.is_none() && oracle.is_none());
            continue;
        }
        let radial = (radius * radius - dy * dy - dz * dz).sqrt();
        let wanted = (2. - half.x - radial) / 4.;
        let hit = hit.unwrap();
        assert!((hit.time - wanted).abs() < 1e-9);
        assert!(
            (oracle.unwrap() - wanted).abs() < 1e-7,
            "{oracle:?} {wanted}"
        );
        assert!(
            hit.local_normal
                .distance(DVec3::new(-radial, dy, dz) / radius)
                < 1e-8
        );
    }
}

#[test]
fn rotating_bar_sweeps_through_a_stationary_particle_with_both_endpoints_clear() {
    let s = Sweep {
        geometry: OrientedShape::Cuboid(DVec3::new(1., 0.08, 0.14)),
        radius: 0.04,
        start: DVec3::X * 0.8,
        end: DVec3::X * 0.8,
        q0: DQuat::from_rotation_z(-0.4),
        q1: DQuat::from_rotation_z(0.4),
    };
    for p in [s.local(0.), s.local(1.)] {
        assert!(s.geometry.witness(p).0 > 0.22);
    }
    let expected = (0.4 - (0.12_f64 / 0.8).asin()) / 0.8;
    let hit = s.cast().unwrap().unwrap();
    assert!(
        (hit.time - expected).abs() < 1e-8 && hit.local_normal.distance(DVec3::Y) < 1e-8,
        "t={}",
        hit.time
    );
    let oracle = native(&s).unwrap();
    assert!((oracle - expected).abs() < 1e-6, "{oracle} {expected}");
    let no_hit = Sweep {
        start: DVec3::new(0.8, 0., 0.181),
        end: DVec3::new(0.8, 0., 0.181),
        ..s
    };
    assert!(no_hit.cast().unwrap().is_none() && native(&no_hit).is_none());
}

#[test]
fn inside_and_antipodal_orientation_are_deterministic() {
    let half = DVec3::new(1., 0.1, 0.1);
    let q = DQuat::from_rotation_z(0.2);
    let overlap = Sweep {
        geometry: OrientedShape::Cuboid(half),
        radius: 0.02,
        start: DVec3::ZERO,
        end: DVec3::X,
        q0: q,
        q1: -q,
    };
    let hit = overlap.cast().unwrap().unwrap();
    assert_eq!(hit.time, 0.);
    assert_eq!(hit.local_normal, DVec3::Y);
    assert_eq!(hit.local_surface, DVec3::Y * 0.1);
    assert!((hit.initial_overlap - 0.12).abs() < 1e-15);
}

#[test]
fn rotating_edge_and_corner_agree_with_intersections_of_their_circular_orbits() {
    let half = DVec3::new(1., 0.08, 0.14);
    let r = 0.04_f64;
    for (radial, dz) in [(0.8_f64, 0.025_f64), (1.032, 0.017)] {
        let xy = (r * r - dz * dz).sqrt();
        // Outside z, project the edge/corner contact onto the plane of rotation.
        let corner = half.x.hypot(half.y);
        let polar = if radial < half.x {
            ((half.y + xy) / radial).asin()
        } else {
            half.y.atan2(half.x)
                + ((radial * radial + corner * corner - xy * xy) / (2. * radial * corner)).acos()
        };
        let expected = (0.4 - polar) / 0.8;
        let point = DVec3::new(radial, 0., half.z + dz);
        let s = Sweep {
            geometry: OrientedShape::Cuboid(half),
            radius: r,
            start: point,
            end: point,
            q0: DQuat::from_rotation_z(-0.4),
            q1: DQuat::from_rotation_z(0.4),
        };
        let hit = s.cast().unwrap().unwrap();
        assert!((hit.time - expected).abs() < 1e-8);
        assert!((hit.local_normal.z - dz / r).abs() < 1e-7);
        assert_eq!(hit.local_normal.x > 0., radial > half.x);
        let oracle = native(&s);
        println!(
            "rotating radius {radial} analytic {expected:.10} cuboid {:.10} native {oracle:?}",
            hit.time
        );
        assert!(oracle.is_some_and(|t| (t - expected).abs() < 2e-6));
    }
}

#[test]
fn combined_translation_and_oblique_rotation_match_native_sweeps_across_features() {
    let mut hits = 0;
    let mut misses = 0;
    let mut max_error = 0.0_f64;
    for i in 0..96 {
        let f = i as f64;
        let half = DVec3::new(0.47, 0.21, 0.13);
        let angle = 0.27 + 0.39 * (1.71 * f).sin();
        let axis = DVec3::new((0.15 * f).sin(), 0.7, (0.36 * f).cos()).normalize();
        let q0 = DQuat::from_rotation_y(0.9 * (1.1 * f).cos())
            * DQuat::from_rotation_z(0.8 * (0.31 * f).sin());
        let q1 = DQuat::from_axis_angle(axis, angle) * q0;
        // Vary transverse offsets past the rounded Minkowski boundary as
        // well as through it. Each world-space particle path is a straight chord.
        let y = 0.28 * (f * 0.79).sin();
        let z = 0.19 * (f * 0.45).cos();
        let s = Sweep {
            geometry: OrientedShape::Cuboid(half),
            radius: 0.035,
            start: q0 * DVec3::new(-1., y, z),
            end: q1
                * DVec3::new(
                    1.1,
                    y + 0.035 * (0.33 * f).cos(),
                    z - 0.04 * (0.99 * f).sin(),
                ),
            q0,
            q1,
        };
        let ours = s.cast().unwrap().map(|hit| hit.time);
        let reference = native(&s);
        match (ours, reference) {
            (Some(a), Some(b)) => {
                hits += 1;
                max_error = max_error.max((a - b).abs());
            }
            (None, None) => misses += 1,
            (a, b) => panic!("case {i} proposed={a:?} native={b:?}"),
        }
    }
    println!("oblique 3D sweeps: {hits} hits, {misses} misses, max time difference {max_error:e}");
    assert!(hits > 30 && misses > 5 && max_error < 2e-6);
}

#[test]
fn tilted_tumbling_capsules_agree_with_rigid_nonlinear_casts() {
    let geometry = OrientedShape::Capsule(
        super::super::capsule::Capsule::new(DVec3::Z.to_array(), 0.52, 0.2, 0.).unwrap(),
    );
    let (mut hits, mut misses) = (0, 0);
    let mut max_error = 0f64;
    for i in 0..192 {
        let f = i as f64;
        let axis = DVec3::new((f * 0.25).cos(), 0.6, (f * 0.49).sin()).normalize();
        let q0 = DQuat::from_euler(
            glam::EulerRot::XYZ,
            0.6 * (0.67 * f).sin(),
            0.8 * (0.3 * f).cos(),
            (0.21 * f).sin(),
        );
        let q1 = DQuat::from_axis_angle(axis, 0.83 * (0.53 * f).cos()) * q0;
        let y = 0.27 * (0.43 * f).sin();
        let z = 0.78 * (0.79 * f).cos();
        let sweep = Sweep {
            geometry,
            radius: 0.025,
            start: q0 * DVec3::new(-1., y, z),
            end: q1
                * DVec3::new(
                    1.,
                    y + 0.035 * (0.11 * f).sin(),
                    z + 0.08 * (0.38 * f).cos(),
                ),
            q0,
            q1,
        };
        match (sweep.cast().unwrap(), native(&sweep)) {
            (Some(hit), Some(toi)) => {
                hits += 1;
                max_error = max_error.max((hit.time - toi).abs());
            }
            (None, None) => misses += 1,
            (a, b) => panic!("different contact in case {i}: {a:?} / {b:?}"),
        }
    }
    println!(
        "tumbling Parry CCD: {hits} impacts, {misses} misses, max normalized TOI error {max_error:e}"
    );
    assert!(hits > 60 && misses > 20 && max_error < 3e-6);
}

#[test]
fn axial_capsule_spin_moves_material_and_preserves_the_geometric_tangent() {
    let c = crate::capsule::Capsule::new(DVec3::Z.to_array(), 0.52, 0.2, 0.).unwrap();
    let q = DQuat::from_euler(glam::EulerRot::YXZ, 0.67, -0.18, 1.43);
    for radius in [0., 0.025] {
        let s = Sweep {
            geometry: OrientedShape::Capsule(c),
            radius,
            start: q * DVec3::new(0.4, 0., 0.37),
            end: q * DVec3::new(0.1, 0., 0.37),
            q0: q,
            q1: q * DQuat::from_rotation_z(2.63),
        };
        assert!(!s.shape_rotates(s.angular_travel()));
        let hit = s.impact().unwrap().unwrap();
        let expected = (0.4 - 0.2 - radius) / 0.3;
        assert!((hit.time - expected).abs() < 1e-12);
        let (n, p) = s.end_witness(&hit);
        assert!(n.distance(q * DVec3::X) < 1e-12);
        assert!(p.distance(q * DVec3::new(0.2, 0., 0.37)) < 1e-12);
    }
}

#[test]
fn zero_radius_point_with_a_tumbling_capsule_distinguishes_contact_from_an_exact_tip_tangent() {
    let c = crate::capsule::Capsule::new(DVec3::Z.to_array(), 0.52, 0.2, 0.).unwrap();
    for (offset, hit) in [(-1e-6, true), (0., false), (1e-6, false)] {
        let x = DVec3::Z * (c.half_length + c.radius + offset);
        let s = Sweep {
            geometry: OrientedShape::Capsule(c),
            radius: 0.,
            start: x,
            end: x,
            q0: DQuat::from_rotation_y(-0.5),
            q1: DQuat::from_rotation_y(0.5),
        };
        assert_eq!(s.impact().unwrap().is_some(), hit, "offset {offset}");
    }
}
