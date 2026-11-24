// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{IsotropicFriction, MaterialLinkMotion, Restitution};
use gz_physics::{BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};

// The raised corner arrives exactly at 50 microseconds in ballistic flight.
fn upward() -> (RapierBackend, UpperPair) {
    let mut w = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let q = Rotation::from_rotation_y(0.2) * Rotation::from_rotation_x(0.3);
    let half = Vector::new(0.18, 0.1, 0.06);
    let top = (q * Vector::new(-half.x, half.y, half.z)).z;
    let at = 50e-6;
    let gap = at - 0.5 * 9.81 * at * at;
    let mut add = |half: Vector, z: f64, vz: f64, m, q: Rotation| {
        w.spawn_material_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: half.to_array(),
                },
                position: [0., 0., z],
                velocity: [0., 0., vz],
                mass: m,
            },
            RotationSpec {
                orientation: q.to_array(),
                angular_velocity: [0.; 3],
            },
            m.map(|_| Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [0.05, 0.05, 0.05, 0., 0., 0.],
            }),
            MaterialLinkMotion::Ordinary,
            IsotropicFriction::new(0.).unwrap(),
        )
        .unwrap()
    };
    let f = add(half, 0.5, 1., Some(2.), q);
    let floor = add(Vector::new(1., 1., 0.05), 0., 0., None, Rotation::IDENTITY);
    let roof = add(
        Vector::new(1., 1., 0.05),
        0.5 + top + gap + 0.05,
        0.,
        None,
        Rotation::IDENTITY,
    );
    let pair = UpperPair::new(&w, (f, floor, roof)).unwrap();
    (w, pair)
}

#[test]
fn shortened_upper_impact_commits_only_the_measured_impulse_and_elapsed_gravity() {
    let (mut w, pair) = upward();
    let before = w.state(pair.ids.0).unwrap();
    assert!(
        pair.shorten(&w, Duration::ZERO, Duration::from_micros(49), None)
            .is_none()
    );
    let (duration, step) = pair
        .shorten(&w, Duration::ZERO, Duration::from_micros(200), None)
        .unwrap();
    assert!((50_000..=50_003).contains(&duration.as_nanos()));
    assert_eq!(w.state(pair.ids.0), Some(before));
    let hit = step.receipt.unwrap();
    assert!((hit.point.after_cut_s - 50e-6).abs() < 1e-11);
    assert!(hit.minimum_other_gap_m > 0. && hit.minimum_footprint_margin_m > 0.);
    assert_eq!(hit.point.impact.support, pair.ids.2);
    assert!(hit.point.impact.impulse_on_foot[2] < 0.);
    step.commit(&mut w);
    let after = w.state(pair.ids.0).unwrap();
    let applied = 2.
        * (Vector::from(after.velocity) - Vector::from(before.velocity)
            + 9.81 * duration.as_secs_f64() * Vector::Z);
    assert!((applied - Vector::from(hit.point.impact.impulse_on_foot)).length() < 1e-11);
    let prior = Some(PriorUpper::from(hit));
    assert!(
        pair.shorten(&w, duration, Duration::from_micros(200), prior)
            .is_none()
    );
}

#[test]
fn rebound_material_declines_lower_projection_without_changing_foot_state() {
    let (mut w, pair) = upward();
    w.set_initial_restitution(pair.ids.1, Restitution::new(0.5).unwrap())
        .unwrap();
    let i = pair.incoming(&w).unwrap();
    assert!(pair.for_lower(&w, i, i.state, 0.).is_none());
    assert!(
        pair.shorten(
            &w,
            Duration::ZERO,
            Duration::from_micros(200),
            Some(PriorUpper {
                corner: 0,
                time: 0.
            })
        )
        .is_none()
    );
}

fn moving_lower() -> (RapierBackend, UpperPair) {
    let (mut w, pair) = upward();
    w.world.gravity = Vector::ZERO;
    let incoming = pair.incoming(&w).unwrap();
    let bottom = incoming
        .arms
        .iter()
        .map(|arm| (incoming.state.com + incoming.state.q * (*arm)).z)
        .fold(f64::INFINITY, f64::min);
    let pad = &mut w.world.bodies[pair.c.pad];
    pad.set_body_type(RigidBodyType::KinematicVelocityBased, true);
    pad.set_translation(Vector::new(0., 0., bottom - 100e-6 - 0.05), true);
    pad.set_linvel(1.5 * Vector::Z, true);
    let pair = UpperPair::new(&w, pair.ids).unwrap();
    (w, pair)
}

#[test]
fn short_upper_and_lower_strikes_match_two_independently_committed_intervals() {
    let (mut w, pair) = moving_lower();
    let initial = w.state(pair.ids.0).unwrap();
    let h = Duration::from_millis(1);
    let (total, joined) = pair.shorten(&w, Duration::ZERO, h, None).unwrap();
    let first = joined.receipt.unwrap();
    let lower = *joined.returned.as_ref().unwrap().as_ref();
    assert!(first.point.after_cut_s > 49e-6 && first.point.after_cut_s < 51e-6);
    assert!(total > Duration::from_micros(240) && total < Duration::from_micros(250));
    assert_eq!(lower.previous_upper_s, first.point.after_cut_s);
    assert!(lower.previous_upper_gaps_m[1] > 0.);
    let j = Vector::from(first.point.impact.impulse_on_foot)
        + Vector::from(lower.lower.point.impact.impulse_on_foot);
    assert!(
        first.point.impact.impulse_on_foot[2] < 0.
            && lower.lower.point.impact.impulse_on_foot[2] > 0.
    );
    joined.commit(&mut w);
    let end = w.state(pair.ids.0).unwrap();
    assert!(
        (2. * (Vector::from(end.velocity) - Vector::from(initial.velocity)) - j).length() < 1e-11
    );

    let (mut separate, pair) = moving_lower();
    let (dt, upper) = pair
        .shorten(&separate, Duration::ZERO, Duration::from_micros(100), None)
        .unwrap();
    let prior = PriorUpper::from(upper.receipt.unwrap());
    assert!(upper.returned.is_none());
    upper.commit(&mut separate);
    let pad = &mut separate.world.bodies[pair.c.pad];
    pad.set_translation(pad.translation() + pad.linvel() * dt.as_secs_f64(), true);
    let (tail, step) = pair.shorten(&separate, dt, h - dt, Some(prior)).unwrap();
    assert_eq!(dt + tail, total);
    assert_eq!(
        step.returned
            .as_ref()
            .unwrap()
            .lower
            .point
            .impact
            .impulse_on_foot,
        lower.lower.point.impact.impulse_on_foot
    );
    step.commit(&mut separate);
    assert_eq!(separate.state(pair.ids.0), Some(end));
}
