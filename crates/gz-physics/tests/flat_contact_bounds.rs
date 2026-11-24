// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::sole_contact::{SoleBody, SoleDeck, SoleError, SoleMaterial, SoleState};

fn approaching() -> (SoleBody, SoleDeck, SoleState) {
    let body = SoleBody {
        mass: 2.,
        inertia: DVec3::splat(0.04),
        half_extents: DVec3::new(0.25, 0.25, 0.125),
        foot_offset: DVec3::ZERO,
    };
    let deck = SoleDeck {
        center: DVec3::new(0.25, -0.25, 0.),
        orientation: DQuat::IDENTITY,
        half_extents: DVec3::new(1., 1., 0.125),
        velocity: DVec3::new(0.25, -0.5, 0.25),
    };
    let initial = SoleState {
        center: deck.center + DVec3::Z * 0.25,
        orientation: DQuat::from_rotation_z(0.5),
        velocity: deck.velocity - DVec3::Z * 0.5,
        spin: DVec3::ZERO,
    };
    (body, deck, initial)
}

#[test]
fn moving_deck_catches_and_releases_the_sole_with_the_impulse_from_momentum() {
    let (body, deck, initial) = approaching();
    // Equal layers in series and 0.25 m² of contact give 200 N/m.
    let mat = SoleMaterial::new(1600., 1600., 0., 0.5, 0.01).unwrap();
    for (phase, compressed, closing, impulse) in [
        (0.5_f64, 0.05_f64, 0., 1.),
        (1.25, -0.5 * std::f64::consts::PI / 40., -0.5, 2.),
    ] {
        let t = phase * std::f64::consts::PI / 10.;
        let mut state = initial;
        let step = body
            .try_flat_elastic_step(
                &mut state,
                &deck,
                &mat,
                DVec3::ZERO,
                t,
                DVec3::ZERO,
                DVec3::ZERO,
            )
            .unwrap()
            .unwrap();
        let expected = initial.center + t * deck.velocity - compressed * DVec3::Z;
        assert!(state.center.distance(expected) < 1e-14);
        assert!(state.velocity.distance(deck.velocity - closing * DVec3::Z) < 1e-14);
        assert_eq!(state.orientation, initial.orientation);
        assert!((step.normal_impulse - impulse).abs() < 1e-13);
        assert!(
            step.impulse
                .distance(body.mass * (state.velocity - initial.velocity))
                < 1e-13
        );
        assert!((step.peak_normal_force - 10.).abs() < 1e-13);
    }
}

#[test]
fn over_travel_is_atomic_and_off_center_or_sliding_soles_reject_the_vertical_model() {
    let (body, deck, init) = approaching();
    let mat = SoleMaterial::new(1600., 1600., 0., 0.5, 0.01).unwrap();
    let solve = |s: &mut SoleState, torque| {
        body.try_flat_elastic_step(s, &deck, &mat, DVec3::ZERO, 0.16, DVec3::ZERO, torque)
    };
    let assert_unchanged = |s: SoleState, before: SoleState| {
        assert_eq!(s.center, before.center);
        assert_eq!(s.velocity, before.velocity);
        assert_eq!(s.orientation, before.orientation);
        assert_eq!(s.spin, before.spin);
    };
    let mut fast = SoleState {
        velocity: deck.velocity - 2. * DVec3::Z,
        ..init
    };
    let before = fast;
    assert!(matches!(
        solve(&mut fast, DVec3::ZERO),
        Err(SoleError::OutsideCoating)
    ));
    assert_unchanged(fast, before);
    for (offset, relative_drift, torque) in [
        (DVec3::X, DVec3::ZERO, DVec3::ZERO),
        (DVec3::ZERO, DVec3::Y, DVec3::ZERO),
        (DVec3::ZERO, DVec3::ZERO, DVec3::Y),
    ] {
        let before = SoleState {
            center: init.center + offset,
            velocity: init.velocity + relative_drift,
            ..init
        };
        let mut state = before;
        assert!(solve(&mut state, torque).unwrap().is_none());
        assert_unchanged(state, before);
    }
}
