// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn sample() -> PlacementReading {
    PlacementReading {
        pad_height_m: 0.31,
        pad_velocity_m_s: 0.,
        parcel_velocity_m_s: 0.,
        receiver_impulse_ns: Some(0.001 * 0.2 * 9.81),
        obstacle_impulse_ns: 0.,
        deck_margin_m: 0.005,
        jaw_clearance_m: 0.,
        jaw_pressure_n: [4.2; 2],
    }
}
fn establish_guided_approach(c: &mut RecoveryPlacement) {
    let mut o = sample();
    o.receiver_impulse_ns = Some(0.);
    assert_eq!(
        c.advance(Duration::from_millis(5), o).phase,
        PlacementPhase::Lower
    );
}
#[test]
fn lowering_opening_retreat_and_completion_use_separate_observations() {
    let dt = Duration::from_millis(1);
    let mut controller = RecoveryPlacement::new(0.2, 0.56, 0.3, 0.045).unwrap();
    establish_guided_approach(&mut controller);
    let mut o = sample();
    for _ in 0..19 {
        assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Lower);
    }
    assert_eq!(
        controller.advance(Duration::ZERO, o).phase,
        PlacementPhase::Lower
    );
    assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Open);
    assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Open);
    o.jaw_clearance_m = 0.002;
    o.jaw_pressure_n = [0.; 2];
    assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Retreat);
    for _ in 0..30 {
        assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Retreat);
    }
    o.pad_height_m += 0.011;
    for _ in 0..19 {
        assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Retreat);
    }
    assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Placed);
    o.receiver_impulse_ns = None;
    for _ in 0..3 {
        assert_eq!(controller.advance(dt, o).phase, PlacementPhase::Placed);
    }
    assert_eq!(controller.advance(dt, o).phase, PlacementPhase::SupportLost);
    assert_eq!(
        controller.advance(dt, sample()).phase,
        PlacementPhase::SupportLost
    );
}
#[test]
fn rejects_invalid_configuration_and_latches_a_bounded_search() {
    assert!(RecoveryPlacement::new(-0.2, 0.56, 0.3, 0.08).is_none());
    assert!(RecoveryPlacement::new(0.2, 0.56, f64::NAN, 0.08).is_none());
    let mut c = RecoveryPlacement::new(0.2, 0.56, 0.3, 0.08).unwrap();
    let mut o = sample();
    o.receiver_impulse_ns = Some(0.);
    o.pad_height_m = 0.3;
    let dt = Duration::from_millis(1);
    for _ in 0..29 {
        assert_eq!(c.advance(dt, o).phase, PlacementPhase::Lower);
    }
    assert_eq!(c.advance(dt, o).phase, PlacementPhase::Hold);
    assert_eq!(c.advance(dt, sample()).phase, PlacementPhase::Hold);
}

#[test]
fn pause_keeps_force_and_invalid_carriage_breaks_release_dwell() {
    let mut c = RecoveryPlacement::new(0.2, 0.56, 0.3, 0.08).unwrap();
    let dt = Duration::from_millis(1);
    establish_guided_approach(&mut c);
    let o = sample();
    let a = c.advance(
        Duration::from_millis(19),
        PlacementReading {
            receiver_impulse_ns: Some(0.019 * 0.2 * 9.81),
            ..o
        },
    );
    let mut bad = o;
    bad.pad_velocity_m_s = f64::NAN;
    bad.pad_height_m = f64::NAN;
    assert_eq!(c.advance(Duration::ZERO, bad).lift_force_n, a.lift_force_n);
    assert_eq!(c.advance(dt, bad).lift_force_n, a.lift_force_n);
    assert_eq!(c.advance(dt, o).phase, PlacementPhase::Lower);
    let mut blind = o;
    blind.receiver_impulse_ns = None;
    blind.pad_height_m = 0.30;
    assert_eq!(
        c.advance(Duration::from_millis(30), blind).phase,
        PlacementPhase::Hold
    );
}
#[test]
fn losing_support_before_clearance_recloses_the_fingers() {
    let mut c = RecoveryPlacement::new(0.2, 0.56, 0.3, 0.08).unwrap();
    establish_guided_approach(&mut c);
    let mut o = sample();
    assert_eq!(
        c.advance(
            Duration::from_millis(20),
            PlacementReading {
                receiver_impulse_ns: Some(0.020 * 0.2 * 9.81),
                ..o
            }
        )
        .phase,
        PlacementPhase::Open
    );
    o.receiver_impulse_ns = None;
    let hold = c.advance(Duration::from_millis(4), o);
    assert_eq!(hold.phase, PlacementPhase::Hold);
    assert!(hold.jaw_force_n > 0.);
}

#[test]
fn a_free_landing_cannot_be_credited_as_a_guided_placement() {
    let mut c = RecoveryPlacement::new(0.2, 0.56, 0.3, 0.08).unwrap();
    establish_guided_approach(&mut c);
    let dt = Duration::from_millis(1);
    let mut o = sample();
    o.jaw_pressure_n = [0.; 2];
    o.parcel_velocity_m_s = -0.6;
    o.receiver_impulse_ns = Some(0.);
    c.advance(dt, o);
    o.receiver_impulse_ns = Some(0.2);
    assert_eq!(c.advance(dt, o).phase, PlacementPhase::UnguidedLanding);
    let stopped = c.advance(dt, sample());
    assert_eq!(stopped.phase, PlacementPhase::UnguidedLanding);
    assert_eq!(stopped.jaw_force_n, 0.);
}

#[test]
fn a_second_touchdown_after_a_blind_flight_requires_a_new_guided_approach() {
    let mut c = RecoveryPlacement::new(0.2, 0.56, 0.3, 0.08).unwrap();
    establish_guided_approach(&mut c);
    let dt = Duration::from_millis(1);
    assert_eq!(c.advance(dt, sample()).phase, PlacementPhase::Lower);
    let flight = PlacementReading {
        jaw_pressure_n: [0.; 2],
        parcel_velocity_m_s: -0.08,
        receiver_impulse_ns: Some(0.),
        ..sample()
    };
    assert_eq!(
        c.advance(Duration::from_millis(4), flight).phase,
        PlacementPhase::Lower
    );
    assert_eq!(
        c.advance(dt, sample()).phase,
        PlacementPhase::UnguidedLanding
    );
}
