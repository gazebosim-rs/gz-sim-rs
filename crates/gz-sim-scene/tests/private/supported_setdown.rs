// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn requires_sustained_load_and_holds_the_opening_clock_on_lost_support() {
    assert!(SupportedSetdown::new(0.).is_none());
    assert!(SupportedSetdown::new(f64::NAN).is_none());
    let mut c = SupportedSetdown::new(2.).unwrap();
    for _ in 0..19 {
        assert_eq!(c.advance(0.002).unwrap().opening_ms, 0);
    }
    for bad in [f64::NAN, -0.1, f64::INFINITY] {
        assert!(c.advance(bad).is_none());
    }
    assert_eq!(c.advance(0.002).unwrap().opening_ms, 1);
    assert!(!c.advance(0.).unwrap().advancing_opening);
    for _ in 0..19 {
        assert_eq!(c.advance(0.002).unwrap().opening_ms, 1);
    }
    for t in 2..=350 {
        assert_eq!(c.advance(0.002).unwrap().opening_ms, t);
    }
    let done = c.advance(0.).unwrap();
    assert_eq!(done.opening_ms, 350);
    assert!(!done.advancing_opening);
}
#[test]
fn search_is_bounded_and_a_high_impact_cannot_authorize_release() {
    let mut c = SupportedSetdown::new(1.).unwrap();
    for _ in 0..40 {
        assert_eq!(c.advance(0.003).unwrap().opening_ms, 0);
    }
    for _ in 0..1700 {
        c.advance(0.).unwrap();
    }
    for impulse in [0., 1.] {
        let stop = c.advance(impulse).unwrap();
        assert!(stop.search_exhausted);
        assert_eq!(stop.opening_ms, 0);
        assert_eq!(stop.lift_offset_m, -0.004);
    }
}

#[test]
fn signed_receiver_impulses_preserve_weight_confirmation_and_downward_load_pauses_release() {
    let mut support = SupportedSetdown::new(1.).unwrap();
    let mut zero = SupportedSetdown::new(1.).unwrap();
    for _ in 0..21 {
        assert_eq!(
            zero.advance(0.001),
            support.advance_vertical_reaction(0.001)
        );
    }
    let expect = zero.advance(0.).unwrap();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(support.advance_vertical_reaction(value).is_none());
    }
    let negative = support.advance_vertical_reaction(-0.0002).unwrap();
    assert_eq!(negative, expect);
    assert!(!negative.advancing_opening);
    assert!(support.advance(-0.0002).is_none());
}
