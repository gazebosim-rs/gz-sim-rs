// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use SlipCatchState::*;

const SAMPLE: Duration = Duration::from_millis(1);

#[test]
fn rejects_invalid_thresholds_or_actuator_limits_before_arming() {
    let defaults = SlipCatchConfig::default();
    for (config, force, height) in [
        (defaults, 0., f64::NAN),
        (defaults, f64::NAN, 0.),
        (defaults, -1., 0.),
        (defaults, 4.201, 0.),
        (
            SlipCatchConfig {
                max_force_n: f64::INFINITY,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                max_force_n: 0.,
                ..defaults
            },
            0.,
            0.,
        ),
        (
            SlipCatchConfig {
                force_rate_n_s: f64::NAN,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                force_rate_n_s: 0.,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                trigger_drop_m: f64::INFINITY,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                trigger_drop_m: 0.,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                trigger_speed_m_s: f64::NAN,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                settled_speed_m_s: f64::NAN,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                settled_speed_m_s: -0.001,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                trigger_speed_m_s: defaults.settled_speed_m_s,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                confirm_time: Duration::ZERO,
                ..defaults
            },
            1.,
            0.,
        ),
        (
            SlipCatchConfig {
                settle_time: Duration::ZERO,
                ..defaults
            },
            1.,
            0.,
        ),
    ] {
        assert!(SlipCatch::arm(config, force, height).is_none());
    }
}

#[test]
fn drop_and_speed_must_persist_and_pausing_freezes_the_sensor_history() {
    let config = SlipCatchConfig {
        confirm_time: 2 * SAMPLE,
        ..SlipCatchConfig::default()
    };
    let mut catch = SlipCatch::arm(config, 2., 0.).unwrap();
    assert_eq!(catch.observe(SAMPLE, -0.00004), Monitoring); // Fast but too shallow.
    assert_eq!(catch.observe(SAMPLE, -0.00020), Monitoring);
    assert_eq!(catch.observe(SAMPLE, -0.00020), Monitoring); // Drops out of speed gate.
    assert_eq!(catch.observe(SAMPLE, -0.00024), Monitoring);
    assert_eq!(catch.observe(Duration::ZERO, f64::NAN), Monitoring);
    assert_eq!(catch.force_n(), 2.);
    assert_eq!(catch.observe(SAMPLE, -0.00028), Tightening);
    assert!((catch.force_n() - 2.12).abs() < 1e-12);
}

#[test]
fn force_saturates_while_slipping_and_rearms_from_the_reseated_height() {
    let config = SlipCatchConfig {
        settle_time: 2 * SAMPLE,
        max_force_n: 2.2,
        ..SlipCatchConfig::default()
    };
    let mut catch = SlipCatch::arm(config, 2., 0.).unwrap();
    assert_eq!(catch.observe(SAMPLE, -0.0010), Tightening);
    assert_eq!(catch.observe(SAMPLE, -0.0011), AtLimit);
    assert_eq!(catch.force_n(), 2.2);
    assert_eq!(catch.observe(SAMPLE, -0.0013), AtLimit);
    assert_eq!(catch.observe(SAMPLE, -0.0013), Tightening);
    assert_eq!(catch.observe(SAMPLE, -0.0011), Tightening); // Rebounds before settling.
    assert_eq!(catch.observe(SAMPLE, -0.0011), Tightening);
    assert_eq!(catch.observe(SAMPLE, -0.0011), Caught);
    assert_eq!(catch.observe(SAMPLE, -0.00115), Caught); // Below new displacement gate.
    assert_eq!(catch.observe(SAMPLE, -0.00125), AtLimit);
    assert_eq!(catch.force_n(), 2.2);
    catch.release();
    assert_eq!(catch.force_n(), 0.);
    assert_eq!(catch.observe(SAMPLE, -0.01), Released);
}

#[test]
fn invalid_derivative_or_baseline_latches_force_until_explicit_release() {
    for (baseline, next) in [(0., f64::NAN), (0., f64::INFINITY), (f64::MAX, -f64::MAX)] {
        let mut catch = SlipCatch::arm(SlipCatchConfig::default(), 1., baseline).unwrap();
        assert_eq!(catch.state(), Monitoring);
        assert_eq!(catch.observe(SAMPLE, next), SensorFault);
        assert_eq!(catch.observe(SAMPLE, 0.), SensorFault);
        assert_eq!(catch.force_n(), 1.);
        catch.release();
        assert_eq!(catch.state(), Released);
    }
    // Individually representable slower samples can still overflow their net displacement.
    let mut catch = SlipCatch::arm(SlipCatchConfig::default(), 1., f64::MAX).unwrap();
    assert_eq!(catch.observe(Duration::MAX, 0.), AtLimit);
    assert_eq!(catch.observe(Duration::MAX, -f64::MAX), SensorFault);
}

#[test]
fn command_slew_tracks_elapsed_sensor_time_and_stops_growing_on_rebound() {
    let sample = Duration::from_micros(200);
    let mut fine = SlipCatch::arm(SlipCatchConfig::default(), 2., 0.).unwrap();
    let mut coarse = SlipCatch::arm(SlipCatchConfig::default(), 2., 0.).unwrap();
    for catch in [&mut fine, &mut coarse] {
        assert_eq!(catch.observe(SAMPLE, -0.001), Tightening);
    }
    for i in 1..=5 {
        fine.observe(sample, -0.001 - 0.0001 * f64::from(i));
    }
    coarse.observe(5 * sample, -0.0015);
    let force = fine.force_n();
    assert!((force - coarse.force_n()).abs() < 1e-12);
    assert_eq!(fine.observe(sample, -0.001), Tightening);
    assert_eq!(fine.force_n(), force);
}
