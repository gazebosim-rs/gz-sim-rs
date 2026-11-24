// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use RecoveryState::*;

const PAIR: Option<[f64; 2]> = Some([3., 4.]);
const LOW: Option<[f64; 2]> = Some([3., 0.]);
const MS: Duration = Duration::from_millis(1);

#[test]
fn missing_samples_do_not_invent_loss_load_or_unmeasured_travel() {
    let mut g = ContactRecovery::default();
    assert_eq!(g.tactile_travel_m(), Some(0.));
    assert_eq!(g.observe(MS, PAIR, Some(-1.)), Held);
    assert_eq!(g.tactile_travel_m(), Some(-0.001));
    for (normal, speed) in [
        (None, Some(-1.)),
        (Some([f64::NAN, 1.]), None),
        (Some([1., -1.]), None),
        (PAIR, Some(f64::NAN)),
    ] {
        assert_eq!(g.observe(MS, normal, speed), Uncertain);
        assert_eq!(g.tactile_travel_m(), None);
        assert_eq!(g.squeeze_n(), 4.2);
    }
    assert_eq!(g.observe(MS, PAIR, Some(-1.)), Held);
    assert_eq!(g.tactile_travel_m(), Some(0.));
    assert_eq!(g.observe(MS, PAIR, Some(-1.)), Held);
    assert_eq!(g.tactile_travel_m(), Some(-0.001));
}

#[test]
fn loss_debounce_and_settle_dwell_use_only_contiguous_advancing_time() {
    let mut g = ContactRecovery::default();
    let half = MS / 2;
    assert_eq!(g.observe(half, LOW, None), ConfirmingLoss);
    assert_eq!(g.observe(MS, None, None), Uncertain);
    assert_eq!(g.observe(half, LOW, None), ConfirmingLoss);
    assert_eq!(g.observe(Duration::ZERO, PAIR, Some(0.)), ConfirmingLoss);
    assert_eq!(g.observe(half, LOW, None), Reclosing);
    assert!((g.squeeze_n() - 4.25).abs() < 1e-12);
    assert_eq!(g.observe(MS * 2, PAIR, Some(0.)), Settling);
    assert_eq!(g.observe(MS, PAIR, Some(-0.006)), Settling);
    assert_eq!(g.observe(MS * 2, PAIR, Some(0.)), Settling);
    assert_eq!(g.observe(MS, PAIR, Some(0.)), Held);
    assert_eq!(g.observe(MS, LOW, None), Reclosing);
}

#[test]
fn ceiling_and_wall_deadline_survive_missing_packets_or_unsettled_return() {
    let mut g = ContactRecovery::default();
    assert_eq!(g.observe(MS * 10, LOW, None), Reclosing);
    assert_eq!(g.observe(MS * 10, LOW, None), Reclosing);
    assert_eq!(g.squeeze_n(), 5.2);
    assert_eq!(g.observe(MS * 3, None, None), Uncertain);
    assert_eq!(g.observe(MS, PAIR, Some(0.03)), Lower);
    assert_eq!(g.observe(MS * 10, PAIR, Some(0.)), Lower);
    let mut blank = ContactRecovery::default();
    assert_eq!(blank.observe(MS * 24, None, None), Lower);
    assert_eq!(blank.squeeze_n(), 4.2);
}

#[test]
fn declared_grasp_limits_are_validated_and_applied() {
    let settings = ContactRecoveryConfig {
        min_load_n: 2.,
        loss_dwell: MS * 2,
        settle_speed_m_s: 0.01,
        settle_dwell: MS,
        timeout: MS * 5,
        squeeze_rate_n_s: 200.,
        squeeze_limit_n: 8.,
    };
    let mut g = ContactRecovery::arm(settings, 6.).unwrap();
    assert_eq!(g.observe(MS, Some([1.9, 3.]), None), ConfirmingLoss);
    assert_eq!(g.observe(MS, Some([1.9, 3.]), None), Reclosing);
    assert_eq!(g.squeeze_n(), 6.2);
    assert_eq!(g.observe(MS, PAIR, Some(0.008)), Held);
    for invalid in [
        ContactRecoveryConfig {
            min_load_n: 0.,
            ..settings
        },
        ContactRecoveryConfig {
            squeeze_rate_n_s: -1.,
            ..settings
        },
        ContactRecoveryConfig {
            squeeze_limit_n: f64::INFINITY,
            ..settings
        },
        ContactRecoveryConfig {
            settle_speed_m_s: f64::NAN,
            ..settings
        },
        ContactRecoveryConfig {
            settle_speed_m_s: -1.,
            ..settings
        },
        ContactRecoveryConfig {
            loss_dwell: Duration::ZERO,
            ..settings
        },
        ContactRecoveryConfig {
            settle_dwell: Duration::ZERO,
            ..settings
        },
        ContactRecoveryConfig {
            timeout: MS * 2,
            ..settings
        },
    ] {
        assert!(ContactRecovery::arm(invalid, 6.).is_none());
    }
    for force in [f64::NAN, -1., 8.01] {
        assert!(ContactRecovery::arm(settings, force).is_none());
    }
}
