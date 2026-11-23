// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{Simulation, StepError, UpdateInfo};
use std::time::Duration;
#[test]
fn restored_exact_paused_and_reset_boundaries_keep_their_next_clock() {
    let h = Duration::from_nanos(1234567);
    let mut source = Simulation::new(h).unwrap();
    source.reset().unwrap();
    source.step_exact(3).unwrap();
    for i in 0..4 {
        let info = *source.world().resource::<UpdateInfo>();
        let mut restored = Simulation::from_committed_clock(h, info, source.is_paused()).unwrap();
        assert_eq!(restored.elapsed(), source.elapsed());
        assert_eq!(restored.epoch(), source.epoch());
        assert_eq!(*restored.world().resource::<UpdateInfo>(), info);
        assert_eq!(restored.tick().unwrap(), source.tick().unwrap());
        if i == 1 {
            source.set_paused(false);
            source.reset().unwrap();
        }
    }
}
#[test]
fn inconsistent_time_iteration_dt_and_pause_are_refused_before_world_creation() {
    let h = Duration::from_nanos(7);
    let v = UpdateInfo {
        start_time: h,
        sim_time: h * 2,
        dt: h,
        iteration: 2,
        epoch: 3,
        paused: false,
    };
    for invalid in [
        UpdateInfo {
            start_time: h * 2,
            ..v
        },
        UpdateInfo { iteration: 3, ..v },
        UpdateInfo { paused: true, ..v },
        UpdateInfo {
            start_time: v.sim_time,
            dt: Duration::ZERO,
            ..v
        },
        UpdateInfo {
            dt: h * 2,
            start_time: Duration::ZERO,
            ..v
        },
        UpdateInfo {
            sim_time: Duration::MAX,
            start_time: Duration::MAX,
            ..v
        },
    ] {
        assert!(matches!(
            Simulation::from_committed_clock(h, invalid, true),
            Err(StepError::InvalidClock)
        ));
    }
    assert!(matches!(
        Simulation::from_committed_clock(Duration::ZERO, v, true),
        Err(StepError::ZeroDuration)
    ));
    let uncommitted = UpdateInfo {
        start_time: Duration::ZERO,
        sim_time: Duration::ZERO,
        dt: Duration::ZERO,
        iteration: 0,
        epoch: 0,
        paused: false,
    };
    assert!(matches!(
        Simulation::from_committed_clock(h, uncommitted, false),
        Err(StepError::InvalidClock)
    ));
    let huge = UpdateInfo {
        iteration: u64::MAX,
        ..v
    };
    assert!(matches!(
        Simulation::from_committed_clock(Duration::MAX, huge, true),
        Err(StepError::InvalidClock)
    ));
}
