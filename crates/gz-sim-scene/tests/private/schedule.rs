// Copyright (C) 2017, 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn original_next_data_update_time_assertions() {
    let seconds = Duration::from_secs;
    let mut schedule = SampleSchedule::new(1.0).unwrap();
    assert!(schedule.take_sample(seconds(0), false).unwrap());
    assert_eq!(schedule.next, seconds(1));
    assert!(schedule.take_sample(seconds(1), false).unwrap());
    assert_eq!(schedule.next, seconds(2));
    schedule.next = seconds(5);
    assert!(!schedule.take_sample(seconds(2), false).unwrap());
    assert_eq!(schedule.next, seconds(5));
    assert!(schedule.take_sample(seconds(3), true).unwrap());
    assert_eq!(schedule.next, seconds(5));
    assert!(schedule.take_sample(seconds(5), false).unwrap());
    assert_eq!(schedule.next, seconds(6));
    schedule.next = seconds(1);
    assert!(schedule.take_sample(seconds(5), false).unwrap());
    assert_eq!(schedule.next, seconds(6));
    schedule = SampleSchedule::new(100.0).unwrap();
    let now = seconds(1_700_000_000);
    assert!(schedule.take_sample(now, false).unwrap());
    assert!(schedule.next > now);
    assert!(schedule.next <= now + Duration::from_millis(10));
    schedule = SampleSchedule::new(2000.0).unwrap();
    assert!(schedule.take_sample(Duration::ZERO, false).unwrap());
    let period = schedule.period.unwrap();
    assert!(period > Duration::ZERO);
    assert_eq!(schedule.next, period);
    assert_eq!(schedule.next, Duration::from_micros(500));
}

#[test]
fn off_grid_catchup_preserves_phase_and_checked_failure_preserves_state() {
    let mut schedule = SampleSchedule::new(100.0).unwrap();
    assert!(
        schedule
            .take_sample(Duration::from_millis(35), false)
            .unwrap()
    );
    assert_eq!(schedule.next, Duration::from_millis(40));
    assert!(
        !schedule
            .take_sample(Duration::from_millis(39), false)
            .unwrap()
    );
    assert_eq!(schedule.next, Duration::from_millis(40));
    let previous = schedule.next;
    assert_eq!(
        schedule.take_sample(Duration::MAX, false),
        Err(ScheduleError::TimeOverflow)
    );
    assert_eq!(schedule.next, previous);
    schedule.next = Duration::MAX;
    assert_eq!(
        schedule.take_sample(Duration::MAX, false),
        Err(ScheduleError::TimeOverflow)
    );
    assert_eq!(schedule.next, Duration::MAX);
}

#[test]
fn unlimited_and_subnanosecond_rates_and_unrepresentable_inputs() {
    for rate in [0.0, -1.0] {
        let mut schedule = SampleSchedule::new(rate).unwrap();
        for _ in 0..2 {
            assert!(schedule.take_sample(Duration::ZERO, false).unwrap());
            assert_eq!(schedule.next, Duration::ZERO);
        }
    }
    let mut fast = SampleSchedule::new(f64::MAX).unwrap();
    assert!(fast.take_sample(Duration::ZERO, false).unwrap());
    assert_eq!(fast.next, Duration::from_nanos(1));
    for rate in [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MIN_POSITIVE,
    ] {
        assert_eq!(
            SampleSchedule::new(rate).unwrap_err(),
            ScheduleError::InvalidRate
        );
    }
}

#[test]
fn recorded_native_sampling_clock_controls() {
    let rows: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("../fixtures/sensor_schedule.json")).unwrap();
    for row in rows {
        let mut schedule = SampleSchedule::new(row["rate_hz"].as_f64().unwrap()).unwrap();
        schedule.next = Duration::from_nanos(row["start_ns"].as_u64().unwrap());
        assert_eq!(
            schedule
                .take_sample(
                    Duration::from_nanos(row["now_ns"].as_u64().unwrap()),
                    row["forced"].as_bool().unwrap()
                )
                .unwrap(),
            row["sampled"].as_bool().unwrap(),
            "{}",
            row["name"]
        );
        assert_eq!(
            schedule.next.as_nanos(),
            u128::from(row["next_ns"].as_u64().unwrap()),
            "{}",
            row["name"]
        );
        schedule.reset();
        assert_eq!(schedule.next, Duration::ZERO);
    }
}

#[test]
fn runtime_rate_changes_preserve_native_phase_and_defined_nonfinite_rules() {
    let data: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/runtime_imu_rate.json")).unwrap();
    let mut schedule = SampleSchedule::new(10.0).unwrap();
    for (index, row) in data["observations"]["phase"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
    {
        if index == 1 {
            schedule.change_runtime_rate(5.0).unwrap();
        }
        if index == 3 {
            schedule.change_runtime_rate(10.0).unwrap();
        }
        assert_eq!(
            schedule
                .take_sample(Duration::from_nanos(row["now_ns"].as_u64().unwrap()), false)
                .unwrap(),
            row["sampled"].as_bool().unwrap()
        );
        assert_eq!(
            schedule.next.as_nanos(),
            u128::from(row["next_ns"].as_u64().unwrap())
        );
    }
    for group in data["observations"]["nonfinite_phase"].as_array().unwrap() {
        let (rate, start) = match group["request"].as_str().unwrap() {
            "nan" => (f64::NAN, 1_000_000_000),
            "inf" => (f64::INFINITY, 0),
            "-inf" => (0.0, 1_000_000_000), // Native service clamps negative requests.
            _ => panic!("unexpected native rate"),
        };
        let mut schedule = SampleSchedule::new(0.0).unwrap();
        schedule.next = Duration::from_nanos(start);
        schedule.change_runtime_rate(rate).unwrap();
        for row in group["rows"].as_array().unwrap() {
            assert_eq!(
                schedule
                    .take_sample(Duration::from_nanos(row["now_ns"].as_u64().unwrap()), false)
                    .unwrap(),
                row["sampled"].as_bool().unwrap()
            );
            assert_eq!(
                schedule.next.as_nanos(),
                u128::from(row["next_ns"].as_u64().unwrap())
            );
        }
    }
    // Runtime support does not relax the separate SDF-loading numerical boundary.
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            SampleSchedule::new(invalid),
            Err(ScheduleError::InvalidRate)
        ));
    }
    let before = (schedule.period, schedule.next);
    assert!(schedule.change_runtime_rate(f64::MIN_POSITIVE).is_err());
    assert_eq!((schedule.period, schedule.next), before);
}
