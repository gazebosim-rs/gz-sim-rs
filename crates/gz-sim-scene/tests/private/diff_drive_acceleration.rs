// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::limits_support::*;
use super::*;
const A: &str = "<min_acceleration>-0.5</min_acceleration><max_acceleration>0.5</max_acceleration>";
const UNBOUNDED: [[f64; 2]; 2] = [[f64::NEG_INFINITY, f64::INFINITY]; 2];

#[test]
fn acceleration_fields_use_native_generic_then_specific_bounds_without_mutating_dom() {
    let fields = "<min_acceleration>-4</min_acceleration><max_acceleration>5</max_acceleration><min_linear_acceleration>-2</min_linear_acceleration><max_linear_acceleration>3</max_linear_acceleration><min_angular_acceleration>-1</min_angular_acceleration><max_angular_acceleration>2</max_angular_acceleration>";
    let model = parsed_model(fields);
    let before = format!("{model:?}");
    let p = config::parse(&model).unwrap().unwrap();
    assert_eq!(p.command_limits.acceleration, [[-2., 3.], [-1., 2.]]);
    assert_eq!(p.command_limits.jerk, UNBOUNDED);
    assert!(p.command_limits.timed());
    assert_eq!(format!("{model:?}"), before);
    let defaults = config::parse(&parsed_model("")).unwrap().unwrap();
    assert_eq!(defaults.command_limits, model::CommandLimits::default());
    assert!(!defaults.command_limits.timed());
    for field in [
        "min_acceleration",
        "max_acceleration",
        "min_linear_acceleration",
        "max_linear_acceleration",
        "min_angular_acceleration",
        "max_angular_acceleration",
    ] {
        for value in ["NaN", "inf", "-inf", "nonsense", ""] {
            assert!(config::parse(&parsed_model(&format!("<{field}>{value}</{field}>"))).is_err());
        }
        assert!(
            config::parse(&parsed_model(&format!(
                "<{field}>1</{field}><{field}>1</{field}>"
            )))
            .is_err()
        );
    }
    for fields in [
        "<min_acceleration>2</min_acceleration><max_acceleration>1</max_acceleration>",
        "<min_linear_acceleration>2</min_linear_acceleration><max_linear_acceleration>1</max_linear_acceleration>",
        "<min_angular_acceleration>2</min_angular_acceleration><max_angular_acceleration>1</max_angular_acceleration>",
    ] {
        assert!(matches!(
            config::parse(&parsed_model(fields)),
            Err(LoadError::Unsupported {
                feature: "DiffDrive acceleration limits",
                ..
            })
        ));
    }
}

#[test]
fn acceleration_command_sequence_has_independent_binary_ramps_reversals_and_variable_dt() {
    let limits = model::CommandLimits {
        acceleration: [[-2., 1.], [-4., 2.]],
        ..Default::default()
    };
    let mut h = model::CommandHistory::default();
    // Independently tabulated a*dt ramps; every value is exactly binary representable.
    for (request, ms, expected) in [
        ([4., -4.], 250, [0.25, -1.]),
        ([4., -4.], 500, [0.75, -3.]),
        ([-4., 4.], 250, [0.25, -2.5]),
        ([-4., 4.], 500, [-0.75, -1.5]),
        ([0., 0.], 250, [-0.5, -1.]),
    ] {
        let actual = limits
            .prepare(request, h, UNBOUNDED, Duration::from_millis(ms))
            .unwrap();
        assert_eq!(actual, expected);
        h = model::CommandHistory {
            latest: actual,
            older: h.latest,
        };
    }
    let positive = model::CommandLimits {
        acceleration: [[1., 2.], [-2., -1.]],
        ..Default::default()
    };
    assert_eq!(
        positive
            .prepare(
                [0.; 2],
                Default::default(),
                UNBOUNDED,
                Duration::from_millis(500)
            )
            .unwrap(),
        [0.5, -0.5]
    );
    let tiny = model::CommandLimits {
        acceleration: [[-1., 1.]; 2],
        ..Default::default()
    };
    let dt = Duration::from_nanos(1);
    assert_eq!(
        tiny.prepare([1., -1.], Default::default(), UNBOUNDED, dt)
            .unwrap(),
        [dt.as_secs_f64(), -dt.as_secs_f64()]
    );
}

#[test]
fn velocity_only_path_keeps_legacy_clamps_and_has_no_timed_history() {
    let mut sim = load("", 1);
    let (e, id) = owners(&sim)[0];
    set_diff_drive_command(&mut sim, id, 100., -100.).unwrap();
    assert_eq!(sim.world().get::<Drive>(e).unwrap().desired, [1., -2.]);
    for _ in 0..16 {
        sim.step_exact(1).unwrap();
    }
    assert_eq!(
        sim.world().get::<Drive>(e).unwrap().command_history,
        model::CommandHistory::default()
    );
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(set_diff_drive_command(&mut sim, id, value, 0.).is_err());
        assert_eq!(sim.world().get::<Drive>(e).unwrap().desired, [1., -2.]);
    }
}

#[test]
fn limited_commands_match_independently_commanded_native_twins_and_real_odometry() {
    for count in [1, 8] {
        let mut limited = load(A, count);
        let mut reference = load("", count);
        let rows = owners(&limited);
        let refs = owners(&reference);
        for &(_, id) in &rows {
            set_diff_drive_command(&mut limited, id, 0.25, 0.).unwrap();
        }
        for step in 1..=128 {
            let request = if step <= 64 {
                f64::from(step) / 256.
            } else {
                0.25
            };
            for &(_, id) in &refs {
                set_diff_drive_command(&mut reference, id, request, 0.).unwrap();
            }
            limited.step_exact(1).unwrap();
            reference.step_exact(1).unwrap();
            for &(e, _) in &rows {
                assert_eq!(
                    limited
                        .world()
                        .get::<Drive>(e)
                        .unwrap()
                        .command_history
                        .latest,
                    [request, 0.]
                );
            }
            assert_physical_twins(&limited, &reference);
        }
        for &(e, _) in &rows {
            let sample = limited.world().get::<DiffDriveSample>(e).unwrap();
            assert!(
                sample.pose[0] > 0.001,
                "native wheel encoders, not the target, report motion"
            );
        }
    }
}

#[test]
fn timed_raw_request_pause_latest_zero_reset_and_model_retirement_have_separate_meanings() {
    let mut sim = load(A, 2);
    let rows = owners(&sim);
    let (e, id) = rows[0];
    let (_, other) = rows[1];
    set_diff_drive_command(&mut sim, id, 100., 0.).unwrap();
    assert_eq!(sim.world().get::<Drive>(e).unwrap().desired, [100., 0.]);
    set_diff_drive_command(&mut sim, id, 0.25, 0.).unwrap();
    sim.step_exact(8).unwrap();
    let before = histories(&sim);
    let states = native(&sim);
    let time = sim.elapsed();
    sim.set_paused(true);
    set_diff_drive_command(&mut sim, id, -0.5, 0.).unwrap();
    set_diff_drive_command(&mut sim, id, 0., 0.).unwrap();
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), time);
    assert_eq!(histories(&sim), before);
    assert_eq!(native(&sim), states);
    sim.step_exact(1).unwrap();
    assert!(
        sim.world().get::<Drive>(e).unwrap().command_history.latest[0] > 0.,
        "zero is a decelerating target when timed limits are selected"
    );
    sim.reset().unwrap();
    for &(e, _) in &rows {
        assert_eq!(
            sim.world().get::<Drive>(e).unwrap().command_history,
            Default::default()
        );
    }
    assert!(remove_model(&mut sim, id).unwrap().contains(&id));
    set_diff_drive_command(&mut sim, other, 0.25, 0.).unwrap();
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(owners(&sim).len(), 2);
    for (e, _) in owners(&sim) {
        assert_eq!(
            sim.world().get::<Drive>(e).unwrap().command_history,
            Default::default()
        );
    }
}

#[test]
fn late_owner_history_or_immutable_bound_corruption_cannot_partially_commit() {
    for corruption in 0..3 {
        let mut sim = load(A, 8);
        let rows = owners(&sim);
        for &(_, id) in &rows {
            set_diff_drive_command(&mut sim, id, 0.25, 0.).unwrap();
        }
        sim.step_exact(4).unwrap();
        let late = rows.last().unwrap().0;
        match corruption {
            0 => {
                sim.world_mut()
                    .get_mut::<Drive>(late)
                    .unwrap()
                    .command_history
                    .latest[0] = f64::NAN
            }
            1 => {
                sim.world_mut()
                    .get_mut::<Drive>(late)
                    .unwrap()
                    .command_history
                    .older[1] = f64::INFINITY
            }
            _ => {
                sim.world_mut()
                    .get_mut::<Drive>(late)
                    .unwrap()
                    .command_limits
                    .acceleration[0][1] = 1.
            }
        }
        let held = histories(&sim);
        let states = native(&sim);
        let time = sim.elapsed();
        assert!(sim.step_exact(1).is_err());
        assert_eq!(histories(&sim), held);
        assert_eq!(native(&sim), states);
        assert_eq!(sim.elapsed(), time);
        assert!(sim.fault().is_some());
        assert!(sim.reset().is_err());
    }
}

#[test]
fn healthy_repair_of_rejected_metadata_is_distinct_from_a_terminal_step_fault() {
    let mut sim = load(A, 1);
    let (e, id) = owners(&sim)[0];
    let original = sim.world().get::<Drive>(e).unwrap().command_limits;
    sim.world_mut()
        .get_mut::<Drive>(e)
        .unwrap()
        .command_limits
        .acceleration[0][1] = 2.;
    assert!(set_diff_drive_command(&mut sim, id, 0.25, 0.).is_err());
    assert!(sim.fault().is_none());
    sim.world_mut().get_mut::<Drive>(e).unwrap().command_limits = original;
    set_diff_drive_command(&mut sim, id, 0.25, 0.).unwrap();
    sim.step_exact(1).unwrap();
    assert!(sim.world().get::<Drive>(e).unwrap().command_history.latest[0] > 0.);
}

#[test]
fn derivative_validation_and_overflow_do_not_change_the_provided_history() {
    let base = model::CommandLimits {
        acceleration: [[-1., 1.]; 2],
        ..Default::default()
    };
    let h = model::CommandHistory::default();
    assert_eq!(
        base.prepare([0.; 2], h, UNBOUNDED, Duration::ZERO),
        Err(model::InvalidDrive::Time)
    );
    for axis in 0..2 {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut request = [0.; 2];
            request[axis] = value;
            assert!(
                base.prepare(request, h, UNBOUNDED, Duration::from_secs(1))
                    .is_err()
            );
            let mut bad = h;
            bad.latest[axis] = value;
            assert!(
                base.prepare([0.; 2], bad, UNBOUNDED, Duration::from_secs(1))
                    .is_err()
            );
            let mut bad = h;
            bad.older[axis] = value;
            assert!(
                base.prepare([0.; 2], bad, UNBOUNDED, Duration::from_secs(1))
                    .is_err()
            );
        }
        for range in [
            [f64::NAN, 1.],
            [-1., f64::NAN],
            [f64::INFINITY, f64::INFINITY],
            [f64::NEG_INFINITY, f64::NEG_INFINITY],
            [2., 1.],
        ] {
            let mut bad = base;
            bad.acceleration[axis] = range;
            assert!(
                bad.prepare([0.; 2], h, UNBOUNDED, Duration::from_secs(1))
                    .is_err()
            );
            let mut velocity = UNBOUNDED;
            velocity[axis] = range;
            assert!(
                base.prepare([0.; 2], h, velocity, Duration::from_secs(1))
                    .is_err()
            );
        }
    }
    assert!(
        base.prepare([f64::MAX, 0.], h, UNBOUNDED, Duration::from_nanos(1))
            .is_err()
    );
    let forced = model::CommandLimits {
        acceleration: [[f64::MAX, f64::MAX]; 2],
        ..Default::default()
    };
    assert!(
        forced
            .prepare([0.; 2], h, UNBOUNDED, Duration::from_secs(2))
            .is_err()
    );
    assert_eq!(h, Default::default());
}
