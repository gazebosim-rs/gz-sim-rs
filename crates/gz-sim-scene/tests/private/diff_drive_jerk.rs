// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::limits_support::*;
use super::*;
const J: &str = "<min_jerk>-4</min_jerk><max_jerk>4</max_jerk>";
const UNBOUNDED: [[f64; 2]; 2] = [[f64::NEG_INFINITY, f64::INFINITY]; 2];

#[test]
fn jerk_configuration_uses_axis_overrides_and_rejects_malformed_or_reversed_bounds() {
    let extra = "<min_jerk>-5</min_jerk><max_jerk>6</max_jerk><min_linear_jerk>-2</min_linear_jerk><max_linear_jerk>3</max_linear_jerk><min_angular_jerk>-4</min_angular_jerk><max_angular_jerk>5</max_angular_jerk>";
    let m = parsed_model(extra);
    let before = format!("{m:?}");
    let p = config::parse(&m).unwrap().unwrap();
    assert_eq!(p.command_limits.jerk, [[-2., 3.], [-4., 5.]]);
    assert!(p.command_limits.timed());
    assert_eq!(format!("{m:?}"), before);
    for field in [
        "min_jerk",
        "max_jerk",
        "min_linear_jerk",
        "max_linear_jerk",
        "min_angular_jerk",
        "max_angular_jerk",
    ] {
        for value in ["NaN", "inf", "-inf", "bad", ""] {
            assert!(config::parse(&parsed_model(&format!("<{field}>{value}</{field}>"))).is_err());
        }
        assert!(
            config::parse(&parsed_model(&format!(
                "<{field}>1</{field}><{field}>1</{field}>"
            )))
            .is_err()
        );
    }
    for (minimum, maximum) in [
        ("min_jerk", "max_jerk"),
        ("min_linear_jerk", "max_linear_jerk"),
        ("min_angular_jerk", "max_angular_jerk"),
    ] {
        assert!(matches!(
            config::parse(&parsed_model(&format!(
                "<{minimum}>2</{minimum}><{maximum}>1</{maximum}>"
            ))),
            Err(LoadError::Unsupported {
                feature: "DiffDrive jerk limits",
                ..
            })
        ));
    }
}

#[test]
fn discrete_jerk_table_uses_current_dt_and_has_no_half_coefficient() {
    let limits = model::CommandLimits {
        jerk: [[-2., 2.]; 2],
        ..Default::default()
    };
    let mut h = model::CommandHistory::default();
    for (request, millis, expected) in [
        (10., 500, 0.5),
        (10., 500, 1.5),
        (10., 250, 2.625),
        (10., 1000, 5.75),
        (-10., 500, 8.375),
    ] {
        let actual = limits
            .prepare(
                [request, -request],
                h,
                UNBOUNDED,
                Duration::from_millis(millis),
            )
            .unwrap();
        assert_eq!(actual, [expected, -expected]);
        h = model::CommandHistory {
            latest: actual,
            older: h.latest,
        };
    }
    let tiny = model::CommandLimits {
        jerk: [[-1., 1.]; 2],
        ..Default::default()
    };
    let dt = Duration::from_nanos(1);
    assert_eq!(
        tiny.prepare([1., -1.], Default::default(), UNBOUNDED, dt)
            .unwrap(),
        [
            dt.as_secs_f64() * dt.as_secs_f64(),
            -dt.as_secs_f64() * dt.as_secs_f64()
        ]
    );
}

#[test]
fn combined_bounds_follow_jerk_then_acceleration_then_velocity() {
    let limits = model::CommandLimits {
        jerk: [[-2., 2.]; 2],
        acceleration: [[-0.5, 0.5]; 2],
    };
    let h = model::CommandHistory {
        latest: [0.5, -0.5],
        older: [0., 0.],
    };
    assert_eq!(
        limits
            .prepare(
                [100., -100.],
                h,
                [[-0.625, 0.625]; 2],
                Duration::from_millis(500)
            )
            .unwrap(),
        [0.625, -0.625]
    );
    let mut sim = load(&format!("{J}<max_acceleration>0.5</max_acceleration>"), 1);
    let (e, id) = owners(&sim)[0];
    set_diff_drive_command(&mut sim, id, 100., 0.).unwrap();
    assert_eq!(sim.world().get::<Drive>(e).unwrap().desired, [100., 0.]);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<Drive>(e).unwrap().command_history.latest[0],
        1. / 4096.
    );
}

#[test]
fn jerk_limited_native_motion_matches_an_independent_integer_difference_command_twin() {
    for count in [1, 8] {
        let mut limited = load(J, count);
        let mut reference = load("", count);
        let rows = owners(&limited);
        let refs = owners(&reference);
        let (mut current, mut older) = (0_i32, 0_i32);
        for step in 1..=160 {
            let requested = if step <= 40 {
                1024
            } else if step <= 80 {
                -1024
            } else {
                0
            };
            // Integer lattice: dt=1/128, jerk=4, so each change in velocity
            // difference is bounded to exactly one unit of1/4096 m/s.
            let previous_difference = current - older;
            let next = (current
                + previous_difference
                + (requested - current - previous_difference).clamp(-1, 1))
            .clamp(-4096, 4096);
            for &(_, id) in &rows {
                set_diff_drive_command(&mut limited, id, f64::from(requested) / 4096., 0.).unwrap();
            }
            for &(_, id) in &refs {
                set_diff_drive_command(&mut reference, id, f64::from(next) / 4096., 0.).unwrap();
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
                    [f64::from(next) / 4096., 0.]
                );
            }
            assert_physical_twins(&limited, &reference);
            older = current;
            current = next;
        }
        assert!(
            native(&limited).values().any(|s| s[0].abs() > 0.001),
            "native encoders must physically turn"
        );
    }
}

#[test]
fn pause_latest_zero_and_reset_never_invent_a_second_history_sample() {
    let mut sim = load(J, 2);
    let rows = owners(&sim);
    for &(_, id) in &rows {
        set_diff_drive_command(&mut sim, id, 0.25, 0.).unwrap();
    }
    sim.step_exact(4).unwrap();
    let held = histories(&sim);
    let state = native(&sim);
    sim.set_paused(true);
    for &(_, id) in &rows {
        set_diff_drive_command(&mut sim, id, -0.25, 0.).unwrap();
        set_diff_drive_command(&mut sim, id, 0., 0.).unwrap();
    }
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(histories(&sim), held);
    assert_eq!(native(&sim), state);
    sim.step_exact(1).unwrap();
    for &(e, _) in &rows {
        let h = sim.world().get::<Drive>(e).unwrap().command_history;
        assert!(
            h.latest[0] > h.older[0],
            "zero target must respect retained positive acceleration while jerk decreases it"
        );
    }
    sim.reset().unwrap();
    for &(e, _) in &rows {
        assert_eq!(
            sim.world().get::<Drive>(e).unwrap().command_history,
            Default::default()
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (e, _) in owners(&sim) {
        assert_eq!(
            sim.world().get::<Drive>(e).unwrap().command_history,
            Default::default()
        );
    }
}

#[test]
fn late_invalid_jerk_history_keeps_all_earlier_owners_and_terminal_fault_intact() {
    let mut sim = load(J, 8);
    let rows = owners(&sim);
    for &(_, id) in &rows {
        set_diff_drive_command(&mut sim, id, 0.25, 0.).unwrap();
    }
    sim.step_exact(4).unwrap();
    let late = rows.last().unwrap().0;
    sim.world_mut()
        .get_mut::<Drive>(late)
        .unwrap()
        .command_history
        .older[0] = f64::MAX;
    let held = histories(&sim);
    let state = native(&sim);
    let time = sim.elapsed();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(histories(&sim), held);
    assert_eq!(native(&sim), state);
    assert_eq!(sim.elapsed(), time);
    assert!(sim.reset().is_err());
}

#[test]
fn wheel_saturation_does_not_relabel_command_history_as_measured_velocity() {
    let extra =
        format!("{J}<min_acceleration>-1</min_acceleration><max_acceleration>1</max_acceleration>");
    let limited_xml =
        xml(&extra, 1).replace("<velocity>10</velocity>", "<velocity>0.125</velocity>");
    let mut sim = load_world_with_sdf_step(
        &limited_xml,
        "m113-saturation.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let (e, id) = owners(&sim)[0];
    set_diff_drive_command(&mut sim, id, 0.75, 0.).unwrap();
    sim.step_exact(64).unwrap();
    let command = sim.world().get::<Drive>(e).unwrap().command_history.latest[0];
    assert!(command > 0.025);
    let measured = sim.world().get::<DiffDriveSample>(e).unwrap().twist[0];
    assert!(measured.is_finite());
    assert!((measured - command).abs() > 0.001);
    // Wheel limits bound targets rather than externally driven physical states.
    let targets = model::Geometry::new(0.2, 0.5)
        .unwrap()
        .bounded_targets(command, 0., [0.125; 2])
        .unwrap();
    assert_eq!(targets, [0.125; 2]);
}

#[test]
fn jerk_overflow_and_invalid_bounds_leave_all_inputs_unmodified() {
    let base = model::CommandLimits {
        jerk: [[-1., 1.]; 2],
        ..Default::default()
    };
    let h = model::CommandHistory::default();
    for axis in 0..2 {
        for range in [
            [f64::NAN, 1.],
            [-1., f64::NAN],
            [f64::INFINITY, f64::INFINITY],
            [f64::NEG_INFINITY, f64::NEG_INFINITY],
            [2., 1.],
        ] {
            let mut bad = base;
            bad.jerk[axis] = range;
            assert!(
                bad.prepare([0.; 2], h, UNBOUNDED, Duration::from_secs(1))
                    .is_err()
            );
        }
    }
    for (request, previous, dt) in [
        ([f64::MAX, 0.], h, Duration::from_nanos(1)),
        (
            [0.; 2],
            model::CommandHistory {
                latest: [f64::MAX, 0.],
                older: [-f64::MAX, 0.],
            },
            Duration::from_secs(1),
        ),
        (
            [0.; 2],
            model::CommandHistory {
                latest: [-f64::MAX, 0.],
                older: [0.; 2],
            },
            Duration::from_secs(1),
        ),
    ] {
        assert!(base.prepare(request, previous, UNBOUNDED, dt).is_err());
    }
    let forced = model::CommandLimits {
        jerk: [[f64::MAX, f64::MAX]; 2],
        ..Default::default()
    };
    assert!(
        forced
            .prepare([0.; 2], h, UNBOUNDED, Duration::from_secs(3))
            .is_err()
    );
    let forced = model::CommandLimits {
        jerk: [[8e307, 8e307]; 2],
        ..Default::default()
    };
    assert!(
        forced
            .prepare([0.; 2], h, UNBOUNDED, Duration::from_secs(2))
            .is_err()
    );
    assert_eq!(h, Default::default());
}
