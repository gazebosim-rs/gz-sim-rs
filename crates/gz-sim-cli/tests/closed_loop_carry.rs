// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise the same SDF, controller and recording path as the public example.
#[allow(dead_code)]
#[path = "../examples/closed_loop_carry.rs"]
mod carry;

use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use carry::{Case, Grid};
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufRead, BufReader},
};

#[derive(Deserialize)]
struct Tick {
    time_ns: u64,
    slip_m: f64,
    force_n: f64,
    lift_force_n: f64,
    next_lift_force_n: f64,
    state: String,
    pad_z_m: f64,
    parcel_com_z_m: f64,
    vertical_friction_n: [f64; 2],
    normal_force_n: [f64; 2],
    tactile_height_m: Option<f64>,
    sampled: bool,
    native_steps: u32,
}

struct Trace(Vec<Tick>);

impl Trace {
    fn run(case: Case, grid: Grid) -> Self {
        ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().num_threads(1).build());
        let dir = tempfile::tempdir().unwrap();
        let out = dir.path().join("recording");
        carry::run(case, grid, &out).unwrap();
        let file = File::open(out.join("ticks.jsonl")).unwrap();
        Self(
            BufReader::new(file)
                .lines()
                .map(|line| serde_json::from_str(&line.unwrap()).unwrap())
                .collect(),
        )
    }

    fn at(&self, ms: u64) -> &Tick {
        self.0.iter().find(|t| t.time_ns == ms * 1_000_000).unwrap()
    }

    fn last(&self) -> &Tick {
        self.0.last().unwrap()
    }

    fn first_state(&self, state: &str) -> &Tick {
        self.0.iter().find(|t| t.state == state).unwrap()
    }

    fn entries(&self, state: &str) -> usize {
        self.0
            .windows(2)
            .filter(|w| w[0].state != state && w[1].state == state)
            .count()
    }

    fn check_slew(&self, rate: f64) {
        for w in self.0.windows(2).filter(|w| w[0].time_ns >= 210_000_000) {
            let elapsed = (w[1].time_ns - w[0].time_ns) as f64 * 1e-9;
            assert!((w[1].force_n - w[0].force_n) <= rate * elapsed + 1e-10);
        }
    }
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} vs {expected} (tolerance {tolerance})"
    );
}

#[test]
fn invalid_recording_paths_preserve_existing_files() {
    use std::path::Path;
    let dir = tempfile::tempdir().unwrap();
    let file = dir.path().join("owner");
    std::fs::write(&file, b"existing").unwrap();
    for output in [
        Path::new(""),
        dir.path(),
        file.as_path(),
        &file.join("child"),
    ] {
        assert!(carry::run(Case::Catch, Grid::Adaptive, output).is_err());
    }
    assert_eq!(std::fs::read(file).unwrap(), b"existing");
}

#[test]
fn measured_contact_forces_explain_the_parcels_motion() {
    let open = Trace::run(Case::Open, Grid::Adaptive);
    let hold = Trace::run(Case::Hold, Grid::Adaptive);
    let lift = 10.4956;
    let parcel = 0.2;
    let gripper = 0.56;
    let capacity = 2. * 0.5 * 2.3;
    near(
        hold.at(250).vertical_friction_n.iter().sum(),
        parcel * lift / (gripper + parcel),
        0.003,
    );
    near(
        open.at(250).vertical_friction_n.iter().sum(),
        capacity,
        0.003,
    );
    // During sustained sliding both bodies accelerate. Gravity cancels in
    // their relative motion; their shared contact force has opposite signs.
    let expected = (lift - capacity) / gripper - capacity / parcel;
    let acceleration =
        (open.at(260).slip_m - 2. * open.at(250).slip_m + open.at(240).slip_m) / 0.01_f64.powi(2);
    near(acceleration, expected, 0.03);
    assert!(open.at(290).slip_m > 0.009 && hold.at(290).slip_m.abs() < 1e-5);
    assert!(hold.at(290).pad_z_m > hold.at(210).pad_z_m + 0.02);
}

#[test]
fn feedback_catches_under_load_and_obeys_the_physical_motor_clock() {
    let nominal = Trace::run(Case::Catch, Grid::Adaptive);
    let delayed = Trace::run(Case::Delayed, Grid::Adaptive);
    let slow = Trace::run(Case::Slow, Grid::Adaptive);
    for trace in [&nominal, &delayed, &slow] {
        let tightening = trace.first_state("Tightening");
        assert!(tightening.sampled && tightening.slip_m >= 0.0001);
        assert_eq!(trace.last().state, "Caught");
        assert!(trace.last().slip_m < 0.009);
        assert!(trace.last().parcel_com_z_m > tightening.parcel_com_z_m);
        trace.check_slew(120.);
    }
    assert!(nominal.last().slip_m < delayed.last().slip_m);
    slow.check_slew(30.);
    assert!(nominal.last().slip_m < slow.last().slip_m);
    assert!(nominal.first_state("Caught").time_ns < slow.first_state("Caught").time_ns);
}

/// Independent force-balance solution with an ideal linear ramp in normal force.
#[test]
fn force_balance_predicts_when_and_how_far_the_parcel_slides() {
    let trace = Trace::run(Case::Catch, Grid::Adaptive);
    let inv_reduced_mass = 1. / 0.56 + 1. / 0.2;
    let acceleration = 10.4956 / 0.56 - 2. * 0.5 * 2.3 * inv_reduced_mass;
    let trigger = (2. * 0.0001_f64 / acceleration).sqrt() + 0.001;
    let initial_speed = acceleration * trigger;
    let jerk = 2. * 0.5 * 120. * inv_reduced_mass;
    let braking = (acceleration + (acceleration.powi(2) + 2. * jerk * initial_speed).sqrt()) / jerk;
    let predicted_drop = 0.5 * acceleration * trigger.powi(2)
        + initial_speed * braking
        + 0.5 * acceleration * braking.powi(2)
        - jerk * braking.powi(3) / 6.;
    near(
        trace.first_state("Tightening").time_ns as f64 * 1e-9,
        0.210 + trigger,
        0.0004,
    );
    near(trace.last().slip_m, predicted_drop, 0.00001);
    near(
        trace.first_state("Caught").time_ns as f64 * 1e-9,
        0.210 + trigger + braking + 0.004,
        0.0008,
    );
}

#[test]
fn easing_the_lift_arrests_slip_at_the_same_finger_force_limit() {
    let limited = Trace::run(Case::Limited, Grid::Adaptive);
    let unloaded = Trace::run(Case::Unload, Grid::Adaptive);
    for trace in [&limited, &unloaded] {
        assert!(trace.entries("AtLimit") > 0);
        near(trace.last().force_n, 2.55, 1e-9);
    }
    // A capped grip carries the static weight but cannot supply the original
    // acceleration. Reducing lift effort to loaded weight restores that margin.
    let grip_limit = 2. * 0.5 * 2.55;
    assert!(0.2 * limited.last().lift_force_n / 0.76 > grip_limit);
    near(unloaded.last().lift_force_n, 0.76 * 9.81, 1e-8);
    assert_eq!(limited.last().state, "AtLimit");
    assert_eq!(unloaded.last().state, "Caught");
    assert!(unloaded.last().slip_m < limited.last().slip_m / 2.);
    assert!(unloaded.last().pad_z_m > unloaded.first_state("AtLimit").pad_z_m);
}

#[test]
fn slower_sensor_waits_for_the_real_drive_to_stay_at_its_limit() {
    let run = Trace::run(Case::DelayedUnload, Grid::Adaptive);
    let requested = run.first_state("AtLimit");
    assert!(requested.force_n < 2.55);
    let applied = run
        .0
        .iter()
        .find(|t| t.time_ns >= requested.time_ns && t.force_n >= 2.55)
        .unwrap();
    let backoff = run
        .0
        .iter()
        .find(|t| t.next_lift_force_n < t.lift_force_n)
        .unwrap();
    // Timestamps close the measured interval, so the first applied sample
    // already accounts for one 200 µs tick of the 3 ms interlock dwell.
    assert_eq!(backoff.time_ns - applied.time_ns + 200_000, 3_000_000);
    assert_eq!(run.last().state, "Caught");
    run.check_slew(120.);
}

#[test]
fn payload_friction_and_offset_change_the_load_the_fingers_recover() {
    let nominal = Trace::run(Case::Catch, Grid::Adaptive);
    let heavy = Trace::run(Case::Heavy, Grid::Adaptive);
    let slick = Trace::run(Case::Slick, Grid::Adaptive);
    let mirror = Trace::run(Case::Mirror, Grid::Adaptive);
    for (trace, mass, mu) in [
        (&nominal, 0.2, 0.5),
        (&heavy, 0.26, 0.5),
        (&slick, 0.2, 0.4),
        (&mirror, 0.2, 0.5),
    ] {
        let last = trace.last();
        assert_eq!(last.state, "Caught");
        let needed = mass * last.lift_force_n / (0.56 + mass);
        near(last.vertical_friction_n.iter().sum(), needed, 0.005);
        assert!(2. * mu * last.force_n >= needed);
    }
    let left = nominal.last().vertical_friction_n;
    let right = mirror.last().vertical_friction_n;
    near(left[0], right[1], 0.005);
    near(left[1], right[0], 0.005);
    assert!(left[1] - left[0] > 0.05);
    near(nominal.last().slip_m, mirror.last().slip_m, 2e-5);
}

#[test]
fn reacquired_grip_can_detect_another_slide_and_can_be_released() {
    let repeat = Trace::run(Case::Repeat, Grid::Adaptive);
    assert_eq!(repeat.entries("Caught"), 2);
    assert_eq!(repeat.last().state, "Caught");
    let release = Trace::run(Case::Release, Grid::Adaptive);
    let released = release.first_state("Released");
    assert!(release.first_state("Caught").time_ns < released.time_ns);
    assert_eq!(release.last().state, "Released");
    assert!(release.last().force_n <= 0.);
    assert!(release.last().slip_m > released.slip_m + 0.001);
    assert_eq!(release.at(270).normal_force_n, [0.; 2]);
    assert_eq!(release.at(280).vertical_friction_n, [0.; 2]);
    let free_acceleration = (release.at(290).parcel_com_z_m - 2. * release.at(280).parcel_com_z_m
        + release.at(270).parcel_com_z_m)
        / 0.01_f64.powi(2);
    near(free_acceleration, -9.81, 1e-5);
}

#[test]
fn both_loaded_fingers_sense_the_slide_without_using_the_parcel_pose() {
    let tactile = Trace::run(Case::Tactile, Grid::Adaptive);
    let pose = Trace::run(Case::Catch, Grid::Adaptive);
    for tick in tactile.0.iter().filter(|t| t.time_ns >= 210_000_000) {
        near(-tick.tactile_height_m.unwrap(), tick.slip_m, 0.000006);
    }
    assert_eq!(tactile.last().state, "Caught");
    near(tactile.last().slip_m, pose.last().slip_m, 0.00001);
    assert!(
        tactile
            .first_state("Caught")
            .time_ns
            .abs_diff(pose.first_state("Caught").time_ns)
            <= 200_000
    );
}

/// The reference uses 200 native 1 µs solves per public actuator tick.
#[test]
#[ignore = "full continuously fine qualification matrix"]
fn feedback_outcomes_agree_with_continuous_fine_physics() {
    use Case::*;
    for case in [
        Open,
        Hold,
        Catch,
        Tactile,
        Delayed,
        Slow,
        Limited,
        Unload,
        DelayedUnload,
        Heavy,
        Slick,
        Mirror,
        Repeat,
        Release,
    ] {
        let adaptive = Trace::run(case, Grid::Adaptive);
        let fine = Trace::run(case, Grid::Fine);
        assert_eq!(adaptive.0.len(), fine.0.len());
        assert_eq!(adaptive.last().state, fine.last().state, "{case:?}");
        assert_eq!(
            adaptive.entries("Caught"),
            fine.entries("Caught"),
            "{case:?}"
        );
        let mut worst: f64 = 0.;
        for (a, f) in adaptive.0.iter().zip(&fine.0) {
            assert_eq!(a.time_ns, f.time_ns);
            assert_eq!(a.sampled, f.sampled);
            assert_eq!(f.native_steps, 200);
            worst = worst.max((a.slip_m - f.slip_m).abs());
        }
        assert!(worst < 0.000025, "{case:?}: {worst} m");
    }
}
