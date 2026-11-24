// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

struct Approach {
    incoming: BodyState,
    launch_pitch_deg: f64,
    launch_spin_rad_s: f64,
    pitch_at_238_4_ms: f64,
    peak: Load,
}

fn pitch_deg(state: BodyState) -> f64 {
    let axis = DQuat::from_array(state.orientation) * DVec3::X;
    (-axis.z).atan2(axis.x).to_degrees()
}

/// Use the same settled coarse grasp for each split except the continuous fine
/// control. All splits enter an identical 1 µs impact window at 238 ms.
fn split(preload: Integration, launch_steps: u32, approach_steps: u32) -> Approach {
    let mut rig = preloaded(preload, true);
    let incoming = rig.state();
    install_schedule(&mut rig, true, move |start| {
        let elapsed = start - CLOCK * PRELOAD as u32;
        if elapsed.is_zero() {
            launch_steps
        } else if elapsed < Duration::from_micros(LANDING_START_US) {
            approach_steps
        } else {
            200
        }
    });
    rig.sim.step_exact(1).unwrap();
    let launch = rig.state();
    rig.sim.step_exact(189).unwrap();
    let mut pitch_at_238_4_ms = None;
    let mut peak = Load::default();
    for _ in 190..200 {
        rig.sim.step_exact(1).unwrap();
        if rig.sim.elapsed() == Duration::from_micros(238_400) {
            pitch_at_238_4_ms = Some(pitch_deg(rig.state()));
        }
        let sample = rig.sim.world().resource::<FineTick>().loads[0];
        if sample.peak > peak.peak {
            peak = sample;
        }
    }
    Approach {
        incoming,
        launch_pitch_deg: pitch_deg(launch),
        launch_spin_rad_s: launch.angular_velocity[1],
        pitch_at_238_4_ms: pitch_at_238_4_ms.unwrap(),
        peak,
    }
}

#[test]
fn matched_grasps_separate_preload_launch_and_loaded_approach_errors() {
    let fine = split(Integration::Continuous, 200, 200);
    let matched = split(Integration::Coarse, 200, 200);
    let impact_only = split(Integration::Coarse, 1, 1);
    let fine_launch = split(Integration::Coarse, 200, 1);
    let fine_approach = split(Integration::Coarse, 1, 200);
    for (mode, a) in [
        ("fine", &fine),
        ("matched", &matched),
        ("impact_only", &impact_only),
        ("fine_launch", &fine_launch),
        ("fine_approach", &fine_approach),
    ] {
        println!(
            "split mode={mode} launch_deg={} launch_rad_s={} approach_deg={} peak_n={} peak_ns={}",
            a.launch_pitch_deg,
            a.launch_spin_rad_s,
            a.pitch_at_238_4_ms,
            a.peak.peak,
            a.peak.peak_time.as_nanos()
        );
    }
    // Hold the fine launch and measurement window fixed and vary only the
    // time grid on the loaded approach. This also detects an accidental change
    // in the public controller cadence when the substep count changes.
    let mut previous_error = fine_launch.peak.peak_time.abs_diff(fine.peak.peak_time);
    for steps in [4, 8, 16] {
        let grid = split(Integration::Coarse, 200, steps);
        let error = grid.peak.peak_time.abs_diff(fine.peak.peak_time);
        assert_eq!(grid.incoming, matched.incoming);
        assert_eq!(grid.launch_pitch_deg, matched.launch_pitch_deg);
        assert_eq!(grid.launch_spin_rad_s, matched.launch_spin_rad_s);
        assert!(error < previous_error);
        previous_error = error;
        println!(
            "grid steps={steps} peak_ns={} peak_n={} error_ns={}",
            grid.peak.peak_time.as_nanos(),
            grid.peak.peak,
            error.as_nanos()
        );
    }
    // Tiny settling differences do not alter the fine return time.
    assert!((pitch_deg(fine.incoming) - pitch_deg(matched.incoming)).abs() < 1e-8);
    assert_eq!(matched.peak.peak_time, fine.peak.peak_time);
    assert!((matched.peak.peak - fine.peak.peak).abs() < 0.001);
    for a in [&impact_only, &fine_launch, &fine_approach] {
        assert_eq!(a.incoming, matched.incoming);
    }
    assert!(impact_only.launch_pitch_deg - matched.launch_pitch_deg > 0.04);
    assert!(matched.launch_spin_rad_s - impact_only.launch_spin_rad_s > 0.9);
    assert_eq!(fine_launch.launch_pitch_deg, matched.launch_pitch_deg);
    assert_eq!(fine_launch.launch_spin_rad_s, matched.launch_spin_rad_s);
    assert_eq!(fine_approach.launch_pitch_deg, impact_only.launch_pitch_deg);
    // Fixing the launch alone actually moves the measured peak farther ahead;
    // the coarse integration of the moving loaded edge dominates the timing error.
    assert!(fine_launch.peak.peak_time < impact_only.peak.peak_time);
    assert!(impact_only.peak.peak_time + Duration::from_micros(130) < fine.peak.peak_time);
    assert!((matched.pitch_at_238_4_ms - impact_only.pitch_at_238_4_ms) > 0.20);
    assert!(fine_approach.peak.peak_time > fine.peak.peak_time);
    assert!(fine_approach.peak.peak_time.abs_diff(fine.peak.peak_time) < Duration::from_micros(30));
}

#[test]
fn moving_loaded_pad_reduces_return_time_and_pitch_errors() {
    let prior = run(Integration::Adaptive);
    let corrected = run(Integration::Rocking);
    let fine = run(Integration::Continuous);
    let pitch_error = |trial: &Trial| {
        trial
            .pitch
            .iter()
            .zip(&fine.pitch)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
            .to_degrees()
    };
    let old_pitch = pitch_error(&prior);
    let new_pitch = pitch_error(&corrected);
    println!(
        "rocking old_deg={old_pitch} new_deg={new_pitch} calls={} budgets={:?} impact={:?} rocking={:?} fine={:?}",
        corrected.calls, corrected.budgets, prior.loads, corrected.loads, fine.loads
    );
    assert!(new_pitch < old_pitch * 0.25);
    assert!(corrected.calls < fine.calls / 50);
    assert!(corrected.budgets.get(&8).is_some_and(|ticks| *ticks > 0));
    assert!(
        corrected
            .budgets
            .get(&200)
            .is_some_and(|ticks| (1..=8).contains(ticks))
    );
    for ((old, new), reference) in prior.loads.iter().zip(&corrected.loads).zip(&fine.loads) {
        let prior_error = old.peak_time.abs_diff(reference.peak_time);
        let new_error = new.peak_time.abs_diff(reference.peak_time);
        assert!(new_error < prior_error / 4 && new_error < Duration::from_micros(30));
        assert!((new.peak - reference.peak).abs() / reference.peak < 0.01);
        assert!(new.linear.distance(reference.linear) / reference.linear.length() < 0.004);
        assert!(new.angular.distance(reference.angular) / reference.angular.length() < 0.005);
    }
}
