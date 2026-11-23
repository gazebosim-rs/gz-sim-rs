// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn sample() -> Tick {
    Tick {
        time_ns: 1_650_000_000,
        phase: "acquire",
        force_y_n: 0.,
        mass_kg: 0.0084,
        momentum_y_kg_m_s: 0.,
        bench_side_reaction_n: 0.,
        command_stroke_m: START,
        cross_panel_n: TARGET,
        panel_gap_m: 0.,
        aperture_m: 0.0432,
        jaw_up_n: [0.04; 2],
        jaw_side_reaction_n: [0.; 2],
        jaw_squeeze_n: [0.9; 2],
        bench_up_n: 0.,
        min_bench_clearance_m: 0.15,
        jaw_frame_y_m: 0.,
        pad_frame_y_m: [0.; 2],
    }
}

#[test]
fn feedback_preserves_the_deadband_and_only_opens_for_excess_load() {
    assert_eq!(START, command(START, TARGET, 0.));
    assert_eq!(START + 25e-6, command(START, 0., 2e-6));
    assert_eq!(START + 0.003 * TARGET, command(START, 0., 0.1e-6));
    assert!(command(START, 2. * TARGET, 0.) < START);
    assert_eq!(command(0., 1., 0.), 0.);
    assert_eq!(command(0.080, 0., 0.), 0.080);
    let mut t = sample();
    assert!(supported(&t, 0.08));
    t.jaw_up_n = [0.08, 0.];
    assert!(!supported(&t, 0.08));
    t.jaw_up_n = [0.03, 0.04];
    assert!(!supported(&t, 0.08));
    t.jaw_up_n = [0.04; 2];
    t.bench_up_n = 0.01;
    assert!(!supported(&t, 0.08));
    t.bench_up_n = 0.;
    t.min_bench_clearance_m = 0.;
    assert!(!supported(&t, 0.08));
}

#[test]
fn the_same_discrete_pulse_integrates_to_its_ramp_and_hold_impulse() {
    assert_eq!(pulse(1), ("ramp", PULL / RAMP as f64));
    assert_eq!(pulse(RAMP), ("ramp", PULL));
    assert_eq!(pulse(RAMP + 1), ("pull", PULL));
    assert_eq!(pulse(RAMP + HOLD), ("pull", PULL));
    assert_eq!(pulse(2 * RAMP + HOLD), ("unload", 0.));
    assert_eq!(pulse(END), ("recovery", 0.));
    let impulse = (1..=END).map(|n| pulse(n).1 * DT).sum::<f64>();
    assert!((impulse - PULL * (RAMP + HOLD) as f64 * DT).abs() < 1e-14);
}

#[test]
fn rigid_motion_cancels_and_an_unequal_mass_grid_reports_its_true_centroid() {
    let rotation = DQuat::from_rotation_z(0.75);
    let origin = DVec3::new(2., -3., 4.);
    let jaw = |x: f64| BodyState {
        position: (origin + rotation * (DVec3::X * x)).to_array(),
        orientation: rotation.to_array(),
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let jaws = [jaw(-1.), jaw(1.)];
    let particles = [(1., 1.), (3., 3.)].map(|(y, m)| Particle {
        position: (origin + rotation * (DVec3::Y * y)).to_array(),
        velocity: [0.; 3],
        inverse_mass: 1. / m,
    });
    let pads = [0, 1].map(|i| PadSample {
        particles: [i; 3],
        weights: [0.2, 0.3, 0.5],
        stiffness_weight: 1.,
    });
    let (y, patch) = lateral_positions(&particles, &pads, &jaws);
    assert!((y - 2.5).abs() < 1e-14);
    assert!((patch[0] - 1.).abs() < 1e-14 && (patch[1] - 3.).abs() < 1e-14);
}

fn series() -> Outcome {
    let mut trace = vec![sample(); WINDOW + END];
    for (i, row) in trace.iter_mut().enumerate() {
        row.time_ns += i as u128 * 1_000_000;
        row.jaw_frame_y_m = i as f64 * 1e-8;
        if i >= WINDOW {
            row.phase = pulse(i + 1 - WINDOW).0;
        }
    }
    Outcome {
        trace,
        anchor: json!({"pending_wrench":[1,2]}),
        baseline: WINDOW - 1,
        weight: 0.08,
    }
}

#[test]
fn slip_metrics_subtract_sham_drift_and_distinguish_recovery_from_peak() -> Result<()> {
    let mut active = series();
    for row in &mut active.trace[WINDOW..] {
        let magnitude = if row.phase == "pull" { 0.002 } else { 0.0001 };
        row.jaw_frame_y_m += magnitude;
        row.pad_frame_y_m = [-magnitude, magnitude * 0.5];
        row.cross_panel_n = 0.;
    }
    let measured = report(5, active, series())?;
    assert_eq!(measured["pull"]["supported_ticks"], END);
    assert_eq!(measured["sham"]["loaded_ticks_in_band"], HOLD);
    assert_eq!(measured["pull"]["loaded_ticks_in_band"], 0);
    for (i, scale) in [1., -1., 0.5].into_iter().enumerate() {
        let peak = measured["peak_signed_displacement_vs_sham_m"][i]
            .as_f64()
            .unwrap();
        let residual = measured["residual_displacement_vs_sham_m"][i]
            .as_f64()
            .unwrap();
        assert!((peak - scale * 0.002).abs() < 1e-14);
        assert!((residual - scale * 0.0001).abs() < 1e-14);
    }
    for mismatch in 0..3 {
        let mut changed = series();
        match mismatch {
            0 => changed.baseline += 1,
            1 => changed.anchor = Value::Null,
            _ => changed.trace[1].command_stroke_m += 1e-6,
        }
        assert!(report(7, series(), changed).is_err());
    }
    Ok(())
}

#[test]
fn a_failed_acquisition_or_sensor_read_cannot_qualify_a_pull() -> Result<()> {
    assert!(Grasp::pickup("<bad>", 3).is_err());
    assert!(Grasp::pickup(arm::SCENE, 3).is_err());
    assert!(trial(4, true, Path::new(".")).is_err());
    let mut grasp = Grasp::pickup(&closing_sheet_scene::scene(3)?, 3)?;
    assert!(grasp.acquire(1).is_err());
    let before = grasp.recording.sim.elapsed();
    assert!(grasp.step("ramp", f64::NAN).is_err());
    assert_eq!(grasp.recording.sim.elapsed(), before);
    grasp.jaws.ids[1] = u64::MAX;
    assert!(grasp.step("pull", 0.).is_err());
    Ok(())
}

#[test]
fn failed_replay_or_trace_cannot_publish_partial_results() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let dest = temp.path().join("result");
    let scene = closing_sheet_scene::scene(3)?;
    for obstacle in [
        "replay",
        "world.sdf",
        "trace.json",
        "particle-frames.json",
        "frames.json",
    ] {
        let err = write_new(&dest, |stage| {
            let mut r = Recording::new(&scene, "test.sdf", "default")?;
            r.advance()?;
            let mut frames = vec![];
            arm_sheet::capture(&mut r, &mut frames)?;
            fs::create_dir(stage.join(obstacle))?;
            save_case(r, frames, &[sample()], &scene, stage, 0, 3)
        });
        assert!(err.is_err() && !dest.exists(), "{obstacle}");
    }
    for simulate in [
        (|_, _, _: &Path| -> Result<Outcome> { panic!("test worker stopped") })
            as fn(usize, bool, &Path) -> Result<Outcome>,
        |_, loaded, _| {
            if loaded {
                Err("refused acquisition".into())
            } else {
                Ok(series())
            }
        },
        |_, loaded, _| {
            if loaded {
                Ok(series())
            } else {
                Err("refused sham".into())
            }
        },
        |_, loaded, _| {
            let mut o = series();
            o.anchor = loaded.into();
            Ok(o)
        },
    ] {
        assert!(write_new(&dest, |p| run_cases(p, simulate)).is_err());
        assert!(!dest.exists());
    }
    write_new(&dest, |p| run_cases(p, |_, _, _| Ok(series())))?;
    assert!(dest.join("comparison.json").exists());
    Ok(())
}
