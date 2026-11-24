// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use identify::{Refusal, START};

fn synthetic(change: impl Fn(usize, &mut JawReadings)) -> Estimate {
    let mut calibration = Calibration::default();
    let mut y_prev = 0.;
    let mut y_prev2 = 0.;
    for tick in START - 399..=CAL_END {
        let t = (tick.saturating_sub(START) as f64 * DT).min(4.);
        // Sampled twice-differentiable motion with stationary measurement windows.
        let outward = t <= 2.;
        let u = ((if outward { t } else { t - 2. }) / 0.75).min(1.);
        let f = u.powi(3) * (10. + u * (-15. + 6. * u));
        let y = 0.035 * (if outward { f } else { 1. - f });
        let mut r = JawReadings {
            rail_y_m: y,
            rail_speed_m_s: (y - y_prev) / DT,
            spindle_rad: controller::NORMAL_LIFT,
            squeeze_n: [1.; 2],
            force_on_payload_n: [
                0.,
                0.027 * (y - 2. * y_prev + y_prev2) / DT.powi(2),
                0.027 * 9.81,
            ],
        };
        change(tick, &mut r);
        calibration.observe(tick, &r);
        y_prev2 = y_prev;
        y_prev = y;
    }
    calibration.finish()
}
#[test]
fn known_acceleration_identifies_carried_mass_and_rejects_vertical_bias() {
    let est = synthetic(|_, _| {});
    assert!((est.mass_kg.unwrap() - 0.027).abs() < 1e-12);
    assert!((est.inertial_mass_kg.unwrap() - 0.027).abs() < 1e-12);
    let est = synthetic(|_, r| r.force_on_payload_n[2] *= 0.6);
    assert_eq!(est.refused, Some(Refusal::MassDisagreement));
    assert!(est.mass_kg.is_none());
    let drift = synthetic(|t, r| {
        if t >= START + 3000 {
            r.force_on_payload_n[2] *= 1.4;
        }
    });
    assert_eq!(drift.refused, Some(Refusal::InconsistentWeight));
}

#[test]
fn partial_plateaus_or_blocked_encoder_motion_cannot_authorize_carry() {
    let mut calibration = Calibration::default();
    assert_eq!(
        calibration.finish().refused,
        Some(Refusal::InsufficientProbe)
    );
    calibration.observe(START, &JawReadings::default());
    assert_eq!(
        calibration.first_problem(),
        Some(Refusal::InsufficientProbe)
    );
    let est = synthetic(|_, r| r.rail_y_m = 0.);
    assert_eq!(est.refused, Some(Refusal::InsufficientProbe));
    assert!(est.inertial_mass_kg.is_none());
}

#[test]
fn reset_during_real_mass_probe_rebuilds_wrenches_and_rail_state() -> Result<()> {
    let xml = Trial::Light.xml();
    let sdf = sdformat::load_str(&xml, "probe-reset.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "probe-reset.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let partial_run = |sim: &mut Simulation| -> Result<_> {
        let robot = Robot::load(sim, &sdf)?;
        let mut blind = WeighedRoute::new(2000);
        for tick in 1..=6000 {
            blind.route.drive(&robot, sim, tick)?;
            Calibration::drive(sim, &robot, tick)?;
            sim.step_exact(1)?;
            blind.observe(tick, &JawReadings::read(sim, &robot)?)?;
        }
        assert!(blind.estimate.is_none());
        let sensors = JawReadings::read(sim, &robot)?;
        assert!(sensors.rail_y_m > 0.02);
        Ok((sensors, blind.calibration.finish()))
    };
    let at_probe = partial_run(&mut sim)?;
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(at_probe, partial_run(&mut sim)?);
    assert_eq!(at_probe.1.refused, Some(Refusal::InsufficientProbe));
    Ok(())
}

#[test]
fn sensor_replay_and_independent_evaluator_detect_different_sources_of_error() -> Result<()> {
    let xml = Trial::Light.xml();
    let sdf = sdformat::load_str(&xml, "unknown-cable.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "unknown-cable.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let (mut rows, est) = run(&mut sim, &sdf, Trial::Light, 2000, None)?;
    check_sensor_replay(2000, &rows, &est)?;
    qualification::measure("light", Trial::Light, 2000, &rows, &est)?;
    assert!(qualification::measure("incorrect payload", Trial::Heavy, 2000, &rows, &est).is_err());
    assert!(qualification::measure("expected refusal", Trial::NoLift, 2000, &rows, &est).is_err());
    let mut wrong = est.clone();
    wrong.mass_kg = Some(0.1);
    assert!(check_sensor_replay(2000, &rows, &wrong).is_err());
    rows[0].stage = Stage::Opening;
    assert!(check_sensor_replay(2000, &rows, &est).is_err());
    rows[0].stage = Stage::Pickup;
    rows[0].signal.raw_drag_n += 1.;
    assert!(check_sensor_replay(2000, &rows, &est).is_err());
    rows[0].signal.raw_drag_n -= 1.;
    // A fictitious obstacle impulse affects the independent safety verdict,
    // while identical gripper readings still select the same robot commands.
    rows[0].rigid_pad_bar_impulse_ns = Some([0., 1.]);
    check_sensor_replay(2000, &rows, &est)?;
    assert!(
        qualification::measure("unexpected rigid strike", Trial::Light, 2000, &rows, &est).is_err()
    );
    Ok(())
}
#[test]
fn incomplete_force_estimate_cannot_publish_a_comparison_chart() {
    let dir = tempfile::tempdir().unwrap();
    let dst = dir.path().join("comparison.svg");
    let missing = Calibration::default().finish();
    assert!(plot::write(&dst, &[("incomplete", 0.02, missing)]).is_err());
    assert!(record(dir.path()).is_err());
}

#[test]
fn missing_rail_encoder_or_occupied_recording_destination_propagates_an_error() -> Result<()> {
    let xml = Trial::Slippery.xml();
    let sdf = sdformat::load_str(&xml, "weigh.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "weigh.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let robot = Robot::load(&sim, &sdf)?;
    sim.world_mut()
        .entity_mut(robot.rail.entity)
        .remove::<JointSample>();
    assert!(Calibration::drive(&mut sim, &robot, START + 1).is_err());
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    let occupied = tempfile::tempdir()?;
    let sentinel = occupied.path().join("prior-receipt");
    std::fs::write(&sentinel, "original")?;
    assert!(run(&mut sim, &sdf, Trial::Slippery, 4000, Some(occupied.path())).is_err());
    assert_eq!(std::fs::read_to_string(sentinel)?, "original");
    assert!(!occupied.path().join("estimate.json").exists());
    Ok(())
}
