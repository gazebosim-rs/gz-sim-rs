// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "weighing/identify.rs"]
mod identify;
use identify::{Calibration, END as CAL_END, Estimate};
#[derive(Clone, Copy, PartialEq)]
enum Trial {
    Nominal,
    Light,
    Heavy,
    Rub,
    Slippery,
    ProbeSlip,
    Assisted,
    NoLift,
    StalledProbe,
    Pulsed,
    Lost,
}
impl Trial {
    fn parameters(self) -> (f64, f64) {
        match self {
            Self::Light => (0.001, 0.5),
            Self::Heavy => (0.003, 0.65),
            Self::Slippery => (0.002, 0.),
            _ => (0.002, 0.7),
        }
    }
    fn xml(self) -> String {
        let (m, mu) = self.parameters();
        let xml = match self {
            Self::Nominal | Self::Heavy => Scene::TooHigh.xml(),
            Self::Rub => obstacle_xml(0., 0.1169),
            _ => Fixture::Clear.xml(),
        };
        xml.replace("mass=\"0.002\"", &format!("mass=\"{m}\""))
            .replace(
                "friction=\"0.7\" stretch",
                &format!("friction=\"{mu}\" stretch"),
            )
    }
    fn setup(self, sim: &mut Simulation, robot: &Robot, tick: usize) -> Result<[f64; 3]> {
        let mut f = [0.; 3];
        if self == Self::Assisted && tick >= 450 {
            f[2] = 0.075;
            deformable::set_material_force(sim, "cable", f)?;
        }
        if self == Self::Pulsed && (5001..=5400).contains(&tick) {
            f[2] = if tick % 100 < 50 { 0.12 } else { -0.12 };
            deformable::set_material_force(sim, "cable", f)?;
        }
        if self == Self::StalledProbe && (identify::START + 1..=CAL_END).contains(&tick) {
            JointForceCommand::set_batch(sim, [(robot.rail.id, 0.)])?;
        }
        if self == Self::NoLift && tick == 900 {
            set_joint_controller_command(sim, robot.spindle.id, 0.)?;
        }
        if (self == Self::Lost && tick >= CAL_END + 650)
            || (self == Self::ProbeSlip && tick >= 6000)
        {
            JointForceCommand::set_batch(sim, [(robot.jaws[0].id, -16.)])?;
        }
        Ok(f)
    }
}

/// All state used to decide whether the cable can travel. Neither the test
/// fixture nor the material evaluator is available at this interface.
struct WeighedRoute {
    route: Controller,
    calibration: Calibration,
    estimate: Option<Estimate>,
    observer: Observer,
    guard: braking::SupportLoss,
}
impl WeighedRoute {
    fn new(requested_ticks: usize) -> Self {
        let mut route = Controller::with_travel_ticks(requested_ticks);
        route.active_braking = true;
        route.pickup_end_tick = CAL_END;
        Self {
            route,
            calibration: Calibration::default(),
            estimate: None,
            observer: Observer::default(),
            guard: braking::SupportLoss::default(),
        }
    }
    fn observe(&mut self, tick: usize, jaws: &JawReadings) -> Result<(Signal, f64)> {
        if self.estimate.is_none() {
            self.calibration.observe(tick, jaws);
        }
        if self.estimate.is_none()
            && (tick == CAL_END
                || self.calibration.probe_lost()
                || (tick == identify::START && self.calibration.first_problem().is_some()))
        {
            let mut decision = self.calibration.finish();
            decision.travel_ticks = decision
                .mass_kg
                .map(|m| self.route.limit_payload_momentum(m));
            self.estimate = Some(decision);
        }
        let weight = self
            .estimate
            .as_ref()
            .and_then(|v| v.mass_kg)
            .unwrap_or_default()
            * 9.81;
        let (mut feed, signal) = self.observer.update_with_weight(jaws, weight);
        let stage = self.route.stage;
        let travel = self.guard.observe_with_weight(stage, jaws, &signal, weight);
        feed.snag |= travel >= braking::UNLOAD_TRAVEL_M;
        if stage == Stage::Pickup && self.estimate.as_ref().is_some_and(|v| v.refused.is_some()) {
            self.route.hold_unidentified(tick, jaws.rail_y_m, &feed);
        } else {
            self.route.apply(tick, jaws.rail_y_m, &feed)?;
        }
        Ok((signal, travel))
    }
}

fn run(
    sim: &mut Simulation,
    sdf: &sdformat::Sdf,
    trial: Trial,
    requested_ticks: usize,
    out: Option<&Path>,
) -> Result<(Vec<Log>, Estimate)> {
    let robot = Robot::load(sim, sdf)?;
    let mut blind = WeighedRoute::new(requested_ticks);
    let rigid = braking::PadContact::new(sim, &robot)?;
    let mut rows = vec![];
    let mut frames = vec![];
    let mut coords = vec![];
    let mut keys = std::collections::BTreeMap::from([("start".to_owned(), 0)]);
    if out.is_some() {
        capture(sim, sdf, &mut frames, &mut coords)?;
    }
    let mut momentum = DVec3::ZERO;
    for tick in 1..=MAX_TICKS + 4000 {
        blind.route.drive(&robot, sim, tick)?;
        if blind.route.stage == Stage::Pickup {
            Calibration::drive(sim, &robot, tick)?;
        }
        let external = trial.setup(sim, &robot, tick)?;
        sim.step_exact(1)?;
        let jaws = JawReadings::read(sim, &robot)?;
        let stage = blind.route.stage;
        let (signal, travel) = blind.observe(tick, &jaws)?;
        // Material truth and the externally applied test load are read only by the evaluator.
        let truth = sample_payload(sim, &robot, momentum, external, trial.parameters().1)?;
        momentum = truth.momentum.into();
        let rigid_pad_bar_impulse_ns = Some(rigid.read(sim)?);
        rows.push(Log {
            tick,
            stage,
            jaws,
            signal,
            truth,
            rail_command_n: sim
                .world()
                .get::<JointForceCommand>(robot.rail.entity)
                .map(JointForceCommand::torque),
            unloaded_travel_m: Some(travel),
            rigid_pad_bar_impulse_ns,
        });
        let done = blind.route.done(tick);
        let changed = stage != blind.route.stage;
        if out.is_some() && (changed || done || tick % 100 == 0) {
            if changed {
                keys.insert(
                    format!("event-{tick}-{:?}", blind.route.stage),
                    frames.len(),
                );
            }
            if tick == identify::START {
                keys.insert("weigh-start".into(), frames.len());
            }
            if tick == identify::START + 2000 {
                keys.insert("weigh-turn".into(), frames.len());
            }
            if done {
                keys.insert("end".into(), frames.len());
            }
            capture(sim, sdf, &mut frames, &mut coords)?;
        }
        if done {
            break;
        }
    }
    if !blind.route.done(rows.len()) {
        return Err("weight-calibrated route did not finish".into());
    }
    let estimate = blind.estimate.ok_or("missing calibration")?;
    if let Some(d) = out {
        write_replay(d, &rows, frames, coords, keys)?;
        std::fs::write(
            d.join("estimate.json"),
            serde_json::to_vec_pretty(&estimate)?,
        )?;
    }
    Ok((rows, estimate))
}
fn check_sensor_replay(requested_ticks: usize, rows: &[Log], est: &Estimate) -> Result<()> {
    let mut shadow = WeighedRoute::new(requested_ticks);
    for r in rows {
        if shadow.route.stage != r.stage {
            return Err("gripper-only observation replay changed the selected action".into());
        }
        let (signal, travel) = shadow.observe(r.tick, &r.jaws)?;
        if signal != r.signal || Some(travel) != r.unloaded_travel_m {
            return Err("gripper-only replay changed its load estimate".into());
        }
    }
    if shadow.estimate.as_ref() != Some(est) {
        return Err("gripper-only replay changed the mass decision".into());
    }
    Ok(())
}
pub(crate) fn record(out: &Path) -> Result<()> {
    std::fs::create_dir(out)?;
    let mut reports = vec![];
    let mut chart = vec![];
    for (trial, name, requested_ticks) in [
        (Trial::Nominal, "nominal", 4000),
        (Trial::Light, "light", 2000),
        (Trial::Heavy, "heavy", 2000),
        (Trial::Rub, "rub", 4000),
        (Trial::Slippery, "slick", 4000),
        (Trial::ProbeSlip, "probe-slip", 4000),
        (Trial::Assisted, "assisted", 4000),
        (Trial::NoLift, "no-lift", 4000),
        (Trial::StalledProbe, "stalled-probe", 4000),
        (Trial::Pulsed, "pulsed", 4000),
        (Trial::Lost, "lost", 4000),
        (Trial::Light, "light-normal", 4000),
    ] {
        let xml = trial.xml();
        let sdf = sdformat::load_str(&xml, "unknown-cable.sdf", &sdformat::NoResources)?;
        let mut sim = load_world_with_sdf_step(
            &xml,
            "unknown-cable.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let (rows, est) = run(
            &mut sim,
            &sdf,
            trial,
            requested_ticks,
            Some(&out.join(name)),
        )?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        let (repeat, r_est) = run(&mut sim, &sdf, trial, requested_ticks, None)?;
        if rows != repeat || est != r_est {
            return Err("calibration or subsequent motion diverged after reset".into());
        }
        check_sensor_replay(requested_ticks, &rows, &est)?;
        let label = match name {
            "nominal" => Some("Nominal"),
            "light" => Some("Light"),
            "heavy" => Some("Heavy"),
            "assisted" => Some("Partly supported"),
            _ => None,
        };
        if let Some(label) = label {
            chart.push((label, trial.parameters().0 * 11., est.clone()));
        }
        reports.push(qualification::measure(
            name,
            trial,
            requested_ticks,
            &rows,
            &est,
        )?);
    }
    plot::write(&out.join("weighing.svg"), &chart)?;
    std::fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(&reports)?,
    )?;
    Ok(())
}
#[path = "weighing/plot.rs"]
mod plot;
#[path = "weighing/qualification.rs"]
mod qualification;
#[cfg(test)]
#[path = "weighing/tests.rs"]
mod tests;
