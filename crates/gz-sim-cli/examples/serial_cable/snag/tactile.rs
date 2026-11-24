// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "tactile/sensor.rs"]
mod sensor;
use sensor::{JawReadings, Observer, Signal};
#[derive(Clone, Copy, PartialEq)]
enum Fixture {
    Clear,
    Fast,
    Snag,
    Rub,
    Limit,
    Lost,
}
impl Fixture {
    fn name(self) -> &'static str {
        match self {
            Self::Clear => "clear",
            Self::Fast => "fast",
            Self::Snag => "snag",
            Self::Rub => "rub",
            Self::Limit => "limit",
            Self::Lost => "lost",
        }
    }
    fn xml(self) -> String {
        let (x, h) = match self {
            Self::Clear | Self::Fast | Self::Lost => (0.60, 0.145),
            Self::Snag => (0., 0.145),
            Self::Rub => (0., 0.1165),
            Self::Limit => (0., 0.16),
        };
        obstacle_xml(x, h)
    }
}
fn obstacle_xml(x: f64, height: f64) -> String {
    SDF.replace("0 0.23 0.5535", &format!("{x} 0.23 {}", 0.50 + height / 2.))
        .replace("0.26 0.024 0.107", &format!("0.26 0.024 {height}"))
}
#[derive(Serialize, PartialEq)]
struct Log {
    tick: usize,
    stage: Stage,
    jaws: JawReadings,
    signal: Signal,
    truth: Sample,
    #[serde(skip_serializing_if = "Option::is_none")]
    rail_command_n: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    unloaded_travel_m: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    rigid_pad_bar_impulse_ns: Option<[f64; 2]>,
}
fn run(
    sim: &mut Simulation,
    sdf: &sdformat::Sdf,
    fixture: Fixture,
    out: Option<&Path>,
) -> Result<Vec<Log>> {
    monitored_run(sim, sdf, fixture, out, None)
}
fn monitored_run(
    sim: &mut Simulation,
    sdf: &sdformat::Sdf,
    fixture: Fixture,
    out: Option<&Path>,
    active_braking: Option<bool>,
) -> Result<Vec<Log>> {
    let robot = Robot::load(sim, sdf)?;
    let mut route =
        Controller::with_travel_ticks(if fixture == Fixture::Fast { 2000 } else { 4000 });
    route.active_braking = active_braking.unwrap_or(false);
    let mut support = braking::SupportLoss::default();
    let rigid_pair = active_braking
        .map(|_| braking::PadContact::new(sim, &robot))
        .transpose()?;
    let mut sensor = Observer::default();
    let mut frames = vec![];
    let mut coords = vec![];
    let mut rows = vec![];
    let mut prior = DVec3::ZERO;
    let mut keys = std::collections::BTreeMap::new();
    keys.insert("start".to_owned(), 0);
    if out.is_some() {
        capture(sim, sdf, &mut frames, &mut coords)?;
    }
    for tick in 1..=MAX_TICKS {
        route.drive(&robot, sim, tick)?;
        if fixture == Fixture::Lost && tick >= 6050 {
            // Failure of one closing drive. The observer receives no fault bit.
            JointForceCommand::set_batch(sim, [(robot.jaws[0].id, -16.)])?;
        }
        let rail_command_n = active_braking.map(|_| {
            sim.world()
                .get::<JointForceCommand>(robot.rail.entity)
                .expect("commanded rail")
                .torque()
        });
        sim.step_exact(1)
            .map_err(|e| format!("tick {tick}: {e} {:?}", sim.fault()))?;
        let jaws = JawReadings::read(sim, &robot)?;
        let (mut feed, signal) = sensor.update(&jaws);
        let unloaded_travel_m = active_braking.map(|active| {
            let distance = support.observe(route.stage, &jaws, &signal);
            feed.snag |= active && distance >= braking::UNLOAD_TRAVEL_M;
            distance
        });
        let stage = route.stage;
        route.apply(tick, jaws.rail_y_m, &feed)?;
        // Independent evaluator reads material truth only after the robot's decision.
        let truth = sample(sim, &robot, prior)?;
        let rigid_pad_bar_impulse_ns = rigid_pair.as_ref().map(|r| r.read(sim)).transpose()?;
        prior = truth.momentum.into();
        let change = stage != route.stage;
        if change {
            eprintln!(
                "{} {tick} {:?} drag {:.5} bar {:.5} up {:.4}",
                fixture.name(),
                route.stage,
                signal.drag_n,
                truth.bar,
                jaws.force_on_payload_n[2]
            );
        }
        let first_rigid_contact = rigid_pad_bar_impulse_ns
            .is_some_and(|j| j.into_iter().any(|n| n > 1e-12))
            && !keys.contains_key("first-rigid-contact");
        let first_cable_contact = active_braking.is_some()
            && truth.bar > 1e-8
            && !keys.contains_key("first-cable-contact");
        rows.push(Log {
            tick,
            stage,
            jaws,
            signal,
            truth,
            rail_command_n,
            unloaded_travel_m,
            rigid_pad_bar_impulse_ns,
        });
        let done = route.done(tick);
        if out.is_some()
            && (tick % 100 == 0 || change || done || first_rigid_contact || first_cable_contact)
        {
            if first_rigid_contact {
                keys.insert("first-rigid-contact".to_owned(), frames.len());
            }
            if first_cable_contact {
                keys.insert("first-cable-contact".to_owned(), frames.len());
            }
            if change {
                keys.insert(format!("event-{tick}-{:?}", route.stage), frames.len());
            }
            if done {
                keys.insert("rest".to_owned(), frames.len());
            }
            capture(sim, sdf, &mut frames, &mut coords)?;
        }
        if done {
            break;
        }
    }
    if !route.done(rows.len()) {
        return Err(format!(
            "controller did not complete in {:?} {:?}",
            route.stage, route.events
        )
        .into());
    }
    if let Some(d) = out {
        write_replay(d, &rows, frames, coords, keys)?;
    }
    Ok(rows)
}
fn write_replay(
    d: &Path,
    rows: &[Log],
    frames: Vec<(String, String)>,
    coords: Vec<Value>,
    keys: std::collections::BTreeMap<String, usize>,
) -> Result<()> {
    std::fs::create_dir(d)?;
    gz_sim_cli::export_archive(&d.join("replay"), Default::default(), frames)?;
    std::fs::write(d.join("trace.json"), serde_json::to_vec(&rows)?)?;
    std::fs::write(d.join("particle-frames.json"), serde_json::to_vec(&coords)?)?;
    std::fs::write(
        d.join("frames.json"),
        serde_json::to_vec(
            &json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"front","particle_visuals":(0..11).map(|i|format!("default::stand::post::cable_{i}")).collect::<Vec<_>>(),"captures":keys}),
        )?,
    )?;
    Ok(())
}
fn trial_order() -> [Fixture; 6] {
    [
        Fixture::Clear,
        Fixture::Fast,
        Fixture::Snag,
        Fixture::Rub,
        Fixture::Limit,
        Fixture::Lost,
    ]
}
#[path = "tactile/braking.rs"]
pub(crate) mod braking;
#[path = "tactile/plot.rs"]
mod plot;
#[path = "tactile/qualification.rs"]
mod qualification;
#[path = "tactile/weighing.rs"]
pub(crate) mod weighing;
pub(crate) fn record(out: &Path) -> Result<()> {
    std::fs::create_dir(out)?;
    let mut baseline = vec![];
    let mut cases = vec![];
    let mut curves = vec![];
    for fixture in trial_order() {
        let xml = fixture.xml();
        let sdf = sdformat::load_str(&xml, "gripper-tactile.sdf", &sdformat::NoResources)?;
        let mut sim = load_world_with_sdf_step(
            &xml,
            "gripper-tactile.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let rows = run(&mut sim, &sdf, fixture, Some(&out.join(fixture.name())))?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        if rows != run(&mut sim, &sdf, fixture, None)? {
            return Err("gripper feedback or material state diverged after reset".into());
        }
        if fixture == Fixture::Clear {
            baseline = rows
                .iter()
                .map(|r| r.truth.particle_state_sha256.clone())
                .collect();
        }
        cases.push(qualification::measure(fixture, &rows, &baseline)?);
        curves.extend(plot::Series::from_run(fixture, &rows));
    }
    plot::write(&out.join("gripper-drag.svg"), &curves)?;
    std::fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(&cases)?,
    )?;
    Ok(())
}

#[cfg(test)]
#[path = "tactile/tests.rs"]
mod tests;
