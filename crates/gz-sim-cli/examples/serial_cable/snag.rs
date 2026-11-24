// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "snag/controller.rs"]
mod controller;
#[path = "snag/qualification.rs"]
mod qualification;
use controller::{Controller, Event, Stage};
use serde_json::{Value, json};
const MAX_TICKS: usize = 30000;
#[derive(Clone, Copy)]
enum Scene {
    Snag,
    Clear,
    TooHigh,
}
impl Scene {
    fn xml(self) -> String {
        // The bar always starts on the pickup table top. Only the test
        // fixture knows the obstacle's height and lateral location.
        let (x, height) = match self {
            Self::Snag => (0., 0.145),
            Self::Clear => (0.60, 0.145),
            Self::TooHigh => (0., 0.20),
        };
        SDF.replace("0 0.23 0.5535", &format!("{x} 0.23 {}", 0.50 + height / 2.))
            .replace("0.26 0.024 0.107", &format!("0.26 0.024 {height}"))
    }
}
#[derive(PartialEq, Serialize)]
struct Trace {
    tick: usize,
    stage: Stage,
    y: f64,
    sample: Sample,
}
fn simulate(
    sim: &mut Simulation,
    sdf: &sdformat::Sdf,
    out: Option<&Path>,
) -> Result<(Vec<Trace>, Vec<Event>)> {
    let robot = Robot::load(sim, sdf)?;
    let mut ctl = Controller::new();
    let mut history = vec![];
    let mut frames = vec![];
    let mut coordinates = vec![];
    if out.is_some() {
        capture(sim, sdf, &mut frames, &mut coordinates)?;
    }
    let mut keys = std::collections::BTreeMap::new();
    keys.insert("pickup".to_string(), 0);
    let mut momentum = DVec3::ZERO;
    for tick in 1..=MAX_TICKS {
        ctl.drive(&robot, sim, tick)?;
        sim.step_exact(1)
            .map_err(|e| format!("tick {tick}: {e} {:?}", sim.fault()))?;
        let s = sample(sim, &robot, momentum)?;
        momentum = s.momentum.into();
        let y = sim
            .world()
            .get::<JointSample>(robot.rail.entity)
            .ok_or("missing rail")?
            .state
            .position;
        let stage = ctl.stage;
        ctl.observe(tick, y, &s)?;
        let changed = ctl.stage != stage;
        if changed {
            eprintln!("{tick}: {:?} at {y:.4}, bar {:.5} N", ctl.stage, s.bar);
        }
        history.push(Trace {
            tick,
            stage,
            y,
            sample: s,
        });
        let done = ctl.done(tick);
        if out.is_some() && (tick % 100 == 0 || changed || done) {
            if changed {
                keys.insert(
                    format!("event-{:02}-{:?}", ctl.events.len(), ctl.stage),
                    frames.len(),
                );
            }
            if done {
                keys.insert("rest".to_string(), frames.len());
            }
            capture(sim, sdf, &mut frames, &mut coordinates)?;
        }
        if done {
            break;
        }
    }
    if !ctl.done(history.len()) {
        return Err(format!("incomplete recovery at {:?}, {:?}", ctl.stage, ctl.events).into());
    }
    if let Some(d) = out {
        std::fs::create_dir(d)?;
        gz_sim_cli::export_archive(&d.join("replay"), Default::default(), frames)?;
        for (name, data) in [
            (
                "frames.json",
                json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"front","particle_visuals":(0..11).map(|i|format!("default::stand::post::cable_{i}")).collect::<Vec<_>>(),"captures":keys}),
            ),
            ("particle-frames.json", json!(coordinates)),
            ("events.json", json!(ctl.events)),
        ] {
            std::fs::write(d.join(name), serde_json::to_vec(&data)?)?;
        }
        std::fs::write(d.join("trace.json"), serde_json::to_vec(&history)?)?;
    }
    Ok((history, ctl.events))
}
pub(super) fn record(out: &Path) -> Result<()> {
    std::fs::create_dir(out)?;
    let mut reports = Vec::<Value>::new();
    let mut baseline: Vec<String> = Vec::new();
    for (name, scene) in [
        ("clear", Scene::Clear),
        ("snag", Scene::Snag),
        ("limit", Scene::TooHigh),
    ] {
        let xml = scene.xml();
        let sdf = sdformat::load_str(&xml, "cable-snag.sdf", &sdformat::NoResources)?;
        let mut sim = load_world_with_sdf_step(
            &xml,
            "cable-snag.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let (observed, events) = simulate(&mut sim, &sdf, Some(&out.join(name)))?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        let (repeat, reevents) = simulate(&mut sim, &sdf, None)?;
        if observed != repeat || events != reevents {
            return Err("restarted grasp trajectory diverged".into());
        }
        let measured = qualification::measure(scene, &observed, &events)?;
        let first = events
            .iter()
            .find(|e| e.stage == Stage::Backoff)
            .map(|e| e.tick);
        if let Some(t) = first {
            if !observed[..t - 1]
                .iter()
                .map(|v| &v.sample.particle_state_sha256)
                .eq(baseline[..t - 1].iter())
                || observed[t - 1].sample.particle_state_sha256 == baseline[t - 1]
            {
                return Err("unmatched cable state before the obstruction".into());
            }
        } else if matches!(scene, Scene::Clear) {
            baseline = observed
                .iter()
                .map(|r| r.sample.particle_state_sha256.clone())
                .collect();
        }
        reports.push(json!({"case":name,"reset_identical_ticks":observed.len(),"matched_clear_prefix_ticks":first.map(|t|t-1),"events":events,"measurements":measured}));
    }
    std::fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(&reports)?,
    )?;
    Ok(())
}

#[cfg(test)]
#[path = "snag/tests.rs"]
mod tests;
