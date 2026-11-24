// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
/// Debounce loss of cable weight over forward travel. A brief load spike on
/// the edge of a grazing contact can be shorter than one millimetre of motion.
pub(super) const UNLOAD_TRAVEL_M: f64 = 0.0007;
#[derive(Default)]
pub(super) struct SupportLoss {
    travel_m: f64,
}
impl SupportLoss {
    pub(super) fn observe(&mut self, stage: Stage, r: &JawReadings, s: &Signal) -> f64 {
        self.observe_with_weight(stage, r, s, WEIGHT)
    }
    pub(super) fn observe_with_weight(
        &mut self,
        stage: Stage,
        r: &JawReadings,
        s: &Signal,
        weight_n: f64,
    ) -> f64 {
        if stage == Stage::Transport
            && r.force_on_payload_n[2] < 0.75 * weight_n
            && s.drag_n > 0.003
            && r.rail_speed_m_s > 0.
            && r.squeeze_n.iter().all(|n| *n > 0.3)
        {
            self.travel_m += r.rail_speed_m_s * DT;
        } else {
            self.travel_m = 0.;
        }
        self.travel_m
    }
}

use gz_physics::{BodyId, PhysicsBackend};
use gz_sim_scene::BodyHandle;
pub(super) struct PadContact {
    bar: BodyId,
    pads: [BodyId; 2],
}
impl PadContact {
    pub(super) fn new(sim: &Simulation, r: &Robot) -> Result<Self> {
        let read = |entity| {
            sim.world()
                .get::<BodyHandle>(entity)
                .map(BodyHandle::id)
                .ok_or("no native rigid link")
        };
        let bar = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::obstacle::bar")
            })
            .ok_or("no rigid obstacle")?
            .id();
        Ok(Self {
            bar: read(bar)?,
            pads: [read(r.pads[0].entity)?, read(r.pads[1].entity)?],
        })
    }
    pub(super) fn read(&self, sim: &Simulation) -> Result<[f64; 2]> {
        let solver = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
            .ok_or("missing native solver")?;
        let mut impulses = [0.; 2];
        for (out, pad) in impulses.iter_mut().zip(self.pads) {
            *out = solver
                .normal_reaction_on(pad, self.bar)?
                .ok_or("missing pad/bar reaction")?
                .magnitude;
        }
        Ok(impulses)
    }
}
#[path = "braking_cost.rs"]
mod cost;
#[path = "braking_plot.rs"]
mod plot;
#[path = "braking_qualification.rs"]
mod qualification;

pub(crate) fn record(out: &Path) -> Result<()> {
    std::fs::create_dir(out)?;
    let mut reports = vec![];
    let mut original = [vec![], vec![]];
    for (name, fixture, xml, active) in [
        ("original", Fixture::Limit, Scene::TooHigh.xml(), false),
        ("tall", Fixture::Limit, Scene::TooHigh.xml(), true),
        ("original-fast", Fixture::Fast, Scene::TooHigh.xml(), false),
        ("snag", Fixture::Snag, Fixture::Snag.xml(), true),
        ("rub", Fixture::Rub, Fixture::Rub.xml(), true),
        ("fast", Fixture::Fast, Fixture::Fast.xml(), true),
        ("lost", Fixture::Lost, Fixture::Lost.xml(), true),
        ("clear", Fixture::Clear, Fixture::Clear.xml(), true),
        ("fast-tall", Fixture::Fast, Scene::TooHigh.xml(), true),
    ] {
        let sdf = sdformat::load_str(&xml, "braking.sdf", &sdformat::NoResources)?;
        let mut sim = load_world_with_sdf_step(
            &xml,
            "braking.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let rows = monitored_run(&mut sim, &sdf, fixture, Some(&out.join(name)), Some(active))?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        let repeat = monitored_run(&mut sim, &sdf, fixture, None, Some(active))?;
        if rows != repeat {
            return Err("brake or material trajectory diverged after reset".into());
        }
        let report = qualification::measure(name, &rows)?;
        let cadence = usize::from(fixture == Fixture::Fast);
        if !active {
            original[cadence] = rows;
        } else if matches!(name, "tall" | "fast-tall") {
            let affected = rows
                .iter()
                .position(|r| r.stage == Stage::Backoff)
                .ok_or("missed retreat")?;
            if original[cadence].get(..affected) != Some(&rows[..affected]) {
                return Err("unmatched incoming state or rail inputs before active braking".into());
            }
            if name == "tall" {
                plot::write(&out.join("rigid-clearance.svg"), &original[cadence], &rows)?;
            }
        } else if matches!(name, "fast" | "rub" | "clear") {
            gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
            let prior = monitored_run(&mut sim, &sdf, fixture, None, Some(false))?;
            if rows != prior {
                return Err("unnecessary braking changed a clear route or harmless rub".into());
            }
        }
        reports.push(report);
    }
    std::fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(&reports)?,
    )?;
    std::fs::write(
        out.join("timing.json"),
        serde_json::to_vec_pretty(&cost::measure()?)?,
    )?;
    Ok(())
}

#[cfg(test)]
#[path = "braking_tests.rs"]
mod tests;
