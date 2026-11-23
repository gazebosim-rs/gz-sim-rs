// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local Bevy step timings for a supported payload, separately from initialization and readback.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ArticulationTier, Name, PhysicsEngine, contact_promotion, load_world_with_sdf_step,
    model_fidelity, queue_link_world_velocity, set_model_fidelity,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{error::Error, time::Instant};

const SCENE: &str = include_str!("../../../examples/multifidelity_sliding_payload.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn id(sim: &Simulation, name: &str) -> Result<u64> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<EntityId>().map(|id| id.get()))
        .ok_or_else(|| format!("missing {name}").into())
}

struct Timings {
    steps: Vec<u64>,
    dynamic_steps: usize,
    first_promotion: Option<usize>,
}

impl Timings {
    fn finish(mut self, repeats: usize) -> Value {
        self.steps.sort_unstable();
        let quantile = |percent: usize| self.steps[(self.steps.len() * percent).div_ceil(100) - 1];
        json!({"samples":self.steps.len(), "p50_ns":quantile(50), "p95_ns":quantile(95),
            "p99_ns":quantile(99), "mean_ns":self.steps.iter().sum::<u64>() as f64/self.steps.len() as f64,
            "first_dynamic_tick":self.first_promotion, "dynamic_steps_per_run":self.dynamic_steps / repeats})
    }
}

fn measure(source: &str, guarded: bool, times: &mut Timings, record: bool) -> Result<()> {
    let mut sim = load_world_with_sdf_step(
        source,
        "cost.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    sim.step_exact(2000)?;
    let arm = id(&sim, "default::arm")?;
    let ram = id(&sim, "default::ram::head")?;
    if guarded {
        set_model_fidelity(&mut sim, arm, ArticulationTier::Prescribed)?;
        contact_promotion::configure(&mut sim, arm, vec![ram])?;
    }
    queue_link_world_velocity(&mut sim, ram, [4., 0., 0.], [0.; 3])?;
    for tick in 1..=500 {
        let start = Instant::now();
        sim.step_exact(1)?;
        let ns = start.elapsed().as_nanos() as u64;
        if record {
            times.steps.push(ns);
            if model_fidelity(&sim, arm)? == ArticulationTier::Dynamic {
                times.dynamic_steps += 1;
                times.first_promotion.get_or_insert(tick);
            }
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let repeats: usize = std::env::args().nth(1).as_deref().unwrap_or("9").parse()?;
    if repeats == 0 {
        return Err("repeat count must be positive".into());
    }
    let mut rows = Vec::new();
    for (layout, offset, clutter) in [
        ("hit", 0.71, 0),
        ("near_miss", 1.15, 0),
        ("cluttered_miss", 1.15, 32),
    ] {
        let mut extra = String::new();
        for i in 0..clutter {
            extra.push_str(&format!(r#"<model name="fixed{i}"><static>true</static><pose>{} 5 1 0 0 0</pose><link name="body"><collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link></model>"#, 4+i));
        }
        let source = SCENE
            .replace("-1.1 0.71 0.17", &format!("-1.1 {offset} 0.17"))
            .replace("</world>", &format!("{extra}</world>"));
        let mut lanes = [false, true].map(|_| Timings {
            steps: Vec::with_capacity(repeats * 500),
            dynamic_steps: 0,
            first_promotion: None,
        });
        for run in 0..=repeats {
            for &i in if run % 2 == 0 { &[0, 1] } else { &[1, 0] } {
                measure(&source, i == 1, &mut lanes[i], run > 0)?;
            }
        }
        let [reference, automatic] = lanes.map(|v| v.finish(repeats));
        rows.push(json!({"layout":layout,"distant_fixed_bodies":clutter,"automatic":automatic,"always_dynamic":reference}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"repeats":repeats,"dt_ms":1,"rows":rows}))?
    );
    Ok(())
}
