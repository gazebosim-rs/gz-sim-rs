// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optimized public-step latency during repeated rail-stop impacts with a swinging wrist.
#![allow(deprecated)]
use gz_sim_core::EntityId;
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force};
use serde_json::json;
use std::{error::Error, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let mut runs = Vec::new();
    for repeat in 0..12 {
        let mut sim = load_world_with_sdf_step(
            include_str!("../../../examples/worlds/local_robot_joints.sdf"),
            "local_robot_joints.sdf",
            "robot_joints",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let slider = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "robot_joints::robot::slide")
            })
            .and_then(|e| e.get::<EntityId>())
            .ok_or("missing slider")?
            .get();
        let mut times = Vec::with_capacity(1500);
        for force in [12., -12.] {
            set_joint_force(&mut sim, slider, force)?;
            for _ in 0..750 {
                let start = Instant::now();
                sim.step_exact(1)?;
                times.push(start.elapsed().as_nanos() as u64);
            }
        }
        if repeat > 0 {
            times.sort_unstable();
            runs.push(times[(95 * times.len()).div_ceil(100) - 1]);
        }
    }
    let mut sorted = runs.clone();
    sorted.sort_unstable();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "step_ms":1,"steps_per_run":1500,"warmup_runs":1,"p95_step_ns":runs,
            "median_p95_step_ns":sorted[sorted.len()/2],
        }))?
    );
    Ok(())
}
