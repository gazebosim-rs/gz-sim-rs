// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure swept compound collisions through the regular local SDF loader and Bevy step.
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{error::Error, time::Instant};

const COURSE: &str = include_str!("../../../examples/local_collision_course.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[allow(deprecated)]
fn id(sim: &Simulation, name: &str) -> Result<BodyId> {
    Ok(sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|s| s.0 == name))
        .and_then(|e| e.get::<BodyHandle>())
        .ok_or("missing link")?
        .id())
}

fn run(xml: &str, times: &mut [Vec<u64>; 3], sample: bool) -> Result<Vec<Value>> {
    let mut sim = load_world_with_sdf_step(
        xml,
        "collision-course.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mover = id(&sim, "default::hit::carriage")?;
    let clear = id(&sim, "default::clear::carriage")?;
    let mut trace = Vec::new();
    for (i, values) in times.iter_mut().enumerate() {
        let timer = Instant::now();
        sim.step_exact(1)?;
        let elapsed = timer.elapsed().as_nanos() as u64;
        if !sample {
            continue;
        }
        values.push(elapsed);
        let physics = sim.world().resource::<PhysicsBackend>();
        let state = physics.state(mover).ok_or("missing state")?;
        let miss = physics.state(clear).ok_or("missing state")?;
        let native = physics
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("wrong engine")?;
        let mut hit_impulse = 0.;
        let mut miss_impulse = 0.;
        for pair in native.rigid_contact_observations() {
            if pair.a == mover || pair.b == mover {
                hit_impulse += pair.normal_impulse;
            }
            if pair.a == clear || pair.b == clear {
                miss_impulse += pair.normal_impulse;
            }
        }
        trace.push(json!({
            "time_ms":(i+1)*10,"hit_x_m":state.position[0],"clear_x_m":miss.position[0],
            "hit_vx_m_s":state.velocity[0],"clear_vx_m_s":miss.velocity[0],
            "hit_normal_impulse_ns":hit_impulse,"clear_normal_impulse_ns":miss_impulse
        }));
    }
    Ok(trace)
}

fn main() -> Result<()> {
    let start = COURSE
        .find("    <plugin filename=\"gz-sim-continuous-collision-system")
        .ok_or("CCD plugin")?;
    let end = start + COURSE[start..].find("</plugin>").ok_or("CCD end")? + "</plugin>".len();
    let discrete = format!("{}{}", &COURSE[..start], &COURSE[end..]);
    let scenes = [discrete.as_str(), COURSE];
    let mut times: [[Vec<u64>; 3]; 2] = Default::default();
    let mut traces: [Vec<Value>; 2] = Default::default();
    const REPEATS: usize = 101;
    for pass in 0..=REPEATS {
        for index in [pass % 2, 1 - pass % 2] {
            traces[index] = run(scenes[index], &mut times[index], pass > 0)?;
        }
    }
    let timing = times.map(|rows| {
        rows.map(|mut row| {
            row.sort_unstable();
            row[(row.len() * 95).div_ceil(100) - 1] as f64 / 1000.
        })
    });
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "repetitions":REPEATS,"timing":"p95 public step wall time in microseconds; timing excludes setup and readback",
            "control":{"trace":traces[0],"p95_us":timing[0]},"swept":{"trace":traces[1],"p95_us":timing[1]},
            "analytical_time_of_impact_ms":6.34,"analytical_first_contact_link_x_m":-0.216,
            "ccd_clamp_time_equivalent_ms":(traces[1][0]["hit_x_m"].as_f64().unwrap()+0.85)/100.*1000.,
        }))?
    );
    Ok(())
}
