// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the native contact patch and public-step cost for the two archived 84 ms drops.
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf, time::Instant};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const IMPACT: usize = 169;
const REPEATS: usize = 41;

#[derive(Default)]
struct Profile {
    flight: Vec<u64>,
    impact: Vec<u64>,
    loaded: Vec<u64>,
    patch: Option<Value>,
}
fn stats(times: &mut [u64]) -> Value {
    times.sort_unstable();
    json!({
        "samples": times.len(),
        "median_ns": times[times.len() / 2],
        "p95_ns": times[(95 * times.len()).div_ceil(100) - 1],
    })
}
impl Profile {
    fn run(&mut self, xml: &str, warmup: bool) -> Result<()> {
        let mut sim = load_world_with_sdf_step(
            xml,
            "ground.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        for tick in 1..=360 {
            let start = Instant::now();
            sim.step_exact(1)?;
            let elapsed = start.elapsed().as_nanos() as u64;
            if warmup {
                continue;
            }
            if tick == IMPACT {
                self.impact.push(elapsed);
                if self.patch.is_none() {
                    let backend = sim
                        .world()
                        .resource::<PhysicsBackend>()
                        .downcast_ref::<MultiFidelityBackend>()
                        .ok_or("unexpected physics engine")?;
                    let mut patches = Vec::new();
                    for patch in backend.rigid_contact_patches() {
                        let points: Vec<_> = patch
                            .points
                            .iter()
                            .map(|point| {
                                json!({
                                    "lever_a": point.lever_a,
                                    "lever_b": point.lever_b,
                                    "normal_ns": point.normal_impulse,
                                    "tangent_on_a_ns": point.tangent_impulse_on_a,
                                })
                            })
                            .collect();
                        patches.push(json!({
                            "a": patch.a.raw(),
                            "b": patch.b.raw(),
                            "normal_on_a": patch.normal_on_a,
                            "points": points,
                        }));
                    }
                    if patches.is_empty() {
                        return Err("expected the first contact patch at 84.5 ms".into());
                    }
                    self.patch = Some(Value::Array(patches));
                }
            } else if (10..IMPACT).contains(&tick) {
                self.flight.push(elapsed)
            } else if (240..=360).contains(&tick) {
                self.loaded.push(elapsed)
            }
        }
        Ok(())
    }
    fn summary(mut self) -> Value {
        json!({
            "flight": stats(&mut self.flight),
            "first_impact": stats(&mut self.impact),
            "settled": stats(&mut self.loaded),
            "first_contact_patches": self.patch,
        })
    }
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let sources = PathBuf::from(
        args.next()
            .ok_or("usage: weld_impact_profile SDF_DIR OUTPUT_JSON")?,
    );
    let out = PathBuf::from(args.next().ok_or("missing output JSON")?);
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let xml = [
        fs::read_to_string(sources.join("ground-single.sdf"))?,
        fs::read_to_string(sources.join("ground-welded.sdf"))?,
    ];
    let mut profiles = [Profile::default(), Profile::default()];
    for repeat in 0..=REPEATS {
        for i in [repeat % 2, 1 - repeat % 2] {
            profiles[i].run(&xml[i], repeat == 0)?;
        }
    }
    let [single, weld] = profiles.map(Profile::summary);
    let report = json!({
        "public_step_us": 500,
        "repetitions": REPEATS,
        "single": single,
        "weld": weld,
    });
    fs::write(out, serde_json::to_vec_pretty(&report)?)?;
    Ok(())
}
