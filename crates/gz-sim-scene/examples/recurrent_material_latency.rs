// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
#[allow(dead_code)]
#[path = "support/rotating_pad.rs"]
mod rotor;
use gz_sim_core::Simulation;
use gz_sim_scene::{deformable, reset_loaded_world};
use std::{error::Error, time::Instant};
const STATIC: &str = include_str!("../../../examples/articulated_recurrent_cloth.sdf");
const TURNING: &str = include_str!("../../../examples/rotating_recurrent_cloth.sdf");
fn bench(sim: &mut Simulation, name: &str, repeats: usize) -> Result<(), Box<dyn Error>> {
    let mut times: [Vec<u64>; 3] = Default::default();
    for run in 0..repeats + 2 {
        if run > 0 {
            reset_loaded_world(sim, |_| ())?;
        }
        sim.step_exact(210)?;
        for _ in 210..285 {
            let now = Instant::now();
            sim.step_exact(1)?;
            let nanos = now.elapsed().as_nanos() as u64;
            let s = deformable::bodies(sim).next().unwrap().1;
            if run >= 2 {
                times[if s.articulated_impact_count() == 0 {
                    0
                } else if s.impacts_this_step().is_empty() {
                    1
                } else {
                    2
                }]
                .push(nanos);
            }
        }
    }
    for (n, mut v) in ["analytic approach", "deforming clear", "contact"]
        .into_iter()
        .zip(times)
    {
        v.sort_unstable();
        let q = |p: usize| v[(p * v.len()).div_ceil(100) - 1];
        println!(
            "{name}, {n}, {} complete 1 ms Bevy ticks, p50/p95/p99 {} / {} / {} ns",
            v.len(),
            q(50),
            q(95),
            q(99)
        );
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let repeats = std::env::args()
        .nth(1)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(12);
    let mut g = fixture::Gripper::new(STATIC)?;
    bench(&mut g.sim, "two-finger sheet", repeats)?;
    bench(
        &mut rotor::Rotor::new(TURNING)?.sim,
        "spinning pad and sheet",
        repeats,
    )
}
