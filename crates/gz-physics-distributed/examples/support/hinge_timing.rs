// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{END, NS_PER_MS, Result, Rig, Simulation, Value, advance_ms, json, setup};
use gz_physics::PhysicsBackend;
use sha2::{Digest, Sha256};
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};
pub fn endpoint(sim: &Simulation, rig: &Rig) -> Result<String> {
    let phy = sim.world().resource::<PhysicsBackend>();
    let mut digest = Sha256::new();
    for link in &rig.links {
        let b = phy.state(link.native).ok_or("benchmark body disappeared")?;
        for x in b
            .position
            .into_iter()
            .chain(b.orientation)
            .chain(b.velocity)
            .chain(b.angular_velocity)
        {
            digest.update(x.to_le_bytes());
        }
    }
    let hinge = phy
        .joint_state(rig.joint)
        .ok_or("benchmark joint disappeared")?;
    digest.update(hinge.position.to_le_bytes());
    digest.update(hinge.velocity.to_le_bytes());
    Ok(format!("{:x}", digest.finalize()))
}
fn percentile(values: &mut [u64], percent: usize) -> u64 {
    values.sort_unstable();
    values[(values.len() * percent).div_ceil(100) - 1]
}
pub fn measure(dir: &Path, rounds: usize, runs: &[Value]) -> Result<()> {
    let mut raw = BufWriter::new(File::create(dir.join("timings.jsonl"))?);
    let mut total = vec![vec![]; runs.len()];
    let mut tails = vec![vec![]; runs.len()];
    for round in 0..rounds {
        for slot in 0..runs.len() {
            let rotation = round % runs.len();
            let ordering = if (round / runs.len()).is_multiple_of(2) {
                slot
            } else {
                runs.len() - 1 - slot
            };
            let k = (rotation + ordering) % runs.len();
            let dt = runs[k]["dt_ns"].as_u64().unwrap();
            let steps = NS_PER_MS / dt;
            let (mut sim, rig) = setup(dt)?;
            let mut latencies = Vec::with_capacity(END as usize);
            let start = Instant::now();
            for t in 0..END {
                let then = Instant::now();
                advance_ms(&mut sim, &rig, t, steps)?;
                latencies.push(then.elapsed().as_nanos() as u64);
            }
            let elapsed = start.elapsed().as_nanos() as u64;
            let p95 = percentile(&mut latencies, 95);
            let end = endpoint(&sim, &rig)?;
            if end != runs[k]["native_endpoint_sha256"]
                || sim.elapsed().as_millis() != u128::from(END)
            {
                return Err("timed trajectory endpoint failed the corresponding qualified native trajectory".into());
            }
            let row = json!({"round":round,"slot":slot,"dt_ns":dt,"elapsed_ns":elapsed,"native_steps":END*steps,"millisecond_window_p95_ns":p95,"native_endpoint_sha256":end});
            serde_json::to_writer(&mut raw, &row)?;
            writeln!(raw)?;
            total[k].push(elapsed);
            tails[k].push(p95);
        }
    }
    raw.flush()?;
    let stats = runs
        .iter()
        .enumerate()
        .map(|(k, r)| {
            json!({
                "dt_ns":r["dt_ns"],"native_endpoint_sha256":r["native_endpoint_sha256"],
                "rounds":rounds,"median_elapsed_ns":percentile(&mut total[k],50),
                "p95_elapsed_ns":percentile(&mut total[k],95),
                "median_millisecond_window_p95_ns":percentile(&mut tails[k],50)
            })
        })
        .collect::<Vec<_>>();
    serde_json::to_writer_pretty(File::create(dir.join("timing-summary.json"))?, &stats)?;
    Ok(())
}
