// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time bare native integration, verifying the exact final state outside the timed region.
use super::{Body, Case, Drive, Result, point};
use serde_json::{Value, json};
use std::{
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
    time::Instant,
};
fn arm(
    seeds: &[Body; 2],
    schedule: &[u8],
    pointwise: bool,
    expected: &[Body; 2],
) -> Result<(u64, u64)> {
    let mut d = point::Direct::new(seeds, pointwise);
    let mut windows = Vec::with_capacity(schedule.len());
    let begin = Instant::now();
    for (k, &rate) in schedule.iter().enumerate() {
        let time = Instant::now();
        if k == 30 {
            d.kick([4.8, 0., 0.], [0., 0., 0.96]);
        }
        for _ in 0..rate {
            d.step(0.01 / f64::from(rate));
        }
        windows.push(time.elapsed().as_nanos() as u64);
    }
    let elapsed = begin.elapsed().as_nanos() as u64;
    for (i, b) in expected.iter().enumerate() {
        let s = d.state(i);
        if (s.position, s.orientation, s.velocity, s.angular_velocity)
            != (b.position, b.orientation, b.linear, b.angular)
        {
            return Err("timed native endpoint disagrees with physical trace".into());
        }
    }
    windows.sort_unstable();
    Ok((elapsed, windows[(windows.len() * 95).div_ceil(100) - 1]))
}
fn stats(v: &[u64]) -> Value {
    let mut x = v.to_owned();
    x.sort_unstable();
    json!({"p50_ns":x[(x.len()*50).div_ceil(100)-1],"p95_ns":x[(x.len()*95).div_ceil(100)-1]})
}
pub fn run(source: &Path, rounds: usize, directory: &Path) -> Result<()> {
    if rounds < 2 || rounds % 2 == 1 {
        return Err("give an even number of matched pairs".into());
    }
    let case = Case {
        square: false,
        moving: true,
        smooth: false,
        coarse: false,
        drive: Drive::Mixed,
    };
    let seeds = case.seeds();
    let mut schedule = vec![];
    let mut end = Value::Null;
    for line in BufReader::new(File::open(source)?).lines() {
        let v: Value = serde_json::from_str(&line?)?;
        schedule.push(v["substeps"].as_u64().ok_or("rate")? as u8);
        end = v;
    }
    if schedule.len() != 120 || end["tick"] != 120 {
        return Err("incomplete native source".into());
    }
    let expected: [Body; 2] = serde_json::from_value(end["bodies"].clone())?;
    let alternate: [Body; 2] = serde_json::from_value(end["point_control"]["bodies"].clone())?;
    let mut w = BufWriter::new(File::create_new(directory.join("pairs.jsonl"))?);
    let mut totals: [Vec<u64>; 2] = Default::default();
    let mut window_p95: [Vec<u64>; 2] = Default::default();
    for round in 0..rounds + 4 {
        let mut results = [(0, 0); 2];
        for first in 0..2 {
            let k = (round + first) % 2;
            results[k] = arm(
                &seeds,
                &schedule,
                k == 1,
                if k == 1 { &alternate } else { &expected },
            )?;
        }
        if round >= 4 {
            let [grouped, per_corner] = results;
            writeln!(
                w,
                "{}",
                json!({
                    "pair": round-3,
                    "first": if round%2==0{"grouped"}else{"per_corner"},
                    "grouped_ns": grouped.0,
                    "per_corner_ns": per_corner.0,
                    "grouped_p95_barrier_ns": grouped.1,
                    "per_corner_p95_barrier_ns": per_corner.1
                })
            )?;
            for (k, (elapsed, p95)) in results.into_iter().enumerate() {
                totals[k].push(elapsed);
                window_p95[k].push(p95);
            }
        }
    }
    w.flush()?;
    let summary = json!({
        "rounds": rounds,
        "untimed_warmup_pairs": 4,
        "public_intervals": schedule.len(),
        "native_steps": schedule.iter().map(|s|u64::from(*s)).sum::<u64>(),
        "simulation_seconds": 1.2,
        "grouped": stats(&totals[0]),
        "per_corner": stats(&totals[1]),
        "grouped_barrier_p95": stats(&window_p95[0]),
        "per_corner_barrier_p95": stats(&window_p95[1]),
        "per_corner_faster_pairs": totals[1].iter().zip(&totals[0]).filter(|(p,g)|p<g).count(),
        "terminal_states_checked": (rounds+4)*4
    });
    serde_json::to_writer_pretty(File::create_new(directory.join("summary.json"))?, &summary)?;
    Ok(())
}
