// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde::{Deserialize, Serialize};
use std::{error::Error, fs::File, io::BufReader, path::PathBuf};

// Output of two_sphere_step_cost --samples, in advancing tick order.
#[derive(Deserialize)]
struct Sample(u64, [u8; 3], [u8; 3], u64);

#[derive(Default, Serialize)]
pub struct TickCost {
    pub owners: [u8; 3],
    pub before_ns: u64,
    pub after_ns: u64,
}

pub fn compare(
    before: &[PathBuf],
    after: &[PathBuf],
    ticks: usize,
    workers: u8,
) -> Result<Vec<TickCost>, Box<dyn Error>> {
    if ticks < 2 || before.is_empty() || after.is_empty() {
        return Err("comparison needs a trajectory and samples from both builds".into());
    }
    let mut owners = None;
    let mut costs: Vec<TickCost> = (0..ticks).map(|_| TickCost::default()).collect();
    for (files, current) in [(before, false), (after, true)] {
        let mut durations = vec![Vec::with_capacity(files.len()); ticks];
        for path in files {
            let rows: Vec<Sample> = serde_json::from_reader(BufReader::new(File::open(path)?))?;
            if rows.len() != ticks - 1 {
                return Err(
                    format!("sample count differs from the trace in {}", path.display()).into(),
                );
            }
            let mut route = Vec::with_capacity(ticks);
            route.push(rows[0].1);
            for (i, Sample(tick, old, new, ns)) in rows.into_iter().enumerate() {
                if tick != i as u64 + 1
                    || old != route[i]
                    || ns == 0
                    || old.into_iter().chain(new).any(|s| s >= workers)
                {
                    return Err(format!(
                        "invalid tick or layout in {} at tick {}",
                        path.display(),
                        i + 1
                    )
                    .into());
                }
                route.push(new);
                durations[i + 1].push(ns);
            }
            if owners.get_or_insert_with(|| route.clone()) != &route {
                return Err("benchmark builds or repetitions took different worker routes".into());
            }
        }
        for (cost, values) in costs[1..].iter_mut().zip(&mut durations[1..]) {
            values.sort_unstable();
            let lo = values[(values.len() - 1) / 2];
            let median = lo + (values[values.len() / 2] - lo) / 2;
            if current {
                cost.after_ns = median;
            } else {
                cost.before_ns = median;
            }
        }
    }
    for (cost, slots) in costs.iter_mut().zip(owners.unwrap()) {
        cost.owners = slots;
    }
    Ok(costs)
}
