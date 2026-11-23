// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit complete records of the seeded, actuated worker-pool campaigns.
use clap::Parser;
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// One or more JSONL ledgers from actuated_pair_sweep. Each file is one arm.
    #[arg(required = true)]
    ledgers: Vec<PathBuf>,
    /// Compare the overlapping seeds from these two ledgers, by file stem.
    #[arg(long, num_args = 2, action = clap::ArgAction::Append)]
    compare: Vec<String>,
}

#[derive(Deserialize)]
struct Episode {
    seed: u64,
    workers: u8,
    ticks: u64,
    states: u64,
    layout_observations: BTreeMap<String, u64>,
    loaded_transfers: u64,
    worker_kills: u64,
    whole_pool_reopens: u64,
    independently_queued: u64,
    pair_loads: u64,
    hinge_loads: u64,
    max_sphere_impulse_residual_ns: f64,
    max_quaternion_squared_norm_error: Option<f64>,
    physical_digest: String,
}
#[derive(Default, Serialize)]
struct Counts {
    episodes: usize,
    ticks: u64,
    states: u64,
    worker_kills: u64,
    whole_pool_reopens: u64,
    loaded_transfers: u64,
    independently_queued: u64,
    pair_loads: u64,
    hinge_loads: u64,
    max_sphere_impulse_residual_ns: f64,
    max_quaternion_squared_norm_error: Option<f64>,
    first_seed: Option<u64>,
    last_seed: Option<u64>,
    worker_counts: BTreeMap<u8, usize>,
}
impl Counts {
    fn observe(&mut self, e: &Episode) -> Result<()> {
        if !(3..=8).contains(&e.workers)
            || e.states.checked_sub(1) != Some(e.ticks)
            || e.layout_observations.values().sum::<u64>() != e.states
            || e.max_sphere_impulse_residual_ns > 1e-8
            || e.max_sphere_impulse_residual_ns < 0.
            || e.max_quaternion_squared_norm_error
                .is_some_and(|x| !(0. ..=1e-12).contains(&x))
            || e.physical_digest.len() != 64
            || !e.physical_digest.bytes().all(|b| b.is_ascii_hexdigit())
        {
            return Err(format!("invalid physical record for seed {}", e.seed).into());
        }
        self.episodes += 1;
        self.ticks += e.ticks;
        self.states += e.states;
        self.worker_kills += e.worker_kills;
        self.whole_pool_reopens += e.whole_pool_reopens;
        self.loaded_transfers += e.loaded_transfers;
        self.independently_queued += e.independently_queued;
        self.pair_loads += e.pair_loads;
        self.hinge_loads += e.hinge_loads;
        self.max_sphere_impulse_residual_ns = self
            .max_sphere_impulse_residual_ns
            .max(e.max_sphere_impulse_residual_ns);
        if let Some(q) = e.max_quaternion_squared_norm_error {
            self.max_quaternion_squared_norm_error =
                Some(self.max_quaternion_squared_norm_error.unwrap_or(0.).max(q));
        }
        self.first_seed = Some(self.first_seed.unwrap_or(e.seed).min(e.seed));
        self.last_seed = Some(self.last_seed.unwrap_or(e.seed).max(e.seed));
        *self.worker_counts.entry(e.workers).or_default() += 1;
        Ok(())
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    let mut counts = BTreeMap::new();
    let mut digests = BTreeMap::new();
    for path in args.ledgers {
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .ok_or("missing arm name")?
            .to_owned();
        let mut arm = BTreeMap::new();
        let mut total = Counts::default();
        let bytes = fs::read(&path)?;
        for line in bytes.split_inclusive(|&b| b == b'\n') {
            if !line.ends_with(b"\n") {
                break;
            } // concurrently appending this episode
            let e: Episode = serde_json::from_slice(line)?;
            total.observe(&e)?;
            if arm.insert(e.seed, (e.ticks, e.physical_digest)).is_some() {
                return Err(format!("duplicate seed {} in {name}", e.seed).into());
            }
        }
        if digests.insert(name.clone(), arm).is_some() {
            return Err(format!("duplicate ledger name {name}").into());
        }
        counts.insert(name, total);
    }
    let mut comparisons = Vec::new();
    for names in args.compare.chunks_exact(2) {
        let a = digests.get(&names[0]).ok_or("missing comparison arm")?;
        let b = digests.get(&names[1]).ok_or("missing comparison arm")?;
        let mut matches = 0;
        for (seed, expected) in a {
            if let Some(other) = b.get(seed) {
                if expected != other {
                    return Err(format!(
                        "physical digest or interval count differs at seed {seed} in {names:?}"
                    )
                    .into());
                }
                matches += 1;
            }
        }
        comparisons.push(serde_json::json!({"arms":names,"matching_seeds":matches,"only_in_first":a.len()-matches,"only_in_second":b.len()-matches}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"arms":counts,"width_comparisons":comparisons})
        )?
    );
    Ok(())
}
