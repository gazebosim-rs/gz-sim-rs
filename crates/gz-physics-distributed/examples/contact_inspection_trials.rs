// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare full native-image inspection with separate on-demand queries on the same archive.
use clap::Parser;
use gz_physics_distributed::pool::{
    NativeContactSnapshot, PoolFrame, PoolNativeRecovery, WorkerContactPatches,
};
use serde_json::json;
use std::{error::Error, path::PathBuf, time::Instant};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    path: PathBuf,
    #[arg(long,default_value_t=32,value_parser=clap::value_parser!(u32).range(2..=4096))]
    rounds: u32,
}

fn evidence(frame: &PoolFrame, patches: &[WorkerContactPatches]) -> Result<Vec<u8>> {
    Ok(bincode::serialize(&(frame.tick, &frame.workers, patches))?)
}

fn timed<T>(read: impl FnOnce() -> Result<T>) -> Result<(u128, T)> {
    let start = Instant::now();
    let contacts = read()?;
    Ok((start.elapsed().as_nanos(), contacts))
}

fn main() -> Result<()> {
    let args = Args::parse();
    let warm = NativeContactSnapshot::open(&args.path)?;
    let combined = || Ok(NativeContactSnapshot::open(&args.path)?);
    let separate = || {
        let plan = if warm.population.is_some() {
            PoolNativeRecovery::open_scene_manifest(&args.path)?.2
        } else {
            PoolNativeRecovery::open(&args.path)?
        };
        let pairs = plan.finite_contact_patches()?;
        let planes = plan.ground_contact_patches()?;
        let patches = pairs
            .into_iter()
            .zip(planes)
            .map(|(finite, ground)| WorkerContactPatches { finite, ground })
            .collect::<Vec<_>>();
        Ok((plan, patches))
    };
    let expected = evidence(&warm.frame, &warm.patches)?;
    let (prior, patches) = separate()?;
    if evidence(prior.frame(), &patches)? != expected {
        return Err("unmatched contact archives".into());
    }
    let mut trials = Vec::new();
    for round in 0..args.rounds {
        let combined_first = round.is_multiple_of(2);
        let (new, old) = if combined_first {
            (timed(combined)?, timed(separate)?)
        } else {
            let old = timed(separate)?;
            (timed(combined)?, old)
        };
        let (plan, patches) = old.1;
        if evidence(&new.1.frame, &new.1.patches)? != expected
            || evidence(plan.frame(), &patches)? != expected
        {
            return Err("changed inspection result".into());
        }
        trials.push(json!({"combined_first":combined_first,"single_pass_ns":new.0,"separate_queries_ns":old.0}));
    }
    let tick = warm.frame.tick;
    let finite_points: usize = warm
        .patches
        .iter()
        .flat_map(|w| &w.finite)
        .map(|p| p.points.len())
        .sum();
    let ground_points: usize = warm
        .patches
        .iter()
        .flat_map(|w| &w.ground)
        .map(|p| p.points.len())
        .sum();
    println!(
        "{}",
        json!({"path":args.path,"tick":tick,"workers":warm.patches.len(),"finite_points":finite_points,"ground_points":ground_points,"trials":trials})
    );
    Ok(())
}
