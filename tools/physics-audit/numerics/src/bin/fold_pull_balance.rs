// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check the lateral impulse budget of a suspended sheet against its unloaded twin.
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

struct Sample {
    time_ns: u64,
    force: f64,
    jaws_on_sheet: [f64; 2],
    bench_on_sheet: f64,
    momentum: f64,
}

fn number(value: &Value) -> Result<f64> {
    value
        .as_f64()
        .ok_or_else(|| "missing finite measurement".into())
}

fn read_branch(path: &Path, start: u64) -> Result<Vec<Sample>> {
    let raw: Vec<Value> = serde_json::from_slice(&fs::read(path)?)?;
    let mut branch = Vec::new();
    for row in raw {
        let time_ns = row["time_ns"].as_u64().ok_or("missing simulation time")?;
        if time_ns >= start {
            branch.push(Sample {
                time_ns,
                force: number(&row["force_y_n"])?,
                jaws_on_sheet: [
                    -number(&row["jaw_side_reaction_n"][0])?,
                    -number(&row["jaw_side_reaction_n"][1])?,
                ],
                bench_on_sheet: -number(&row["bench_side_reaction_n"])?,
                momentum: number(&row["momentum_y_kg_m_s"])?,
            });
        }
    }
    Ok(branch)
}

fn budget(pull: &[Sample], sham: &[Sample], start: u64, step_ns: u64) -> Result<Value> {
    if pull.len() != sham.len() || pull.len() < 3 || step_ns == 0 {
        return Err("need two complete, synchronized traces".into());
    }
    let mut applied = 0.;
    let mut jaws = [0.; 2];
    let mut bench = 0.;
    let mut max_residual: f64 = 0.;
    let mut at_release = None;
    let dt = step_ns as f64 * 1e-9;
    for (i, (p, s)) in pull.iter().zip(sham).enumerate() {
        let expected = start as u128 + i as u128 * step_ns as u128;
        if u128::from(p.time_ns) != expected
            || p.time_ns != s.time_ns
            || s.force != 0.
            || p.force < 0.
        {
            return Err("unexpected clock or external load".into());
        }
        if i == 0 {
            if p.force != 0.
                || p.momentum != s.momentum
                || p.jaws_on_sheet != s.jaws_on_sheet
                || p.bench_on_sheet != s.bench_on_sheet
            {
                return Err("unmatched incoming lateral state".into());
            }
            continue;
        }
        applied += p.force * dt;
        for (k, impulse) in jaws.iter_mut().enumerate() {
            *impulse += (p.jaws_on_sheet[k] - s.jaws_on_sheet[k]) * dt;
        }
        bench += (p.bench_on_sheet - s.bench_on_sheet) * dt;
        let residual = (p.momentum - s.momentum) - applied - jaws.iter().sum::<f64>() - bench;
        max_residual = max_residual.max(residual.abs());
        if p.force > 0. {
            at_release = Some(
                json!({"time_ns":p.time_ns,"applied_ns":applied,"jaws_on_sheet_ns":jaws,"bench_on_sheet_ns":bench,"sheet_momentum_change_ns":p.momentum-s.momentum,"residual_ns":residual}),
            );
        }
    }
    let last = pull.last().unwrap();
    let reference = sham.last().unwrap();
    if at_release.is_none() || last.force != 0. {
        return Err("need a lateral pull followed by unloaded recovery".into());
    }
    Ok(
        json!({"last_loaded_tick":at_release, "after_recovery":{"time_ns":last.time_ns,"applied_ns":applied,
        "jaws_on_sheet_ns":jaws,"bench_on_sheet_ns":bench,"sheet_momentum_change_ns":last.momentum-reference.momentum},
        "max_cumulative_residual_ns":max_residual,"max_residual_fraction_of_applied_impulse":max_residual/applied}),
    )
}

fn audit(root: &Path) -> Result<Value> {
    let study: Value = serde_json::from_slice(&fs::read(root.join("comparison.json"))?)?;
    let dt = study["dt_ns"].as_u64().ok_or("missing public clock")?;
    let trials = study["trials"].as_array().ok_or("missing mesh trials")?;
    let mut results = Vec::new();
    for cols in [5, 7] {
        let trial = trials
            .iter()
            .find(|t| t["columns"] == cols)
            .ok_or("missing mesh")?;
        let start = trial["baseline_time_ns"]
            .as_u64()
            .ok_or("missing baseline")?;
        let branch =
            |name| read_branch(&root.join(format!("mesh-{cols}-{name}/trace.json")), start);
        results.push(
            json!({"columns":cols,"balance":budget(&branch("pull")?,&branch("sham")?,start,dt)?}),
        );
    }
    Ok(json!({"frame":"world +Y","method":"pulled minus matched unloaded branch","meshes":results}))
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [root] = &args[..] else {
        return Err("usage: fold_pull_balance STUDY_DIR".into());
    };
    println!(
        "{}",
        serde_json::to_string_pretty(&audit(Path::new(root))?)?
    );
    Ok(())
}

#[cfg(test)]
#[path = "../../tests/support/fold_pull_balance.rs"]
mod tests;
