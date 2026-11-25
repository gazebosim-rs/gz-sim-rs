// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare trajectories on identical committed ticks. Angular coordinates are
//! native joint positions with zero defined by the SDF release configuration.
use pendulum_native_reference::{HEADER, LAST_TICK, STRIDE};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path};

type Sample = [f64; 15];

fn read(path: &Path) -> Result<[Vec<Sample>; 2], Box<dyn Error>> {
    let contents = fs::read_to_string(path)?;
    let mut lines = contents.lines();
    if lines.next() != Some(HEADER) {
        return Err(format!("unrecognized header in {}", path.display()).into());
    }
    let mut models = [Vec::new(), Vec::new()];
    for (i, line) in lines.enumerate() {
        let mut fields = line.split(',');
        let tick: u64 = fields.next().ok_or("missing tick")?.parse()?;
        let model: usize = fields.next().ok_or("missing model")?.parse()?;
        if (tick, model) != ((i / 2) as u64 * STRIDE, i % 2) {
            return Err(format!("unexpected model or tick on row {}", i + 2).into());
        }
        let raw = fields
            .map(str::parse::<f64>)
            .collect::<Result<Vec<_>, _>>()?;
        let row: Sample = raw.try_into().map_err(|_| "wrong number of columns")?;
        if !row.iter().all(|n| n.is_finite()) || (row[14] - row[13] - row[12]).abs() > 1e-9 {
            return Err(format!("invalid state on row {}", i + 2).into());
        }
        models[model].push(row);
    }
    if !models
        .iter()
        .all(|m| m.len() == (LAST_TICK / STRIDE + 1) as usize)
    {
        return Err("incomplete trajectory".into());
    }
    Ok(models)
}

fn crossings(samples: &[Sample]) -> Result<Vec<f64>, Box<dyn Error>> {
    let target = -std::f64::consts::PI + 15708.0 / 10000.0;
    let s_per_sample = STRIDE as f64 * 0.001;
    let crossings: Vec<_> = samples
        .windows(2)
        .enumerate()
        .filter_map(|(i, pair)| {
            let (before, after) = (pair[0][0], pair[1][0]);
            (before >= target && after < target)
                .then(|| s_per_sample * (i as f64 + (target - before) / (after - before)))
        })
        .collect();
    if crossings.len() < 2 {
        return Err("no complete descending return".into());
    }
    Ok(crossings)
}

fn separation(left: &Sample, right: &Sample, start: usize) -> f64 {
    (start..start + 3)
        .map(|i| (left[i] - right[i]).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn model_report(rust: &[Sample], native: &[Sample]) -> Result<Value, Box<dyn Error>> {
    let initial_energy = native[0][14];
    if initial_energy <= 0.0 || (rust[0][14] - initial_energy).abs() > 1e-10 {
        return Err("initial energies differ or normalization unavailable".into());
    }
    let max = |f: &dyn Fn(&Sample, &Sample) -> f64| {
        rust.iter()
            .zip(native)
            .enumerate()
            .map(|(i, (r, d))| (f(r, d), i as u64 * STRIDE))
            .max_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap()
    };
    let (upper, tu) = max(&|r, d| (r[0] - d[0]).abs());
    let (lower, tl) = max(&|r, d| (r[1] - d[1]).abs());
    let (energy_gap, te) = max(&|r, d| (r[14] - d[14]).abs());
    let energy_i = (te / STRIDE) as usize;
    // The SDF has 100 kg at the base origin and two unit-mass moving links.
    let base_z = |r: &Sample| (r[12] / 9.8 - r[8] - r[11]) / 100.0;
    let dart_cross = crossings(native)?;
    let rust_cross = crossings(rust)?;
    let dart_period = dart_cross[1] - dart_cross[0];
    let rust_period = rust_cross[1] - rust_cross[0];
    let period_error = ((rust_period - dart_period) / dart_period).abs();
    Ok(json!({
        "upper_max_abs_q_error_rad": upper, "upper_error_tick": tu,
        "lower_max_abs_q_error_rad": lower, "lower_error_tick": tl,
        "upper_max_abs_dq_error_rad_s": max(&|r,d| (r[2]-d[2]).abs()).0,
        "lower_max_abs_dq_error_rad_s": max(&|r,d| (r[3]-d[3]).abs()).0,
        "upper_max_com_distance_m": max(&|r,d| separation(r,d,6)).0,
        "lower_max_com_distance_m": max(&|r,d| separation(r,d,9)).0,
        "dart_upper_descending_crossings_s": dart_cross, "rust_upper_descending_crossings_s": rust_cross,
        "dart_first_return_s": dart_period, "rust_first_return_s": rust_period,
        "first_return_fraction_error": period_error,
        "initial_energy_j": initial_energy, "max_energy_gap_j": energy_gap, "energy_gap_tick": te,
        "max_energy_gap_fraction_initial": energy_gap / initial_energy,
        "dart_max_energy_drift_fraction_initial": max(&|_,d| (d[14]-initial_energy).abs()).0 / initial_energy,
        "rust_max_energy_drift_fraction_initial": max(&|r,_| (r[14]-initial_energy).abs()).0 / initial_energy,
        "dart_base_z_m_at_peak_energy_gap": base_z(&native[energy_i]),
        "rust_base_z_m_at_peak_energy_gap": base_z(&rust[energy_i]),
        "angle_within_0_06_rad": upper <= 0.06 && lower <= 0.06,
        "period_within_1_percent": period_error <= 0.01,
        "energy_gap_within_2_percent": energy_gap / initial_energy <= 0.02,
    }))
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 3 {
        return Err("usage: compare RUST.csv DART.csv OUTPUT.json".into());
    }
    let rust = read(Path::new(&args[0]))?;
    let native = read(Path::new(&args[1]))?;
    let mut models = Vec::new();
    for i in 0..2 {
        models.push(model_report(&rust[i], &native[i])?);
    }
    let pass = models.iter().all(|m| {
        [
            "angle_within_0_06_rad",
            "period_within_1_percent",
            "energy_gap_within_2_percent",
        ]
        .iter()
        .all(|&k| m[k] == true)
    });
    let report = json!({"samples_per_model": native[0].len(), "step_seconds":0.001,
        "sample_every_ticks": STRIDE, "passed_all_native_thresholds":pass, "models":models});
    fs::write(
        &args[2],
        format!("{}\n", serde_json::to_string_pretty(&report)?),
    )?;
    if !pass {
        return Err(
            "native comparison exceeded one or more declared thresholds; see output".into(),
        );
    }
    Ok(())
}
