// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check the replay's gauge readings against the original per-cell forces and trajectories.
use super::{Result, close, vector};
use serde_json::Value;
use std::{fs::File, io::BufReader, path::Path};

fn n(v: &Value, field: &str) -> Result<f64> {
    v[field]
        .as_f64()
        .filter(|n| n.is_finite())
        .ok_or_else(|| format!("missing finite {field}").into())
}

pub(super) fn verify(path: &Path, raw: &Value, bins: usize) -> Result<()> {
    let input: Value = serde_json::from_reader(BufReader::new(File::open(path)?))?;
    let profiles = input["profiles"]
        .as_array()
        .ok_or("missing material profiles")?;
    let frames = raw.as_array().ok_or("no archive frames")?;
    if bins != 16 || profiles.len() != 3 || frames.len() < 2 {
        return Err("unexpected material replay layout".into());
    }
    let mut readings = Vec::new();
    let mut peak_force = 0f64;
    let mut max_distance = 0f64;
    for (k, p) in profiles.iter().enumerate() {
        let samples = p["frames"].as_array().ok_or("missing material samples")?;
        if samples.len() != frames.len() {
            return Err("sample count differs from archive".into());
        }
        for (i, s) in samples.iter().enumerate() {
            let cells = s["pressure"].as_array().ok_or("no contact pressure")?;
            if cells.is_empty() || !cells.len().is_multiple_of(bins) {
                return Err("incomplete spatial pressure bins".into());
            }
            let mut integrated = Vec::new();
            for chunk in cells.chunks_exact(cells.len() / bins) {
                let mut load = 0.;
                for c in chunk {
                    load += n(c, "weight_m")? * n(c, "line_load_n_m")?;
                }
                integrated.push(load);
                peak_force = peak_force.max(load);
            }
            let com = vector::<3>(&s["com_m"])?;
            let baseline = vector::<3>(&profiles[0]["frames"][i]["com_m"])?;
            let distance = com
                .into_iter()
                .zip(baseline)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            max_distance = max_distance.max(distance);
            readings.push((i, k, integrated, distance));
        }
    }
    if peak_force <= 0. || max_distance <= 0. {
        return Err("empty comparison scales".into());
    }
    for (i, k, forces, distance) in readings {
        let profile = profiles[k]["name"]
            .as_str()
            .ok_or("missing material name")?;
        let visuals = frames[i]["scene"]["visuals"]
            .as_array()
            .ok_or("no visual stream")?;
        let point = |name: &str| -> Result<[f64; 3]> {
            let id = format!("default::{profile} {name}::visual");
            let item = visuals
                .iter()
                .find(|v| v["name"] == id)
                .ok_or_else(|| format!("missing {id}"))?;
            vector(&item["translation"])
        };
        for (j, load) in forces.iter().enumerate() {
            let z = point(&format!("edge bin {}", j + 1))?[2];
            if !close((z + 0.42) / 0.22, load / peak_force) {
                return Err(format!(
                    "incorrect {profile} pressure at frame {} bin {}",
                    i + 1,
                    j + 1
                )
                .into());
            }
        }
        let y = point("path difference")?[1];
        let lane_center = 0.76 * (k as f64 - 1.);
        if !close((y - lane_center) / 0.57 + 0.5, distance / max_distance) {
            return Err(format!("incorrect {profile} distance at frame {}", i + 1).into());
        }
    }
    Ok(())
}
