// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check measured reactions and poses in the two uniform-material replay lanes.
use super::{Result, close, vector};
use serde_json::Value;
use std::{fs::File, io::BufReader, path::Path};

fn raw(path: &Path, prior: bool) -> Result<Vec<Value>> {
    let doc: Value = serde_json::from_reader(BufReader::new(File::open(path)?))?;
    if doc["profiles"][0]["name"] != "Uniform layers" {
        return Err("missing uniform layer history".into());
    }
    let selected = if prior {
        &doc["earlier_start_control"]
    } else {
        &doc["profiles"][0]
    };
    Ok(selected["frames"]
        .as_array()
        .ok_or("missing landing frames")?
        .clone())
}
fn number(v: &Value, key: &str) -> Result<f64> {
    v[key]
        .as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| format!("missing finite {key}").into())
}
fn bins(v: &Value, count: usize) -> Result<Vec<f64>> {
    let mut forces = vec![0.; count];
    for c in v["pressure"].as_array().ok_or("missing pressure cells")? {
        let f = number(c, "fraction")?;
        if !(0.0..1.0).contains(&f) {
            return Err("invalid edge location".into());
        }
        forces[(f * count as f64).floor() as usize] +=
            number(c, "weight_m")? * number(c, "line_load_n_m")?;
    }
    if (forces.iter().sum::<f64>() - number(v, "normal_force_n")?).abs() > 1e-9 {
        return Err("pressure bins did not recover the total reaction".into());
    }
    Ok(forces)
}
fn distance(a: &Value, b: &Value) -> Result<f64> {
    let a = vector::<3>(&a["com_m"])?;
    Ok(a.into_iter()
        .zip(vector::<3>(&b["com_m"])?)
        .map(|(u, v)| (u - v).powi(2))
        .sum::<f64>()
        .sqrt())
}
pub(super) fn verify(
    early: &Path,
    late: &Path,
    archive: &Value,
    count: usize,
) -> Result<(usize, usize)> {
    let p = raw(early, true)?;
    let a = raw(early, false)?;
    let b = raw(late, false)?;
    let frames = archive.as_array().ok_or("missing archive")?;
    if frames.len() != p.len() || count != 16 {
        return Err("unexpected archive or bin count".into());
    }
    let mut peak = 0f64;
    for s in a.iter().chain(&b).chain(&p) {
        peak = bins(s, count)?.into_iter().fold(peak, f64::max);
    }
    let s_times = b
        .iter()
        .map(|s| number(s, "original_time_s"))
        .collect::<Result<Vec<_>>>()?;
    let mut pairs = vec![];
    let mut max_distance = 0f64;
    for pre in &p {
        let at = number(pre, "original_time_s")?;
        let s = a.iter().find(|v| {
            v["original_time_s"]
                .as_f64()
                .is_some_and(|t| (t - at).abs() < 1e-12)
        });
        let other = s_times
            .iter()
            .position(|t| (t - at).abs() < 1e-12)
            .map(|k| &b[k]);
        if let (Some(s), Some(v)) = (s, other) {
            max_distance = max_distance.max(distance(s, v)?);
        }
        pairs.push((pre, s, other));
    }
    if pairs.iter().filter(|(_, _, other)| other.is_some()).count() != b.len() {
        return Err("different output clocks".into());
    }
    let (mut bin_checks, mut gauge_checks) = (0, 0);
    for (record, (pre, s, other)) in frames.iter().zip(pairs) {
        let visuals = record["scene"]["visuals"].as_array().ok_or("no scene")?;
        let at = number(pre, "original_time_s")?;
        // The original restart is 500 µs after the donor integration begins.
        let ns = ((at - 0.0005) * 1e9).round() as u64;
        if record["ns"].as_u64() != Some(ns) {
            return Err("incorrect displayed restart time".into());
        }
        let point = |name: &str| -> Result<[f64; 3]> {
            let label = format!("default::{name}::visual");
            let item = visuals
                .iter()
                .find(|v| v["name"] == label)
                .ok_or_else(|| format!("missing {label}"))?;
            vector(&item["translation"])
        };
        for (name, sample, shift) in [
            ("Elastic before corner", Some(pre), -0.77),
            ("Elastic from corner", s, 0.0),
            ("Elastic from line", other, 0.77),
        ] {
            let Some(v) = sample else {
                if visuals
                    .iter()
                    .any(|x| x["name"].as_str().unwrap_or("").contains(name))
                {
                    return Err("unexpected line-start entry before sample".into());
                }
                continue;
            };
            let [x, y, z] = vector::<3>(&v["foot_m"])?;
            let measured = point(&format!("{name} foot"))?;
            if measured
                .into_iter()
                .zip([-y, x + shift, z + 0.522 * at])
                .any(|(x, y)| !close(x, y))
            {
                return Err("replay foot position differs from the source trajectory".into());
            }
            for (k, force) in bins(v, count)?.into_iter().enumerate() {
                let z = point(&format!("{name} edge bin {}", k + 1))?[2];
                if !close((z + 0.42) / 0.22, force / peak) {
                    return Err("replay pressure differs from source".into());
                }
                bin_checks += 1;
            }
        }
        if let (Some(s), Some(v)) = (s, other) {
            if !close(
                point("Measured path separation")?[1] / 0.84 + 0.5,
                distance(s, v)? / max_distance,
            ) {
                return Err("replay separation differs from source".into());
            }
            gauge_checks += 1;
        }
    }
    Ok((bin_checks, gauge_checks))
}
