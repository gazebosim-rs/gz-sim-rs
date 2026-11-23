// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate each rendered link pose and SDF shape against the native Bevy snapshot.
use super::{Result, same, vector};
use serde_json::Value;
use std::{fs, path::Path};
pub(super) fn verify(path: &Path, archive: &Value, branch: bool) -> Result<(usize, usize)> {
    let runs: Vec<Value> = serde_json::from_slice(&fs::read(path)?)?;
    if runs.len() != if branch { 3 } else { 4 } {
        return Err("missing native/composite trial".into());
    }
    let mut maximum = 0_f64;
    for h in &runs {
        for s in h["frames"].as_array().ok_or("missing frame")? {
            maximum = maximum.max(
                s["normal_impulse_ns"]
                    .as_f64()
                    .ok_or("missing native normal impulse")?,
            );
        }
    }
    let mut count = (0, 0);
    for frame in archive.as_array().ok_or("missing archive")? {
        let ns = frame["ns"].as_u64().ok_or("missing physical stamp")?;
        if !ns.is_multiple_of(500000) {
            return Err("unmatched public boundary".into());
        }
        let tick = (ns / 500000) as usize;
        for (i, h) in runs.iter().enumerate() {
            let first = h["frames"][0]["tick"]
                .as_u64()
                .ok_or("missing initial clock")? as usize;
            let f = &h["frames"][tick
                .checked_sub(first)
                .ok_or("clock precedes initial boundary")?];
            if f["tick"] != tick {
                return Err("incorrect public boundary".into());
            }
            let name = h["name"].as_str().ok_or("missing trial name")?;
            let lookup = |suffix: &str| -> Result<&Value> {
                let n = format!("default::{name} {suffix}::visual");
                frame["scene"]["visuals"]
                    .as_array()
                    .and_then(|v| v.iter().find(|r| r["name"] == n))
                    .ok_or_else(|| "missing live visual".into())
            };
            let offset = (i as f64 - if branch { 1. } else { 1.5 }) * 0.59;
            for link in ["foot", "upper"] {
                let original = &f[link];
                let [x, y, z] = vector::<3>(&original["p"])?;
                let [a, b, c, d] = vector::<4>(&original["q"])?;
                let k = std::f64::consts::FRAC_1_SQRT_2;
                let measured = lookup(link)?;
                if !same(&measured["translation"], [-y, x + offset, z])?
                    || measured["shape"]["size"] != original["size"]
                    || vector::<4>(&measured["rotation"])?
                        .into_iter()
                        .zip([k * (a - b), k * (a + b), k * (c + d), k * (d - c)])
                        .any(|(v, q)| (v - q).abs() > 1e-12)
                {
                    return Err(format!("incorrect rendered {name} {link} at tick {tick}").into());
                }
                count.0 += 1;
            }
            if branch {
                let c = vector::<3>(&f["com"])?;
                let b = vector::<3>(&h["frames"][0]["com"])?;
                if !same(
                    &lookup("horizontal travel at 100x")?["translation"],
                    [
                        0.01,
                        offset + 100. * (c[0] - b[0]),
                        -0.29 + 100. * (c[1] - b[1]),
                    ],
                )? {
                    return Err("incorrect lateral travel gauge".into());
                }
            }
            if h["ground"] == true {
                let impulse = f["normal_impulse_ns"]
                    .as_f64()
                    .ok_or("missing measured ground impulse")?;
                if !same(
                    &lookup("ground impulse")?["translation"],
                    [0., offset + 0.34 * (impulse / maximum - 0.5), -0.08],
                )? {
                    return Err("incorrect rigid contact impulse cursor".into());
                }
                count.1 += 1;
            }
        }
    }
    Ok(count)
}
