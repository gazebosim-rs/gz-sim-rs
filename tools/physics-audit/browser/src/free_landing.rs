// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently recover the new surface-contact indicators from the solver samples.
use super::{Result, close, same, vector};
use serde_json::Value;
use std::{fs, path::Path};

fn frames(h: &Value) -> Result<&[Value]> {
    h["frames"]
        .as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing contact history".into())
}
fn num(v: &Value, key: &str) -> Result<f64> {
    v[key]
        .as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| format!("missing {key}").into())
}

pub(super) fn verify(
    path: &Path,
    archive: &Value,
    damped: bool,
) -> Result<(usize, usize, usize, usize)> {
    let study: Value = serde_json::from_slice(&fs::read(path)?)?;
    let hs = study["profiles"].as_array().ok_or("missing surface runs")?;
    let scene = archive.as_array().ok_or("missing archive frames")?;
    if hs.len() != 3 || scene.len() != frames(&hs[0])?.len() {
        return Err("missing free-approach frames".into());
    }
    let mut peaks = vec![];
    for h in hs {
        let mut max = 0f64;
        for f in frames(h)? {
            max = max.max(num(f, "normal_force_n")?);
        }
        peaks.push(max);
    }
    let mut dissipated_peak = 0_f64;
    if damped {
        for h in hs {
            let last = frames(h)?.last().ok_or("missing last damping frame")?;
            dissipated_peak = dissipated_peak.max(num(last, "normal_dissipation_j")?);
        }
        if dissipated_peak <= 0. {
            return Err("missing damping energy".into());
        }
        let all = peaks.iter().copied().fold(0., f64::max);
        peaks.fill(all);
    }
    let (mut forces, mut centers, mut edges, mut energies) = (0, 0, 0, 0);
    for (record, free) in scene.iter().zip(frames(&hs[0])?) {
        let t = num(free, "original_time_s")?;
        if record["ns"].as_u64() != Some((1e9 * t).round() as u64) {
            return Err("mismatched physical clock".into());
        }
        let visuals = record["scene"]["visuals"]
            .as_array()
            .ok_or("missing contact scene")?;
        for (i, h) in hs.iter().enumerate() {
            let name = h["name"].as_str().ok_or("missing label")?;
            let prefix = format!("default::{name} ");
            let Some(f) = frames(h)?.iter().find(|f| {
                f["original_time_s"]
                    .as_f64()
                    .is_some_and(|u| (u - t).abs() < 1e-12)
            }) else {
                if visuals
                    .iter()
                    .any(|v| v["name"].as_str().unwrap_or("").starts_with(&prefix))
                {
                    return Err("restart appeared outside recorded clock".into());
                }
                continue;
            };
            let lookup = |suffix: &str| {
                let label = format!("{prefix}{suffix}::visual");
                visuals.iter().find(|v| v["name"] == label)
            };
            let check = |suffix: &str, point: [f64; 3]| -> Result<()> {
                let v = lookup(suffix).ok_or_else(|| format!("missing {name} {suffix}"))?;
                if !same(&v["translation"], point)? {
                    return Err(format!("{name} {suffix} differs from scientific data").into());
                }
                Ok(())
            };
            let shift = (i as f64 - 1.) * 0.8;
            let [x, y, z] = vector::<3>(&f["foot_m"])?;
            check("foot", [-y, x + shift, z + 0.522 * t])?;
            let load = num(f, "normal_force_n")?;
            check(
                "measured force",
                [0., shift + 0.54 * (load / peaks[i] - 0.5), -0.57],
            )?;
            forces += 1;
            if damped {
                let loss = num(f, "normal_dissipation_j")?;
                check(
                    "absorbed energy",
                    [0., shift + 0.54 * (loss / dissipated_peak - 0.5), -0.64],
                )?;
                energies += 1;
            }
            let map =
                |p: [f64; 3], depth| [depth, shift + 1.5 * (p[0] - x), -0.34 + 1.5 * (p[1] - y)];
            if f["center_of_pressure_m"].is_null() {
                if lookup("pressure center").is_some() || load > 0. {
                    return Err("unexpected pressure center".into());
                }
            } else {
                check(
                    "pressure center",
                    map(vector(&f["center_of_pressure_m"])?, 0.015),
                )?;
                centers += 1;
            }
            let outline = f["loaded_polygon_m"]
                .as_array()
                .ok_or("missing area outline")?;
            let mut count = 0;
            for (j, (a, b)) in outline
                .iter()
                .zip(outline.iter().cycle().skip(1))
                .enumerate()
            {
                let a = map(vector(a)?, 0.006);
                let b = map(vector(b)?, 0.006);
                let span = (a[1] - b[1]).hypot(a[2] - b[2]);
                if span <= 1e-12 {
                    continue;
                }
                let edge = format!("loaded patch {}", j + 1);
                check(&edge, [0.006, 0.5 * (a[1] + b[1]), 0.5 * (a[2] + b[2])])?;
                let v = lookup(&edge).unwrap();
                if !close(vector::<3>(&v["shape"]["size"])?[1], span) {
                    return Err("displayed contact edge has the wrong length".into());
                }
                count += 1;
            }
            if count
                != visuals
                    .iter()
                    .filter(|v| {
                        v["name"]
                            .as_str()
                            .unwrap_or("")
                            .starts_with(&format!("{prefix}loaded patch "))
                    })
                    .count()
            {
                return Err("extra patch edges in scene".into());
            }
            edges += count;
        }
    }
    Ok((forces, centers, edges, energies))
}
