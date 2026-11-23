// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify turntable and foot attitudes and work against the measured physics clocks.
use super::{Result, same, vector};
use serde_json::Value;
use std::{fs, path::Path};
pub(super) fn verify(path: &Path, archive: &Value) -> Result<(usize, usize)> {
    let runs: Vec<Value> = serde_json::from_slice(&fs::read(path)?)?;
    if runs.len() != 3 {
        return Err("expected three live turntable histories".into());
    }
    let mut scale = 0_f64;
    for h in &runs {
        for s in h["samples"].as_array().ok_or("missing samples")? {
            scale = scale.max(s["cumulative_spin_work_j"].as_f64().ok_or("missing work")?);
        }
    }
    let mut counts = (0, 0);
    for frame in archive.as_array().ok_or("missing archive")? {
        let ns = frame["ns"].as_u64().ok_or("missing clock")?;
        for (lane, h) in runs.iter().enumerate() {
            let samples = h["samples"].as_array().ok_or("missing samples")?;
            let s = samples
                .iter()
                .find(|s| s["tick"].as_u64().is_some_and(|t| t * 500000 == ns))
                .ok_or("unmatched physical tick")?;
            let prefix = h["name"].as_str().ok_or("missing lane name")?;
            let viz = |suffix: &str| -> Result<&Value> {
                let n = format!("default::{prefix} {suffix}::visual");
                frame["scene"]["visuals"]
                    .as_array()
                    .and_then(|v| v.iter().find(|r| r["name"] == n))
                    .ok_or_else(|| "missing turntable visual".into())
            };
            for (label, key) in [
                ("driven table", "table_orientation_xyzw"),
                ("foot", "orientation_xyzw"),
            ] {
                let actual = vector::<4>(&viz(label)?["rotation"])?;
                let measured = vector::<4>(&s[key])?;
                if actual
                    .iter()
                    .zip(measured)
                    .any(|(a, b)| (a - b).abs() > 1e-12)
                {
                    return Err(format!(
                        "{prefix} {label} orientation does not match sample at {ns} ns"
                    )
                    .into());
                }
                counts.0 += 1;
            }
            let offset = 0.81 * (lane as f64 - 1.);
            let work = s["cumulative_spin_work_j"]
                .as_f64()
                .ok_or("missing sample work")?;
            if !same(
                &viz("rotational work")?["translation"],
                [offset + 0.4 * (work / scale - 0.5), -0.34, -0.02],
            )? {
                return Err("work gauge differs from turntable measurement".into());
            }
            counts.1 += 1;
        }
    }
    Ok(counts)
}
