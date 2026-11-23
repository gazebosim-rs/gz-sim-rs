// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recover the moving-panel load cursors and body poses directly from the physical samples.
use super::{Result, same, vector};
use serde_json::Value;
use std::{fs, path::Path};

pub(super) fn verify(path: &Path, archive: &Value) -> Result<usize> {
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(path)?)?;
    if rows.len() != 2 {
        return Err("expected both seam materials".into());
    }
    let samples = |lane: usize| {
        rows[lane]["samples"]
            .as_array()
            .ok_or("missing seam history")
    };
    let mut maximum = 0_f64;
    for lane in 0..2 {
        for s in samples(lane)? {
            let panels = s["panels"]
                .as_array()
                .filter(|v| v.len() == 2)
                .ok_or("missing reactions")?;
            for p in panels {
                maximum = maximum.max(vector::<3>(&p["force_on_foot_n"])?[2]);
            }
        }
    }
    if maximum <= 0. {
        return Err("missing positive seam reaction".into());
    }
    let mut checked = 0;
    for frame in archive.as_array().ok_or("missing archive")? {
        let ns = frame["ns"].as_u64().ok_or("missing clock")?;
        for lane in 0..2 {
            let history = samples(lane)?;
            let s = history
                .iter()
                .find(|s| s["tick"].as_u64().is_some_and(|t| t * 500000 == ns))
                .ok_or("unmatched physical tick")?;
            let prefix = rows[lane]["name"].as_str().ok_or("missing material name")?;
            let check = |suffix: &str, p: [f64; 3]| -> Result<()> {
                let name = format!("default::{prefix} {suffix}::visual");
                let vis = frame["scene"]["visuals"]
                    .as_array()
                    .and_then(|v| v.iter().find(|r| r["name"] == name))
                    .ok_or("missing rendered seam pose")?;
                if !same(&vis["translation"], p)? {
                    return Err(format!("wrong {name} position at {ns} ns").into());
                }
                Ok(())
            };
            let lane_y = lane as f64 - 0.5;
            let [x, y, z] = vector::<3>(&s["center_m"])?;
            check("foot", [-y, x + lane_y, z])?;
            for (i, name) in ["left", "right"].iter().enumerate() {
                let p = &s["panels"][i];
                let [x, y, z] = vector::<3>(&p["center_m"])?;
                check(&format!("{name} panel"), [-y, x + lane_y, z])?;
                let f = vector::<3>(&p["force_on_foot_n"])?[2];
                check(
                    &format!("{name} normal load"),
                    [0., x + lane_y + 0.28 * (f / maximum - 0.5), -0.18],
                )?;
                checked += 1;
            }
        }
    }
    Ok(checked)
}
