// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare each published pose and landing indicator against independent native samples.
use super::{Result, same, vector};
use serde_json::Value;
use std::{fs, path::Path};

pub fn input(source: &Path) -> Result<Value> {
    let mut trace: Value = serde_json::from_slice(&fs::read(source.join("trajectory.json"))?)?;
    let summary: Value = serde_json::from_slice(&fs::read(source.join("summary.json"))?)?;
    trace["cycles"] = summary["cycles"].clone();
    Ok(trace)
}
pub fn scene(scene: &Value, trace: &Value, index: &Value, tick: u64) -> Result<()> {
    let step = &trace["frames"][tick as usize];
    let visuals = scene["visuals"].as_array().ok_or("missing visuals")?;
    if visuals.len() != 22 {
        return Err(format!("visual count at native step {tick}").into());
    }
    let field = |key: &str| index[key].as_f64().ok_or_else(|| format!("missing {key}"));
    let center = field("center")?;
    let width = field("width")?;
    let spacing = field("spacing")?;
    let ruler_z = field("gauge_z")?;
    let full = field("max_landing_ns")?;
    let cycles = trace["cycles"].as_array().ok_or("native cycles missing")?;
    for (lane, name, shift, landing_key) in [
        ("switched", "switched", -spacing / 2., "switched_landing"),
        (
            "reference",
            "always_dynamic",
            spacing / 2.,
            "reference_landing",
        ),
    ] {
        let get = |k: &str| {
            visuals
                .iter()
                .find(|v| v["name"] == format!("default::{name}_{k}::visual"))
                .ok_or_else(|| format!("missing {name} {k} at {tick}"))
        };
        let point = |[x, y, z]: [f64; 3]| [-y, shift + (x - center), z];
        for body in ["foot", "upper", "pad"] {
            let native = &step[lane][body];
            let v = get(body)?;
            let [x, y, z, w] = vector(&native["orientation"])?;
            let c = std::f64::consts::FRAC_1_SQRT_2;
            if v["shape"]["kind"] != "box"
                || !same(
                    &v["rotation"],
                    [c * (x - y), c * (y + x), c * (z + w), c * (w - z)],
                )?
                || !same(&v["translation"], point(vector(&native["position"])?))?
                || !same(
                    &v["shape"]["size"],
                    vector::<3>(&trace["geometry"][format!("{body}_half_extents")])?
                        .map(|h| 2. * h),
                )?
            {
                return Err(format!("pose/shape {name} {body} at {tick}").into());
            }
        }
        let mut impulse = 0.;
        for c in cycles {
            let due = c[landing_key]["tick"]
                .as_u64()
                .ok_or("native landing tick")?;
            if due <= tick {
                impulse = trace["frames"][due as usize][lane]["normal_impulse_ns"]
                    .as_f64()
                    .ok_or("missing landing impulse")?;
            }
        }
        let tier = if lane == "switched" && step["prescribed"] == true {
            0.
        } else {
            1.
        };
        for (key, value, r) in [("tier", tier, 0.), ("landing", impulse / full, 1.)] {
            let p = point([
                center + (value - 0.5) * 0.80 * width,
                0.,
                ruler_z + (0.021 - 0.13 * r) * width,
            ]);
            if !same(&get(key)?["translation"], p)? {
                return Err(format!("{key} indicator for {name} at {tick}").into());
            }
        }
    }
    Ok(())
}
