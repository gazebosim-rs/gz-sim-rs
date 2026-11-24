// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Chromium wire readback for the two independently measured support reactions.
use super::{Result, same, vector};
use serde_json::Value;

pub(super) const EVENTS: &[(&str, &str)] = &[
    ("a-heavy", "peak_a_share_tick"),
    ("b-heavy", "peak_b_share_tick"),
    ("transfer", "load_transfer_tick"),
    ("demotion", "demotion_tick"),
    ("loss", "loss_tick"),
    ("resumed", "promotion_tick"),
];

pub(super) fn scene(scene: &Value, trace: &Value, index: &Value, tick: u64) -> Result<()> {
    let row = &trace["frames"][tick as usize];
    let g = &trace["geometry"];
    let b_present = row["support_b_present"]
        .as_bool()
        .ok_or("support presence")?;
    let shapes = scene["visuals"].as_array().ok_or("visuals")?;
    if shapes.len() != 26 + 2 * usize::from(b_present) {
        return Err(format!("unexpected topology at {tick}").into());
    }
    let field = |k: &str| index[k].as_f64().ok_or_else(|| format!("missing {k}"));
    let span = field("spacing_m")?;
    let cx = field("center_x")?;
    let width = field("lane_width")?;
    let ruler = field("gauge_width")?;
    let z = field("gauge_z")?;
    let max = field("gauge_max_ns")?;
    for (lane, name, shift) in [
        ("switched", "switched", span / 2.),
        ("reference", "always_dynamic", -span / 2.),
    ] {
        let find = |key: &str| {
            shapes
                .iter()
                .find(|v| v["name"] == format!("default::{name}_{key}::visual"))
        };
        let point = |[x, y, z]: [f64; 3]| [y, shift - (x - cx), z];
        for key in ["foot", "upper"] {
            let shape = find(key).ok_or_else(|| format!("missing {name}_{key} at {tick}"))?;
            let p = &row[lane][key];
            let [qx, qy, qz, qw] = vector(&p["orientation"])?;
            let c = std::f64::consts::FRAC_1_SQRT_2;
            let q = [c * (qx + qy), c * (qy - qx), c * (qz - qw), c * (qw + qz)];
            if shape["shape"]["kind"] != "box"
                || !same(&shape["translation"], point(vector(&p["position"])?))?
                || !same(&shape["rotation"], q)?
                || !same(
                    &shape["shape"]["size"],
                    vector::<3>(&g[format!("{key}_half_extents")])?.map(|a| a * 2.),
                )?
            {
                return Err(format!("incorrect {name}/{key} pose/shape at {tick}").into());
            }
        }
        let loads = vector::<2>(&row[lane]["normal_impulses_ns"])?;
        for (j, key) in ["a", "b"].iter().enumerate() {
            let support = find(&format!("support_{key}"));
            if (j == 0 || b_present) != support.is_some() {
                return Err(format!("support {key} presence at {tick}").into());
            }
            if let Some(v) = support {
                let center = vector::<3>(&g[format!("support_{key}_center")])?;
                let size = vector::<3>(&g[format!("support_{key}_half_extents")])?.map(|n| 2. * n);
                if v["shape"]["kind"] != "box"
                    || !same(&v["translation"], point(center))?
                    || !same(&v["shape"]["size"], size)?
                {
                    return Err(format!("support {key} geometry at {tick}").into());
                }
            }
            let marker = find(&format!("{key}_load")).ok_or("load marker")?;
            let p = point([
                cx + (loads[j] / max - 0.5) * ruler,
                0.,
                z - (j as f64 * 0.17 - 0.034) * width,
            ]);
            if !same(&marker["translation"], p)? {
                return Err(format!("incorrect {lane}/{key} load at {tick}").into());
            }
        }
    }
    Ok(())
}

pub(super) fn catalog(catalog: &Value, present: bool) -> Result<()> {
    let items = catalog["models"].as_array().ok_or("catalog models")?;
    for lane in ["switched", "always_dynamic"] {
        for (name, exists) in [("a", true), ("b", present)] {
            let key = format!("default::{lane}_support_{name}");
            if items.iter().any(|v| v["name"] == key) != exists {
                return Err(format!("incorrect support catalog entry {key}").into());
            }
        }
    }
    Ok(())
}
