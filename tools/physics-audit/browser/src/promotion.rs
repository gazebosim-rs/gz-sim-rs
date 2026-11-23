// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join scene/catalog/status packets by publication sequence to verify repeated simulation times.
use super::{Result, same, vector};
use serde_json::{Value, json};
use std::{fs, path::Path};
pub const LABELS: &[&str] = &[
    "before_edit",
    "after_edit",
    "paused_tick_3",
    "before_prepare",
    "prepare",
    "first_solve",
];
pub fn input(dir: &Path) -> Result<Value> {
    let read = |k: &str| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(dir.join(k))?)?) };
    Ok(
        json!({"running":read("trajectory.json")?,"paused":read("paused/trajectory.json")?,"running_policy":read("summary.json")?["policy"],"paused_policy":read("paused/summary.json")?["policy"]}),
    )
}
fn boundary<'a>(policy: &'a Value, source: &str) -> Result<&'a Value> {
    if ["commit", "prepare", "before_prepare"].contains(&source) {
        return policy
            .get(source)
            .ok_or_else(|| format!("missing {source}").into());
    }
    policy["boundary"]
        .as_array()
        .ok_or("boundary records")?
        .iter()
        .find(|r| r["label"] == source)
        .ok_or_else(|| format!("missing {source}").into())
}
pub fn check(
    scene: &Value,
    catalog: &Value,
    status: &Value,
    raw: &Value,
    index: &Value,
) -> Result<u64> {
    let n = status["frame"].as_u64().ok_or("ordinal")?;
    let row = &index["frames"][usize::try_from(n.checked_sub(1).ok_or("zero ordinal")?)?];
    let tick = row["tick"].as_u64().ok_or("expected native step")?;
    let ns = scene["seconds"].as_u64().ok_or("seconds")? * 1_000_000_000
        + scene["nanoseconds"].as_u64().ok_or("nanos")?;
    if row["ordinal"] != n
        || status["recorded_sequence"] != n
        || tick * index["dt_ns"].as_u64().ok_or("dt")? != ns
        || status["scene_sequence"] != scene["sequence"]
        || catalog["scene_sequence"] != scene["sequence"]
        || catalog["epoch"] != scene["epoch"]
    {
        return Err(format!("clock or publication mismatch at ordinal {n}").into());
    }
    let actual = scene["visuals"].as_array().ok_or("visuals")?;
    let models = catalog["models"].as_array().ok_or("models")?;
    let g = &raw["running"]["geometry"];
    let width = index["width"].as_f64().ok_or("width")?;
    let cx = index["cx"].as_f64().ok_or("center")?;
    let z = index["gauge_z"].as_f64().ok_or("ruler z")?;
    let max = index["gauge_max_ns"].as_f64().ok_or("ruler span")?;
    for (name, key, shift, reference) in [
        ("running", "running", 1., false),
        ("paused_edit", "paused", 0., false),
        ("always_dynamic", "running", -1., true),
    ] {
        let source = row[key].as_str().ok_or("receipt source")?;
        let frame = &raw[key]["frames"][tick as usize];
        let (body, mode, present, paused) = if source == "step" {
            (
                &frame[if reference { "reference" } else { "switched" }],
                if !reference && frame["prescribed"] == true {
                    "prescribed"
                } else {
                    "dynamic"
                },
                frame["support_b_present"] == true,
                false,
            )
        } else {
            let r = boundary(&raw[format!("{key}_policy")], source)?;
            if (r["time_s"].as_f64().ok_or("receipt clock")? - ns as f64 * 1e-9).abs() > 1e-12 {
                return Err("incorrect measured receipt time".into());
            }
            (
                &r[if reference { "reference" } else { "sample" }],
                r[if reference { "reference_tier" } else { "tier" }]
                    .as_str()
                    .ok_or("tier")?,
                r["support_b_present"] == true,
                !reference && r["paused"] == true,
            )
        };
        let model = |k: &str| format!("default::{name}_{k}");
        let get = |k: &str| {
            actual
                .iter()
                .find(|v| v["name"] == format!("{}::visual", model(k)))
        };
        let pt = |[x, y, h]: [f64; 3]| [y, shift * 1.22 * width - (x - cx), h];
        for id in ["a", "b"] {
            let k = format!("support_{id}");
            let exists = id == "a" || present;
            let v = get(&k);
            if v.is_some() != exists || models.iter().any(|v| v["name"] == model(&k)) != exists {
                return Err(format!("pad presence at {n}").into());
            }
            if let Some(v) = v
                && (!same(
                    &v["translation"],
                    pt(vector(&g[format!("support_{id}_center")])?),
                )? || !same(
                    &v["shape"]["size"],
                    vector::<3>(&g[format!("support_{id}_half_extents")])?.map(|v| v * 2.),
                )?)
            {
                return Err("pad shape/pose".into());
            }
        }
        for id in ["foot", "upper"] {
            let v = get(id).ok_or("missing body")?;
            let p = &body[id];
            let [x, y, z, w] = vector(&p["orientation"])?;
            let c = std::f64::consts::FRAC_1_SQRT_2;
            if !same(&v["translation"], pt(vector(&p["position"])?))?
                || !same(
                    &v["rotation"],
                    [c * (x + y), c * (y - x), c * (z - w), c * (w + z)],
                )?
                || !same(
                    &v["shape"]["size"],
                    vector::<3>(&g[format!("{id}_half_extents")])?.map(|x| 2. * x),
                )?
            {
                return Err(format!("body {name}/{id} at {n}").into());
            }
        }
        for (id, level, dy) in [
            ("tier", if mode == "dynamic" { 1. } else { 0. }, 0.0),
            (
                "a_reaction",
                body["normal_impulses_ns"][0]
                    .as_f64()
                    .ok_or("pad A impulse")?
                    / max,
                -0.17,
            ),
        ] {
            let marker = get(id).ok_or("missing gauge")?;
            if !same(
                &marker["translation"],
                pt([
                    cx + (level - 0.5) * 0.76 * width,
                    0.,
                    z + (dy + 0.018) * width,
                ]),
            )? {
                return Err(format!("{name}/{id} gauge at {n}").into());
            }
        }
        for id in ["pause_0", "pause_1"] {
            if get(id).is_some() != paused {
                return Err(format!("pause state {name} at {n}").into());
            }
        }
    }
    Ok(n - 1)
}
