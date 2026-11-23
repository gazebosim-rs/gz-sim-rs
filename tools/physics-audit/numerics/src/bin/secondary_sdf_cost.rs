// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconcile repeat timings with the public SDF states and independently checked physics.
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn rows(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing rows".into())
}
fn dist(mut xs: Vec<u64>) -> Value {
    xs.sort_unstable();
    let sample =
        |percent: usize| -> f64 { xs[(percent * xs.len()).div_ceil(100) - 1] as f64 * 0.001 };
    json!({"samples":xs.len(),"p50_us":sample(50),"p95_us":sample(95),"max_us":sample(100)})
}
fn main() -> Result<()> {
    let inputs = std::env::args_os().skip(1).collect::<Vec<_>>();
    if inputs.len() != 1 {
        return Err("secondary_sdf_cost SDF_STUDY".into());
    }
    let input = Path::new(&inputs[0]);
    let source = read(&input.join("study.json"))?;
    let material = read(&input.join("report/authored.json"))?;
    let cases = rows(&source["cases"])?;
    if cases.len() != 50 {
        return Err("expected 50 complete SDF worlds per profile".into());
    }
    let mut pooled: BTreeMap<(u64, bool, String, String), Vec<u64>> = BTreeMap::new();
    let mut donors = Vec::new();
    let mut ids = BTreeSet::new();
    for case in cases {
        let id = case["case"].as_u64().ok_or("missing coordinate")?;
        if !ids.insert(id) {
            return Err("repeated donor".into());
        }
        let held = case["held"].as_bool().ok_or("missing split")?;
        for repeat in [1, 2] {
            let raw = read(&input.join(format!("cost/repeat-{repeat}/case-{id:03}.json")))?;
            if raw["case"] != id
                || raw["slot"] != case["slot"]
                || raw["held"] != held
                || raw["repeat"] != repeat
            {
                return Err("repeat source coordinate mismatch".into());
            }
            let mut lanes = BTreeSet::new();
            for profile in rows(&raw["rows"])? {
                let label = profile["lane"].as_str().ok_or("missing profile")?;
                if !lanes.insert(label) {
                    return Err("duplicate profile".into());
                }
                let reference = rows(&case["rows"])?
                    .iter()
                    .find(|r| r["lane"] == label)
                    .ok_or("missing physical control")?;
                let audit = rows(&material["rows"])?
                    .iter()
                    .find(|r| r["lane"] == label && r["case"] == id)
                    .ok_or("missing physical readback")?;
                if profile["normal_ns"] != reference["normal_ns"]
                    || profile["calls"] != reference["calls"]
                    || profile["calls"] != audit["calls"]
                {
                    return Err("timing changed public impulse or native counts".into());
                }
                let mut row = json!({"case":id,"held":held,"repeat":repeat,"lane":label,"calls":profile["calls"],"impulse_error_ns":audit["momentum_vector_error_ns"]});
                for clock in ["wall_ns", "thread_ns"] {
                    let sample = rows(&profile[clock])?
                        .iter()
                        .map(|x| x.as_u64().filter(|t| *t > 0).ok_or("invalid time"))
                        .collect::<std::result::Result<Vec<_>, _>>()?;
                    if sample.len() != 32 {
                        return Err("missing public step measurements".into());
                    }
                    pooled
                        .entry((repeat, held, label.into(), clock.into()))
                        .or_default()
                        .extend_from_slice(&sample);
                    row[clock] = dist(sample);
                }
                donors.push(row);
            }
            if lanes.len() != rows(&case["rows"])?.len() {
                return Err("incomplete treatment cohort".into());
            }
        }
    }
    let mut summary = Vec::new();
    for ((repeat, held, lane, clock), values) in pooled {
        if values.len() != 800 {
            return Err("held/calibration split incomplete".into());
        }
        let distribution = dist(values);
        summary.push(json!({"repeat":repeat,"held":held,"lane":lane,"clock":clock,"distribution":distribution}));
    }
    fs::write(
        input.join("report/cost.json"),
        serde_json::to_vec(&json!({"source_worlds":ids.len(),"timings":donors,"summary":summary}))?,
    )?;
    Ok(())
}
