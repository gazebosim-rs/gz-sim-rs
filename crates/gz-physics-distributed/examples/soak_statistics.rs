// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Summarize independent numerical controls from the long-running physical qualification fixtures.
use clap::Parser;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "artifacts")]
    artifacts: PathBuf,
}
fn rows(path: &Path, mut consume: impl FnMut(&Value) -> Result<()>) -> Result<Value> {
    let mut r = BufReader::new(File::open(path)?);
    let mut line = String::new();
    let (mut first, mut last) = (None, None);
    let mut count = 0;
    let mut faults = BTreeMap::<String, usize>::new();
    let mut truncated = false;
    loop {
        line.clear();
        if r.read_line(&mut line)? == 0 {
            break;
        }
        if !line.ends_with('\n') {
            truncated = true;
            break;
        }
        let v: Value = serde_json::from_str(&line)?;
        let id = v["seed"].as_u64().ok_or("missing seed")?;
        if last.is_some_and(|n| n + 1 != id) {
            return Err(format!("noncontiguous seeds in {}", path.display()).into());
        }
        first.get_or_insert(id);
        last = Some(id);
        count += 1;
        if let Some(e) = v["error"].as_str() {
            *faults.entry(e.to_string()).or_default() += 1;
        }
        consume(&v)?;
    }
    Ok(
        json!({"cases":count,"first_seed":first,"last_seed":last,"errors":faults,"incomplete_final_line":truncated}),
    )
}
fn max(v: &Value, key: &str, dst: &mut BTreeMap<String, f64>) {
    if let Some(x) = v[key].as_f64() {
        let p = dst.entry(key.into()).or_insert(0.);
        *p = p.max(x);
    }
}
fn field(path: &Path) -> Result<Value> {
    let (mut ticks, mut whole, mut impulses, mut transfers, mut restarts, mut exact, mut qualified) =
        (0, 0, 0, 0, 0, 0, 0);
    let mut maxima = BTreeMap::new();
    let mut out = rows(path, |v| {
        let n = v["ticks"].as_u64().ok_or("missing field ticks")?;
        ticks += n;
        let finished = n == 216 && v["error"].is_null();
        whole += u64::from(finished);
        let hit = v["impacts"]
            .as_array()
            .map_or(0, |x| x.iter().filter(|x| x.is_u64()).count());
        impulses += hit;
        let m = v["transfers"].as_u64().unwrap_or(0);
        transfers += m;
        let restart = v["replacement"] == true;
        restarts += u64::from(restart);
        let same = v["bitwise"] == true && v["exact_contact_sets"] == true;
        exact += u64::from(finished && same);
        qualified += u64::from(finished && same && hit == 2 && m > 0 && restart);
        for key in [
            "position_m",
            "velocity_m_s",
            "orientation_rad",
            "spin_rad_s",
            "impulse_n_s",
        ] {
            max(v, key, &mut maxima);
        }
        Ok(())
    })?;
    out["measurements"] = json!({"ticks":ticks,"completed":whole,"completed_exact":exact,"all_obligations":qualified,"positive_encounters":impulses,"transfers":transfers,"replacements":restarts,"maxima":maxima});
    Ok(out)
}
fn warm(path: &Path) -> Result<Value> {
    let (mut ticks, mut same, mut complete, mut exact, mut joints, mut moves, mut impacts) =
        (0u64, 0, 0, 0, 0, 0, 0);
    let mut maxima = BTreeMap::new();
    let mut out = rows(path, |v| {
        let n = v["ticks"].as_u64().ok_or("missing warm ticks")?;
        ticks += n;
        let s = v["bitwise_frames"]
            .as_u64()
            .ok_or("missing warm exact count")?;
        same += s;
        let done = n == 335 && v["error"].is_null();
        complete += u64::from(done);
        exact += u64::from(done && n == s);
        joints += u64::from(v["warm_route"].is_u64());
        moves += u64::from(v["transfer"].is_u64());
        impacts += u64::from(v["first_cross_impact"].is_u64());
        for k in [
            "position_m",
            "velocity_m_s",
            "angle_rad",
            "spin_rad_s",
            "impulse_n_s",
            "before_join_position_m",
        ] {
            max(v, k, &mut maxima);
        }
        Ok(())
    })?;
    out["measurements"] = json!({"ticks":ticks,"bitwise_frames":same,"completed":complete,"complete_exact":exact,"warm_routes":joints,"transfers":moves,"positive_cross_impacts":impacts,"maxima":maxima});
    Ok(out)
}
#[derive(Default)]
struct Impacts {
    cases: usize,
    impacts: usize,
    frames: u64,
    exact_frames: u64,
    exact_trials: usize,
    moved: usize,
    timing: Vec<f64>,
    maxima: BTreeMap<String, f64>,
}
fn sphere(path: &Path) -> Result<Value> {
    let mut bins = BTreeMap::<u64, Impacts>::new();
    let mut out = rows(path, |v| {
        let ms = v["step_ms"].as_u64().ok_or("missing sphere step")?;
        let b = bins.entry(ms).or_default();
        let n = v["frames"].as_u64().unwrap_or(0);
        let identical = v["exact_frames"].as_u64().unwrap_or(0);
        b.cases += 1;
        b.frames += n;
        b.exact_frames += identical;
        b.moved += usize::from(v["transferred"] == true);
        if let Some(t) = v["impact"].as_u64() {
            b.impacts += 1;
            if ms > 0 {
                b.timing
                    .push(t as f64 - v["analytic_impact_s"].as_f64().unwrap() * 1000. / ms as f64);
            }
            b.exact_trials += usize::from(
                n > 0 && n == identical && v["error"].is_null() && v["transferred"] == true,
            );
        }
        for k in [
            "max_position_delta",
            "max_linear_momentum_error",
            "max_angular_momentum_error",
            "max_energy_gain",
        ] {
            max(v, k, &mut b.maxima);
        }
        Ok(())
    })?;
    for (ms, mut b) in bins {
        b.timing.sort_by(f64::total_cmp);
        let q = |p: usize| {
            b.timing
                .get((b.timing.len() * p).div_ceil(100).saturating_sub(1))
                .copied()
        };
        out["by_step_ms"][ms.to_string()] = json!({"cases":b.cases,"impacts":b.impacts,"transfers":b.moved,"frames":b.frames,"exact_frames":b.exact_frames,"complete_exact_impacts":b.exact_trials,"observed_minus_analytic_impact_in_ticks":{"min":b.timing.first(),"p50":q(50),"p95":q(95),"p99":q(99),"max":b.timing.last()},"maxima":b.maxima});
    }
    Ok(out)
}
/// Recheck only seeds refused in the original run. A missing recheck is reported
/// explicitly and never changes the original result.
fn rechecks(source: &Path, frozen: &Path, revised: &Path, field: bool) -> Result<Value> {
    let mut results = Vec::new();
    let mut reproduced = 0;
    let mut exact = 0;
    rows(source, |original| {
        let Some(original_error) = original["error"].as_str() else {
            return Ok(());
        };
        let id = original["seed"].as_u64().ok_or("missing seed")?;
        let load = |path: PathBuf| -> Result<Option<Value>> {
            if !path.try_exists()? {
                return Ok(None);
            }
            let trial: Value = serde_json::from_reader(File::open(path)?)?;
            if trial["seed"] != id {
                return Err("wrong rechecked seed".into());
            }
            Ok(Some(trial))
        };
        let old = load(frozen.join(id.to_string()).join("trials.jsonl"))?;
        let path = if field {
            revised.join(format!("{id}.jsonl"))
        } else {
            revised.join(id.to_string()).join("trials.jsonl")
        };
        let new = load(path)?;
        let same = old
            .as_ref()
            .is_some_and(|v| v["error"] == original_error && v["ticks"] == original["ticks"]);
        let qualified = new.as_ref().is_some_and(|v| {
            v["error"].is_null()
                && if field {
                    v["ticks"] == 216
                        && v["bitwise"] == true
                        && v["replacement"] == true
                        && v["exact_contact_sets"] == true
                        && v["impacts"]
                            .as_array()
                            .is_some_and(|xs| xs.len() == 2 && xs.iter().all(Value::is_u64))
                } else {
                    v["ticks"] == 335
                        && v["bitwise_frames"] == 335
                        && v["first_cross_impact"].is_u64()
                }
        });
        reproduced += usize::from(same);
        exact += usize::from(qualified);
        results.push(json!({"seed":id,"frozen_refusal_reproduced":same,"revised_completed_exact_with_impacts":qualified}));
        Ok(())
    })?;
    Ok(
        json!({"original_refusals":results.len(),"frozen_refusal_reproduced":reproduced,"revised_completed_exact_with_impacts":exact,"seeds":results}),
    )
}
fn main() -> Result<()> {
    let a = Args::parse().artifacts;
    let mut result = json!({});
    for name in ["run-0738Z", "run-strict", "run-strict-2"] {
        result["fields"][name] = field(
            &a.join("multifidelity-field-sweep")
                .join(name)
                .join("trials.jsonl"),
        )?;
    }
    let fields = a.join("multifidelity-field-sweep");
    let warm_dir = a.join("multifidelity-warm-join");
    result["fields"]["corrected_10000"] = field(&fields.join("corrected/adjacent/trials.jsonl"))?;
    result["fields"]["final_native_2048"] =
        field(&fields.join("corrected/final-native/trials.jsonl"))?;
    result["field_rechecks"] = rechecks(
        &fields.join("run-strict-2/trials.jsonl"),
        &fields.join("rechecks"),
        &fields.join("corrected"),
        true,
    )?;
    result["warm_rechecks"] = rechecks(
        &warm_dir.join("soak/trials.jsonl"),
        &warm_dir.join("rechecks"),
        &warm_dir.join("current-rechecks"),
        false,
    )?;
    result["warm_joins"] = warm(&a.join("multifidelity-warm-join/soak/trials.jsonl"))?;
    result["sphere_impacts"] = sphere(&a.join("multifidelity-rolling-sphere/sweep.jsonl"))?;
    let contact: Value = serde_json::from_reader(File::open(
        a.join("multifidelity-contact-sweep/20261119T0602Z/results/summary.json"),
    )?)?;
    for key in [
        "completed",
        "exact",
        "steps",
        "migrations",
        "worker_replacements",
        "inputs",
        "first_seed",
        "start_seed",
        "last_seed",
        "max_native_bytes",
        "category_counts",
        "peak_monolithic",
        "peak_same_partition",
        "finished",
        "interrupted_seed",
    ] {
        if let Some(v) = contact.get(key) {
            result["contact_groups"][key] = v.clone();
        }
    }
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}
