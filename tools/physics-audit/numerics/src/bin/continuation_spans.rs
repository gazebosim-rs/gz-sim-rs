// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent readback of all six public-receipt durations, using both raw high-grid archives.
use glam::DVec3;
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const WEIGHT: f64 = 2. * 9.81 * 0.0005;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn arr(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "missing array".into())
}
fn n(v: &Value) -> Result<usize> {
    v.as_u64()
        .and_then(|n| n.try_into().ok())
        .ok_or_else(|| "missing integer".into())
}
fn real(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "missing finite value".into())
}
fn vec(v: &Value) -> Result<DVec3> {
    let a: [f64; 3] = serde_json::from_value(v.clone())?;
    Ok(DVec3::from(a))
}
fn choose(v: &Value, f: impl Fn(&Value) -> bool) -> Result<&Value> {
    arr(v)?
        .iter()
        .find(|s| f(s))
        .ok_or_else(|| "measurement absent".into())
}
fn near(a: f64, b: f64) -> Result<()> {
    if (a - b).abs() > 1e-10 * a.abs().max(b.abs()).max(1.) {
        return Err(format!("numeric mismatch {a} vs {b}").into());
    }
    Ok(())
}
fn check_stats(s: &Value, v: &mut [f64]) -> Result<()> {
    if n(&s["n"])? != v.len() {
        return Err("incorrect statistic population".into());
    }
    if v.is_empty() {
        return Ok(());
    }
    near(real(&s["mean"])?, v.iter().sum::<f64>() / v.len() as f64)?;
    v.sort_unstable_by(f64::total_cmp);
    near(real(&s["p50"])?, v[v.len().div_ceil(2) - 1])?;
    near(real(&s["p95"])?, v[(95 * v.len()).div_ceil(100) - 1])?;
    let positive = v.iter().filter(|&&x| x > 0.).count();
    if n(&s["positive"])? != positive || n(&s["negative"])? != v.iter().filter(|&&x| x < 0.).count()
    {
        return Err("sign counts differ".into());
    }
    Ok(())
}
#[derive(Default)]
struct Sample {
    e: Vec<f64>,
    de: Vec<f64>,
    spin: Vec<f64>,
    dj: Vec<f64>,
}
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 4 {
        return Err("continuation_spans SPAN_DIR RELAXATION_DIR PRIMARY_DIR REPORT_DIR".into());
    }
    let out = Path::new(&args[3]);
    let result = read(&out.join("continuation-spans.json"))?;
    let scheduled = n(&result["complete"])?;
    let mut completed = 0;
    let mut donors = 0;
    let mut pending = 0;
    let mut absent = 0;
    let mut hist = BTreeMap::<(usize, usize, usize, usize), Sample>::new();
    let mut release = BTreeMap::<(usize, usize, usize), ([usize; 3], Vec<f64>, Vec<f64>)>::new();
    let mut budget = BTreeMap::<(usize, usize), Vec<f64>>::new();
    let mut max: Option<(usize, usize, f64)> = None;
    let mut shared_intervals = 0;
    for i in 0..1250 {
        // Fixed publication snapshots may be checked after additional cases exist; read exactly the published IDs.
        let name = format!("cases/case-{i:04}.json");
        let file = Path::new(&args[0]).join(&name);
        if !file.exists() {
            continue;
        }
        if let Some(indices) = result["completed_indices"].as_array()
            && !indices.iter().any(|v| v == i)
        {
            continue;
        }
        if result["completed_indices"].is_null() && completed == scheduled {
            break;
        }
        completed += 1;
        let finite = read(&file)?;
        let cuts = arr(&finite["result"]["trials"])?;
        absent += 3 - cuts.len();
        let op = Path::new(&args[2]).join(&name);
        let rp = Path::new(&args[1]).join(&name);
        let omitted = result["paired_reference_indices"]
            .as_array()
            .is_some_and(|indices| !indices.iter().any(|v| v == i));
        if omitted || !rp.exists() || !op.exists() {
            pending += cuts.len();
            continue;
        }
        let r = read(&rp)?;
        let o = read(&op)?;
        for s in cuts {
            let stroke = n(&s["stroke"])?;
            let tick = n(&s["cut"]["tick"])?;
            let rt = choose(&r["result"]["trials"], |x| x["stroke"] == stroke)?;
            let ot = choose(&o["result"]["trials"], |x| x["stroke"] == stroke)?;
            if rt["cut"] != s["cut"] || ot["cut"] != s["cut"] {
                return Err("different native donors".into());
            }
            let ref8 = choose(&rt["paths"], |p| {
                p["profile"]["calls"] == 4096 && p["profile"]["relaxation"] == 8
            })?;
            let ref1 = &choose(&ot["paths"], |p| {
                p["method"] == "uniform" && p["calls"] == 4096
            })?["record"];
            let paths = arr(&s["paths"])?;
            if paths.len() != 6 {
                return Err("expected six causal interventions".into());
            }
            let split =
                usize::from([175, 200, 225, 250, 275].contains(&(165 + 5 * (i * 613 % 1250 / 50))));
            donors += 1;
            let mut dists: Vec<[f64; 4]> = vec![];
            for (k, p) in paths.iter().enumerate() {
                if p["samples"][0] != paths[0]["samples"][0] {
                    return Err("first public receipt differs".into());
                }
                let normals = arr(&p["normal_ns"])?
                    .iter()
                    .map(real)
                    .collect::<Result<Vec<_>>>()?;
                if normals.len() != 26 {
                    return Err("missing native public intervals".into());
                }
                let rich = if k < 5 {
                    [1, 2, 4, 8, 26][k]
                } else {
                    normals
                        .iter()
                        .position(|&j| j < WEIGHT)
                        .map_or(26, |x| x + 1)
                };
                if rich != n(&p["eight_pass_ticks"])? {
                    return Err("noncausal public budget".into());
                }
                budget.entry((split, k)).or_default().push(rich as f64);
                for q in &paths[..k] {
                    let through = rich.min(n(&q["eight_pass_ticks"])?);
                    if arr(&p["normal_ns"])?[..through] != arr(&q["normal_ns"])?[..through] {
                        return Err("unequal prior reactions at common budget".into());
                    }
                    shared_intervals += through;
                }
                for (j, cutoff) in [0., 1e-4, WEIGHT].into_iter().enumerate() {
                    let flags = normals
                        .iter()
                        .map(|&n| if j == 2 { n >= cutoff } else { n > cutoff })
                        .collect::<Vec<_>>();
                    let rr = release.entry((split, k, j)).or_default();
                    rr.0[0] += flags.iter().filter(|&&f| f).count();
                    if let Some(q) = flags.iter().position(|&f| !f) {
                        rr.0[1] += 1;
                        rr.1.push((q + 1) as f64);
                        if let Some(t) = flags.iter().skip(q + 1).position(|&f| f) {
                            rr.0[2] += 1;
                            rr.2.push((q + 2 + t) as f64);
                        }
                    }
                }
                let mut per = [0.; 4];
                for (w, step) in [10, 26].into_iter().enumerate() {
                    if paths.iter().chain([ref8]).any(|p| {
                        p["first_disjoint_tick"]
                            .as_u64()
                            .is_some_and(|t| t <= (tick + step) as u64)
                    }) {
                        continue;
                    }
                    let s = choose(&p["samples"], |s| s["i"] == step)?;
                    for (g, rf) in [ref8, ref1].into_iter().enumerate() {
                        let r = choose(&rf["samples"], |s| s["i"] == step)?;
                        let e = 1000. * vec(&s["com_m"])?.distance(vec(&r["com_m"])?);
                        per[2 * w + g] = e;
                        let vals = hist.entry((split, k, w, g)).or_default();
                        vals.e.push(e);
                        vals.de
                            .push(if k == 0 { 0. } else { e - dists[0][2 * w + g] });
                        vals.spin
                            .push(vec(&s["spin_rad_s"])?.distance(vec(&r["spin_rad_s"])?));
                        vals.dj.push(
                            real(&s["cumulative_normal_ns"])? - real(&r["cumulative_normal_ns"])?,
                        );
                    }
                }
                dists.push(per);
            }
            let terminal_admitted = !paths.iter().chain([ref8]).any(|p| {
                p["first_disjoint_tick"]
                    .as_u64()
                    .is_some_and(|t| t <= (tick + 26) as u64)
            });
            if split == 1 && terminal_admitted {
                let d = 1000.
                    * vec(&paths[4]["samples"][3]["com_m"])?
                        .distance(vec(&paths[5]["samples"][3]["com_m"])?);
                if max.is_none_or(|(pi, ps, x)| d > x || (d == x && (i, stroke) < (pi, ps))) {
                    max = Some((i, stroke, d));
                }
            }
        }
    }
    if completed != scheduled
        || donors != n(&result["joined_strokes"])?
        || pending != n(&result["pending_reference_strokes"])?
        || absent != n(&result["missing_donor_strokes"])?
    {
        return Err("incorrect published or missing coordinates".into());
    }
    let mut scores = [[0.; 5]; 2];
    let mut vectors = 0;
    for (split, name) in ["training", "held_out"].into_iter().enumerate() {
        for k in 0..6 {
            let record = &result[name]["policies"][k];
            let v = budget.entry((split, k)).or_default();
            check_stats(&record["eight_pass_intervals"], v)?;
            vectors += 1;
            for j in 0..3 {
                let rs = &record["release"][j];
                let v = release.entry((split, k, j)).or_default();
                if n(&rs["loaded_public_intervals"])? != v.0[0] {
                    return Err("positive interval count".into());
                }
                check_stats(&rs["first_subthreshold_tick"], &mut v.1)?;
                check_stats(&rs["first_later_reload_tick"], &mut v.2)?;
                vectors += 2;
            }
            for w in 0..2 {
                for g in 0..2 {
                    let v = hist.entry((split, k, w, g)).or_default();
                    let s = &record["windows"][w]["scores"][g];
                    check_stats(&s["com_error_mm"], &mut v.e)?;
                    check_stats(&s["change_vs_onset_mm"], &mut v.de)?;
                    check_stats(&s["spin_error_rad_s"], &mut v.spin)?;
                    check_stats(&s["cumulative_normal_delta_ns"], &mut v.dj)?;
                    vectors += 4;
                    if split == 0 && k < 5 && w == 1 {
                        scores[g][k] = v.e.iter().sum::<f64>() / v.e.len() as f64;
                    }
                }
            }
        }
    }
    if scores[0][0].is_finite() {
        let mins = scores.map(|a| a.into_iter().fold(f64::INFINITY, f64::min));
        let mut selection = (0, f64::INFINITY);
        for k in 0..5 {
            let regret = (scores[0][k] - mins[0]).max(scores[1][k] - mins[1]);
            near(regret, real(&result["training_worst_mean_regret_mm"][k])?)?;
            if regret < selection.1 {
                selection = (k, regret);
            }
        }
        if n(&result["selected_fixed_span"])? != [1, 2, 4, 8, 26][selection.0] {
            return Err("different calibration winner".into());
        }
    } else if !result["selected_fixed_span"].is_null() {
        return Err("calibration winner without comparable training observations".into());
    }
    if let Some((i, s, d)) = max {
        let p = &result["outlier"];
        if n(&p["site"]["index"])? != i || n(&p["site"]["stroke"])? != s {
            return Err("incorrect held-out outlier".into());
        }
        near(real(&p["causal_to_continuous_13ms_mm"])?, d)?;
    }
    fs::write(
        out.join("span-readback.json"),
        serde_json::to_vec_pretty(
            &json!({"complete_fixtures":completed,"matched_donor_cuts":donors,
    "common_public_impulse_prefix_intervals_checked":shared_intervals,"statistic_vectors_checked":vectors,"selected_fixed_span":result["selected_fixed_span"],"verified_worst_held_out_coordinate":max}),
        )?,
    )?;
    Ok(())
}
