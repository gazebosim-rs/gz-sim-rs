// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent comparisons on identical three-stroke fixtures.
use super::*;
use glam::DVec3;

#[path = "secondary.rs"]
pub(crate) mod secondary;

#[path = "physical_long.rs"]
pub(crate) mod physical;

#[path="internal_long.rs"]
pub(crate) mod internal;
#[path="fine_onset.rs"]
pub(crate) mod fine_onset;
#[path = "whole_long.rs"]
pub(crate) mod whole;
#[path = "physical_handoff.rs"]
pub(crate) mod handoff;
#[path = "fine_suffix.rs"]
pub(crate) mod suffix;
type Rows = BTreeMap<(String, String), Series>;
fn vec3(v: &Value) -> Result<DVec3> {
    let [x, y, z] = array(v)? else {
        return Err("missing 3-vector".into());
    };
    Ok(DVec3::new(num(x)?, num(y)?, num(z)?))
}
fn com(t: &Value) -> Result<DVec3> {
    Ok((vec3(&t["foot"]["position"])? + vec3(&t["upper"]["position"])?) * 0.5)
}
fn store(rows: &mut Rows, pair: &str, key: &str, value: f64, k: usize) {
    rows.entry((pair.into(), key.into()))
        .or_default()
        .push(value, k);
}
fn rows_value(r: Rows) -> Value {
    json!(r.into_iter().map(|((comparison,metric),series)|json!({"comparison":comparison,"metric":metric,"statistics":series.stats()})).collect::<Vec<_>>())
}
fn file(dir: &Path, k: usize) -> PathBuf {
    dir.join("cases").join(format!("case-{k:03}.json"))
}
fn subset(coordinate: usize) -> Option<usize> {
    (0..150).find(|k| {
        let i = k * 71 % 150;
        coordinate == 10 * (i / 6) + 4 * ((i / 2) % 3) + i % 2
    })
}
fn repeated(local: &Value, foreign: &Value, other_type: bool) -> bool {
    let (calls, events) = if other_type {
        ("calls", "events")
    } else {
        ("native_calls", "outcomes")
    };
    local["end"] == foreign["end"]
        && local["outcomes"] == foreign[events]
        && local["native_calls"] == foreign[calls]
}
fn close(a: f64, b: f64) -> Result<()> {
    if !a.is_finite() || !b.is_finite() || (a - b).abs() > 3e-12 * (1. + a.abs().max(b.abs())) {
        return Err(format!("independent reconstruction disagrees {a} vs {b}").into());
    }
    Ok(())
}
fn cross_checks(result: &Value, other: &Value) -> Result<()> {
    if result["fixture"] != other["fixture"]
        || result["cut"] != other["cut"]
        || result["end_tick"] != other["end_tick"]
    {
        return Err("cross-campaign state or clock mismatch".into());
    }
    Ok(())
}
fn endpoint(rows: &mut Rows, label: &str, a: &Value, b: &Value, k: usize) -> Result<()> {
    let end = com(&a["end"])?.distance(com(&b["end"])?);
    store(rows, label, "final_com_m", end, k);
    store(
        rows,
        label,
        "final_spin_rad_s",
        vec3(&a["end"]["foot"]["angular_velocity"])?
            .distance(vec3(&b["end"]["foot"]["angular_velocity"])?),
        k,
    );
    let ea = &a["outcomes"];
    let eb = if b["events"].is_null() {
        &b["outcomes"]
    } else {
        &b["events"]
    };
    store(
        rows,
        label,
        "signed_total_normal_change_ns",
        num(&ea["foot_normal_ns"])? - num(&eb["foot_normal_ns"])?,
        k,
    );
    store(
        rows,
        label,
        "total_tangent_difference_ns",
        vec3(&ea["foot_tangent_ns"])?.distance(vec3(&eb["foot_tangent_ns"])?),
        k,
    );
    for i in 0..3 {
        let (x, y) = (&ea["strokes"][i], &eb["strokes"][i]);
        match (
            x["first_native_end_ns"].as_u64(),
            y["first_native_end_ns"].as_u64(),
        ) {
            (Some(t), Some(u)) => {
                store(
                    rows,
                    label,
                    &format!("stroke{}_abs_arrival_us", i + 1),
                    t.abs_diff(u) as f64 * 0.001,
                    k,
                );
                if x["window_public_ticks"] == 10 && y["window_public_ticks"] == 10 {
                    store(
                        rows,
                        label,
                        &format!("stroke{}_abs_first_5ms_normal_ns", i + 1),
                        (num(&x["sum_first_ten_loaded_tick_window_ns"])?
                            - num(&y["sum_first_ten_loaded_tick_window_ns"])?)
                        .abs(),
                        k,
                    );
                }
            }
            (None, None) => store(rows, label, &format!("stroke{}_both_missing", i + 1), 1., k),
            _ => store(rows, label, &format!("stroke{}_only_one", i + 1), 1., k),
        }
    }
    Ok(())
}
fn labels<'a>(
    paths: &'a Value,
    names: &[&str],
    n: u64,
    starts: &[u64],
    tails: &[u64],
) -> Result<&'a [Value]> {
    let v = array(paths)?;
    if n == 0 || v.len() != names.len() {
        return Err("missing public samples or paths".into());
    }
    for (i, p) in v.iter().enumerate() {
        if p["profile"] != names[i] || uint(&p["native_calls"])? != starts[i] + (n - 1) * tails[i] {
            return Err("bad profile or work count".into());
        }
    }
    Ok(v)
}
pub(super) fn finite(dir: &Path, orig: &Path, conv: &Path) -> Result<()> {
    let names = [
        "8r1_then512",
        "finite8_then512",
        "finite2_then512",
        "native8",
        "finite8",
        "native64",
        "finite64",
    ];
    let mut rows = Rows::new();
    let mut counts = [0usize; 4];
    let mut worst: Option<(f64, usize)> = None;
    let mut censored = vec![];
    for k in 0..250 {
        if !file(dir, k).exists() {
            continue;
        }
        let doc = read(&file(dir, k))?;
        if doc["case"] != k || doc["source"] != k || doc["format"] != "finite-three-stroke-v1" {
            return Err("invalid finite long archive".into());
        }
        counts[0] += 1;
        let r = &doc["result"];
        if r["cut"].is_null() {
            censored.push(k);
            continue;
        }
        let n = uint(&r["end_tick"])? - uint(&r["cut_tick"])?;
        let paths = labels(
            &r["paths"],
            &names,
            n,
            &[8, 8, 2, 8, 8, 64, 64],
            &[512, 512, 512, 8, 8, 64, 64],
        )?;
        for (i, p) in paths.iter().enumerate() {
            let peer = [0, 0, 0, 3, 3, 5, 5][i];
            let tag = format!("{}_vs_{}", names[i], names[peer]);
            endpoint(&mut rows, &tag, p, &paths[peer], k)?;
            let t = &p["trajectory_change"];
            close(
                num(&t["final_com_m"])?,
                com(&p["end"])?.distance(com(&paths[peer]["end"])?),
            )?;
            for f in [
                "rms_com_m",
                "max_com_m",
                "rms_spin_rad_s",
                "max_public_reaction_difference_ns",
            ] {
                store(&mut rows, &tag, f, num(&t[f])?, k)
            }
            let hits = array(&p["impacts"])?;
            if ([0, 3, 5].contains(&i) && !hits.is_empty())
                || ([1, 2].contains(&i) && hits.iter().any(|h| h["tick"] != p["first"]["tick"]))
            {
                return Err("finite impulses outside the selected interval".into());
            }
            store(
                &mut rows,
                names[i],
                "physical_projected_contacts",
                hits.len() as f64,
                k,
            );
        }
        let d = num(&paths[1]["trajectory_change"]["final_com_m"])?;
        if worst.is_none_or(|(w, _)| d > w) {
            worst = Some((d, k))
        }
        let mut references = vec![];
        if file(orig, k).exists() {
            let other = read(&file(orig, k))?;
            let s = &other["result"];
            cross_checks(r, s)?;
            if !repeated(&paths[0], &s["paths"][0], false)
                || paths[0]["first"] != s["paths"][0]["first"]
            {
                return Err(format!("ordinary history mismatch case {k}").into());
            }
            counts[1] += 1;
            for (label, j) in [("4096r1", 6), ("4096r8", 7)] {
                references.push((label, s["paths"][j].clone()));
            }
        }
        if let Some(j) = subset(k * 117 % 250)
            && file(conv, j).exists()
        {
            let other = read(&file(conv, j))?;
            let s = &other["result"];
            cross_checks(r, s)?;
            if !repeated(&paths[0], &s["paths"][3], true) {
                return Err("grid ordinary branch mismatch".into());
            }
            counts[2] += 1;
            references.push(("8192r1", s["paths"][2].clone()));
        }
        for (name, reference) in references {
            for (i, p) in paths.iter().enumerate() {
                let label = format!("{}_vs_{name}", names[i]);
                endpoint(&mut rows, &label, p, &reference, k)?;
                if [1, 2, 4, 6].contains(&i) {
                    let peer = [0, 0, 0, 3, 3, 5, 5][i];
                    let target = com(&reference["end"])?;
                    let change = com(&p["end"])?.distance(target)
                        - com(&paths[peer]["end"])?.distance(target);
                    store(
                        &mut rows,
                        &label,
                        "change_final_com_vs_ordinary_m",
                        change,
                        k,
                    );
                    store(
                        &mut rows,
                        &label,
                        "fraction_improved_final_com",
                        f64::from(change < 0.),
                        k,
                    );
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"cases":counts[0],"complete":counts[0]==250,"censored":censored,"compared_propagation":counts[1],"compared_grid_subset":counts[2],"largest_causal_case":worst,"rows":rows_value(rows)}),
    )
}
pub(super) fn deep(dir: &Path, conv: &Path, finite: &Path) -> Result<()> {
    let mut rows = Rows::new();
    let mut counts = [0usize; 3];
    let mut worst = None;
    let mut ranks = BTreeMap::new();
    let mut censored = vec![];
    let names = ["8192", "16384", "32768", "8r1_then512", "finite8_then512"];
    for k in 0..50 {
        if !file(dir, k).exists() {
            continue;
        }
        let doc = read(&file(dir, k))?;
        let orig = uint(&doc["source"])? as usize;
        let j = k * 17 % 50;
        if doc["case"] != k
            || doc["format"] != "nominal-three-stroke-deep-v1"
            || orig >= 250
            || orig * 117 % 250 != 10 * (j / 2) + 4 + j % 2
        {
            return Err("deep coordinates mismatch".into());
        }
        counts[0] += 1;
        let r = &doc["result"];
        if r["cut"].is_null() {
            censored.push(k);
            continue;
        }
        let cut = uint(&r["cut_tick"])?;
        let n = uint(&r["end_tick"])? - cut;
        let paths = labels(
            &r["paths"],
            &names,
            n,
            &[8192, 16384, 32768, 8, 8],
            &[8192, 16384, 32768, 512, 512],
        )?;
        let file_j = subset(orig * 117 % 250).ok_or("deep fixture absent from grid")?;
        for (dir, j, other_type) in [(conv, file_j, true), (finite, orig, false)] {
            if !file(dir, j).exists() {
                continue;
            }
            let d = read(&file(dir, j))?;
            let other = &d["result"];
            cross_checks(r, other)?;
            let pairs = if other_type {
                &[(0, 2), (3, 3)][..]
            } else {
                &[(3, 0), (4, 1)]
            };
            for &(here, there) in pairs {
                if !repeated(&paths[here], &other["paths"][there], other_type) {
                    return Err(format!("deep repeat {k}/{here}/{there}").into());
                }
            }
            counts[if other_type { 1 } else { 2 }] += 1;
        }
        let tr = read(&dir.join("trajectories").join(format!("case-{k:03}.json")))?;
        if tr["case"] != k || tr["source"] != orig || array(&tr["paths"])?.len() != 5 {
            return Err("invalid trajectory source".into());
        }
        let mut histories = vec![];
        for (i, p) in array(&tr["paths"])?.iter().enumerate() {
            let ticks = array(&p["ticks"])?;
            if ticks.len() as u64 != n || p["profile"] != names[i] {
                return Err("incomplete deep trajectory".into());
            }
            let mut h = vec![];
            let mut normal = 0.;
            let mut tangent = DVec3::ZERO;
            for (idx, row) in ticks.iter().enumerate() {
                let [t, c, w, j, xy, onset] = array(row)? else {
                    return Err("bad deep sample".into());
                };
                if uint(t)? != cut + idx as u64 + 1 || onset.as_u64().is_some_and(|x| x > 500000) {
                    return Err("bad deep clock".into());
                }
                let z = num(j)?;
                if z < 0. {
                    return Err("negative compression".into());
                }
                normal += z;
                tangent += vec3(xy)?;
                h.push((vec3(c)?, vec3(w)?, z));
            }
            close(normal, num(&paths[i]["outcomes"]["foot_normal_ns"])?)?;
            close(
                tangent.distance(vec3(&paths[i]["outcomes"]["foot_tangent_ns"])?),
                0.,
            )?;
            close(h.last().unwrap().0.distance(com(&paths[i]["end"])?), 0.)?;
            histories.push(h);
        }
        let mut rms = [[0.; 3]; 5];
        for (i, h) in histories.iter().enumerate() {
            for (j, g) in histories.iter().take(3).enumerate() {
                let (mut pos, mut spin, mut peak, mut load) = (0., 0., 0.0_f64, 0.0_f64);
                for (a, b) in h.iter().zip(g) {
                    let d = a.0.distance(b.0);
                    pos += d * d;
                    spin += a.1.distance_squared(b.1);
                    peak = peak.max(d);
                    load = load.max((a.2 - b.2).abs())
                }
                rms[i][j] = (pos / n as f64).sqrt();
                let final_d = h.last().unwrap().0.distance(g.last().unwrap().0);
                let label = format!("{}_vs_{}", names[i], names[j]);
                for (metric, val) in [
                    ("rms_com_m", rms[i][j]),
                    ("rms_spin_rad_s", (spin / n as f64).sqrt()),
                    ("max_com_m", peak),
                    ("max_public_reaction_difference_ns", load),
                    ("final_com_m", final_d),
                ] {
                    close(val, num(&paths[i]["against"][j][metric])?)?;
                    store(&mut rows, &label, metric, val, k);
                }
                if i == 1 && j == 2 && worst.is_none_or(|(old, _)| final_d > old) {
                    worst = Some((final_d, k))
                }
            }
        }
        if rms[0][1] > 0. {
            store(
                &mut rows,
                "successive_grids",
                "fine_to_coarse_rms_ratio",
                rms[1][2] / rms[0][1],
                k,
            )
        }
        let winner = |i| {
            if rms[4][i] + 1e-12 < rms[3][i] {
                "finite"
            } else if rms[3][i] + 1e-12 < rms[4][i] {
                "native"
            } else {
                "tie"
            }
        };
        *ranks
            .entry(format!("{}/{}/{}", winner(0), winner(1), winner(2)))
            .or_insert(0usize) += 1;
    }
    write(
        &dir.join("summary.json"),
        &json!({"cases":counts[0],"complete":counts[0]==50,"censored":censored,"matched_grid":counts[1],"matched_finite":counts[2],"largest_fine_grid_case":worst,"treatment_rms_rank_by_reference":ranks,"rows":rows_value(rows)}),
    )
}
pub(super) fn retimed(
    dir: &Path,
    finite: &Path,
    prop: &Path,
    conv: &Path,
    deep: &Path,
) -> Result<()> {
    let mut counts = BTreeMap::<String, usize>::new();
    let mut stats = Rows::new();
    let mut largest = (0., 0);
    let mut worst_deep = None;
    let count = |counts: &mut BTreeMap<String, usize>, s: &str| {
        *counts.entry(s.to_owned()).or_default() += 1
    };
    let mut ranks = BTreeMap::<String, usize>::new();
    for k in 0..250 {
        if !file(dir, k).exists() {
            continue;
        }
        let v = read(&file(dir, k))?;
        let baseline = read(&file(finite, k))?;
        let b = &baseline["result"];
        cross_checks(&v, b)?;
        let cut = uint(&b["cut_tick"])?;
        let len = uint(&v["end_tick"])? - cut;
        let names = ["finite8_then512", "retimed8_then512", "native8_then512"];
        let paths = labels(&v["paths"], &names, len, &[8; 3], &[512; 3])?;
        if v["format"] != "retimed-first-three-strokes-v1" || v["case"] != k {
            return Err("retiming fixture identity changed".into());
        }
        for (i, j) in [(0, 1), (2, 0)] {
            for key in ["first", "end", "native_calls", "outcomes", "impacts"] {
                if paths[i][key] != b["paths"][j][key] {
                    return Err(format!("retiming control {k} changed: {key}").into());
                }
            }
        }
        let a = array(&paths[0]["impacts"])?;
        let z = array(&paths[1]["impacts"])?;
        let both = match (a, z) {
            ([], []) => false,
            ([a], [z]) => {
                let mut original = z.clone();
                let ret = original
                    .as_object_mut()
                    .ok_or("invalid impact")?
                    .remove("retimed");
                if &original != a {
                    return Err("first physical impulse changed under retiming".into());
                }
                ret.is_some()
            }
            _ => return Err("intervention changed first physical site".into()),
        };
        count(&mut counts, "fixtures");
        if both {
            count(&mut counts, "retimed")
        }
        let raw = read(&dir.join("trajectories").join(format!("case-{k:03}.json")))?;
        let series = array(&raw["paths"])?;
        if raw["case"] != k || series.len() != 3 {
            return Err("retimed raw source mismatch".into());
        }
        let mut data = Vec::new();
        for (i, p) in series.iter().enumerate() {
            let ticks = array(&p["ticks"])?;
            if ticks.len() as u64 != len || p["profile"] != names[i] {
                return Err("retimed raw budget mismatch".into());
            }
            let mut positions = vec![];
            for (j, t) in ticks.iter().enumerate() {
                if uint(&t[0])? != cut + j as u64 + 1 {
                    return Err("lost actuator tick".into());
                }
                positions.push(vec3(&t[1])?);
                if j == 0 || j + 1 == ticks.len() {
                    let fr = &paths[i][if j == 0 { "first" } else { "end" }];
                    close(com(fr)?.distance(*positions.last().unwrap()), 0.)?;
                    close(num(&t[3])?, num(&fr["normal_impulse_ns"])?)?;
                    if t[2] != fr["foot"]["angular_velocity"]
                        || t[4] != fr["tangent_on_foot_ns"]
                        || t[5] != fr["first_local_ns"]
                    {
                        return Err("retimed endpoint reconstruction mismatch".into());
                    }
                }
            }
            data.push(positions);
        }
        if !both && series[0]["ticks"] != series[1]["ticks"] {
            return Err("inactive retiming changed the public state".into());
        }
        let discrepancy = |u: &[DVec3], v: &[DVec3]| {
            let sum = u
                .iter()
                .zip(v)
                .map(|(x, y)| x.distance_squared(*y))
                .sum::<f64>();
            (
                (sum / u.len() as f64).sqrt(),
                u.last().unwrap().distance(*v.last().unwrap()),
            )
        };
        let (rms, last) = discrepancy(&data[1], &data[0]);
        close(num(&v["change"]["rms_com_m"])?, rms)?;
        close(num(&v["change"]["final_com_m"])?, last)?;
        store(&mut stats, "causal", "rms_com_m", rms, k);
        store(&mut stats, "causal", "final_com_m", last, k);
        if last > largest.0 {
            largest = (last, k)
        }
        let coord = k * 117 % 250;
        let mut refs = vec![];
        if file(prop, k).exists() {
            let src = read(&file(prop, k))?;
            cross_checks(&v, &src["result"])?;
            for (i, name) in [(6, "4096r1"), (7, "4096r8")] {
                refs.push((name, src["result"]["paths"][i].clone()))
            }
        }
        if let Some(i) = subset(coord).filter(|i| file(conv, *i).exists()) {
            let src = read(&file(conv, i))?;
            cross_checks(&v, &src["result"])?;
            refs.push(("8192r1", src["result"]["paths"][2].clone()));
        }
        for (name, reference) in refs {
            let target = com(&reference["end"])?;
            let diff =
                com(&paths[1]["end"])?.distance(target) - com(&paths[0]["end"])?.distance(target);
            store(&mut stats, name, "retimed_minus_uniform_final_m", diff, k);
            if diff < 0. {
                count(&mut counts, &format!("improved_{name}"))
            }
            endpoint(
                &mut stats,
                &format!("retimed_vs_{name}"),
                &paths[1],
                &reference,
                k,
            )?;
        }
        if let Some(j) = (0..50)
            .find(|j| {
                let q = j * 17 % 50;
                coord == 10 * (q / 2) + 4 + q % 2
            })
            .filter(|j| file(deep, *j).exists())
        {
            let dx = read(&file(deep, j))?;
            if dx["source"] != k {
                return Err("invalid nominal link".into());
            }
            cross_checks(&v, &dx["result"])?;
            let dr = read(&deep.join("trajectories").join(format!("case-{j:03}.json")))?;
            let rows = array(&dr["paths"])?;
            if rows[3]["ticks"] != series[2]["ticks"] || rows[4]["ticks"] != series[0]["ticks"] {
                return Err("nominal entire common history changed".into());
            }
            count(&mut counts, "deep_nominal");
            let mut rank = vec![];
            for (i, name) in ["8192", "16384", "32768"].into_iter().enumerate() {
                let positions = array(&rows[i]["ticks"])?
                    .iter()
                    .map(|t| vec3(&t[1]))
                    .collect::<Result<Vec<_>>>()?;
                let (uniform, end) = discrepancy(&data[0], &positions);
                let (retimed, end2) = discrepancy(&data[1], &positions);
                for (m, val) in [
                    ("uniform_rms_m", uniform),
                    ("retimed_rms_m", retimed),
                    ("retimed_minus_uniform_rms_m", retimed - uniform),
                    ("change_final_m", end2 - end),
                ] {
                    store(&mut stats, name, m, val, k);
                }
                rank.push(if retimed + 1e-12 < uniform {
                    "retimed"
                } else if uniform + 1e-12 < retimed {
                    "uniform"
                } else {
                    "tie"
                });
                if i == 2 && worst_deep.is_none_or(|(w, _)| retimed - uniform > w) {
                    worst_deep = Some((retimed - uniform, k))
                }
            }
            count(&mut ranks, &rank.join("/"));
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"largest_causal_terminal_m_case":largest,"largest_32768_rms_regression_m_case":worst_deep,"nominal_ranking":ranks,"rows":rows_value(stats)}),
    )
}
