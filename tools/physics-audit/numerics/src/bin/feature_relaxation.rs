// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent comparison of the original serialized finite-contact, relaxation, and reference paths.
use glam::{DQuat, DVec2, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const HELD: [u64; 5] = [175, 200, 225, 250, 275];
fn load(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn list(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "expected list".into())
}
fn one(v: &Value, f: impl Fn(&Value) -> bool) -> Result<&Value> {
    let mut it = list(v)?.iter().filter(|x| f(x));
    let r = it.next().ok_or("missing observation")?;
    if it.next().is_some() {
        return Err("duplicate observation".into());
    }
    Ok(r)
}
fn num(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing finite scalar".into())
}
fn uint(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing integer".into())
}
fn vec(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(v.clone())?))
}
fn near(a: f64, b: f64) -> Result<()> {
    if (a - b).abs() > 2e-11 * a.abs().max(b.abs()).max(1.) {
        return Err(format!("mismatch {a} vs {b}").into());
    }
    Ok(())
}
fn snapshot(path: &Value, step: u64) -> Result<&Value> {
    one(&path["samples"], |s| s["i"] == step)
}
fn distances(s: &Value, reference: &Value) -> Result<[f64; 4]> {
    let distance = |k: &str| -> Result<f64> { Ok(vec(&s[k])?.distance(vec(&reference[k])?)) };
    Ok([
        distance("com_m")? * 1000.,
        distance("spin_rad_s")?,
        (num(&s["cumulative_normal_ns"])? - num(&reference["cumulative_normal_ns"])?).abs(),
        distance("cumulative_tangent_ns")?,
    ])
}
// Clip the four projected sole corners directly against the four vertical sides of the support.
// A nonempty final polygon includes a line or point touching the pad boundary.
fn overlaps(s: &Value, offset: f64) -> Result<bool> {
    let q = DQuat::from_array(serde_json::from_value(s["orientation_xyzw"].clone())?);
    let com = vec(&s["com_m"])?;
    let mut poly: Vec<DVec2> = [(-0.18, -0.10), (0.18, -0.10), (0.18, 0.10), (-0.18, 0.10)]
        .map(|(x, y)| (com + q * DVec3::new(x, y, -0.13) - DVec3::X * offset).truncate())
        .to_vec();
    for (axis, limit) in [
        (DVec2::X, 0.33),
        (-DVec2::X, 0.33),
        (DVec2::Y, 0.18),
        (-DVec2::Y, 0.18),
    ] {
        if poly.is_empty() {
            return Ok(false);
        }
        let mut out = Vec::new();
        let mut prev = *poly.last().unwrap();
        for &next in &poly {
            let a = limit - axis.dot(prev);
            let b = limit - axis.dot(next);
            if (a >= 0.) != (b >= 0.) {
                out.push(prev + (next - prev) * (a / (a - b)));
            }
            if b >= 0. {
                out.push(next);
            }
            prev = next;
        }
        poly = out;
    }
    Ok(!poly.is_empty())
}
struct Cut {
    index: u64,
    stroke: u64,
    group: String,
    stable: bool,
    admitted: bool,
    matched: bool,
    held: bool,
    // [5/13 ms][candidate][fine grid][metric]
    delta: [Option<[[[f64; 4]; 2]; 3]>; 2],
    ordinary_and_eight: [[[f64; 2]; 2]; 2],
}
fn verify_stats(report: &Value, mut values: Vec<f64>) -> Result<()> {
    if report["n"] != values.len() {
        return Err("population mismatch".into());
    }
    if values.is_empty() {
        return Ok(());
    }
    values.sort_unstable_by(f64::total_cmp);
    let n = values.len();
    for (k, x) in [
        ("min", values[0]),
        ("p50", values[n.div_ceil(2) - 1]),
        ("p95", values[(95 * n).div_ceil(100) - 1]),
        ("max", values[n - 1]),
        ("mean", values.iter().sum::<f64>() / n as f64),
    ] {
        near(num(&report[k])?, x)?;
    }
    for (k, c) in [
        ("negative", values.iter().filter(|&&x| x < 0.).count()),
        ("positive", values.iter().filter(|&&x| x > 0.).count()),
        ("zero", values.iter().filter(|&&x| x == 0.).count()),
    ] {
        if report[k] != c {
            return Err(format!("wrong {k} population").into());
        }
    }
    Ok(())
}
fn population(report: &Value, cuts: &[&Cut]) -> Result<usize> {
    if report["n"] != cuts.len() {
        return Err("wrong group size".into());
    }
    let mut count = 0;
    for (w, win) in list(&report["windows"])?.iter().enumerate() {
        let rows = cuts.iter().filter_map(|c| c.delta[w]).collect::<Vec<_>>();
        if win["qualified"] != rows.len() || win["excluded"] != cuts.len() - rows.len() {
            return Err("wrong overlap mask".into());
        }
        for p in 0..3 {
            let profile = &win["treatments"][p];
            let mut mask = [0; 4];
            for row in &rows {
                mask[usize::from(row[p][0][0] < 0.) + 2 * usize::from(row[p][1][0] < 0.)] += 1;
            }
            if profile["improved_reference_bitmask"] != json!(mask) {
                return Err("wrong two-reference signs".into());
            }
            for g in 0..2 {
                for m in 0..4 {
                    verify_stats(
                        &profile["delta_by_reference"][g]["delta_distance"][m],
                        rows.iter().map(|r| r[p][g][m]).collect(),
                    )?;
                    count += 1;
                }
            }
        }
    }
    Ok(count)
}
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 4 {
        return Err("feature_relaxation FINITE RELAX PRIMARY REPORT".into());
    }
    let dir = Path::new(&args[3]);
    let summary = load(&dir.join("feature-relaxation.json"))?;
    let mut cuts = Vec::new();
    let mut matched_impulses = vec![];
    let mut matched_times = vec![];
    let mut identical_time = 0;
    let lines = fs::read_to_string(dir.join("feature-cuts.jsonl"))?;
    let mut rows = BTreeMap::new();
    for line in lines.lines() {
        let row: Value = serde_json::from_str(line)?;
        let key = (uint(&row["site"]["index"])?, uint(&row["site"]["stroke"])?);
        if rows.insert(key, row).is_some() {
            return Err("duplicate cut row".into());
        }
    }
    for number in list(&summary["finite_indices"])? {
        let i = uint(number)?;
        let name = format!("cases/case-{i:04}.json");
        let finite = load(&Path::new(&args[0]).join(&name))?;
        let rp = Path::new(&args[1]).join(&name);
        let dp = Path::new(&args[2]).join(&name);
        if !rp.exists() || !dp.exists() {
            continue;
        }
        let relax = load(&rp)?;
        let primary = load(&dp)?;
        let offset = num(&finite["result"]["fixture"]["offset_m"])?;
        let held = HELD.contains(&((offset * 1000.).round() as u64));
        for f in list(&finite["result"]["trials"])? {
            let s = uint(&f["stroke"])?;
            let row = rows.get(&(i, s)).ok_or("missing joined row")?;
            let r = one(&relax["result"]["trials"], |t| t["stroke"] == s)?;
            let h = one(&primary["result"]["trials"], |t| t["stroke"] == s)?;
            if f["cut"] != r["cut"] || r["cut"] != h["cut"] {
                return Err("incoming mismatch".into());
            }
            let t0 = uint(&r["cut"]["tick"])?;
            let path = |calls, passes, onset| {
                one(&r["paths"], |p| {
                    p["profile"] == json!({"calls":calls,"relaxation":passes,"onset_only":onset})
                })
            };
            let ordinary = path(8, 1, false)?;
            let eight = path(8, 8, false)?;
            let onset = path(8, 8, true)?;
            let fine = path(4096, 8, false)?;
            let alternate = &one(&h["paths"], |p| {
                p["method"] == "uniform" && p["calls"] == 4096 && p["iterations"] == 4
            })?["record"];
            let fb = one(&f["paths"], |p| p["profile"] == "native8")?;
            let fa = one(&f["paths"], |p| p["profile"] == "finite8")?;
            identical_time +=
                usize::from(ordinary["first"]["end_ns"] == fb["first_loaded_native_end_ns"]);
            let geo = &f["geometry"];
            let raw = geo["status"].as_str().unwrap_or("missing_geometry");
            let (family, law) = if raw == "crossing" {
                let full = geo["feature"].as_str().ok_or("missing axis")?;
                (
                    full.split_once('(').map_or(full, |a| a.0),
                    match geo["sticking"].as_bool() {
                        Some(true) => "sticking",
                        Some(false) => "sliding",
                        None => "no_isolated_law",
                    },
                )
            } else {
                (raw, "no_isolated_law")
            };
            let group = format!("{family}/{law}");
            let hits = list(&fa["impacts"])?;
            let admitted = !hits.is_empty();
            let first = hits.first().filter(|p| {
                p["feature"] == geo["feature"]
                    && p["sticking"] == geo["sticking"]
                    && geo["instantaneous_impulse_ns"].is_array()
            });
            if row["native_admitted"] != admitted
                || row["feature"] != family
                || row["point_mode"] != law
            {
                return Err("feature/mode mismatch".into());
            }
            if let Some(p) = first {
                let de = vec(&p["impulse_ns"])?.distance(vec(&geo["instantaneous_impulse_ns"])?);
                let dt = uint(&p["local_ns"])? as f64 - uint(&geo["hi_ns"])? as f64;
                near(num(&row["matched_point_jump_error_ns"])?, de)?;
                near(num(&row["matched_point_time_difference_ns"])?, dt)?;
                matched_impulses.push(de);
                matched_times.push(dt);
            } else if !row["matched_point_jump_error_ns"].is_null() {
                return Err("spurious point comparison".into());
            }
            let mut drift = [0f64; 6];
            let mut lost = None;
            for step in [1, 2, 10, 26] {
                let a = snapshot(fb, step)?;
                let b = snapshot(ordinary, step)?;
                if !overlaps(a, offset)?
                    || !overlaps(snapshot(fa, step)?, offset)?
                    || !overlaps(snapshot(alternate, step)?, offset)?
                {
                    lost.get_or_insert(step);
                }
                for (d, k) in [
                    (0, "com_m"),
                    (1, "velocity_m_s"),
                    (2, "spin_rad_s"),
                    (5, "cumulative_tangent_ns"),
                ] {
                    drift[d] = drift[d].max(vec(&a[k])?.distance(vec(&b[k])?));
                }
                for k in 0..4 {
                    drift[3] = drift[3].max(
                        (num(&a["orientation_xyzw"][k])? - num(&b["orientation_xyzw"][k])?).abs(),
                    );
                }
                drift[4] = drift[4].max(
                    (num(&a["cumulative_normal_ns"])? - num(&b["cumulative_normal_ns"])?).abs(),
                );
            }
            for (k, &v) in drift.iter().enumerate() {
                near(num(&row["driver_drift"][k])?, v)?;
            }
            let stable =
                drift[0] <= 1e-6 && drift[2] <= 1e-3 && drift[4] <= 1e-4 && drift[5] <= 1e-4;
            if row["stable_controls"] != stable {
                return Err("wrong cross-driver tolerance class".into());
            }
            let mut delta = [None; 2];
            let mut ordinary_and_eight = [[[0.; 2]; 2]; 2];
            for (w, step) in [10, 26].into_iter().enumerate() {
                if lost.is_some_and(|n| n <= step)
                    || [ordinary, eight, onset, fine].iter().any(|p| {
                        p["first_disjoint_tick"]
                            .as_u64()
                            .is_some_and(|n| n <= t0 + step)
                    })
                {
                    continue;
                }
                let mut values = [[[0.; 4]; 2]; 3];
                for (g, reference) in [fine, alternate].into_iter().enumerate() {
                    let ref_state = snapshot(reference, step)?;
                    for (p, (base, active)) in [(ordinary, eight), (ordinary, onset), (fb, fa)]
                        .into_iter()
                        .enumerate()
                    {
                        let a = distances(snapshot(base, step)?, ref_state)?;
                        let b = distances(snapshot(active, step)?, ref_state)?;
                        for k in 0..4 {
                            values[p][g][k] = b[k] - a[k];
                        }
                        if p == 0 {
                            ordinary_and_eight[w][g] = [a[0], b[0]];
                        }
                    }
                }
                delta[w] = Some(values);
            }
            cuts.push(Cut {
                index: i,
                stroke: s,
                group,
                stable,
                admitted,
                matched: first.is_some(),
                held,
                delta,
                ordinary_and_eight,
            });
        }
    }
    if cuts.len() != rows.len() || summary["ordinary_first_positive_time_exact"] != identical_time {
        return Err("mismatched donor/time count".into());
    }
    let held = cuts.iter().filter(|c| c.held).collect::<Vec<_>>();
    let stable = held
        .iter()
        .copied()
        .filter(|c| c.stable)
        .collect::<Vec<_>>();
    let mut measures = population(&summary["held_all"], &held)?
        + population(&summary["held_stable_controls"], &stable)?;
    for group in list(&summary["held_by_native_admission"])? {
        let rows = held
            .iter()
            .copied()
            .filter(|c| group["admitted"] == c.admitted)
            .collect::<Vec<_>>();
        measures += population(&group["held"], &rows)?;
    }
    let mut rule = BTreeMap::new();
    for bin in list(&summary["groups"])? {
        let key = bin["feature_and_mode"].as_str().ok_or("group label")?;
        let (test, train): (Vec<_>, Vec<_>) =
            cuts.iter().filter(|c| c.group == key).partition(|c| c.held);
        measures += population(&bin["held"], &test)? + population(&bin["training"], &train)?;
        let tr = train.iter().filter_map(|c| c.delta[1]).collect::<Vec<_>>();
        let choose =
            !tr.is_empty() && (0..2).all(|g| tr.iter().map(|v| v[0][g][0]).sum::<f64>() < 0.);
        if choose != bin["choose_eight"] {
            return Err("training decision mismatch".into());
        }
        rule.insert(key, choose);
    }
    for (g, reps) in list(&summary["held_rules"])?.iter().enumerate() {
        for (w, report) in list(reps)?.iter().enumerate() {
            let mut errors: [Vec<f64>; 4] = Default::default();
            let mut count = 0;
            for c in &held {
                if c.delta[w].is_some() {
                    let choose = rule[&c.group.as_str()];
                    count += usize::from(choose);
                    let ab = c.ordinary_and_eight[w][g];
                    let p = ab[usize::from(choose)];
                    for (dst, x) in errors.iter_mut().zip([ab[0], ab[1], p, p - ab[0]]) {
                        dst.push(x);
                    }
                }
            }
            if report["selected_eight"] != count {
                return Err("incorrect held-out selected count".into());
            }
            for (k, v) in [
                "one_mm",
                "eight_mm",
                "feature_selected_mm",
                "change_vs_one_mm",
            ]
            .into_iter()
            .zip(errors)
            {
                verify_stats(&report[k], v)?;
                measures += 1;
            }
        }
    }
    let winner = held
        .iter()
        .filter_map(|c| {
            let x = c.delta[1]?;
            (c.matched && x[2][0][0] < 0. && x[2][1][0] < 0. && x[0][0][0] > 0.)
                .then_some((c.index, c.stroke, x[0][0][0]))
        })
        .max_by(|a, b| a.2.total_cmp(&b.2).then((b.0, b.1).cmp(&(a.0, a.1))));
    if let Some((i, s, delta)) = winner {
        let pick = &summary["outlier"];
        if pick["site"]["index"] != i || pick["site"]["stroke"] != s {
            return Err("different maximum".into());
        }
        near(num(&pick["continuous_eight_minus_one_mm"][0])?, delta)?;
    } else if !summary["outlier"].is_null() {
        return Err("missing qualified maximum".into());
    }
    let compare = &summary["geometry_vs_transaction"];
    verify_stats(&compare["instantaneous_vector_error_ns"], matched_impulses)?;
    verify_stats(&compare["signed_time_difference_ns"], matched_times)?;
    fs::write(
        dir.join("feature-readback.json"),
        serde_json::to_vec_pretty(
            &json!({"matched_cuts":cuts.len(),"held":held.len(),"stable_held":stable.len(),"point_geometry_checks":cuts.iter().filter(|s|s.matched).count(),"distribution_vectors_checked":measures+2,"independent_winner":winner,"method":"raw three-way incoming states and four checkpoint support-polygon clips; physical instantaneous jump compared only on matching site and mode"}),
        )?,
    )?;
    Ok(())
}
