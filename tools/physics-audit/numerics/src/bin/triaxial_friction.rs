// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent momentum, Coulomb, dissipated work, provenance and latency readback.
use glam::{DMat3, DQuat, DVec3};
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
type G = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn read(p: impl AsRef<Path>) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(x: &V) -> &Vec<V> {
    x.as_array().unwrap()
}
fn f(x: &V) -> f64 {
    x.as_f64().unwrap()
}
fn n(x: &V) -> u64 {
    x.as_u64().unwrap()
}
fn s(x: &V) -> &str {
    x.as_str().unwrap()
}
fn arr<const N: usize>(v: &V) -> [f64; N] {
    std::array::from_fn(|i| f(&v[i]))
}
fn v(x: &V) -> DVec3 {
    arr::<3>(x).into()
}
fn add(g: &mut G, k: &str, m: &str, x: f64) {
    assert!(x.is_finite());
    g.entry(k.into())
        .or_default()
        .entry(m.into())
        .or_default()
        .push(x);
}
fn inc(count: &mut BTreeMap<String, BTreeMap<String, u64>>, k: &str, m: &str) {
    *count
        .entry(k.to_owned())
        .or_default()
        .entry(m.into())
        .or_default() += 1;
}
fn quant(mut x: Vec<f64>) -> V {
    x.sort_by(f64::total_cmp);
    let n = x.len();
    json!({"n":n,"p50":x[n.div_ceil(2)-1],"p95":x[(95*n).div_ceil(100)-1],"max":x[n-1]})
}
fn stats(g: G) -> V {
    json!(
        g.into_iter()
            .map(|(k, v)| (
                k,
                v.into_iter()
                    .map(|(k, s)| (k, quant(s)))
                    .collect::<BTreeMap<_, _>>()
            ))
            .collect::<BTreeMap<_, _>>()
    )
}
fn native(p: &V) -> String {
    format!(
        "{}-pgs{}-relax{}",
        if p["point"] == true {
            "point"
        } else {
            "grouped"
        },
        n(&p["pgs"]),
        n(&p["relax"])
    )
}
fn production(p: &V, b: u64) -> String {
    format!(
        "{b:02}-{}-{}",
        if p["follow"] == true {
            "departure"
        } else if p["project"] == true {
            "project"
        } else {
            "native"
        },
        if p["compact"] == true {
            "compact"
        } else {
            "full"
        }
    )
}
fn check(r: &V) {
    let q = DQuat::from_array(arr(&r["pad_q"]));
    let k = DMat3::from_cols_array(&arr(&r["delassus_pad_columns"]));
    let j = v(&r["target_impulse"]);
    let n = v(&r["normal"]);
    let before = v(&r["before"]);
    let after = v(&r["after"]);
    let mu = f(&r["mu"]);
    assert!(before.dot(n) < 0. && after.dot(n).abs() < 1e-12);
    assert!((q * (k * (q.conjugate() * j)) + before).distance(after) < 1e-11);
    let d = -0.5 * j.dot(before + after);
    assert!(d > 0. && (d - f(&r["dissipation_j"])).abs() < 1e-12);
    let normal = n.dot(j);
    let tj = j - normal * n;
    if s(&r["mode"]).ends_with("stick") {
        assert!(after.length() < 1e-12 && tj.length() < normal * mu);
    } else {
        assert!(
            after.length() > 0.
                && (tj.length() - normal * mu).abs() < 1e-10
                && tj.normalize().dot(after.normalize()) < -1. + 1e-10
        );
    }
    let z = k.z_axis.z;
    let xx = k.x_axis.x - k.z_axis.x.powi(2) / z;
    let yy = k.y_axis.y - k.z_axis.y.powi(2) / z;
    let xy = k.x_axis.y - k.z_axis.x * k.z_axis.y / z;
    let ds = (xx - yy).hypot(2. * xy);
    let rho = (xx + yy + ds) / (xx + yy - ds);
    let margin = z * 2. * rho.sqrt() / (1. + rho) - mu * k.z_axis.truncate().length();
    assert!(margin > 0. && (f(&r["monotonic_margin"]) - margin).abs() < 1e-10);
}
fn main() {
    let args = std::env::args_os().collect::<Vec<_>>();
    let dir = Path::new(&args[1]);
    let departure = args.get(2).is_some_and(|x| x == "departure");
    let projected = departure || args.get(2).is_some_and(|x| x == "projection");
    let data = read(dir.join("study.json"));
    let mut stats_by = G::new();
    let mut totals = BTreeMap::new();
    let mut donors = BTreeSet::new();
    let mut land = BTreeMap::new();
    for c in a(&data["cases"]) {
        let r = &c["reference"];
        check(r);
        let id = n(&r["id"]);
        let mode = s(&r["mode"]);
        let at = n(&r["arrival_ns"]);
        let true_j = v(&r["target_impulse"]);
        let normal = v(&r["normal"]);
        let corner = n(&r["corner"]) as usize;
        assert!(donors.insert((id, mode)));
        land.insert((id, mode.to_owned()), at.div_ceil(500000));
        let mut comparison = BTreeMap::new();
        for p in a(&c["trials"]) {
            let row = if projected { &p["run"] } else { p };
            let k = if projected {
                production(p, n(&p["budget"]))
            } else {
                native(&p["profile"])
            };
            let mut sum = DVec3::ZERO;
            let mut others = 0.;
            let mut first = None;
            let mut first_endpoint = None;
            let mut hits = 0u64;
            let mut accepted = None;
            for l in a(&row["loads"]) {
                let j = v(&l["momentum_impulse"]);
                sum += j;
                let end = n(&l["end_ns"]);
                let projection = &l["projection"];
                let physical = if projection.is_null() {
                    end
                } else {
                    hits += 1;
                    let t = n(&projection["application_ns"]);
                    assert!(t <= end && end - t <= n(&projection["pulse_duration_ns"]));
                    assert!(
                        n(&projection["corner"]) == corner as u64
                            && f(&projection["outward_acceleration"]) > 0.
                            && f(&projection["gap_m"]).abs() < 1e-11
                    );
                    assert!(v(&projection["impulse"]).distance(j) < 1e-10);
                    assert!(v(&projection["after"]).dot(normal).abs() < 1e-11);
                    accepted = Some(projection);
                    t
                };
                first.get_or_insert(physical);
                first_endpoint.get_or_insert(end);
                others += a(&l["corners"])
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != corner)
                    .map(|(_, x)| f(x))
                    .sum::<f64>();
                if !l["native_tangent"].is_null() {
                    assert!((v(&l["native_tangent"]) + v(&l["native_normal"])).distance(j) < 1e-10);
                }
            }
            if departure {
                let spans = a(&row["departures"]);
                assert!(spans.is_empty() || p["follow"] == true && hits == 1 && spans.len() == 1);
                for span in spans {
                    let start = n(&span["start_ns"]);
                    let end = start + n(&span["duration_ns"]);
                    let loaded = a(&row["loads"]);
                    assert!(
                        loaded
                            .iter()
                            .any(|l| !l["projection"].is_null() && n(&l["end_ns"]) == start)
                    );
                    assert!(
                        !loaded
                            .iter()
                            .any(|l| (start + 1..=end).contains(&n(&l["end_ns"])))
                    );
                    assert!(start / 500_000 == (end - 1) / 500_000 && end <= 4_000_000);
                    inc(&mut totals, &k, "certified_departure");
                }
                if a(&row["loads"]).len() > 1 {
                    inc(&mut totals, &k, "additional_load");
                }
            }
            let impulse = v(&row["world_impulse"]);
            assert!(sum.distance(impulse) < 1e-10);
            let delta = impulse - true_j;
            let jerr = delta.length();
            let jt = (delta - normal * normal.dot(delta)).length();
            assert!(
                (jerr - f(&row["vector_error_ns"])).abs() < 1e-12
                    && (jt - f(&row["tangent_error_ns"])).abs() < 1e-12
            );
            assert!((others - f(&row["wrong_corner_ns"])).abs() < 1e-11);
            if !row["initial_energy_j"].is_null() {
                let diss = f(&row["initial_energy_j"]) - f(&row["final_energy_j"])
                    + f(&row["pad_work_j"])
                    + f(&row["gravity_work_j"]);
                assert!((diss - f(&row["loss_j"])).abs() < 1e-11);
            }
            let mut groups = vec![k.clone(), format!("{k}/{mode}")];
            if projected {
                let done = hits > 0;
                if let Some(hit) = accepted {
                    inc(
                        &mut totals,
                        &k,
                        if hit["sticking"] == true {
                            "sticking"
                        } else {
                            "sliding"
                        },
                    );
                }
                if id >= 1344 {
                    groups.push(format!("{k}/held_out"));
                }
                if p["project"] == true {
                    groups.push(format!("{k}/{}", if done { "used" } else { "fallback" }));
                    inc(
                        &mut totals,
                        &k,
                        if done { "projected" } else { "native_fallback" },
                    );
                    for a in a(&row["attempts"]) {
                        if let Some(why) = a["decline"].as_str() {
                            inc(&mut totals, &k, why);
                        }
                    }
                }
            } else {
                let p = &row["profile"];
                let key = (n(&p["pgs"]), n(&p["relax"]));
                let value = [
                    row["calls"].clone(),
                    row["world_impulse"].clone(),
                    row["final_com_m"].clone(),
                    row["final_spin_rad_s"].clone(),
                ];
                if let Some(old) = comparison.insert(key, value.clone()) {
                    assert_eq!(old, value);
                }
            }
            for k in groups {
                for (m, x) in [
                    ("vector_ns", jerr),
                    ("tangent_ns", jt),
                    ("normal_ns", normal.dot(delta).abs()),
                    (
                        "loss_error_j",
                        (f(&row["loss_j"]) - f(&r["dissipation_j"])).abs(),
                    ),
                    ("wrong_corner_ns", others),
                    ("lag_ns", first.unwrap().abs_diff(at) as f64),
                    (
                        "first_native_endpoint_ns",
                        first_endpoint.unwrap().abs_diff(at) as f64,
                    ),
                    ("com_um", 1e6 * f(&row["final_com_m"])),
                    ("spin_rad_s", f(&row["final_spin_rad_s"])),
                    ("native_calls", f(&row["calls"])),
                ] {
                    add(&mut stats_by, &k, m, x);
                }
            }
        }
    }
    if !projected {
        assert_eq!(donors.len(), 483);
        let u = &data["uncertain"];
        assert!(
            f(&u["last_certified_post_s"]) < f(&u["first_unresolved_post_s"])
                && f(&u["first_unresolved_post_s"]) <= 0.004
        );
    }
    if projected && !departure {
        let rejected = read(dir.join("refusals.json"));
        let rows = a(&rejected["withdrawn"]);
        assert_eq!(rows.len(), 96);
        let mut controls = BTreeSet::new();
        for r in rows {
            assert!(
                f(&r["outward_acceleration"]) < -0.5
                    && r["same_state"] == true
                    && r["same_interval"] == true
                    && r["declined_departure"] == true
            );
            assert!(a(&r["tried"]).iter().any(|t| t["decline"] == "Departure"));
            assert!(controls.insert((n(&r["id"]), s(&r["mode"]))));
        }
        inc(&mut totals, "refusals", "complete");
    }
    if departure {
        let returns = read(dir.join("short_returns.json"));
        let rows = a(&returns["cases"]);
        assert_eq!(rows.len(), 160);
        for c in rows {
            let (u, w, g) = (
                f(&c["sin_arm_m"]),
                f(&c["cos_arm_m"]),
                f(&c["normal_acceleration"]),
            );
            let omega = f(&c["speed_rad_s"]);
            let t = f(&c["return_after_s"]);
            let phase = omega * 0.0005;
            let jerk = omega.powi(3) * (u * phase.cos() - w.abs() * phase.sin());
            assert!(jerk > 0. && (jerk - f(&c["negative_jerk_min"])).abs() < 1e-11);
            assert!(g + w * omega * omega > 0.);
            // The independent sine series on this short arc is resolved at small angle.
            let gap = |dt: f64| {
                let x = omega * dt;
                let rem =
                    x.powi(3) * (-1. / 6. + x * x / 120. - x.powi(4) / 5040. + x.powi(6) / 362880.);
                u * rem + w * 2. * (0.5 * x).sin().powi(2) + 0.5 * g * dt * dt
            };
            assert!(gap(t / 2.) > 0. && (gap(t / 2.) - f(&c["mid_gap_m"])).abs() < 1e-18);
            assert!(gap(t).abs() < 1e-16 && gap(t * 1.1) < 0. && f(&c["return_rate_m_s"]) < 0.);
            let first = n(&c["first_ns"]);
            let second = n(&c["second_ns"]);
            assert_eq!(first + (t * 1e9).round() as u64, second);
            for trial in a(&c["trials"]) {
                let group = format!(
                    "short_return/{}/{}",
                    if trial["compact"] == true {
                        "compact"
                    } else {
                        "full"
                    },
                    if trial["follow"] == true {
                        "departure"
                    } else {
                        "project"
                    }
                );
                let hits = a(&trial["applications"]);
                assert_eq!(hits.len(), 1);
                assert_eq!(n(&hits[0]["application_ns"]), first);
                for span in a(&trial["certified_ns"]) {
                    assert!(n(&span[0]) >= first && n(&span[1]) < second);
                    inc(&mut totals, &group, "certified_before_return");
                }
                let observed = a(&trial["native"]);
                if observed.iter().any(|p| n(&p["end_ns"]) < second) {
                    inc(&mut totals, &group, "native_before_return");
                }
                if observed.iter().any(|p| n(&p["end_ns"]) >= second) {
                    inc(&mut totals, &group, "native_at_or_after_return");
                }
            }
        }
    }
    let mut timings = BTreeMap::new();
    for name_run in ["cost-a", "cost-b"] {
        let raw = read(dir.join(name_run).join("cost.json"));
        let mut g = G::new();
        for r in a(&raw["rows"]) {
            let k = if projected {
                production(r, 8)
            } else {
                native(&r["profile"])
            };
            let tick = land[&(n(&r["id"]), s(&r["mode"]).to_owned())];
            for samples in a(&r["steps_ns"]) {
                assert_eq!(a(samples).len(), 8);
                for (j, t) in a(samples).iter().enumerate() {
                    add(&mut g, &k, "all_us", f(t) / 1000.);
                    if j as u64 + 1 == tick {
                        add(&mut g, &k, "landing_us", f(t) / 1000.);
                    }
                }
            }
        }
        timings.insert(name_run, stats(g));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"cases":donors.len(),"decisions":totals,"accuracy":stats(stats_by),"cost":timings})).unwrap());
}
