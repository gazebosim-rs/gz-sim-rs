// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recheck matched native impulse times, isolated pulse receipts and post-impact ledgers.
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
fn read(p: impl AsRef<Path>) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn arr(v: &V) -> &Vec<V> {
    v.as_array().unwrap()
}
fn f(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &V) -> u64 {
    v.as_u64().unwrap()
}
type Groups = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn push(m: &mut Groups, group: &str, name: &str, x: f64) {
    assert!(x.is_finite());
    m.entry(group.into())
        .or_default()
        .entry(name.into())
        .or_default()
        .push(x);
}
fn summary(m: Groups) -> V {
    let mut out = BTreeMap::new();
    for (key, vars) in m {
        let mut stats = BTreeMap::new();
        for (field, mut v) in vars {
            v.sort_by(f64::total_cmp);
            let n = v.len();
            stats.insert(field,json!({"n":n,"p50":v[n.div_ceil(2)-1],"p95":v[(95*n).div_ceil(100)-1],"max":v[n-1]}));
        }
        out.insert(key, stats);
    }
    json!(out)
}
fn main() {
    let root = std::env::args().nth(1).unwrap();
    let dir = Path::new(&root);
    let data = read(dir.join("pulse.json"));
    assert_eq!(arr(&data["catalog"]).len(), 320);
    let mut groups = Groups::new();
    let mut ids = BTreeSet::new();
    for case in arr(&data["cases"]) {
        let id = u(&case["id"]);
        assert!(ids.insert(id));
        let held = if id >= 160 { "held" } else { "original" };
        assert_eq!(case["held_out"], id >= 160);
        assert_eq!(arr(&case["trials"]).len(), 18);
        for row in arr(&case["trials"]) {
            let n = u(&row["calls"]);
            let mode = row["mode"].as_str().unwrap();
            let group = format!("{held}-{n:02}-{mode}");
            let audit = &row["ledger"];
            let events = arr(&audit["loads"]);
            assert_eq!(events[0]["end_ns"], row["first_ns"]);
            assert_eq!(events.last().unwrap()["end_ns"], audit["last_load_ns"]);
            assert!(
                events
                    .windows(2)
                    .all(|w| u(&w[0]["end_ns"]) < u(&w[1]["end_ns"]))
            );
            assert!(f(&row["early_ns"]) <= 1e-12);
            assert!(u(&row["first_ns"]) >= u(&case["arrival_ns"]));
            let delivered: f64 = events.iter().map(|p| f(&p["normal_ns"])).sum();
            let vector_error = f(&row["metrics"]["impulse_vector_error_ns"]);
            assert!((delivered - f(&case["j_ns"])).abs() <= vector_error + 1e-9);
            let mut accepted = 0;
            for p in arr(&audit["pulses"]) {
                if let Some(h) = p["applied_ns"].as_u64() {
                    accepted += 1;
                    let latest = u(&p["arrives_by_ns"]);
                    assert_eq!(p["corner"], case["corner"]);
                    assert!(
                        p["decline"].is_null() && f(&p["oracle_local_ns"]) <= latest as f64 + 0.001
                    );
                    assert!(h > latest && h <= 4000);
                    assert_eq!(
                        h,
                        match mode {
                            "measured" => 4 * (latest + 1),
                            s => s.trim_end_matches("ns").parse().unwrap(),
                        }
                    );
                    push(&mut groups, &group, "accepted_span_ns", h as f64);
                } else {
                    assert!(p["decline"].is_string());
                }
            }
            push(
                &mut groups,
                &group,
                "pulses_per_trajectory",
                accepted as f64,
            );
            push(
                &mut groups,
                &group,
                "native_impact_end_late_ns",
                f(&row["first_ns"]) - f(&case["arrival_ns"]),
            );
            for (key, k, scale) in [
                ("impulse_vector_error_ns", "reaction_error_ns", 1.),
                ("com_error_m", "com_error_um", 1e6),
                ("spin_error_rad_s", "final_spin_error_rad_s", 1.),
            ] {
                push(&mut groups, &group, k, f(&row["metrics"][key]) * scale);
            }
            push(
                &mut groups,
                &group,
                "first_outgoing_spin_error_rad_s",
                f(&events[0]["spin_error_rad_s"]),
            );
            push(
                &mut groups,
                &group,
                "first_outgoing_L_error_nms",
                f(&events[0]["angular_momentum_error_nms"]),
            );
            for key in [
                "free_propagation_spin_residual_rad_s",
                "oracle_from_outgoing_spin_error_rad_s",
            ] {
                push(&mut groups, &group, key, f(&audit[key]));
            }
        }
    }
    assert_eq!(ids.len(), 209);
    let adverse = read(dir.join("adverse.json"));
    let mut negatives = BTreeMap::new();
    for r in arr(&adverse["rows"]) {
        if r["mode"] != "measured" {
            continue;
        }
        let family = r["family"].as_str().unwrap();
        if family != "isolated" {
            assert_eq!(u(&r["applied"]), 0);
        }
        let g = negatives.entry(family).or_insert([0; 4]);
        g[0] += 1;
        g[1] += u(&r["witnesses"]);
        g[2] += u(&r["applied"]);
        g[3] += arr(&r["declines"]).len() as u64;
    }
    let mut costs = BTreeMap::new();
    for label in ["cost-a", "cost-b"] {
        let raw = read(dir.join(label).join("cost.json"));
        let mut cost = Groups::new();
        let mut samples = 0;
        for r in arr(&raw["rows"]) {
            let mode = r["mode"].as_str().unwrap();
            for seq in arr(&r["tick_ns"]) {
                assert_eq!(arr(seq).len(), 8);
                for (i, t) in arr(seq).iter().enumerate() {
                    push(&mut cost, mode, "all_bevy_us", f(t) / 1000.);
                    if (i + 1) as u64 == u(&r["primary_tick"]) {
                        push(&mut cost, mode, "primary_bevy_us", f(t) / 1000.);
                    }
                    samples += 1;
                }
            }
        }
        assert_eq!(samples, 101 * 8 * 6 * 6);
        costs.insert(label, summary(cost));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"measured":summary(groups),"adverse_counts":[
        "intervals","first_witnesses","applied_pulses","declined_pulses"],"adverse":negatives,"cost":costs})).unwrap());
}
