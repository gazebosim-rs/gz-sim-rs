// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify pair-scoped post-impact rotation receipts, independent bounds and matched timings.
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
fn read(p: impl AsRef<Path>) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(v: &V) -> &Vec<V> {
    v.as_array().unwrap()
}
fn f(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &V) -> u64 {
    v.as_u64().unwrap()
}
type Groups = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn put(g: &mut Groups, key: &str, name: &str, x: f64) {
    assert!(x.is_finite());
    g.entry(key.into())
        .or_default()
        .entry(name.into())
        .or_default()
        .push(x);
}
fn report(g: Groups) -> V {
    let mut all = BTreeMap::new();
    for (k, row) in g {
        let mut stats = BTreeMap::new();
        for (key, mut d) in row {
            d.sort_by(f64::total_cmp);
            let n = d.len();
            stats.insert(key,json!({"n":n,"min":d[0],"p50":d[n.div_ceil(2)-1],"p95":d[(95*n).div_ceil(100)-1],"max":d[n-1]}));
        }
        all.insert(k, stats);
    }
    json!(all)
}
fn main() {
    let data = std::env::args().nth(1).unwrap();
    let dir = Path::new(&data);
    let raw = read(dir.join("released.json"));
    let mut ids = BTreeSet::new();
    let mut measured = Groups::new();
    let mut counts = BTreeMap::new();
    for c in a(&raw["cases"]) {
        let id = u(&c["id"]);
        assert!(ids.insert(id));
        assert_eq!(a(&c["trials"]).len(), 6);
        for pair in a(&c["trials"]).chunks_exact(2) {
            let (prev, next) = (&pair[0], &pair[1]);
            assert_eq!(prev["release"], false);
            assert_eq!(next["release"], true);
            assert_eq!(prev["calls"], next["calls"]);
            assert_eq!(prev["first_ns"], next["first_ns"]);
            assert_eq!(prev["ledger"]["loads"], next["ledger"]["loads"]);
            assert_eq!(prev["early_ns"], next["early_ns"]);
            assert_eq!(f(&prev["early_ns"]), 0.);
            for r in pair {
                let k = format!(
                    "{}-{:02}-{}",
                    if id >= 160 { "held" } else { "original" },
                    u(&r["calls"]),
                    if r["release"] == true {
                        "released"
                    } else {
                        "baseline"
                    }
                );
                let n = counts.entry(k.clone()).or_insert([0u32; 4]);
                n[0] += 1;
                let mut applied = 0;
                for step in a(&r["departure"]) {
                    n[1] += 1;
                    if step["integrated"] != true {
                        assert!(step["min_gap_pm"].is_null());
                        continue;
                    }
                    n[2] += 1;
                    applied += 1;
                    let check = &step["independent"];
                    assert!(f(&check["quadratic_lower_m"]) > 0.);
                    assert!(f(&check["sample_min_m"]) + 2e-12 >= f(&step["min_gap_pm"]) * 1e-12);
                    let h = u(&step["end_ns"]) - u(&step["begin_ns"]);
                    assert!(h > 0 && h <= 500_000);
                    assert!((4..=64).contains(&u(&step["corner_spans"])));
                    assert!((2..=32).contains(&u(&step["native_subdivisions"])));
                    for key in [
                        "quadratic_lower_m",
                        "end_spin_error_rad_s",
                        "end_attitude_error_rad",
                        "angular_momentum_drift_nms",
                    ] {
                        put(&mut measured, &k, key, f(&check[key]));
                    }
                }
                n[3] += u32::from(applied > 0);
                for (key, scale) in [
                    ("spin_error_rad_s", 1.),
                    ("angular_momentum_error_nms", 1.),
                    ("com_error_m", 1e6),
                    ("impulse_vector_error_ns", 1.),
                ] {
                    put(
                        &mut measured,
                        &k,
                        if key == "com_error_m" {
                            "com_error_um"
                        } else {
                            key
                        },
                        f(&r["metrics"][key]) * scale,
                    );
                }
            }
        }
    }
    assert_eq!(ids.len(), 209);
    let mut costs = BTreeMap::new();
    for round in ["cost-a", "cost-b"] {
        let data = read(dir.join(round).join("cost.json"));
        let mut g = Groups::new();
        let mut samples = 0usize;
        for r in a(&data["rows"]) {
            let key = if r["release"] == true {
                "released"
            } else {
                "baseline"
            };
            let impact = u(&r["impact_tick"]);
            for run in a(&r["tick_ns"]) {
                assert_eq!(a(run).len(), 8);
                for (i, t) in a(run).iter().enumerate() {
                    let tick = (i + 1) as u64;
                    let bucket = if tick > impact {
                        "postimpact_us"
                    } else if tick == impact {
                        "impact_us"
                    } else {
                        "approach_us"
                    };
                    put(&mut g, key, bucket, f(t) / 1000.);
                    put(&mut g, key, "all_steps_us", f(t) / 1000.);
                    samples += 1;
                }
            }
        }
        assert_eq!(samples, 25_856);
        costs.insert(round, report(g));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"measurements":report(measured),"count_fields":["trajectories","attempts","accepted","trajectories_with_acceptance"],"counts":counts,"cost":costs})).unwrap());
}
