// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit exact COM-gravity work, geometric qualification, impulse timing and native cost.
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
type Groups = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn read(p: impl AsRef<Path>) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(x: &V) -> &Vec<V> {
    x.as_array().unwrap()
}
fn n(x: &V) -> u64 {
    x.as_u64().unwrap()
}
fn f(x: &V) -> f64 {
    x.as_f64().unwrap()
}
fn add(g: &mut Groups, k: &str, m: &str, x: f64) {
    assert!(x.is_finite());
    g.entry(k.into())
        .or_default()
        .entry(m.into())
        .or_default()
        .push(x);
}
fn quant(mut x: Vec<f64>) -> V {
    x.sort_by(f64::total_cmp);
    let n = x.len();
    json!({"n":n,"p95":x[(95*n).div_ceil(100)-1],"max":x[n-1],"min":x[0]})
}
fn stats(g: Groups) -> V {
    json!(
        g.into_iter()
            .map(|(k, v)| (
                k,
                v.into_iter()
                    .map(|(m, x)| (m, quant(x)))
                    .collect::<BTreeMap<_, _>>()
            ))
            .collect::<BTreeMap<_, _>>()
    )
}
fn mode(v: &V) -> &str {
    if v["compact"] == true {
        "compact"
    } else if v["ballistic"] == true {
        "ballistic"
    } else {
        "native"
    }
}
fn main() {
    let arg = std::env::args_os().nth(1).unwrap();
    let dir = Path::new(&arg);
    let data = read(dir.join("study.json"));
    let mut ids = BTreeSet::new();
    let mut groups = Groups::new();
    let mut work = BTreeMap::new();
    let mut totals = BTreeMap::new();
    let mut physical = BTreeMap::new();
    for c in a(&data["cases"]) {
        let (id, field) = (n(&c["id"]), n(&c["field"]));
        assert!(ids.insert((id, field)));
        let at = n(&c["arrival_ns"]);
        physical.insert((id, field), at.div_ceil(500000));
        for run in a(&c["trials"]) {
            let k = format!("{:02}-{}", n(&run["budget"]), mode(run));
            let mut calls = 0;
            let mut count = 0;
            let mut first = None;
            let mut early = 0.;
            let mut selected = BTreeSet::new();
            for s in a(&run["spans"]) {
                let tick = n(&s["tick"]);
                let ends = a(&s["end_ns"]);
                assert_eq!(n(ends.last().unwrap()), 500000);
                assert!(ends.windows(2).all(|v| n(&v[0]) < n(&v[1])));
                calls += ends.len() as u64;
                if ends.len() > 1 {
                    selected.insert(tick);
                }
                for (i, (end, load)) in ends.iter().zip(a(&s["load"])).enumerate() {
                    let t = (tick - 1) * 500000 + n(end);
                    if f(load) > 0. {
                        first.get_or_insert(t);
                        if t < at {
                            early += f(load);
                        }
                    }
                    if s["ballistic"].as_array().is_some_and(|v| v[i] == true) {
                        count += 1;
                        assert_eq!(f(load), 0.);
                        assert_eq!(s["guard"][i], true);
                    }
                }
            }
            assert_eq!(calls, n(&run["native_calls"]));
            assert_eq!(first, run["first_ns"].as_u64());
            assert!((early - f(&run["early_ns"])).abs() < 1e-12);
            let fields = totals.entry(k.clone()).or_insert([0u64; 4]);
            fields[0] += 1;
            fields[1] += count;
            fields[2] += u64::from(early > 0.);
            fields[3] += calls;
            let m = &run["metrics"];
            if n(&run["budget"]) == 8 {
                work.insert((id, field, mode(run)), selected);
                if mode(run) != "native" {
                    let check = &run["flight_check"];
                    assert_eq!(n(&check["spans"]), count);
                    assert!(
                        f(&check["min_positive_gap_m"]) > 0.
                            && f(&check["max_center_error_m"]) < 1e-12
                            && f(&check["max_speed_error_m_s"]) < 1e-12
                            && early == 0.
                    );
                    add(
                        &mut groups,
                        &k,
                        "actual_span_spin_error_rad_s",
                        f(&check["max_spin_error_rad_s"]),
                    );
                }
            }
            for group in [
                k.clone(),
                format!("{k}/{}", if id >= 960 { "held_out" } else { "original" }),
            ] {
                for (measure, value) in [
                    ("lag_abs_ns", first.unwrap().abs_diff(at) as f64),
                    ("free_com_um", f(&m["free_com_error_m"]) * 1e6),
                    ("free_energy_j", f(&m["free_energy_drift_j"])),
                    ("com_um", 1e6 * f(&m["com_error_m"])),
                    ("spin_rad_s", f(&m["spin_error_rad_s"])),
                    ("impulse_error_ns", f(&m["impulse_vector_error_ns"])),
                    ("momentum_closure_ns", f(&m["linear_impulse_closure_ns"])),
                    ("dissipation_error_j", f(&m["dissipation_error_j"])),
                ] {
                    add(&mut groups, &group, measure, value);
                }
            }
        }
    }
    assert_eq!(ids.len(), 370);
    let mut perf = BTreeMap::new();
    for name in ["cost-a", "cost-b"] {
        let p = read(dir.join(name).join("cost.json"));
        let mut measured = Groups::new();
        for r in a(&p["rows"]) {
            let k = mode(r);
            let key = (n(&r["id"]), n(&r["field"]));
            let selected = &work[&(key.0, key.1, k)];
            let at = physical[&key];
            for samples in a(&r["steps_ns"]) {
                assert_eq!(a(samples).len(), 8);
                for (i, x) in a(samples).iter().enumerate() {
                    let t = i as u64 + 1;
                    let us = f(x) / 1000.;
                    add(&mut measured, k, "all_us", us);
                    if t == at {
                        add(&mut measured, k, "physical_impact_us", us);
                    }
                    if selected.contains(&t) {
                        add(&mut measured, k, "selected_us", us);
                    }
                }
            }
        }
        perf.insert(name, stats(measured));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"qualified":ids.len(),"declines":data["declines"],"counts_fields":["cases","ballistic_spans","any_early_load","native_calls"],"totals":totals,"accuracy":stats(groups),"cost":perf})).unwrap());
}
