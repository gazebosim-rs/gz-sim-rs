// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently check the recorded native clocks, impact errors and timing samples.
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
fn s(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &V) -> u64 {
    v.as_u64().unwrap()
}
fn quantile(mut v: Vec<f64>) -> V {
    v.sort_by(f64::total_cmp);
    if v.is_empty() {
        return V::Null;
    }
    let n = v.len();
    json!({"n":n,"min":v[0],"p50":v[n.div_ceil(2)-1],"p95":v[(95*n).div_ceil(100)-1],"max":v[n-1]})
}
type Groups = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn push(m: &mut Groups, g: &str, k: &str, x: f64) {
    assert!(x.is_finite());
    m.entry(g.to_owned())
        .or_default()
        .entry(k.to_owned())
        .or_default()
        .push(x);
}
fn summary(m: Groups) -> V {
    json!(
        m.into_iter()
            .map(|(k, a)| (
                k,
                a.into_iter()
                    .map(|(s, v)| (s, quantile(v)))
                    .collect::<BTreeMap<_, _>>()
            ))
            .collect::<BTreeMap<_, _>>()
    )
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let dir = Path::new(&path);
    let data = read(dir.join("cubic.json"));
    let mut groups = Groups::new();
    let mut ids = BTreeSet::new();
    for r in arr(&data["cases"]) {
        assert!(ids.insert(u(&r["id"])));
        let reference = s(&r["arrival_ns"]);
        assert_eq!(arr(&r["trials"]).len(), 8);
        for run in arr(&r["trials"]) {
            let n = u(&run["profile"]["calls"]);
            let cubic = run["cubic"].as_bool().unwrap();
            let group = format!("{n:02}-{}", if cubic { "cubic" } else { "frozen" });
            let mut first = None;
            let mut calls = 0;
            let mut summed = 0.;
            let mut protected = 0;
            let mut events = 0;
            for t in arr(&run["ticks"]) {
                let start = (u(&t["tick"]) - 1) * 500_000;
                let ends = arr(&t["native_end_ns"]);
                let loads = arr(&t["native_normal_ns"]);
                let guards = t["guarded"].as_array();
                assert_eq!(ends.len(), u(&t["calls"]) as usize);
                assert_eq!(u(ends.last().unwrap()), 500_000);
                assert_eq!(loads.len(), ends.len());
                assert!(ends.windows(2).all(|p| u(&p[0]) < u(&p[1])));
                calls += ends.len() as u64;
                for (ix, (end, j)) in ends.iter().zip(loads).enumerate() {
                    if guards.is_some_and(|g| g[ix] == true) {
                        protected += 1;
                        assert_eq!(s(j), 0.);
                    }
                    let ns = start + u(end);
                    if s(j) > 1e-12 {
                        first.get_or_insert(ns);
                    }
                    if (ns as f64) < reference {
                        summed += s(j);
                    }
                }
            }
            assert_eq!(first, Some(u(&run["first_positive_end_ns"])));
            assert_eq!(protected, u(&run["guarded_calls"]));
            assert_eq!(calls, u(&run["native_calls"]));
            assert!(summed < 1e-12);
            if cubic {
                for tick in arr(&run["clocks"]) {
                    for w in arr(&tick["witnesses"]) {
                        let a = arr(&w["bracket_ns"]);
                        assert_eq!(a.len(), 2);
                        assert!(
                            u(&w["cut_ns"]) <= u(&a[0])
                                && u(&a[0]) <= u(&a[1])
                                && u(&a[1]) <= 500_000
                        );
                        let oracle = s(&w["rk4_ns"]);
                        assert!(s(&a[0]) - 0.001 <= oracle && oracle <= s(&a[1]) + 0.001);
                        assert_eq!(u(&w["corner"]), u(&r["reference_corner"]));
                        push(&mut groups, &group, "bracket_width_ns", s(&a[1]) - s(&a[0]));
                        events += 1;
                    }
                }
            }
            push(&mut groups, &group, "witnesses", events as f64);
            push(
                &mut groups,
                &group,
                "solve_endpoint_late_us",
                (first.unwrap() as f64 - reference) / 1000.,
            );
            push(&mut groups, &group, "total_native_calls", calls as f64);
            for (key, label, scale) in [
                ("impulse_vector_error_ns", "reaction_norm_error_ns", 1.),
                ("com_error_m", "com_error_um", 1e6),
                ("spin_error_rad_s", "spin_error_rad_s", 1.),
                ("linear_impulse_closure_ns", "closure_ns", 1.),
            ] {
                push(&mut groups, &group, label, s(&run["metrics"][key]) * scale);
            }
        }
    }
    assert_eq!(ids.len(), 108);
    let adverse = read(dir.join("adverse.json"));
    let mut adverse_data = Groups::new();
    let mut decisions = BTreeMap::<String, BTreeMap<String, u32>>::new();
    for row in arr(&adverse["rows"]) {
        if row["cubic"] != true {
            continue;
        }
        let family = row["family"].as_str().unwrap();
        let t = &row["clock"];
        let reason = t["decline"].as_str().unwrap_or("accepted");
        assert_eq!(arr(&t["durations_ns"]).len(), u(&row["calls"]) as usize);
        assert_eq!(arr(&t["durations_ns"]).iter().map(u).sum::<u64>(), 500_000);
        if ["face", "edge", "subroundoff", "grazing"].contains(&family) {
            assert_eq!(u(&t["witnesses"]), 0);
        }
        if matches!(family, "isolated" | "near_edge") {
            let w = &t["first"];
            let crossing = s(&t["reference_arrival_ns"]);
            assert!(s(&w["lo"]) - 0.001 <= crossing && crossing <= s(&w["hi"]) + 0.001);
        }
        *decisions
            .entry(family.into())
            .or_default()
            .entry(reason.to_string())
            .or_default() += 1;
        push(&mut adverse_data, family, "witnesses", s(&t["witnesses"]));
        if let Some(n) = t["decline_after"].as_u64() {
            push(
                &mut adverse_data,
                family,
                "native_calls_at_decline",
                n as f64,
            );
        }
    }
    let mut costs = BTreeMap::new();
    for round in ["cost-a", "cost-b"] {
        let data = read(dir.join(round).join("cost.json"));
        let mut groups = Groups::new();
        let mut samples = 0usize;
        for r in arr(&data["rows"]) {
            let group = format!(
                "{:02}-{}",
                u(&r["calls"]),
                if r["cubic"] == true {
                    "cubic"
                } else {
                    "frozen"
                }
            );
            for run in arr(&r["tick_ns"]) {
                assert_eq!(arr(run).len(), 8);
                for (i, ns) in arr(run).iter().enumerate() {
                    push(&mut groups, &group, "all_steps_us", s(ns) / 1000.);
                    if i + 1 == u(&r["primary_tick"]) as usize {
                        push(&mut groups, &group, "primary_step_us", s(ns) / 1000.);
                    }
                    samples += 1;
                }
            }
        }
        assert_eq!(samples, 55_296);
        let adversity = read(dir.join(round).join("adverse_cost.json"));
        let mut ab = Groups::new();
        let mut native = 0;
        for r in arr(&adversity["rows"]) {
            let group = format!("{}-{:02}", r["family"].as_str().unwrap(), u(&r["calls"]));
            let field = if r["cubic"] == true {
                "cubic_us"
            } else {
                "frozen_us"
            };
            for t in arr(&r["elapsed_ns"]) {
                push(&mut ab, &group, field, s(t) / 1000.);
                native += 1;
            }
        }
        assert_eq!(native, 4_608);
        costs.insert(round,json!({"bevy_steps":samples,"steps":summary(groups),"adverse_intervals":native,"adverse":summary(ab)}));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"physical":summary(groups),"adverse":summary(adverse_data),"adverse_decisions":decisions,"cost":costs})).unwrap());
}
