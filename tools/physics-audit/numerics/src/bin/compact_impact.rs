// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently reconstruct reaction bins and assign measured public costs to causal windows.
use serde_json::{Value as V, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    path::Path,
};
type Groups = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn read(path: impl AsRef<Path>) -> V {
    serde_json::from_reader(File::open(path).unwrap()).unwrap()
}
fn a(v: &V) -> &Vec<V> {
    v.as_array().unwrap()
}
fn n(v: &V) -> u64 {
    v.as_u64().unwrap()
}
fn f(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn name(row: &V) -> String {
    format!(
        "{:02}-{}",
        n(&row["budget"]),
        if row["compact"] == true {
            "compact"
        } else {
            "full"
        }
    )
}
fn add(g: &mut Groups, k: &str, m: &str, x: f64) {
    assert!(x.is_finite());
    g.entry(k.into())
        .or_default()
        .entry(m.into())
        .or_default()
        .push(x);
}
fn quant(mut d: Vec<f64>) -> V {
    d.sort_by(f64::total_cmp);
    let n = d.len();
    json!({"n":n,"mean":d.iter().sum::<f64>()/n as f64,"p50":d[n.div_ceil(2)-1],"p95":d[(n*95).div_ceil(100)-1],"p99":d[(n*99).div_ceil(100)-1],"max":d[n-1]})
}
fn stats(g: Groups) -> V {
    json!(
        g.into_iter()
            .map(|(k, v)| (
                k,
                v.into_iter()
                    .map(|(m, v)| (m, quant(v)))
                    .collect::<BTreeMap<_, _>>()
            ))
            .collect::<BTreeMap<_, _>>()
    )
}
fn bump(count: &mut BTreeMap<String, BTreeMap<String, u64>>, k: &str, s: &str) {
    *count
        .entry(k.into())
        .or_default()
        .entry(s.into())
        .or_default() += 1;
}
fn main() {
    let arg = std::env::args_os().nth(1).unwrap();
    let dir = Path::new(&arg);
    let single = read(dir.join("single.json"));
    assert_eq!(a(&single["cases"]).len(), 421);
    let mut solitary = Groups::new();
    let mut status = BTreeMap::new();
    for c in a(&single["cases"]) {
        let mut keys = BTreeSet::new();
        for r in a(&c["trials"]) {
            let k = name(r);
            assert!(keys.insert(k.clone()));
            let budget = n(&r["budget"]);
            let mut calls = 8;
            for e in a(&r["nominated"]) {
                let work = n(&e["calls"]);
                assert!((3..=budget).contains(&work));
                let ends = a(&e["end_ns"]);
                let loads = a(&e["loads"]);
                assert_eq!(ends.len(), work as usize);
                assert_eq!(loads.len(), ends.len());
                assert_eq!(n(ends.last().unwrap()), 500000);
                assert!(ends.windows(2).all(|w| n(&w[0]) < n(&w[1])));
                calls += work - 1;
                if r["compact"] == true {
                    let mark = &e["compact"];
                    if mark["complete"] == true {
                        assert_eq!(mark["pulse_loaded"], true);
                        assert!(mark["decline"].is_null());
                        assert!(
                            work < budget
                                && f(&loads[0]) == 0.
                                && f(&loads[1]) > 0.
                                && f(loads.last().unwrap()) == 0.
                        );
                        bump(&mut status, &k, &format!("completed_in_{work}"));
                    } else {
                        assert_eq!(work, budget);
                        bump(
                            &mut status,
                            &k,
                            mark["decline"].as_str().unwrap_or("native_decline"),
                        );
                    }
                } else {
                    assert_eq!(work, budget);
                }
            }
            assert_eq!(calls, n(&r["native_calls"]));
            for cohort in [
                "all",
                if n(&c["id"]) >= 640 {
                    "held_out"
                } else {
                    "original"
                },
            ] {
                let group = format!("{cohort}/{k}");
                add(
                    &mut solitary,
                    &group,
                    "lag_abs_us",
                    (f(&r["first_ns"]) - f(&c["arrival_ns"])).abs() / 1000.,
                );
                add(&mut solitary, &group, "preimpact_j_ns", f(&r["early_ns"]));
                add(
                    &mut solitary,
                    &group,
                    "impulse_error_ns",
                    f(&r["metrics"]["impulse_vector_error_ns"]),
                );
                add(
                    &mut solitary,
                    &group,
                    "com_um",
                    f(&r["metrics"]["com_error_m"]) * 1e6,
                );
                add(
                    &mut solitary,
                    &group,
                    "spin_rad_s",
                    f(&r["metrics"]["spin_error_rad_s"]),
                );
                add(&mut solitary, &group, "native_calls", calls as f64);
            }
        }
        assert_eq!(keys.len(), 6);
    }
    let data = read(dir.join("returns.json"));
    assert_eq!(a(&data["cases"]).len(), 274);
    let mut runs = BTreeMap::<(u64, String), (u64, BTreeSet<u64>, BTreeSet<u64>)>::new();
    let mut repeated = Groups::new();
    let mut counts = BTreeMap::new();
    for c in a(&data["cases"]) {
        let id = n(&c["id"]);
        let nt = n(&c["ticks"]);
        let events = a(&c["events"])
            .iter()
            .filter(|e| f(&e["time_s"]) <= nt as f64 * 5e-4)
            .collect::<Vec<_>>();
        let hits = events
            .iter()
            .map(|e| (f(&e["time_s"]) * 2000.).ceil() as u64)
            .collect::<BTreeSet<_>>();
        assert!(events.len() >= 2);
        for row in a(&c["trials"]) {
            let k = name(row);
            let r = &row["trial"];
            let mut selected = BTreeSet::new();
            let mut native = nt;
            for state in a(&r["nominated"]) {
                assert!(selected.insert(n(&state["tick"])));
                let calls = n(&state["calls"]);
                native += calls - 1;
                let mark = &state["compact"];
                if mark["complete"] == true {
                    assert!(calls < n(&row["budget"]));
                    assert!(f(&state["tail_control"]["quadratic_lower_m"]) > 0.);
                    add(
                        &mut repeated,
                        &k,
                        "tail_spin_check_rad_s",
                        f(&state["tail_control"]["end_spin_error_rad_s"]),
                    );
                    bump(&mut counts, &k, &format!("completed_in_{calls}"));
                } else {
                    assert_eq!(calls, n(&row["budget"]));
                    if row["compact"] == true {
                        bump(
                            &mut counts,
                            &k,
                            mark["decline"].as_str().unwrap_or("native_decline"),
                        );
                    }
                }
            }
            assert_eq!(native, n(&r["native_calls"]));
            assert!(
                runs.insert((id, k.clone()), (nt, selected, hits.clone()))
                    .is_none()
            );
            let mut bins = vec![([0.; 4], None); events.len()];
            for receipt in a(&r["receipts"]) {
                let end = f(&receipt["end_ns"]);
                let index = (0..events.len())
                    .min_by(|&i, &j| {
                        (f(&events[i]["time_s"]) * 1e9 - end)
                            .abs()
                            .total_cmp(&(f(&events[j]["time_s"]) * 1e9 - end).abs())
                    })
                    .unwrap();
                bins[index].1.get_or_insert(end);
                for (v, z) in bins[index].0.iter_mut().zip(a(&receipt["corners"])) {
                    *v += f(z);
                }
            }
            for (e, (points, end)) in events.iter().zip(bins).skip(1) {
                let total = points.iter().sum::<f64>();
                add(
                    &mut repeated,
                    &k,
                    "return_impulse_error_ns",
                    (total - f(&e["impulse_ns"])).abs(),
                );
                add(
                    &mut repeated,
                    &k,
                    "return_wrong_corner_ns",
                    total - points[n(&e["corner"]) as usize],
                );
                if let Some(end) = end {
                    add(
                        &mut repeated,
                        &k,
                        "return_endpoint_abs_us",
                        (end - f(&e["time_s"]) * 1e9).abs() / 1000.,
                    );
                } else {
                    bump(&mut counts, &k, "missing_return");
                }
            }
            add(&mut repeated, &k, "com_um", 1e6 * f(&r["final_com_m"]));
            add(&mut repeated, &k, "spin_rad_s", f(&r["final_spin_rad_s"]));
            add(&mut repeated, &k, "native_calls", native as f64);
        }
    }
    let mut perf = BTreeMap::new();
    for name_run in ["cost-a", "cost-b"] {
        let data = read(dir.join(name_run).join("cost.json"));
        let mut costs = Groups::new();
        let mut tail = BTreeMap::<String, Vec<(u64, &str)>>::new();
        let mut loc = BTreeMap::new();
        let mut physical_max = BTreeMap::<(u64, u64, bool, usize), (u64, &str)>::new();
        for r in a(&data["rows"]) {
            let k = name(r);
            let id = n(&r["id"]);
            let budget = n(&r["budget"]);
            let on = r["compact"] == true;
            let (nt, chosen, hits) = &runs[&(id, k.clone())];
            let category = |t: u64| {
                if chosen.contains(&t) {
                    "nominated"
                } else if chosen.contains(&(t - 1)) {
                    "following"
                } else if chosen.contains(&(t + 1)) {
                    "preceding"
                } else if t == 1 {
                    "first_update"
                } else {
                    "other"
                }
            };
            for (rep, s) in a(&r["steps_ns"]).iter().enumerate() {
                assert_eq!(a(s).len(), *nt as usize);
                let mut maximum = (0, 0);
                let mut total = 0;
                for (i, x) in a(s).iter().enumerate() {
                    let t = i as u64 + 1;
                    let time = n(x);
                    total += time;
                    let class = category(t);
                    let us = time as f64 / 1000.;
                    add(&mut costs, &k, "all_us", us);
                    add(&mut costs, &k, class, us);
                    if hits.contains(&t) {
                        add(&mut costs, &k, "physical_event_us", us);
                    }
                    tail.entry(k.clone()).or_default().push((time, class));
                    if time > maximum.1 {
                        maximum = (t, time);
                    }
                }
                add(&mut costs, &k, "whole_trajectory_us", total as f64 / 1000.);
                bump(
                    &mut loc,
                    &k,
                    &format!("prefix_worst_{}", category(maximum.0)),
                );
                physical_max.insert((id, budget, on, rep), (maximum.0, category(maximum.0)));
            }
        }
        for (k, mut rows) in tail {
            assert_eq!(rows.len(), 72028);
            rows.sort_unstable_by_key(|x| std::cmp::Reverse(x.0));
            for (_, category) in &rows[..rows.len().div_ceil(100)] {
                bump(&mut loc, &k, &format!("slowest_one_percent_{category}"));
            }
        }
        for (&(id, budget, on, rep), &(tick, where_)) in &physical_max {
            if on {
                let &(base, base_where) = &physical_max[&(id, budget, false, rep)];
                let k = format!("{budget:02}-compact");
                if base_where == "nominated" {
                    bump(&mut loc, &k, "matched_baseline_worst_on_nomination");
                    if tick == base + 1 && where_ == "following" {
                        bump(&mut loc, &k, "matched_worst_transferred_to_next");
                    }
                }
            }
        }
        let competing = read(dir.join(name_run).join("contending.json"));
        let reference = read(dir.join("contending.json"));
        let mut side = Groups::new();
        for (r, b) in a(&competing["cases"]).iter().zip(a(&reference["cases"])) {
            assert_eq!(r["id"], b["id"]);
            assert_eq!(r["selected"], b["selected"]);
            assert_eq!(r["shared_exact_path"], b["shared_exact_path"]);
            assert!(
                a(&r["shared_exact_path"])
                    .iter()
                    .map(|p| f(&p["other_impulse_ns"]))
                    .sum::<f64>()
                    > 1.
            );
            for (lane, key) in [("full", "full_step_ns"), ("compact", "compact_step_ns")] {
                for samples in a(&r[key]) {
                    for (i, s) in a(samples).iter().enumerate() {
                        add(&mut side, lane, "all_us", f(s) / 1000.);
                        if a(&r["selected"]).iter().any(|t| n(t) == i as u64 + 1) {
                            add(&mut side, lane, "nominated_us", f(s) / 1000.);
                        }
                    }
                }
            }
        }
        perf.insert(
            name_run,
            json!({"latency":stats(costs),"tail_location_counts":loc,"competing_body_us":stats(side)}),
        );
    }
    println!("{}",serde_json::to_string_pretty(&json!({"single":stats(solitary),"isolated_decisions":status,"returns":stats(repeated),"return_decisions":counts,"timings":perf})).unwrap());
}
