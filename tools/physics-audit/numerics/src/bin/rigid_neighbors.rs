// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check actual public reactions, moving-opponent handoff, independent body and wall cost.
use glam::DVec3;
use serde_json::{Value as V, json};
use std::{collections::BTreeMap, fs::File, path::Path};
fn read(p: impl AsRef<Path>) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn arr(v: &V) -> &Vec<V> {
    v.as_array().unwrap()
}
fn f(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn n(v: &V) -> u64 {
    v.as_u64().unwrap()
}
fn vec(v: &V) -> DVec3 {
    DVec3::new(f(&v[0]), f(&v[1]), f(&v[2]))
}
type Samples = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn add(g: &mut Samples, key: &str, metric: &str, value: f64) {
    assert!(value.is_finite());
    g.entry(key.into())
        .or_default()
        .entry(metric.into())
        .or_default()
        .push(value);
}
fn stats(g: Samples) -> V {
    json!(g.into_iter().map(|(k,r)|(k,r.into_iter().map(|(m,mut v)|{v.sort_by(f64::total_cmp);let n=v.len();(m,json!({"n":n,"p50":v[n.div_ceil(2)-1],"p95":v[(95*n).div_ceil(100)-1],"max":v[n-1]}))}).collect::<BTreeMap<_,_>>())).collect::<BTreeMap<_,_>>())
}
fn group(n: u64, clear: bool) -> String {
    format!("{n}-{}", if clear { "swept" } else { "isolated" })
}
fn main() {
    let dir = std::env::args_os().nth(1).unwrap();
    let p = Path::new(&dir);
    let study = read(p.join("study.json"));
    assert_eq!(arr(&study["cases"]).len(), 128);
    let mut all = Samples::new();
    let mut land = BTreeMap::new();
    for c in arr(&study["cases"]) {
        let r = &c["reference"];
        let true_j = vec(&r["target_impulse"]);
        let cut = n(&r["arrival_ns"]);
        land.insert(
            (n(&r["id"]), r["mode"].as_str().unwrap()),
            cut.div_ceil(500000),
        );
        let rows = arr(&c["trials"]);
        let bare = &rows[0]["run"];
        for lane in rows {
            let shelves = n(&lane["static_shelves"]);
            let clear = lane["clear_neighbors"] == true;
            let row = &lane["run"];
            let first = &arr(&row["loads"])[0];
            let hit = &first["projection"];
            let name = group(shelves, clear);
            assert_eq!(!hit.is_null(), clear || shelves == 0);
            if !hit.is_null() {
                assert_eq!(n(&hit["application_ns"]), cut);
                for k in [
                    "loads",
                    "final_com_m",
                    "final_spin_rad_s",
                    "world_impulse",
                    "calls",
                    "departures",
                ] {
                    assert_eq!(row[k], bare[k]);
                }
            }
            let actual = vec(&row["world_impulse"]);
            let j = actual.distance(true_j);
            assert!((j - f(&row["vector_error_ns"])).abs() < 1e-12);
            add(&mut all, &name, "world_reaction_error_ns", j);
            add(&mut all, &name, "com_um", 1e6 * f(&row["final_com_m"]));
            add(&mut all, &name, "spin_rad_s", f(&row["final_spin_rad_s"]));
            add(&mut all, &name, "applied", f64::from(!hit.is_null()));
        }
    }
    let handoff = read(p.join("handoff.json"));
    assert_eq!(arr(&handoff["cases"]).len(), 96);
    let mut hands = Samples::new();
    for c in arr(&handoff["cases"]) {
        let primary = n(&c["primary_ns"]);
        let intended = n(&c["gate_ns"]);
        let d = c["delay_ns"].as_i64().unwrap();
        let reference = vec(&c["reference"]["target_impulse"]);
        let both = arr(&c["trials"]);
        let strict = &both[0]["run"];
        for lane in both {
            let clear = lane["clear"] == true;
            let name = format!("{d}/{}", if clear { "swept" } else { "isolated" });
            let r = &lane["run"];
            let first = n(&r["first_opponent_ns"]);
            let apps = arr(&r["applications"]);
            assert!(apps.len() <= 1);
            if d < 0 {
                assert!(apps.is_empty());
                assert_eq!(strict, r);
            }
            if let Some(h) = apps.first() {
                assert!(clear && d > 0 && n(&h["at_ns"]) < first);
                add(
                    &mut hands,
                    &name,
                    "application_error_ns",
                    n(&h["at_ns"]).abs_diff(primary) as f64,
                );
                add(
                    &mut hands,
                    &name,
                    "primary_vector_error_ns",
                    vec(&h["world"]).distance(reference),
                );
            }
            add(&mut hands, &name, "projected", apps.len() as f64);
            add(
                &mut hands,
                &name,
                "gate_report_signed_lag_us",
                (first as f64 - intended as f64) / 1000.,
            );
            let mut total = 0.;
            for tick in arr(&r["ticks"]) {
                let gate = f(&tick["opponent_normal_ns"]);
                assert!((f(&tick["mean_opponent_force_n"]) * 0.0005 - gate).abs() < 1e-10);
                total += gate;
                if let Some(spans) = tick["spans"].as_array() {
                    for span in spans {
                        assert!(n(&span[1]) <= intended);
                    }
                }
            }
            assert!(total > 0.);
            add(&mut hands, &name, "gate_normal_ns", total);
        }
    }
    let active = read(p.join("active.json"));
    assert_eq!(arr(&active["cases"]).len(), 48);
    let mut acts = Samples::new();
    for c in arr(&active["cases"]) {
        let masked = c["masked"] == true;
        let name = if masked {
            "noncolliding_dynamic"
        } else {
            "dynamic_opponent"
        };
        let trials = arr(&c["trials"]);
        let a = &trials[0];
        let b = &trials[1];
        assert_eq!(n(&a["projected"]), 0);
        assert_eq!(n(&b["projected"]), u64::from(masked));
        let mut nf = 0.;
        for (x, y) in arr(&a["ticks"]).iter().zip(arr(&b["ticks"])) {
            for key in ["witness", "grid_ns", "floor_ns", "force_n"] {
                assert_eq!(x[key], y[key]);
            }
            if !masked {
                assert_eq!(x["foot"], y["foot"]);
            }
            nf += f(&x["floor_ns"]);
        }
        assert!(nf > 0.);
        add(&mut acts, name, "floor_impulse_ns", nf);
    }
    let mut timing = BTreeMap::new();
    for repeat in ["cost-a", "cost-b", "cpu6-a", "cpu6-b"] {
        let input = read(p.join(repeat).join("cost.json"));
        let mut times = Samples::new();
        for row in arr(&input["rows"]) {
            let name = group(n(&row["shelves"]), row["clear"] == true);
            let t = land[&(n(&row["id"]), row["mode"].as_str().unwrap())] as usize;
            for (rep, ts) in arr(&row["steps_ns"]).iter().enumerate() {
                assert_eq!(arr(ts).len(), 8);
                for (j, v) in arr(ts).iter().enumerate() {
                    let us = f(v) / 1e3;
                    add(&mut times, &name, "public_us", us);
                    if let Some(ns) = row["process_cpu_ns"]
                        .get(rep)
                        .and_then(|r| r.get(j))
                        .and_then(V::as_u64)
                    {
                        let cpu = ns as f64 / 1e3;
                        add(&mut times, &name, "public_cpu_us", cpu);
                        if j + 1 == t {
                            add(&mut times, &name, "landing_cpu_us", cpu);
                        }
                    }
                    if j + 1 == t {
                        add(&mut times, &name, "landing_us", us);
                    }
                }
            }
        }
        timing.insert(repeat, stats(times));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"physical":stats(all),"handoff":stats(hands),"active":stats(acts),"cost":timing})).unwrap());
}
