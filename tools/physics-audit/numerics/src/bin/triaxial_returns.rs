// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rebuild each event-bin impulse, point attribution, timing and prefix cost from native receipts.
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
fn put(g: &mut Groups, k: &str, m: &str, x: f64) {
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
    json!({"n":n,"p50":d[n.div_ceil(2)-1],"p95":d[(95*n).div_ceil(100)-1],"max":d[n-1]})
}
fn stats(g: Groups) -> V {
    json!(
        g.into_iter()
            .map(|(k, v)| (
                k,
                v.into_iter()
                    .map(|(m, d)| (m, quant(d)))
                    .collect::<BTreeMap<_, _>>()
            ))
            .collect::<BTreeMap<_, _>>()
    )
}
fn main() {
    let argv = std::env::args().collect::<Vec<_>>();
    let dir = Path::new(&argv[1]);
    let option = if argv.get(2).is_some_and(|v| v == "sweep") {
        "sweep"
    } else {
        "release"
    };
    let raw = read(dir.join("returns.json"));
    let mut g = Groups::new();
    let mut counts = BTreeMap::new();
    let mut cases = BTreeSet::new();
    let mut work = BTreeMap::new();
    let mut convergence = vec![];
    let mut jcon = vec![];
    for c in a(&raw["cases"]) {
        if c["trials"].is_null() {
            continue;
        }
        let id = u(&c["id"]);
        assert!(cases.insert(id));
        let last = u(&c["ticks"]) * 500_000;
        let mut events = vec![];
        for e in a(&c["events"]) {
            let t = f(&e["time_s"]) * 1e9;
            if t > last as f64 {
                continue;
            }
            let b = a(&e["bracket_s"]);
            assert!(f(&b[0]) <= f(&e["time_s"]) && f(&b[1]) >= f(&e["time_s"]));
            assert!(
                f(&e["impulse_ns"]) > 0. && f(&e["closing"]) > 0. && f(&e["neighbor_gap_m"]) > 2e-9
            );
            assert!(f(&e["post_speed"]).abs() < 1e-9 && f(&e["energy_residue_j"]).abs() < 1e-9);
            events.push((t, u(&e["corner"]) as usize, f(&e["impulse_ns"])));
        }
        assert!(events.len() >= 2 && events.windows(2).all(|w| w[0].0 < w[1].0));
        work.insert(
            id,
            (
                last / 500000,
                events
                    .iter()
                    .map(|(time, _, _)| (*time / 500000.).ceil() as u64)
                    .collect::<BTreeSet<_>>(),
            ),
        );
        if option == "release" {
            convergence.push(f(&c["half_step_root_convergence_ns"]));
            jcon.push(f(&c["half_step_impulse_convergence_ns"]));
        }
        for run in a(&c["trials"]) {
            let enabled = run[option] == true;
            let k = format!(
                "{:02}-{}",
                u(&run["budget"]),
                if enabled { "enabled" } else { "baseline" }
            );
            let tbl = counts.entry(k.clone()).or_insert([0u64; 6]);
            let r = &run["trial"];
            let mut bins = vec![([0.; 4], None); events.len()];
            let mut last_end = 0;
            for p in a(&r["receipts"]) {
                let end = u(&p["end_ns"]);
                assert!(end > last_end && end <= last);
                last_end = end;
                let near = (0..events.len())
                    .min_by(|&x, &y| {
                        (end as f64 - events[x].0)
                            .abs()
                            .total_cmp(&(end as f64 - events[y].0).abs())
                    })
                    .unwrap();
                bins[near].1.get_or_insert(end);
                for (j, v) in a(&p["corners"]).iter().enumerate() {
                    let x = f(v);
                    assert!(x >= 0.);
                    bins[near].0[j] += x;
                }
            }
            for (j, ((t, corner, target), (loads, first))) in events.iter().zip(bins).enumerate() {
                if j == 0 {
                    continue;
                }
                tbl[0] += 1;
                let sum = loads.iter().sum::<f64>();
                put(&mut g, &k, "return_reaction_error_ns", (sum - target).abs());
                put(&mut g, &k, "return_other_corner_ns", sum - loads[*corner]);
                if let Some(end) = first {
                    put(
                        &mut g,
                        &k,
                        "first_return_endpoint_abs_us",
                        (end as f64 - t).abs() / 1000.,
                    );
                } else {
                    tbl[1] += 1;
                }
            }
            for h in a(&r["release_intervals_ns"]) {
                let h = a(h);
                let (start, end) = (u(&h[0]), u(&h[1]));
                assert!(start < end && end <= last);
                tbl[2] += 1;
                if events
                    .iter()
                    .any(|(t, _, _)| (start as f64) < *t && *t <= end as f64)
                {
                    tbl[3] += 1;
                }
            }
            if let Some(z) = r["minimum_departure_gap_m"].as_f64() {
                assert!(z > 0.);
                put(&mut g, &k, "independent_min_departure_gap_m", z);
            }
            if let Some(x) = r["sweep_queries"].as_u64() {
                tbl[4] += x;
                tbl[5] += a(&r["sweep_candidates"]).len() as u64;
            }
            put(
                &mut g,
                &k,
                "max_departure_spin_residual_rad_s",
                f(&r["maximum_departure_spin_residual_rad_s"]),
            );
            put(
                &mut g,
                &k,
                "prefix_com_error_um",
                f(&r["final_com_m"]) * 1e6,
            );
            put(
                &mut g,
                &k,
                "prefix_spin_error_rad_s",
                f(&r["final_spin_rad_s"]),
            );
            put(&mut g, &k, "native_calls", f(&r["native_calls"]));
        }
    }
    assert_eq!(cases.len(), 274);
    let mut timed = BTreeMap::new();
    for name in ["cost-a", "cost-b"] {
        let data = read(dir.join(name).join("cost.json"));
        let mut g = Groups::new();
        let mut n = 0;
        for r in a(&data["rows"]) {
            let (ticks, impacts) = &work[&u(&r["id"])];
            let k = if r[option] == true {
                "enabled"
            } else {
                "baseline"
            };
            for samples in a(&r["steps_ns"]) {
                assert_eq!(a(samples).len(), *ticks as usize);
                for (i, s) in a(samples).iter().enumerate() {
                    put(&mut g, k, "all_steps_us", f(s) / 1000.);
                    if impacts.contains(&(i as u64 + 1)) {
                        put(&mut g, k, "impact_steps_us", f(s) / 1000.);
                    }
                    n += 1;
                }
            }
        }
        assert_eq!(n, 144_056);
        timed.insert(name, stats(g));
    }
    let exact = if convergence.is_empty() {
        V::Null
    } else {
        json!({"half_grid_time_ns":quant(convergence),"half_grid_impulse_ns":quant(jcon)})
    };
    println!("{}",serde_json::to_string_pretty(&json!({"trajectories":cases.len(),"convergence":exact,"comparisons":stats(g),"count_fields":["secondary_events","missing_bin_load","protected_release_intervals","global_reference_crossings_during_protection","public_sweep_queries","public_sweep_candidates"],"counts":counts,"cost":timed})).unwrap());
}
