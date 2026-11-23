// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Score the raw matched and continuous native trajectories plus two independent timing process outputs.
use serde_json::{Value as V, json};
use std::{collections::BTreeMap, env, fs, path::Path};
fn read(p: &Path, key: &str) -> V {
    serde_json::from_reader(fs::File::open(p.join(key)).unwrap()).unwrap()
}
fn rows(v: &V) -> &[V] {
    v.as_array().unwrap()
}
fn val(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn dist(a: &V, b: &V) -> f64 {
    assert_eq!(rows(a).len(), rows(b).len());
    rows(a)
        .iter()
        .zip(rows(b))
        .map(|(x, y)| (val(x) - val(y)).powi(2))
        .sum::<f64>()
        .sqrt()
}
fn avg(v: &[f64]) -> f64 {
    v.iter().sum::<f64>() / v.len() as f64
}
const LIVE: [&str; 5] = ["current", "4x16", "8x4", "8x8", "16x4"];
const REFS: [&str; 3] = ["uniform16", "uniform32", "uniform64"];
fn lookup<'a>(list: &'a V, key: &str, field: &str) -> &'a V {
    &rows(list).iter().find(|r| r["mode"] == key).unwrap()[field]
}
fn first_positive(samples: &[V]) -> &V {
    samples[1..]
        .iter()
        .find(|r| val(&r["normal_ns"]) > 0.)
        .unwrap()
}
fn evaluate(a: &[V], b: &[V], load: usize) -> V {
    let (fa, fb) = (first_positive(a), first_positive(b));
    let impulse = |v: &[V]| {
        v[load..load + 10]
            .iter()
            .map(|r| val(&r["normal_ns"]))
            .sum::<f64>()
    };
    assert!(a.len() == b.len() && a.len() == load + 26);
    json!({"first_normal_ns":(val(&fa["normal_ns"])-val(&fb["normal_ns"])).abs(),
        "first_corner_l2_ns":dist(&fa["corners_ns"],&fb["corners_ns"]),
        "spin_at_common_load_rad_s":dist(&a[load]["spin_rad_s"],&b[load]["spin_rad_s"]),
        "impulse_5ms_ns":(impulse(a)-impulse(b)).abs(),
        "com_5ms_mm":1000.*dist(&a[load+9]["com_m"],&b[load+9]["com_m"]),
        "com_endpoint_mm":1000.*dist(&a.last().unwrap()["com_m"],&b.last().unwrap()["com_m"])})
}
fn classify(held: bool) -> &'static str {
    if held { "held_out" } else { "development" }
}
fn score(p: &Path) -> V {
    let mut metrics: BTreeMap<String, Vec<(f64, f64)>> = BTreeMap::new();
    let mut event_rows = 0;
    let m = read(p, "matched.json");
    for f in rows(&m["fixtures"]) {
        let class = classify(f["held_out"].as_bool().unwrap());
        for t in rows(&f["primaries"]) {
            let load = (t["baseline_first_load"].as_u64().unwrap()
                - t["cut_tick"].as_u64().unwrap()) as usize;
            let list = &t["entries"];
            let clock = t["cut_tick"].as_u64().unwrap();
            for mode in LIVE.into_iter().chain(REFS) {
                let v = rows(lookup(list, mode, "samples"));
                assert_eq!(v[0]["tick"], clock);
                for r in &v[1..] {
                    let sum: f64 = rows(&r["corners_ns"]).iter().map(val).sum();
                    assert!((sum - val(&r["normal_ns"])).abs() < 1e-12);
                    event_rows += 1;
                }
            }
            for reference in REFS.into_iter().chain(["16x4"]) {
                let ref_data = rows(lookup(list, reference, "samples"));
                let baseline = evaluate(rows(lookup(list, "current", "samples")), ref_data, load);
                for policy in LIVE {
                    let errors = evaluate(rows(lookup(list, policy, "samples")), ref_data, load);
                    for (name, v) in errors.as_object().unwrap() {
                        metrics
                            .entry(format!("{class}.{reference}.{policy}.{name}"))
                            .or_default()
                            .push((val(v), val(&baseline[name])));
                    }
                }
            }
        }
    }
    let matched=metrics.into_iter().map(|(k,a)|{
        let e=a.iter().map(|&(v,_)|v).collect::<Vec<_>>();
        (k,json!({"n":e.len(),"mean":avg(&e),"max":e.into_iter().max_by(f64::total_cmp).unwrap(),"lower_than_current":a.iter().filter(|(v,b)|v<b).count()}))
    }).collect::<serde_json::Map<_,_>>();
    let c = read(p, "continuous.json");
    let mut counts = BTreeMap::<String, (usize, usize, usize)>::new();
    let mut rms = BTreeMap::<String, Vec<(f64, f64)>>::new();
    for f in rows(&c["fixtures"]) {
        let class = classify(f["held_out"].as_bool().unwrap());
        let list = &f["histories"];
        for policy in LIVE {
            let r = lookup(list, policy, "run");
            let end = rows(&r["motion_impulse"]).len();
            let triggers = rows(&r["triggers"]);
            let obs = rows(&r["observed_primary_ticks"]);
            let counts = counts.entry(format!("{class}.{policy}")).or_default();
            counts.0 += triggers.len();
            for a in triggers {
                counts.1 +=
                    usize::from(obs.iter().any(|v| v == &a["tick"]) && val(&a["normal_ns"]) > 0.);
                counts.2 += usize::from(a["parked"] == true);
                let i = a["tick"].as_u64().unwrap() as usize;
                assert_eq!(r["motion_impulse"][i - 1][1], a["normal_ns"]);
            }
            for reference in &REFS[..2] {
                let v = rows(&lookup(list, reference, "run")["motion_impulse"]);
                let error = |name| {
                    let h = rows(&lookup(list, name, "run")["motion_impulse"]);
                    assert_eq!(h.len(), end);
                    assert_eq!(v.len(), end);
                    1000.
                        * (h.iter()
                            .zip(v)
                            .map(|(a, b)| dist(&a[0], &b[0]).powi(2))
                            .sum::<f64>()
                            / end as f64)
                            .sqrt()
                };
                rms.entry(format!("{class}.{reference}.{policy}"))
                    .or_default()
                    .push((error(policy), error("current")));
            }
        }
    }
    let continuous=rms.into_iter().map(|(k,v)|(k,json!({"n":v.len(),"mean_trajectory_rms_mm":v.iter().map(|a|a.0).sum::<f64>()/v.len() as f64,
        "improved_count":v.iter().filter(|(p,b)|p<b).count()}))).collect::<serde_json::Map<_,_>>();
    json!({"matched_native_outer_receipts":event_rows,"matched_errors":matched,"continuous":continuous,"continuous_primary_request_hit_park_counts":counts})
}
fn quant(v: &mut [u64]) -> V {
    v.sort_unstable();
    let q = |k: usize| v[(v.len() * k).div_ceil(100) - 1];
    json!({"n":v.len(),"p50_ns":q(50),"p95_ns":q(95)})
}
fn timings(src: &V, mask_source: &V) -> V {
    let mut pools = BTreeMap::<String, Vec<u64>>::new();
    let mut checked = 0;
    let policies = rows(&src["policies"]);
    for (f, f_old) in rows(&src["fixtures"])
        .iter()
        .zip(rows(&mask_source["fixtures"]))
    {
        assert_eq!(f["fixture"], f_old["fixture"]);
        let class = classify(f["held_out"].as_bool().unwrap());
        assert_eq!(rows(&f["orders"]).len(), 10);
        for (pi, mode) in policies.iter().enumerate() {
            for position in 0..5 {
                assert_eq!(
                    rows(&f["orders"])
                        .iter()
                        .filter(|row| row[position] == pi)
                        .count(),
                    2
                );
            }
            let mode = mode.as_str().unwrap();
            let rec = &f["timings"][pi];
            assert_eq!(rec["trigger_ticks"], f_old["timings"][pi]["trigger_ticks"]);
            let mut all = vec![];
            let mut own = vec![];
            let mut candidate = vec![];
            for series in rows(&rec["elapsed_ns"]) {
                let ticks = rows(series);
                assert_eq!(ticks.len(), 2171);
                all.extend(ticks.iter().map(|v| v.as_u64().unwrap()));
                let select = |r: &V| {
                    rows(r)
                        .iter()
                        .map(|i| ticks[i.as_u64().unwrap() as usize - 1].as_u64().unwrap())
                        .collect::<Vec<_>>()
                };
                own.extend(select(&rec["trigger_ticks"]));
                candidate.extend(select(&f["timings"][3]["trigger_ticks"]));
            }
            assert_eq!(quant(&mut all), f["summary"][pi]["all"]);
            assert_eq!(quant(&mut own), f["summary"][pi]["own_primary"]);
            checked += 4;
            for (scope, vals) in [
                ("all", all),
                ("own_primary", own),
                ("candidate_clock", candidate),
            ] {
                pools
                    .entry(format!("{class}.{mode}.{scope}"))
                    .or_default()
                    .extend_from_slice(&vals);
                pools
                    .entry(format!("pooled.{mode}.{scope}"))
                    .or_default()
                    .extend(vals);
            }
        }
    }
    json!({"quantiles_recomputed":checked,"groups":pools.into_iter().map(|(k,mut v)|(k,quant(&mut v))).collect::<BTreeMap<_,_>>()})
}
fn main() {
    let p = env::args().nth(1).unwrap();
    let p = Path::new(&p);
    let mut result = score(p);
    if p.join("timing.json").exists() && p.join("checks/timing-repeat.json").exists() {
        let a = read(p, "timing.json");
        let b = read(p, "checks/timing-repeat.json");
        result["first_process"] = timings(&a, &b);
        result["second_process"] = timings(&b, &a);
    }
    fs::write(
        p.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result).unwrap(),
    )
    .unwrap();
}
