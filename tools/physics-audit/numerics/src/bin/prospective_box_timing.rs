// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recompute physics and cost metrics from committed native boundaries and source cases.
use glam::DVec3;
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs::File, path::Path};
fn read(p: &Path) -> Value {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(v: &Value) -> &[Value] {
    v.as_array().unwrap()
}
fn u(v: &Value) -> u64 {
    v.as_u64().unwrap()
}
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn xyz(v: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
fn com(s: &Value) -> DVec3 {
    0.5 * (xyz(&s["foot"]["position"]) + xyz(&s["upper"]["position"]))
}
fn same(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-10, "{a} {b}");
}
fn p95(s: &mut [f64]) -> f64 {
    s.sort_by(f64::total_cmp);
    s[(95 * s.len()).div_ceil(100) - 1]
}
fn label(p: &Value) -> String {
    format!(
        "{}_{}",
        if p["prospective"] == true {
            "prospective"
        } else {
            "uniform"
        },
        u(&p["calls"])
    )
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let p = Path::new(&path);
    let refs = read(&p.join("../multifidelity-impact-sequence/sequence.json"));
    let source = read(&p.join("../multifidelity-impact-certification/certification.json"));
    let mut summaries = vec![];
    let mut timings = vec![];
    let mut totals = vec![];
    for f in ["checks/two-share-native.json", "native.json"] {
        let d = read(&p.join(f));
        let mut groups = BTreeMap::new();
        let mut solves = 0;
        let mut eligible = 0;
        for c in a(&d["cases"]) {
            let id = u(&c["id"]) as usize;
            let expected = &refs["cases"][id];
            assert_eq!(expected["impacts"], c["reference"]);
            let events = a(&c["reference"]);
            let t = (n(&events[0]["time_s"]) * 1e9).round() as i64;
            let j = events
                .iter()
                .flat_map(|e| a(&e["corner_ns"]))
                .map(n)
                .sum::<f64>()
                * xyz(&source["cases"][id]["normal"]);
            for r in a(&c["trials"]) {
                let mut first = None;
                let mut changes = 0;
                for (tick, x) in a(&r["ticks"]).iter().enumerate() {
                    let ends = a(&x["end_ns"]);
                    let count = u(&r["profile"]["calls"]);
                    assert_eq!(ends.len() as u64, count);
                    assert_eq!(u(ends.last().unwrap()), 500000);
                    assert!(ends.windows(2).all(|w| u(&w[0]) < u(&w[1])));
                    assert!(x["decline"].is_null() && x["decline_after"].is_null());
                    let mut boundaries = 0;
                    let mut from = 0;
                    for (i, h) in ends.iter().enumerate() {
                        let to = u(h);
                        assert!(to > from);
                        boundaries +=
                            usize::from(to - from != (500000 - from) / (count - i as u64));
                        from = to;
                    }
                    if r["profile"]["prospective"] == true {
                        eligible += 1;
                        changes += boundaries;
                        assert_eq!(boundaries as u64, u(&x["retimed"]));
                    }
                    if let Some(v) = x["first_positive_end_ns"].as_u64() {
                        assert!(ends.iter().any(|h| u(h) == v));
                        first.get_or_insert(tick as u64 * 500000 + v);
                    }
                    solves += ends.len();
                }
                assert_eq!(Some(u(&r["first_positive_end_ns"])), first);
                assert_eq!(u(&r["retimed_calls"]), changes as u64);
                let onset = first.unwrap() as i64 - t;
                assert_eq!(onset, r["onset_end_error_ns"].as_i64().unwrap());
                let m = &r["metrics"];
                same(
                    xyz(&m["impulse_ns"]).distance(j),
                    n(&m["impulse_vector_error_ns"]),
                );
                if f == "native.json" {
                    let end = &r["end_state"];
                    let ideal = &expected["final_certified_state"];
                    same(com(end).distance(com(ideal)), n(&m["com_error_m"]));
                    same(
                        xyz(&end["foot"]["angular_velocity"])
                            .distance(xyz(&ideal["foot"]["angular_velocity"])),
                        n(&m["spin_error_rad_s"]),
                    );
                }
                let group = groups
                    .entry((events.len() > 1, label(&r["profile"])))
                    .or_insert_with(|| vec![vec![]; 4]);
                for (i, k) in ["impulse_vector_error_ns", "com_error_m", "spin_error_rad_s"]
                    .iter()
                    .enumerate()
                {
                    group[i].push(n(&m[k]));
                }
                group[3].push(onset.unsigned_abs() as f64);
            }
        }
        assert_eq!(a(&d["cases"]).len(), 73);
        assert_eq!(eligible, 2336);
        for ((multi, profile), mut g) in groups {
            summaries.push(json!({"source":f,"multiple":multi,"profile":profile,"count":g[0].len(),"p95_vector_error_ns":p95(&mut g[0]),"p95_com_error_m":p95(&mut g[1]),"p95_spin_error_rad_s":p95(&mut g[2]),"p95_onset_end_abs_ns":p95(&mut g[3])}));
        }
        totals.push(
            json!({"source":f,"native_observations":solves,"qualified_prospective_ticks":eligible}),
        );
    }
    for f in [
        "checks/two-share-cost.json",
        "cost.json",
        "checks/repeat-cost/cost.json",
    ] {
        let data = read(&p.join(f));
        let mut groups: BTreeMap<_, Vec<f64>> = BTreeMap::new();
        let mut count = 0;
        for s in a(&data["samples"]) {
            let g = groups
                .entry((u(&s["event_count"]) > 1, label(&s["profile"])))
                .or_default();
            let primary = u(&s["primary_tick"]) as usize - 1;
            for t in a(&s["tick_ns"]) {
                assert_eq!(a(t).len(), 8);
                count += 8;
                g.push(n(&t[primary]));
            }
        }
        for ((multi, profile), mut v) in groups {
            timings.push(json!({"source":f,"multiple":multi,"profile":profile,"samples":v.len(),"primary_p95_ns":p95(&mut v)}));
        }
        assert_eq!(count, 56_064);
    }
    let raw = read(&p.join("authored/authored.json"));
    let manifest = read(&p.join("authored/comparison.json"));
    let exact = read(&p.join("authored/traces/analytic.json"));
    let events = a(&raw["reference"]);
    let expected_j = events
        .iter()
        .flat_map(|e| a(&e["corner_ns"]))
        .map(n)
        .sum::<f64>();
    for (i, row) in a(&raw["cases"]).iter().enumerate() {
        let run = &row["run"];
        let j = a(&run["ticks"])
            .iter()
            .map(|t| xyz(&t["impulse_ns"]))
            .sum::<DVec3>();
        same(
            j.distance(expected_j * DVec3::Z),
            n(&run["metrics"]["impulse_vector_error_ns"]),
        );
        let file = manifest["lanes"][i]["file"].as_str().unwrap();
        let lane = read(&p.join("authored").join(file));
        let frames = a(&lane["frames"]);
        let final_state = frames.last().unwrap();
        let reference = a(&exact["frames"]).last().unwrap();
        same(
            com(reference).distance(com(final_state)),
            n(&run["metrics"]["com_error_m"]),
        );
        same(
            xyz(&final_state["foot"]["angular_velocity"])
                .distance(xyz(&reference["foot"]["angular_velocity"])),
            n(&run["metrics"]["spin_error_rad_s"]),
        );
        for (f, t) in frames.iter().skip(1).zip(a(&run["ticks"])) {
            assert_eq!(f["corner_ns"], t["corner_ns"]);
            assert_eq!(f["native_steps"], t["native_steps"]);
        }
        assert_eq!(
            a(&run["ticks"])
                .iter()
                .map(|t| u(&t["native_steps"]))
                .sum::<u64>(),
            u(&run["calls"])
        );
    }
    for f in [
        "authored/cost.json",
        "checks/repeat-cost/authored/cost.json",
    ] {
        let d = read(&p.join(f));
        for k in ["uniform", "prospective"] {
            let mut v = a(&d[k])
                .iter()
                .map(|row| n(&row[(u(&d["primary_tick"]) - 1) as usize]))
                .collect::<Vec<_>>();
            timings.push(
                json!({"source":f,"profile":k,"samples":v.len(),"primary_p95_ns":p95(&mut v)}),
            );
        }
    }
    println!(
        "{}",
        json!({"summary":summaries,"timings":timings,"totals":totals,"authored_events":events.len()})
    );
}
