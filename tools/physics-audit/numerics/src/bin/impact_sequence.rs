// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent piecewise propagation, reaction accumulation and p95 reconstruction from raw events.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs::File, path::Path};
fn load(p: &Path) -> Value {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(v: &Value) -> &[Value] {
    v.as_array().unwrap()
}
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn vec3(v: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
fn quat(v: &Value) -> DQuat {
    DQuat::from_array(serde_json::from_value(v.clone()).unwrap())
}
fn mean(s: &Value, k: &str) -> DVec3 {
    (vec3(&s["foot"][k]) + vec3(&s["upper"][k])) * 0.5
}
fn near(x: f64, y: f64, e: f64) {
    assert!((x - y).abs() < e, "{x} {y}");
}
fn p95(xs: &mut [f64]) -> f64 {
    xs.sort_by(f64::total_cmp);
    xs[(95 * xs.len()).div_ceil(100) - 1]
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let path = Path::new(&path);
    let d = load(&path.join("sequence.json"));
    let source = load(&path.join("../multifidelity-impact-certification/certification.json"));
    let native = load(&path.join("native.json"));
    let mut hist = BTreeMap::new();
    let mut min_gap = f64::INFINITY;
    let mut min_separation = f64::INFINITY;
    let mut maximum_width = 0.0_f64;
    let mut events = 0;
    for (refrow, control) in a(&d["cases"]).iter().zip(a(&source["cases"])) {
        if refrow["reason"] != "certified" {
            continue;
        }
        let initial = &control["initial"];
        let normal = vec3(&control["normal"]);
        let pad_v = vec3(&initial["pad"]["velocity"]);
        let pad0 = vec3(&initial["pad"]["position"]) + normal * 0.06;
        let mut pos = mean(initial, "position");
        let mut v = mean(initial, "velocity");
        let mut w = vec3(&initial["foot"]["angular_velocity"]);
        let mut q = quat(&initial["foot"]["orientation"]);
        let mut t = 0.;
        let points = [
            [-0.18, -0.10, -0.13],
            [-0.18, 0.10, -0.13],
            [0.18, -0.10, -0.13],
            [0.18, 0.10, -0.13],
        ]
        .map(DVec3::from);
        let flight =
            |t: f64, until: f64, pos: DVec3, v: DVec3, w: DVec3, q: DQuat, minimum: &mut f64| {
                for i in 0..=64 {
                    let dt = (until - t) * i as f64 / 64.;
                    let center = pos + v * dt;
                    let orientation = DQuat::from_scaled_axis(w * dt) * q;
                    for p in points {
                        let gap = (center + orientation * p - pad0 - pad_v * (t + dt)).dot(normal);
                        *minimum = minimum.min(gap);
                        assert!(gap > -2e-11);
                    }
                }
            };
        for (i, event) in a(&refrow["impacts"]).iter().enumerate() {
            let hit = n(&event["time_s"]);
            let dt = hit - t;
            assert!(dt > 0.);
            if i > 0 {
                min_separation = min_separation.min(dt);
            }
            maximum_width =
                maximum_width.max(n(&event["bracket_s"][1]) - n(&event["bracket_s"][0]));
            flight(t, hit, pos, v, w, q, &mut min_gap);
            pos += v * dt;
            q = DQuat::from_scaled_axis(w * dt) * q;
            t = hit;
            let mut torque = DVec3::ZERO;
            let mut total = 0.;
            for (k, pt) in points.iter().enumerate() {
                let r = q * *pt;
                let j = n(&event["corner_ns"][k]);
                assert!(j >= 0.);
                near(
                    (v - pad_v + w.cross(r)).dot(normal),
                    n(&event["before_speed_m_s"][k]),
                    1e-10,
                );
                if event["active"][k].as_bool().unwrap() {
                    near((pos + r - pad0 - pad_v * t).dot(normal), 0., 3e-12);
                } else {
                    assert!((pos + r - pad0 - pad_v * t).dot(normal) > 0.);
                    assert_eq!(j, 0.);
                }
                total += j;
                torque += r.cross(normal * j);
            }
            v += total * normal / 2.;
            w += torque / 0.05;
            events += 1;
            for (k, pt) in points.iter().enumerate() {
                let speed = (v - pad_v + w.cross(q * *pt)).dot(normal);
                near(speed, n(&event["after_speed_m_s"][k]), 1e-10);
                if n(&event["corner_ns"][k]) > 0. {
                    near(speed, 0., 1e-10);
                }
            }
        }
        flight(t, 0.004, pos, v, w, q, &mut min_gap);
        let end = &refrow["final_certified_state"];
        assert!(mean(end, "position").distance(pos + v * (0.004 - t)) < 1e-11);
        assert!(vec3(&end["foot"]["angular_velocity"]).distance(w) < 1e-10);
        *hist.entry(a(&refrow["impacts"]).len()).or_insert(0usize) += 1;
    }
    let mut groups: BTreeMap<(&str, &str, bool), Vec<&Value>> = BTreeMap::new();
    let mut trials = 0;
    for r in a(&native["cases"]) {
        let multi = a(&r["reference"]).len() > 1;
        let label = if multi { "multiple" } else { "single" };
        for row in a(&r["runs"]) {
            trials += 1;
            let cold = row["matched_after_primary"].as_bool().unwrap();
            let name = row["profile"].as_str().unwrap();
            if multi {
                let cut = n(&row["common_midpoint"]["time_ns"]) * 1e-9;
                assert!(
                    cut > n(&r["reference"][0]["time_s"]) && cut < n(&r["reference"][1]["time_s"])
                );
            }
            let target: f64 = a(&r["reference"])
                .iter()
                .skip(usize::from(cold))
                .flat_map(|e| a(&e["corner_ns"]).iter().map(n))
                .sum();
            let id = r["id"].as_u64().unwrap() as usize;
            let normal = vec3(&source["cases"][id]["normal"]);
            near(
                vec3(&row["impulse_vector_ns"]).distance(normal * target),
                n(&row["impulse_vector_error_ns"]),
                1e-10,
            );
            groups.entry((label, name, cold)).or_default().push(row);
        }
    }
    let mut summary = vec![];
    for ((kind, policy, cold), g) in groups {
        let mut m = BTreeMap::new();
        for k in [
            "impulse_vector_error_ns",
            "spin_error_rad_s",
            "com_error_m",
            "corner_l1_error_ns",
        ] {
            let mut v: Vec<_> = g.iter().map(|r| n(&r[k])).collect();
            m.insert(k, p95(&mut v));
        }
        if kind == "multiple" {
            let mut mid: Vec<_> = g
                .iter()
                .map(|r| n(&r["common_midpoint"]["vector_error_ns"]))
                .collect();
            m.insert("midpoint_reaction_error_ns", p95(&mut mid));
        }
        summary.push(
            json!({"cases":kind,"profile":policy,"cold_postprimary":cold,"count":g.len(),"p95":m}),
        );
    }
    let mut cost = vec![];
    let mut samples = 0;
    for file in ["cost.json", "checks/cost-repeat.json"] {
        let d = load(&path.join(file));
        let mut group: BTreeMap<(&str, &str), Vec<f64>> = BTreeMap::new();
        for row in a(&d["samples"]) {
            let k = row["primary_tick"].as_u64().unwrap() as usize - 1;
            let label = if row["event_count"] == 1 {
                "single"
            } else {
                "multiple"
            };
            for t in a(&row["tick_ns"]) {
                samples += a(t).len();
                group
                    .entry((label, row["profile"].as_str().unwrap()))
                    .or_default()
                    .push(n(&t[k]));
            }
        }
        for ((kind, profile), mut v) in group {
            cost.push(json!({"file":file,"cases":kind,"profile":profile,"primary_count":v.len(),"primary_p95_ns":p95(&mut v)}));
        }
    }
    assert_eq!(samples, 228928);
    assert_eq!(trials, 1378);
    assert_eq!(hist.values().sum::<usize>(), 191);
    println!(
        "{}",
        json!({"certified_count_by_impacts":hist,"independent_event_count":events,"min_sampled_clearance_m":min_gap,"minimum_interimpact_s":min_separation,
  "maximum_bracket_width_s":maximum_width,"native_comparisons":trials,"timed_actuator_ticks":samples,"summary":summary,"cost":cost})
    );
}
