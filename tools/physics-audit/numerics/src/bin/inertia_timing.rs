// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct triaxial impact and separation measurements from independent boundary records.
#[path = "shared/gyro_cut.rs"]
mod gyro_cut;
use glam::{DMat3, DQuat, DVec3};
use serde_json::{Value as V, json};
use std::{collections::BTreeMap, fs::File, path::Path};

fn read(p: &Path) -> V {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(v: &V) -> &[V] {
    v.as_array().unwrap()
}
fn n(v: &V) -> f64 {
    v.as_f64().unwrap()
}
fn u(v: &V) -> u64 {
    v.as_u64().unwrap()
}
fn v(x: &V) -> DVec3 {
    DVec3::from_array(serde_json::from_value(x.clone()).unwrap())
}
fn q(x: &V) -> DQuat {
    DQuat::from_array(serde_json::from_value(x.clone()).unwrap())
}
fn avg(s: &V, field: &str) -> DVec3 {
    (v(&s["foot"][field]) + v(&s["upper"][field])) * 0.5
}
fn tensor(c: &V) -> DMat3 {
    DMat3::from_cols_array(&serde_json::from_value(c["aggregate_tensor_columns"].clone()).unwrap())
}
fn euclidean(a: f64, b: f64) {
    assert!(
        (a - b).abs() < 1e-10 * (1. + a.abs().max(b.abs())),
        "{a} vs {b}"
    );
}
fn energy(t: DMat3, s: &V) -> (DVec3, f64) {
    let orientation = q(&s["foot"]["orientation"]);
    let w = v(&s["foot"]["angular_velocity"]);
    let l = orientation * (t * (orientation.conjugate() * w));
    (l, avg(s, "velocity").length_squared() + 0.5 * w.dot(l))
}
fn reference(c: &V) {
    let t = tensor(c);
    let nrm = v(&c["normal"]);
    let post = &c["exact_at"];
    let j = n(&c["impulse_ns"]);
    let l = v(&c["incoming_L"]);
    let rot = q(&post["foot"]["orientation"]);
    let inverse = DMat3::from_quat(rot) * t.inverse() * DMat3::from_quat(rot).transpose();
    let k = u(&c["corner"]);
    let arm = rot
        * DVec3::new(
            if k >= 2 { 0.18 } else { -0.18 },
            if k % 2 == 1 { 0.1 } else { -0.1 },
            -0.13,
        );
    let axis = arm.cross(nrm);
    let incoming = avg(&c["initial"], "velocity");
    let pad = v(&post["pad"]["velocity"]);
    let closing = -(incoming - pad + (inverse * l).cross(arm)).dot(nrm);
    euclidean(closing, n(&c["closing_speed_m_s"]));
    euclidean(j, closing / (0.5 + axis.dot(inverse * axis)));
    euclidean((l + axis * j - v(&c["outgoing_L"])).length(), 0.);
    euclidean(
        (avg(post, "velocity") - incoming - 0.5 * j * nrm).length(),
        0.,
    );
    euclidean(
        (energy(t, post).1 - energy(t, &c["initial"]).1) - j * nrm.dot(pad),
        -0.5 * j * closing,
    );
    let hole = v(&post["foot"]["position"]) + arm + rot * DVec3::new(0., 0., 0.07)
        - v(&post["pad"]["position"]);
    euclidean(hole.dot(nrm), 0.06);
}
fn summarize(mut values: Vec<f64>) -> V {
    values.sort_by(f64::total_cmp);
    if values.is_empty() {
        return V::Null;
    }
    let rank = |p: usize| values[(p * values.len()).div_ceil(100) - 1];
    json!({"n":values.len(),"min":values[0],"p50":rank(50),"p95":rank(95),"max":values.last()})
}
type Series = BTreeMap<String, BTreeMap<String, Vec<f64>>>;
fn add(groups: &mut Series, group: &str, metric: &str, x: f64) {
    assert!(x.is_finite());
    groups
        .entry(group.into())
        .or_default()
        .entry(metric.into())
        .or_default()
        .push(x);
}
fn group(locked: bool, calls: u64, enabled: bool) -> String {
    format!(
        "{}_{}_{}",
        if locked { "locked" } else { "free" },
        calls,
        if enabled { "on" } else { "off" }
    )
}
fn aggregate(groups: Series) -> BTreeMap<String, BTreeMap<String, V>> {
    groups
        .into_iter()
        .map(|(g, s)| (g, s.into_iter().map(|(m, v)| (m, summarize(v))).collect()))
        .collect()
}
fn trajectory(c: &V, r: &V, group: &str, groups: &mut Series) {
    let ticks = a(&r["ticks"]);
    assert_eq!(ticks.len(), 8);
    let arrive = (n(&c["arrival_s"]) * 1e9).round() as u64;
    let mut first = None;
    let mut pre = 0.;
    let mut j = DVec3::ZERO;
    let mut guarded = 0;
    let mut calls = 0;
    for (i, tick) in ticks.iter().enumerate() {
        assert_eq!(u(&tick["tick"]), i as u64 + 1);
        let ends = a(&tick["native_end_ns"]);
        let loads = a(&tick["native_normal_ns"]);
        let gaps = a(&tick["native_end_gap_m"]);
        assert_eq!(ends.len(), loads.len());
        assert_eq!(ends.len(), gaps.len());
        assert_eq!(u(&tick["calls"]), ends.len() as u64);
        assert_eq!(ends.last().unwrap(), 500000);
        assert!(ends.windows(2).all(|t| u(&t[0]) < u(&t[1])));
        calls += ends.len() as u64;
        let flags = tick["guarded"].as_array();
        if let Some(flags) = flags {
            assert_eq!(flags.len(), ends.len());
        }
        for (i_end, (end, load)) in ends.iter().zip(loads).enumerate() {
            let full = i as u64 * 500000 + u(end);
            let load = n(load);
            assert!(load >= 0.);
            if load > 0. {
                first.get_or_insert(full);
            }
            if full < arrive {
                pre += load;
            }
            if flags.is_some_and(|f| f[i_end] == true) {
                assert_eq!(load, 0.);
                guarded += 1;
                add(groups, group, "guarded_end_clearance_m", n(&gaps[i_end]));
            }
        }
        let reaction = v(&tick["j"]);
        j += reaction;
        euclidean(reaction.length(), loads.iter().map(n).sum());
    }
    assert_eq!(first, r["first_positive_end_ns"].as_u64());
    euclidean(pre, n(&r["preimpact_measured_ns"]));
    assert_eq!(guarded, u(&r["guarded_calls"]));
    assert_eq!(calls, u(&r["native_calls"]));
    let jref = v(&c["normal"]) * n(&c["impulse_ns"]);
    let end = &r["end"];
    let exact = &c["exact_end"];
    let t = tensor(c);
    let (l, e) = energy(t, end);
    let e0 = energy(t, &c["initial"]).1;
    let padwork = j.dot(v(&exact["pad"]["velocity"]));
    let metrics = [
        ("impulse_vector_error_ns", j.distance(jref)),
        (
            "com_error_m",
            avg(end, "position").distance(avg(exact, "position")),
        ),
        (
            "spin_error_rad_s",
            v(&end["foot"]["angular_velocity"]).distance(v(&exact["foot"]["angular_velocity"])),
        ),
        (
            "angular_momentum_error_nms",
            l.distance(v(&c["outgoing_L"])),
        ),
        (
            "dissipation_error_j",
            (e0 + padwork - e - 0.5 * n(&c["closing_speed_m_s"]) * n(&c["impulse_ns"])).abs(),
        ),
        (
            "linear_impulse_closure_ns",
            (2. * (avg(end, "velocity") - avg(&c["initial"], "velocity")) - j).length(),
        ),
    ];
    for (key, value) in metrics {
        euclidean(value, n(&r["metrics"][key]));
        add(groups, group, key, value);
    }
    for k in [
        "free_orientation_drift_rad",
        "free_energy_drift_j",
        "free_angular_momentum_drift_nms",
        "max_link_relative_rotation_rad",
        "max_link_spin_difference_rad_s",
    ] {
        add(groups, group, k, n(&r["metrics"][k]));
    }
    add(
        groups,
        group,
        "first_load_error_ns",
        first.unwrap() as f64 - arrive as f64,
    );
    add(groups, group, "preimpact_impulse_ns", pre);
    add(groups, group, "native_calls", calls as f64);
    add(groups, group, "guarded_calls", guarded as f64);
    add(groups, group, "timing_declines", n(&r["declined_ticks"]));
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let path = Path::new(&path);
    let guard = path.join("guard.json").exists();
    let gyro = path.join("flight.json").exists();
    let variants = if gyro { 4 } else { 8 };

    let refs = read(&if guard || gyro {
        path.join("../multifidelity-inertia-timing/inertia.json")
    } else {
        path.join("inertia.json")
    });
    let source = if gyro {
        read(&path.join("flight.json"))
    } else if guard {
        read(&path.join("guard.json"))
    } else {
        refs.clone()
    };
    let mut groups = Series::new();
    let mut pre_counts = BTreeMap::<String, usize>::new();
    let mut accepted = 0;
    let mut cases = BTreeMap::new();
    let mut reference_checks = Series::new();
    let mut excluded = BTreeMap::<String, usize>::new();
    for c in a(&refs["cases"]) {
        if let Some(reason) = c["certificate"]["reason"].as_str() {
            *excluded.entry(reason.into()).or_default() += 1;
            continue;
        }
        reference(c);
        accepted += 1;
        for k in ["momentum_error", "energy_error", "orientation_error"] {
            add(&mut reference_checks, "Euler", k, n(&c["certificate"][k]));
        }
        add(
            &mut reference_checks,
            "Euler",
            "frozen_error_rad",
            n(&c["frozen_incoming_orientation_error_rad"]),
        );
    }
    for c in a(&source["cases"]) {
        let id = u(&c["id"]) as usize;
        let oracle = &refs["cases"][id];
        let Some(trials) = c["trials"].as_array() else {
            continue;
        };
        assert!(oracle["certificate"]["reason"].is_null());
        assert_eq!(trials.len(), variants);
        for r in trials {
            let live = if gyro {
                r["precise_rotation"] == true
            } else if guard {
                r["guard_enabled"] == true
            } else {
                r["profile"]["prospective"] == true
            };
            let g = group(r["locked"] == true, u(&r["profile"]["calls"]), live);
            trajectory(oracle, r, &g, &mut groups);
            if gyro {
                let measured = &r["first_cut"];
                let at = n(&measured["measured_crossing_ns"]);
                gyro_cut::verify(
                    oracle,
                    &measured["begin"],
                    at,
                    u(&measured["measured_corner"]) as usize,
                );
                add(
                    &mut groups,
                    &g,
                    "measured_cut_root_error_ns",
                    (at - n(&oracle["arrival_s"]) * 1e9).abs(),
                );
                add(
                    &mut groups,
                    &g,
                    "cut_angular_momentum_drift_nms",
                    n(&measured["cut_momentum_drift_nms"]),
                );
                add(
                    &mut groups,
                    &g,
                    "cut_energy_drift_j",
                    n(&measured["cut_energy_drift_j"]).abs(),
                );
                if live {
                    for tick in a(&r["ticks"]) {
                        if let Some(slices) = tick["rotation_slices"].as_array() {
                            let f = a(&tick["guarded"]);
                            assert_eq!(slices.len(), f.len());
                            for (i, s) in slices.iter().enumerate() {
                                let n = u(s);
                                assert!([0, 2, 4, 8, 16, 32].contains(&n));
                                if n > 0 {
                                    assert_eq!(f[i], true);
                                    add(&mut groups, &g, "accepted_rotation_slices", n as f64);
                                }
                            }
                        }
                    }
                }
            }
            *pre_counts.entry(g.clone()).or_default() +=
                usize::from(n(&r["preimpact_measured_ns"]) > 1e-12);
            cases.insert(
                (id, r["locked"] == true, u(&r["profile"]["calls"]), live),
                r,
            );
        }
    }
    if gyro {
        let cuts = read(&path.join("cuts.json"));
        assert_eq!(a(&cuts["cases"]).len(), 108);
        for c in a(&cuts["cases"]) {
            let oracle = &refs["cases"][u(&c["id"]) as usize];
            let time = n(&c["reference_ns"]);
            let warm = &c["guarded"];
            gyro_cut::verify(
                oracle,
                &warm["begin"],
                n(&warm["measured_crossing_ns"]),
                u(&warm["measured_corner"]) as usize,
            );
            for x in a(&c["ghost"]) {
                let g = format!("ghost_{}", u(&x["calls"]));
                let root = n(&x["measured_crossing_ns"]);
                gyro_cut::verify(oracle, &x["cut"], root, u(&x["measured_corner"]) as usize);
                add(
                    &mut reference_checks,
                    &g,
                    "crossing_error_ns",
                    (root - time).abs(),
                );
                for k in [
                    "orientation_error_rad",
                    "angular_momentum_error_nms",
                    "energy_error_j",
                    "com_error_m",
                    "angular_velocity_error_rad_s",
                ] {
                    add(&mut reference_checks, &g, k, n(&x[k]).abs());
                }
            }
        }
    }
    assert_eq!(accepted, 108);
    assert_eq!(cases.len(), accepted * variants);
    let mut costs = vec![];
    for file in ["cost.json", "checks/repeat-cost/cost.json"] {
        let data = read(&path.join(file));
        let mut stats = Series::new();
        let mut count = 0;
        for r in a(&data["rows"]) {
            let id = u(&r["id"]) as usize;
            let locked = r["locked"] == true;
            let calls = if guard || gyro {
                u(&r["calls"])
            } else {
                u(&r["profile"]["calls"])
            };
            let live = if gyro {
                r["precise_rotation"] == true
            } else if guard {
                r["guard_enabled"] == true
            } else {
                r["profile"]["prospective"] == true
            };
            let g = group(locked, calls, live);
            let trial = cases[&(id, locked, calls, live)];
            let primary = u(&r["primary_tick"]) as usize - 1;
            assert_eq!(
                primary,
                (n(&refs["cases"][id]["arrival_s"]) * 2000.).ceil() as usize - 1
            );
            assert_eq!(a(&r["tick_ns"]).len(), 8);
            for trial_times in a(&r["tick_ns"]) {
                assert_eq!(a(trial_times).len(), 8);
                for (i, b) in a(trial_times).iter().enumerate() {
                    let us = n(b) / 1000.;
                    count += 1;
                    add(&mut stats, &g, "all_us", us);
                    if i == primary {
                        add(&mut stats, &g, "impact_us", us);
                    }
                    if i < primary {
                        add(&mut stats, &g, "preceding_us", us);
                    }
                    if trial["ticks"][i]["calls"] == 1 {
                        add(&mut stats, &g, "one_call_us", us);
                    }
                }
            }
        }
        assert_eq!(count, accepted * variants * 8 * 8);
        costs.push(json!({"file":file,"timed_public_steps":count,"groups":aggregate(stats)}));
    }
    println!("{}",serde_json::to_string_pretty(&json!({"certified":accepted,"excluded":excluded,"reference":aggregate(reference_checks),"trajectories":cases.len(),"preimpact_cases":pre_counts,"grouped":aggregate(groups),"costs":costs})).unwrap());
}
