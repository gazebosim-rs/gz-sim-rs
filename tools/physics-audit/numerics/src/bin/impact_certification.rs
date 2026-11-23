// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently reconstruct impacts from exported initial states, certify trajectory samples,
//! and reproduce pooled p95 from Rust-generated physical and elapsed-time receipts.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs::File, path::Path};
fn load(p: &Path) -> Value {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn a(x: &Value) -> &[Value] {
    x.as_array().unwrap()
}
fn n(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(x.clone()).unwrap())
}
fn q(x: &Value) -> DQuat {
    DQuat::from_array(serde_json::from_value(x.clone()).unwrap())
}
fn vecs(s: &Value, field: &str) -> DVec3 {
    (v(&s["foot"][field]) + v(&s["upper"][field])) * 0.5
}
fn close(x: f64, y: f64, tol: f64) {
    assert!((x - y).abs() < tol, "{x} {y}");
}
fn p95(xs: &mut [f64]) -> f64 {
    xs.sort_by(f64::total_cmp);
    xs[(95 * xs.len()).div_ceil(100) - 1]
}
fn primary_cost(path: &Path) -> (Value, usize) {
    let d = load(path);
    let mut group: BTreeMap<&str, Vec<f64>> = BTreeMap::new();
    let mut ticks = 0;
    for s in a(&d["samples"]) {
        let k = s["primary_tick"].as_u64().unwrap() as usize - 1;
        let pp = group.entry(s["policy"].as_str().unwrap()).or_default();
        for row in a(&s["tick_ns"]) {
            ticks += a(row).len();
            pp.push(n(&row[k]));
        }
    }
    let mut output = BTreeMap::new();
    for s in a(&d["summaries"]) {
        let label = s["policy"].as_str().unwrap();
        let vals = group.get_mut(label).unwrap();
        assert_eq!(vals.len() as u64, s["primary_count"].as_u64().unwrap());
        let primary = p95(vals);
        close(primary, n(&s["primary_p95_ns"]), 1e-8);
        output.insert(label, primary);
    }
    (json!(output), ticks)
}
fn main() {
    let p = std::env::args().nth(1).unwrap();
    let p = Path::new(&p);
    let d = load(&p.join("certification.json"));
    let frames = load(&p.join("traces/analytic.json"));
    let half = v(&frames["geometry"]["foot_half_extents"]);
    let ph = v(&frames["geometry"]["pad_half_extents"]);
    let mut certified = 0;
    let mut runs = 0;
    let mut minimum = f64::INFINITY;
    let mut max_unilateral = 0.0_f64;
    let mut cases: BTreeMap<&str, usize> = BTreeMap::new();
    let mut grouped: BTreeMap<(&str, &str), Vec<&Value>> = BTreeMap::new();
    for c in a(&d["cases"]) {
        runs += a(&c["runs"]).len();
        if !c["certified"].as_bool().unwrap() {
            continue;
        }
        certified += 1;
        let feature = c["feature"].as_str().unwrap();
        *cases.entry(feature).or_default() += 1;
        let initial = &c["initial"];
        let target = &c["target"];
        let t = n(&c["arrival_ns"]) * 1e-9;
        let normal = q(&initial["pad"]["orientation"]) * DVec3::Z;
        assert!(normal.distance(v(&c["normal"])) < 1e-12);
        let center = vecs(initial, "position");
        let vel = vecs(initial, "velocity");
        let omega = v(&initial["foot"]["angular_velocity"]);
        let q0 = q(&initial["foot"]["orientation"]);
        let cc = q0.conjugate() * (v(&initial["foot"]["position"]) - center);
        let qhit = DQuat::from_scaled_axis(omega * t) * q0;
        let hit = center + vel * t;
        let top0 = v(&initial["pad"]["position"]) + normal * ph.z;
        let vp = v(&initial["pad"]["velocity"]);
        let top = top0 + vp * t;
        let mut local = vec![];
        let mut torque = DVec3::ZERO;
        let mut j = 0.;
        for (i, x) in [-half.x, half.x].iter().enumerate() {
            for (k, y) in [-half.y, half.y].iter().enumerate() {
                let offset = cc + DVec3::new(*x, *y, -half.z);
                let r = qhit * offset;
                local.push(offset);
                let load = n(&c["impulses_ns"][2 * i + k]);
                let active = c["active"][2 * i + k].as_bool().unwrap();
                if active {
                    assert!(load > 0.);
                    close((hit + r - top).dot(normal), 0., 1e-12);
                } else {
                    assert!((hit + r - top).dot(normal) > 0.);
                    assert_eq!(load, 0.);
                }
                j += load;
                torque += r.cross(normal * load);
            }
        }
        let vo = vel + normal * j / 2.;
        let wo = omega + torque / 0.05;
        assert!(vo.distance(vecs(target, "velocity")) < 1e-10);
        assert!(wo.distance(v(&target["foot"]["angular_velocity"])) < 1e-10);
        let post = 0.004 - t;
        assert!((hit + vo * post).distance(vecs(target, "position")) < 1e-12);
        for (k, p) in local.iter().enumerate() {
            let relative = (vo - vp + wo.cross(qhit * *p)).dot(normal);
            if c["active"][k].as_bool().unwrap() {
                max_unilateral = max_unilateral.max(relative.abs());
            }
            for step in 0..=256 {
                let time = 0.004 * step as f64 / 256.;
                let offset = time - t;
                let (w, velocity) = if offset < 0. { (omega, vel) } else { (wo, vo) };
                let corner =
                    hit + velocity * offset + DQuat::from_scaled_axis(w * offset) * (qhit * *p);
                let local =
                    q(&initial["pad"]["orientation"]).conjugate() * (corner - top0 - vp * time);
                assert!(local.x.abs() < ph.x && local.y.abs() < ph.y);
                minimum = minimum.min(local.z);
                assert!(local.z > -1e-12);
            }
        }
        for row in a(&c["runs"]) {
            if row["representation"] != "locked_pair" {
                continue;
            }
            let policy = row["policy"].as_str().unwrap();
            grouped.entry(("all", policy)).or_default().push(row);
            grouped.entry((feature, policy)).or_default().push(row);
            close(
                v(&row["reaction_vector_ns"]).distance(normal * j),
                n(&row["impulse_vector_error_ns"]),
                1e-11,
            );
        }
    }
    assert_eq!(runs, 2400);
    assert_eq!(certified, 156);
    assert!(max_unilateral < 1e-10);
    let mut summaries = vec![];
    for ((feature, policy), rows) in grouped {
        let mut values = BTreeMap::new();
        for k in [
            "impulse_vector_error_ns",
            "angular_error_rad_s",
            "com_error_m",
            "refreshed_calls",
            "relaxed_calls",
        ] {
            let mut v: Vec<_> = rows.iter().map(|r| n(&r[k])).collect();
            values.insert(
                k,
                json!({"max":v.iter().copied().fold(0.,f64::max),"p95":p95(&mut v)}),
            );
        }
        summaries
            .push(json!({"feature":feature,"policy":policy,"count":rows.len(),"metrics":values}));
    }
    let (cost, ticks) = primary_cost(&p.join("cost.json"));
    let (repeat, other) = primary_cost(&p.join("checks/cost-repeat.json"));
    assert_eq!(ticks, 39_936);
    assert_eq!(ticks, other);
    println!(
        "{}",
        json!({"runs":runs,"certified":certified,"feature_count":cases,"min_sampled_clearance_m":minimum,
        "max_postimpact_active_speed_m_s":max_unilateral,"timed_tick_count":ticks+other,"primary_p95_ns":cost,"repeat_primary_p95_ns":repeat,"summaries":summaries})
    );
}
