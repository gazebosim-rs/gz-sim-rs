// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recompute the analytical corner impulse from the authored scene's loaded rigid states.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{fs::File, path::Path};
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
fn close(x: f64, y: f64, eps: f64) {
    assert!((x - y).abs() < eps, "{x} != {y}");
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let path = Path::new(&path);
    let data = load(&path.join("sdf.json"));
    let first = load(&path.join("traces/default.json"));
    let initial = &first["frames"][0];
    let foot = &initial["foot"];
    let upper = &initial["upper"];
    let pad = &initial["pad"];
    let q = DQuat::from_array(serde_json::from_value(foot["orientation"].clone()).unwrap());
    let p = v(&foot["position"]);
    let center = (p + v(&upper["position"])) * 0.5;
    let half = v(&first["geometry"]["foot_half_extents"]);
    let mut bottom = vec![];
    for x in [-half.x, half.x] {
        for y in [-half.y, half.y] {
            bottom.push(p + q * DVec3::new(x, y, -half.z));
        }
    }
    bottom.sort_by(|a, b| a.z.total_cmp(&b.z));
    let r = bottom[0] - center;
    let perp = r.cross(DVec3::Z);
    let c = (v(&pad["velocity"]) - v(&foot["velocity"])).z;
    let exact = c / (0.5 + perp.length_squared() / 0.05);
    let spin = perp * exact / 0.05;
    close(exact, n(&data["ideal_normal_ns"]), 1e-12);
    for (i, c) in a(&data["cases"]).iter().enumerate() {
        let trace = load(&path.join(if i == 0 {
            "traces/default.json"
        } else {
            "traces/fresh.json"
        }));
        let frames = a(&trace["frames"]);
        assert_eq!(frames.len(), 9);
        let ticks = a(&c["ticks"]);
        let mut total = 0.;
        let mut changes = 0;
        let mut scopes = 0;
        for ((k, r), f) in ticks.iter().enumerate().zip(frames) {
            assert!(f.get("corner_ns").is_none());
            assert_eq!(f["time_us"].as_u64().unwrap(), 500 * k as u64);
            close(n(&r["total_normal_ns"]), n(&f["normal_impulse_ns"]), 1e-14);
            if r["native_steps"] == 1 {
                close(
                    a(&r["last_native_observed_corner_ns"]).iter().map(n).sum(),
                    n(&r["total_normal_ns"]),
                    1e-14,
                );
            }
            total += n(&f["normal_impulse_ns"]);
            changes += r["rebuilt_manifolds"].as_u64().unwrap();
            scopes += u64::from(r["feature_requests"].as_u64().unwrap() > 0);
        }
        assert_eq!(changes, c["rebuilt_manifolds"].as_u64().unwrap());
        assert_eq!(scopes, c["feature_ticks"].as_u64().unwrap());
        close(total - exact, n(&c["total_error_ns"]), 1e-12);
        close(
            v(&frames[8]["foot"]["angular_velocity"]).distance(spin),
            n(&c["spin_error_rad_s"]),
            1e-12,
        );
        if i == 1 {
            assert!(changes > 0);
            assert!(n(&c["normal_after_primary_ns"]).abs() < 1e-12);
        }
    }
    let mut observed = 0;
    let mut times = vec![];
    for file in ["cost.json", "checks/cost-repeat.json"] {
        let cost = load(&path.join(file));
        for row in a(&cost["timing"]) {
            let records = a(&row["tick_ns"]);
            assert_eq!(records.len(), 80);
            let mut primary: Vec<_> = records.iter().map(|t| t[4].as_u64().unwrap()).collect();
            let mut sustained: Vec<_> = records
                .iter()
                .flat_map(|t| a(t)[5..].iter().map(|v| v.as_u64().unwrap()))
                .collect();
            for (values, key) in [
                (&mut primary, "primary_p95_ns"),
                (&mut sustained, "following_ticks_p95_ns"),
            ] {
                values.sort_unstable();
                assert_eq!(
                    row[key].as_u64().unwrap(),
                    values[(95 * values.len()).div_ceil(100) - 1]
                );
            }
            observed += records.len() * 8;
            times.push(json!({"enabled":row["enabled"],"primary_p95_ns":row["primary_p95_ns"],"following_ticks_p95_ns":row["following_ticks_p95_ns"]}));
        }
    }
    assert_eq!(observed, 2560);
    println!(
        "{}",
        json!({"exact_normal_ns":exact,"physical_runs":2,"recorded_timed_ticks":observed,"timing":times})
    );
}
