// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently recompute perfect-plastic impulses from physical initial states and p95 from raw samples.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{fs::File, path::Path};
fn load(p: &Path) -> Value {
    serde_json::from_reader(File::open(p).unwrap()).unwrap()
}
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(a.clone()).unwrap())
}
fn a(v: &Value) -> &[Value] {
    v.as_array().unwrap()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-10, "{a} != {b}")
}
fn main() {
    let path = std::env::args().nth(1).unwrap();
    let p = Path::new(&path);
    let d = load(&p.join("analytic.json"));
    let visual = load(&p.join("traces/rigid.json"));
    let half = v(&visual["geometry"]["foot_half_extents"]);
    let mut total = 0;
    let mut min_gap = f64::INFINITY;
    for c in a(&d["cases"]) {
        let ideal = &c["ideal"];
        let init = &ideal["initial"];
        let foot = &init["foot"];
        let q = DQuat::from_array(serde_json::from_value(foot["orientation"].clone()).unwrap());
        let com = (v(&foot["position"]) + v(&init["upper"]["position"])) * 0.5;
        // Build the bottom corner list directly from serialized geometry.
        let mut points = vec![];
        for x in [-half.x, half.x] {
            for y in [-half.y, half.y] {
                points.push(v(&foot["position"]) + q * DVec3::new(x, y, -half.z));
            }
        }
        let min = points.iter().map(|x| x.z).min_by(f64::total_cmp).unwrap();
        let bottom: Vec<_> = points
            .into_iter()
            .filter(|x| (x.z - min).abs() < 1e-12)
            .collect();
        let lever = bottom.iter().sum::<DVec3>() / bottom.len() as f64 - com;
        let torque = lever.cross(DVec3::Z);
        let closing = n(&c["fixture"]["relative_closing_m_s"]);
        let j = closing / (0.5 + torque.length_squared() / 0.05);
        close(j, n(&ideal["normal_impulse_ns"]));
        let spin = torque * j / 0.05;
        assert!(spin.distance(v(&ideal["outgoing_angular_velocity_rad_s"])) < 1e-10);
        let time = (min
            - init["pad"]["position"][2].as_f64().unwrap()
            - v(&visual["geometry"]["pad_half_extents"]).z)
            / closing;
        close(time, n(&ideal["first_time_ns"]) * 1e-9);
        for group in ["trials", "fixed_pgs_controls", "iteration_controls"] {
            for run in a(&c[group]) {
                let mut impulse = 0.;
                let mut last = 0;
                for receipt in a(&run["native_receipts"]) {
                    let t = receipt["time_ns"].as_u64().unwrap();
                    assert!(t > last);
                    last = t;
                    let js = a(&receipt["corner_ns"]).iter().map(n).sum::<f64>();
                    close(js, n(&receipt["normal_ns"]));
                    impulse += js;
                    if let Some(k) = ideal["corner"].as_u64() {
                        close(js, n(&receipt["corner_ns"][k as usize]));
                    }
                }
                close(impulse - j, n(&run["normal_error_ns"]));
                close(
                    v(&run["final_angular_velocity_rad_s"]).distance(spin),
                    n(&run["angular_velocity_error_rad_s"]),
                );
                if let Some(x) = run["min_other_corner_gap_m"].as_f64() {
                    min_gap = min_gap.min(x);
                }
                total += 1;
            }
        }
    }
    let mut measured = 0;
    for file in ["cost.json", "checks/cost-repeat.json"] {
        let data = load(&p.join(file));
        let primary = data["analytic_impact_tick"].as_u64().unwrap() as usize - 1;
        for row in a(&data["results"]) {
            let ticks = a(&row["step_ns"]);
            assert_eq!(ticks.len(), 48);
            let mut all: Vec<_> = ticks
                .iter()
                .flat_map(|r| a(r).iter().map(|x| x.as_u64().unwrap()))
                .collect();
            measured += all.len();
            let mut hit: Vec<_> = ticks.iter().map(|r| r[primary].as_u64().unwrap()).collect();
            for (vals, field) in [(&mut all, "all_p95_ns"), (&mut hit, "primary_p95_ns")] {
                vals.sort_unstable();
                assert_eq!(
                    vals[(95 * vals.len()).div_ceil(100) - 1],
                    row[field].as_u64().unwrap()
                );
            }
        }
    }
    assert_eq!(total, 204);
    assert_eq!(measured, 18432);
    println!(
        "{}",
        json!({"trials":total,"cost_samples":measured,"minimum_other_corner_native_gap_m":min_gap,"fixtures":a(&d["cases"]).len()})
    );
}
