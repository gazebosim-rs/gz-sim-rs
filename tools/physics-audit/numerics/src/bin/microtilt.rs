// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent reflection, Coulomb, momentum, reaction and sampled-phase checks.
use glam::{DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeSet, error::Error, fs, path::PathBuf};
fn num(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|i| num(&x[i])))
}
fn q(x: &Value) -> DQuat {
    DQuat::from_array(std::array::from_fn(|i| num(&x[i])))
}
fn median(values: &[f64]) -> f64 {
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    (v[(v.len() - 1) / 2] + v[v.len() / 2]) * 0.5
}
fn max(values: &[f64]) -> f64 {
    values.iter().copied().fold(0., f64::max)
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("pass phase root")?);
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("first.json"))?)?;
    let public: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("public.json"))?)?;
    assert_eq!((rows.len(), public.len()), (18, 108));
    let mut max_law = 0f64;
    let mut max_balance = 0f64;
    let mut max_match = 0f64;
    let mut min_clear = f64::INFINITY;
    let mut profiles = BTreeSet::new();
    let mut total = 0;
    for r in &rows {
        let law = &r["physical"];
        let mu = num(&r["mu"]);
        let k = law["first_corner"].as_u64().unwrap() as usize;
        let local = DVec3::new(
            if k / 2 == 0 { -0.18 } else { 0.18 },
            if k % 2 == 0 { -0.10 } else { 0.10 },
            -0.13,
        );
        let before = &law["incoming"];
        let after = &law["outgoing"];
        let arm = q(&before["orientation"]) * local;
        let impulse = v(&law["first_normal_tangent_impulse_ns"]);
        let v0 = v(&before["velocity_m_s"]);
        let w0 = v(&before["spin_rad_s"]);
        let vel = v0 + impulse / 2.;
        let spin = w0 + arm.cross(impulse) / 0.05;
        assert!(
            (vel - v(&after["velocity_m_s"])).length() < 1e-13
                && (spin - v(&after["spin_rad_s"])).length() < 1e-13
        );
        let vp = vel + spin.cross(arm);
        let j = impulse.truncate();
        assert!(impulse.z > 0. && vp.z.abs() < 1e-12 && j.length() < mu * impulse.z + 1e-12);
        let law_err = if law["sticking_at_impact"] == true {
            vp.truncate().length()
        } else {
            (j + mu * impulse.z * vp.truncate().normalize()).length()
        };
        assert!(law_err < 1e-12);
        max_law = max_law.max(law_err);
        let e = vel.length_squared() + 0.025 * spin.length_squared();
        let e0 = v0.length_squared() + 0.025 * w0.length_squared();
        assert!((e0 - e - num(&law["energy_loss_j"])).abs() < 1e-12);
        let speed = e.sqrt() + 9.81 * 0.0005;
        let bound = speed * (1. + (2.0f64 / 0.05).sqrt() * local.length());
        let dt = num(&law["common_cut_ns"]) * 1e-9 - num(&law["time_s"]);
        let clearance = num(&law["other_corner_gap_at_impact_m"]) - bound * dt;
        assert!(
            (clearance - num(&law["other_corner_clearance_bound_m"])).abs() < 1e-14
                && clearance > 0.85e-6
        );
        assert!(num(&law["rk4_repeat"]) < 1e-12);
        min_clear = min_clear.min(clearance);
        let cases = r["native"].as_array().unwrap();
        assert_eq!(cases.len(), 28);
        for c in cases {
            total += 1;
            profiles.insert(c["grid"].as_str().unwrap().to_owned());
            let p = c["native_quadrants_ns"].as_array().unwrap();
            let other = p
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != k)
                .map(|(_, x)| num(x))
                .sum::<f64>();
            assert!((other - num(&c["wrong_corner_normal_ns"])).abs() < 1e-14);
            let balance = (v(&c["delta_h_nms"]) - v(&c["signed_point_moment_nms"])).length();
            assert!((balance - num(&c["angular_balance_nms"])).abs() < 1e-14);
            max_balance = max_balance.max(balance);
            max_match = max_match.max(num(&c["initial_max_component_norm_difference"]));
            assert!(
                ((v(&c["native_point_impulse_ns"]) - v(&law["impulse_ns"])).length()
                    - num(&c["impulse_error_ns"]))
                .abs()
                    < 1e-12
            );
        }
        // Opposite sole orientations also reflect the physical contact reaction.
        if r["case"].as_str().unwrap().starts_with("reflect") {
            let src = rows
                .iter()
                .find(|s| s["mu"] == r["mu"] && s["case"] == "measured")
                .unwrap();
            let sign = DVec3::new(num(&r["reflection"][0]), num(&r["reflection"][1]), 1.);
            assert!(
                (v(&src["physical"]["impulse_ns"]) * sign - v(&law["impulse_ns"])).length() < 1e-11
            );
        }
    }
    assert!(max_balance < 1e-12 && max_match < 1e-12);
    let mut stats = vec![];
    for single in [false, true] {
        for grid in &profiles {
            let group = rows
                .iter()
                .flat_map(|r| r["native"].as_array().unwrap())
                .filter(|c| c["single_rigid"] == single && c["grid"] == grid.as_str())
                .collect::<Vec<_>>();
            assert_eq!(group.len(), 18);
            let stats_for = |key: &str| {
                let vals = group.iter().map(|r| num(&r[key])).collect::<Vec<_>>();
                json!({"median":median(&vals),"max":max(&vals)})
            };
            stats.push(json!({"single_rigid":single,"grid":grid,"impulse_error_ns":stats_for("impulse_error_ns"),"moment_error_nms":stats_for("angular_impulse_error_nms"),"wrong_corner_ns":stats_for("wrong_corner_normal_ns")}));
        }
    }
    for r in &public {
        let s = &r["result"];
        assert!(num(&s["cold_start_difference"]) < 1e-12);
        let e = s["events"].as_array().unwrap();
        assert_eq!(e.len(), usize::from(s["policy"] != "ordinary"));
        if !e.is_empty() {
            assert_eq!(e[0]["tick"], 1531);
        }
        assert_eq!(s["states"].as_array().unwrap().len(), 8);
    }
    let svg = dir.join("first-impulse.svg");
    let surface = SVGBackend::new(&svg, (1120, 650)).into_drawing_area();
    surface.fill(&WHITE)?;
    let halves = surface.split_evenly((1, 2));
    let chosen = [
        ("uniform8", "Uniform 8"),
        ("prospective8", "Prospective 8"),
        ("approach_passes_1", "Approach 1 pass"),
        ("approach_passes_0", "Approach 0 passes"),
        ("shoulder8", "Shorter approach"),
        ("exact_touch_32", "At true touchdown"),
    ];
    for (i, area) in halves.iter().enumerate() {
        let mut chart = ChartBuilder::on(area)
            .margin(17)
            .caption(
                if i == 0 {
                    "Locked two-link foot"
                } else {
                    "Equivalent single rigid foot"
                },
                ("sans-serif", 22),
            )
            .x_label_area_size(100)
            .y_label_area_size(68)
            .build_cartesian_2d(-0.6f64..5.6, (1e-8f64..2.).log_scale())?;
        chart
            .configure_mesh()
            .y_desc("Vector impulse error (N·s)")
            .x_labels(6)
            .x_label_formatter(&|x| {
                let n = x.round() as usize;
                if n < chosen.len() && (x - x.round()).abs() < 0.05 {
                    chosen[n].1.to_owned()
                } else {
                    String::new()
                }
            })
            .x_label_style(
                ("sans-serif", 13)
                    .into_font()
                    .transform(FontTransform::Rotate90),
            )
            .light_line_style(RGBColor(228, 232, 235))
            .draw()?;
        for (col, (id, _)) in chosen.iter().enumerate() {
            for (n, r) in rows.iter().enumerate() {
                let m = r["native"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|m| m["grid"] == *id && m["single_rigid"] == (i == 1))
                    .unwrap();
                let color = if r["mu"] == 0.2 {
                    RGBColor(28, 104, 137)
                } else if r["mu"] == 0.5 {
                    RGBColor(172, 79, 28)
                } else {
                    RGBColor(37, 131, 81)
                };
                chart.draw_series(std::iter::once(Circle::new(
                    (
                        col as f64 + (n as f64 - 8.5) * 0.013,
                        num(&m["impulse_error_ns"]),
                    ),
                    3,
                    color.filled(),
                )))?;
            }
        }
    }
    surface.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"physical_cases":rows.len(),"first_event_controls":total,"public_return_controls":public.len(),"max_start_state_norm_difference":max_match,"max_point_law_residual_ns_or_m_s":max_law,"max_angular_balance_nms":max_balance,"min_other_corner_clearance_m":min_clear,"statistics":stats})
        )?
    );
    Ok(())
}
