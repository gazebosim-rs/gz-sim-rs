// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate both ends of the physically admissible self-stress interval.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|i| f(&x[i])))
}
fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(std::env::args_os().nth(1).ok_or("pass study directory")?);
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(path.join("mixed.json"))?)?;
    let declined: Vec<Value> = serde_json::from_slice(&fs::read(path.join("declines.json"))?)?;
    let mut error = 0f64;
    let mut work_error = 0f64;
    let mut max_width = 0f64;
    let mut beta = 0f64;
    let mut spin_error = 0f64;
    let mut counts = BTreeMap::new();
    let mut worst = [[0f64; 6]; 2];
    let mut plot_data = vec![];
    for r in &rows {
        let mode = r["mode"].as_str().unwrap();
        *counts.entry(mode).or_insert(0) += 1;
        let both = mode == "both-stick";
        let mu = f(&r["mu"]);
        let axis = v(&r["axis"]);
        let t = v(&r["edge_transverse"]);
        let arm = [v(&r["arms_m"][0]), v(&r["arms_m"][1])];
        let before = &r["before"];
        let v0 = v(&before["velocity_m_s"]);
        let w0 = v(&before["spin_rad_s"]);
        let v1 = v(&r["vpost"]);
        let w1 = v(&r["wpost"]);
        let g = &r["identifiable"];
        let n = [f(&g[2]), f(&g[4])];
        let ty = [f(&g[1]), f(&g[3])];
        let gx = f(&g[0]);
        let range = [
            f(&r["first_along_interval_ns"][0]),
            f(&r["first_along_interval_ns"][1]),
        ];
        let width = range[1] - range[0];
        assert!(
            r["singleton_other_upper_m_s"]
                .as_array()
                .unwrap()
                .iter()
                .all(|v| f(v) < 0.)
        );
        let split = |x: f64| {
            [
                axis * x + t * ty[0] + DVec3::Z * n[0],
                axis * (gx - x) + t * ty[1] + DVec3::Z * n[1],
            ]
        };
        let mut span_energy = [0.; 2];
        for (k, x) in range.into_iter().enumerate() {
            let j = split(x);
            let du = (j[0] + j[1]) * 0.5;
            let dw = (arm[0].cross(j[0]) + arm[1].cross(j[1])) / 0.05;
            error = error.max((v0 + du).distance(v1));
            spin_error = spin_error.max((w0 + dw).distance(w1));
            let mut loss = 0.;
            for i in 0..2 {
                assert!(j[i].distance(v(&r["endpoint_splits"][k][i])) < 1e-13);
                let pre = v0 + w0.cross(arm[i]);
                let post = v1 + w1.cross(arm[i]);
                assert!(
                    pre.z < 0.
                        && post.z.abs() < 1e-13
                        && j[i].truncate().length() <= mu * n[i] + 1e-13
                );
                if both || (i == 0) == (mode == "first-sticks") {
                    assert!(post.length() < 1e-12);
                } else {
                    assert!(
                        post.length() > 0.1
                            && post.dot(axis).abs() < 1e-12
                            && (j[i].truncate() + mu * n[i] * post.truncate().normalize()).length()
                                < 1e-12
                    );
                }
                loss -= 0.5 * j[i].dot(pre + post);
            }
            span_energy[k] = loss;
            let kinetic = v0.length_squared() - v1.length_squared()
                + 0.025 * (w0.length_squared() - w1.length_squared());
            work_error = work_error.max((kinetic - loss).abs());
        }
        assert!((span_energy[0] - span_energy[1]).abs() < 1e-12);
        if both {
            let bound = |i: usize| ((mu * n[i] - ty[i].abs()) * (mu * n[i] + ty[i].abs())).sqrt();
            assert!(
                (range[0] - (-bound(0)).max(gx - bound(1))).abs() < 1e-13
                    && (range[1] - bound(0).min(gx + bound(1))).abs() < 1e-13
            );
            if r["roll"] == 0.18 && r["yaw_deg"] == 0. {
                plot_data.push((mu, range));
            }
        } else {
            assert_eq!(width, 0.);
        }
        max_width = max_width.max(width);
        beta = beta.max(f(&r["beta"]));
        let target = r["manufactured_ns"].as_array().unwrap();
        let desired_x = axis.dot(v(&target[0]));
        assert!(desired_x >= range[0] - 1e-13 && desired_x <= range[1] + 1e-13);
        for (i, obs) in r["native"].as_array().unwrap().iter().enumerate() {
            let j = [v(&obs["impulses_ns"][0]), v(&obs["impulses_ns"][1])];
            let zero = split(0.);
            let x = (0.5 * axis.dot((j[0] - zero[0]) - (j[1] - zero[1]))).clamp(range[0], range[1]);
            let nearest = split(x);
            assert!((x - f(&obs["closest_along_ns"])).abs() < 1e-14);
            let e = j[0].distance(nearest[0]).max(j[1].distance(nearest[1]));
            worst[usize::from(!both)][i] = worst[usize::from(!both)][i].max(e);
        }
    }
    assert_eq!((rows.len(), declined.len(), counts.len()), (120, 24, 3));
    assert!(counts.values().all(|n| *n == 40));
    assert!(
        beta < 1. && error < 1e-12 && spin_error < 1e-12 && work_error < 1e-12 && max_width > 0.59
    );
    let mut fallback = 0f64;
    for row in &declined {
        let p = &row["native_fallback"];
        fallback = fallback.max(f(&p["max_state_difference"]));
        for n in p["paired_native_counts"].as_array().unwrap() {
            assert_eq!(n[0], n[1]);
        }
    }
    assert!(fallback < 1e-12);
    let file = path.join("self-stress.svg");
    let area = SVGBackend::new(&file, (920, 430)).into_drawing_area();
    area.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&area)
        .caption(
            "Along-edge reaction at the first of two sticking vertices",
            ("sans-serif", 22),
        )
        .margin(16)
        .x_label_area_size(46)
        .y_label_area_size(75)
        .build_cartesian_2d(0f64..0.71, -0.35f64..0.35)?;
    chart
        .configure_mesh()
        .x_desc("Isotropic friction coefficient")
        .y_desc("Admissible longitudinal impulse (N·s)")
        .light_line_style(RGBColor(231, 235, 238))
        .draw()?;
    for (mu, b) in plot_data {
        chart.draw_series(std::iter::once(PathElement::new(
            [(mu, b[0]), (mu, b[1])],
            RGBColor(44, 119, 139).stroke_width(6),
        )))?;
        chart.draw_series(b.map(|y| Circle::new((mu, y), 5, RGBColor(24, 78, 102).filled())))?;
    }
    area.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"certified_modes":counts,"declined":declined.len(),"native_runs":6*rows.len(),"max_uniqueness_beta":beta,"max_indistinguishable_impulse_span_ns":max_width,"max_ends_velocity_difference_m_s":error,"max_ends_spin_difference_rad_s":spin_error,"max_ends_energy_residual_j":work_error,"native_control_order":["locked1","locked8","locked32","rigid1","rigid8","rigid32"],"max_distance_to_feasible_set_both_and_one_stick_ns":worst,"max_declined_fallback_difference":fallback})
        )?
    );
    Ok(())
}
