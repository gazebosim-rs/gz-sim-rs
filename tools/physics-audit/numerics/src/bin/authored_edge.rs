// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check Bevy reaction ownership, integer event clocks and transformed two-site Coulomb laws.
use glam::{DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(a: &Value) -> f64 {
    a.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|i| f(&a[i])))
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |n: &str| -> Result<Vec<Value>, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(
            dir.join(format!("{n}.json")),
        )?)?)
    };
    let points = read("tangent")?;
    let flights = read("flights")?;
    let public = read("public")?;
    let mut admitted = 0;
    let mut terr = 0f64;
    for p in &points {
        let r = &p["result"];
        if r["admitted"] == true {
            admitted += 1;
            assert_eq!(r["native_calls"], 8);
            assert_eq!(r["first_positive_ns"], 0);
            terr = terr.max(f(&r["site_error_ns"]));
        }
    }
    let mut clock = 0f64;
    let mut ferr = 0f64;
    let mut coulomb = 0f64;
    let mut max_angle = 0f64;
    let mut min_angle = f64::INFINITY;
    let mut plot_data = vec![];
    for p in &flights {
        let r = &p["result"];
        let at = r["actual_ns"].as_u64().unwrap();
        let tick = r["tick"].as_u64().unwrap();
        assert_eq!(r["completed_public_ns"], tick * 500000);
        assert_eq!(r["local_ns"], at - (tick - 1) * 500000);
        assert_eq!(r["preceding_reaction_ns"], 0.);
        let err = at as f64 - f(&r["known_ns"]);
        clock = clock.max(err.abs());
        let normal = DQuat::from_array(std::array::from_fn(|i| f(&p["frame"][i]))) * DVec3::Z;
        let mut dirs = [DVec3::ZERO; 2];
        for (i, d) in dirs.iter_mut().enumerate() {
            let j = v(&r["impulses_ns"][i]);
            let u = v(&r["outgoing_point_velocity"][i]);
            let n = j.dot(normal);
            let tangent = j - n * normal;
            assert!(n > 0. && u.dot(normal).abs() < 1e-12);
            *d = u.normalize();
            coulomb = coulomb.max((tangent + f(&p["mu"]) * n * *d).length());
            ferr = ferr.max(f(&r["errors_ns"][i]));
        }
        let angle = dirs[0].angle_between(dirs[1]).to_degrees();
        min_angle = min_angle.min(angle);
        max_angle = max_angle.max(angle);
        plot_data.push((f(&r["known_ns"]) * 1e-3, err));
    }
    assert_eq!((points.len(), admitted, flights.len()), (384, 276, 128));
    assert!(clock <= 1.001 && coulomb < 1e-12 && terr < 1e-12 && ferr < 1e-12 && min_angle > 0.1);
    let exact = &public[2]["events"][0];
    let first = exact["jump"].as_array().unwrap();
    assert_eq!(first.len(), 2);
    assert_eq!(first[0]["at_ns"], 138197);
    assert_eq!(first[1]["at_ns"], 138197);
    let trace: Value = serde_json::from_slice(&fs::read(dir.join("traces/coupled.json"))?)?;
    let row = &trace["frames"][5];
    let jn = first.iter().map(|s| v(&s["j"]).z).sum::<f64>();
    let total = f(&row["normal_impulse_ns"]);
    assert!(total > jn && total > f(&exact["native_last_ns"]));
    assert!((total - f(&exact["interval_normal_ns"])).abs() < 1e-12);
    let path = dir.join("phase-clock.svg");
    let a = SVGBackend::new(&path, (1000, 420)).into_drawing_area();
    a.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&a)
        .caption(
            "Native event time minus independently prescribed edge arrival",
            ("sans-serif", 22),
        )
        .margin(15)
        .x_label_area_size(48)
        .y_label_area_size(65)
        .build_cartesian_2d((0.0001f64..4000.).log_scale(), -1f64..1.)?;
    chart
        .configure_mesh()
        .x_desc("Physical arrival after boundary (µs, log scale)")
        .y_desc("Clock error (ns)")
        .light_line_style(RGBColor(227, 232, 237))
        .draw()?;
    chart.draw_series(
        plot_data
            .into_iter()
            .map(|p| Circle::new(p, 4, RGBColor(35, 111, 149).filled())),
    )?;
    a.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"exact_tangencies":points.len(),"admitted":admitted,"declined":points.len()-admitted,"fractional_flights":flights.len(),"max_clock_error_ns":clock,"max_tangent_impulse_error_ns":terr,"max_flight_impulse_error_ns":ferr,"max_coulomb_residual_ns":coulomb,"slip_direction_separation_deg":[min_angle,max_angle],"replay_event_ns":2138197,"instantaneous_normal_ns":jn,"public_interval_normal_ns":total,"last_solve_normal_ns":exact["native_last_ns"]})
        )?
    );
    Ok(())
}
