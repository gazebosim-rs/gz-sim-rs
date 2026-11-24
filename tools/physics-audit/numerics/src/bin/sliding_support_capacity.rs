// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fmt::Write, fs, path::PathBuf};
fn n(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |f: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(dir.join(f))?)?)
    };
    let field = read("sliding.json")?;
    let mut ok = 0;
    let mut nozero = 0;
    let mut err = [0f64; 5];
    let mut spread = [0f64; 2];
    for row in field.as_array().unwrap() {
        let r = &row["result"];
        if r["mode"] != "feasible" {
            continue;
        }
        ok += 1;
        if r["zero_yaw_fraction"].is_null() {
            nozero += 1;
        }
        for (v, raw) in err.iter_mut().zip(
            r["errors_zero_slice_torque_power_dual_hull_wrench"]
                .as_array()
                .unwrap(),
        ) {
            *v = v.max(n(raw));
        }
        spread[0] = spread[0].max(n(&r["acceleration_diameter_m_s2"]));
        spread[1] = spread[1].max(n(&r["yaw_range"][1]) - n(&r["yaw_range"][0]));
    }
    assert_eq!(ok, 320);
    assert_eq!(nozero, 16);
    assert!(err.iter().all(|v| *v < 2e-9));
    let public = read("public.json")?;
    let p = &public["physical_initial"];
    let to_x = |f: f64| 80. + 650. * f;
    let yr = p["yaw_range"].as_array().unwrap();
    let mid = n(&yr[0]);
    let range = n(&yr[1]) - mid;
    let to_y = |y: f64| 290. - 230. * (y - mid) / range;
    let shape = p["hull"]
        .as_array()
        .unwrap()
        .iter()
        .map(|pt| format!("{:.3},{:.3}", to_x(n(&pt[0])), to_y(n(&pt[1]))))
        .collect::<Vec<_>>()
        .join(" ");
    let mut svg = String::from(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="800" height="383" viewBox="0 0 800 383"><rect width="800" height="383" fill="white"/><g font-family="sans-serif" fill="#243640"><text x="64" y="27" font-size="20">Physically admissible initial sliding responses</text><text x="64" y="48" font-size="13">Yaw acceleration (rad/s²)</text>"##,
    );
    write!(
        svg,
        r##"<polygon points="{shape}" fill="#c5dfde" stroke="#247469" stroke-width="2"/>"##
    )?;
    for y in [mid, 0., n(&yr[1])] {
        write!(
            svg,
            r##"<line x1="74" x2="741" y1="{0}" y2="{0}" stroke="#d0d9de" stroke-dasharray="3 4"/><text x="68" y="{1}" text-anchor="end" font-size="12">{y:.1}</text>"##,
            to_y(y),
            to_y(y) + 4.
        )?;
    }
    for x in [0., 0.25, 0.50, 0.75, 1.] {
        write!(
            svg,
            r#"<text x="{:.0}" y="313" text-anchor="middle" font-size="12">{:.0}%</text>"#,
            to_x(x),
            100. * x
        )?;
    }
    let zero = &p["zero_yaw_fraction"];
    write!(
        svg,
        r##"<line x1="{}" x2="{}" y1="{2}" y2="{2}" stroke="#21564e" stroke-width="4"/>"##,
        to_x(n(&zero[0])),
        to_x(n(&zero[1])),
        to_y(0.)
    )?;
    let colors = ["#c36520", "#79518f", "#126f94"];
    let mut observed = vec![];
    for (i, lane) in public["native"].as_array().unwrap().iter().enumerate() {
        let t = &lane["ticks"][1];
        let j = &t["impulses_ns"];
        let f = n(&j[1]) / (n(&j[0]) + n(&j[1]));
        let a = n(&t["spin"][2]) / 0.0005;
        observed.push(json!([lane["mode"], f, a]));
        write!(
            svg,
            r#"<circle cx="{}" cy="{}" r="4.5" fill="{}"/>"#,
            to_x(f),
            to_y(a),
            colors[i]
        )?;
    }
    svg.push_str(r#"<text x="410" y="338" text-anchor="middle" font-size="14">Toe share of normal load</text><text x="80" y="364" font-size="12">Dots show 500 µs native averages (orange 1 solve, purple 8, blue 32). Green line shows zero yaw.</text></g></svg>"#);
    fs::write(dir.join("initial-acceleration.svg"), svg)?;
    let summary = json!({"fixtures":ok,"free_yaw_feasible_without_zero_yaw":nozero,"maximum_planar_acceleration_and_yaw_acceleration_diameters":spread,"max_errors_zero_fraction_torque_energy_dual_wrench":err,"replay_initial_fractions":p["fraction_range"],"replay_initial_yaw":p["yaw_range"],"first_500us_native_fraction_yaw_average":observed});
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
