// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fmt::Write, fs, path::PathBuf};
fn num(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn chart(out: &std::path::Path, data: &[Value]) -> Result<(), Box<dyn Error>> {
    let width = 960.;
    let left = 64.;
    let right = 24.;
    let tmax = data[0]["ticks"].as_array().unwrap().len() as f64 - 1.;
    let x = |i: usize| left + (width - left - right) * (i as f64 / tmax);
    let y = |f: f64| 225. - 160. * f.clamp(0., 1.);
    let mut svg = String::from(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="960" height="304" viewBox="0 0 960 304"><rect width="960" height="304" fill="white"/><g font-family="sans-serif" fill="#182c38"><text x="64" y="26" font-size="20">Share of normal reaction carried by the heel pad</text>"##,
    );
    let ticks = data[0]["ticks"].as_array().unwrap();
    let valid = |i: usize, k: usize| num(&ticks[i]["analytical_bounds_ns"][0][k]) / 0.00981;
    let poly = (1..ticks.len())
        .map(|i| (i, 0))
        .chain((1..ticks.len()).rev().map(|i| (i, 1)))
        .map(|(i, k)| format!("{:.2},{:.2}", x(i), y(valid(i, k))))
        .collect::<Vec<_>>()
        .join(" ");
    write!(svg, r##"<polygon points="{poly}" fill="#d4e8d4"/>"##)?;
    for (l, label) in [(0., "0%"), (0.5, "50%"), (1., "100%")] {
        write!(
            svg,
            r##"<line x1="64" y1="{0}" x2="936" y2="{0}" stroke="#d4dce1"/><text x="54" y="{1}" text-anchor="end" font-size="12">{label}</text>"##,
            y(l),
            y(l) + 4.
        )?;
    }
    for v in (0..=tmax as usize).step_by(200) {
        write!(
            svg,
            r#"<text x="{:.1}" y="246" text-anchor="middle" font-size="12">{} ms</text>"#,
            x(v),
            v / 2
        )?;
    }
    let colors = ["#c36520", "#79518f", "#126f94"];
    let mut xpos = 235.;
    for (i, lane) in data.iter().enumerate() {
        let rows = lane["ticks"].as_array().unwrap();
        let p = (1..rows.len())
            .map(|k| {
                let j0 = num(&rows[k]["impulses_ns"][0]);
                let j1 = num(&rows[k]["impulses_ns"][1]);
                format!(
                    "{:.2},{:.2}",
                    x(k),
                    y(if j0 + j1 > 0. { j0 / (j0 + j1) } else { 0. })
                )
            })
            .collect::<Vec<_>>()
            .join(" ");
        write!(
            svg,
            r#"<polyline points="{p}" fill="none" stroke="{}" stroke-width="1.5"/>"#,
            colors[i]
        )?;
        let label = match lane["mode"].as_str().unwrap() {
            "ordinary" => "1 native solve",
            "eight" => "8 native solves",
            "shared" => "Registered",
            "thirtytwo" => "32 native solves",
            _ => unreachable!(),
        };
        write!(
            svg,
            r#"<line x1="{xpos}" y1="277" x2="{}" y2="277" stroke="{}" stroke-width="3"/><text x="{}" y="281" font-size="13">{label}</text>"#,
            xpos + 17.,
            colors[i],
            xpos + 21.
        )?;
        xpos += 170.;
    }
    svg.push_str(r##"<rect x="65" y="270" width="18" height="11" fill="#d4e8d4"/><text x="89" y="281" font-size="13">Feasible heel share</text></g></svg>"##);
    fs::write(out.join("pressure-shares.svg"), svg)?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let get = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(out.join(name))?)?)
    };
    let f = get("field.json")?;
    let mut statuses = [0usize; 5];
    let mut err = 0f64;
    for row in f.as_array().unwrap() {
        if row["fraction_toe"].is_null() {
            statuses[0] += 1;
            assert!(row["linear_program"].is_null());
            continue;
        }
        let range = [num(&row["fraction_toe"][0]), num(&row["fraction_toe"][1])];
        for (k, v) in range.into_iter().enumerate() {
            err = err.max((v - num(&row["linear_program"][k])).abs());
        }
        let mode = match (range[0] > 1e-12, range[1] < 1. - 1e-12) {
            (true, true) => 1,
            (true, false) => 2,
            (false, true) => 3,
            (false, false) => 4,
        };
        statuses[mode] += 1;
    }
    assert_eq!(f.as_array().unwrap().len(), 1440);
    assert!(err < 2e-10);
    let data = get("loads.json")?;
    let mut panels = vec![];
    for lane in data.as_array().unwrap() {
        let mut last = "";
        let mut transitions = vec![];
        let mut loss = vec![];
        let mut max_com = 0f64;
        let mut max_spin = 0f64;
        let mut max_total = 0f64;
        let mut max_outside = 0f64;
        let mut separated_ns = 0.;
        let mut gap = 0f64;
        let mut shared = 0;
        let mut max_cap_error = 0f64;
        let mut selected_witness_error = 0f64;
        let mut intervals = Vec::new();
        let mut in_loss = false;
        for row in lane["ticks"].as_array().unwrap() {
            let k = row["tick"].as_u64().unwrap();
            let kind = row["status"].as_str().unwrap();
            if kind != last {
                transitions.push(json!([k, kind]));
                last = kind;
            }
            if k == 0 {
                continue;
            }
            let j = [num(&row["impulses_ns"][0]), num(&row["impulses_ns"][1])];
            let bounds = |s: usize, i: usize| num(&row["analytical_bounds_ns"][s][i]);
            assert!(j.iter().all(|v| *v >= 0.));
            max_total = max_total.max((j.iter().sum::<f64>() - 0.00981).abs());
            for (i, &v) in j.iter().enumerate() {
                max_outside = max_outside.max((bounds(i, 0) - v).max(v - bounds(i, 1)));
            }
            max_com = max_com.max(num(&row["com_error_m"]));
            max_spin = max_spin.max(num(&row["spin_rad_s"]));
            separated_ns += row["positive_separated_native_ns"].as_f64().unwrap_or(0.);
            gap = gap.max(row["separated_native_gap_m"].as_f64().unwrap_or(0.));
            let missing = bounds(0, 1) < 1e-15;
            if missing && !in_loss {
                loss.push(json!([k, j]));
            }
            if missing && j[0] > 1e-10 {
                intervals.push(k);
            }
            in_loss = missing;
            if let Some(g) = row["registered"].as_object() {
                shared += 1;
                for (i, &v) in j.iter().enumerate() {
                    let low = num(&g["enclosing_ns"][i][0]);
                    let high = num(&g["enclosing_ns"][i][1]);
                    max_cap_error = max_cap_error.max((low - v).max(v - high));
                    selected_witness_error =
                        selected_witness_error.max((v - num(&g["selected_ns"][i])).abs());
                }
            }
        }
        if shared > 0 {
            assert!(
                max_com < 1e-10
                    && max_spin < 1e-10
                    && max_total < 1e-11
                    && max_cap_error < 1e-12
                    && selected_witness_error < 1e-12
                    && max_outside < 1e-10
            );
            assert_eq!(intervals.len(), 0);
        }
        panels.push(json!({"mode":lane["mode"],"tick_transitions":transitions,"first_heel_absent_reactions_ns":loss,"positive_heel_after_physical_loss_ticks":intervals,"maximum_total_error_ns":max_total,"maximum_outside_physical_per_site_bounds_ns":max_outside,"max_COM_error_m":max_com,"max_spin_rad_s":max_spin,"measured_positive_impulse_with_positive_x_gap_both_native_ends_ns":separated_ns,"max_such_min_endpoint_gap_m":gap,"registered_public_ticks":shared,"maximum_selected_vs_receipt_error_ns":selected_witness_error}));
    }
    let summary = json!({"fixtures":1440,"no_equilibrium_both_required_toe_required_heel_required_neither_required":statuses,"max_load_fraction_error_vs_vertex_LP":err,"public_trajectories":panels});
    fs::write(
        out.join("checks/readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    chart(&out, data.as_array().unwrap())?;
    println!("{summary}");
    Ok(())
}
