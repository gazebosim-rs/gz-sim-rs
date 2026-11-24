// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct two-point reactions and separate trajectory, contact and clock timing.
use glam::{DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(a: &Value) -> f64 {
    a.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::new(f(&a[0]), f(&a[1]), f(&a[2]))
}
fn quantile(s: &[f64], p: f64) -> f64 {
    let mut c = s.to_vec();
    c.sort_by(f64::total_cmp);
    c[(p * c.len() as f64).ceil() as usize - 1]
}
fn stats(s: &[f64]) -> Value {
    let a: Vec<_> = s.iter().map(|d| d.abs()).collect();
    json!({"count":s.len(),"min":quantile(s,1./s.len()as f64),"median":quantile(s,0.5),"p95_abs":quantile(&a,0.95),"max":quantile(s,1.)})
}
fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(root.join(name))?)?)
    };
    let rows = read("events.json")?;
    let control = read("planar.json")?;
    assert!(f(&control["time_error_s"]) < 1e-12);
    let mut original = 0;
    let mut both = 0;
    let mut both_force = 0;
    let mut error = 0f64;
    let mut max_energy = 0f64;
    let mut normal_error = 0f64;
    let mut work_error = 0f64;
    let mut timing: [Vec<f64>; 3] = Default::default();
    let mut shadow: [Vec<f64>; 3] = Default::default();
    let mut sensor: [Vec<f64>; 3] = Default::default();
    let mut quant: [Vec<f64>; 3] = Default::default();
    let mut max_restore = [[0f64; 2]; 3];
    let mut plots = vec![];
    let corners = [(-0.18, -0.1), (-0.18, 0.1), (0.18, -0.1), (0.18, 0.1)]
        .map(|(x, y)| DVec3::new(x, y, -0.13));
    for row in rows.as_array().unwrap() {
        assert_eq!(row["initially_certified"], true);
        let r = &row["result"];
        let j = &r["jump"];
        let i = j["old"].as_u64().unwrap() as usize;
        let k = j["new"].as_u64().unwrap() as usize;
        original += usize::from(r["initially_opening"].as_u64().unwrap() as usize == k);
        both += usize::from(j["old_stays_loaded"].as_bool().unwrap());
        both_force += usize::from(j["both_active_immediately_after"].as_bool().unwrap());
        let pre = &r["state"];
        let q = DQuat::from_array(std::array::from_fn(|k| f(&pre["q"][k])));
        let arm = [q * corners[i], q * corners[k]];
        let n = DVec3::Z;
        let v0 = v(&pre["v"]);
        let w0 = v(&pre["w"]);
        let c = v(&pre["c"]);
        let loads = [f(&j["j_ns"][0]), f(&j["j_ns"][1])];
        assert!(loads[0] >= 0. && loads[1] > 0.);
        let v1 = v0 + n * (0.5 * (loads[0] + loads[1]));
        let w1 = w0 + 20. * (arm[0].cross(n) * loads[0] + arm[1].cross(n) * loads[1]);
        let after = &j["after"];
        assert!(v1.distance(v(&after["v"])) < 1e-12 && w1.distance(v(&after["w"])) < 1e-12);
        let mut work = 0.;
        for a in 0..2 {
            assert!((c + arm[a]).z.abs() < 1e-12);
            let before = (v0 + w0.cross(arm[a])).z;
            let post = (v1 + w1.cross(arm[a])).z;
            work -= 0.5 * loads[a] * (before + post);
            assert!(post >= -1e-12);
            normal_error = normal_error.max((post * loads[a]).abs());
        }
        let k0 = v0.length_squared() + 0.025 * w0.length_squared();
        let k1 = v1.length_squared() + 0.025 * w1.length_squared();
        work_error = work_error
            .max((work - (k0 - k1)).abs())
            .max((work - f(&j["dissipated_j"])).abs());
        error = error.max(f(&row["coarse_fine_event_error_s"]));
        max_energy = max_energy.max(f(&r["max_energy_j"]));
        let event = f(&r["time_s"]) * 1e6;
        for (lane, b) in r["native"].as_array().unwrap().iter().enumerate() {
            let h = 500. / [1., 8., 32.][lane];
            let loaded = &b["first_after_clearance"];
            let obs = f(&loaded["time_ns"]) * 1e-3;
            assert_eq!(loaded["site"], k);
            let sh = &b["matched_bearing_shadow"];
            assert_eq!(sh["end"], r["end"]);
            let physical = f(&sh["next_event_ns"]) * 1e-3;
            let traj = physical - event;
            let native = obs - physical;
            let enclosure = (event / h).ceil() * h;
            timing[lane].push(obs - event);
            shadow[lane].push(traj);
            sensor[lane].push(native);
            quant[lane].push(obs - enclosure);
            max_restore[lane][0] = max_restore[lane][0].max(f(&sh["position_correction_m"]).abs());
            max_restore[lane][1] = max_restore[lane][1].max(f(&sh["speed_correction_m_s"]).abs());
            if lane == 1 {
                plots.push((event * 1e-3, traj, native));
            }
        }
    }
    assert_eq!(rows.as_array().unwrap().len(), 96);
    assert_eq!(original, 32);
    assert_eq!(both, 32);
    assert!(error < 1e-8 && max_energy < 1e-9 && normal_error < 1e-12 && work_error < 1e-12);
    let pubdata = read("public.json")?;
    let demo = &pubdata[2];
    let out = root.join("onset.svg");
    let plot = SVGBackend::new(&out, (840, 390)).into_drawing_area();
    plot.fill(&WHITE)?;
    let scale = plots
        .iter()
        .map(|x| x.1.abs().max(x.2.abs()))
        .fold(1., f64::max)
        * 1.1;
    let mut c = ChartBuilder::on(&plot)
        .margin(20)
        .caption(
            "Eight-call recollision timing from a matched bearing cut",
            ("sans-serif", 22),
        )
        .x_label_area_size(40)
        .y_label_area_size(64)
        .build_cartesian_2d(
            0f64..plots.iter().map(|x| x.0).fold(1., f64::max) * 1.08,
            -scale..scale,
        )?;
    c.configure_mesh()
        .x_desc("Physical recollision time (ms)")
        .y_desc("Signed time error (µs)")
        .draw()?;
    for (idx, (label, color)) in [
        ("Trajectory at cut", RGBColor(38, 98, 151)),
        ("Remaining native onset", RGBColor(190, 87, 33)),
    ]
    .into_iter()
    .enumerate()
    {
        c.draw_series(
            plots
                .iter()
                .map(|v| Circle::new((v.0, if idx == 0 { v.1 } else { v.2 }), 3, color.filled())),
        )?
        .label(label)
        .legend(move |(x, y)| Circle::new((x + 6, y), 4, color.filled()));
    }
    c.configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .draw()?;
    plot.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"fields":96,"original_vertex_returns":original,"other_vertex_returns":96-original,"old_site_takes_second_jump":both,"both_sustained_forces_positive":both_force,"max_step_quartering_shift_s":error,"max_smooth_energy_error_j":max_energy,"max_impulse_complementarity_residual":normal_error,"max_recollision_energy_residual_j":work_error,"native_1_8_32_signed_onset_us":timing.map(|v|stats(&v)),"matched_cut_trajectory_us":shadow.map(|v|stats(&v)),"post_cut_observed_minus_shadow_us":sensor.map(|v|stats(&v)),"observed_minus_first_enclosing_native_boundary_us":quant.map(|v|stats(&v)),"max_vertical_manifold_restorations_m_m_s":max_restore,"quadrature_control":control,"demo_event_s":demo["event_s"],"demo_second_jump":demo["instant_jump_ns"],"demo_public_work_residual_ns":demo["max_public_momentum_residual_ns"]})
        )?
    );
    Ok(())
}
