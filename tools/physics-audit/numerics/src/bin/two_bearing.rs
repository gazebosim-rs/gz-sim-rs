// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare independently integrated 3-D Newton–Euler and cyclic-coordinate paths.
use glam::{DMat3, DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::new(f(&x[0]), f(&x[1]), f(&x[2]))
}
fn q(x: &Value) -> DQuat {
    DQuat::from_xyzw(f(&x[0]), f(&x[1]), f(&x[2]), f(&x[3]))
}
fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |s: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(root.join(s))?)?)
    };
    let field = read("co-bearing.json")?;
    let survey = read("unload-field.json")?;
    let mut modes = BTreeMap::new();
    let mut errors = [0f64; 8];
    let mut accepted = 0;
    let mut arcs = Vec::new();
    for candidate in field.as_array().unwrap() {
        let mode = candidate["result"]["limit"]
            .as_str()
            .unwrap_or("initially unavailable");
        *modes.entry(mode).or_insert(0usize) += 1;
        if mode != "initially unavailable" {
            accepted += 1;
            arcs.push(candidate);
        }
    }
    let release = survey["releases"].as_array().unwrap();
    arcs.extend(release);
    for case in arcs {
        let r = &case["result"];
        let time = f(&r["time_s"]);
        errors[0] = errors[0].max((time - f(&r["spatial_s"])).abs());
        errors[1] = errors[1].max((time - f(&r["coarse_s"])).abs());
        let i = DMat3::from_cols_array(&std::array::from_fn(|k| f(&case["inertia_body"][k])));
        let samples = r["samples"].as_array().unwrap();
        let mut p0 = None;
        let mut h0 = None;
        let mut e0 = None;
        for s in samples {
            let a = &s["cyclic"];
            let b = &s["spatial"];
            let (x, y, w, att) = (v(&a["c"]), v(&a["v"]), v(&a["w"]), q(&a["q"]));
            errors[2] = errors[2]
                .max(x.distance(v(&b["c"])))
                .max(y.distance(v(&b["v"])))
                .max(w.distance(v(&b["w"])));
            let qb = q(&b["q"]);
            errors[3] =
                errors[3].max(4. * (0.5 * (att - qb).length().min((att + qb).length())).asin());
            for k in 0..2 {
                errors[4] = errors[4]
                    .max((f(&s["cyclic_force_n"][k]) - f(&s["reference_force_n"][k])).abs());
            }
            let energy = y.length_squared()
                + 0.5 * w.dot(att * (i * (att.conjugate() * w)))
                + 2. * 9.81 * x.z;
            let reference = *e0.get_or_insert(energy);
            errors[5] = errors[5].max((energy - reference).abs());
            let h = (att * (i * (att.conjugate() * w))).z;
            let initial = *h0.get_or_insert(h);
            errors[6] = errors[6].max((h - initial).abs());
            let normal = f(&s["signed_impulse_ns"][0]) + f(&s["signed_impulse_ns"][1]);
            let momentum = 2. * (y - *p0.get_or_insert(y) + 9.81 * f(&s["time_s"]) * DVec3::Z)
                - normal * DVec3::Z;
            errors[7] = errors[7].max(momentum.length());
        }
    }
    assert_eq!(accepted, 152);
    assert_eq!(release.len(), 29);
    assert!(errors.iter().all(|u| *u < 5e-8));
    let natives = [read("traces/ordinary.json")?, read("traces/refined.json")?];
    let analytical = read("traces/cyclic.json")?;
    let public = read("public.json")?;
    let physical_end = f(&public["physical_end_s"]);
    assert!(physical_end > 0.0495 && physical_end < 0.05);
    let mut measured = vec![];
    let mut lines = [vec![], vec![], vec![]];
    for label in 0..3 {
        let lane = if label == 2 {
            &analytical
        } else {
            &natives[label]
        };
        let mut impulse = [0f64; 4];
        let mut com = 0.;
        let mut spin = 0.;
        for (r, b) in lane["frames"]
            .as_array()
            .unwrap()
            .iter()
            .zip(analytical["frames"].as_array().unwrap())
        {
            let center = |s: &Value| (v(&s["foot"]["position"]) + v(&s["upper"]["position"])) * 0.5;
            com = center(r).distance(center(b));
            spin = v(&r["foot"]["angular_velocity"]).distance(v(&b["foot"]["angular_velocity"]));
            for (g, z) in impulse.iter_mut().zip(r["corner_ns"].as_array().unwrap()) {
                *g += f(z);
            }
            let t = f(&r["time_us"]) * 1e-3;
            lines[label].push((t, 1000. * (f(&r["corner_ns"][2]) - f(&r["corner_ns"][0]))));
        }
        measured.push(json!({"corner_integrals_ns":impulse,"last_com_error_m":com,"last_spin_error_rad_s":spin}));
    }
    let path = root.join("bearing-reactions.svg");
    let svg = SVGBackend::new(&path, (860, 490)).into_drawing_area();
    svg.fill(&WHITE)?;
    let ymax = lines
        .iter()
        .flatten()
        .map(|(_, v)| v.abs())
        .fold(0., f64::max)
        * 1.08;
    let mut c = ChartBuilder::on(&svg)
        .margin(22)
        .caption(
            "Two vertices share a sustained normal reaction",
            ("sans-serif", 24),
        )
        .x_label_area_size(52)
        .y_label_area_size(86)
        .build_cartesian_2d(0.0..50., -ymax..ymax)?;
    c.configure_mesh()
        .x_desc("Time after co-bearing begins (ms)")
        .y_desc("Vertex 2 − vertex 0 (mN·s per 0.5 ms)")
        .draw()?;
    for (series, (col, name)) in lines.into_iter().zip([
        (RGBColor(188, 93, 32), "One native call"),
        (RGBColor(25, 138, 92), "Eight native calls"),
        (RGBColor(42, 95, 157), "Constrained trajectory"),
    ]) {
        c.draw_series(LineSeries::new(series, col.stroke_width(2)))?
            .label(name)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 18, y)], col.stroke_width(2)));
    }
    c.configure_series_labels()
        .background_style(WHITE.mix(0.88))
        .draw()?;
    svg.present()?;
    let data = json!({"field_candidates":160,"admitted":accepted,"field_modes":modes,"release_survey_candidates":survey["census"].as_array().unwrap().len(),"confirmed_unloadings":release.len(),"max_independent_event_seconds":errors[0],"max_step_quartering_seconds":errors[1],"max_state_component_error":errors[2],"max_orientation_error_rad":errors[3],"max_normal_force_error_n":errors[4],"max_mechanical_energy_drift_j":errors[5],"max_yaw_momentum_drift_nms":errors[6],"max_linear_impulse_balance_ns":errors[7],"replay_lanes":measured,"physical_end_seconds":physical_end});
    fs::write(
        root.join("checks/readback.json"),
        serde_json::to_vec_pretty(&data)?,
    )?;
    println!("{data}");
    Ok(())
}
