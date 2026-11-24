// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rebuild native site loads against the separate continuous-arc publication.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let p = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |name: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(p.join(name))?)?)
    };
    let cases = read("returns.json")?;
    let previous = read("../multifidelity-rocking-recontact/events.json")?;
    let mut errors = [0f64; 7];
    let mut counts = [0; 2];
    let mut plot = vec![];
    let mut force = 0f64;
    for (case, prior) in cases
        .as_array()
        .unwrap()
        .iter()
        .zip(previous.as_array().unwrap())
    {
        for k in ["pitch", "opening", "pitch_rate", "yaw"] {
            assert_eq!(case[k], prior[k]);
        }
        let r = &prior["result"];
        let j = &r["jump"];
        let both = j["old_stays_loaded"].as_bool().unwrap();
        counts[usize::from(both)] += 1;
        assert!((f(&case["time_s"]) - f(&r["time_s"])).abs() < 1e-12);
        assert_eq!(case["old"], j["old"]);
        assert_eq!(case["new"], j["new"]);
        let reference = [f(&j["j_ns"][0]), f(&j["j_ns"][1])];
        assert!(reference[1] > 0. && (reference[0] > 0.) == both);
        let loads = case["force"]["following_n"].as_array().unwrap();
        assert_eq!(loads.len(), if both { 2 } else { 1 });
        for (k, load) in loads.iter().enumerate() {
            assert!(f(load) > 0.);
            force = force.max((f(load) - f(&j["following_bearing_forces_n"][k])).abs());
        }
        for row in case["native"].as_array().unwrap() {
            for (i, j) in reference.into_iter().enumerate() {
                errors[0] = errors[0].max((f(&row["normal_old_new_ns"][i]) - j).abs());
            }
            for (i, key) in [
                "point_error_m",
                "speed_error_m_s",
                "linear_error_ns",
                "angular_error_nms",
                "loss_error_j",
                "work_error_j",
            ]
            .into_iter()
            .enumerate()
            {
                errors[i + 1] = errors[i + 1].max(f(&row[key]));
            }
            assert!(
                f(&row["old_before_m_s"]).abs() < 1e-12
                    && f(&row["new_before_m_s"]) < -1e-3
                    && f(&row["new_after_m_s"]).abs() < 1e-12
            );
            assert!((f(&row["old_after_m_s"]) > 1e-8) != both);
        }
        plot.push((reference[0], reference[1], both));
    }
    assert_eq!(counts, [64, 32]);
    assert!(errors.iter().all(|e| *e < 1e-11) && force < 1e-11);
    let path = p.join("site-loads.svg");
    let svg = SVGBackend::new(&path, (830, 480)).into_drawing_area();
    svg.fill(&WHITE)?;
    let mut c = ChartBuilder::on(&svg)
        .margin(22)
        .caption(
            "A second vertex reaches a moving support",
            ("sans-serif", 24),
        )
        .x_label_area_size(52)
        .y_label_area_size(74)
        .build_cartesian_2d(-0.004..0.055, -0.04..1.36)?;
    c.configure_mesh()
        .x_desc("Impulse at previous bearing (N·s)")
        .y_desc("Impulse at arriving vertex (N·s)")
        .draw()?;
    c.draw_series(plot.iter().map(|(x, y, b)| {
        Circle::new(
            (*x, *y),
            4,
            if *b {
                RGBColor(4, 116, 97)
            } else {
                RGBColor(196, 94, 31)
            }
            .filled(),
        )
    }))?;
    svg.present()?;
    let public = read("public.json")?;
    let e = &public[2]["events"][0];
    assert_eq!(e["at_ns"], 0);
    assert_eq!(e["tick"], 1);
    for (id, label) in ["ordinary", "refined", "edge"].into_iter().enumerate() {
        let v = read(&format!("traces/{label}.json"))?;
        let frames = v["frames"].as_array().unwrap();
        assert_eq!(frames.len(), 27);
        for r in frames {
            let total = r["corner_ns"]
                .as_array()
                .unwrap()
                .iter()
                .map(f)
                .sum::<f64>();
            assert!((total - f(&r["normal_impulse_ns"])).abs() < 1e-12);
        }
        assert_eq!(
            frames[1]["normal_impulse_ns"],
            public[id]["first_tick_reaction_ns"]
        );
    }
    let a = json!({"physical_donors":96,"native_and_authored_pairs":192,"single_transfers":counts[0],"two_bearing":counts[1],"max_endpoint_error_ns":errors[0],"max_site_error_m":errors[1],"max_twist_error_m_s":errors[2],"max_linear_balance_ns":errors[3],"max_angular_balance_nms":errors[4],"max_dissipation_residual_j":errors[5],"max_work_residual_j":errors[6],"max_force_error_n":force,"public_first":e});
    fs::write(
        p.join("checks/readback.json"),
        serde_json::to_vec_pretty(&a)?,
    )?;
    println!("{a}");
    Ok(())
}
