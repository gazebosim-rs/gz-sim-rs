// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify the point mechanics independently from the impact/root solver.
use glam::{DQuat, DVec2, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeSet, error::Error, fs, path::PathBuf};

fn f(a: &Value) -> f64 {
    a.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|i| f(&a[i])))
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("pass phase root")?);
    let cases: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("law.json"))?)?;
    let public: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("public.json"))?)?;
    let mut keys = BTreeSet::new();
    let mut stick = 0;
    let mut max_kkt = 0f64;
    let mut max_energy_residual = 0f64;
    let mut max_angular_balance = 0f64;
    let mut angle = [0f64; 5];
    let mut impulse_error = [0f64; 5];
    let mut model_error = [0f64; 5];
    let mut measurements = 0;
    for row in &cases {
        let mu = f(&row["mu"]);
        let yaw = f(&row["yaw_deg"]);
        let slip = row["slip"].as_str().unwrap();
        assert!(keys.insert((mu.to_bits(), yaw.to_bits(), slip)));
        let k = row["corner"].as_u64().unwrap() as usize;
        let r = v(&row["r"]);
        let u = v(&row["relative_incoming_m_s"]);
        let physical = v(&row["physical_j_ns"]);
        let kick = |j: DVec3| j * 0.5 + (r.cross(j) / 0.05).cross(r);
        let post = |j: DVec3| u + kick(j);
        for i in 0..3 {
            assert!(kick(DVec3::AXES[i]).distance(v(&row["inverse_mass"][i])) < 1e-12);
        }
        let base = cases
            .iter()
            .find(|a| a["mu"] == mu && a["slip"] == slip && a["yaw_deg"] == 0.)
            .unwrap();
        assert!(
            (DQuat::from_rotation_z(yaw.to_radians()) * v(&base["physical_j_ns"]))
                .distance(physical)
                < 1e-12
        );
        assert!(physical.distance(v(&row["polar_j_ns"])) < 1e-12);
        let sticking = row["physical_sticking"].as_bool().unwrap();
        stick += usize::from(sticking);
        for (metric, name) in ["physical_j_ns", "diagonal_j_ns", "coupled_j_ns"]
            .into_iter()
            .enumerate()
        {
            let j = v(&row[name]);
            let out = post(j);
            assert!(j.z > 0. && out.z.abs() < 1e-12 && j.truncate().length() <= mu * j.z + 1e-12);
            let kx = kick(DVec3::X);
            let ky = kick(DVec3::Y);
            let t = out.truncate();
            let direction = match metric {
                0 => t,
                1 => t / DVec2::new(kx.x, ky.y),
                _ => {
                    DVec2::new(ky.y * t.x - ky.x * t.y, kx.x * t.y - kx.y * t.x)
                        / (kx.x * ky.y - kx.y * ky.x)
                }
            };
            let residual = if sticking {
                t.length()
            } else {
                (j.truncate() + mu * j.z * direction.normalize()).length()
            };
            assert!(residual < 1e-12);
            max_kkt = max_kkt.max(residual);
        }
        let spent = -0.5 * physical.dot(u + post(physical));
        let kinetic = 0.25 * physical.length_squared() + r.cross(physical).length_squared() / 0.10;
        let energy_residual = (spent + physical.dot(u) + kinetic).abs();
        assert!(spent >= 0. && energy_residual < 1e-12);
        max_energy_residual = max_energy_residual.max(energy_residual);
        let mut seen = BTreeSet::new();
        for obs in row["native"].as_array().unwrap() {
            let single = obs["single_rigid"].as_bool().unwrap();
            let p = obs["passes"].as_u64().unwrap();
            assert!(seen.insert((single, p)));
            let index = match (single, p) {
                (false, 8) => 0,
                (false, 32) => 1,
                (false, 0) => 2,
                (true, 8) => 3,
                (true, 32) => 4,
                _ => panic!("unknown control"),
            };
            assert_eq!(obs["projected"], p == 0);
            let j = v(&obs["impulse_ns"]);
            let out = v(&obs["post_contact_velocity_m_s"]);
            assert!(out.distance(post(j)) < 2e-8);
            let quadrant = &obs["normal_by_corner_ns"];
            for i in 0..4 {
                assert!((f(&quadrant[i]) - if i == k { j.z } else { 0. }).abs() < 1e-12);
            }
            let dh = r.cross(j).distance(v(&obs["net_angular_impulse_nms"]));
            assert!(dh < 1e-13);
            max_angular_balance = max_angular_balance.max(dh);
            let a = out.truncate();
            let b = -j.truncate();
            if !sticking {
                let error = a.perp_dot(b).abs().atan2(a.dot(b)).to_degrees();
                assert!((error - f(&obs["coulomb_slip_misalignment_deg"])).abs() < 1e-12);
                angle[index] = angle[index].max(error);
            }
            impulse_error[index] = impulse_error[index].max(j.distance(physical));
            let key = if p == 0 {
                "physical_j_ns"
            } else if single {
                "coupled_j_ns"
            } else {
                "diagonal_j_ns"
            };
            model_error[index] = model_error[index].max(j.distance(v(&row[key])));
            measurements += 1;
        }
        assert_eq!(seen.len(), 5);
    }
    assert_eq!((keys.len(), measurements, stick), (72, 360, 6));
    assert!(model_error[1] < 6.1e-8 && model_error[2] < 2e-15 && model_error[4] < 1.9e-8);
    let event = &public[2]["events"][0];
    assert_eq!(event["tick"], 5);
    assert_eq!(event["projected"][0]["at_ns"], 187500);
    let path = dir.join("sliding-law.svg");
    let surface = SVGBackend::new(&path, (1080, 510)).into_drawing_area();
    surface.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&surface)
        .margin(20)
        .caption(
            "Oblique slip under a common horizontal rotation",
            ("sans-serif", 24),
        )
        .x_label_area_size(45)
        .y_label_area_size(65)
        .build_cartesian_2d(-2f64..141., -0.5f64..20.)?;
    chart
        .configure_mesh()
        .x_desc("Incoming twist and geometry yaw (degrees)")
        .y_desc("Impulse vs opposing slip (degrees)")
        .light_line_style(RGBColor(232, 236, 239))
        .draw()?;
    for (i, name, color) in [
        (1, "Locked articulation, 32 passes", RGBColor(178, 79, 32)),
        (4, "Single body, 32 passes", RGBColor(52, 113, 159)),
        (2, "Locked articulation, projected", RGBColor(48, 130, 77)),
    ] {
        let data = cases
            .iter()
            .filter(|r| r["mu"] == 0.5 && r["slip"] == "oblique")
            .map(|r| {
                (
                    f(&r["yaw_deg"]),
                    f(&r["native"][i]["coulomb_slip_misalignment_deg"]),
                )
            })
            .collect::<Vec<_>>();
        chart
            .draw_series(LineSeries::new(data.clone(), color.stroke_width(2)))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 24, y)], color.stroke_width(2)));
        chart.draw_series(data.into_iter().map(|p| Circle::new(p, 4, color.filled())))?;
    }
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .border_style(BLACK)
        .draw()?;
    surface.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"solved_inputs":cases.len(),"native_measurements":measurements,"stick":stick,"slide":cases.len()-stick,"max_kkt":max_kkt,"max_energy_balance_j":max_energy_residual,"max_angular_balance_nms":max_angular_balance,"control_order":["locked8","locked32","locked_project","rigid8","rigid32"],"max_sliding_angle_deg":angle,"max_vector_impulse_error_ns":impulse_error,"max_model_impulse_error_ns":model_error})
        )?
    );
    Ok(())
}
