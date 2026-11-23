// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec2;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn main() -> Result<(), Box<dyn Error>> {
    let base = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(base.join(file))?)?)
    };
    let mut counts = BTreeMap::new();
    let mut groups = vec![];
    let mut max_width = 0f64;
    let mut errors = [0f64; 4];
    let mut pts = vec![];
    for name in ["physical", "manufactured"] {
        let cases = read(&format!("{name}.json"))?;
        let mut stats = [[0f64; 4]; 3];
        for case in cases.as_array().unwrap() {
            let r = &case["result"];
            let l = r["mode"].as_str().unwrap();
            *counts.entry(format!("{name}:{l}")).or_insert(0usize) += 1;
            let total = f(&r["resultant_ns"]);
            let cp = DVec2::new(f(&r["center_normalized"][0]), f(&r["center_normalized"][1]));
            max_width = max_width.max(f(&r["null_width_ns"]));
            if let Some(p) = case["prescribed_center"].as_array() {
                errors[0] = errors[0].max(cp.distance(DVec2::new(f(&p[0]), f(&p[1]))));
            }
            if name == "physical" {
                assert!(l.starts_with("Edge("));
                assert!(f(&r["null_width_ns"]) < 1e-12);
            }
            let u = &r["free_m_s"];
            let w = &r["inverse_mass"];
            for e in r["possible_corner_normals_ns"].as_array().unwrap() {
                let j = [0, 1, 2, 3].map(|k| f(&e[k]));
                let t = j.iter().sum::<f64>();
                let center = DVec2::new(j[2] + j[3] - j[0] - j[1], j[1] + j[3] - j[0] - j[2]) / t;
                assert!(j.into_iter().all(|x| x >= -1e-13));
                errors[0] = errors[0].max(cp.distance(center));
                errors[1] = errors[1].max((t - total).abs());
                for k in 0..4 {
                    let v = f(&u[k]) + (0..4).map(|i| f(&w[k][i]) * j[i]).sum::<f64>();
                    errors[2] = errors[2].max((v - f(&r["post_m_s"][k])).abs());
                    errors[3] = errors[3].max((j[k] * v).abs());
                }
            }
            for (idx, n) in r["native"].as_array().unwrap().iter().enumerate() {
                for (k, field) in [
                    "total_error_ns",
                    "pressure_center_normalized_error",
                    "load_on_opening_vertices_ns",
                    "outgoing_w_error_rad_s",
                ]
                .into_iter()
                .enumerate()
                {
                    stats[idx][k] = stats[idx][k].max(f(&n[field]));
                }
            }
            pts.push((cp, name == "physical"));
        }
        groups.push(json!({"cohort":name,"scenes":cases.as_array().unwrap().len(),"native_maxima_in_pass_order_1_8_32":stats}));
    }
    assert!(max_width > 0.7 && errors.iter().all(|x| *x < 1e-12));
    assert_eq!(pts.iter().filter(|p| p.1).count(), 42);
    let p = base.join("pressure-centers.svg");
    let svg = SVGBackend::new(&p, (820, 535)).into_drawing_area();
    svg.fill(&WHITE)?;
    let mut c = ChartBuilder::on(&svg)
        .margin(22)
        .caption(
            "Normal impulse centers across the rectangular sole",
            ("sans-serif", 23),
        )
        .x_label_area_size(54)
        .y_label_area_size(75)
        .build_cartesian_2d(-1.12..1.12, -1.14..1.14)?;
    c.configure_mesh()
        .x_desc("Sole x / half-length")
        .y_desc("Sole y / half-width")
        .draw()?;
    c.draw_series(std::iter::once(Rectangle::new(
        [(-1., -1.), (1., 1.)],
        BLACK.mix(0.75).stroke_width(2),
    )))?;
    c.draw_series(
        pts.iter()
            .filter(|(_, n)| !*n)
            .map(|(p, _)| Circle::new((p.x, p.y), 5, RGBColor(22, 129, 112).filled())),
    )?
    .label("Designed control")
    .legend(|(x, y)| Circle::new((x + 9, y), 5, RGBColor(22, 129, 112).filled()));
    c.draw_series(
        pts.iter()
            .filter(|(_, n)| *n)
            .map(|(p, _)| TriangleMarker::new((p.x, p.y), 8, RGBColor(190, 83, 29).filled())),
    )?
    .label("Rocking return")
    .legend(|(x, y)| TriangleMarker::new((x + 9, y), 7, RGBColor(190, 83, 29).filled()));
    c.configure_series_labels()
        .background_style(WHITE.mix(0.86))
        .draw()?;
    svg.present()?;
    let public = read("public.json")?;
    for r in public["lanes"].as_array().unwrap() {
        let label = r["label"].as_str().unwrap();
        let lane = read(&format!("traces/{label}.json"))?;
        assert_eq!(lane["frames"][1]["normal_impulse_ns"], r["first_tick_ns"]);
        assert!(
            lane["frames"]
                .as_array()
                .unwrap()
                .iter()
                .all(|x| x.get("corner_ns").is_none())
        );
    }
    let result = json!({"modes":counts,"cohorts":groups,"native_stat_columns":["total_error_ns","normalized_cop_error","positive_load_on_opening_vertices_ns","angular_speed_error_rad_s"],"maximum_equivalent_diagonal_shift_width_ns":max_width,"max_recomputed_cop_error":errors[0],"max_recomputed_total_error_ns":errors[1],"max_recomputed_port_error_m_s":errors[2],"max_complementarity_residual_j":errors[3],"replay":public});
    fs::write(
        base.join("checks/readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{result}");
    Ok(())
}
