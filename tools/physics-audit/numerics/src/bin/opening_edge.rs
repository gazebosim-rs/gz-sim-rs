// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct the separating hyperplane, work and Coulomb reaction and audit both native bodies.
use glam::{DVec2, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
fn f(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn v(x: &Value) -> DVec3 {
    DVec3::new(f(&x[0]), f(&x[1]), f(&x[2]))
}
fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let read = |file: &str| -> Result<Value, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(path.join(file))?)?)
    };
    let data = read("opening.json")?;
    let mut modes = BTreeMap::new();
    let mut declines = BTreeMap::new();
    let mut max = [[0f64; 3]; 6];
    let mut certified = 0;
    let mut normal_error = 0f64;
    let mut min_margin = f64::INFINITY;
    let mut polar_error = 0f64;
    let mut maxloss = 0f64;
    for x in data.as_array().unwrap() {
        let r = &x["result"];
        if r["admitted"] != true {
            *declines.entry(r["reason"].as_str().unwrap()).or_insert(0) += 1;
            continue;
        }
        certified += 1;
        let active = x["active"].as_u64().unwrap() as usize;
        let other = 1 - active;
        let mu = f(&x["mu"]);
        let stick = x["sticking"].as_bool().unwrap();
        *modes.entry((active, stick)).or_insert(0) += 1;
        let m = |i, j: usize, a: DVec3| {
            0.5 * a + (20. * v(&x["arms_m"][j]).cross(a)).cross(v(&x["arms_m"][i]))
        };
        let pre = [v(&x["pre_m_s"][0]), v(&x["pre_m_s"][1])];
        let j = v(&r["physical_j_ns"]);
        let closing = DVec2::new(-pre[active].z, -pre[other].z).normalize();
        for k in 0..2 {
            let row = DVec3::from_array(
                [DVec3::X, DVec3::Y, DVec3::Z]
                    .map(|d| closing.perp_dot(DVec2::new(m(active, k, d).z, m(other, k, d).z))),
            );
            let exact = row.z - mu * row.truncate().length();
            assert!(exact >= f(&r["cone_margin_per_kg"]) - 1e-13);
            min_margin = min_margin.min(exact);
        }
        let post = [pre[0] + m(0, active, j), pre[1] + m(1, active, j)];
        for i in 0..2 {
            assert!(post[i].distance(v(&r["post_m_s"][i])) < 1e-13);
        }
        normal_error = normal_error
            .max(post[active].z.abs())
            .max((post[other].z - f(&x["manufactured_opening_m_s"])).abs());
        let tan = j.truncate();
        assert!(tan.length() <= mu * j.z + 1e-13 && post[other].z > 0. && j.z > 0.);
        if stick {
            assert!(post[active].length() < 1e-13);
        } else {
            polar_error =
                polar_error.max((tan + mu * j.z * post[active].truncate().normalize()).length());
        }
        maxloss = maxloss.max((f(&r["loss_j"]) + 0.5 * j.dot(pre[active] + post[active])).abs());
        for (idx, n) in r["native"].as_array().unwrap().iter().enumerate() {
            assert_eq!(n["passes"], [1, 8, 32][idx % 3]);
            assert_eq!(n["ordinary_rigid"], idx >= 3);
            max[idx][0] = max[idx][0].max(f(&n["active_error_ns"]));
            max[idx][1] = max[idx][1].max(f(&n["opening_vertex_load_ns"]));
            max[idx][2] = max[idx][2].max(f(&n["opening_speed_error_m_s"]).abs());
        }
    }
    assert_eq!(certified, 342);
    assert_eq!(modes.len(), 4);
    assert!(min_margin > 0. && normal_error < 1e-12 && polar_error < 1e-12 && maxloss < 1e-12);
    let public = read("public.json")?;
    let mut histories = vec![];
    for lane in public.as_array().unwrap() {
        let id = lane["opening_quadrant"].as_u64().unwrap() as usize;
        let h = lane["history"].as_array().unwrap();
        histories.push(json!({"label":lane["label"],"first_unloaded_site_ns":h[1]["corner_ns"][id],"last_gap_m":h.last().unwrap()["corner_gaps_m"][id],"later_recontact_at_tick":h.iter().skip(2).find(|r|f(&r["corner_ns"][id])>1e-6).map(|r|r["tick"].clone())}));
    }
    let file = path.join("opening-load.svg");
    let root = SVGBackend::new(&file, (870, 400)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root.split_evenly((1, 2));
    let colors = [
        RGBColor(191, 93, 17),
        RGBColor(28, 108, 121),
        RGBColor(49, 74, 143),
    ];
    for (k, area) in areas.iter().enumerate() {
        let mut chart = ChartBuilder::on(area)
            .margin(18)
            .caption(
                ["Opening vertex gap", "First 2 ms normal reactions"][k],
                ("sans-serif", 20),
            )
            .x_label_area_size(38)
            .y_label_area_size(52)
            .build_cartesian_2d(0f64..[80., 2.][k], 0f64..[21., 0.9][k])?;
        chart
            .configure_mesh()
            .x_desc("Public time (ms)")
            .y_desc(["Gap (mm)", "Impulse (N·s)"][k])
            .draw()?;
        for (i, lane) in public.as_array().unwrap().iter().enumerate() {
            let opening = lane["opening_quadrant"].as_u64().unwrap() as usize;
            let active = lane["active_quadrant"].as_u64().unwrap() as usize;
            let h = lane["history"].as_array().unwrap();
            chart
                .draw_series(LineSeries::new(
                    h.iter().filter(|r| k == 0 || f(&r["tick"]) <= 4.).map(|r| {
                        (
                            f(&r["tick"]) * 0.5,
                            if k == 0 {
                                1000. * f(&r["corner_gaps_m"][opening])
                            } else {
                                f(&r["corner_ns"][active])
                            },
                        )
                    }),
                    colors[i].stroke_width(2),
                ))?
                .label(["Ordinary", "32 PGS", "Physical jump"][i])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 14, y)], colors[i].stroke_width(2))
                });
            if k == 1 {
                chart.draw_series(h.iter().take(5).map(|r| {
                    TriangleMarker::new(
                        (0.5 * f(&r["tick"]), f(&r["corner_ns"][opening])),
                        4,
                        colors[i].filled(),
                    )
                }))?;
            }
        }
        if k == 0 {
            chart
                .configure_series_labels()
                .background_style(WHITE)
                .draw()?;
        }
    }
    root.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"cases":data.as_array().unwrap().len(),"certified":certified,"native_runs":certified*6,"declines":declines,"modes":modes.iter().map(|((i,s),n)|json!({"active":i,"sticks":s,"count":n})).collect::<Vec<_>>(),"min_exact_cone_margin_per_kg":min_margin,"max_kinematic_residual_m_s":normal_error,"max_friction_direction_residual_ns":polar_error,"max_energy_residual_j":maxloss,"native_max_active_error_inactive_impulse_opening_error":max,"replay":histories})
        )?
    );
    Ok(())
}
