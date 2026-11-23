// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn u(v: &Value) -> usize {
    v.as_u64().unwrap() as usize
}
fn v(a: &Value) -> DVec3 {
    DVec3::from(serde_json::from_value::<[f64; 3]>(a.clone()).unwrap())
}
fn main() -> Result<(), Box<dyn Error>> {
    let p = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let data: Value = serde_json::from_slice(&fs::read(p.join("field.json"))?)?;
    let aa = data["tiered"]["samples"].as_array().unwrap();
    let bb = data["dynamic"]["samples"].as_array().unwrap();
    assert_eq!(aa.len(), 346);
    let transitions = data["tiered"]["transitions"].as_array().unwrap();
    let stops = [0.01821381976, 0.04349999975, 0.08700000025, 0.12312957];
    let mut at = [0f64; 4];
    let mut rearm = [0f64; 4];
    for (i, r) in transitions[..4].iter().enumerate() {
        assert_eq!(u(&r["station"]), i);
        assert_eq!(r["prescribed"], true);
        at[i] = (u(&r["tick"]) - 1) as f64 * 0.0005;
        rearm[i] = at[i] - stops[i];
        assert!(rearm[i] > 0.002 && rearm[i] <= 0.0035);
    }
    let mut pre = [0f64; 4];
    let mut post = [0f64; 4];
    let mut j = 0f64;
    for (i, (a, b)) in aa.iter().zip(bb).enumerate() {
        assert_eq!(u(&a["tick"]), i);
        for k in 0..4 {
            let delta = v(&a["positions"][k]).distance(v(&b["positions"][k]));
            let dst = if i < 270 { &mut pre } else { &mut post };
            dst[k] = dst[k].max(delta);
        }
        if i >= 270 {
            for k in 0..2 {
                j = j.max(
                    (a["station0_ns"][k].as_f64().unwrap() - b["station0_ns"][k].as_f64().unwrap())
                        .abs(),
                );
            }
            assert_eq!(a["prescribed"], json!([false, true, true, true]));
        }
    }
    assert_eq!(pre, [0.; 4]);
    assert_eq!(post[0], 0.);
    assert_eq!(j, 0.);
    let first = aa[270]["station0_ns"].clone();
    let path = p.join("station-tiers.svg");
    let area = SVGBackend::new(&path, (970, 430)).into_drawing_area();
    area.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&area)
        .margin(14)
        .caption(
            "Station demotions and a local support loss",
            ("sans-serif", 22),
        )
        .x_label_area_size(42)
        .y_label_area_size(125)
        .build_cartesian_2d(0f64..173f64, 0f64..4.5)?;
    chart
        .configure_mesh()
        .disable_y_mesh()
        .y_labels(0)
        .x_desc("Public simulation time (ms)")
        .draw()?;
    for (k, at) in at.iter().enumerate() {
        let y = 3.75 - k as f64;
        let new = at * 1e3;
        let end = if k == 0 { 134.5 } else { 172.5 };
        chart.draw_series([
            Rectangle::new(
                [(0., y - 0.18), (new, y + 0.18)],
                RGBColor(174, 83, 41).filled(),
            ),
            Rectangle::new(
                [(new, y - 0.18), (end, y + 0.18)],
                RGBColor(35, 127, 118).filled(),
            ),
        ])?;
        if k == 0 {
            chart.draw_series([Rectangle::new(
                [(end, y - 0.18), (172.5, y + 0.18)],
                RGBColor(174, 83, 41).filled(),
            )])?;
        }
        chart.draw_series([Text::new(
            format!("Foot {k}"),
            (1., y + 0.2),
            ("sans-serif", 15),
        )])?;
    }
    chart.draw_series([
        Text::new(
            "Dynamic",
            (4., 4.12),
            ("sans-serif", 16).into_font().color(&RGBColor(174, 83, 41)),
        ),
        Text::new(
            "Prescribed",
            (29., 4.12),
            ("sans-serif", 16)
                .into_font()
                .color(&RGBColor(35, 127, 118)),
        ),
    ])?;
    area.present()?;
    fs::write(
        p.join("checks/readback.json"),
        serde_json::to_vec_pretty(
            &json!({"source_demotion_start_seconds":at,"stop_to_demotion_seconds":rearm,"departing_pad_command_seconds":0.1345,"first_resumed_station_zero_ns":first,"max_pre_loss_position_difference_m":pre,"max_post_loss_position_difference_m":post,"max_woken_station_per_pad_impulse_difference_ns":j}),
        )?,
    )?;
    Ok(())
}
