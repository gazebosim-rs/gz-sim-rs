// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure single support from physical loads and collision geometry at every solver interval.
#[path = "support/humanoid_perturbations.rs"]
mod humanoid_perturbations;
use plotters::prelude::*;
use serde::{Deserialize, Serialize};
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Deserialize)]
struct Cut {
    tick: usize,
    com: [f64; 3],
    velocity: [f64; 3],
    feet_force_n: [f64; 2],
    other_force_n: f64,
    foot_bottom_m: [f64; 2],
    foot_centers_m: [[f64; 3]; 2],
    loaded_sole_points: [usize; 2],
    foot_support_margin_m: Option<f64>,
    capture_margin_m: Option<f64>,
    tilt_rad: f64,
}
#[derive(Serialize)]
struct Evidence {
    start_single_support_tick: usize,
    end_single_support_tick: usize,
    continuous_swing_intervals: usize,
    first_loaded_right_tick_after_swing: usize,
    minimum_stance_load_n: f64,
    maximum_swing_load_n: f64,
    minimum_loaded_stance_corners: usize,
    peak_swing_sole_clearance_m: f64,
    minimum_com_margin_in_single_support_m: f64,
    minimum_capture_margin_in_single_support_m: f64,
    peak_stance_center_drift_from_takeoff_m: f64,
    right_foot_forward_displacement_at_landing_m: f64,
    right_foot_forward_displacement_final_m: f64,
    final_feet_load_n: [f64; 2],
    final_speed_m_s: f64,
    peak_tilt_rad: f64,
    other_support_intervals: usize,
}
fn read(root: &Path, name: &str) -> Result<Vec<Cut>> {
    let rows = BufReader::new(File::open(
        root.join("qualification").join(format!("{name}.jsonl")),
    )?)
    .lines()
    .map(|line| Ok(serde_json::from_str::<Cut>(&line?)?))
    .collect::<Result<Vec<_>>>()?;
    if rows.len() != 10001 || rows.iter().enumerate().any(|(i, c)| i != c.tick) {
        return Err(format!("{name}: expected every millisecond through tick 10000").into());
    }
    Ok(rows)
}
fn measure(rows: &[Cut]) -> Result<Evidence> {
    let airborne = |c: &Cut| {
        c.feet_force_n[1] == 0.
            && c.loaded_sole_points[1] == 0
            && c.foot_bottom_m[1] > 0.001
            && c.feet_force_n[0] > 100.
            && c.loaded_sole_points[0] >= 3
            && c.other_force_n < 1.
    };
    let (mut best, mut current) = (0..0, 0..0);
    for c in rows {
        if airborne(c) {
            if current.is_empty() {
                current = c.tick..c.tick + 1;
            } else {
                current.end = c.tick + 1;
            }
            if current.len() > best.len() {
                best = current.clone();
            }
        } else {
            current = 0..0;
        }
    }
    if best.len() < 1000 {
        return Err("no sustained, torque-supported airborne swing".into());
    }
    let swings = &rows[best.clone()];
    let pre = &rows[best.start - 1];
    let landed = rows[best.end..]
        .iter()
        .find(|c| c.feet_force_n[1] > 1. && c.loaded_sole_points[1] > 0)
        .ok_or("no ground reacquisition")?;
    let last = rows.last().unwrap();
    let min_margin = |extract: fn(&Cut) -> Option<f64>| -> Result<f64> {
        swings
            .iter()
            .map(|r| extract(r).ok_or_else(|| "degenerate support polygon in swing".into()))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .reduce(f64::min)
            .ok_or_else(|| "empty swing".into())
    };
    Ok(Evidence {
        start_single_support_tick: best.start,
        end_single_support_tick: best.end - 1,
        continuous_swing_intervals: best.len(),
        first_loaded_right_tick_after_swing: landed.tick,
        minimum_stance_load_n: swings
            .iter()
            .map(|r| r.feet_force_n[0])
            .fold(f64::INFINITY, f64::min),
        maximum_swing_load_n: swings.iter().map(|r| r.feet_force_n[1]).fold(0., f64::max),
        minimum_loaded_stance_corners: swings
            .iter()
            .map(|r| r.loaded_sole_points[0])
            .min()
            .unwrap(),
        peak_swing_sole_clearance_m: swings.iter().map(|r| r.foot_bottom_m[1]).fold(0., f64::max),
        minimum_com_margin_in_single_support_m: min_margin(|r| r.foot_support_margin_m)?,
        minimum_capture_margin_in_single_support_m: min_margin(|r| r.capture_margin_m)?,
        peak_stance_center_drift_from_takeoff_m: swings
            .iter()
            .map(|r| {
                (r.foot_centers_m[0][0] - pre.foot_centers_m[0][0])
                    .hypot(r.foot_centers_m[0][1] - pre.foot_centers_m[0][1])
            })
            .fold(0., f64::max),
        right_foot_forward_displacement_at_landing_m: landed.foot_centers_m[1][0]
            - pre.foot_centers_m[1][0],
        right_foot_forward_displacement_final_m: last.foot_centers_m[1][0]
            - rows[0].foot_centers_m[1][0],
        final_feet_load_n: last.feet_force_n,
        final_speed_m_s: last.velocity.iter().map(|v| v * v).sum::<f64>().sqrt(),
        peak_tilt_rad: rows.iter().map(|r| r.tilt_rad).fold(0., f64::max),
        other_support_intervals: rows.iter().filter(|r| r.other_force_n > 1.).count(),
    })
}
fn main() -> Result<()> {
    let root = std::env::args_os()
        .nth(1)
        .ok_or("supply humanoid step artifact directory")?;
    let root = Path::new(&root);
    humanoid_perturbations::write(root)?;
    let stationary = read(root, "single-support")?;
    let step = read(root, "step")?;
    let proof = serde_json::json!({"stationary_single_support":measure(&stationary)?,"forward_step":measure(&step)?});
    serde_json::to_writer_pretty(File::create(root.join("measurements.json"))?, &proof)?;
    let output = root.join("step.svg");
    let area = SVGBackend::new(&output, (1050, 860)).into_drawing_area();
    area.fill(&WHITE)?;
    let panels = area.split_evenly((3, 1));
    let (blue, red, green) = (
        RGBColor(34, 88, 148),
        RGBColor(184, 94, 35),
        RGBColor(31, 127, 105),
    );
    type Series<'a> = (&'a str, RGBColor, Box<dyn Fn(&Cut) -> Option<f64> + 'a>);
    let chart = |index: usize,
                 title: &str,
                 y: &str,
                 range: std::ops::Range<f64>,
                 series: Vec<Series>|
     -> Result<()> {
        let mut panel = ChartBuilder::on(&panels[index])
            .margin(12)
            .caption(title, ("sans-serif", 22))
            .x_label_area_size(37)
            .y_label_area_size(68)
            .build_cartesian_2d(0.0..7.5, range)?;
        panel
            .configure_mesh()
            .x_desc("Simulation time (s)")
            .y_desc(y)
            .light_line_style(RGBColor(231, 233, 235))
            .draw()?;
        for (name, color, extract) in series {
            panel
                .draw_series(LineSeries::new(
                    step.iter()
                        .step_by(4)
                        .filter(|r| r.tick <= 7500)
                        .filter_map(|r| extract(r).map(|v| (r.tick as f64 / 1000., v))),
                    color.stroke_width(3),
                ))?
                .label(name)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 24, y)], color.stroke_width(3))
                });
        }
        panel
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.9))
            .draw()?;
        Ok(())
    };
    chart(
        0,
        "Weight transfers to the left foot before the right foot advances",
        "Normal force (N)",
        -25.0..950.,
        vec![
            ("Left sole", blue, Box::new(|r| Some(r.feet_force_n[0]))),
            ("Right sole", red, Box::new(|r| Some(r.feet_force_n[1]))),
        ],
    )?;
    let x0 = step[0].foot_centers_m[1][0];
    chart(
        1,
        "Right-foot motion measured from the dynamic box and its lowest corner",
        "Distance (mm)",
        -8.0..130.,
        vec![
            (
                "Forward displacement",
                blue,
                Box::new(|r| Some(1000. * (r.foot_centers_m[1][0] - x0))),
            ),
            (
                "Sole clearance",
                red,
                Box::new(|r| Some(1000. * r.foot_bottom_m[1])),
            ),
        ],
    )?;
    chart(
        2,
        "Balance relative to the convex hull of loaded contact points",
        "Distance (mm)",
        -10.0..170.,
        vec![
            (
                "COM margin",
                blue,
                Box::new(|r| r.foot_support_margin_m.map(|v| v * 1000.)),
            ),
            (
                "Capture-point margin",
                green,
                Box::new(|r| r.capture_margin_m.map(|v| v * 1000.)),
            ),
            ("Lateral COM", red, Box::new(|r| Some(1000. * r.com[1]))),
        ],
    )?;
    area.present()?;
    println!("{}", serde_json::to_string_pretty(&proof)?);
    Ok(())
}
