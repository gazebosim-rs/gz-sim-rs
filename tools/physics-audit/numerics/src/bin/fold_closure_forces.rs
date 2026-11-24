// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot measured layer force against the motion of the two responding jaws.
use plotters::prelude::*;
use serde_json::Value;
use std::{error::Error, fs, io::Write, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const COLORS: [RGBColor; 2] = [RGBColor(7, 111, 122), RGBColor(166, 76, 16)];

#[derive(Debug)]
struct Stage {
    command: f64,
    aperture_mm: f64,
    force_mn: f64,
    sustained: bool,
}

fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .ok_or_else(|| "missing or invalid measurement".into())
}

fn stages(trial: &Value, floor: f64) -> Result<Vec<Stage>> {
    let values = trial["stages"]
        .as_array()
        .filter(|s| !s.is_empty())
        .ok_or("missing stages")?;
    let mut points = Vec::new();
    for stage in values {
        let command = number(&stage["command_stroke_m"])?;
        let aperture = number(&stage["mean_aperture_m"])?;
        let mean = number(&stage["mean_cross_panel_n"])?;
        let minimum = number(&stage["min_cross_panel_n"])?;
        let sustained = stage["sustained"]
            .as_bool()
            .ok_or("missing sustained status")?;
        if aperture <= 0.
            || minimum < 0.
            || mean + 1e-14 < minimum
            || sustained != (minimum >= floor)
        {
            return Err("inconsistent aperture, load or window status".into());
        }
        points.push(Stage {
            command,
            aperture_mm: aperture * 1e3,
            force_mn: mean * 1e3,
            sustained,
        });
    }
    Ok(points)
}

fn first_confirmed(mesh: &[Stage]) -> Option<usize> {
    mesh.windows(2)
        .position(|pair| pair.iter().all(|s| s.sustained))
}

fn onset(points: &[Stage]) -> String {
    match first_confirmed(points) {
        Some(0) => format!("Loaded at first opening ({:.6} mm)", points[0].aperture_mm),
        Some(i) if points[i - 1].command == points[i].command => {
            "Load began at an unchanged setting".into()
        }
        Some(i) => format!(
            "{:.6} to {:.6} mm",
            points[i - 1].aperture_mm,
            points[i].aperture_mm
        ),
        None => "No confirmed load in this sweep".into(),
    }
}

struct Clock {
    name: String,
    meshes: [Vec<Stage>; 2],
}

fn mesh_index(trial: &Value) -> Result<usize> {
    Ok(match trial["columns"].as_u64() {
        Some(5) => 0,
        Some(7) => 1,
        _ => return Err("unknown mesh".into()),
    })
}

fn clock(records: &Value, floor: f64) -> Result<Clock> {
    let trials = records.as_array().ok_or("missing mesh trials")?;
    let mut meshes = [None, None];
    let mut settings = None;
    for trial in trials {
        let idx = mesh_index(trial)?;
        let steps = trial["material_substeps"]
            .as_u64()
            .ok_or("missing material clock")?;
        let iterations = trial["solver_iterations"]
            .as_u64()
            .ok_or("missing iteration count")?;
        if steps == 0
            || iterations == 0
            || settings
                .replace((steps, iterations))
                .is_some_and(|old| old != (steps, iterations))
        {
            return Err("unequal or empty solver resolutions in pair".into());
        }
        if meshes[idx].replace(stages(trial, floor)?).is_some() {
            return Err("duplicate mesh".into());
        }
    }
    let [Some(five), Some(seven)] = meshes else {
        return Err("missing mesh".into());
    };
    let (steps, iters) = settings.unwrap();
    Ok(Clock {
        name: format!("{steps} substeps × {iters} iterations"),
        meshes: [five, seven],
    })
}

fn fixed_means(report: &Value, primary: &Clock, floor: f64) -> Result<[String; 2]> {
    let mut forces = [None; 2];
    let expected = primary.meshes.each_ref().map(|m| {
        first_confirmed(m)
            .filter(|&i| i > 0 && m[i].command != m[i - 1].command)
            .map(|i| m[i - 1].command)
    });
    for trial in report.as_array().ok_or("missing fixed-target controls")? {
        let i = mesh_index(trial)?;
        let hold = stages(trial, floor)?;
        let [.., last, a, b] = hold.as_slice() else {
            return Err("incomplete fixed-target hold".into());
        };
        if expected[i] != Some(last.command) || last.command != a.command || a.command != b.command
        {
            return Err("control did not hold the preceding target".into());
        }
        if forces[i].replace(b.force_mn).is_some() {
            return Err("duplicate fixed-target control".into());
        }
    }
    for i in 0..2 {
        if expected[i].is_some() && forces[i].is_none() {
            return Err("missing bracket control".into());
        }
    }
    Ok(forces.map(|v| v.map_or_else(|| "no bracket".into(), |f| format!("{f:.3} mN"))))
}

fn draw(report: &Value) -> Result<String> {
    let floor = number(&report["force_floor_n"])?;
    let dt = report["dt_ns"].as_u64().ok_or("missing public clock")?;
    let window = report["window_ticks"]
        .as_u64()
        .ok_or("missing observation window")?;
    if dt == 0 || window == 0 || floor <= 0. {
        return Err("invalid observation window or threshold".into());
    }
    let clocks = [
        clock(&report["trials"], floor)?,
        clock(&report["reference_trials"], floor)?,
    ];
    let [a, b] = fixed_means(&report["fixed_target_controls"], &clocks[0], floor)?;
    let plateau = report["plateau_ticks"]
        .as_u64()
        .filter(|t| *t >= window)
        .ok_or("invalid plateau duration")?;
    let meshes = clocks.iter().flat_map(|c| &c.meshes);
    let start = meshes.clone().map(|m| m[0].aperture_mm).fold(0., f64::max);
    let x = |s: &Stage| (start - s.aperture_mm) * 1e3;
    // Retain two unloaded points leading into the first nonzero force. All
    // clock/mesh curves share exactly the same axes over the neighborhood.
    let lo = meshes
        .clone()
        .map(|m| {
            let first = m.iter().position(|s| s.force_mn > 0.).unwrap_or_default();
            x(&m[first.saturating_sub(2)])
        })
        .fold(f64::INFINITY, f64::min);
    let hi = meshes.clone().flatten().map(x).fold(lo, f64::max);
    let ymax = meshes
        .clone()
        .flatten()
        .map(|s| s.force_mn)
        .fold(floor * 1e3, f64::max);
    let xpad = ((hi - lo) * 0.06).max(0.05);
    let mut svg = String::new();
    {
        let page = SVGBackend::with_string(&mut svg, (1130, 704)).into_drawing_area();
        page.fill(&WHITE)?;
        page.draw(&Text::new(
            "When do the folded layers stay in contact?",
            (47, 20),
            ("sans-serif", 30),
        ))?;
        page.draw(&Text::new(format!("Measured closure relative to a common {:.6} mm jaw opening. Shading marks the onset brackets.",start),(48,62),("sans-serif",17)))?;
        let (top, lower) = page.split_vertically(124);
        for (i, xx) in [50, 330].iter().copied().enumerate() {
            top.draw(&PathElement::new(
                vec![(xx, 103), (xx + 31, 103)],
                COLORS[i].stroke_width(3),
            ))?;
            top.draw(&Circle::new((xx + 15, 103), 4, COLORS[i].filled()))?;
            top.draw(&Text::new(
                format!("{} points across the sheet", 5 + 2 * i),
                (xx + 43, 94),
                ("sans-serif", 17),
            ))?;
        }
        let (graphs, footer) = lower.split_vertically(368);
        for (area, clock) in graphs.split_evenly((1, 2)).iter().zip(&clocks) {
            let mut chart = ChartBuilder::on(area)
                .caption(&clock.name, ("sans-serif", 21))
                .margin_left(8)
                .margin_right(17)
                .x_label_area_size(60)
                .y_label_area_size(71)
                .build_cartesian_2d(lo - xpad..hi + xpad, -0.04 * ymax..1.1 * ymax)?;
            chart
                .configure_mesh()
                .x_desc("Measured jaw closure (µm)")
                .y_desc("Mean force between layers (mN)")
                .x_labels(6)
                .y_labels(6)
                .max_light_lines(0)
                .axis_desc_style(("sans-serif", 16))
                .label_style(("sans-serif", 14))
                .light_line_style(RGBColor(226, 229, 228))
                .draw()?;
            for (mesh, color) in clock.meshes.iter().zip(COLORS) {
                if let Some(i) = first_confirmed(mesh)
                    .filter(|&i| i > 0 && mesh[i].command != mesh[i - 1].command)
                {
                    chart.draw_series(std::iter::once(Rectangle::new(
                        [(x(&mesh[i - 1]), 0.), (x(&mesh[i]), ymax)],
                        color.mix(0.16).filled(),
                    )))?;
                }
                let coords = mesh
                    .iter()
                    .filter(|s| x(s) >= lo - xpad)
                    .map(|s| (x(s), s.force_mn));
                chart.draw_series(LineSeries::new(coords.clone(), color.stroke_width(2)))?;
                chart.draw_series(coords.map(|xy| Circle::new(xy, 3, color.filled())))?;
            }
        }
        footer.draw(&Text::new(
            "Jaw opening at the first confirmed load",
            (48, 8),
            ("sans-serif", 21),
        ))?;
        for (i, clock) in clocks.iter().enumerate() {
            for (j, mesh) in clock.meshes.iter().enumerate() {
                let xx = 48 + i as i32 * 563;
                let yy = 49 + j as i32 * 29;
                footer.draw(&Text::new(
                    format!("{} points", 5 + 2 * j),
                    (xx, yy),
                    ("sans-serif", 16).into_font().color(&COLORS[j]),
                ))?;
                footer.draw(&Text::new(
                    onset(mesh),
                    (xx + 97, yy - 2),
                    ("sans-serif", 18),
                ))?;
            }
        }
        footer.draw(&Text::new(format!("At each plateau the final {:.0} ms must stay at or above {:.3} mN. The next plateau must also meet this test.",dt as f64*window as f64/1e6,floor*1e3),(48,135),("sans-serif",16)))?;
        footer.draw(&Text::new(format!("After another {:.0} ms at the preceding jaw setting, the final means were {a} (5 points) and {b} (7 points).",2.*dt as f64*plateau as f64/1e6),(48,165),("sans-serif",16)))?;
        page.present()?;
    }
    Ok(svg)
}

fn render(input: &Path, output: &Path) -> Result<()> {
    let report = serde_json::from_slice(&fs::read(input)?)?;
    let svg = draw(&report)?;
    fs::File::create_new(output)?.write_all(svg.as_bytes())?;
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [input, output] = &args[..] else {
        return Err("usage: fold_closure_forces COMPARISON.json NEW_FIGURE.svg".into());
    };
    render(Path::new(input), Path::new(output))
}

#[cfg(test)]
#[path = "../../tests/support/fold_closure_forces.rs"]
mod tests;
