// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare identical free-space sphere impacts across the four public steps.
use clap::Parser;
use plotters::prelude::*;
use serde::Deserialize;
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Directory containing 1ms.jsonl, 2ms.jsonl, 5ms.jsonl and 10ms.jsonl.
    directory: PathBuf,
    #[arg(long)]
    figure: PathBuf,
}

#[derive(Deserialize)]
struct Trial {
    seed: u64,
    step_ms: u64,
    frames: u64,
    exact_frames: u64,
    friction: f64,
    analytic_impact_s: f64,
    impact: u64,
    transferred: bool,
    analytical_normal_impulse: f64,
    impact_impulse: f64,
    max_angular_momentum_error: f64,
    smooth_final_velocity_error: Option<f64>,
    smooth_final_spin_error: Option<f64>,
    error: Option<String>,
}

fn read(path: &Path, ms: u64) -> Result<Vec<Trial>> {
    let file = File::open(path.join(format!("{ms}ms.jsonl")))?;
    let mut result = Vec::new();
    for (i, line) in BufReader::new(file).lines().enumerate() {
        let v: Trial = serde_json::from_str(&line?)?;
        if v.step_ms != ms
            || v.error.is_some()
            || !v.transferred
            || v.impact == 0
            || v.frames != v.exact_frames
            || v.frames * ms != 310
            || v.analytical_normal_impulse <= 0.
            || v.impact_impulse <= 0.
            || v.smooth_final_velocity_error.is_some() != (v.friction == 0.)
            || v.smooth_final_spin_error.is_some() != (v.friction == 0.)
        {
            return Err(format!("unqualified sphere trial on line {} at {ms} ms", i + 1).into());
        }
        if result
            .last()
            .is_some_and(|p: &Trial| p.seed.checked_add(1) != Some(v.seed))
        {
            return Err("seeds must be unique, ordered and contiguous".into());
        }
        result.push(v);
    }
    if result.is_empty() {
        return Err("empty sphere campaign".into());
    }
    Ok(result)
}

fn p95(mut x: Vec<f64>) -> Result<f64> {
    if x.is_empty() || x.iter().any(|v| !v.is_finite() || *v < 0.) {
        return Err("nonfinite or missing observations".into());
    }
    x.sort_unstable_by(f64::total_cmp);
    Ok(x[(95 * x.len()).div_ceil(100) - 1])
}

fn main() -> Result<()> {
    let args = Args::parse();
    let steps = [1, 2, 5, 10];
    let grid = steps
        .iter()
        .map(|&ms| read(&args.directory, ms))
        .collect::<Result<Vec<_>>>()?;
    let base = &grid[0];
    for cells in &grid[1..] {
        if cells.len() != base.len()
            || cells.iter().zip(base).any(|(a, b)| {
                a.seed != b.seed
                    || a.friction != b.friction
                    || a.analytic_impact_s != b.analytic_impact_s
                    || a.analytical_normal_impulse != b.analytical_normal_impulse
            })
        {
            return Err("physical seeds or analytical controls differ across timesteps".into());
        }
    }
    let mut points = Vec::new();
    println!(
        "| Public step | Exact frames | First impulse timing p95 (ms) | Angular momentum p95 (kg·m²/s) | Smooth impulse error p95 (%) | Smooth outgoing velocity error p95 (m/s) |"
    );
    println!("| ---: | ---: | ---: | ---: | ---: | ---: |");
    for (ms, rows) in steps.into_iter().zip(&grid) {
        let h = p95(rows.iter().map(|v| v.max_angular_momentum_error).collect())?;
        let t = p95(rows
            .iter()
            .map(|v| (v.impact as f64 * ms as f64 - v.analytic_impact_s * 1000.).abs())
            .collect())?;
        let j = p95(rows
            .iter()
            .filter(|v| v.friction == 0.)
            .map(|v| 100. * (v.impact_impulse / v.analytical_normal_impulse - 1.).abs())
            .collect())?;
        let v = p95(rows
            .iter()
            .filter_map(|s| s.smooth_final_velocity_error)
            .collect())?;
        println!(
            "| {ms} ms | {} | {t:.3} | {h:.5} | {j:.3} | {v:.5} |",
            rows.iter().map(|v| v.frames).sum::<u64>()
        );
        points.push((ms as f64, [t, h, j, v]));
    }
    let spin = grid
        .iter()
        .flatten()
        .filter_map(|v| v.smooth_final_spin_error)
        .fold(0.0, f64::max);
    println!(
        "\n{} matched physical seeds per step, including {} frictionless impacts. Maximum frictionless spin change {spin:.3e} rad/s.",
        base.len(),
        base.iter().filter(|v| v.friction == 0.).count()
    );
    let out = SVGBackend::new(&args.figure, (940, 620)).into_drawing_area();
    out.fill(&WHITE)?;
    let main = out.titled(
        "Oblique sphere impacts  ·  95th-percentile error",
        ("sans-serif", 25),
    )?;
    let labels = [
        "First positive impulse timing (ms)",
        "Angular momentum (kg·m²/s)",
        "Frictionless normal impulse (%)",
        "Frictionless outgoing velocity (m/s)",
    ];
    for (i, panel) in main.split_evenly((2, 2)).into_iter().enumerate() {
        let max = points.iter().map(|(_, r)| r[i]).fold(0.0, f64::max);
        let mut chart = ChartBuilder::on(&panel)
            .margin(14)
            .caption(labels[i], ("sans-serif", 16))
            .x_label_area_size(36)
            .y_label_area_size(62)
            .build_cartesian_2d(0f64..11., 0f64..max.max(1e-12) * 1.12)?;
        chart
            .configure_mesh()
            .x_desc("Public step (ms)")
            .light_line_style(RGBColor(229, 232, 236))
            .draw()?;
        let samples = points.iter().map(|(x, r)| (*x, r[i]));
        chart.draw_series(LineSeries::new(
            samples.clone(),
            RGBColor(20, 99, 142).stroke_width(3),
        ))?;
        chart.draw_series(samples.map(|p| Circle::new(p, 4, RGBColor(203, 94, 28).filled())))?;
    }
    out.present()?;
    Ok(())
}
