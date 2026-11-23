// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot physical errors measured by convoy_inspect --series.
use plotters::prelude::*;
use serde::Deserialize;
use std::{env, error::Error, fs::File, io::Write};

#[derive(Deserialize)]
struct Report {
    samples: Vec<Sample>,
}

#[derive(Deserialize)]
struct Sample {
    row: usize,
    reference_clock: (u64, u64),
    physical_errors: Option<Errors>,
}

#[derive(Deserialize)]
struct Errors {
    max_position_m: f64,
    max_linear_velocity_m_s: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    let [input, output] = args.as_slice() else {
        return Err("usage: convoy_inspection_plot SERIES_REPORT.json OUTPUT.svg".into());
    };
    let report: Report = serde_json::from_reader(File::open(input)?)?;
    let n = report.samples.len();
    if n == 0
        || report
            .samples
            .iter()
            .enumerate()
            .any(|(i, s)| s.row != i + 1)
    {
        return Err("invalid comparison series ordering".into());
    }
    if !report.samples.iter().any(|s| s.physical_errors.is_some()) {
        return Err("no physically comparable observations".into());
    }
    let mut svg = String::new();
    {
        let root = SVGBackend::with_string(&mut svg, (1040, 540)).into_drawing_area();
        root.fill(&WHITE)?;
        let area = root.titled("Convoy checkpoint comparison", ("sans-serif", 30))?;
        let panels = area.margin(8, 10, 10, 10).split_evenly((2, 1));
        for (i, panel) in panels.iter().enumerate() {
            let (metric, color) = if i == 0 {
                ("Maximum position difference (µm)", BLUE)
            } else {
                ("Maximum linear velocity difference (mm/s)", RED)
            };
            let values: Vec<_> = report
                .samples
                .iter()
                .map(|s| {
                    s.physical_errors.as_ref().map(|p| {
                        if i == 0 {
                            1e6 * p.max_position_m
                        } else {
                            1e3 * p.max_linear_velocity_m_s
                        }
                    })
                })
                .collect();
            let max = values.iter().flatten().copied().fold(0f64, f64::max);
            let mut chart = ChartBuilder::on(panel)
                .caption(metric, ("sans-serif", 19))
                .margin(5)
                .x_label_area_size(44)
                .y_label_area_size(77)
                .build_cartesian_2d(
                    0.9..(n as f64 + 0.1),
                    0.0..if max > 0. { max * 1.1 } else { 1. },
                )?;
            chart
                .configure_mesh()
                .x_labels(n.min(12))
                .x_desc("Paired checkpoint (reference epoch/tick)")
                .x_label_formatter(&|x| {
                    let row = f64::round(*x);
                    if (row - *x).abs() > 1e-8 || row < 1. || row > n as f64 {
                        return String::new();
                    }
                    let clock = report.samples[row as usize - 1].reference_clock;
                    format!("{}/{}", clock.0, clock.1)
                })
                .light_line_style(RGBColor(226, 231, 235))
                .draw()?;
            // Separate incomparable rows; a missing comparison cannot contribute
            // a zero or connect the two neighboring measurements.
            for run in values
                .iter()
                .enumerate()
                .collect::<Vec<_>>()
                .split(|(_, v)| v.is_none())
            {
                let points: Vec<_> = run
                    .iter()
                    .filter_map(|(j, v)| v.map(|y| ((*j + 1) as f64, y)))
                    .collect();
                chart.draw_series(LineSeries::new(
                    points.iter().copied(),
                    color.stroke_width(2),
                ))?;
                chart.draw_series(
                    points
                        .into_iter()
                        .map(|point| Circle::new(point, 4, color.filled())),
                )?;
            }
        }
        root.present()?;
    }
    File::create_new(output)?.write_all(svg.as_bytes())?;
    Ok(())
}
