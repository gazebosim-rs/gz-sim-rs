// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::trial::{self, Frame, Result};
use plotters::prelude::*;
use std::path::Path;

pub struct Point {
    t: f64,
    impulses: [f64; 3],
}
impl Point {
    pub fn read(f: &Frame, heavy_base: u64, light_base: u64) -> Self {
        let floor = |id| {
            f.floors
                .iter()
                .find(|p| p.body == id)
                .map_or(0., |p| p.normal_impulse)
        };
        Self {
            t: f.tick as f64 * 0.01,
            impulses: [
                floor(heavy_base),
                f.pairs
                    .iter()
                    .filter(|p| p.a == heavy_base)
                    .map(|p| p.normal_impulse)
                    .sum(),
                floor(light_base),
            ],
        }
    }
}

pub fn write(path: &Path, points: &[Point]) -> Result<()> {
    let page = SVGBackend::new(path, (1030, 540)).into_drawing_area();
    page.fill(&WHITE)?;
    let page = page.titled(
        "Normal contact impulses in the uneven workload",
        ("sans-serif", 25),
    )?;
    let (left, right) = page.split_horizontally(460);
    let end = trial::END as f64 * 0.01;
    let settled = trial::SETTLED as f64 * 0.01;
    for (area, caption, start, stop) in [
        (&left, "Initial contact", 0., 0.35),
        (&right, "Migration and recovery", settled, end),
    ] {
        let window = || points.iter().filter(|p| p.t >= start && p.t <= stop);
        let hi = window().flat_map(|p| p.impulses).fold(0., f64::max) * 1.25;
        let mut plot = ChartBuilder::on(area)
            .caption(caption, ("sans-serif", 20))
            .margin(14)
            .x_label_area_size(45)
            .y_label_area_size(55)
            .build_cartesian_2d(start..stop, 0.0..hi)?;
        plot.configure_mesh()
            .max_light_lines(1)
            .bold_line_style(RGBColor(222, 227, 231))
            .x_desc("Simulation time (s)")
            .y_desc("Impulse per 10 ms tick (N·s)")
            .draw()?;
        for (i, (label, color)) in [
            ("Floor under slab", RGBColor(78, 69, 161)),
            ("Cargo on slab (sum)", RGBColor(176, 76, 30)),
            ("Floor under one light stack", RGBColor(23, 115, 117)),
        ]
        .into_iter()
        .enumerate()
        {
            plot.draw_series(LineSeries::new(
                window().map(|p| (p.t, p.impulses[i])),
                color.stroke_width(3),
            ))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 19, y)], color.stroke_width(3)));
        }
        if start > 0. {
            for (i, (tick, name)) in [
                (trial::HANDOFF, "Handoff"),
                (trial::RETRY, "Retry"),
                (trial::REPLACEMENT, "Worker"),
                (trial::RESTORE, "Restore"),
            ]
            .into_iter()
            .enumerate()
            {
                let x = tick as f64 * 0.01;
                let y = hi * (0.89 + 0.075 * (i % 2) as f64);
                plot.draw_series([PathElement::new(
                    [(x, 0.), (x, y * 0.985)],
                    RGBColor(153, 158, 164),
                )])?;
                plot.draw_series([Text::new(name, (x, y), ("sans-serif", 14))])?;
            }
        } else {
            plot.configure_series_labels()
                .position(SeriesLabelPosition::UpperRight)
                .background_style(WHITE.mix(0.95))
                .draw()?;
        }
    }
    page.present()?;
    Ok(())
}
