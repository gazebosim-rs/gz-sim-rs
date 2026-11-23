// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::trial::{self, Frame, Result};
use plotters::prelude::*;
use std::path::Path;

pub struct Point {
    time: f64,
    heights: [f64; 4],
    pair_range: Option<(f64, f64)>,
    floor_range: Option<(f64, f64)>,
}
impl Point {
    pub fn from_frame(f: &Frame<'_>) -> Self {
        fn range(mut values: impl Iterator<Item = f64>) -> Option<(f64, f64)> {
            let first = values.next()?;
            Some(values.fold((first, first), |(low, high), v| (low.min(v), high.max(v))))
        }
        Self {
            time: f.tick as f64 * 0.01,
            heights: std::array::from_fn(|i| f.bodies[2 * i + 1].position[2]),
            pair_range: (f.tick >= trial::SETTLED)
                .then(|| range(f.pairs.iter().map(|p| p.normal_impulse)))
                .flatten(),
            floor_range: (f.tick >= trial::SETTLED)
                .then(|| range(f.floors.iter().map(|p| p.normal_impulse)))
                .flatten(),
        }
    }
}

pub fn write(path: &Path, points: &[Point]) -> Result<()> {
    let root = SVGBackend::new(path, (1100, 620)).into_drawing_area();
    root.fill(&WHITE)?;
    let page = root.titled(
        "Four independent stacks through migration and recovery",
        ("sans-serif", 24),
    )?;
    let (upper, lower) = page.split_vertically(275);
    let end = trial::END as f64 * 0.01;
    let settled = trial::SETTLED as f64 * 0.01;
    let mut heights = ChartBuilder::on(&upper)
        .margin(12)
        .x_label_area_size(32)
        .y_label_area_size(62)
        .caption("Height of the four dropped spheres", ("sans-serif", 18))
        .build_cartesian_2d(0.0..end, 0.0..4.1)?;
    heights
        .configure_mesh()
        .max_light_lines(1)
        .x_desc("Simulation time (s)")
        .y_desc("Center height (m)")
        .draw()?;
    let colors = [
        RGBColor(19, 97, 150),
        RGBColor(172, 83, 28),
        RGBColor(118, 76, 151),
        RGBColor(39, 130, 99),
    ];
    for (i, (color, label)) in colors
        .into_iter()
        .zip(["West", "North", "East", "South"])
        .enumerate()
    {
        heights
            .draw_series(LineSeries::new(
                points.iter().map(|p| (p.time, p.heights[i])),
                color.stroke_width(3),
            ))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 22, y)], color.stroke_width(3)));
    }
    heights
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.95))
        .draw()?;
    let mut loads = ChartBuilder::on(&lower)
        .margin(12)
        .x_label_area_size(40)
        .y_label_area_size(62)
        .caption(
            "Settled contact impulse per 10 ms tick, range across four stacks",
            ("sans-serif", 18),
        )
        .build_cartesian_2d(settled..end, 0.0..0.245)?;
    loads
        .configure_mesh()
        .max_light_lines(1)
        .x_desc("Simulation time (s)")
        .y_desc("Normal impulse (N·s)")
        .draw()?;
    for (tick, label) in [
        (trial::HANDOFF, "Handoff"),
        (trial::RETRY, "Retry"),
        (trial::REPLACEMENT, "Worker"),
        (trial::RESTORE, "Restore"),
    ] {
        let x = tick as f64 * 0.01;
        loads.draw_series([PathElement::new(
            [(x, 0.), (x, 0.218)],
            RGBColor(156, 167, 175),
        )])?;
        loads.draw_series([Text::new(label, (x, 0.232), ("sans-serif", 14).into_font())])?;
    }
    for (floor, label, color) in [
        (false, "Between spheres", RGBColor(180, 70, 45)),
        (true, "Against ground", RGBColor(128, 75, 168)),
    ] {
        let observed = |p: &Point| if floor { p.floor_range } else { p.pair_range };
        let lo = points
            .iter()
            .filter_map(|p| observed(p).map(|(v, _)| (p.time, v)));
        let hi = points
            .iter()
            .filter_map(|p| observed(p).map(|(_, v)| (p.time, v)));
        loads
            .draw_series(LineSeries::new(lo, color.stroke_width(2)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 22, y)], color.stroke_width(3)));
        loads.draw_series(LineSeries::new(hi, color.stroke_width(2)))?;
    }
    loads
        .configure_series_labels()
        .position(SeriesLabelPosition::LowerRight)
        .background_style(WHITE.mix(0.95))
        .draw()?;
    root.present()?;
    Ok(())
}
