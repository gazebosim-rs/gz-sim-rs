// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot the published contact receipts and cylinder velocity from the replay trace.
use super::Sample;
use gz_physics_distributed::physics::Primitive;
use plotters::prelude::*;
use std::{error::Error, path::Path};

pub(super) fn draw(samples: &[Sample], path: &Path) -> Result<(), Box<dyn Error>> {
    let end = samples.last().ok_or("empty trace")?.tick;
    let last_time = end as f64 * 0.01;
    let mut loads = [Vec::new(), Vec::new()];
    let mut velocity = Vec::new();
    for s in samples {
        let t = s.tick as f64 * 0.01;
        loads[0].push((t, s.impulses.iter().map(|p| p.normal_impulse).sum::<f64>()));
        loads[1].push((t, s.supports.iter().map(|p| p.normal_impulse).sum::<f64>()));
        let c = s
            .bodies
            .iter()
            .find(|b| b.primitive == Primitive::Cylinder)
            .ok_or("missing cylinder in encounter")?;
        velocity.push((t, c.linear[0]));
    }
    let root = SVGBackend::new(path, (920, 510)).into_drawing_area();
    root.fill(&WHITE)?;
    let area = root.titled("Four-worker contact and recovery", ("sans-serif", 26))?;
    let (upper, lower) = area.split_vertically(255);
    let blue = RGBColor(35, 102, 151);
    let amber = RGBColor(185, 92, 16);
    let peak = loads.iter().flatten().map(|p| p.1).fold(0., f64::max);
    let mut chart = ChartBuilder::on(&upper)
        .margin(10)
        .x_label_area_size(25)
        .y_label_area_size(77)
        .build_cartesian_2d(0.0..last_time, 0.0..(peak.max(1.) * 1.15))?;
    chart
        .configure_mesh()
        .y_desc("Impulse (N·s / tick)")
        .axis_desc_style(("sans-serif", 15))
        .max_light_lines(0)
        .bold_line_style(RGBColor(223, 229, 232))
        .draw()?;
    for ((points, color), label) in loads
        .iter()
        .zip([blue, amber])
        .zip(["Sphere / capsule", "Cylinder / ground"])
    {
        chart
            .draw_series(LineSeries::new(
                points.iter().copied(),
                color.stroke_width(2),
            ))?
            .label(label)
            .legend(move |(x, y)| {
                PathElement::new(vec![(x, y), (x + 22, y)], color.stroke_width(3))
            });
        if let Some(&(time, impulse)) = points.iter().max_by(|a, b| a.1.total_cmp(&b.1)) {
            chart.draw_series(std::iter::once(Text::new(
                format!("{impulse:.2} N·s"),
                (time + 0.015, impulse),
                ("sans-serif", 16).into_font().color(&color),
            )))?;
        }
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.92))
        .border_style(RGBColor(211, 217, 220))
        .draw()?;
    let (lo, hi) = velocity
        .iter()
        .map(|p| p.1)
        .fold((0_f64, 0_f64), |(lo, hi), v| (lo.min(v), hi.max(v)));
    let margin = ((hi - lo) * 0.3).max(0.01);
    let mut chart = ChartBuilder::on(&lower)
        .margin(10)
        .x_label_area_size(44)
        .y_label_area_size(77)
        .build_cartesian_2d(0.0..last_time, lo - margin..hi + margin)?;
    chart
        .configure_mesh()
        .x_desc("Simulation time (s)")
        .y_desc("Cylinder vₓ (m/s)")
        .axis_desc_style(("sans-serif", 15))
        .max_light_lines(0)
        .bold_line_style(RGBColor(223, 229, 232))
        .draw()?;
    if let Some(&(t, v)) = velocity.iter().find(|p| p.1 > (hi + lo) / 2.) {
        chart.draw_series(std::iter::once(Text::new(
            format!("{v:.3} m/s after restore"),
            (t + 0.025, v - margin * 0.55),
            ("sans-serif", 16),
        )))?;
    }
    chart.draw_series(LineSeries::new(
        velocity,
        RGBColor(66, 123, 88).stroke_width(3),
    ))?;
    root.present()?;
    Ok(())
}
