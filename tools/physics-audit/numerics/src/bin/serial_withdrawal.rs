// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot measured shelf-withdrawal trials from the serial_place timing sweep.
use plotters::{coord::Shift, prelude::*};
use serde_json::Value;
use std::{error::Error, fs};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn panel(
    area: &DrawingArea<SVGBackend<'_>, Shift>,
    points: &[(u32, f64, bool)],
    title: &str,
    max_delay: u32,
    max_drop: f64,
) -> Result<()> {
    let mut chart = ChartBuilder::on(area)
        .caption(title, ("sans-serif", 20))
        .margin(14)
        .x_label_area_size(48)
        .y_label_area_size(57)
        .build_cartesian_2d(0..max_delay, 0.0..max_drop)?;
    chart
        .configure_mesh()
        .x_desc("Additional reaction delay (ms)")
        .y_desc("Maximum drop (mm)")
        .light_line_style(RGBColor(229, 233, 234))
        .draw()?;
    chart.draw_series(
        points
            .iter()
            .filter(|(t, h, _)| *t <= max_delay && *h <= max_drop)
            .map(|&(t, h, placed)| {
                Circle::new(
                    (t, h),
                    4,
                    if placed {
                        RGBColor(15, 118, 110)
                    } else {
                        RGBColor(183, 72, 50)
                    }
                    .filled(),
                )
            }),
    )?;
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [log, output] = args.as_slice() else {
        return Err("expected sweep log and destination SVG".into());
    };
    let mut points = Vec::new();
    for line in fs::read_to_string(log)?
        .lines()
        .filter_map(|s| s.strip_prefix("delay_ms="))
    {
        let (delay, body) = line.split_once(' ').ok_or("missing outcome")?;
        let value: Value = serde_json::from_str(body)?;
        let r = &value["recovery"];
        points.push((
            delay.parse()?,
            r["max_drop_before_return_mm"]
                .as_f64()
                .ok_or("missing drop")?,
            r["final_open_tick"].is_u64(),
        ));
    }
    if points.is_empty() {
        return Err("no measured withdrawal trials".into());
    }
    points.sort_by_key(|p| p.0);
    let root = SVGBackend::new(output, (1040, 490)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "How far does the parcel fall while the shelf moves away?",
        ("sans-serif", 27),
    )?;
    let (plots, foot) = body.split_vertically(392);
    let (left, right) = plots.split_horizontally(520);
    panel(&left, &points, "First 50 milliseconds", 52, 6.)?;
    panel(&right, &points, "Full measured range", 305, 95.)?;
    foot.draw(&Circle::new((127, 18), 5, RGBColor(15, 118, 110).filled()))?;
    foot.draw(&Text::new(
        "Second release commanded",
        (142, 24),
        ("sans-serif", 16),
    ))?;
    foot.draw(&Circle::new((476, 18), 5, RGBColor(183, 72, 50).filled()))?;
    foot.draw(&Text::new(
        "Clearance check prevented second release",
        (491, 24),
        ("sans-serif", 16),
    ))?;
    root.present()?;
    Ok(())
}
