// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Value, read_trace};
use plotters::prelude::*;
use std::path::Path;

pub fn draw(dir: &Path, runs: &[Value]) -> Result<()> {
    let path = dir.join("convergence.svg");
    let root = SVGBackend::new(&path, (1150, 550)).into_drawing_area();
    root.fill(&WHITE)?;
    let ink = RGBColor(41, 51, 60);
    let drawing = root.titled(
        "Floating hinge convergence at fixed physical inputs",
        ("sans-serif", 29).into_font().color(&ink),
    )?;
    let panels = drawing.margin(12, 18, 15, 15).split_evenly((1, 2));
    let mut history = ChartBuilder::on(&panels[0])
        .caption(
            "Hinge angle error through the maneuver",
            ("sans-serif", 19).into_font(),
        )
        .margin(12)
        .x_label_area_size(54)
        .y_label_area_size(67)
        .build_cartesian_2d(0_u32..376_u32, 0_f64..0.65_f64)?;
    history
        .configure_mesh()
        .x_labels(6)
        .y_labels(7)
        .max_light_lines(1)
        .label_style(("sans-serif", 14))
        .axis_desc_style(("sans-serif", 15))
        .bold_line_style(RGBColor(223, 228, 232))
        .x_desc("Physical time (ms)")
        .y_desc("Absolute angle error (mrad)")
        .light_line_style(RGBColor(227, 232, 236))
        .draw()?;
    history.draw_series(std::iter::once(PathElement::new(
        vec![(120, 0.), (120, 0.65)],
        RGBColor(143, 152, 158).mix(0.72),
    )))?;
    history.draw_series(std::iter::once(Text::new(
        "cut",
        (124, 0.59),
        ("sans-serif", 14).into_font().color(&ink),
    )))?;
    for (ns, label, color) in [
        (1_000_000, "1,000 µs", RGBColor(168, 73, 42)),
        (250_000, "250 µs", RGBColor(50, 99, 148)),
        (31_250, "31.25 µs", RGBColor(35, 133, 103)),
    ] {
        let series = read_trace(&dir.join(format!("{ns}-ns.json")))?;
        let points = series
            .as_array()
            .ok_or("invalid hinge observations")?
            .iter()
            .map(|r| {
                (
                    r["time_ms"].as_u64().unwrap() as u32,
                    1000. * r["error"]["q_rad"].as_f64().unwrap(),
                )
            });
        history
            .draw_series(LineSeries::new(points, color.stroke_width(3)))?
            .label(label)
            .legend(move |(x, y)| {
                PathElement::new(vec![(x, y), (x + 19, y)], color.stroke_width(3))
            });
    }
    history
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.88))
        .border_style(RGBColor(211, 217, 223))
        .draw()?;
    let mut drift = ChartBuilder::on(&panels[1])
        .caption(
            "Peak angular momentum drift after the cut",
            ("sans-serif", 19).into_font(),
        )
        .margin(12)
        .x_label_area_size(54)
        .y_label_area_size(76)
        .build_cartesian_2d(0_f64..1050_f64, 0_f64..395_f64)?;
    drift
        .configure_mesh()
        .x_labels(6)
        .y_labels(7)
        .max_light_lines(1)
        .label_style(("sans-serif", 14))
        .axis_desc_style(("sans-serif", 15))
        .bold_line_style(RGBColor(223, 228, 232))
        .x_desc("Native integration step (µs)")
        .y_desc("Drift about world origin (µN·m·s)")
        .light_line_style(RGBColor(227, 232, 236))
        .draw()?;
    let samples = runs
        .iter()
        .map(|r| {
            (
                r["dt_ns"].as_f64().unwrap() / 1000.,
                1e6 * r["from_cut"]["post_cut_angular_drift_nms"]
                    .as_f64()
                    .unwrap(),
            )
        })
        .collect::<Vec<_>>();
    let green = RGBColor(29, 116, 97);
    drift.draw_series(LineSeries::new(
        samples.iter().copied(),
        green.stroke_width(2),
    ))?;
    drift.draw_series(samples.iter().map(|&p| Circle::new(p, 5, green.filled())))?;
    root.present()?;
    Ok(())
}
