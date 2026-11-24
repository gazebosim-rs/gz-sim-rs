// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use plotters::prelude::*;

pub(super) fn write(path: &std::path::Path, cases: &[(&str, f64, Estimate)]) -> Result<()> {
    let panel = SVGBackend::new(path, (920, 460)).into_drawing_area();
    panel.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&panel)
        .caption("Weighing the cable at pickup", ("sans-serif", 26))
        .margin(18)
        .x_label_area_size(57)
        .y_label_area_size(64)
        .build_cartesian_2d(0.0..cases.len() as f64, 0.0..38.0)?;
    chart
        .configure_mesh()
        .disable_x_mesh()
        .x_labels(0)
        .max_light_lines(0)
        .y_desc("Mass (g)")
        .draw()?;
    for (series, label, color) in [
        (0, "Authored cable", RGBColor(91, 97, 112)),
        (1, "Vertical jaw force", RGBColor(0, 119, 140)),
        (2, "Horizontal inertia", RGBColor(193, 97, 44)),
    ] {
        let mut bars = Vec::new();
        for (i, (_, mass, est)) in cases.iter().enumerate() {
            let kg = match series {
                0 => Some(*mass),
                1 => est.apparent_weight_mass_kg,
                _ => est.inertial_mass_kg,
            }
            .ok_or("missing comparison estimate to plot")?;
            let x = i as f64 + 0.15 + 0.24 * series as f64;
            bars.push(Rectangle::new(
                [(x, 0.), (x + 0.20, kg * 1000.)],
                color.filled(),
            ));
        }
        chart
            .draw_series(bars)?
            .label(label)
            .legend(move |(x, y)| Rectangle::new([(x, y - 5), (x + 18, y + 5)], color.filled()));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    let baseline = chart.as_coord_spec().translate(&(0., 0.)).1;
    for (i, (name, _, _)) in cases.iter().enumerate() {
        let x = chart.as_coord_spec().translate(&(i as f64 + 0.30, 0.)).0;
        panel.draw(&Text::new(*name, (x, baseline + 20), ("sans-serif", 17)))?;
    }
    panel.present()?;
    Ok(())
}
