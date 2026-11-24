// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use plotters::prelude::*;

pub(super) fn write(path: &std::path::Path, prior: &[Log], active: &[Log]) -> Result<()> {
    let arrival = active
        .iter()
        .position(|r| r.truth.bar > 1e-8)
        .ok_or("no impact to plot")?;
    let panel = SVGBackend::new(path, (900, 530)).into_drawing_area();
    panel.fill(&WHITE)?;
    let areas = panel.split_evenly((2, 1));
    for (i, area) in areas.iter().enumerate() {
        let observation = |r: &Log| {
            if i == 0 {
                r.truth.robot_bar_gap * 1000.
            } else {
                r.jaws.rail_speed_m_s * 1000.
            }
        };
        let range = |r: &[Log]| {
            r[arrival..=arrival + 200]
                .iter()
                .map(observation)
                .fold((0_f64, 0_f64), |(lo, hi), v| (lo.min(v), hi.max(v)))
        };
        let ((lo_a, hi_a), (lo_b, hi_b)) = (range(prior), range(active));
        let ymax = hi_a.max(hi_b);
        let ymin = lo_a.min(lo_b);
        let mut chart = ChartBuilder::on(area)
            .margin(14)
            .x_label_area_size(38)
            .y_label_area_size(72)
            .caption(
                if i == 0 {
                    "Clearance between the rigid robot and the 700 mm barrier"
                } else {
                    "Forward rail speed during the same stop"
                },
                ("sans-serif", 20),
            )
            .build_cartesian_2d(0.0..200.0, (ymin - 0.05 * (ymax - ymin))..(1.05 * ymax))?;
        chart
            .configure_mesh()
            .max_light_lines(0)
            .x_desc("Time since first cable contact (ms)")
            .y_desc(if i == 0 {
                "Clearance (mm)"
            } else {
                "Speed (mm/s)"
            })
            .draw()?;
        for (rows, color, label) in [
            (prior, RGBColor(161, 64, 32), "Original stop"),
            (active, RGBColor(20, 99, 137), "Active brake"),
        ] {
            let points = rows[arrival..=arrival + 200]
                .iter()
                .enumerate()
                .map(|(tick, r)| (tick as f64, observation(r)));
            chart
                .draw_series(LineSeries::new(points, color.stroke_width(3)))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 20, y)], color.stroke_width(3))
                });
        }
        if i == 0 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::UpperRight)
                .background_style(WHITE.mix(0.9))
                .draw()?;
        }
    }
    panel.present()?;
    Ok(())
}
