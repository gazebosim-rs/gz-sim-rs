// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Summary, Trace};
use plotters::prelude::*;
use std::path::Path;

const SWITCHED: RGBColor = RGBColor(11, 123, 115);
const DYNAMIC: RGBColor = RGBColor(52, 92, 176);
const LIFT: RGBColor = RGBColor(175, 101, 24);
const GRID: RGBColor = RGBColor(225, 230, 236);

pub(super) fn draw(t: &Trace, summary: &Summary, out: &Path) -> Result<()> {
    let cycles = &summary.cycles;
    let root = SVGBackend::new(out, (1140, 900)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root
        .titled("Repeated landing on a moving support", ("sans-serif", 25))?
        .split_evenly((3, 1));
    let end = t.frames.last().ok_or("empty trace")?.time_s;
    let height = |s: &super::Step| {
        [
            1000. * (s.switched.pad.position[2] - t.frames[0].switched.pad.position[2]),
            1000. * (s.switched.foot.position[2] - t.frames[0].switched.foot.position[2]),
            1000. * (s.reference.foot.position[2] - t.frames[0].reference.foot.position[2]),
        ]
    };
    let (lo, hi) = t
        .frames
        .iter()
        .flat_map(height)
        .fold((0.0_f64, 0.0_f64), |(lo, hi), z| (lo.min(z), hi.max(z)));
    let margin = ((hi - lo) * 0.10).max(0.01);
    let mut chart = ChartBuilder::on(&panels[0])
        .caption(
            "Lift and foot displacement from the initial settled pose",
            ("sans-serif", 19),
        )
        .margin(12)
        .x_label_area_size(41)
        .y_label_area_size(85)
        .build_cartesian_2d(0.0..end, lo - margin..hi + margin)?;
    chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc("Vertical travel (mm)")
        .max_light_lines(1)
        .light_line_style(GRID)
        .draw()?;
    for (idx, (label, color)) in [("Lift", LIFT), ("Switched foot", SWITCHED)]
        .into_iter()
        .enumerate()
    {
        let pen = color.stroke_width(2);
        chart
            .draw_series(LineSeries::new(
                t.frames.iter().map(|s| (s.time_s, height(s)[idx])),
                pen,
            ))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 16, y)], pen));
    }
    chart
        .draw_series(
            t.frames
                .iter()
                .step_by(t.frames.len().div_ceil(105))
                .map(|s| Circle::new((s.time_s, height(s)[2]), 2, DYNAMIC.filled())),
        )?
        .label("Always-dynamic foot")
        .legend(|(x, y)| Circle::new((x + 8, y), 3, DYNAMIC.filled()));
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.94))
        .position(SeriesLabelPosition::LowerRight)
        .draw()?;
    let mut tiers = ChartBuilder::on(&panels[1])
        .caption("Committed tier of the switching foot", ("sans-serif", 19))
        .margin(12)
        .x_label_area_size(41)
        .y_label_area_size(85)
        .build_cartesian_2d(0.0..end, -0.15..1.24)?;
    tiers
        .configure_mesh()
        .x_desc("Time (s)")
        .y_labels(2)
        .y_label_formatter(&|v: &f64| {
            if (*v - 1.).abs() < 0.01 {
                "Dynamic".into()
            } else if v.abs() < 0.01 {
                "Prescribed".into()
            } else {
                String::new()
            }
        })
        .max_light_lines(1)
        .light_line_style(GRID)
        .draw()?;
    let mut states = Vec::new();
    let mut old = f64::from(!t.frames[0].prescribed);
    states.push((0., old));
    for s in &t.frames {
        let new = f64::from(!s.prescribed);
        if old != new {
            states.extend([(s.time_s, old), (s.time_s, new)]);
            old = new;
        }
    }
    states.push((end, old));
    tiers.draw_series(LineSeries::new(states, SWITCHED.stroke_width(2)))?;
    let max = cycles
        .iter()
        .flat_map(|c| {
            [
                c.switched_landing.window_normal_impulse_ns,
                c.reference_landing.window_normal_impulse_ns,
            ]
        })
        .fold(0., f64::max);
    let count = cycles.len() as f64;
    let window_caption = format!(
        "First {:.1} ms of each landing",
        summary.window_steps as f64 * t.dt_s * 1000.
    );
    for (panel, (caption, window)) in panels[2].split_evenly((1, 2)).iter().zip([
        ("First landing solve", false),
        (window_caption.as_str(), true),
    ]) {
        let mut land = ChartBuilder::on(panel)
            .caption(caption, ("sans-serif", 19))
            .margin(12)
            .x_label_area_size(41)
            .y_label_area_size(70)
            .build_cartesian_2d(0.5..count + 0.5, 0.0..(1.30 * max))?;
        land.configure_mesh()
            .x_desc("Cycle")
            .x_labels(cycles.len() + 1)
            .x_label_formatter(&|x: &f64| {
                if x.fract().abs() < 1e-8 {
                    format!("{x:.0}")
                } else {
                    String::new()
                }
            })
            .y_desc("Normal impulse (N·s)")
            .max_light_lines(1)
            .light_line_style(GRID)
            .draw()?;
        for (name, color, offset) in [
            ("Switched", SWITCHED, -0.11),
            ("Always dynamic", DYNAMIC, 0.11),
        ] {
            land.draw_series(cycles.iter().enumerate().map(|(i, c)| {
                let x = i as f64 + 1. + offset;
                let l = if offset < 0. {
                    &c.switched_landing
                } else {
                    &c.reference_landing
                };
                let value = if window {
                    l.window_normal_impulse_ns
                } else {
                    l.normal_impulse_ns
                };
                Rectangle::new([(x - 0.095, 0.), (x + 0.095, value)], color.filled())
            }))?
            .label(name)
            .legend(move |(x, y)| Rectangle::new([(x, y - 4), (x + 15, y + 4)], color.filled()));
        }
        if !window {
            land.configure_series_labels()
                .background_style(WHITE.mix(0.94))
                .position(SeriesLabelPosition::UpperRight)
                .draw()?;
        }
    }
    root.present()?;
    Ok(())
}
