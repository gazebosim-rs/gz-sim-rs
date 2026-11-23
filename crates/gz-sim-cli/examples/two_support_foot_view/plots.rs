// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Events, Result, Sample, Trace};
use glam::DVec3;
use plotters::{coord::Shift, prelude::*};
use std::path::Path;

const COLORS: [RGBColor; 2] = [RGBColor(0, 117, 117), RGBColor(187, 79, 31)];

struct Axis<'a> {
    title: &'a str,
    unit: &'a str,
    labels: [&'a str; 2],
    marks: bool,
}
fn panel(
    area: &DrawingArea<SVGBackend<'_>, Shift>,
    rows: &[Sample],
    axis: Axis<'_>,
    events: &Events,
    dt: f64,
    values: impl Fn(&Sample) -> [f64; 2],
) -> Result<()> {
    let first = rows.first().ok_or("empty plot interval")?.time_s * 1e3;
    let last = rows.last().unwrap().time_s * 1e3;
    let high = rows.iter().flat_map(&values).fold(0., f64::max).max(1e-6);
    let mut c = ChartBuilder::on(area)
        .margin(12)
        .caption(axis.title, ("sans-serif", 20))
        .x_label_area_size(42)
        .y_label_area_size(80)
        .build_cartesian_2d(first..last.max(first + dt * 1e3), -0.03 * high..1.15 * high)?;
    c.configure_mesh()
        .x_desc("Time (ms)")
        .y_desc(axis.unit)
        .max_light_lines(1)
        .light_line_style(RGBColor(225, 230, 233))
        .draw()?;
    for (event, color) in [
        (events.loss_tick, RGBColor(162, 116, 62)),
        (events.promotion_tick, RGBColor(86, 104, 127)),
    ] {
        let ms = event as f64 * dt * 1e3;
        if (first..=last).contains(&ms) {
            c.draw_series(LineSeries::new(
                [(ms, 0.), (ms, 1.10 * high)],
                color.mix(0.55),
            ))?;
        }
    }
    for (i, label) in axis.labels.into_iter().enumerate() {
        let series = rows.iter().map(|s| (s.time_s * 1e3, values(s)[i]));
        let color = COLORS[i];
        let style = color.stroke_width(2);
        c.draw_series(LineSeries::new(series.clone(), style))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], style));
        if axis.marks {
            c.draw_series(series.map(|p| Circle::new(p, 3, color.filled())))?;
        }
    }
    c.configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.87))
        .draw()?;
    Ok(())
}

pub(super) fn draw(trace: &Trace, events: &Events, out: &Path) -> Result<()> {
    let root = SVGBackend::new(out, (1140, 1020)).into_drawing_area();
    root.fill(&WHITE)?;
    let ms = trace.dt_s * 1000.;
    let title = format!(
        "Two-support foot   |   first step without B {:.3} ms, first resumed step {:.3} ms",
        events.loss_tick as f64 * ms,
        events.promotion_tick as f64 * ms
    );
    let panels = root
        .titled(&title, ("sans-serif", 25))?
        .split_evenly((4, 1));
    let labels = ["Switched", "Always dynamic"];
    for (index, title) in ["Surviving support A", "Removed support B"]
        .into_iter()
        .enumerate()
    {
        panel(
            &panels[index],
            &trace.frames,
            Axis {
                title,
                unit: "Normal impulse (mN·s)",
                labels,
                marks: false,
            },
            events,
            trace.dt_s,
            |s| {
                [
                    s.switched.normal_impulses_ns[index] * 1e3,
                    s.reference.normal_impulses_ns[index] * 1e3,
                ]
            },
        )?;
    }
    let first = events.promotion_tick as usize;
    let end = (first + 12).min(trace.frames.len());
    panel(
        &panels[2],
        &trace.frames[first..end],
        Axis {
            title: "Per-support normal impulse error in the first resumed dynamic steps",
            unit: "Absolute error (µN·s)",
            labels: ["Support A", "Support B"],
            marks: true,
        },
        events,
        trace.dt_s,
        |s| {
            [0, 1].map(|i| {
                1e6 * (s.switched.normal_impulses_ns[i] - s.reference.normal_impulses_ns[i]).abs()
            })
        },
    )?;
    let displacement = |s: &Sample| {
        [
            (&s.switched.foot, &s.reference.foot),
            (&s.switched.upper, &s.reference.upper),
        ]
        .map(|(a, b)| DVec3::from(a.position).distance(b.position.into()))
    };
    let peak_m = trace
        .frames
        .iter()
        .flat_map(&displacement)
        .fold(0., f64::max);
    let (distance_unit, distance_scale) = if peak_m < 1e-6 {
        ("Distance (nm)", 1e9)
    } else {
        ("Distance (mm)", 1e3)
    };
    panel(
        &panels[3],
        &trace.frames,
        Axis {
            title: "Center displacement from the independently stepped always-dynamic run",
            unit: distance_unit,
            labels: ["Foot", "Upper link"],
            marks: false,
        },
        events,
        trace.dt_s,
        |s| displacement(s).map(|value| value * distance_scale),
    )?;
    root.present()?;
    Ok(())
}
