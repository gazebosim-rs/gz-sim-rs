// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use plotters::{coord::Shift, prelude::*};
use serde::Deserialize;
use serde_json::Value;
use std::{error::Error, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Deserialize)]
struct Body {
    position: [f64; 3],
    orientation: [f64; 4],
}

impl Body {
    fn errors(&self, reference: &Self) -> [f64; 2] {
        let displacement = DVec3::from(self.position) - DVec3::from(reference.position);
        let delta = DQuat::from_array(self.orientation).conjugate()
            * DQuat::from_array(reference.orientation);
        [
            displacement.length() * 1e3,
            (2. * delta.xyz().length().atan2(delta.w.abs())).to_degrees(),
        ]
    }
}

#[derive(Deserialize)]
struct Lane {
    foot: Body,
    upper: Body,
    normal_impulse_ns: f64,
    edge_distance_m: f64,
}

#[derive(Deserialize)]
struct Frame {
    tick: u64,
    time_s: f64,
    switched: Lane,
    reference: Lane,
}

struct Sample {
    ms: f64,
    values: [[f64; 2]; 4],
}

const COLORS: [RGBColor; 2] = [RGBColor(0, 109, 115), RGBColor(190, 86, 30)];

fn panel(
    area: &DrawingArea<SVGBackend, Shift>,
    samples: &[Sample],
    index: usize,
    caption: &str,
    unit: &str,
    labels: [&str; 2],
    events: [f64; 2],
) -> Result<()> {
    let start = samples.first().ok_or("empty plot interval")?.ms;
    let end = samples.last().ok_or("empty plot interval")?.ms;
    let mut low = 0.0_f64;
    let mut high = 0.0_f64;
    for value in samples.iter().flat_map(|p| p.values[index]) {
        if !value.is_finite() {
            return Err("nonfinite rocking-foot measurement".into());
        }
        low = low.min(value);
        high = high.max(value);
    }
    let pad = ((high - low) * 0.08).max(1e-9);
    let bounds = low - pad..high + pad;
    let mut chart = ChartBuilder::on(area)
        .margin(10)
        .caption(caption, ("sans-serif", 19))
        .x_label_area_size(40)
        .y_label_area_size(92)
        .build_cartesian_2d(start..end, bounds.clone())?;
    chart
        .configure_mesh()
        .x_desc("Time (ms)")
        .y_desc(unit)
        .max_light_lines(1)
        .light_line_style(RGBColor(224, 229, 231))
        .draw()?;
    for ms in events {
        if (start..=end).contains(&ms) {
            chart.draw_series(LineSeries::new(
                [(ms, bounds.start), (ms, bounds.end)],
                RGBColor(155, 164, 174),
            ))?;
        }
    }
    for (lane, label) in labels.into_iter().enumerate() {
        let style = COLORS[lane].stroke_width(2);
        chart
            .draw_series(LineSeries::new(
                samples.iter().map(|p| (p.ms, p.values[index][lane])),
                style,
            ))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], style));
        if index == 1 {
            chart.draw_series(
                samples
                    .iter()
                    .map(|p| Circle::new((p.ms, p.values[index][lane]), 3, COLORS[lane].filled())),
            )?;
        }
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.85))
        .draw()?;
    Ok(())
}

pub(super) fn draw(data: &Value, summary: &Value, output: &Path) -> Result<()> {
    let frames: Vec<Frame> = Deserialize::deserialize(&data["frames"])?;
    let event = |name: &str| -> Result<usize> {
        let tick = summary[name]
            .as_u64()
            .ok_or_else(|| format!("missing {name}"))?;
        frames
            .binary_search_by_key(&tick, |f| f.tick)
            .map_err(|_| format!("missing frame for {name}").into())
    };
    let crossing = event("crossing_tick")?;
    let first = event("promotion_tick")?;
    let events = [frames[crossing].time_s * 1e3, frames[first].time_s * 1e3];
    let samples: Vec<_> = frames
        .iter()
        .map(|f| {
            let foot = f.switched.foot.errors(&f.reference.foot);
            let upper = f.switched.upper.errors(&f.reference.upper);
            Sample {
                ms: f.time_s * 1e3,
                values: [
                    [
                        f.switched.edge_distance_m * 1e3,
                        f.reference.edge_distance_m * 1e3,
                    ],
                    [
                        f.switched.normal_impulse_ns * 1e3,
                        f.reference.normal_impulse_ns * 1e3,
                    ],
                    [foot[0], upper[0]],
                    [foot[1], upper[1]],
                ],
            }
        })
        .collect();
    let area = SVGBackend::new(output, (1070, 1050)).into_drawing_area();
    area.fill(&WHITE)?;
    let title = format!(
        "Rocking foot  |  Edge crossing {:.3} ms  ·  first resumed solve {:.3} ms",
        events[0], events[1]
    );
    let plots = area
        .titled(&title, ("sans-serif", 23))?
        .split_evenly((4, 1));
    let lane_names = ["Switched", "Always dynamic"];
    panel(
        &plots[0],
        &samples,
        0,
        "Outside toe corner relative to the support edge",
        "Past edge (mm)",
        lane_names,
        events,
    )?;
    let window = &samples[first.saturating_sub(4)..(first + 12).min(samples.len())];
    panel(
        &plots[1],
        window,
        1,
        "Foot–ledge normal impulse around the first resumed solve",
        "Impulse (mN·s)",
        lane_names,
        events,
    )?;
    panel(
        &plots[2],
        &samples,
        2,
        "Center-position difference from the always-dynamic run",
        "Distance (mm)",
        ["Foot", "Upper link"],
        events,
    )?;
    panel(
        &plots[3],
        &samples,
        3,
        "Angular difference from the always-dynamic run",
        "Angle (degrees)",
        ["Foot", "Upper link"],
        events,
    )?;
    area.present()?;
    Ok(())
}
