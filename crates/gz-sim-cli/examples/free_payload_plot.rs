// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot the velocity reversal and hinge response from free_payload_promotion's three local runs.
use plotters::{coord::Shift, prelude::*};
use serde::Deserialize;
use std::{error::Error, fs::File, io::BufReader, path::PathBuf};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Deserialize)]
struct Trace {
    dt_s: f64,
    samples: Vec<Sample>,
}
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    tier: String,
    automatic_impulse_ns: f64,
    hinge_rate_rad_s: f64,
    reference_hinge_rate_rad_s: f64,
    near_miss_hinge_rate_rad_s: f64,
    automatic: [Link; 3],
    reference: [Link; 3],
    near_miss: [Link; 3],
}
#[derive(Deserialize)]
struct Link {
    velocity: [f64; 3],
}

fn panel(
    area: &DrawingArea<SVGBackend, Shift>,
    trace: &Trace,
    events_ms: [f64; 2],
    title: &str,
    units: &str,
    values: impl Fn(&Sample) -> [f64; 3],
) -> Result<()> {
    let time = |s: &Sample| s.tick as f64 * trace.dt_s * 1000.;
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for s in &trace.samples {
        for v in values(s) {
            if !v.is_finite() {
                return Err("nonfinite sample".into());
            }
            low = low.min(v);
            high = high.max(v);
        }
    }
    let padding = ((high - low) * 0.15).max(0.01);
    let last = time(trace.samples.last().ok_or("empty physics trace")?);
    let mut chart = ChartBuilder::on(area)
        .margin(12)
        .caption(title, ("sans-serif", 22))
        .x_label_area_size(43)
        .y_label_area_size(65)
        .build_cartesian_2d(0.0..last, low - padding..high + padding)?;
    chart
        .configure_mesh()
        .x_desc("Simulation time (ms)")
        .y_desc(units)
        .light_line_style(RGBColor(232, 237, 238))
        .max_light_lines(1)
        .draw()?;
    for (time, color) in events_ms
        .into_iter()
        .zip([RGBColor(76, 145, 80), RGBColor(148, 74, 90)])
    {
        chart.draw_series(LineSeries::new(
            [(time, low - padding), (time, high + padding)],
            color,
        ))?;
    }
    let amber = RGBColor(195, 116, 24);
    let teal = RGBColor(15, 104, 120);
    chart
        .draw_series(LineSeries::new(
            trace.samples.iter().map(|s| (time(s), values(s)[2])),
            amber.stroke_width(2),
        ))?
        .label("Near miss")
        .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], amber.stroke_width(2)));
    chart
        .draw_series(LineSeries::new(
            trace.samples.iter().map(|s| (time(s), values(s)[0])),
            teal.stroke_width(3),
        ))?
        .label("Automatic")
        .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], teal.stroke_width(3)));
    let stride = (trace.samples.len() / 26).max(1);
    chart
        .draw_series(
            trace
                .samples
                .iter()
                .step_by(stride)
                .map(|s| Circle::new((time(s), values(s)[1]), 3, BLACK.stroke_width(1))),
        )?
        .label("Always dynamic")
        .legend(|(x, y)| Circle::new((x + 8, y), 3, BLACK.stroke_width(1)));
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.92))
        .border_style(RGBColor(215, 222, 225))
        .draw()?;
    Ok(())
}

fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1).map(PathBuf::from);
    let source = args
        .next()
        .ok_or("expected trajectory.json and output.svg")?;
    let output = args.next().ok_or("missing output.svg")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let trace: Trace = serde_json::from_reader(BufReader::new(File::open(source)?))?;
    let tick = trace
        .samples
        .iter()
        .find(|s| s.tier == "dynamic")
        .ok_or("no promotion")?
        .tick;
    let contact = trace
        .samples
        .iter()
        .find(|s| s.automatic_impulse_ns > 0.)
        .ok_or("no contact impulse")?
        .tick;
    let events_ms = [tick, contact].map(|value| value as f64 * trace.dt_s * 1000.);
    let root = SVGBackend::new(&output, (1050, 580)).into_drawing_area();
    root.fill(&WHITE)?;
    let title = format!(
        "Free payload striking a two-link arm  |  promotion at {tick}, impact at {contact} (ticks)"
    );
    let body = root.titled(&title, ("sans-serif", 26))?;
    let panels = body.split_evenly((2, 1));
    panel(
        &panels[0],
        &trace,
        events_ms,
        "Payload velocity along the approach axis",
        "m/s",
        |s| {
            [
                s.automatic[2].velocity[0],
                s.reference[2].velocity[0],
                s.near_miss[2].velocity[0],
            ]
        },
    )?;
    panel(
        &panels[1],
        &trace,
        events_ms,
        "Hinge angular velocity",
        "rad/s",
        |s| {
            [
                s.hinge_rate_rad_s,
                s.reference_hinge_rate_rad_s,
                s.near_miss_hinge_rate_rad_s,
            ]
        },
    )?;
    root.present()?;
    Ok(())
}
