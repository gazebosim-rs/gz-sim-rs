// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare signed public-tick impulses around a tool's touchdown and release.
use plotters::prelude::*;
use serde_json::Value;
use std::{error::Error, fs, io::Write, ops::RangeInclusive, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOLVERS: [(&str, &str, RGBColor); 3] = [
    ("gpu", "GPU", RGBColor(22, 101, 126)),
    ("rapier", "Rapier", RGBColor(191, 75, 26)),
    ("cartesian", "Fine Cartesian", RGBColor(103, 74, 143)),
];
struct Study {
    lane: u64,
    mu: f64,
    mass: f64,
    lift: u64,
    ticks: Vec<[[f64; 2]; 3]>,
    first: [usize; 3],
    last: [usize; 3],
}
fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .ok_or_else(|| "missing numeric observation".into())
}
fn case(rows: &Value, lane: u64) -> Result<&Value> {
    rows.as_array()
        .ok_or("missing cases")?
        .iter()
        .find(|r| r["lane_id"].as_u64() == Some(lane))
        .ok_or_else(|| "missing lane".into())
}
fn read(report: &Value, trace: &Value, lane: u64) -> Result<Study> {
    let observed = case(&report["cases"], lane)?;
    let mut first = [0; 3];
    let mut last = [0; 3];
    let mut totals = [[0.; 2]; 3];
    let mut ticks = Vec::new();
    for (i, row) in trace.as_array().ok_or("missing trace")?.iter().enumerate() {
        let tick = i + 1;
        if row["tick"].as_u64() != Some(tick as u64) {
            return Err("missing public tick".into());
        }
        let r = case(&row["cases"], lane)?;
        let mut bin = [[0.; 2]; 3];
        for (j, (k, _, _)) in SOLVERS.iter().enumerate() {
            bin[j] = [number(&r[k][2])?, number(&r[k][3])?];
            if bin[j][0] < 0. {
                return Err("negative normal impulse".into());
            }
            if bin[j][0] > 0. {
                if first[j] == 0 {
                    first[j] = tick;
                }
                last[j] = tick;
            }
            for (sum, v) in totals[j].iter_mut().zip(bin[j]) {
                *sum += v;
            }
        }
        ticks.push(bin);
    }
    if report["ticks"].as_u64() != Some(ticks.len() as u64) || first.contains(&0) {
        return Err("incomplete contact comparison".into());
    }
    for (j, (k, _, _)) in SOLVERS.iter().enumerate() {
        for (v, key) in totals[j]
            .into_iter()
            .zip(["total_normal_ns", "total_tangent_ns"])
        {
            if (v - number(&observed[k][key])?).abs() > 1e-10 * (1. + v.abs()) {
                return Err("public impulses disagree with the trial totals".into());
            }
        }
    }
    Ok(Study {
        lane,
        mu: number(&observed["friction"])?,
        mass: number(&observed["payload_kg"])?,
        lift: report["lift_before_tick"]
            .as_u64()
            .ok_or("missing lift command")?,
        ticks,
        first,
        last,
    })
}
fn window(events: [usize; 3], ticks: usize) -> RangeInclusive<usize> {
    let lo = events.into_iter().min().unwrap().saturating_sub(5).max(1);
    let hi = (events.into_iter().max().unwrap() + 8).min(ticks);
    lo..=hi
}
fn draw(s: &Study) -> Result<String> {
    let mut svg = String::new();
    {
        let page = SVGBackend::with_string(&mut svg, (1100, 670)).into_drawing_area();
        page.fill(&WHITE)?;
        let (heading, rest) = page.split_vertically(112);
        for (text, point, size) in [
            (
                "Ground reaction at touchdown and lift-off".to_owned(),
                (28, 15),
                29,
            ),
            (
                format!(
                    "Lane {}   ·   {:.2} kg payload   ·   friction coefficient {:.2}   ·   lift commanded before tick {}",
                    s.lane, s.mass, s.mu, s.lift
                ),
                (30, 55),
                17,
            ),
        ] {
            heading.draw(&Text::new(text, point, ("sans-serif", size)))?;
        }
        for (i, ((_, name, color), x)) in SOLVERS.iter().zip([34, 191, 364]).enumerate() {
            match i {
                0 => heading.draw(&Circle::new((x, 96), 4, color.stroke_width(2)))?,
                1 => heading.draw(&Cross::new((x, 96), 5, color.stroke_width(2)))?,
                _ => heading.draw(&TriangleMarker::new((x, 96), 5, color.stroke_width(1)))?,
            }
            heading.draw(&Text::new(*name, (x + 11, 84), ("sans-serif", 16)))?;
        }
        let (panels, note) = rest.split_vertically(506);
        let areas = panels.split_evenly((2, 2));
        for (i, area) in areas.iter().enumerate() {
            let component = i / 2;
            let touchdown = i % 2 == 0;
            let ticks = window(if touchdown { s.first } else { s.last }, s.ticks.len());
            let bounds = ticks
                .clone()
                .flat_map(|t| s.ticks[t - 1].map(|q| q[component] * 1000.))
                .fold((0_f64, 0_f64), |(a, b), x| (a.min(x), b.max(x)));
            let spare = (bounds.1 - bounds.0).max(0.01) * 0.12;
            let title = match (component, touchdown) {
                (0, true) => "Touchdown · upward",
                (0, false) => "Lift-off · upward",
                (_, true) => "Touchdown · sideways",
                (_, false) => "Lift-off · sideways",
            };
            let mut chart = ChartBuilder::on(area)
                .margin(8)
                .caption(title, ("sans-serif", 21))
                .x_label_area_size(38)
                .y_label_area_size(66)
                .build_cartesian_2d(
                    *ticks.start() as f64 - 0.5..*ticks.end() as f64 + 0.5,
                    bounds.0 - spare..bounds.1 + spare,
                )?;
            chart
                .configure_mesh()
                .x_desc("Time at end of step (ms)")
                .y_desc("Impulse (mN·s)")
                .x_labels(7)
                .y_labels(5)
                .max_light_lines(0)
                .bold_line_style(RGBColor(222, 226, 227))
                .draw()?;
            for (j, (_, _, color)) in SOLVERS.iter().enumerate() {
                let points = ticks
                    .clone()
                    .map(|t| (t as f64, 1000. * s.ticks[t - 1][j][component]));
                chart.draw_series(LineSeries::new(points.clone(), color.stroke_width(2)))?;
                match j {
                    0 => {
                        chart.draw_series(
                            points.map(|p| Circle::new(p, 3, color.stroke_width(2))),
                        )?;
                    }
                    1 => {
                        chart
                            .draw_series(points.map(|p| Cross::new(p, 4, color.stroke_width(2))))?;
                    }
                    _ => {
                        chart.draw_series(
                            points.map(|p| TriangleMarker::new(p, 4, color.stroke_width(1))),
                        )?;
                    }
                }
            }
        }
        note.draw(&Text::new("Each point sums the tool impulse during the preceding 1 ms. Positive sideways impulse acts along world +X.",(29,13),("sans-serif",15)))?;
        page.present()?;
    }
    Ok(svg)
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [root, lane, out] = &args[..] else {
        return Err("usage: gpu_tool_impulses STUDY_DIR LANE_ID NEW_FIGURE.svg".into());
    };
    let load = |name| -> Result<Value> {
        Ok(serde_json::from_slice(&fs::read(
            Path::new(root).join(name),
        )?)?)
    };
    let study = read(
        &load("comparison.json")?,
        &load("trace.json")?,
        lane.parse()?,
    )?;
    let svg = draw(&study)?;
    fs::File::create_new(out)?.write_all(svg.as_bytes())?;
    Ok(())
}
#[cfg(test)]
#[path = "../../tests/support/gpu_tool_impulses.rs"]
mod tests;
