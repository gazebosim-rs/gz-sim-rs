// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot tested loads and displacement remaining at the original finger patches.
use plotters::prelude::*;
use serde_json::Value;
use std::{error::Error, fs, io::Write, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const PRIMARY: RGBColor = RGBColor(0, 105, 113);
const FINE: RGBColor = RGBColor(167, 70, 30);

struct Point {
    force_n: f64,
    travel_mm: f64,
    supported: bool,
}

struct Sweep {
    points: Vec<Point>,
    bracket: Option<(f64, f64)>,
}

fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .ok_or_else(|| "missing numeric observation".into())
}

fn sweep(v: &Value, threshold_m: f64, samples: u64) -> Result<Sweep> {
    let cases = v["cases"].as_array().ok_or("missing load cases")?;
    let mut points = Vec::new();
    for row in cases {
        let force_n = row["load_mn"].as_u64().ok_or("missing applied load")? as f64 / 1000.;
        let pad = row["final_min_pad_m"]
            .as_array()
            .filter(|s| s.len() == 2)
            .ok_or("expected both finger patches")?;
        let patch_m = [number(&pad[0])?, number(&pad[1])?];
        let travel = patch_m[0].max(patch_m[1]);
        let supported = row["recovered_support_ticks"]
            .as_u64()
            .filter(|n| *n <= samples)
            .ok_or("invalid final support count")?
            == samples;
        let lasting = row["lasting_slip"]
            .as_bool()
            .ok_or("missing slip classification")?;
        if lasting != (travel > threshold_m) || (force_n == 0. && patch_m != [0.; 2]) {
            return Err(
                "reported classification disagrees with the matched no-pull measurement".into(),
            );
        }
        points.push(Point {
            force_n,
            travel_mm: travel * 1e3,
            supported,
        });
    }
    points.sort_by(|a, b| a.force_n.total_cmp(&b.force_n));
    if points.len() < 2
        || points[0].force_n != 0.
        || points.windows(2).any(|p| p[0].force_n == p[1].force_n)
    {
        return Err("expected one unloaded control and distinct tested loads".into());
    }
    let bracket = if v["bracket_mn"].is_null() {
        if points.iter().any(|p| p.travel_mm > threshold_m * 1e3) {
            return Err("measured slip has no reported bracket".into());
        }
        None
    } else {
        let pair = v["bracket_mn"]
            .as_array()
            .filter(|a| a.len() == 2)
            .ok_or("invalid load interval")?;
        let a = number(&pair[0])? / 1000.;
        let b = number(&pair[1])? / 1000.;
        if !points.windows(2).any(|p| {
            p[0].force_n == a
                && p[1].force_n == b
                && p[0].travel_mm <= threshold_m * 1e3
                && p[1].travel_mm > threshold_m * 1e3
        }) {
            return Err(
                "bracket endpoints must be adjacent measured holding and slipping loads".into(),
            );
        }
        Some((a, b))
    };
    Ok(Sweep { points, bracket })
}

fn draw(report: &Value) -> Result<String> {
    let threshold = number(&report["slip_m"])?;
    let window = report["window_ticks"]
        .as_u64()
        .filter(|n| *n > 0)
        .ok_or("invalid sampling window")?;
    let dt = report["dt_ns"]
        .as_u64()
        .filter(|n| *n > 0)
        .ok_or("missing public step")?;
    if threshold <= 0. {
        return Err("invalid slip threshold".into());
    }
    let trials = report["trials"]
        .as_array()
        .filter(|t| t.len() == 2)
        .ok_or("expected two mesh trials")?;
    let mut meshes = Vec::new();
    for columns in [5, 7] {
        let mesh = trials
            .iter()
            .find(|t| t["columns"] == columns)
            .ok_or("missing mesh")?;
        meshes.push([
            sweep(&mesh["primary"], threshold, window)?,
            sweep(&mesh["fine"], threshold, window)?,
        ]);
    }
    let mut svg = String::new();
    {
        let page = SVGBackend::with_string(&mut svg, (1100, 630)).into_drawing_area();
        page.fill(&WHITE)?;
        let (title, content) = page.split_vertically(114);
        title.draw(&Text::new(
            "When does the folded sheet start slipping?",
            (32, 24),
            ("sans-serif", 29),
        ))?;
        title.draw(&Text::new(format!("Each point shows travel sustained by either finger patch against the unloaded control throughout the last {:.0} ms of recovery.",window as f64*dt as f64/1e6),(34,62),("sans-serif",16)))?;
        title.draw(&Circle::new((42, 98), 6, PRIMARY.stroke_width(2)))?;
        title.draw(&Text::new("Primary clock", (54, 88), ("sans-serif", 15)))?;
        title.draw(&Cross::new((191, 98), 6, FINE.stroke_width(2)))?;
        title.draw(&Text::new("Finer clock", (203, 88), ("sans-serif", 15)))?;
        title.draw(&Rectangle::new([(347, 92), (357, 102)], BLACK))?;
        title.draw(&Text::new(
            "Incomplete support during recovery",
            (366, 88),
            ("sans-serif", 15),
        ))?;
        title.draw(&PathElement::new(
            [(660, 98), (688, 98)],
            RGBColor(95, 79, 105).stroke_width(2),
        ))?;
        title.draw(&Text::new(
            format!("Slip criterion  {:.2} mm", threshold * 1e3),
            (698, 88),
            ("sans-serif", 15),
        ))?;
        let (charts, notes) = content.split_vertically(394);
        let panes = charts.split_evenly((1, 2));
        let threshold_mm = threshold * 1e3;
        let all = meshes.iter().flatten().flat_map(|s| s.points.iter());
        let xmax = all.clone().map(|p| p.force_n).fold(0., f64::max);
        let ymin = all.clone().map(|p| p.travel_mm).fold(0., f64::min);
        let ymax = all.map(|p| p.travel_mm).fold(threshold_mm, f64::max);
        // An asinh scale resolves small displacements at onset while retaining large positive or negative travel.
        let transform = |mm: f64| (1e6 * (mm / threshold_mm).asinh()).round() as i32;
        let extent = transform(ymin) - 100_000..transform(ymax) + 150_000;
        let ticks: Vec<_> = [
            -4000., -1000., -100., -20., -4., -1., 0., 1., 4., 20., 100., 1000., 4000.,
        ]
        .into_iter()
        .map(|multiple| transform(threshold_mm * multiple))
        .filter(|x| extent.contains(x))
        .collect();
        let label = |x: &i32| {
            let v = (f64::from(*x) * 1e-6).sinh() * threshold_mm;
            if v.abs() < 1e-9 {
                "0".into()
            } else {
                format!("{v:.2}")
            }
        };
        for (column, (clocks, pane)) in meshes.iter().zip(panes).enumerate() {
            let mut chart = ChartBuilder::on(&pane)
                .caption(
                    format!("{} points across the sheet", [5, 7][column]),
                    ("sans-serif", 22),
                )
                .margin(12)
                .set_label_area_size(LabelAreaPosition::Left, 73)
                .set_label_area_size(LabelAreaPosition::Bottom, 49)
                .build_cartesian_2d(
                    -xmax * 0.025..xmax * 1.035,
                    extent.clone().with_key_points(ticks.clone()),
                )?;
            chart
                .configure_mesh()
                .max_light_lines(0)
                .label_style(("sans-serif", 14))
                .axis_desc_style(("sans-serif", 15))
                .x_desc("Peak sideways load (N)")
                .y_desc("Persistent finger travel (mm, asinh scale)")
                .y_label_formatter(&label)
                .light_line_style(RGBColor(231, 231, 231))
                .draw()?;
            chart.draw_series(LineSeries::new(
                [
                    (0., transform(threshold_mm)),
                    (xmax, transform(threshold_mm)),
                ],
                RGBColor(95, 79, 105).stroke_width(2),
            ))?;
            for (clock, s) in clocks.iter().enumerate() {
                for p in &s.points {
                    let xy = (p.force_n, transform(p.travel_mm));
                    if clock == 0 {
                        chart.draw_series(std::iter::once(Circle::new(
                            xy,
                            6,
                            PRIMARY.stroke_width(2),
                        )))?;
                    } else {
                        chart.draw_series(std::iter::once(Cross::new(
                            xy,
                            4,
                            FINE.stroke_width(2),
                        )))?;
                    }
                    if !p.supported {
                        chart.draw_series(std::iter::once(
                            EmptyElement::at(xy) + Rectangle::new([(7, -10), (12, -5)], BLACK),
                        ))?;
                    }
                }
            }
        }
        let notes = notes.split_evenly((1, 2));
        for (pane, clocks) in notes.iter().zip(&meshes) {
            pane.draw(&Text::new(
                format!("Load interval at {threshold_mm:.2} mm sustained travel"),
                (35, 7),
                ("sans-serif", 17),
            ))?;
            for (row, (s, clock, color)) in [
                (&clocks[0], "Primary", PRIMARY),
                (&clocks[1], "Finer", FINE),
            ]
            .into_iter()
            .enumerate()
            {
                let interval = s.bracket.map_or_else(
                    || "onset unbracketed".into(),
                    |(a, b)| format!("{a:.3} to {b:.3} N"),
                );
                pane.draw(&Text::new(
                    format!("{clock} clock   {interval}"),
                    (35, 38 + row as i32 * 27),
                    ("sans-serif", 17).into_font().color(&color),
                ))?;
            }
        }
        page.present()?;
    }
    Ok(svg)
}

fn render(input: &Path, output: &Path) -> Result<()> {
    let data = serde_json::from_slice(&fs::read(input)?)?;
    let svg = draw(&data)?;
    fs::File::create_new(output)?.write_all(svg.as_bytes())?;
    Ok(())
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [input, output] = &args[..] else {
        return Err("usage: fold_breakaway_response COMPARISON.json NEW_FIGURE.svg".into());
    };
    render(Path::new(input), Path::new(output))
}
#[cfg(test)]
#[path = "../../tests/support/fold_breakaway_response.rs"]
mod tests;
