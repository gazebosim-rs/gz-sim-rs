// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot recorded sheet motion in the jaw frame and force between the folded layers.
use plotters::prelude::*;
use serde_json::Value;
use std::{error::Error, fs, io::Write, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const PULL: RGBColor = RGBColor(0, 105, 113);
const SHAM: RGBColor = RGBColor(164, 77, 32);

#[derive(Clone, Copy)]
struct Sample {
    ns: u64,
    sideways: f64,
    slip_mm: f64,
    layer_mn: f64,
}

fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .ok_or_else(|| "missing numeric measurement".into())
}

fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn samples(trace: &Value, baseline_ns: u64) -> Result<Vec<Sample>> {
    let rows = trace.as_array().ok_or("missing trace")?;
    let mut points = vec![];
    for row in rows {
        let ns = row["time_ns"].as_u64().ok_or("missing sample clock")?;
        if ns < baseline_ns {
            continue;
        }
        points.push(Sample {
            ns: ns - baseline_ns,
            sideways: number(&row["force_y_n"])?,
            slip_mm: 1e3 * number(&row["jaw_frame_y_m"])?,
            layer_mn: 1e3 * number(&row["cross_panel_n"])?,
        });
    }
    let first = points.first().ok_or("empty post-grip trace")?;
    if first.ns != 0 || first.sideways != 0. {
        return Err("missing unloaded baseline".into());
    }
    let baseline = first.slip_mm;
    for p in &mut points {
        p.slip_mm -= baseline;
    }
    Ok(points)
}

struct Pair {
    columns: u64,
    pull: Vec<Sample>,
    sham: Vec<Sample>,
    last_loaded_ms: f64,
}

impl Pair {
    fn new(columns: u64, pull: Vec<Sample>, sham: Vec<Sample>, dt: u64) -> Result<Self> {
        if pull.len() != sham.len()
            || pull.iter().zip(&sham).enumerate().any(|(i, (a, b))| {
                Some(a.ns) != (i as u64).checked_mul(dt) || a.ns != b.ns || b.sideways != 0.
            })
        {
            return Err("control and pull need matching clocks and an unloaded control".into());
        }
        let last_load = pull
            .iter()
            .rposition(|p| p.sideways != 0.)
            .ok_or("empty pull")?;
        if last_load + 1 == pull.len() {
            return Err("missing post-pull recovery".into());
        }
        let last_loaded_ms = pull[last_load].ns as f64 / 1e6;
        Ok(Self {
            columns,
            pull,
            sham,
            last_loaded_ms,
        })
    }
}

fn load(root: &Path) -> Result<(Vec<Pair>, f64)> {
    let report = read(&root.join("comparison.json"))?;
    let dt = report["dt_ns"]
        .as_u64()
        .filter(|&v| v != 0)
        .ok_or("missing time step")?;
    let target = 1e3 * number(&report["target_n"])?;
    let trials = report["trials"].as_array().ok_or("missing trials")?;
    let mut out = vec![];
    for columns in [5, 7] {
        let trial = trials
            .iter()
            .find(|t| t["columns"] == columns)
            .ok_or("missing mesh")?;
        let baseline = trial["baseline_time_ns"]
            .as_u64()
            .ok_or("missing grip baseline")?;
        let branch = |name| {
            let f = root.join(format!("mesh-{columns}-{name}/trace.json"));
            samples(&read(&f)?, baseline)
        };
        out.push(Pair::new(columns, branch("pull")?, branch("sham")?, dt)?);
    }
    Ok((out, target))
}

fn draw(pairs: &[Pair], target_mn: f64) -> Result<String> {
    let mut svg = String::new();
    {
        let page = SVGBackend::with_string(&mut svg, (1120, 735)).into_drawing_area();
        page.fill(&WHITE)?;
        let (top, grid) = page.split_vertically(109);
        top.draw(&Text::new(
            "How far does the folded sheet slide?",
            (36, 19),
            ("sans-serif", 30),
        ))?;
        top.draw(&Text::new("Sheet motion is measured from the jaws at the last tick before the pull. Shading marks the applied load.", (37, 54), ("sans-serif", 16)))?;
        for (x, color, label) in [(38, PULL, "Sideways pull"), (218, SHAM, "No-pull control")] {
            top.draw(&PathElement::new(
                [(x, 92), (x + 28, 92)],
                color.stroke_width(3),
            ))?;
            top.draw(&Text::new(label, (x + 38, 82), ("sans-serif", 16)))?;
        }
        let all = pairs.iter().flat_map(|p| p.pull.iter().chain(&p.sham));
        let ymax = all.clone().map(|e| e.slip_mm).fold(0., f64::max);
        let ymin = all.clone().map(|e| e.slip_mm).fold(0., f64::min);
        let (scale, unit) = if (ymax - ymin) < 0.001 {
            (1e6, "Sheet displacement (nm)")
        } else if (ymax - ymin) < 0.1 {
            (1e3, "Sheet displacement (µm)")
        } else {
            (1., "Sheet displacement (mm)")
        };
        let margin = ((ymax - ymin) * 0.08).max(1e-6);
        let pull_n = all.clone().map(|s| s.sideways.abs()).fold(0., f64::max);
        let tmax = all.map(|e| e.ns).max().unwrap_or(0) as f64 / 1e6;
        // Each force point is an average of successive ten public samples.
        let force = |s: &[Sample]| s.iter().map(|p| p.layer_mn).sum::<f64>() / s.len() as f64;
        let force_means = pairs
            .iter()
            .flat_map(|p| {
                p.pull[1..]
                    .chunks_exact(10)
                    .chain(p.sham[1..].chunks_exact(10))
            })
            .map(force);
        let fmin = force_means.clone().fold(target_mn, f64::min);
        let fmax = force_means.fold(target_mn, f64::max);
        let fpad = ((fmax - fmin) * 0.1).max(0.001);
        let panels = grid.split_evenly((2, 2));
        for (col, p) in pairs.iter().enumerate() {
            for (row, label, bounds) in [
                (0, unit, (ymin - margin) * scale..(ymax + margin) * scale),
                (
                    1,
                    "Mean layer force (mN)",
                    (fmin - fpad).max(0.)..fmax + fpad,
                ),
            ] {
                let mut chart = ChartBuilder::on(&panels[2 * row + col])
                    .caption(
                        if row == 0 {
                            format!("{} points across the sheet", p.columns)
                        } else {
                            "Layer force, 10-sample means".into()
                        },
                        ("sans-serif", 20),
                    )
                    .margin_left(9)
                    .margin_right(20)
                    .margin_top(10)
                    .y_label_area_size(77)
                    .x_label_area_size(56)
                    .build_cartesian_2d(0.0..tmax, bounds.clone())?;
                chart
                    .configure_mesh()
                    .x_desc("Time after grip (ms)")
                    .y_desc(label)
                    .x_labels(6)
                    .y_labels(5)
                    .max_light_lines(0)
                    .axis_desc_style(("sans-serif", 15))
                    .label_style(("sans-serif", 13))
                    .light_line_style(RGBColor(219, 224, 224))
                    .draw()?;
                chart.draw_series(std::iter::once(Rectangle::new(
                    [(0., bounds.start), (p.last_loaded_ms, bounds.end)],
                    RGBColor(0, 105, 113).mix(0.07).filled(),
                )))?;
                if row == 1 {
                    chart.draw_series(std::iter::once(PathElement::new(
                        [(0., target_mn), (tmax, target_mn)],
                        BLACK.mix(0.40).stroke_width(1),
                    )))?;
                }
                for (points, color) in [(&p.sham, SHAM), (&p.pull, PULL)] {
                    let coords: Vec<_> = if row == 0 {
                        points
                            .iter()
                            .map(|s| (s.ns as f64 / 1e6, s.slip_mm * scale))
                            .collect()
                    } else {
                        points[1..]
                            .chunks_exact(10)
                            .map(|s| ((s[0].ns as f64 + s[s.len() - 1].ns as f64) / 2e6, force(s)))
                            .collect()
                    };
                    chart.draw_series(LineSeries::new(coords, color.stroke_width(2)))?;
                }
            }
        }
        top.draw(&Text::new(
            format!("Grip target  {target_mn:.2} mN       Applied pull  {pull_n:.2} N"),
            (444, 82),
            ("sans-serif", 16),
        ))?;
        page.present()?;
    }
    Ok(svg)
}

fn render(root: &Path, output: &Path) -> Result<()> {
    let (pairs, target) = load(root)?;
    let svg = draw(&pairs, target)?;
    fs::File::create_new(output)?.write_all(svg.as_bytes())?;
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [root, output] = &args[..] else {
        return Err("usage: fold_pull_response STUDY_DIRECTORY NEW_FIGURE.svg".into());
    };
    render(Path::new(root), Path::new(output))
}

#[cfg(test)]
#[path = "../../tests/support/fold_pull_response.rs"]
mod tests;
