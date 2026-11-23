// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot paired particle-pad and area-patch measurements from the fold-patch study.
use plotters::{coord::Shift, prelude::*};
use serde_json::Value;
use std::{error::Error, fs, io::Write, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const COLUMNS: [usize; 3] = [3, 5, 7];
const MODELS: [&str; 2] = ["particle", "patch"];
const COLORS: [RGBColor; 2] = [RGBColor(112, 107, 102), RGBColor(5, 111, 125)];

#[derive(Clone, Copy)]
enum Outcome {
    Held { values: [f64; 4], loaded: u64 },
    Refused,
}

fn number(value: &Value) -> Result<f64> {
    value.as_f64().ok_or_else(|| "missing measurement".into())
}

fn pairs(report: &Value) -> Result<[[Outcome; 2]; 3]> {
    let trials = report["trials"].as_array().ok_or("missing trials")?;
    let mut pairs = [[None; 2]; 3];
    for t in trials {
        let model = MODELS
            .iter()
            .position(|m| t["model"] == *m)
            .ok_or("unknown pad model")?;
        let mesh = COLUMNS
            .iter()
            .position(|&n| t["columns"] == n)
            .ok_or("unknown mesh width")?;
        let ticks = t["completed_ticks"]
            .as_u64()
            .ok_or("missing hold duration")?;
        let outcome = match (t["status"].as_str(), ticks) {
            (Some("completed"), 300) => {
                let force = number(&t["mean_cross_panel_n"])?;
                let gap = number(&t["mean_closest_panel_skin_gap_m"])?;
                let up = &t["mean_jaw_up_n"];
                let support = number(&up[0])? + number(&up[1])?;
                let cost = number(&t["step_p95_ns"])?;
                if force < 0. || cost <= 0. {
                    return Err("invalid normal load or step duration".into());
                }
                let loaded = t["ticks_with_cross_panel_impulse"]
                    .as_u64()
                    .filter(|&n| n <= ticks)
                    .ok_or("invalid loaded tick count")?;
                if (force > 0.) != (loaded > 0) {
                    return Err("contact force and occupied tick count disagree".into());
                }
                Outcome::Held {
                    values: [force * 1e3, gap * 1e6, support * 1e3, cost * 1e-6],
                    loaded,
                }
            }
            (Some("refused"), 0..300) => Outcome::Refused,
            _ => return Err("inconsistent trial duration or status".into()),
        };
        if pairs[mesh][model].replace(outcome).is_some() {
            return Err("duplicate trial".into());
        }
    }
    if pairs.iter().flatten().any(Option::is_none) {
        return Err("missing paired trial".into());
    }
    Ok(pairs.map(|p| p.map(Option::unwrap)))
}

fn label(y: f64) -> String {
    if y != 0. && y.abs() < 0.01 {
        format!("{y:.0e}")
    } else {
        format!("{y:.2}")
    }
}

fn panel(
    area: &DrawingArea<SVGBackend, Shift>,
    pairs: &[[Outcome; 2]; 3],
    metric: usize,
    title: &str,
    units: &str,
) -> Result<()> {
    let values = pairs.iter().flatten().filter_map(|o| match o {
        Outcome::Held { values, .. } => Some(values[metric]),
        Outcome::Refused => None,
    });
    let (lo, hi) = values.fold((0f64, 0f64), |(lo, hi), v| (lo.min(v), hi.max(v)));
    let span = (hi - lo).max(0.1);
    let mut chart = ChartBuilder::on(area)
        .margin(12)
        .caption(title, ("sans-serif", 20))
        .x_label_area_size(34)
        .y_label_area_size(70)
        .build_cartesian_2d(-1.0..5.0, lo - 0.04 * span..hi + 0.19 * span)?;
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(0)
        .x_labels(7)
        .x_label_formatter(&|x: &f64| match *x as i32 {
            0 if *x == 0. => "3 points".into(),
            2 if *x == 2. => "5 points".into(),
            4 if *x == 4. => "7 points".into(),
            _ => String::new(),
        })
        .y_desc(units)
        .axis_desc_style(("sans-serif", 15))
        .label_style(("sans-serif", 14))
        .draw()?;
    for (i, p) in pairs.iter().enumerate() {
        for (m, result) in p.iter().enumerate() {
            let x = 2. * i as f64 + (m as f64 - 0.5) * 0.72;
            let (height, value) = match result {
                Outcome::Held { values, .. } => {
                    let y = values[metric];
                    chart.draw_series(std::iter::once(Rectangle::new(
                        [(x - 0.30, 0.), (x + 0.30, y)],
                        COLORS[m].filled(),
                    )))?;
                    (y.max(0.), label(y))
                }
                Outcome::Refused => (0., "stopped".into()),
            };
            chart.draw_series(std::iter::once(
                EmptyElement::at((x, height + 0.025 * span))
                    + Text::new(
                        value,
                        (0, 0),
                        ("sans-serif", 13).into_font().color(&COLORS[m]).pos(
                            plotters::style::text_anchor::Pos::new(
                                plotters::style::text_anchor::HPos::Center,
                                plotters::style::text_anchor::VPos::Bottom,
                            ),
                        ),
                    ),
            ))?;
        }
    }
    Ok(())
}

/// Draw both material-attached footprints to the same physical scale.
fn footprints(area: &DrawingArea<SVGBackend, Shift>, report: &Value) -> Result<()> {
    let width = number(&report["patch_width_m"])?;
    let height = number(&report["patch_height_m"])?;
    if width <= 0. || height <= 0. {
        return Err("invalid footprint dimensions".into());
    }
    let scale = (92. / height).min(190. / width);
    let w = (scale * width).round() as i32;
    let h = (scale * height).round() as i32;
    let origins = [650, 891];
    let bottom = 109;
    let sites = report["sites"].as_array().ok_or("missing contact sites")?;
    let mut counts = [0; 2];
    for (panel, &cx) in origins.iter().enumerate() {
        area.draw(&Rectangle::new(
            [(cx - w / 2, bottom - h), (cx + w / 2, bottom)],
            COLORS[1].mix(0.12).filled(),
        ))?;
        area.draw(&Rectangle::new(
            [(cx - w / 2, bottom - h), (cx + w / 2, bottom)],
            COLORS[1],
        ))?;
        area.draw(&Text::new(
            ["Left cloth panel", "Right cloth panel"][panel],
            (cx - 54, 0),
            ("sans-serif", 15),
        ))?;
    }
    for site in sites {
        let idx = site["panel"].as_u64().ok_or("missing site panel")? as usize;
        let &cx = origins.get(idx).ok_or("unknown site panel")?;
        let x = number(&site["width_m"])?;
        let z = number(&site["height_m"])?;
        if x.abs() > width / 2. || !(0. ..=height).contains(&z) {
            return Err("site outside the pad footprint".into());
        }
        area.draw(&Circle::new(
            (
                cx + (scale * x).round() as i32,
                bottom - (scale * z).round() as i32,
            ),
            5,
            COLORS[1].filled(),
        ))?;
        counts[idx] += 1;
    }
    if counts.contains(&0) {
        return Err("empty pad footprint".into());
    }
    for (x, y, text, size) in [
        (49, 17, "Contact locations on the cloth".into(), 21),
        (
            49,
            55,
            format!(
                "Each pad covers {:.0} × {:.0} mm of one panel.",
                width * 1e3,
                height * 1e3
            ),
            16,
        ),
        (
            49,
            84,
            "Dots show the pressure samples used on every mesh.".into(),
            16,
        ),
    ] {
        area.draw(&Text::new(text, (x, y), ("sans-serif", size)))?;
    }
    Ok(())
}

fn draw(report: &Value) -> Result<String> {
    let pairs = pairs(report)?;
    let mut svg = String::new();
    {
        let page = SVGBackend::with_string(&mut svg, (1100, 848)).into_drawing_area();
        page.fill(&WHITE)?;
        let page = page.titled("Jaw contact on a folded sheet", ("sans-serif", 28))?;
        let (diagram, data) = page.split_vertically(126);
        footprints(&diagram, report)?;
        let (legend, plots) = data.split_vertically(40);
        let (plots, note) = plots.split_vertically(606);
        for (x, color, title) in [
            (280, COLORS[0], "Contact at cloth particles"),
            (580, COLORS[1], "Contact across a cloth patch"),
        ] {
            legend.draw(&Rectangle::new([(x, 13), (x + 18, 26)], color.filled()))?;
            legend.draw(&Text::new(title, (x + 26, 11), ("sans-serif", 16)))?;
        }
        let areas = plots.split_evenly((2, 2));
        for (metric, (title, units)) in [
            ("Force between the two layers", "Mean force (mN)"),
            ("Closest gap between the layers", "Mean gap (µm)"),
            ("Weight carried by both jaws", "Mean upward force (mN)"),
            ("Time to advance the sheet by 1 ms", "p95 step cost (ms)"),
        ]
        .iter()
        .enumerate()
        {
            panel(&areas[metric], &pairs, metric, title, units)?;
        }
        if let [
            Outcome::Held { loaded: a, .. },
            Outcome::Held { loaded: b, .. },
        ] = pairs[0]
        {
            let line = format!(
                "On the 3-point mesh, the layers carried force on {a} of 300 steps with particle contact and {b} of 300 steps with the patch."
            );
            note.draw(&Text::new(line, (49, 14), ("sans-serif", 17)))?;
        }
        page.present()?;
    }
    Ok(svg)
}

fn render(input: &Path, output: &Path) -> Result<()> {
    let report: Value = serde_json::from_slice(&fs::read(input)?)?;
    let svg = draw(&report)?;
    fs::File::create_new(output)?.write_all(svg.as_bytes())?;
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [input, output] = &args[..] else {
        return Err("usage: fold_patch_forces COMPARISON.json NEW_FIGURE.svg".into());
    };
    render(Path::new(input), Path::new(output))
}

#[cfg(test)]
#[path = "../../tests/support/fold_patch_forces.rs"]
mod tests;
