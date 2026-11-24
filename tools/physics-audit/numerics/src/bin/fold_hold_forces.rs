// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit measured fold-hold loads and plot temporal and spatial resolution effects.
//! Usage: fold_hold_forces RUN_DIRECTORY NEW_FIGURE.svg
#[path = "../fold_hold_mesh_plot.rs"]
mod mesh;
use plotters::prelude::*;
use serde_json::Value;
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
type Settings = (u64, u64);
const COARSE: Settings = (8, 24);
const COARSE_COLOR: RGBColor = RGBColor(163, 75, 27);
const FINE_COLOR: RGBColor = RGBColor(17, 103, 120);
const BIN_TICKS: usize = 10;

struct Loads {
    mean: f64,
    history: Vec<(f64, f64)>,
    width: [f64; 3],
}

fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite() && *x >= 0.)
        .ok_or_else(|| "expected a nonnegative force".into())
}
fn integer(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "expected unsigned integer".into())
}
fn settings(v: &Value) -> Result<Settings> {
    Ok((integer(&v["substeps"])?, integer(&v["iterations"])?))
}
fn equal(a: f64, b: f64) -> Result<()> {
    if (a - b).abs() > 1e-9 * a.abs().max(b.abs()).max(1.) {
        return Err(format!("force accounting differs: {a} vs {b} N").into());
    }
    Ok(())
}
fn read_loads(trial: &Value, matched: u64, dt: u64, ticks: usize) -> Result<Loads> {
    let samples = trial["samples"].as_array().ok_or("missing hold samples")?;
    if samples.len() != ticks || ticks == 0 || dt == 0 {
        return Err("unexpected hold duration".into());
    }
    let mut cross = Vec::with_capacity(ticks);
    let mut total = 0.;
    for (i, s) in samples.iter().enumerate() {
        let elapsed = dt.checked_mul((i + 1) as u64).ok_or("duration overflow")?;
        if matched.checked_add(elapsed) != Some(integer(&s["time_ns"])?) {
            return Err("missing or off-clock hold sample".into());
        }
        let load = number(&s["self_contact_n"])?;
        let opposed = number(&s["cross_panel_n"])?;
        if opposed > load + 1e-9 {
            return Err("opposed-panel force exceeds total sheet self-contact".into());
        }
        total += load;
        cross.push(opposed);
    }
    let mean = cross.iter().sum::<f64>() / ticks as f64;
    equal(mean, number(&trial["mean_cross_panel_n"])?)?;
    equal(total / ticks as f64, number(&trial["mean_self_contact_n"])?)?;
    let nodes = trial["mean_cross_panel_node_normal_n"]
        .as_array()
        .ok_or("missing nodal reactions")?;
    if nodes.len() != 12 {
        return Err("expected two panels with two rows and three columns each".into());
    }
    let mut width = [0.; 3];
    for panel in nodes.chunks_exact(6) {
        let mut side = 0.;
        for (i, n) in panel.iter().enumerate() {
            let force = number(n)?;
            side += force;
            width[i % 3] += 0.5 * force;
        }
        equal(side, mean)?;
    }
    let history = cross
        .chunks(BIN_TICKS)
        .enumerate()
        .map(|(index, bin)| {
            // Mean of the contributing interval end times, in ms after the fork.
            let center = index * BIN_TICKS;
            let ms = (center as f64 + (bin.len() + 1) as f64 / 2.) * dt as f64 * 1e-6;
            (ms, 1000. * bin.iter().sum::<f64>() / bin.len() as f64)
        })
        .collect();
    Ok(Loads {
        mean,
        history,
        width,
    })
}
fn short((steps, iterations): Settings) -> String {
    format!("{steps} × {iterations}")
}
fn limit(values: impl Iterator<Item = f64>) -> f64 {
    (values.fold(0., f64::max) * 1.16).max(0.001)
}

fn draw(
    trials: &BTreeMap<Settings, Loads>,
    reference: Settings,
    duration_ms: f64,
    bin_ms: f64,
    meshes: &Value,
) -> Result<String> {
    let coarse = trials.get(&COARSE).ok_or("missing coarse hold")?;
    let fine = trials.get(&reference).ok_or("missing reference hold")?;
    let selected = [
        (COARSE, coarse, COARSE_COLOR),
        (reference, fine, FINE_COLOR),
    ];
    let mut svg = String::new();
    {
        let root = SVGBackend::with_string(&mut svg, (1430, 890)).into_drawing_area();
        root.fill(&WHITE)?;
        let root = root.titled(
            "Normal force between the two held panels",
            ("sans-serif", 31),
        )?;
        let panels = root.split_evenly((2, 2));
        let mut chart = ChartBuilder::on(&panels[0])
            .margin(14)
            .caption(
                format!("Force history averaged over {bin_ms:.0} ms"),
                ("sans-serif", 21),
            )
            .x_label_area_size(45)
            .y_label_area_size(78)
            .build_cartesian_2d(
                0.0..duration_ms,
                0.0..limit(
                    selected
                        .iter()
                        .flat_map(|(_, t, _)| t.history.iter().map(|p| p.1)),
                ),
            )?;
        chart
            .configure_mesh()
            .axis_desc_style(("sans-serif", 16))
            .label_style(("sans-serif", 14))
            .x_desc("Time after matching the loaded fold (ms)")
            .y_desc("Normal force (mN)")
            .max_light_lines(1)
            .draw()?;
        for (key, t, color) in selected {
            let label = format!("{} substeps, {} iterations", key.0, key.1);
            chart
                .draw_series(LineSeries::new(
                    t.history.iter().copied(),
                    color.stroke_width(3),
                ))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new(vec![(x, y), (x + 22, y)], color.stroke_width(3))
                });
        }
        chart
            .configure_series_labels()
            .label_font(("sans-serif", 15))
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.92))
            .border_style(TRANSPARENT)
            .draw()?;
        let ordered: Vec<_> = trials.iter().collect();
        let mut chart = ChartBuilder::on(&panels[1])
            .margin(14)
            .caption("Mean force over the full hold", ("sans-serif", 21))
            .x_label_area_size(50)
            .y_label_area_size(78)
            .build_cartesian_2d(
                0.0..limit(ordered.iter().map(|(_, t)| t.mean * 1000.)),
                (0..ordered.len() * 100)
                    .with_key_points((0..ordered.len()).map(|i| 100 * i + 50).collect()),
            )?;
        chart
            .configure_mesh()
            .axis_desc_style(("sans-serif", 16))
            .label_style(("sans-serif", 14))
            .x_desc("Mean normal force (mN)")
            .y_desc("Substeps × iterations")
            .y_labels(ordered.len())
            .y_label_formatter(&|y| short(*ordered[*y / 100].0))
            .disable_y_mesh()
            .max_light_lines(1)
            .draw()?;
        for (i, (&key, t)) in ordered.iter().copied().enumerate() {
            let color = if key == COARSE {
                COARSE_COLOR
            } else if key == reference {
                FINE_COLOR
            } else {
                RGBColor(126, 142, 149)
            };
            let bar = Rectangle::new(
                [(0.0, 100 * i + 15), (1000. * t.mean, 100 * i + 85)],
                color.filled(),
            );
            chart.draw_series(std::iter::once(bar))?;
        }
        let mut chart = ChartBuilder::on(&panels[2])
            .margin(14)
            .caption("Where the panels carry the load", ("sans-serif", 21))
            .x_label_area_size(50)
            .y_label_area_size(72)
            .build_cartesian_2d(
                (0..300i32).with_key_points(vec![50, 150, 250]),
                0.0..limit(
                    selected
                        .iter()
                        .flat_map(|(_, t, _)| t.width.iter().map(|x| 1000. * x)),
                ),
            )?;
        chart
            .configure_mesh()
            .axis_desc_style(("sans-serif", 16))
            .label_style(("sans-serif", 14))
            .x_desc("Material column across the width")
            .y_desc("Mean column force (mN)")
            .x_labels(4)
            .disable_x_mesh()
            .x_label_formatter(&|x| ["Left", "Middle", "Right"][*x as usize / 100].to_owned())
            .max_light_lines(1)
            .draw()?;
        for column in 0..3 {
            let x = 100 * column as i32;
            for (j, (_, t, color)) in selected.iter().enumerate() {
                let start = x + 16 + j as i32 * 35;
                chart.draw_series(std::iter::once(Rectangle::new(
                    [(start, 0.), (start + 32, 1000. * t.width[column])],
                    color.filled(),
                )))?;
            }
        }
        mesh::draw(&panels[3], meshes)?;
        root.present()?;
    }
    Ok(svg)
}

// Check the common prefix of the actual transport archives and the independently
// sampled soft particles, including the precise instant where resolution changes.
fn verify_prefix(run: &Path, matched: u64) -> Result<usize> {
    let observation = |branch: &str| -> Result<Vec<Value>> {
        Ok(serde_json::from_slice(&fs::read(
            run.join(branch).join("particle-frames.json"),
        )?)?)
    };
    let coarse = observation("coarse")?;
    let fine = observation("fine")?;
    let stream = |branch: &str| -> Result<_> {
        Ok(BufReader::new(fs::File::open(
            run.join(branch).join("replay/frames.jsonl"),
        )?)
        .lines())
    };
    let mut a = stream("coarse")?;
    let mut b = stream("fine")?;
    for lines in [&mut a, &mut b] {
        let header: Value = serde_json::from_str(&lines.next().ok_or("empty replay")??)?;
        if header["kind"] != "header" {
            return Err("missing replay header".into());
        }
    }
    let mut previous = None;
    for (idx, (left, right)) in coarse.iter().zip(&fine).enumerate() {
        let time = integer(&left["time_ns"])?;
        if time > matched {
            break;
        }
        if left != right || previous.map_or(time != 0, |t| t >= time) {
            return Err(format!("material prefix differs at {time} ns").into());
        }
        previous = Some(time);
        let left = a.next().ok_or("incomplete coarse replay")??;
        let right = b.next().ok_or("incomplete fine replay")??;
        if left != right {
            return Err(format!("replay publications differ at {time} ns").into());
        }
        let row: Value = serde_json::from_str(&left)?;
        let scene: Value = serde_json::from_str(row["scene"].as_str().ok_or("missing scene")?)?;
        if u128::from(integer(&scene["seconds"])?) * 1_000_000_000
            + u128::from(integer(&scene["nanoseconds"])?)
            != u128::from(time)
        {
            return Err("material and scene clocks differ".into());
        }
        if time == matched {
            return Ok(idx + 1);
        }
    }
    Err("no matched replay frame".into())
}

fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let input = args.next().ok_or("supply fold-hold output directory")?;
    let output = args.next().ok_or("supply a fresh SVG path")?;
    run(Path::new(&input), Path::new(&output))
}

fn run(run: &Path, output: &Path) -> Result<()> {
    let data: Value = serde_json::from_slice(&fs::read(run.join("comparison.json"))?)?;
    let matched = integer(&data["matched_time_ns"])?;
    let prefix = verify_prefix(run, matched)?;
    let dt = integer(&data["dt_ns"])?;
    let ticks: usize = integer(&data["hold_ticks"])?.try_into()?;
    let mut trials = BTreeMap::new();
    for trial in data["trials"].as_array().ok_or("no measured holds")? {
        if trials
            .insert(settings(trial)?, read_loads(trial, matched, dt, ticks)?)
            .is_some()
        {
            return Err("duplicate solver setting".into());
        }
    }
    let svg = draw(
        &trials,
        settings(&data["reference"])?,
        ticks as f64 * dt as f64 * 1e-6,
        BIN_TICKS as f64 * dt as f64 * 1e-6,
        &serde_json::from_slice(&fs::read(run.join("mesh/comparison.json"))?)?,
    )?;
    let mut file = BufWriter::new(fs::File::create_new(output)?);
    file.write_all(svg.as_bytes())?;
    file.flush()?;
    println!(
        "{prefix} common replay frames and material observations through {matched} ns; {} force histories reconciled.",
        trials.len()
    );
    Ok(())
}

#[cfg(test)]
#[path = "../../tests/support/fold_hold_forces.rs"]
mod tests;
