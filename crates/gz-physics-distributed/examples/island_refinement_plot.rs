// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot committed resolution by body alongside measured paired four-worker timings.
use clap::Parser;
use gz_physics_distributed::physics::Body;
use plotters::prelude::*;
use serde::{Deserialize, de::DeserializeOwned};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    trace: PathBuf,
    timings: PathBuf,
    output: PathBuf,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: Vec<Body>,
    fine_ids: Vec<u64>,
}
#[derive(Deserialize)]
struct Timing {
    round: usize,
    policy: String,
    elapsed_ms: f64,
    quiet_bodies: usize,
    ticks: u64,
}

fn rows<T: DeserializeOwned>(path: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(path)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect()
}

fn main() -> Result<()> {
    let args = Args::parse();
    let frames: Vec<Frame> = rows(&args.trace)?;
    let timings: Vec<Timing> = rows(&args.timings)?;
    let first = frames.first().ok_or("empty physics trace")?;
    let end = frames.last().unwrap().tick;
    let ids: Vec<_> = first.bodies.iter().map(|b| b.id).collect();
    if first.tick != 0
        || end == 0
        || ids.is_empty()
        || ids.len() > 32
        || frames.iter().enumerate().any(|(i, f)| {
            f.tick != i as u64
                || !f.bodies.iter().map(|b| b.id).eq(ids.iter().copied())
                || f.fine_ids.iter().any(|id| !ids.contains(id))
        })
    {
        return Err("invalid physics trace".into());
    }
    let mut pairs = BTreeMap::<usize, [Option<f64>; 2]>::new();
    let total = timings.first().ok_or("empty benchmark")?.quiet_bodies + 3;
    for t in timings {
        let lane = match t.policy.as_str() {
            "island" => 0,
            "scene" => 1,
            _ => return Err("unknown policy".into()),
        };
        if t.ticks != end
            || t.quiet_bodies + 3 != total
            || !t.elapsed_ms.is_finite()
            || t.elapsed_ms <= 0.
            || pairs.entry(t.round).or_default()[lane]
                .replace(t.elapsed_ms)
                .is_some()
        {
            return Err("unmatched timing series".into());
        }
    }
    let paired = pairs
        .values()
        .map(|p| match p {
            [Some(a), Some(b)] => Ok([*a, *b]),
            _ => Err("incomplete pair"),
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let blue = RGBColor(45, 116, 163);
    let orange = RGBColor(218, 108, 31);
    let root = SVGBackend::new(&args.output, (1040, 355)).into_drawing_area();
    root.fill(&WHITE)?;
    let (left, right) = root.split_horizontally(570);
    let (plots, keys) = left.split_vertically(324);
    let (right_plots, right_keys) = right.split_vertically(324);
    for (area, blue_label, orange_label) in [
        (&keys, "One 10 ms solve", "Ten 1 ms solves"),
        (&right_keys, "Island-local", "Scene-wide"),
    ] {
        let middle = area.dim_in_pixel().0 as i32 / 2;
        for (x, color, label) in [
            (middle - 145, blue, blue_label),
            (middle + 12, orange, orange_label),
        ] {
            area.draw(&Circle::new((x, 12), 5, color.filled()))?;
            area.draw(&Text::new(label, (x + 11, 4), ("sans-serif", 14)))?;
        }
    }
    let mut chart = ChartBuilder::on(&plots)
        .margin(15)
        .caption(
            format!("Native step by body  ·  {}-body replay", ids.len()),
            ("sans-serif", 20),
        )
        .x_label_area_size(47)
        .y_label_area_size(105)
        .build_cartesian_2d(0.0..end as f64 * 0.01, 0.0..ids.len() as f64)?;
    chart
        .configure_mesh()
        .disable_mesh()
        .y_labels(0)
        .x_desc("Simulation time (s)")
        .draw()?;
    for (i, b) in first.bodies.iter().enumerate() {
        let y = (ids.len() - i - 1) as f64;
        for f in &frames[1..] {
            let x = f.tick as f64 * 0.01;
            let color = if f.fine_ids.binary_search(&b.id).is_ok() {
                orange
            } else {
                blue
            };
            chart.draw_series(std::iter::once(Rectangle::new(
                [(x - 0.01, y + 0.09), (x, y + 0.91)],
                color.filled(),
            )))?;
        }
        let (_, h) = chart.plotting_area().map_coordinate(&(0., y + 0.5));
        left.draw(&Text::new(
            format!("{:?}  {}", b.primitive, b.id),
            (22, h - 7),
            ("sans-serif", 14),
        ))?;
    }
    let n = paired.len();
    let max = paired.iter().flatten().copied().fold(0_f64, f64::max);
    let mut chart = ChartBuilder::on(&right_plots)
        .margin(15)
        .caption(
            format!("Wall time  ·  {total}-body benchmark"),
            ("sans-serif", 20),
        )
        .x_label_area_size(47)
        .y_label_area_size(55)
        .build_cartesian_2d(0.0..max * 1.15, 0.5..(n as f64 + 0.5))?;
    chart
        .configure_mesh()
        .disable_y_mesh()
        .max_light_lines(0)
        .y_labels(n)
        .y_label_formatter(&|y| format!("{y:.0}"))
        .x_desc(format!(
            "Milliseconds per {:.2} s of simulation",
            end as f64 * 0.01
        ))
        .y_desc("Round")
        .draw()?;
    for (i, [a, b]) in paired.iter().enumerate() {
        let y = (i + 1) as f64;
        chart.draw_series(std::iter::once(PathElement::new(
            [(*a, y), (*b, y)],
            RGBColor(171, 183, 191).stroke_width(2),
        )))?;
        for (x, color) in [(*a, blue), (*b, orange)] {
            chart.draw_series(std::iter::once(Circle::new((x, y), 5, color.filled())))?;
        }
    }
    root.present()?;
    Ok(())
}
