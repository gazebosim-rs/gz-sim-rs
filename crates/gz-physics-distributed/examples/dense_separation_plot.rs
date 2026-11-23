// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot the committed dense-scene traces and the measured release timings.
use clap::Parser;
use plotters::prelude::*;
use serde::{Deserialize, de::DeserializeOwned};
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

#[derive(Parser)]
struct Args {
    /// Evidence directory containing prior/tight trajectories and benchmark.jsonl.
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Frame {
    time_s: f64,
    components: Vec<Vec<u64>>,
}
#[derive(Deserialize)]
struct Trial {
    round: i32,
    bound: String,
    elapsed_ms: f64,
}
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn read<T: DeserializeOwned>(path: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(path)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect()
}
fn main() -> Result<()> {
    let dir = Args::parse().directory;
    let prior = read::<Frame>(&dir.join("prior/distributed.jsonl"))?;
    let tight = read::<Frame>(&dir.join("tight/distributed.jsonl"))?;
    let trials = read::<Trial>(&dir.join("benchmark.jsonl"))?;
    let end = tight.last().ok_or("empty trace")?.time_s;
    let last = trials
        .iter()
        .map(|t| t.round)
        .max()
        .ok_or("empty timings")?;
    let ymax = trials.iter().map(|t| t.elapsed_ms).fold(0., f64::max);
    let path = dir.join("dense-separation.svg");
    let page = SVGBackend::new(&path, (1100, 520)).into_drawing_area();
    page.fill(&WHITE)?;
    let title = page.titled(
        "Dense encounter   |   26 bodies on four workers",
        ("sans-serif", 29),
    )?;
    let (left, right) = title.split_horizontally(620);
    let mut counts = ChartBuilder::on(&left)
        .margin(22)
        .x_label_area_size(48)
        .y_label_area_size(55)
        .caption(
            "Bodies sharing the loaded pair’s solver",
            ("sans-serif", 21),
        )
        .build_cartesian_2d(0.0..end, 0u32..30)?;
    counts
        .configure_mesh()
        .x_desc("Simulation time (s)")
        .y_desc("Bodies")
        .max_light_lines(1)
        .bold_line_style(RGBColor(227, 231, 235))
        .axis_desc_style(("sans-serif", 15))
        .label_style(("sans-serif", 13))
        .draw()?;
    let prior_color = RGBColor(171, 81, 19);
    let tight_color = RGBColor(27, 104, 169);
    for (frames, color, name) in [
        (&prior, prior_color, "Prior bound"),
        (&tight, tight_color, "Tighter bound"),
    ] {
        let mut points = Vec::new();
        let mut last_count = None;
        for f in frames {
            let n = f
                .components
                .iter()
                .find(|group| group.contains(&1))
                .ok_or("missing loaded body")?
                .len() as u32;
            if let Some(previous) = last_count {
                points.push((f.time_s, previous));
            }
            points.push((f.time_s, n));
            last_count = Some(n);
        }
        counts
            .draw_series(LineSeries::new(points, color.stroke_width(3)))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 22, y)], color.stroke_width(3)));
    }
    counts
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperMiddle)
        .background_style(WHITE.mix(0.94))
        .border_style(WHITE)
        .draw()?;
    let mut timing = ChartBuilder::on(&right)
        .margin(22)
        .x_label_area_size(48)
        .y_label_area_size(55)
        .caption("Wall time for 230 ticks", ("sans-serif", 21))
        .build_cartesian_2d(0..last + 1, 0.0..(ymax / 10.).ceil() * 10. + 5.)?;
    timing
        .configure_mesh()
        .x_desc("Paired release trial")
        .y_desc("Wall time (ms)")
        .disable_x_mesh()
        .max_light_lines(1)
        .bold_line_style(RGBColor(227, 231, 235))
        .axis_desc_style(("sans-serif", 15))
        .label_style(("sans-serif", 13))
        .draw()?;
    for round in 1..=last {
        let mut times = [0.; 2];
        for (i, label) in ["prior", "tight"].into_iter().enumerate() {
            times[i] = trials
                .iter()
                .find(|t| t.round == round && t.bound == label)
                .ok_or("incomplete trial")?
                .elapsed_ms;
        }
        timing.draw_series([PathElement::new(
            [(round, times[0]), (round, times[1])],
            RGBColor(179, 184, 189).stroke_width(2),
        )])?;
        timing.draw_series([Circle::new((round, times[0]), 5, prior_color.filled())])?;
        timing.draw_series([Circle::new((round, times[1]), 5, tight_color.filled())])?;
    }
    page.present()?;
    Ok(())
}
