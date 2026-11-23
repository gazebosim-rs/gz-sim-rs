// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use plotters::prelude::*;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
type Rows = BTreeMap<(usize, u8, bool, bool), Vec<f64>>;
const CPU: RGBColor = RGBColor(164, 69, 141);
const GPU: RGBColor = RGBColor(17, 119, 147);
const MIXED: RGBColor = RGBColor(100, 109, 120);
fn median(values: &mut [f64]) -> f64 {
    values.sort_unstable_by(f64::total_cmp);
    (values[(values.len() - 1) / 2] + values[values.len() / 2]) * 0.5
}
pub fn draw(cost: &Path, crossover: &Path, dir: &Path) -> Result<()> {
    let mut costs: BTreeMap<(usize, u8, usize), Vec<f64>> = BTreeMap::new();
    for row in BufReader::new(File::open(cost)?).lines().skip(1) {
        let row = row?;
        let f: Vec<_> = row.split(',').collect();
        if f[3].parse::<usize>()? <= 30 {
            continue;
        }
        costs
            .entry((f[1].parse()?, f[0].parse()?, f[2].parse()?))
            .or_default()
            .push(f[12].parse::<f64>()? * 1e-3);
    }
    let mut rows = Rows::new();
    for row in BufReader::new(File::open(crossover)?).lines().skip(1) {
        let row = row?;
        let f: Vec<_> = row.split(',').collect();
        let key = (f[0].parse()?, f[1].parse()?, f[2] == "Cpu", f[3] == "Cpu");
        rows.entry(key)
            .or_default()
            .push((f[5].parse::<f64>()? + f[6].parse::<f64>()?) * 1e-6);
    }
    let path = dir.join("placement-cost.svg");
    let canvas = SVGBackend::new(&path, (1020, 660)).into_drawing_area();
    canvas.fill(&WHITE)?;
    let (upper, lower) = canvas.split_vertically(320);
    let mut top = ChartBuilder::on(&upper)
        .margin(15)
        .x_label_area_size(35)
        .y_label_area_size(64)
        .caption("Whole Gazebo step by execution owner", ("sans-serif", 20))
        .build_cartesian_2d(0f64..2120., 0f64..700.)?;
    top.configure_mesh()
        .x_desc("Independent SDF environments")
        .y_desc("Median step (µs)")
        .draw()?;
    for (percent, color, label) in [(100, CPU, "CPU"), (0, GPU, "GPU"), (50, MIXED, "50% CPU")] {
        let counts = [3, 128, 2048];
        let mut points = vec![];
        for n in counts {
            let mut runs: Vec<_> = (0..3)
                .map(|rep| median(costs.get_mut(&(n, rep, percent)).unwrap()))
                .collect();
            let m = median(&mut runs);
            points.push((n as f64, m));
            top.draw_series(
                runs.iter()
                    .map(|&x| Circle::new((n as f64, x), 4, color.filled())),
            )?;
        }
        top.draw_series(LineSeries::new(points, color.stroke_width(2)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], color.stroke_width(3)));
    }
    top.configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .border_style(WHITE)
        .background_style(WHITE)
        .draw()?;
    let (a, b) = lower.split_horizontally(510);
    for (panel, count) in [(&a, 128), (&b, 2048)] {
        let mut lines = Vec::new();
        for rep in 0..3 {
            let staying = &rows[&(count, rep, true, true)];
            let switched = &rows[&(count, rep, true, false)];
            let mut total = 0.;
            let line = staying
                .iter()
                .zip(switched)
                .enumerate()
                .map(|(i, (s, m))| {
                    total += m - s;
                    ((i + 1) as f64, total)
                })
                .collect::<Vec<_>>();
            lines.push(line);
        }
        let min = lines
            .iter()
            .flatten()
            .map(|p| p.1)
            .fold(0., f64::min)
            .floor()
            - 1.;
        let max = lines
            .iter()
            .flatten()
            .map(|p| p.1)
            .fold(0., f64::max)
            .ceil()
            + 1.;
        let mut plot = ChartBuilder::on(panel)
            .margin(14)
            .x_label_area_size(38)
            .y_label_area_size(67)
            .caption(
                format!("CPU → GPU at {count} environments"),
                ("sans-serif", 18),
            )
            .build_cartesian_2d(0f64..300., min..max)?;
        plot.configure_mesh()
            .x_desc("Advancing ticks after switch")
            .y_desc("Cumulative extra time (ms)")
            .draw()?;
        plot.draw_series(LineSeries::new(
            [(0., 0.), (300., 0.)],
            BLACK.stroke_width(1),
        ))?;
        for (i, line) in lines.into_iter().enumerate() {
            let color = [
                RGBColor(32, 102, 140),
                RGBColor(35, 154, 150),
                RGBColor(150, 103, 68),
            ][i];
            plot.draw_series(LineSeries::new(line, color.stroke_width(2)))?;
        }
    }
    canvas.present()?;
    Ok(())
}
