// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Summarize paired release trials and export the p95 gauges used by the gz-web-rs replay.
use clap::Parser;
use plotters::prelude::*;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    out: PathBuf,
    #[arg(required = true)]
    trials: Vec<PathBuf>,
}
#[derive(Deserialize)]
struct Timed {
    elapsed_ms: f64,
    tick_p95_us: f64,
}
#[derive(Deserialize)]
struct Row {
    round: usize,
    workload: String,
    baseline: Timed,
    current: Timed,
}
#[derive(Serialize)]
struct Summary {
    pairs: usize,
    faster: usize,
    baseline_ms: f64,
    current_ms: f64,
    baseline_p95_us: f64,
    current_p95_us: f64,
    median_paired_reduction_pct: f64,
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_unstable_by(f64::total_cmp);
    let mid = values.len() / 2;
    (values[mid] + values[(values.len() - 1) / 2]) * 0.5
}
fn reduction(r: &Row) -> f64 {
    100. * (1. - r.current.elapsed_ms / r.baseline.elapsed_ms)
}
fn summarize(rows: &[Row]) -> Summary {
    Summary {
        pairs: rows.len(),
        faster: rows
            .iter()
            .filter(|r| r.current.elapsed_ms < r.baseline.elapsed_ms)
            .count(),
        baseline_ms: median(rows.iter().map(|r| r.baseline.elapsed_ms).collect()),
        current_ms: median(rows.iter().map(|r| r.current.elapsed_ms).collect()),
        baseline_p95_us: median(rows.iter().map(|r| r.baseline.tick_p95_us).collect()),
        current_p95_us: median(rows.iter().map(|r| r.current.tick_p95_us).collect()),
        median_paired_reduction_pct: median(rows.iter().map(reduction).collect()),
    }
}
fn read(path: &Path) -> Result<BTreeMap<String, Vec<Row>>> {
    let mut map = BTreeMap::<_, Vec<_>>::new();
    let mut seen = BTreeSet::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let r: Row = serde_json::from_str(&line?)?;
        if !seen.insert((r.workload.clone(), r.round))
            || r.round == 0
            || [
                r.baseline.elapsed_ms,
                r.current.elapsed_ms,
                r.baseline.tick_p95_us,
                r.current.tick_p95_us,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.)
        {
            return Err(format!("invalid or duplicate trial in {}", path.display()).into());
        }
        map.entry(r.workload.clone()).or_default().push(r);
    }
    Ok(map)
}

fn plot(path: &Path, rows: &BTreeMap<String, Vec<Row>>) -> Result<()> {
    let names = [
        ("first", "Heavy first"),
        ("middle", "Heavy middle"),
        ("last", "Heavy last"),
        ("twins", "Two heavy"),
        ("uniform", "All light"),
    ];
    let mut groups = Vec::new();
    for (key, _) in names {
        let values: Vec<_> = rows
            .get(key)
            .ok_or("missing profiled layout")?
            .iter()
            .map(reduction)
            .collect();
        groups.push(values);
    }
    let low = (groups.iter().flatten().copied().fold(0., f64::min) / 10.).floor() * 10. - 3.;
    let high = (groups.iter().flatten().copied().fold(0., f64::max) / 10.).ceil() * 10. + 3.;
    let root = SVGBackend::new(path, (1000, 460)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .margin(22)
        .caption("Wall-time reduction over 200 ticks", ("sans-serif", 26))
        .x_label_area_size(52)
        .y_label_area_size(75)
        .build_cartesian_2d(0.5_f64..5.5, low..high)?;
    chart
        .configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .bold_line_style(RGBColor(225, 230, 235))
        .x_labels(5)
        .x_label_formatter(&|x| {
            names
                .iter()
                .enumerate()
                .find(|(i, _)| (*x - (*i + 1) as f64).abs() < 1e-6)
                .map_or("", |(_, v)| v.1)
                .to_owned()
        })
        .y_desc("Reduction per paired trial (%)")
        .label_style(("sans-serif", 15))
        .axis_desc_style(("sans-serif", 16))
        .draw()?;
    chart.draw_series([PathElement::new(
        [(0.5, 0.), (5.5, 0.)],
        RGBColor(98, 104, 110),
    )])?;
    for (i, group) in groups.iter().enumerate() {
        let x = (i + 1) as f64;
        chart.draw_series(group.iter().enumerate().map(|(j, &y)| {
            let dx = 0.32 * ((j as f64 + 0.5) / group.len() as f64 - 0.5);
            Circle::new((x + dx, y), 4, RGBColor(44, 134, 158).mix(0.74).filled())
        }))?;
        let center = median(group.clone());
        chart.draw_series([PathElement::new(
            [(x - 0.22, center), (x + 0.22, center)],
            BLACK.stroke_width(3),
        )])?;
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let a = Args::parse();
    let mut combined = BTreeMap::<String, Vec<Row>>::new();
    let mut by_file = BTreeMap::new();
    for path in &a.trials {
        let set = read(path)?;
        if !combined.is_empty() && combined.keys().ne(set.keys()) {
            return Err("workloads differ between files".into());
        }
        let summary: BTreeMap<_, _> = set.iter().map(|(k, r)| (k.clone(), summarize(r))).collect();
        by_file.insert(path.display().to_string(), summary);
        for (k, mut r) in set {
            combined.entry(k).or_default().append(&mut r);
        }
    }
    let total: BTreeMap<_, _> = combined
        .iter()
        .map(|(k, r)| (k.clone(), summarize(r)))
        .collect();
    let first = total.get("first").ok_or("missing heavy-first workload")?;
    fs::write(
        a.out.join("viewer-timings.json"),
        serde_json::to_vec_pretty(
            &json!({"layout":"first","rounds":first.pairs,"baseline_p95_us":first.baseline_p95_us,"weighted_p95_us":first.current_p95_us}),
        )?,
    )?;
    fs::write(
        a.out.join("summary.json"),
        serde_json::to_vec_pretty(&json!({"combined":total,"by_file":by_file}))?,
    )?;
    plot(&a.out.join("paired-timings.svg"), &combined)
}
