// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare identical independent-pair intervals across uninstrumented timing runs.
use clap::Parser;
use plotters::prelude::*;
use serde::Serialize;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};

#[derive(Parser)]
struct Args {
    /// Directory of SCENE-STORAGE-ORDER-BUILD-samples.json benchmark files.
    directory: PathBuf,
    /// Export a side-by-side SVG of steady-pair p95 with separate units for each filesystem.
    #[arg(long)]
    plot: Option<PathBuf>,
}

type Interval = (u64, [u8; 3], [u8; 3], u64);
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Serialize)]
struct Distribution {
    pooled_p50_ns: u64,
    pooled_p95_ns: u64,
    replicate_p95_ns: Vec<u64>,
    median_full_run_ns: u64,
}
#[derive(Serialize)]
struct Comparison<'a> {
    scene: &'a str,
    storage: &'a str,
    intervals_per_run: usize,
    matched_pair_intervals: usize,
    before: Distribution,
    after: Distribution,
}

fn p95(sorted: &[u64]) -> u64 {
    sorted[(sorted.len() * 95).div_ceil(100) - 1]
}

fn median(sorted: &[u64]) -> u64 {
    let lo = sorted[(sorted.len() - 1) / 2];
    lo + (sorted[sorted.len() / 2] - lo) / 2
}

fn compare<'a>(dir: &Path, scene: &'a str, storage: &'a str) -> Result<Comparison<'a>> {
    let mut route = Vec::new();
    let mut measure = |build: &str| -> Result<Distribution> {
        let mut files = Vec::new();
        for entry in fs::read_dir(dir)? {
            let path = entry?.path();
            let name = path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("");
            if let Some(order) = name
                .strip_prefix(&format!("{scene}-{storage}-"))
                .and_then(|rest| rest.strip_suffix(&format!("-{build}-samples.json")))
            {
                files.push((order.parse::<usize>()?, path));
            }
        }
        files.sort();
        if files.is_empty() {
            return Err(format!("no samples for {scene} {storage} {build}").into());
        }
        let mut replicate_p95_ns = Vec::new();
        let mut totals = Vec::new();
        let mut pooled = Vec::new();
        for (_, file) in files {
            let rows: Vec<Interval> = serde_json::from_slice(&fs::read(file)?)?;
            let mut samples = Vec::new();
            let mut path = Vec::new();
            let mut total = 0u64;
            for (i, (tick, before, after, ns)) in rows.into_iter().enumerate() {
                if tick != i as u64 + 1
                    || ns == 0
                    || path.last().is_some_and(|(_, old)| *old != before)
                {
                    return Err("invalid benchmark interval sequence".into());
                }
                path.push((before, after));
                total = total.checked_add(ns).ok_or("duration overflow")?;
                if before == after && after[0] != after[1] && after[1] == after[2] {
                    samples.push(ns);
                }
            }
            if samples.is_empty() {
                return Err("missing stable independently owned pair".into());
            }
            if route.is_empty() {
                route = path;
            } else if route != path {
                return Err("worker route mismatch between trials".into());
            }
            samples.sort_unstable();
            replicate_p95_ns.push(p95(&samples));
            pooled.extend(samples);
            totals.push(total);
        }
        pooled.sort_unstable();
        totals.sort_unstable();
        Ok(Distribution {
            pooled_p50_ns: median(&pooled),
            pooled_p95_ns: p95(&pooled),
            replicate_p95_ns,
            median_full_run_ns: median(&totals),
        })
    };
    let before = measure("before")?;
    let after = measure("after")?;
    if before.replicate_p95_ns.len() != after.replicate_p95_ns.len() {
        return Err("unequal build repetitions".into());
    }
    Ok(Comparison {
        scene,
        storage,
        intervals_per_run: route.len(),
        matched_pair_intervals: route
            .iter()
            .filter(|(old, new)| old == new && new[0] != new[1] && new[1] == new[2])
            .count(),
        before,
        after,
    })
}

fn plot(path: &Path, results: &[Comparison]) -> Result<()> {
    let page = SVGBackend::new(path, (1050, 370)).into_drawing_area();
    page.fill(&WHITE)?;
    let body = page.titled(
        "Steady sphere-pair contact  |  95th percentile per tick",
        ("sans-serif", 23),
    )?;
    let (panels, legend) = body.split_vertically(282);
    for (x, label, color) in [
        (362, "Previous build", RGBColor(171, 89, 32)),
        (550, "Current build", RGBColor(19, 130, 146)),
    ] {
        legend.draw(&Rectangle::new([(x, 12), (x + 19, 24)], color.filled()))?;
        legend.draw(&Text::new(label, (x + 26, 10), ("sans-serif", 15)))?;
    }
    for ((storage, unit, divisor), panel) in [("tmpfs", "µs", 1_000.0), ("ext4", "ms", 1_000_000.0)]
        .into_iter()
        .zip(panels.split_evenly((1, 2)))
    {
        let rows: Vec<_> = results
            .iter()
            .filter(|row| row.storage == storage)
            .collect();
        let upper = rows
            .iter()
            .flat_map(|r| [r.before.pooled_p95_ns, r.after.pooled_p95_ns])
            .max()
            .ok_or("missing storage group")? as f64
            / divisor
            * 1.24;
        let mut chart = ChartBuilder::on(&panel)
            .margin(18)
            .x_label_area_size(44)
            .y_label_area_size(72)
            .caption(storage, ("sans-serif", 19))
            .build_cartesian_2d(0.0..upper, -0.5..1.5)?;
        chart
            .configure_mesh()
            .disable_y_mesh()
            .y_labels(2)
            .y_label_formatter(&|y| {
                if *y == 0. {
                    "Distant".into()
                } else if *y == 1. {
                    "Convoy".into()
                } else {
                    String::new()
                }
            })
            .x_desc(format!("Wall time ({unit})"))
            .label_style(("sans-serif", 13))
            .axis_desc_style(("sans-serif", 14))
            .max_light_lines(0)
            .draw()?;
        for (index, row) in rows.iter().enumerate() {
            let y = 1.0 - index as f64;
            for (delta, value, color) in [
                (0.15, row.before.pooled_p95_ns, RGBColor(171, 89, 32)),
                (-0.15, row.after.pooled_p95_ns, RGBColor(19, 130, 146)),
            ] {
                let v = value as f64 / divisor;
                chart.draw_series(std::iter::once(Rectangle::new(
                    [(0., y + delta - 0.095), (v, y + delta + 0.095)],
                    color.filled(),
                )))?;
                chart.draw_series(std::iter::once(Text::new(
                    format!("{v:.1}"),
                    (v + 0.012 * upper, y + delta + 0.045),
                    ("sans-serif", 14),
                )))?;
            }
        }
    }
    page.present()?;
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut results = Vec::new();
    for scene in ["convoy", "distant"] {
        for storage in ["tmpfs", "ext4"] {
            results.push(compare(&args.directory, scene, storage)?);
        }
    }
    if let Some(path) = args.plot {
        plot(&path, &results)?;
    }
    println!("{}", serde_json::to_string_pretty(&results)?);
    Ok(())
}
