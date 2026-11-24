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
type Key = (usize, u8, String, String);
#[derive(Default)]
struct Run {
    ns: u128,
    ticks: u32,
    gpu: Vec<bool>,
}
fn measured(path: &Path) -> Result<BTreeMap<Key, Run>> {
    let mut map = BTreeMap::<Key, Run>::new();
    for row in BufReader::new(File::open(path)?).lines().skip(1) {
        let row = row?;
        let v: Vec<_> = row.split(',').collect();
        let key = (
            v[0].parse()?,
            v[1].parse()?,
            v[2].to_owned(),
            v[3].to_owned(),
        );
        let r = map.entry(key).or_default();
        r.ns += v[10].parse::<u128>()?;
        r.ticks += 1;
        if v[3] == "Automatic" {
            r.gpu.push(v[6] == "Gpu");
        }
    }
    Ok(map)
}
pub fn draw(dir: &Path) -> Result<()> {
    let long = measured(&dir.join("long/steps.csv"))?;
    let first = measured(&dir.join("first/steps.csv"))?;
    let burst = measured(&dir.join("bursts/steps.csv"))?;
    let rows = [
        (&long, 128, "Stationary", "128 stationary"),
        (&long, 3072, "Stationary", "3,072 stationary"),
        (&first, 2048, "Phases", "2,048 staged acquisition"),
        (
            &burst,
            2048,
            "UnannouncedAcquisition",
            "2,048 acquisition bursts",
        ),
    ];
    let path = dir.join("automatic-placement.svg");
    let canvas = SVGBackend::new(&path, (1060, 680)).into_drawing_area();
    canvas.fill(&WHITE)?;
    let (upper, lower) = canvas.split_vertically(360);
    let mut ymax = 0_f64;
    for &(set, n, class, _) in &rows {
        for rep in 0..3 {
            for t in ["FixedCpu", "FixedGpu", "Automatic"] {
                let r = &set[&(n, rep, class.into(), t.into())];
                ymax = ymax.max(r.ns as f64 / (r.ticks as f64 * 1000.));
            }
        }
    }
    let mut chart = ChartBuilder::on(&upper)
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(65)
        .caption(
            "Whole advancing SDF step, three independent treatments",
            ("sans-serif", 20),
        )
        .build_cartesian_2d(-0.5f64..3.5f64, 0f64..(ymax * 1.15))?;
    chart
        .configure_mesh()
        .disable_x_mesh()
        .x_labels(4)
        .x_label_formatter(&|x| {
            let i = x.round();
            if (i - x).abs() < 0.05 && (0.0..4.).contains(&i) {
                rows[i as usize].3.into()
            } else {
                String::new()
            }
        })
        .y_desc("Mean step (µs)")
        .draw()?;
    let colors = [
        RGBColor(160, 59, 129),
        RGBColor(34, 119, 151),
        RGBColor(34, 51, 63),
    ];
    for (index, t) in ["FixedCpu", "FixedGpu", "Automatic"].iter().enumerate() {
        let color = colors[index];
        for (i, &(set, n, class, _)) in rows.iter().enumerate() {
            let mut times: Vec<_> = (0..3)
                .map(|rep| {
                    let r = &set[&(n, rep, class.into(), (*t).into())];
                    r.ns as f64 / (r.ticks as f64 * 1000.)
                })
                .collect();
            times.sort_unstable_by(f64::total_cmp);
            let x = i as f64 + (index as f64 - 1.) * 0.15;
            chart.draw_series([PathElement::new(
                vec![(x, times[0]), (x, times[2])],
                color.stroke_width(2),
            )])?;
            chart.draw_series(
                times
                    .into_iter()
                    .map(|v| Circle::new((x, v), 4, color.filled())),
            )?;
        }
        chart
            .draw_series(std::iter::empty::<Circle<(f64, f64), i32>>())?
            .label(["Fixed CPU", "Fixed GPU", "Measured controller"][index])
            .legend(move |(x, y)| Circle::new((x, y), 4, color.filled()));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE)
        .draw()?;
    let names = [
        "128 · run 1",
        "128 · run 2",
        "128 · run 3",
        "3,072 · run 1",
        "3,072 · run 2",
        "3,072 · run 3",
    ];
    let mut time = ChartBuilder::on(&lower)
        .margin(18)
        .x_label_area_size(45)
        .y_label_area_size(98)
        .caption(
            "Observed execution owner in 2,400 committed stationary steps",
            ("sans-serif", 19),
        )
        .build_cartesian_2d(0u32..2400u32, -0.5f64..5.5f64)?;
    time.configure_mesh()
        .disable_y_mesh()
        .x_desc("Simulation time (ms)")
        .y_labels(6)
        .y_label_formatter(&|y| {
            let index = y.round();
            if (0.0..6.).contains(&index) && (y - index).abs() < 0.1 {
                names[index as usize].into()
            } else {
                String::new()
            }
        })
        .draw()?;
    for (i, &n) in [128, 3072].iter().enumerate() {
        for rep in 0..3 {
            let r = &long[&(n, rep, "Stationary".into(), "Automatic".into())];
            let mut start = 0;
            let mut g = r.gpu[0];
            let y = (3 * i + rep as usize) as f64;
            for j in 1..=r.gpu.len() {
                if j == r.gpu.len() || r.gpu[j] != g {
                    let c = if g { colors[1] } else { colors[0] };
                    time.draw_series([Rectangle::new(
                        [(start as u32, y - 0.32), (j as u32, y + 0.32)],
                        c.filled(),
                    )])?;
                    if j < r.gpu.len() {
                        g = r.gpu[j];
                        start = j;
                    }
                }
            }
        }
    }
    canvas.present()?;
    Ok(())
}
