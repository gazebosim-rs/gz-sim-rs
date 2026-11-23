// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use plotters::prelude::*;
use std::{collections::BTreeMap, fs, path::Path};

pub fn draw(dir: &Path) -> Result<()> {
    let mut positions = BTreeMap::<u32, BTreeMap<u64, f64>>::new();
    for row in fs::read_to_string(dir.join("trajectory.csv"))?
        .lines()
        .skip(1)
    {
        let v: Vec<_> = row.split(',').collect();
        positions
            .entry(v[0].parse()?)
            .or_default()
            .insert(v[1].parse()?, v[3].parse()?);
    }
    let mut latencies = BTreeMap::<(usize, usize), Vec<f64>>::new();
    for row in fs::read_to_string(dir.join("batch/timing.csv"))?
        .lines()
        .skip(1)
    {
        let v: Vec<_> = row.split(',').collect();
        if v[2].parse::<usize>()? > 30 {
            latencies
                .entry((v[0].parse()?, v[1].parse()?))
                .or_default()
                .push(v[4].parse::<f64>()? * 1e-6);
        }
    }
    for samples in latencies.values_mut() {
        samples.sort_by(f64::total_cmp);
    }
    let path = dir.join("native-handoff.svg");
    let area = SVGBackend::new(&path, (1020, 750)).into_drawing_area();
    area.fill(&WHITE)?;
    let (upper, lower) = area.split_vertically(352);
    let ink = RGBColor(48, 62, 70);
    let gold = RGBColor(185, 122, 12);
    let purple = RGBColor(158, 55, 134);
    let mut chart = ChartBuilder::on(&upper)
        .caption(
            "Sphere A deflection relative to the continuously native pair",
            ("sans-serif", 22),
        )
        .margin(20)
        .x_label_area_size(43)
        .y_label_area_size(76)
        .build_cartesian_2d(0u32..700, -2f64..24.)?;
    chart
        .configure_mesh()
        .x_desc("Simulation time (ms)")
        .y_desc("Lateral position difference (µm)")
        .draw()?;
    for (id, label, color) in [
        (9, "Pair 9  ·  native impact", purple),
        (31, "Pair 31  ·  GPU impact", gold),
    ] {
        let series = positions.iter().map(|(&t, p)| (t, 1e6 * (p[&id] - p[&80])));
        chart
            .draw_series(LineSeries::new(series, color.stroke_width(3)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 28, y)], color.stroke_width(3)));
    }
    chart.draw_series([PathElement::new(
        [(309, -1.5), (309, 22.)],
        ink.stroke_width(1),
    )])?;
    chart.draw_series([Text::new(
        "Impact, 309 ms",
        (317, 21.),
        ("sans-serif", 14).into_font().color(&ink),
    )])?;
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .draw()?;

    let ymax = latencies.values().map(|v| v[178]).fold(0., f64::max) * 1.11;
    let mut chart = ChartBuilder::on(&lower)
        .caption(
            "Whole Bevy step for 2,048 environments on the RTX 5090",
            ("sans-serif", 22),
        )
        .margin(20)
        .x_label_area_size(50)
        .y_label_area_size(76)
        .build_cartesian_2d(0f64..104., 0f64..ymax)?;
    chart
        .configure_mesh()
        .x_desc("Pairs solved in native Rapier (remaining pairs on GPU)")
        .y_desc("Wall latency (ms)")
        .draw()?;
    let counts = [0, 16, 48, 96];
    for (idx, label, color) in [(89, "p50", RGBColor(28, 109, 131)), (178, "p99", purple)] {
        for r in 0..3 {
            let serie: Vec<_> = counts
                .iter()
                .map(|&n| (n as f64, latencies[&(r, n)][idx]))
                .collect();
            chart.draw_series(LineSeries::new(
                serie.clone(),
                color.mix(0.42).stroke_width(1),
            ))?;
            let annotation =
                chart.draw_series(serie.into_iter().map(|v| Circle::new(v, 4, color.filled())))?;
            if r == 0 {
                annotation
                    .label(label)
                    .legend(move |(x, y)| Circle::new((x + 9, y), 4, color.filled()));
            }
        }
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    area.present()?;
    Ok(())
}

pub fn draw_support(dir: &Path) -> Result<()> {
    let raw = fs::read_to_string(dir.join("trajectory.csv"))?;
    let mut rows = vec![];
    for line in raw.lines().skip(1) {
        let v: Vec<_> = line.split(',').collect();
        if v[1] != "9" {
            continue;
        }
        rows.push((
            v[0].parse::<u32>()?,
            v[2] == "NativeRapier",
            v[6].parse::<f64>()?,
            [
                v[8].parse::<f64>()?,
                v[9].parse()?,
                v[10].parse()?,
                v[11].parse()?,
            ],
        ));
    }
    let path = dir.join("support-handoff.svg");
    let area = SVGBackend::new(&path, (1000, 690)).into_drawing_area();
    area.fill(&WHITE)?;
    let (top, bottom) = area.split_vertically(322);
    let purple = RGBColor(138, 57, 127);
    let ocher = RGBColor(167, 109, 0);
    let ink = RGBColor(53, 66, 71);
    let mut plot = ChartBuilder::on(&top)
        .margin(19)
        .x_label_area_size(38)
        .y_label_area_size(75)
        .caption(
            "Inclined support, external liftoff and guarded release (environment 9)",
            ("sans-serif", 22),
        )
        .build_cartesian_2d(0u32..1100, -10f64..340.)?;
    plot.configure_mesh()
        .x_desc("Simulation time (ms)")
        .y_desc("Surface clearance (mm)")
        .draw()?;
    for (native, color, label) in [
        (true, purple, "Native Rapier"),
        (false, ocher, "Specialized GPU"),
    ] {
        plot.draw_series(
            rows.iter()
                .filter(|r| r.1 == native)
                .map(|r| Circle::new((r.0, 1000. * r.2), 2, color.filled())),
        )?
        .label(label)
        .legend(move |(x, y)| Circle::new((x + 10, y), 4, color.filled()));
    }
    plot.draw_series([PathElement::new(
        [(400, 0.), (400, 290.)],
        ink.stroke_width(1),
    )])?;
    plot.draw_series([Text::new(
        "External impulse, 400 ms",
        (410, 312.),
        ("sans-serif", 14).into_font().color(&ink),
    )])?;
    plot.configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.95))
        .draw()?;
    let mut chart = ChartBuilder::on(&bottom)
        .margin(19)
        .x_label_area_size(40)
        .y_label_area_size(75)
        .caption(
            "Contact impulse applied in each tick and native warmstart at its boundary",
            ("sans-serif", 20),
        )
        .build_cartesian_2d(0u32..1100, (0.006f64..5000.).log_scale())?;
    chart
        .configure_mesh()
        .x_desc("Simulation time (ms)")
        .y_desc("Impulse magnitude (mN·s, log scale)")
        .draw()?;
    for (k, color, label) in [
        (0, RGBColor(21, 98, 126), "Applied normal"),
        (1, RGBColor(212, 126, 27), "Applied tangent"),
        (2, purple, "Stored normal"),
        (3, RGBColor(84, 115, 57), "Stored tangent"),
    ] {
        chart
            .draw_series(
                rows.iter()
                    .filter(|r| r.3[k] * 1000. >= 0.006)
                    .map(|r| Circle::new((r.0, 1000. * r.3[k]), 2, color.filled())),
            )?
            .label(label)
            .legend(move |(x, y)| Circle::new((x + 10, y), 4, color.filled()));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.95))
        .draw()?;
    area.present()?;
    Ok(())
}
