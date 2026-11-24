// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Row};
use plotters::prelude::*;
use std::path::Path;

pub(super) fn write(rows: &[Row], path: &Path) -> Result<()> {
    let root = SVGBackend::new(path, (1080, 1080)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root.split_evenly((3, 2));
    type Measure = fn(&Row) -> f64;
    let metrics: [(&str, &str, Measure); 6] = [
        ("Peak position difference between layouts", "mm", |r| {
            1000. * r.peak_delta
        }),
        ("Peak horizontal drift from initial position", "mm", |r| {
            1000. * r.peak_xy
        }),
        ("Final half-second height error", "mm", |r| 1000. * r.late_z),
        ("Settled continuously through 3 s", "seconds", |r| {
            r.settle as f64 / 1000.
        }),
        ("Worst mean floor load error", "mN", |r| {
            1000. * r.floor_load_error
        }),
        ("Worst mean interface load error", "mN", |r| {
            1000. * r.pair_load_error
        }),
    ];
    let &largest = rows.iter().map(|r| &r.count).max().unwrap();
    for (i, (area, (title, unit, measure))) in panels.iter().zip(metrics).enumerate() {
        let mut groups = Vec::new();
        if i == 0 {
            let mut counts: Vec<_> = rows.iter().map(|r| r.count).collect();
            counts.sort_unstable();
            counts.dedup();
            for (index, n) in counts.into_iter().enumerate() {
                let points = rows
                    .iter()
                    .filter(|r| r.count == n && r.solver == "distributed")
                    .map(|r| (r.step as f64, measure(r)))
                    .collect::<Vec<_>>();
                groups.push((
                    format!("{n} boxes"),
                    [
                        RGBColor(111, 131, 140),
                        RGBColor(25, 130, 167),
                        RGBColor(82, 71, 172),
                        RGBColor(212, 92, 42),
                    ][index % 4],
                    points,
                ));
            }
        } else {
            for (solver, color) in [
                ("distributed", RGBColor(25, 130, 167)),
                ("monolithic", RGBColor(212, 92, 42)),
            ] {
                let points = rows
                    .iter()
                    .filter(|r| r.count == largest && r.solver == solver)
                    .map(|r| (r.step as f64, measure(r)))
                    .collect::<Vec<_>>();
                groups.push((format!("{solver} ({largest})"), color, points));
            }
        }
        let max = groups
            .iter()
            .flat_map(|(_, _, p)| p.iter().map(|p| p.1))
            .fold(0., f64::max);
        let mut chart = ChartBuilder::on(area)
            .caption(title, ("sans-serif", 20))
            .margin(12)
            .x_label_area_size(42)
            .y_label_area_size(63)
            .build_cartesian_2d(
                0f64..(rows.iter().map(|r| r.step).max().unwrap() as f64 * 1.06),
                0f64..(max * 1.15).max(0.001),
            )?;
        chart
            .configure_mesh()
            .x_desc("Public step (ms)")
            .y_desc(unit)
            .x_labels(6)
            .y_labels(6)
            .max_light_lines(1)
            .light_line_style(RGBColor(225, 230, 234))
            .draw()?;
        for (label, color, mut points) in groups {
            points.sort_by(|a, b| a.0.total_cmp(&b.0));
            chart
                .draw_series(LineSeries::new(
                    points.iter().copied(),
                    color.stroke_width(2),
                ))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 16, y)], color.stroke_width(2))
                });
            chart.draw_series(points.iter().map(|p| Circle::new(*p, 4, color.filled())))?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.88))
            .border_style(RGBColor(200, 205, 208))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

pub(super) fn options(data: &[(u64, usize, f64, f64, f64)], path: &Path) -> Result<()> {
    let root = SVGBackend::new(path, (1040, 410)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root.split_evenly((1, 2));
    for (i, panel) in panels.iter().enumerate() {
        let mut series = Vec::new();
        let mut timesteps: Vec<_> = data.iter().map(|r| r.0).collect();
        timesteps.sort_unstable();
        timesteps.dedup();
        for (j, h) in timesteps.into_iter().enumerate() {
            let points = data
                .iter()
                .filter(|r| r.0 == h)
                .map(|r| (r.1 as f64, 1000. * if i == 0 { r.2 } else { r.3.max(r.4) }))
                .collect::<Vec<_>>();
            series.push((
                h,
                [
                    RGBColor(101, 89, 160),
                    RGBColor(11, 120, 142),
                    RGBColor(212, 91, 44),
                    RGBColor(67, 75, 84),
                ][j % 4],
                points,
            ));
        }
        let limit = series
            .iter()
            .flat_map(|(_, _, p)| p.iter().map(|p| p.1))
            .fold(0., f64::max);
        let mut chart = ChartBuilder::on(panel)
            .caption(
                [
                    "Difference between the two native layouts",
                    "Maximum horizontal drift in either layout",
                ][i],
                ("sans-serif", 19),
            )
            .margin(14)
            .x_label_area_size(52)
            .y_label_area_size(57)
            .build_cartesian_2d(0.7f64..8.3, 0f64..(limit * 1.13).max(1.))?;
        chart
            .configure_mesh()
            .x_desc("Inner PGS iterations per substep")
            .y_desc("mm")
            .x_labels(6)
            .y_labels(6)
            .max_light_lines(1)
            .light_line_style(RGBColor(225, 230, 234))
            .draw()?;
        for (h, color, p) in series {
            chart
                .draw_series(LineSeries::new(p.iter().copied(), color.stroke_width(2)))?
                .label(format!("{h} ms"))
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 15, y)], color.stroke_width(2))
                });
            chart.draw_series(p.into_iter().map(|q| Circle::new(q, 4, color.filled())))?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.9))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
