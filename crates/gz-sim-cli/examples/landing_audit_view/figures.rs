// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "tables.rs"]
mod tables;
use super::{Path, Result, Trace, read};
use plotters::prelude::*;
const SWITCHED: RGBColor = RGBColor(10, 123, 115);
const CONTROL: RGBColor = RGBColor(48, 91, 171);
const GRID: RGBColor = RGBColor(226, 231, 235);
pub(super) fn draw(audit: &Path, out: &Path) -> Result<()> {
    std::fs::create_dir_all(out)?;
    let mut trials = Vec::new();
    for step in [500, 250, 125] {
        trials.push(read::<Trace>(&audit.join(format!("sweep/{step}us.json")))?);
    }
    response(&trials, &out.join("timestep.svg"))?;
    corner_response(&trials, &out.join("corners.svg"))?;
    tables::sweep(audit, out)?;
    tables::matched(audit, out)?;
    tables::admission(audit, out)?;
    tables::rigid(audit, &trials, out)?;
    Ok(())
}
fn response(trials: &[Trace], out: &Path) -> Result<()> {
    let root = SVGBackend::new(out, (1200, 820)).into_drawing_area();
    root.fill(&WHITE)?;
    let area = root
        .titled(
            "Landing impulse across native timesteps",
            ("sans-serif", 26),
        )?
        .split_evenly((3, 3));
    for (column, trace) in trials.iter().enumerate() {
        for row in 0..3 {
            let get = |l: &super::Landing| match row {
                0 => l.first_ns,
                1 => l.first_ns + l.second_ns,
                _ => l.sum_5ms_ns,
            };
            let max = trials
                .iter()
                .flat_map(|t| t.cycles.iter())
                .flat_map(|c| [get(&c.switched_landing), get(&c.reference_landing)])
                .fold(0f64, f64::max);
            let measure = ["First step", "First two steps", "First 5 ms"][row];
            let mut chart = ChartBuilder::on(&area[row * 3 + column])
                .margin(10)
                .caption(
                    format!("{measure}   |   {} µs", trace.step_us),
                    ("sans-serif", 19),
                )
                .x_label_area_size(42)
                .y_label_area_size(62)
                .build_cartesian_2d(0.55..3.55, 0.0..max * 1.22)?;
            chart
                .configure_mesh()
                .x_labels(3)
                .x_label_formatter(&|x: &f64| {
                    if x.fract().abs() < 1e-8 {
                        format!("{x:.0}")
                    } else {
                        String::new()
                    }
                })
                .x_desc("Landing")
                .y_desc("N·s")
                .max_light_lines(1)
                .light_line_style(GRID)
                .draw()?;
            for (name, color, offset) in [
                ("Switched", SWITCHED, -0.13),
                ("Always dynamic", CONTROL, 0.13),
            ] {
                chart
                    .draw_series(trace.cycles.iter().map(|c| {
                        let x = c.cycle as f64 + offset;
                        let l = if offset < 0. {
                            &c.switched_landing
                        } else {
                            &c.reference_landing
                        };
                        Rectangle::new([(x - 0.11, 0.), (x + 0.11, get(l))], color.filled())
                    }))?
                    .label(name)
                    .legend(move |(x, y)| {
                        Rectangle::new([(x, y - 4), (x + 12, y + 4)], color.filled())
                    });
            }
            if row == 0 && column == 0 {
                chart
                    .configure_series_labels()
                    .position(SeriesLabelPosition::UpperRight)
                    .background_style(WHITE.mix(0.94))
                    .label_font(("sans-serif", 12))
                    .draw()?;
            }
        }
    }
    root.present()?;
    Ok(())
}

fn corner_response(trials: &[Trace], out: &Path) -> Result<()> {
    let root = SVGBackend::new(out, (1120, 800)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(
            "Third landing by sole corner and native solve",
            ("sans-serif", 26),
        )?
        .split_evenly((3, 2));
    let colors = [
        RGBColor(31, 111, 152),
        RGBColor(215, 124, 19),
        RGBColor(17, 135, 97),
        RGBColor(139, 78, 145),
    ];
    let labels = ["−X −Y", "−X +Y", "+X −Y", "+X +Y"];
    let mut graphs = Vec::new();
    let mut max = 0f64;
    for trace in trials {
        let cycle = trace.cycles.last().ok_or("missing final landing")?;
        for (switched, impact) in [
            (true, &cycle.switched_landing),
            (false, &cycle.reference_landing),
        ] {
            let mut data = Vec::new();
            for s in &trace.frames {
                if s.tick >= impact.tick && (s.tick - impact.tick) * trace.step_us < 5000 {
                    let j = super::corners(if switched { &s.switched } else { &s.reference })?;
                    max = j.into_iter().fold(max, f64::max);
                    data.push((
                        (s.tick - impact.tick) as f64 * trace.step_us as f64 / 1000.,
                        j,
                    ));
                }
            }
            if data.len() as u64 * trace.step_us != 5000 {
                return Err("partial landing window".into());
            }
            graphs.push((trace.step_us, switched, data));
        }
    }
    for (index, (us, switched, data)) in graphs.iter().enumerate() {
        let name = if *switched {
            "Switched"
        } else {
            "Always dynamic"
        };
        let mut chart = ChartBuilder::on(&areas[index])
            .margin(10)
            .caption(format!("{name}   |   {us} µs"), ("sans-serif", 18))
            .x_label_area_size(42)
            .y_label_area_size(58)
            .build_cartesian_2d(-0.12..5.02, 0.0..max * 1.14)?;
        chart
            .configure_mesh()
            .y_desc("Impulse (N·s)")
            .x_desc("Time since lane’s landing solve (ms)")
            .max_light_lines(1)
            .light_line_style(GRID)
            .draw()?;
        for (corner, color) in colors.into_iter().enumerate() {
            let values = data.iter().map(|(t, j)| (*t, j[corner]));
            chart
                .draw_series(LineSeries::new(values.clone(), color.stroke_width(2)))?
                .label(labels[corner])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 16, y)], color.stroke_width(2))
                });
            chart.draw_series(values.map(|v| Circle::new(v, 3, color.filled())))?;
        }
        if index == 0 {
            chart
                .configure_series_labels()
                .background_style(WHITE.mix(0.94))
                .label_font(("sans-serif", 13))
                .draw()?;
        }
    }
    root.present()?;
    Ok(())
}
