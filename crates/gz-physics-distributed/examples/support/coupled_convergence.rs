// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Refine the same loaded cut while varying the direct solver's post-drive timestep.
use super::{Case, Drive, Inertia, Interval, Observer, Result, Trial, point};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
fn draw(dir: &Path, rows: &[Value], trace: &[Value]) -> Result<()> {
    let path = dir.join("work-bound.svg");
    let area = SVGBackend::new(&path, (1200, 550)).into_drawing_area();
    area.fill(&WHITE)?;
    let area = area.titled(
        "Combined sliding and yaw on a moving rough table",
        ("sans-serif", 29),
    )?;
    let p = area.margin(8, 15, 15, 15).split_evenly((1, 2));
    let orange = RGBColor(188, 85, 31);
    let green = RGBColor(38, 126, 111);
    let blue = RGBColor(48, 92, 157);
    let mut c = ChartBuilder::on(&p[0])
        .caption("Opposing work against each patch bound", ("sans-serif", 19))
        .margin(10)
        .x_label_area_size(53)
        .y_label_area_size(67)
        .build_cartesian_2d(0.30_f64..1.20, 0.0_f64..1.72)?;
    c.configure_mesh()
        .x_desc("Time in the moving-table trial (s)")
        .y_desc("W / B, peak in 10 ms window")
        .x_labels(7)
        .y_labels(7)
        .max_light_lines(1)
        .bold_line_style(RGBColor(220, 225, 230))
        .light_line_style(RGBColor(237, 240, 242))
        .draw()?;
    c.draw_series(LineSeries::new(
        [(0.3, 1.), (1.2, 1.)],
        RGBColor(122, 128, 134).stroke_width(2),
    ))?
    .label("Pointwise ceiling")
    .legend(|(x, y)| {
        PathElement::new(
            vec![(x, y), (x + 18, y)],
            RGBColor(122, 128, 134).stroke_width(2),
        )
    });
    for (which, name, color) in [(0, "Grouped", orange), (1, "Per corner", green)] {
        let iter = trace.iter().filter_map(|r| {
            let t = r["tick"].as_f64()? * 0.01;
            let raw = if which == 0 { r } else { &r["point_control"] };
            let y = raw["microsteps"]
                .as_array()?
                .iter()
                .filter(|m| m["eligible"] == true)
                .filter_map(|m| m["ratio"].as_f64())
                .max_by(f64::total_cmp)?;
            Some((t, y))
        });
        c.draw_series(LineSeries::new(iter, color.stroke_width(3)))?
            .label(name)
            .legend(move |(x, y)| {
                PathElement::new(vec![(x, y), (x + 18, y)], color.stroke_width(3))
            });
    }
    c.configure_series_labels()
        .position(SeriesLabelPosition::LowerLeft)
        .background_style(WHITE.mix(0.85))
        .border_style(RGBColor(207, 213, 219))
        .draw()?;
    let mut c = ChartBuilder::on(&p[1])
        .caption(
            "Does a smaller native step remove the excess?",
            ("sans-serif", 19),
        )
        .margin(10)
        .x_label_area_size(53)
        .y_label_area_size(68)
        .build_cartesian_2d((0.027_f64..12.).log_scale(), 0.80_f64..1.95)?;
    c.configure_mesh()
        .x_desc("Native step after the drive (ms, log scale)")
        .y_desc("Largest eligible W / B")
        .x_labels(7)
        .y_labels(7)
        .max_light_lines(1)
        .bold_line_style(RGBColor(220, 225, 230))
        .light_line_style(RGBColor(237, 240, 242))
        .draw()?;
    c.draw_series(LineSeries::new(
        [(0.03125, 1.), (10., 1.)],
        RGBColor(122, 128, 134).stroke_width(2),
    ))?;
    for (point, color) in [(false, orange), (true, green)] {
        let pts = rows
            .iter()
            .filter(|r| r["pointwise"] == point)
            .map(|r| {
                (
                    r["dt_ns"].as_f64().unwrap() / 1e6,
                    r["maximum_ratio"].as_f64().unwrap(),
                )
            })
            .collect::<Vec<_>>();
        c.draw_series(LineSeries::new(pts.iter().copied(), color.stroke_width(3)))?;
        c.draw_series(pts.iter().map(|v| Circle::new(*v, 4, color.filled())))?;
    }
    c.draw_series(std::iter::once(Text::new(
        "Pointwise ceiling",
        (0.05, 1.025),
        ("sans-serif", 14).into_font().color(&blue),
    )))?;
    area.present()?;
    Ok(())
}
pub fn run(source: &Path, directory: &Path) -> Result<()> {
    let case = Case {
        square: false,
        moving: true,
        smooth: false,
        coarse: false,
        drive: Drive::Mixed,
    };
    let seeds = case.seeds();
    let inertia = Inertia::new(&seeds[1], &seeds[0]);
    let trace = BufReader::new(File::open(source)?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect::<Result<Vec<Value>>>()?;
    if trace.len() != 120 {
        return Err("full representative study required".into());
    }
    let mut rows = vec![];
    for dt_ns in [
        10_000_000_u64,
        5_000_000,
        2_000_000,
        1_000_000,
        500_000,
        250_000,
        125_000,
        62_500,
        31_250,
    ] {
        for pointwise in [false, true] {
            let mut direct = point::Direct::new(&seeds, pointwise);
            let mut trial = Trial::default();
            for r in &trace[..30] {
                let rate = r["substeps"].as_u64().ok_or("missing rate")?;
                for _ in 0..rate {
                    direct.step(0.01 / rate as f64);
                }
            }
            let verify = |direct: &point::Direct, frame: &Value| -> Result<()> {
                let bodies = if pointwise {
                    &frame["point_control"]["bodies"]
                } else {
                    &frame["bodies"]
                };
                let bodies: [super::Body; 2] = serde_json::from_value(bodies.clone())?;
                for (i, b) in bodies.into_iter().enumerate() {
                    let state = direct.state(i);
                    if (
                        state.position,
                        state.orientation,
                        state.velocity,
                        state.angular_velocity,
                    ) != (b.position, b.orientation, b.linear, b.angular)
                    {
                        return Err(
                            "mismatched direct world pose or twist at the measured barrier".into(),
                        );
                    }
                }
                Ok(())
            };
            verify(&direct, &trace[29])?;
            direct.kick([4.8, 0., 0.], [0., 0., 0.96]);
            let dt = dt_ns as f64 / 1e9;
            let per = 10_000_000 / dt_ns;
            let mut saturated = 0.;
            let mut all_work = 0.;
            let mut settled_peak = 0.0_f64;
            for tick in 31..=120 {
                let mut interval = Interval::default();
                let mut observer = Observer {
                    i: &inertia,
                    trial: &mut trial,
                    interval: &mut interval,
                    keep: false,
                    tick,
                    dt,
                };
                for sub in 0..per {
                    let pre = direct.state(1);
                    direct.step(dt);
                    let post = direct.state(1);
                    let points = direct.points().into_iter().map(|mut p| {
                        if !pointwise {
                            p.tangent = None;
                        }
                        p
                    });
                    observer.observe(pre, post, points, sub);
                }
                if tick >= 36 {
                    settled_peak = settled_peak.max(interval.max_ratio);
                }
                saturated += interval.bound;
                all_work += interval.work;
            }
            if dt_ns == 1_000_000 {
                verify(&direct, &trace[119])?;
            }
            let last = direct.state(1);
            rows.push(json!({
                "dt_ns": dt_ns,
                "pointwise": pointwise,
                "native_steps_after_kick": 90*per,
                "eligible": trial.eligible,
                "violations": trial.violations,
                "maximum_ratio": trial.max_ratio,
                "maximum_ratio_after_50ms": settled_peak,
                "peak": trial.peak,
                "sum_declared_excess_j": trial.sum_excess,
                "sum_bound_j": saturated,
                "sum_work_j": all_work,
                "max_cone_excess_ns": trial.cone_excess,
                "end_position": last.position,
                "end_orientation": last.orientation,
                "end_linear": last.velocity,
                "end_angular": last.angular_velocity,
                "end_energy_j": inertia.energy(last),
                "max_cumulative_energy_error_j": trial.max_cumulative_residual
            }));
        }
    }
    serde_json::to_writer_pretty(File::create_new(directory.join("summary.json"))?, &rows)?;
    draw(directory, &rows, &trace)?;
    Ok(())
}
