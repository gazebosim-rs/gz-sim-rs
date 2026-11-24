// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched lateral capture controls and physical first/sustained touchdown measurements.
use plotters::prelude::*;
use serde::Deserialize;
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader, Write},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    com: [f64; 3],
    velocity: [f64; 3],
    feet_force_n: [f64; 2],
    loaded_sole_points: [usize; 2],
    foot_bottom_m: [f64; 2],
    foot_centers_m: [[f64; 3]; 2],
    tilt_rad: f64,
}
#[derive(Deserialize)]
struct Summary {
    push_at_tick: u64,
    push_n: [f64; 3],
    first_fall: Option<u64>,
    nonfoot_support_ticks: u64,
    max_torso_tilt_rad: f64,
    maximum_command_nm: f64,
    max_cumulative_momentum_residual_ns: f64,
}
fn frames(p: &Path) -> Result<Vec<Frame>> {
    Ok(BufReader::new(File::open(p)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect::<Result<_>>()?)
}
fn main() -> Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .ok_or("supply humanoid recovery artifact directory")?;
    let root = Path::new(&path);
    if root.join("acquired").is_dir() {
        return acquired(root);
    }
    let comparison = root.parent().unwrap().join("multifidelity-humanoid-step");
    let mut table = File::create(root.join("matched.csv"))?;
    writeln!(
        table,
        "axis,push_N,feedback,fall_ms,first_right_contact_after_push_ms,begin_continuous_bilateral_50ms,peak_right_N_in_100ms_after_touch,terminal_right_x_travel_mm,nonfoot_support_ticks,peak_torso_tilt_rad,peak_effort_Nm,peak_cumulative_momentum_residual_Ns,observation_spacing_ms"
    )?;
    let mut figures = Vec::new();
    for axis in ["x", "y"] {
        for force in [-100, -60, -30, 30, 60, 100] {
            for recovery in [false, true] {
                let label = format!("{axis}-{force}");
                let sensitive = axis == "y" && (force == -60 || force == -100);
                let base = if sensitive {
                    root.join("transitions").join(format!(
                        "{}-{label}",
                        if recovery { "early" } else { "planned" }
                    ))
                } else if recovery {
                    root.join("qualified").join(&label)
                } else {
                    comparison
                        .join("perturbations")
                        .join(format!("closed-{label}"))
                };
                let rows = frames(&base.with_extension("jsonl"))?;
                let report: Summary =
                    serde_json::from_reader(File::open(base.with_extension("json"))?)?;
                let stride = rows[1].tick - rows[0].tick;
                if rows.last().unwrap().tick != 10000
                    || rows.windows(2).any(|p| p[1].tick != p[0].tick + stride)
                {
                    return Err("incomplete physical trace".into());
                }
                let index = rows
                    .iter()
                    .position(|f| {
                        f.tick >= 3380 && f.feet_force_n[1] > 1. && f.loaded_sole_points[1] > 0
                    })
                    .ok_or("right foot never touched")?;
                let first = &rows[index];
                let dwell = (50u64.div_ceil(stride)) as usize;
                let bilateral = rows[index..]
                    .windows(dwell)
                    .find(|w| w.iter().all(|r| r.feet_force_n.iter().all(|f| *f > 1.)))
                    .map(|w| w[0].tick);
                let peak = rows[index..]
                    .iter()
                    .take_while(|f| f.tick <= first.tick + 100)
                    .map(|f| f.feet_force_n[1])
                    .fold(0., f64::max);
                let dx = rows.last().unwrap().foot_centers_m[1][0] - rows[0].foot_centers_m[1][0];
                writeln!(
                    table,
                    "{axis},{force},{},{},{},{},{peak:.3},{:.4},{},{:.7},{:.5},{:.7},{stride}",
                    if recovery { "early" } else { "planned" },
                    report.first_fall.map(|v| v.to_string()).unwrap_or_default(),
                    first.tick,
                    bilateral.map(|v| v.to_string()).unwrap_or_default(),
                    dx * 1000.,
                    report.nonfoot_support_ticks,
                    report.max_torso_tilt_rad,
                    report.maximum_command_nm,
                    report.max_cumulative_momentum_residual_ns
                )?;
                if axis == "y" && force == -60 {
                    figures.push(rows);
                }
            }
        }
    }
    let file = root.join("recovery.svg");
    let area = SVGBackend::new(&file, (1050, 875)).into_drawing_area();
    area.fill(&WHITE)?;
    let areas = area.split_evenly((3, 1));
    let samples: [(&str, &str, std::ops::Range<f64>, fn(&Frame) -> f64); 3] = [
        (
            "Right foot reaches the floor earlier after the −60 N swing-side push",
            "Sole clearance (mm)",
            -12.0..65.,
            |f| f.foot_bottom_m[1] * 1000.,
        ),
        (
            "Load acquired by the released right foot",
            "Normal force (N)",
            -30.0..2000.,
            |f| f.feet_force_n[1],
        ),
        (
            "Torso response under the same 4.8 N·s chest impulse",
            "Torso tilt (rad)",
            -0.03..1.9,
            |f| f.tilt_rad,
        ),
    ];
    let colors = [RGBColor(190, 96, 42), RGBColor(32, 117, 139)];
    for (i, (title, y, yr, field)) in samples.into_iter().enumerate() {
        let mut c = ChartBuilder::on(&areas[i])
            .margin(14)
            .caption(title, ("sans-serif", 22))
            .x_label_area_size(37)
            .y_label_area_size(65)
            .build_cartesian_2d(3.1..7.3, yr.clone())?;
        c.configure_mesh()
            .x_desc("Simulation time (s)")
            .y_desc(y)
            .light_line_style(RGBColor(228, 231, 235))
            .draw()?;
        c.draw_series(std::iter::once(Rectangle::new(
            [(3.38, yr.start), (3.46, yr.end)],
            RGBColor(240, 232, 203).filled(),
        )))?;
        for (k, rows) in figures.iter().enumerate() {
            let color = colors[k];
            c.draw_series(LineSeries::new(
                rows.iter()
                    .filter(|f| (3100..7300).contains(&f.tick))
                    .map(|f| (f.tick as f64 * 0.001, field(f))),
                color.stroke_width(2),
            ))?
            .label(["Scheduled swing", "Capture-triggered landing"][k])
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2)));
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.9))
            .draw()?;
    }
    area.present()?;
    let mut cases = Vec::new();
    for item in std::fs::read_dir(root.join("envelope"))? {
        let path = item?.path();
        if path.extension().and_then(|v| v.to_str()) != Some("json") {
            continue;
        }
        let case: Summary = serde_json::from_reader(File::open(&path)?)?;
        let rows = frames(&path.with_extension("jsonl"))?;
        let end = rows.last().ok_or("missing envelope measurements")?;
        if end.tick != 10000 {
            return Err("incomplete envelope trial".into());
        }
        cases.push((
            case,
            end.foot_centers_m[1][0] - rows[0].foot_centers_m[1][0],
        ));
    }
    cases.sort_by(|(a, _), (b, _)| {
        a.push_at_tick
            .cmp(&b.push_at_tick)
            .then_with(|| b.push_n[1].total_cmp(&a.push_n[1]))
    });
    let mut envelope = File::create(root.join("envelope.csv"))?;
    writeln!(
        envelope,
        "start_ms,lateral_force_N,fall_ms,peak_torso_tilt_rad,nonfoot_support_ticks,terminal_right_x_travel_mm"
    )?;
    for (r, distance) in cases {
        writeln!(
            envelope,
            "{},{},{},{:.6},{},{:.3}",
            r.push_at_tick,
            r.push_n[1],
            r.first_fall.map(|x| x.to_string()).unwrap_or_default(),
            r.max_torso_tilt_rad,
            r.nonfoot_support_ticks,
            distance * 1000.
        )?;
    }
    let zero = frames(&root.join("transitions/early-0.jsonl"))?;
    let ordinary = frames(&comparison.join("qualification/step.jsonl"))?;
    if ordinary.len() != zero.len()
        || ordinary.iter().zip(zero).any(|(a, b)| {
            a.tick != b.tick
                || a.com != b.com
                || a.velocity != b.velocity
                || a.feet_force_n != b.feet_force_n
                || a.foot_centers_m != b.foot_centers_m
                || a.tilt_rad != b.tilt_rad
        })
    {
        return Err("unforced recovery changed the nominal physical trace".into());
    }
    println!(
        "All 10,001 nominal motion/contact measurements matched with and without early landing."
    );
    Ok(())
}

/// Compare the same force, onset and physical duration around the missed-foothold witness.
fn acquired(root: &Path) -> Result<()> {
    let before = root
        .parent()
        .ok_or("missing sibling recovery measurements")?
        .join("multifidelity-humanoid-recovery");
    let mut cases: Vec<_> = [2400, 2700, 3000, 3380, 3700, 4100, 4450]
        .into_iter()
        .flat_map(|t| [-60, -100].map(|f| (t, f)))
        .collect();
    cases.extend([-120, -160, -200, -260, -350, -500].map(|f| (3380, f)));
    let mut table = File::create(root.join("acquired-pushes.csv"))?;
    writeln!(
        table,
        "begin_ms,force_N,controller,first_fall_ms,peak_tilt_rad,nonfoot_support_ticks,right_foot_terminal_travel_mm,peak_accumulated_momentum_residual_Ns"
    )?;
    let mut fig = Vec::new();
    for (t, f) in cases {
        let previous = if t != 3380 {
            before.join("envelope").join(format!("t{t}-{f}"))
        } else if f >= -100 {
            before.join("transitions").join(format!("early-y-{f}"))
        } else {
            before.join("envelope").join(format!("load-{f}"))
        };
        let new = root.join("acquired").join(format!("t{t}-{f}"));
        for (label, base) in [("early", previous), ("acquired", new)] {
            let rows = frames(&base.with_extension("jsonl"))?;
            let summary: Summary =
                serde_json::from_reader(File::open(base.with_extension("json"))?)?;
            let first = rows.first().ok_or("empty acquired-foot trace")?;
            let last = rows.last().unwrap();
            let spacing = rows.get(1).ok_or("one-frame acquired trace")?.tick - first.tick;
            if first.tick != 0
                || last.tick != 10000
                || !rows.windows(2).all(|r| r[1].tick == r[0].tick + spacing)
                || summary.push_at_tick != t
                || summary.push_n != [0., f64::from(f), 0.]
            {
                return Err("incomplete or mismatched acquired-foot trial".into());
            }
            let travel = (last.foot_centers_m[1][0] - first.foot_centers_m[1][0]) * 1000.;
            writeln!(
                table,
                "{t},{f},{label},{},{:.7},{},{travel:.4},{:.7}",
                summary
                    .first_fall
                    .map(|n| n.to_string())
                    .unwrap_or_default(),
                summary.max_torso_tilt_rad,
                summary.nonfoot_support_ticks,
                summary.max_cumulative_momentum_residual_ns
            )?;
            if (t, f) == (3000, -60) {
                fig.push(rows);
            }
        }
    }
    let canvas = root.join("acquired-push.svg");
    let backend = SVGBackend::new(&canvas, (1050, 900)).into_drawing_area();
    backend.fill(&WHITE)?;
    let axes = backend.split_evenly((3, 1));
    type Axis = (
        &'static str,
        &'static str,
        std::ops::Range<f64>,
        fn(&Frame) -> f64,
    );
    let series: [Axis; 3] = [
        (
            "Swing-foot contact under the 3.000 s chest push",
            "Right foot load (N)",
            -30.0..2400.,
            |r| r.feet_force_n[1],
        ),
        (
            "Right foot position along the walking direction",
            "Foot center X (mm)",
            -150.0..147.,
            |r| 1000. * r.foot_centers_m[1][0],
        ),
        (
            "Whole-body recovery from the early foothold",
            "Torso inclination (rad)",
            -0.03..1.80,
            |r| r.tilt_rad,
        ),
    ];
    for (panel, (title, label, y, field)) in axes.iter().zip(series) {
        let mut c = ChartBuilder::on(panel)
            .margin(12)
            .caption(title, ("sans-serif", 21))
            .x_label_area_size(35)
            .y_label_area_size(72)
            .build_cartesian_2d(2.4..8.0, y.clone())?;
        c.configure_mesh()
            .x_desc("Simulation time (s)")
            .y_desc(label)
            .light_line_style(RGBColor(226, 231, 235))
            .draw()?;
        c.draw_series(std::iter::once(Rectangle::new(
            [(3., y.start), (3.080, y.end)],
            RGBColor(246, 235, 197).filled(),
        )))?;
        for (rows, (name, color)) in fig.iter().zip([
            ("Early landing", RGBColor(185, 91, 38)),
            ("Acquired foothold", RGBColor(0, 114, 134)),
        ]) {
            c.draw_series(LineSeries::new(
                rows.iter()
                    .filter(|f| (2400..=8000).contains(&f.tick))
                    .map(|f| (f.tick as f64 * 0.001, field(f))),
                color.stroke_width(2),
            ))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2)));
        }
        c.configure_series_labels()
            .background_style(WHITE.mix(0.9))
            .border_style(RGBColor(180, 188, 192))
            .draw()?;
    }
    backend.present()?;
    Ok(())
}
