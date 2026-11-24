// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recreate the measured humanoid tradeoffs from a saved qualification run.
use plotters::prelude::*;
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, Write},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Deserialize)]
struct Frame {
    tick: u32,
    com: [f64; 3],
    velocity: [f64; 3],
    pelvis_height: f64,
    feet_force_n: [f64; 2],
}
#[derive(Deserialize)]
struct Run {
    mass_kg: f64,
    first_fall: Option<u32>,
    nonfoot_support_ticks: u32,
    maximum_command_nm: f64,
    max_torso_tilt_rad: f64,
    minimum_loaded_foot_support_margin_m_after_100ms: Option<f64>,
    p50_ns: u64,
    p95_ns: u64,
    p99_ns: u64,
    physics_wall_ns: u64,
}
const RUNS: &[&str] = &[
    "baseline",
    "push_forward",
    "push_back",
    "push_left",
    "push_right",
    "push_diagonal",
    "fixed_posture",
    "no_actuation",
    "overload_forward",
    "overload_side",
    "reach",
    "squat",
];
fn read<T: for<'a> Deserialize<'a>>(path: &Path) -> Result<T> {
    Ok(serde_json::from_reader(File::open(path)?)?)
}
fn median(values: &mut [u64]) -> u64 {
    values.sort_unstable();
    values[values.len() / 2]
}
fn error(f: &Frame, reference: &Frame) -> f64 {
    (f.com[0] - reference.com[0]).hypot(f.com[1] - reference.com[1])
}
fn main() -> Result<()> {
    let root = std::env::args_os()
        .nth(1)
        .ok_or("supply humanoid artifact directory")?;
    let root = Path::new(&root);
    let data = root.join("qualified");
    let mut frames = BTreeMap::new();
    for &name in RUNS {
        let mut rows = Vec::new();
        for line in BufReader::new(File::open(data.join(format!("{name}.jsonl")))?).lines() {
            rows.push(serde_json::from_str::<Frame>(&line?)?);
        }
        if rows.len() != 501
            || rows
                .iter()
                .enumerate()
                .any(|(i, r)| r.tick != 10 * i as u32)
        {
            return Err(
                format!("expected a 0–5000 ms ordinary run sampled every 10 ms: {name}").into(),
            );
        }
        frames.insert(name, rows);
    }
    let base = &frames["baseline"];
    let mut out = File::create(root.join("scenarios.csv"))?;
    writeln!(
        out,
        "scenario,first_fall_ms,max_com_displacement_from_baseline_mm,recovery_after_push_ms,max_torso_tilt_rad,min_loaded_support_margin_mm,max_torque_Nm,nonfoot_support_ticks"
    )?;
    for &name in RUNS {
        let r: Run = read(&data.join(format!("{name}.json")))?;
        let mut stable = None;
        let mut max = 0f64;
        for (f, b) in frames[name].iter().zip(base) {
            let delta = error(f, b);
            max = max.max(delta);
            if f.tick >= 1080 && delta <= 0.002 && f.velocity[0].hypot(f.velocity[1]) <= 0.005 {
                stable.get_or_insert(f.tick - 1080);
            } else {
                stable = None;
            }
        }
        let recovery = if name.starts_with("push_") || name == "fixed_posture" {
            stable
        } else {
            None
        };
        writeln!(
            out,
            "{name},{},{:.6},{},{:.8},{:.6},{:.4},{}",
            r.first_fall.map(|n| n.to_string()).unwrap_or_default(),
            max * 1000.,
            recovery.map(|n| n.to_string()).unwrap_or_default(),
            r.max_torso_tilt_rad,
            1000.
                * r.minimum_loaded_foot_support_margin_m_after_100ms
                    .unwrap_or_default(),
            r.maximum_command_nm,
            r.nonfoot_support_ticks
        )?;
    }
    let mut out = File::create(root.join("cost.csv"))?;
    writeln!(
        out,
        "controller,trials,total_intervals,median_p50_us,median_p95_us,median_p99_us,median_solver_s,median_real_time_factor"
    )?;
    for name in ["balance", "posture", "passive"] {
        let trials = (1..=5)
            .map(|i| read::<Run>(&root.join(format!("timing/{name}-{i}.json"))))
            .collect::<Result<Vec<_>>>()?;
        let mid = |f: fn(&Run) -> u64| median(&mut trials.iter().map(f).collect::<Vec<_>>()) as f64;
        let seconds = mid(|r| r.physics_wall_ns) * 1e-9;
        writeln!(
            out,
            "{name},5,25000,{:.3},{:.3},{:.3},{seconds:.6},{:.4}",
            mid(|r| r.p50_ns) * 1e-3,
            mid(|r| r.p95_ns) * 1e-3,
            mid(|r| r.p99_ns) * 1e-3,
            5. / seconds
        )?;
    }
    let path = root.join("standing.svg");
    let area = SVGBackend::new(&path, (1050, 870)).into_drawing_area();
    area.fill(&WHITE)?;
    let sub = area.split_evenly((3, 1));
    let (blue, teal, orange, gray) = (
        RGBColor(47, 91, 155),
        RGBColor(17, 130, 127),
        RGBColor(185, 98, 27),
        RGBColor(102, 109, 122),
    );
    let draw = |i: usize,
                title: &str,
                y: &str,
                yr: std::ops::Range<f64>,
                data: Vec<(&str, RGBColor, Vec<(f64, f64)>)>|
     -> Result<()> {
        let mut c = ChartBuilder::on(&sub[i])
            .margin(14)
            .caption(title, ("sans-serif", 22))
            .x_label_area_size(38)
            .y_label_area_size(64)
            .build_cartesian_2d(0f64..5., yr)?;
        c.configure_mesh()
            .light_line_style(RGBColor(228, 230, 235))
            .x_desc("Simulation time (s)")
            .y_desc(y)
            .draw()?;
        for (label, color, points) in data {
            c.draw_series(LineSeries::new(points, color.stroke_width(3)))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 25, y)], color.stroke_width(3))
                });
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .border_style(RGBColor(200, 204, 209))
            .background_style(WHITE.mix(0.92))
            .draw()?;
        Ok(())
    };
    let trace = |name: &str, f: fn(&Frame) -> f64| {
        frames[name]
            .iter()
            .map(|r| (r.tick as f64 * 1e-3, f(r)))
            .collect::<Vec<_>>()
    };
    draw(
        0,
        "Recovery after the 80 ms chest push at 1 s",
        "COM shift (mm)",
        0. ..60.,
        vec![
            (
                "140 N forward",
                blue,
                frames["push_forward"]
                    .iter()
                    .zip(base)
                    .map(|(a, b)| (a.tick as f64 * 1e-3, 1000. * error(a, b)))
                    .collect(),
            ),
            (
                "180 N left",
                teal,
                frames["push_left"]
                    .iter()
                    .zip(base)
                    .map(|(a, b)| (a.tick as f64 * 1e-3, 1000. * error(a, b)))
                    .collect(),
            ),
            (
                "140 N, fixed posture",
                orange,
                frames["fixed_posture"]
                    .iter()
                    .zip(base)
                    .map(|(a, b)| (a.tick as f64 * 1e-3, 1000. * error(a, b)))
                    .collect(),
            ),
        ],
    )?;
    let mass: Run = read(&data.join("push_forward.json"))?;
    draw(
        1,
        "Two loaded soles during forward recovery",
        "Ground reaction (N)",
        0. ..850.,
        vec![
            (
                "Left + right",
                blue,
                trace("push_forward", |r| r.feet_force_n.iter().sum()),
            ),
            (
                "Left foot",
                teal,
                trace("push_forward", |r| r.feet_force_n[0]),
            ),
            (
                "Weight",
                gray,
                vec![(0., mass.mass_kg * 9.81), (5., mass.mass_kg * 9.81)],
            ),
        ],
    )?;
    draw(
        2,
        "Pelvis height with active and failed support",
        "Height (m)",
        0. ..1.,
        vec![
            (
                "140 N, balance",
                blue,
                trace("push_forward", |r| r.pelvis_height),
            ),
            (
                "500 N, balance",
                orange,
                trace("overload_forward", |r| r.pelvis_height),
            ),
            (
                "140 N, passive",
                gray,
                trace("no_actuation", |r| r.pelvis_height),
            ),
        ],
    )?;
    area.present()?;
    if fs::metadata(&path)?.len() == 0 {
        return Err("empty plot".into());
    }
    Ok(())
}
