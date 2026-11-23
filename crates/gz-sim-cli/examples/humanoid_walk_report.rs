// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Load-bearing left/right foot placements and stability during an alternating walk.
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
struct Sample {
    tick: u64,
    com: [f64; 3],
    feet_force_n: [f64; 2],
    foot_bottom_m: [f64; 2],
    foot_centers_m: [[f64; 3]; 2],
    loaded_sole_points: [u32; 2],
    capture_margin_m: Option<f64>,
    tilt_rad: f64,
    other_force_n: f64,
}
#[derive(Deserialize)]
struct Run {
    first_fall: Option<u64>,
    max_torso_tilt_rad: f64,
    nonfoot_support_ticks: u64,
}
fn trace(p: &Path) -> Result<Vec<Sample>> {
    let rows = BufReader::new(File::open(p)?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect::<Result<Vec<Sample>>>()?;
    if rows.len() < 3
        || rows
            .windows(2)
            .any(|r| r[1].tick - r[0].tick != rows[1].tick - rows[0].tick)
    {
        return Err("expected regularly sampled physical states".into());
    }
    Ok(rows)
}
struct Stride {
    swing: usize,
    liftoff: u64,
    touchdown: u64,
    duration: u64,
    clearance: f64,
    min_margin: f64,
    drift: f64,
    advance: f64,
}
fn strides(rows: &[Sample], count: u32, first_fall: Option<u64>) -> Vec<Stride> {
    let sample_period = rows[1].tick - rows[0].tick;
    let mut placements = Vec::new();
    for n in 0..count {
        let t = u64::from(n) * 7000;
        let s = (1 - (n % 2)) as usize;
        let stand = 1 - s;
        let start = rows.partition_point(|r| r.tick < t + 1900);
        let end = rows.partition_point(|r| r.tick <= t + 5500);
        let mut best = 0..0;
        let mut current = 0..0;
        for (i, r) in rows[start..end].iter().enumerate() {
            if r.feet_force_n[s] == 0.
                && r.loaded_sole_points[s] == 0
                && r.foot_bottom_m[s] > 0.001
                && r.feet_force_n[stand] > 100.
                && r.loaded_sole_points[stand] >= 3
                && r.other_force_n < 1.
            {
                if current.is_empty() {
                    current = i..i + 1;
                } else {
                    current.end = i + 1;
                }
                if current.len() > best.len() {
                    best = current.clone();
                }
            } else {
                current = 0..0;
            }
        }
        if best.len() as u64 * sample_period < 500 {
            break;
        }
        let max_slice = &rows[(start + best.start)..(start + best.end)];
        let Some(land) = rows[(start + best.end)..end]
            .iter()
            .find(|r| r.feet_force_n[s] > 1. && r.loaded_sole_points[s] > 0)
        else {
            break;
        };
        if first_fall.is_some_and(|fall| fall <= land.tick) {
            break;
        }
        let takeoff = &max_slice[0];
        let stance_origin = takeoff.foot_centers_m[stand];
        let drift = max_slice
            .iter()
            .map(|r| {
                (r.foot_centers_m[stand][0] - stance_origin[0])
                    .hypot(r.foot_centers_m[stand][1] - stance_origin[1])
            })
            .fold(0., f64::max);
        let clearance = max_slice
            .iter()
            .map(|r| r.foot_bottom_m[s])
            .fold(0., f64::max);
        let margin = max_slice
            .iter()
            .filter_map(|r| r.capture_margin_m)
            .fold(f64::INFINITY, f64::min);
        let advance = land.foot_centers_m[s][0] - rows[start].foot_centers_m[s][0];
        placements.push(Stride {
            swing: s,
            liftoff: takeoff.tick,
            touchdown: land.tick,
            duration: max_slice.len() as u64 * sample_period,
            clearance,
            min_margin: margin,
            drift,
            advance,
        });
    }
    placements
}
fn main() -> Result<()> {
    let arg = std::env::args_os()
        .nth(1)
        .ok_or("supply walk artifact directory")?;
    let root = Path::new(&arg);
    let acquired = root.join("planted/walk-4.json").is_file();
    let mut scenarios = if acquired {
        vec![
            (
                "nominal-32",
                "../multifidelity-humanoid-walk/endurance/walk-32-original",
                32,
            ),
            (
                "measured-32",
                "../multifidelity-humanoid-walk/qualified/track-32",
                32,
            ),
            ("acquired-4", "planted/walk-4", 4),
            ("acquired-16", "planted/walk-16", 16),
            ("acquired-32", "planted/walk-32", 32),
        ]
    } else {
        vec![
            ("nominal-4", "sweep/stride-.095", 4),
            ("nominal-8", "endurance/walk-8", 8),
            ("nominal-16", "endurance/walk-16", 16),
            ("nominal-32", "endurance/walk-32-original", 32),
            ("measured-4", "qualified/track-4", 4),
            ("measured-8", "qualified/track-8", 8),
            ("measured-16", "qualified/track-16", 16),
            ("measured-32", "qualified/track-32", 32),
        ]
    };
    if acquired {
        for (name, path, count) in [
            ("acquired-64", "endurance/walk-64", 64),
            ("acquired-128", "endurance/walk-128", 128),
            ("short-stride-64", "endurance/short-64", 64),
            ("long-stride-64", "endurance/long-64", 64),
            ("roll-bounded-long-64", "long-limit/corrected-walk-64", 64),
        ] {
            if root
                .join(format!("{path}.json"))
                .metadata()
                .is_ok_and(|v| v.len() > 0)
            {
                scenarios.push((name, path, count));
            }
        }
    }
    let mut summary = File::create(root.join("walks.csv"))?;
    writeln!(
        summary,
        "scenario,intended_placements,load_verified_placements,first_fall_ms,last_ms,net_com_x_m,peak_tilt_rad,nonfoot_support_ms,sample_spacing_ms"
    )?;
    let mut out = File::create(root.join("footsteps.csv"))?;
    writeln!(
        out,
        "scenario,step,swing_leg,release_clear_ms,first_loaded_landing_ms,unloaded_ms,clearance_mm,min_capture_margin_mm,max_stance_drift_mm,swing_travel_at_landing_mm"
    )?;
    let mut short = None;
    let mut long = Vec::new();
    for (name, path, steps) in scenarios {
        let rows = trace(&root.join(format!("{path}.jsonl")))?;
        let r: Run = serde_json::from_reader(File::open(root.join(format!("{path}.json")))?)?;
        let owned = strides(&rows, steps, r.first_fall);
        let last = rows.last().unwrap();
        writeln!(
            summary,
            "{name},{steps},{},{},{},{:.6},{:.6},{},{}",
            owned.len(),
            r.first_fall.map(|t| t.to_string()).unwrap_or_default(),
            last.tick,
            last.com[0] - rows[0].com[0],
            r.max_torso_tilt_rad,
            r.nonfoot_support_ticks,
            rows[1].tick - rows[0].tick
        )?;
        for (i, s) in owned.iter().enumerate() {
            writeln!(
                out,
                "{name},{},{},{},{},{},{:.3},{:.3},{:.3},{:.3}",
                i + 1,
                if s.swing == 0 { "left" } else { "right" },
                s.liftoff,
                s.touchdown,
                s.duration,
                s.clearance * 1000.,
                s.min_margin * 1000.,
                s.drift * 1000.,
                s.advance * 1000.
            )?;
        }
        if name == (if acquired { "acquired-4" } else { "measured-4" }) {
            short = Some(rows);
        } else if steps == 32 {
            long.push((name, rows));
        }
    }
    let short = short.unwrap();
    let path = root.join("walking.svg");
    let drawing = SVGBackend::new(&path, (1040, 865)).into_drawing_area();
    drawing.fill(&WHITE)?;
    let panels = drawing.split_evenly((3, 1));
    let blue = RGBColor(39, 90, 151);
    let orange = RGBColor(191, 99, 34);
    let gray = RGBColor(112, 117, 129);
    let charts: [(&str, &str, std::ops::Range<f64>); 2] = [
        (
            "Alternating loaded sole and airborne clearance",
            "Clearance (mm)",
            -8.0..70.,
        ),
        (
            "Actual floor-referenced foot placement and COM",
            "World X (m)",
            -0.03..0.47,
        ),
    ];
    for (i, (title, y, range)) in charts.into_iter().enumerate() {
        let mut c = ChartBuilder::on(&panels[i])
            .margin(13)
            .caption(title, ("sans-serif", 22))
            .x_label_area_size(35)
            .y_label_area_size(66)
            .build_cartesian_2d(0.0..30., range)?;
        c.configure_mesh()
            .x_desc("Simulation time (s)")
            .y_desc(y)
            .light_line_style(RGBColor(230, 232, 236))
            .draw()?;
        for (k, name, color) in [(0, "Left foot", blue), (1, "Right foot", orange)] {
            c.draw_series(LineSeries::new(
                short.iter().map(|r| {
                    (
                        r.tick as f64 * 0.001,
                        if i == 0 {
                            r.foot_bottom_m[k] * 1000.
                        } else {
                            r.foot_centers_m[k][0]
                        },
                    )
                }),
                color.stroke_width(3),
            ))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(3)));
        }
        if i == 1 {
            c.draw_series(LineSeries::new(
                short.iter().map(|r| (r.tick as f64 * 0.001, r.com[0])),
                gray.stroke_width(2),
            ))?
            .label("COM")
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], gray.stroke_width(2)));
        }
        c.configure_series_labels()
            .border_style(RGBColor(205, 208, 214))
            .background_style(WHITE.mix(0.9))
            .position(SeriesLabelPosition::UpperLeft)
            .draw()?;
    }
    let endpoint = long
        .iter()
        .map(|(_, r)| r.last().unwrap().tick)
        .max()
        .unwrap() as f64
        * 0.001;
    let mut c = ChartBuilder::on(&panels[2])
        .margin(13)
        .caption(
            "Endurance control through the full 32-placement request",
            ("sans-serif", 22),
        )
        .x_label_area_size(35)
        .y_label_area_size(66)
        .build_cartesian_2d(0.0..endpoint, -0.05..2.3_f64)?;
    c.configure_mesh()
        .x_desc("Simulation time (s)")
        .y_desc("Torso tilt (rad)")
        .light_line_style(RGBColor(230, 232, 236))
        .draw()?;
    for ((name, rows), color) in long.into_iter().zip([orange, blue, RGBColor(48, 130, 93)]) {
        c.draw_series(LineSeries::new(
            rows.iter()
                .step_by(4)
                .map(|r| (r.tick as f64 * 0.001, r.tilt_rad)),
            color.stroke_width(2),
        ))?
        .label(name)
        .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2)));
    }
    c.configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    drawing.present()?;
    Ok(())
}
