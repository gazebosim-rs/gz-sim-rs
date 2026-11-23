// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently reconstruct the adjacent finest-grid errors on every public tick of a nominal-drive replay.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn rows(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "expected array".into())
}
fn num(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "nonfinite value".into())
}
fn sequence(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing tick".into())
}
fn vec(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(v.clone())?))
}
fn close(a: f64, b: f64) -> Result<()> {
    if (a - b).abs() > 2e-11 * a.abs().max(b.abs()).max(1.) {
        return Err(format!("mismatched values {a} and {b}").into());
    }
    Ok(())
}
struct State {
    tick: u64,
    com: DVec3,
    spin: DVec3,
    normal: f64,
    tangent: DVec3,
    first: Option<u64>,
}
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 2 {
        return Err("three_stroke_grids DEEP_CONVERGENCE_ARCHIVE CASE".into());
    }
    let dir = Path::new(&args[0]);
    let k: usize = args[1].parse()?;
    let file = format!("case-{k:03}.json");
    let raw = read(&dir.join("trajectories").join(&file))?;
    let saved = read(&dir.join("cases").join(&file))?;
    if raw["case"] != k || saved["case"] != k || raw["source"] != saved["source"] {
        return Err("wrong physical coordinate".into());
    }
    let result = &saved["result"];
    let cut = sequence(&result["cut_tick"])?;
    let end = sequence(&result["end_tick"])?;
    let out = dir.join(format!("case-{k:03}"));
    let labels = ["8192", "16384", "32768"];
    let (mut all, mut summaries, mut checks) = (vec![], vec![], 0usize);
    for tag in labels {
        let lane = |v: &Value| -> Result<Value> {
            Ok(rows(&v["paths"])?
                .iter()
                .find(|p| p["profile"] == tag)
                .ok_or("no grid")?
                .clone())
        };
        let p = lane(&raw)?;
        let mut path = vec![];
        let summary = lane(result)?;
        for (i, r) in rows(&p["ticks"])?.iter().enumerate() {
            let t = sequence(&r[0])?;
            if t != cut + i as u64 + 1 {
                return Err("nonconsecutive public clock".into());
            }
            path.push(State {
                tick: t,
                com: vec(&r[1])?,
                spin: vec(&r[2])?,
                normal: num(&r[3])?,
                tangent: vec(&r[4])?,
                first: r[5].as_u64(),
            });
        }
        if path.last().map(|x| x.tick) != Some(end) {
            return Err("truncated numerical grid".into());
        }
        let frames = read(&out.join("traces").join(format!("{tag}.json")))?;
        for f in rows(&frames["frames"])?.iter().skip(1) {
            let t = sequence(&f["tick"])?;
            let s = &path[(t - cut - 1) as usize];
            close(
                s.com
                    .distance((vec(&f["foot"]["position"])? + vec(&f["upper"]["position"])?) * 0.5),
                0.,
            )?;
            close(s.spin.distance(vec(&f["foot"]["angular_velocity"])?), 0.)?;
            close(s.normal, num(&f["normal_impulse_ns"])?)?;
            close(s.tangent.distance(vec(&f["tangent_on_foot_ns"])?), 0.)?;
            if s.first != f["first_local_ns"].as_u64() || f["native_steps"] != tag.parse::<u64>()? {
                return Err("receipt timing or budget mismatch".into());
            }
            checks += 1;
        }
        all.push(path);
        summaries.push(summary);
    }
    let mut trajectories = vec![];
    let mut comparison = vec![];
    for (i, j) in [(0, 1), (1, 2), (0, 2)] {
        let points = all[i]
            .iter()
            .zip(&all[j])
            .map(|(a, b)| {
                [
                    (a.tick - cut) as f64 * 0.5,
                    1000. * a.com.distance(b.com),
                    a.spin.distance(b.spin),
                    (a.normal - b.normal).abs(),
                ]
            })
            .collect::<Vec<_>>();
        let n = points.len();
        let rms = (points.iter().map(|p| p[1] * p[1]).sum::<f64>() / n as f64).sqrt();
        let max = points.iter().map(|p| p[1]).fold(0f64, f64::max);
        let final_com = points[n - 1][1];
        let expected = &summaries[i]["against"][j];
        close(rms, 1000. * num(&expected["rms_com_m"])?)?;
        close(max, 1000. * num(&expected["max_com_m"])?)?;
        close(final_com, 1000. * num(&expected["final_com_m"])?)?;
        let first = [0.001, 0.1, 1., 10.].map(|threshold| {
            points
                .iter()
                .position(|s| s[1] > threshold)
                .map(|p| all[i][p].tick)
        });
        if expected["first_above_1um_100um_1mm_10mm_ticks"] != json!(first) {
            return Err("threshold crossing mismatch".into());
        }
        let peak_tick = sequence(&expected["max_tick"])?;
        let peak_time = (peak_tick - cut) as f64 * 0.5;
        comparison.push(json!({"first_grid":labels[i],"second_grid":labels[j],"end_mm":final_com,"rms_mm":rms,"peak_mm":max,"peak_time_ms":peak_time,"first_above_1um_100um_1mm_10mm_ticks":first}));
        trajectories.push(points);
    }
    fs::write(
        out.join("grid-trajectory.json"),
        serde_json::to_vec_pretty(
            &json!({"source":raw["source"],"case":k,"cut_tick":cut,"clock_end":end,"public_points_per_grid":end-cut,"sparse_native_samples_checked":checks,"grid_comparisons":comparison}),
        )?,
    )?;
    let p = out.join("grid-trajectory.svg");
    let root = SVGBackend::new(&p, (1160, 855)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "Native grid sensitivity across three returns",
        (26, 36),
        ("sans-serif", 24).into_font(),
    ))?;
    root.draw(&Text::new(
        format!(
            "Nominal drive • case {k} • {} matched public updates • 0.5 ms per public update",
            end - cut
        ),
        (26, 62),
        ("sans-serif", 15).into_font(),
    ))?;
    let panes = root.margin(82, 25, 10, 10).split_evenly((3, 1));
    let t = (end - cut) as f64 * 0.5;
    for (m, axis) in [
        "COM distance (mm)",
        "Spin distance (rad/s)",
        "Pair impulse distance (N·s)",
    ]
    .into_iter()
    .enumerate()
    {
        let y = m + 1;
        let scale = trajectories[..2]
            .iter()
            .flat_map(|p| p.iter().map(|v| v[y]))
            .fold(0.001f64, f64::max);
        let mut chart = ChartBuilder::on(&panes[m])
            .margin(5)
            .x_label_area_size(31)
            .y_label_area_size(76)
            .build_cartesian_2d(0.0..t, 0.0..scale * 1.08)?;
        chart
            .configure_mesh()
            .x_desc("Time from the common released state (ms)")
            .y_desc(axis)
            .light_line_style(WHITE.mix(0.1))
            .draw()?;
        for (i, color) in [RGBColor(25, 124, 101), RGBColor(178, 83, 34)]
            .into_iter()
            .enumerate()
        {
            let line = trajectories[i].iter().map(|p| (p[0], p[y]));
            chart
                .draw_series(LineSeries::new(line, color.stroke_width(2)))?
                .label(format!("{} vs {}", labels[i], labels[i + 1]))
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 18, y)], color.stroke_width(3))
                });
        }
        if m == 0 {
            chart
                .configure_series_labels()
                .background_style(WHITE.mix(0.9))
                .border_style(BLACK.mix(0.25))
                .draw()?;
        }
    }
    root.present()?;
    Ok(())
}
