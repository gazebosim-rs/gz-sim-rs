// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate and summarize observed operator-directed cold restores and graph their timings.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const CASES: [(&str, &str); 3] = [
    ("paused", "Paused · pipes"),
    ("advancing", "In flight · pipes"),
    ("migrated", "Migrated · SSH"),
];
const METRICS: [(&str, &str); 4] = [
    ("parked_ms", "Start to parked scene"),
    ("admission_ms", "Request to admission"),
    (
        "verified_install_from_request_ms",
        "Request to verified paused scene",
    ),
    ("checked_step_ms", "Checked step to acknowledgement"),
];
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn value(row: &Value, field: &str) -> Result<f64> {
    row[field]
        .as_f64()
        .filter(|v| v.is_finite() && *v >= 0.)
        .ok_or_else(|| format!("missing finite elapsed time {field}").into())
}
fn event<'a>(events: &'a [Value], label: &str) -> Result<&'a Value> {
    events
        .iter()
        .find(|e| e["event"] == label)
        .ok_or_else(|| format!("missing event {label}").into())
}
fn median(samples: &[f64]) -> f64 {
    let mut v = samples.to_vec();
    v.sort_by(f64::total_cmp);
    (v[(v.len() - 1) / 2] + v[v.len() / 2]) / 2.
}
fn run(dir: &Path) -> Result<()> {
    let mut groups = Vec::new();
    let mut traces: Vec<Vec<Vec<f64>>> = Vec::new();
    let mut verified = 0;
    for (case, _) in CASES {
        let p = dir.join(case);
        let mut paths = fs::read_dir(&p)?
            .map(|e| e.map(|v| v.path()))
            .collect::<std::io::Result<Vec<_>>>()?;
        paths.retain(|p| p.is_dir());
        paths.sort();
        let mut rows = Vec::new();
        let mut values: Vec<Vec<f64>> = vec![vec![]; METRICS.len()];
        for path in paths {
            let mut row = read(&path.join("result.json"))?;
            let oracle = read(&path.join("oracle.json"))?;
            let data = read(&path.join("events.json"))?;
            let events = data.as_array().ok_or("invalid events")?;
            if events
                .windows(2)
                .any(|w| w[0]["elapsed_ms"].as_f64() > w[1]["elapsed_ms"].as_f64())
            {
                return Err("event clock went backwards".into());
            }
            event(events, "supervisor_sigkill")?;
            let admitted = event(events, "admitted")?;
            let (start, parked) = events
                .windows(2)
                .rfind(|w| w[0]["event"] == "cold_start" && w[1]["event"] == "parked")
                .map(|v| (&v[0], &v[1]))
                .ok_or("missing cold start")?;
            let proof = &event(events, "verified_restored")?["detail"]["proof"];
            let ack = &event(events, "checked_step")?["detail"];
            let duplicate = &event(events, "replayed_step_denied")?["detail"];
            if admitted["detail"]["accepted"] != true
                || parked["detail"]["offer"]["token"] != proof["token"]
                || parked["detail"]["worker_processes"] != 0
                || row["source_tick"] != proof["tick"]
                || ack["ok"] != true
                || duplicate["ok"] != false
                || row["restored_pose_matches"] != 4
                || row["successor_pose_matches"] != 4
                || row["successor_sha256"] != oracle["samples"][1]["sha256"]
                || row["source_sha256"] != oracle["samples"][0]["sha256"]
            {
                return Err(format!("invalid restore evidence at {}", path.display()).into());
            }
            match case {
                "advancing" => {
                    let rpc = &event(events, "interrupted_worker_rpc")?["detail"];
                    if rpc["unread_pipe_bytes"].as_u64().is_none_or(|n| n == 0)
                        || rpc["last_archive_sha256"] != row["source_sha256"]
                        || event(events, "old_package_denied")?["detail"]["accepted"] != false
                    {
                        return Err("incomplete in-flight fault".into());
                    }
                }
                "paused" => {
                    if row["durable_pending_nm"] != 0.015
                        || event(events, "old_offer_denied")?["detail"]["accepted"] != false
                        || event(events, "admitted_old_offer_denied")?["detail"]["accepted"]
                            != false
                    {
                        return Err("incomplete old-offer refusal".into());
                    }
                }
                _ => {
                    if row["transfer"]["before_owners"] != json!([0, 1, 1])
                        || row["transfer"]["after_owners"] != json!([1, 0, 0])
                        || row["source_impulse_ns"].as_f64().is_none_or(|x| x < 0.49)
                    {
                        return Err("missing loaded SSH migration".into());
                    }
                }
            }
            row["parked_ms"] = (value(parked, "elapsed_ms")? - value(start, "elapsed_ms")?).into();
            for (k, (name, _)) in METRICS.iter().enumerate() {
                values[k].push(value(&row, name)?);
            }
            rows.push(row);
            verified += 1;
        }
        if rows.is_empty() {
            return Err(format!("missing {case} trials").into());
        }
        let summary = METRICS
            .iter()
            .enumerate()
            .map(|(i, (name, _))| ((*name).to_owned(), json!(median(&values[i]))))
            .collect::<serde_json::Map<_, _>>();
        groups.push(json!({"case":case,"trials":rows.len(),"medians_ms":summary,"observed":rows}));
        traces.push(values);
    }
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(
            &json!({"trials":verified,"groups":groups,"measurement":"Wall-clock elapsed time on one workstation. Each group has one observation per fault trial. Install includes admission; parked timing excludes any previous failed selection."}),
        )?,
    )?;
    let path = dir.join("timings.svg");
    let root = SVGBackend::new(&path, (1220, 690)).into_drawing_area();
    root.fill(&WHITE)?;
    let parts = root.split_vertically(110);
    parts.0.draw(&Text::new(
        "Recovery after supervisor loss",
        (32, 43),
        ("sans-serif", 31)
            .into_font()
            .style(FontStyle::Bold)
            .color(&RGBColor(18, 46, 59)),
    ))?;
    parts.0.draw(&Text::new("A dot marks each observed trial. The dark bar marks the group median. Axes are scaled independently.",(33,78),("sans-serif",18).into_font().color(&RGBColor(53,75,87))))?;
    let grid = parts.1.margin(0, 12, 12, 12).split_evenly((2, 2));
    let colors = [
        RGBColor(42, 123, 169),
        RGBColor(32, 137, 119),
        RGBColor(155, 93, 142),
    ];
    for (i, area) in grid.iter().enumerate() {
        let upper = traces
            .iter()
            .flat_map(|g| &g[i])
            .copied()
            .fold(0f64, f64::max)
            * 1.18;
        let mut chart = ChartBuilder::on(area)
            .caption(
                METRICS[i].1,
                ("sans-serif", 21).into_font().color(&RGBColor(22, 52, 65)),
            )
            .margin(14)
            .x_label_area_size(49)
            .y_label_area_size(135)
            .build_cartesian_2d(0.0..upper, -0.55..2.55)?;
        chart
            .configure_mesh()
            .y_labels(3)
            .y_label_formatter(&|v: &f64| {
                let row = v.round() as i32;
                if (*v - f64::from(row)).abs() < 1e-8 && (0..=2).contains(&row) {
                    CASES[2 - row as usize].1.to_string()
                } else {
                    String::new()
                }
            })
            .x_desc("Elapsed wall time (ms)")
            .x_labels(6)
            .light_line_style(RGBColor(232, 238, 240))
            .label_style(("sans-serif", 14))
            .axis_desc_style(("sans-serif", 15))
            .draw()?;
        for (j, g) in traces.iter().enumerate() {
            let yy = (2 - j) as f64;
            for (rep, x) in g[i].iter().enumerate() {
                let d = (rep as f64 - (g[i].len() as f64 - 1.) / 2.) * 0.055;
                chart.draw_series(std::iter::once(Circle::new(
                    (*x, yy + d),
                    4,
                    colors[j].filled(),
                )))?;
            }
            let m = median(&g[i]);
            chart.draw_series(std::iter::once(PathElement::new(
                [(m, yy - 0.18), (m, yy + 0.18)],
                RGBColor(23, 44, 54).stroke_width(2),
            )))?;
        }
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let dir = std::env::args_os()
        .nth(1)
        .ok_or("usage: convoy_supervisor_report QUALIFICATION_DIR")?;
    run(Path::new(&dir))
}
