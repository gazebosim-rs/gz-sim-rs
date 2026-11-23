// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare wall / caller CPU on matched synchronous zone decisions.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    path::{Path, PathBuf},
};
fn read(d: &Path, name: &str) -> Vec<Value> {
    serde_json::from_slice(&fs::read(d.join(name).join("tiers.json")).unwrap()).unwrap()
}
fn u(v: &Value) -> u64 {
    v.as_u64().unwrap()
}
fn q(v: &[f64], p: usize) -> f64 {
    let mut x = v.to_vec();
    x.sort_by(f64::total_cmp);
    x[(x.len() * p).div_ceil(100) - 1]
}
fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply parked-batch phase directory")?,
    );
    let base = read(&out, "before");
    let mut result = vec![];
    let mut workloads = 0usize;
    let mut peak = 0f64;
    let mut chart: BTreeMap<String, Vec<(f64, f64)>> = BTreeMap::new();
    for name in ["before", "before-repeat", "after", "after-repeat"] {
        let rows = read(&out, name);
        assert_eq!(rows.len(), 40);
        for r in &rows {
            let matched = base
                .iter()
                .find(|b| {
                    ["rep", "stations", "switched"]
                        .iter()
                        .all(|k| b[*k] == r[*k])
                })
                .unwrap();
            assert_eq!(matched["wakes"], r["wakes"]);
            for (a, b) in matched["samples"]
                .as_array()
                .unwrap()
                .iter()
                .zip(r["samples"].as_array().unwrap())
            {
                for field in ["tick", "phase", "held", "wake", "registered", "total_j_ns"] {
                    assert_eq!(a[field], b[field]);
                }
                workloads += 1;
            }
        }
        for n in [1, 4, 16, 48] {
            for label in ["dwell", "demote", "wake", "parked", "dynamic"] {
                let mut reps: [Vec<f64>; 4] = Default::default();
                let mut absolute_ratio = vec![];
                let mut samples_per = 0;
                for rep in 0..5 {
                    let sample = |sw| {
                        rows.iter()
                            .find(|v| {
                                u(&v["stations"]) == n && u(&v["rep"]) == rep && v["switched"] == sw
                            })
                            .unwrap()["samples"]
                            .as_array()
                            .unwrap()
                    };
                    let selected = sample(true);
                    let control = sample(false);
                    let demote: Vec<_> = selected
                        .iter()
                        .filter(|s| s["phase"] == "demote")
                        .map(|s| u(&s["tick"]))
                        .collect();
                    assert_eq!(demote.len(), 2);
                    let indices: Vec<_> = selected
                        .iter()
                        .enumerate()
                        .filter(|(_, s)| {
                            if label == "dwell" {
                                let t = u(&s["tick"]);
                                demote.iter().any(|end| t < *end && t + 4 >= *end)
                            } else {
                                s["phase"] == label
                            }
                        })
                        .map(|(i, _)| i)
                        .collect();
                    samples_per = indices.len();
                    assert!(samples_per > 0);
                    for (i, k) in ["thread_cpu_ns", "wall_ns"].into_iter().enumerate() {
                        let micros: Vec<_> = indices
                            .iter()
                            .map(|j| u(&selected[*j][k]) as f64 / 1e3)
                            .collect();
                        for (p, slot) in [(50, 0), (95, 1)] {
                            reps[2 * i + slot].push(q(&micros, p));
                        }
                    }
                    let cpu: Vec<_> = indices
                        .iter()
                        .map(|j| u(&control[*j]["thread_cpu_ns"]) as f64 / 1e3)
                        .collect();
                    absolute_ratio.push(reps[1].last().unwrap() / q(&cpu, 95));
                }
                let m = reps.map(|v| q(&v, 50));
                peak = peak.max(m[1]);
                if label == "dwell" {
                    chart
                        .entry(name.to_owned())
                        .or_default()
                        .push((n as f64, m[1] / 1e3));
                }
                result.push(json!({"run":name,"stations":n,"phase":label,"per_run_samples":samples_per,"median_cpu_p50_p95_wall_p50_p95_us":m,"p95_cpu_relative_to_always_dynamic_on_identical_ticks":q(&absolute_ratio,50)}));
            }
        }
    }
    let file = out.join("dwell-cost.svg");
    let page = SVGBackend::new(&file, (950, 480)).into_drawing_area();
    page.fill(&WHITE)?;
    let mut plot = ChartBuilder::on(&page)
        .margin(18)
        .caption(
            "Validating stationary feet before a synchronized tier switch",
            ("sans-serif", 22),
        )
        .x_label_area_size(50)
        .y_label_area_size(70)
        .build_cartesian_2d(0f64..50f64, 0f64..(1.08 * peak / 1e3))?;
    plot.configure_mesh()
        .x_desc("Two-pad assemblies in one Bevy world")
        .y_desc("p95 caller CPU time per dwell tick (ms)")
        .draw()?;
    for (tag, label, col) in [
        ("before", "Separate proof per foot", RGBColor(173, 77, 44)),
        ("after", "One joint proof", RGBColor(28, 119, 99)),
    ] {
        let data = &chart[tag];
        let repeat = &chart[&format!("{tag}-repeat")];
        let points: Vec<_> = data
            .iter()
            .zip(repeat)
            .map(|(a, b)| (a.0, (a.1 + b.1) / 2.))
            .collect();
        plot.draw_series(LineSeries::new(points.iter().copied(), col.stroke_width(2)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 25, y)], col.stroke_width(2)));
        for (a, b) in data.iter().zip(repeat) {
            plot.draw_series([PathElement::new(vec![*a, *b], col.stroke_width(1))])?;
        }
        plot.draw_series(points.iter().map(|p| Circle::new(*p, 4, col.filled())))?;
    }
    plot.configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .position(SeriesLabelPosition::UpperLeft)
        .draw()?;
    page.present()?;
    fs::write(
        out.join("checks/readback.json"),
        serde_json::to_vec_pretty(
            &json!({"matched_public_tick_rows":workloads,"counterbalanced_runs":result}),
        )?,
    )?;
    Ok(())
}
