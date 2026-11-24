// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recompute all nearest-rank p95s and verify the native-state digests.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(std::env::args_os().nth(1).ok_or("pass study directory")?);
    let files = [
        "baseline",
        "baseline-repeat",
        "index",
        "index-repeat",
        "vectors",
        "vectors-repeat",
        "final",
        "final-repeat",
        "ordered",
        "ordered-repeat",
    ];
    let data: Vec<Value> = files
        .iter()
        .map(|name| {
            serde_json::from_slice(&fs::read(out.join(format!("{name}.json"))).unwrap()).unwrap()
        })
        .collect();
    let mut checked = 0;
    for k in ["native", "bevy"] {
        let first = data[0][k].as_array().unwrap();
        for run in &data {
            let rows = run[k].as_array().unwrap();
            assert_eq!(first.len(), rows.len());
            for (a, b) in first.iter().zip(rows) {
                for field in [
                    "static_extras",
                    "recycled_handle",
                    "native_calls",
                    "authored_pairs",
                    "native_digest",
                ] {
                    assert_eq!(a[field], b[field]);
                }
                for group in ["tick", "query"] {
                    if b[group].is_null() {
                        continue;
                    }
                    let mut samples: Vec<_> = b[group]["samples_ns"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|v| v.as_u64().unwrap())
                        .collect();
                    assert_eq!(samples.len(), 128);
                    samples.sort();
                    assert_eq!(
                        samples[(samples.len() * 95).div_ceil(100) - 1],
                        b[group]["p95_ns"].as_u64().unwrap()
                    );
                    checked += samples.len();
                }
            }
        }
    }
    let path = out.join("cost.svg");
    let root = SVGBackend::new(&path, (1120, 470)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root.split_evenly((1, 2));
    for (index, area) in panels.iter().enumerate() {
        let title = if index == 0 {
            "Eight native solves with one contact"
        } else {
            "Settled Bevy world with authored pairs"
        };
        let mut chart = ChartBuilder::on(area)
            .margin(22)
            .caption(title, ("sans-serif", 19))
            .x_label_area_size(53)
            .y_label_area_size(60)
            .build_cartesian_2d(
                -0.45f64..3.45,
                0.0f64..if index == 0 { 145.0 } else { 109.0 },
            )?;
        let counts = [0, 32, 256, 1024];
        chart
            .configure_mesh()
            .disable_x_mesh()
            .x_desc("Additional static boxes")
            .y_desc("p95 step time (µs)")
            .x_labels(4)
            .x_label_formatter(&|v| {
                let i = v.round();
                if (v - i).abs() < 0.01 && (0.0..=3.0).contains(&i) {
                    counts[i as usize].to_string()
                } else {
                    String::new()
                }
            })
            .draw()?;
        let get = |r: usize, n: usize| {
            let src = if index == 0 { "native" } else { "bevy" };
            let y = data[r][src]
                .as_array()
                .unwrap()
                .iter()
                .find(|row| {
                    row["static_extras"] == n
                        && if index == 0 {
                            row["recycled_handle"] == false && row["native_calls"] == 8
                        } else {
                            row["authored_pairs"] == n.min(128) + 1
                        }
                })
                .unwrap();
            y["tick"]["p95_ns"].as_u64().unwrap() as f64 / 1000.
        };
        for (r, color, label, dx) in [
            (0, RGBColor(177, 94, 27), "Before", -0.16),
            (6, RGBColor(26, 109, 137), "After", 0.16),
        ] {
            let values: Vec<_> = counts
                .iter()
                .enumerate()
                .map(|(i, n)| (i as f64 + dx, get(r, *n), get(r + 1, *n)))
                .collect();
            chart
                .draw_series(values.iter().map(|(x, a, b)| {
                    Rectangle::new([(*x - 0.14, 0.), (*x + 0.14, (a + b) / 2.)], color.filled())
                }))?
                .label(label)
                .legend(move |(x, y)| {
                    Rectangle::new([(x, y - 4), (x + 14, y + 4)], color.filled())
                });
            chart.draw_series(
                values
                    .iter()
                    .map(|(x, a, b)| PathElement::new([(*x, *a), (*x, *b)], BLACK.stroke_width(2))),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.92))
            .draw()?;
    }
    root.present()?;
    println!(
        "{}",
        json!({"timings_recomputed":checked,"native_layouts":16,"bevy_layouts":11,"files":files.len(),"all_physical_digests_match":true})
    );
    Ok(())
}
