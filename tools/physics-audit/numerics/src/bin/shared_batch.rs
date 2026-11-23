// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
fn read(d: &Path, f: &str) -> Value {
    serde_json::from_slice(&fs::read(d.join(f)).unwrap()).unwrap()
}
fn u(v: &Value) -> u64 {
    v.as_u64().unwrap()
}
fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn q(mut v: Vec<f64>, p: usize) -> f64 {
    v.sort_by(f64::total_cmp);
    v[(v.len() * p).div_ceil(100) - 1]
}
fn field(dir: &Path) -> Value {
    let data = read(dir, "field.json");
    let ticks = data["ticks"].as_array().unwrap();
    assert_eq!(ticks.len(), 300);
    let mut max = 0f64;
    let mut feet = 0;
    let mut native = 0;
    for t in ticks {
        native += u(&t["native_steps"]);
        for j in t["support_ns"].as_array().unwrap() {
            let [a, b]: [f64; 2] = serde_json::from_value(j.clone()).unwrap();
            assert!(a > 0. && b > 0.);
            max = max.max((a + b - 0.00981).abs());
            feet += 1;
        }
    }
    let expected = [0.01821381976, 0.04349999975, 0.08700000025, 0.12312957];
    let mut time = 0f64;
    for a in data["arrests"].as_array().unwrap() {
        time = time.max((f(&a["event_s"]) - expected[u(&a["station"]) as usize]).abs());
    }
    assert_eq!((native, feet), (300, 1200));
    assert!(max < 1e-13 && time < 1e-12);
    assert!(
        data["matched_errors_position_velocity_spin_quaternion_each_impulse"]
            .as_array()
            .unwrap()
            .iter()
            .all(|a| f(a) == 0.)
    );
    let controls = read(dir, "controls.json");
    for c in &controls.as_array().unwrap()[..2] {
        assert!(
            c["native_differences"].as_array().unwrap().iter().all(|v| v
                .as_array()
                .unwrap()
                .iter()
                .all(|v| f(v) == 0.))
        );
    }
    json!({"foot_ticks":feet,"whole_world_native_steps":native,"arrest_times_s":expected,"max_fractional_clock_error_s":time,"max_tick_net_reaction_error_ns":max,"physical_position_velocity_weight_errors":data["physical_errors_position_velocity_weight"],"isolated_world_differences":data["matched_errors_position_velocity_spin_quaternion_each_impulse"],"fallback_and_tier_controls":controls})
}
fn timings(dir: &Path) -> Value {
    let rows = read(dir, "cost.json");
    let rows = rows.as_array().unwrap();
    let mut out = vec![];
    for n in [1, 4, 16, 48] {
        let sample = rows
            .iter()
            .filter(|r| u(&r["stations"]) == n)
            .collect::<Vec<_>>();
        if sample.is_empty() {
            continue;
        }
        for mode in ["batch", "isolated", "ordinary"] {
            let joined = sample
                .iter()
                .filter(|r| r["mode"] == mode)
                .collect::<Vec<_>>();
            assert_eq!(joined.len(), 9);
            let stats = |key: &str| {
                if joined[0][key].is_null() {
                    return Value::Null;
                }
                let pair = [50, 95].map(|p| {
                    q(
                        joined
                            .iter()
                            .map(|r| {
                                q(
                                    r[key]
                                        .as_array()
                                        .unwrap()
                                        .iter()
                                        .map(|v| f(v) / 1000.)
                                        .collect(),
                                    p,
                                )
                            })
                            .collect(),
                        50,
                    )
                });
                json!(pair)
            };
            let ownership = if mode == "ordinary" { 0 } else { 100 * n };
            for sample in &joined {
                assert_eq!(u(&sample["registered_foot_ticks"]), ownership);
            }
            out.push(json!({"stations":n,"mode":mode,"median_repetition_wall_p50_p95_us":stats("wall_ns"),"median_repetition_cpu_p50_p95_us":stats("thread_cpu_ns")}));
        }
    }
    json!(out)
}
fn plot(dir: &Path, runs: &serde_json::Map<String, Value>) -> Result<(), Box<dyn Error>> {
    let file = dir.join("batch-cost.svg");
    let page = SVGBackend::new(&file, (950, 500)).into_drawing_area();
    page.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&page)
        .margin(20)
        .caption(
            "Measured batch cost across disjoint support groups",
            ("sans-serif", 24),
        )
        .x_label_area_size(55)
        .y_label_area_size(78)
        .build_cartesian_2d(0f64..50., 0.4f64..1.8)?;
    chart
        .configure_mesh()
        .x_desc("Local two-pad stations in one world")
        .y_desc("p95 caller CPU / total for isolated worlds")
        .draw()?;
    chart.draw_series([PathElement::new(vec![(0., 1.), (50., 1.)], BLACK.mix(0.45))])?;
    for (keys, label, col) in [
        (
            &["before", "before-repeat"][..],
            "Full shape scan",
            RGBColor(160, 67, 43),
        ),
        (
            &["after", "after-repeat", "index-final"][..],
            "Swept AABB index",
            RGBColor(56, 110, 164),
        ),
        (
            &["commit", "commit-repeat"][..],
            "Index + shared collider update",
            RGBColor(31, 135, 96),
        ),
    ] {
        let mut mean = vec![];
        for n in [1, 4, 16, 48] {
            let ratios: Vec<_> = keys
                .iter()
                .filter_map(|key| {
                    let v = runs.get(*key)?.as_array()?;
                    let p95 = |mode: &str| {
                        v.iter()
                            .find(|x| x["stations"] == n && x["mode"] == mode)
                            .and_then(|x| x["median_repetition_cpu_p50_p95_us"][1].as_f64())
                    };
                    Some(p95("batch")? / p95("isolated")?)
                })
                .collect();
            if ratios.is_empty() {
                continue;
            }
            for y in &ratios {
                chart.draw_series([Circle::new((n as f64, *y), 4, col.filled())])?;
            }
            mean.push((n as f64, ratios.iter().sum::<f64>() / ratios.len() as f64));
        }
        chart
            .draw_series(LineSeries::new(mean, col.stroke_width(2)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 24, y)], col.stroke_width(2)));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.90))
        .draw()?;
    page.present()?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let data = if dir.join("field.json").exists() {
        json!({"native":field(&dir),"complete_Bevy_public_tick_cost":timings(&dir)})
    } else {
        let mut runs = serde_json::Map::new();
        let baseline = read(&dir.join("before"), "field.json");
        for key in [
            "before",
            "before-repeat",
            "after",
            "after-repeat",
            "commit",
            "commit-repeat",
            "index-final",
        ] {
            let p = dir.join(key);
            if !p.exists() {
                continue;
            }
            let row = read(&p, "field.json");
            for key in [
                "ticks",
                "arrests",
                "physical_errors_position_velocity_weight",
                "matched_errors_position_velocity_spin_quaternion_each_impulse",
            ] {
                assert_eq!(baseline[key], row[key], "{key}");
            }
            runs.insert(key.to_owned(), timings(&p));
        }
        plot(&dir, &runs)?;
        json!({"native":field(&dir.join("commit")),"runs":runs})
    };
    fs::write(
        dir.join("checks/readback.json"),
        serde_json::to_vec_pretty(&data)?,
    )?;
    Ok(())
}
