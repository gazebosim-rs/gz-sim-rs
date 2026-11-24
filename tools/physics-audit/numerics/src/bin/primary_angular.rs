// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently reconstruct angular balances and both closed-form impacts from saved data.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
type V = [f64; 3];
fn vec3(x: &Value) -> V {
    std::array::from_fn(|i| x[i].as_f64().unwrap())
}
fn add(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] + b[i])
}
fn sub(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] - b[i])
}
fn norm(a: V) -> f64 {
    a.into_iter().map(|x| x * x).sum::<f64>().sqrt()
}
fn num(x: &Value) -> f64 {
    x.as_f64().unwrap()
}
fn load(d: &std::path::Path) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&fs::read(d)?)?)
}
fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(std::env::args_os().nth(1).ok_or("pass phase root")?);
    let data = load(&dir.join("angular.json"))?;
    let mixed = data["mixed"].as_array().unwrap();
    let controls = data["closed_form"].as_array().unwrap();
    assert_eq!((mixed.len(), controls.len()), (12, 12));
    let mut max_angular = 0.0f64;
    let mut max_linear = 0.0f64;
    let mut max_native_angular = 0.0f64;
    let mut max_native_linear = 0.0f64;
    let mut normal_scalar = 0.0f64;
    for r in mixed {
        let family = mixed
            .iter()
            .filter(|m| r["mu"] == m["mu"])
            .collect::<Vec<_>>();
        assert_eq!(family.len(), 4);
        assert!(family.iter().all(|m| m["initial"] == r["initial"]));
        assert_eq!(r["first_loaded"].as_array().unwrap().len(), 2);
        normal_scalar = normal_scalar.max(num(&r["max_scalar_point_sum_disagreement_ns"]));
        for ev in r["first_loaded"].as_array().unwrap() {
            assert_eq!(ev["tick"], 1531);
            let samples = ev["native"].as_array().unwrap();
            for field in [
                "normal_linear_ns",
                "tangent_linear_ns",
                "normal_angular_nms",
                "tangent_angular_nms",
            ] {
                let sum = samples
                    .iter()
                    .fold([0.; 3], |s, n| add(s, vec3(&n["delivered"][field])));
                assert!(norm(sub(sum, vec3(&ev["delivered"][field]))) < 1e-14);
            }
        }
        for check in r["checkpoints"].as_array().unwrap() {
            let s = &check["sample"];
            let j = &s["integrated"];
            let r_h = sub(
                vec3(&s["delta_h_nms"]),
                add(
                    vec3(&j["normal_angular_nms"]),
                    vec3(&j["tangent_angular_nms"]),
                ),
            );
            let initial = &r["initial"][check["rig"].as_u64().unwrap() as usize];
            let p0 = add(vec3(&initial[0]["velocity"]), vec3(&initial[1]["velocity"]));
            let r_p = sub(
                sub(vec3(&s["p_ns"]), p0),
                add(
                    vec3(&s["external_gravity_ns"]),
                    add(vec3(&j["normal_linear_ns"]), vec3(&j["tangent_linear_ns"])),
                ),
            );
            assert!(norm(sub(r_h, vec3(&s["angular_residual_nms"]))) < 1e-13);
            assert!(norm(sub(r_p, vec3(&s["linear_residual_ns"]))) < 1e-13);
            max_angular = max_angular.max(norm(r_h));
            max_linear = max_linear.max(norm(r_p));
            max_native_angular = max_native_angular.max(num(&s["max_native_angular_residual_nms"]));
            max_native_linear = max_native_linear.max(num(&s["max_native_linear_residual_ns"]));
        }
    }
    assert!(
        max_angular < 1e-12
            && max_native_angular < 1e-12
            && max_native_linear < 4e-5
            && max_linear < 5e-5
            && normal_scalar < 1e-12
    );
    let mut physical = vec![];
    for r in controls {
        let tilted = r["tilted"].as_bool().unwrap();
        // Active point, world frame: Ry(-0.16) Rx(0.14) times (-.18,-.1,-.13).
        let (sr, cr) = 0.14f64.sin_cos();
        let (sp, cp) = (-0.16f64).sin_cos();
        let rx = cp * (-0.18) + sp * (-0.1 * sr - 0.13 * cr);
        let ry = -0.1 * cr + 0.13 * sr;
        let j = if tilted {
            1.4 / (0.5 + (rx * rx + ry * ry) / 0.05)
        } else {
            2.8
        };
        let h = if tilted {
            [ry * j, -rx * j, 0.]
        } else {
            [0.; 3]
        };
        assert!((num(&r["expected_normal_ns"]) - j).abs() < 1e-13);
        assert!(norm(sub(vec3(&r["expected_angular_nms"]), h)) < 1e-13);
        assert!(
            norm(sub(
                vec3(&r["signed_point_moment_nms"]),
                vec3(&r["actual_angular_nms"])
            )) < 1e-12
        );
        if tilted {
            assert_eq!(r["positive_quadrants"], 1);
            assert!(num(&r["other_corner_gap_m"]) > 0.027);
        }
        if r["policy"] == "relax32" && (!tilted || r["rigid"] == true) {
            assert!(num(&r["moment_error_nms"]) < 1e-12);
        }
        physical.push(json!({"tilted":tilted,"single_rigid":r["rigid"],"policy":r["policy"],"impulse_error_ns":r["normal_error_ns"],"moment_error_nms":r["moment_error_nms"]}));
    }
    let policies = ["ordinary", "relax8", "relax32", "refresh8"];
    let names = ["Ordinary", "8 passes", "32 passes", "8 + fresh"];
    let colors = [
        RGBColor(172, 76, 34),
        RGBColor(33, 117, 83),
        RGBColor(86, 80, 149),
        RGBColor(22, 124, 165),
    ];
    let times = 1473..=2300;
    let series: Vec<Vec<Value>> = policies
        .iter()
        .map(|p| {
            load(&dir.join(format!("ledgers/{p}.json"))).map(|v| v.as_array().unwrap().clone())
        })
        .collect::<Result<_, _>>()?;
    let mut first = None;
    for (i, t) in times.enumerate() {
        assert!(series.iter().all(|s| s.len() == 828 && s[i]["tick"] == t));
        if series[1][i]["after"] != series[0][i]["after"] && first.is_none() {
            first = Some(t);
        }
    }
    assert_eq!(first, Some(1531));
    let svg = dir.join("momentum.svg");
    let root = SVGBackend::new(&svg, (1000, 780));
    let drawing = root.into_drawing_area();
    drawing.fill(&WHITE)?;
    let panels = drawing.split_evenly((2, 1));
    for (i, (area, title, ylabel, range)) in [
        (
            panels[0].clone(),
            "Roll angular momentum through braking",
            "Angular momentum (mN·m·s)",
            -25.0..64.0,
        ),
        (
            panels[1].clone(),
            "Foot roll orientation",
            "Roll (mrad)",
            -45.0..48.0,
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let mut chart = ChartBuilder::on(&area)
            .margin(15)
            .caption(title, ("sans-serif", 24))
            .x_label_area_size(40)
            .y_label_area_size(83)
            .build_cartesian_2d(760f64..1065f64, range.clone())?;
        chart
            .configure_mesh()
            .x_desc("Physical time (ms)")
            .y_desc(ylabel)
            .light_line_style(RGBColor(234, 237, 238))
            .draw()?;
        for (t, text) in [
            (765.5, "impact"),
            (806., "brake"),
            (866., "park"),
            (1060., "gate"),
        ] {
            chart.draw_series(std::iter::once(PathElement::new(
                [(t, range.start), (t, range.end)],
                RGBColor(182, 186, 190),
            )))?;
            chart.draw_series(std::iter::once(Text::new(
                text,
                (
                    if t > 1050. { t - 8. } else { t + 1. },
                    range.end - (range.end - range.start) * 0.07,
                ),
                ("sans-serif", 13).into_font().color(&RGBColor(60, 65, 73)),
            )))?;
        }
        for (lane, s) in series.iter().enumerate() {
            let points = s
                .iter()
                .map(|row| {
                    (
                        num(&row["time_ms"]),
                        1000.
                            * num(&row["after"][if i == 0 {
                                "h_nms"
                            } else {
                                "roll_pitch_yaw_rad"
                            }][0]),
                    )
                })
                .filter(|(x, _)| (760.0..=1065.).contains(x));
            chart
                .draw_series(LineSeries::new(points, colors[lane].stroke_width(2)))?
                .label(names[lane])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 14, y)], colors[lane].stroke_width(2))
                });
        }
        if i == 1 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::LowerRight)
                .background_style(WHITE.mix(0.9))
                .border_style(RGBColor(190, 190, 190))
                .draw()?;
        }
    }
    drawing.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"matched_controls":mixed.len(),"bodies_per_mixed_control":2,"first_different_tick":first,"max_angular_residual_nms":max_angular,"max_native_angular_residual_nms":max_native_angular,"max_linear_residual_ns":max_linear,"max_native_linear_residual_ns":max_native_linear,"max_scalar_receipt_difference_ns":normal_scalar,"closed_form":physical})
        )?
    );
    Ok(())
}
