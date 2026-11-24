// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare committed native normals with finite oriented boxes at the first paid native solve.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
#[path = "../finite_box_normal.rs"]
mod geometry;
use geometry::{ONE_NM, event, number, vec3};
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn distribution(mut v: Vec<f64>) -> Value {
    v.sort_by(f64::total_cmp);
    let q = |p: usize| (!v.is_empty()).then(|| v[(p * v.len()).div_ceil(100) - 1]);
    json!({"n":v.len(),"min":v.first(),"median":q(50),"p95":q(95),"max":v.last()})
}
fn plot(dir: &Path, rows: &[Value]) -> Result<()> {
    let file = dir.join("first-normal.svg");
    let root = SVGBackend::new(&file, (1130, 745)).into_drawing_area();
    root.fill(&WHITE)?;
    let (top, graphs) = root.split_vertically(90);
    top.draw(&Text::new(
        "The normal at the first paid second-contact solve",
        (28, 34),
        ("sans-serif", 26).into_font(),
    ))?;
    top.draw(&Text::new(format!("{} / 50 source files  ·  the two material boxes use the measured incoming native poses",rows.len()),(28,66),("sans-serif",16).into_font()))?;
    for (panel, held) in graphs.split_evenly((2, 1)).into_iter().zip([false, true]) {
        let cohort = rows
            .iter()
            .filter(|r| r["status"] == "measured" && r["held"] == held)
            .collect::<Vec<_>>();
        let peak = cohort
            .iter()
            .flat_map(|r| {
                ["retained_r1", "cold_r1"]
                    .map(|k| r[k]["before"]["native_angle_mrad"].as_f64().unwrap_or(0.))
            })
            .fold(1., f64::max);
        let mut chart = ChartBuilder::on(&panel)
            .margin(12)
            .x_label_area_size(34)
            .y_label_area_size(65)
            .caption(
                if held {
                    "Held-out offsets"
                } else {
                    "Calibration offsets"
                },
                ("sans-serif", 18),
            )
            .build_cartesian_2d(0usize..250, 0.0..peak * 1.15)?;
        chart
            .configure_mesh()
            .x_desc("Original fixture index")
            .y_desc("Normal error (mrad)")
            .draw()?;
        for (name, label, color) in [
            ("retained_r1", "Retained pair", RGBColor(183, 72, 42)),
            ("cold_r1", "Rebuilt pair", RGBColor(30, 117, 142)),
        ] {
            chart
                .draw_series(cohort.iter().map(|r| {
                    Circle::new(
                        (
                            r["case"].as_u64().unwrap() as usize,
                            r[name]["before"]["native_angle_mrad"].as_f64().unwrap(),
                        ),
                        4,
                        color.filled(),
                    )
                }))?
                .label(label)
                .legend(move |(x, y)| Circle::new((x, y), 4, color.filled()));
        }
        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.85))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let a = env::args().skip(1).collect::<Vec<_>>();
    if a.len() != 3 {
        return Err(
            "secondary_box_normals --events STEPS OUTPUT, or --field NATURAL OUTPUT".into(),
        );
    }
    let src = Path::new(&a[1]);
    let out = Path::new(&a[2]);
    fs::create_dir_all(out)?;
    let result = if a[0] == "--events" {
        let j = read(src)?;
        let e = &j["events"]["first_retained"];
        json!({"case":j["case"],"retained":event(&e["retained"])?,"rebuilt":event(&e["rebuilt"])?})
    } else if a[0] == "--field" {
        let mut rows = vec![];
        for dir in fs::read_dir(src.join("cases"))? {
            let file = dir?.path();
            if file.extension().and_then(|f| f.to_str()) != Some("json") {
                continue;
            }
            let j = read(&file)?;
            if j["format"] != "secondary-native-unmodified-relaxation-v1" {
                return Err("wrong source version".into());
            }
            if j["status"] != "measured" {
                rows.push(json!({"case":j["case"],"status":j["status"],"reason":j["reason"]}));
                continue;
            }
            let mut r = json!({"case":j["case"],"held":j["held"],"status":"measured"});
            for t in j["treatments"]
                .as_array()
                .ok_or("missing native treatments")?
            {
                if t["calls"] != 4096 || (t["passes"] != 1 && t["passes"] != 32) {
                    continue;
                }
                let item = &t["first_positive"];
                if item["elapsed_ns"] != t["first_ns"] {
                    return Err("wrong native interval geometry".into());
                }
                let name = format!(
                    "{}_r{}",
                    t["graph"].as_str().ok_or("missing graph")?,
                    t["passes"]
                );
                r[name] = event(item)?;
            }
            rows.push(r);
        }
        rows.sort_by_key(|r| r["case"].as_u64());
        let mut summary = json!({"published":rows.len(),"expected":50});
        for (held, label) in [(false, "calibration"), (true, "holdout")] {
            let cohort = rows
                .iter()
                .filter(|r| r["held"] == held && r["status"] == "measured")
                .collect::<Vec<_>>();
            for p in [1, 32] {
                let warm = format!("retained_r{p}");
                let cold = format!("cold_r{p}");
                let mut before_gaps = vec![];
                let mut after_gaps = vec![];
                let mut warm_angles = vec![];
                let mut cold_angles = vec![];
                let mut material = vec![];
                let mut closer = [0; 3];
                let mut ambiguous = 0;
                let mut joined = 0;
                let mut crossed = 0;
                for r in &cohort {
                    let a = &r[&warm];
                    let b = &r[&cold];
                    if a["elapsed_ns"] != b["elapsed_ns"] {
                        continue;
                    }
                    let pre = number(&a["before"]["signed_distance_m"])?;
                    let post = number(&a["after"]["signed_distance_m"])?;
                    if (pre - number(&b["before"]["signed_distance_m"])?).abs() > 1e-10
                        || vec3(&a["before"]["normal_to_foot"])?
                            .distance(vec3(&b["before"]["normal_to_foot"])?)
                            > 1e-5
                    {
                        continue;
                    }
                    joined += 1;
                    before_gaps.push(pre * 1e9);
                    after_gaps.push(post * 1e9);
                    crossed += usize::from(pre > 0. && post <= 0.);
                    let w = &a["before"];
                    let c = &b["before"];
                    if number(&w["near_normal_span_mrad"])? > 1. || pre.abs() < ONE_NM {
                        ambiguous += 1;
                        continue;
                    }
                    let wa = number(&w["native_angle_mrad"])?;
                    let ca = number(&c["native_angle_mrad"])?;
                    warm_angles.push(wa);
                    cold_angles.push(ca);
                    material.push(
                        number(&w["over_one_um_from_nearest_fraction"])?
                            .max(number(&c["over_one_um_from_nearest_fraction"])?),
                    );
                    closer[if wa - ca > 0.01 {
                        0
                    } else if ca - wa > 0.01 {
                        1
                    } else {
                        2
                    }] += 1;
                }
                summary[label][format!("r{p}")] = json!({"measured":cohort.len(),"same_observed_time_geometry":joined,"crossed_during_retained_loaded_solve":crossed,
                  "ambiguous_within_1nm_or_1mrad":ambiguous,"rebuilt_closer_over_0_01_mrad":closer[0],"retained_closer_over_0_01_mrad":closer[1],"within_0_01_mrad":closer[2],
                  "retained_normal_angle_mrad":distribution(warm_angles),"rebuilt_normal_angle_mrad":distribution(cold_angles),"separation_before_nm":distribution(before_gaps),
                  "separation_after_nm":distribution(after_gaps),"largest_fraction_on_anchors_over_1um_from_nearest":distribution(material)});
            }
        }
        plot(out, &rows)?;
        summary["cases"] = json!(rows);
        summary
    } else {
        return Err("unknown mode".into());
    };
    fs::write(
        out.join("geometry.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    Ok(())
}
