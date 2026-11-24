// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Crosscheck matched trajectories and isolate the first finite-geometry impulse from the later reactions.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn get(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn vec(v: &Value) -> Result<DVec3> {
    let x: [f64; 3] = serde_json::from_value(v.clone())?;
    Ok(DVec3::from(x))
}
fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "finite quantity missing".into())
}
fn list(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "expected list".into())
}
fn one(v: &Value, f: impl Fn(&Value) -> bool) -> Result<&Value> {
    let mut it = list(v)?.iter().filter(|x| f(x));
    let r = it.next().ok_or("required measurement absent")?;
    if it.next().is_some() {
        return Err("duplicate match".into());
    }
    Ok(r)
}
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 4 {
        return Err("finite_policy_case FINITE_DIR ORIGINAL_DIR INDEX STROKE".into());
    }
    let base = Path::new(&args[0]);
    let index: usize = args[2].parse()?;
    let stroke: u64 = args[3].parse()?;
    let file = format!("cases/case-{index:04}.json");
    let d = get(&base.join(&file))?;
    let o = get(&Path::new(&args[1]).join(file))?;
    let trial = one(&d["result"]["trials"], |v| v["stroke"] == stroke)?;
    let historic = one(&o["result"]["trials"], |v| v["stroke"] == stroke)?;
    if trial["cut"] != historic["cut"] {
        return Err("physical comparison requires identical incoming state".into());
    }
    let native = one(&trial["paths"], |p| p["profile"] == "native8")?;
    let finite = one(&trial["paths"], |p| p["profile"] == "finite8")?;
    let previous = &one(&historic["paths"], |p| {
        p["method"] == "uniform" && p["calls"] == 4096
    })?["record"];
    let out = base.join(format!("case-{index:04}-stroke-{stroke}"));
    let fine_trace = get(&out.join("traces/fine4096.json"))?;
    let fine = &fine_trace["measurements"];
    let mut reference_delta = vec![];
    let mut rows = vec![];
    let view = [("native8", native), ("finite8", finite), ("fine4096", fine)];
    for (name, record) in view {
        let trace = get(&out.join(format!("traces/{name}.json")))?;
        if &trace["measurements"] != record {
            return Err("replay measurement archive mismatch".into());
        }
        let frames = list(&trace["frames"])?;
        if frames.len() != 27 || frames[0]["tick"] != trial["cut"]["tick"] {
            return Err("unexpected visual frames".into());
        }
        let mut sums = 0.;
        for (i, frame) in frames.iter().enumerate().skip(1) {
            sums += number(&frame["normal_impulse_ns"])?;
            if [1, 2, 10, 26].contains(&i) {
                let s = one(&record["samples"], |s| s["i"] == i)?;
                let r = one(&fine["samples"], |s| s["i"] == i)?;
                if name == "fine4096" {
                    let h = one(&previous["samples"], |s| s["i"] == i)?;
                    reference_delta.push(json!({"step":i,"native_vs_prior_com_m":vec(&s["com_m"])?.distance(vec(&h["com_m"])?),
                        "native_minus_prior_normal_ns":number(&s["cumulative_normal_ns"])?-number(&h["cumulative_normal_ns"])?,
                        "native_vs_prior_tangent_ns":vec(&s["cumulative_tangent_ns"])?.distance(vec(&h["cumulative_tangent_ns"])?)}));
                }
                let pos =
                    (vec(&frame["foot"]["position"])? + vec(&frame["upper"]["position"])?) * 0.5;
                if pos.distance(vec(&s["com_m"])?) > 1e-12
                    || (sums - number(&s["cumulative_normal_ns"])?).abs() > 1e-10
                {
                    return Err("native visual trace disagrees with experiment".into());
                }
                rows.push(json!({"lane":name,"step":i,"first_end_ns":record["first_loaded_native_end_ns"],"normal_ns":sums,
                 "tangent_ns":s["cumulative_tangent_ns"],"tangent_distance_to_4096_ns":vec(&s["cumulative_tangent_ns"])?.distance(vec(&r["cumulative_tangent_ns"])?),
                 "com_to_4096_mm":1000.*pos.distance(vec(&r["com_m"])?),"spin_to_4096_rad_s":vec(&s["spin_rad_s"])?.distance(vec(&r["spin_rad_s"])?)}));
            }
        }
    }
    let geometry = &trial["geometry"];
    let jump = list(&finite["impacts"])?.first().ok_or("no finite event")?;
    let impulse = vec(&jump["impulse_ns"])?;
    let n = vec(&jump["normal"])?;
    let jn = impulse.dot(n);
    let jt = impulse - n * jn;
    let physical = vec(&geometry["instantaneous_impulse_ns"])?;
    if impulse.distance(physical) > 1e-10
        || jump["local_ns"] != geometry["hi_ns"]
        || jump["feature"] != geometry["feature"]
    {
        return Err("predicted and native registered jumps disagree".into());
    }
    let path = out.join("view/finite-landing.svg");
    let root = SVGBackend::new(&path, (1020, 830)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(
            "Contact and motion after the same geometric touchdown",
            ("sans-serif", 24),
        )?
        .split_evenly((3, 1));
    for (a, (key, title, units)) in areas.iter().zip([
        ("normal_ns", "Cumulative scalar normal reaction", "N·s"),
        (
            "tangent_distance_to_4096_ns",
            "Cumulative tangential vector distance from 4,096 calls",
            "N·s",
        ),
        (
            "com_to_4096_mm",
            "Center of mass distance from 4,096 calls",
            "mm",
        ),
    ]) {
        let top = rows
            .iter()
            .map(|r| number(&r[key]))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .fold(0.001, f64::max)
            * 1.15;
        let mut c = ChartBuilder::on(a)
            .margin(8)
            .caption(title, ("sans-serif", 18))
            .x_label_area_size(36)
            .y_label_area_size(65)
            .build_cartesian_2d(0f64..13.3, 0f64..top)?;
        c.configure_mesh()
            .x_desc("Time since the incoming cut (ms)")
            .y_desc(units)
            .max_light_lines(0)
            .draw()?;
        for ((lane, _), (label, color)) in view.into_iter().zip([
            ("Eight native calls", RGBColor(188, 76, 30)),
            ("Finite impact + eight", RGBColor(13, 125, 99)),
            ("4,096 calls", RGBColor(70, 95, 176)),
        ]) {
            let pts = rows
                .iter()
                .filter(|r| r["lane"] == lane)
                .map(|r| Ok((number(&r["step"])? * 0.5, number(&r[key])?)))
                .collect::<Result<Vec<_>>>()?;
            c.draw_series(LineSeries::new(pts.clone(), color.stroke_width(2)))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 18, y)], color.stroke_width(2))
                });
            c.draw_series(pts.into_iter().map(|p| Circle::new(p, 4, color.filled())))?;
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::MiddleRight)
            .background_style(WHITE.mix(0.94))
            .draw()?;
    }
    root.present()?;
    fs::write(
        out.join("checks/finite-landing.json"),
        serde_json::to_vec_pretty(
            &json!({"case":index,"stroke":stroke,"feature":jump["feature"],"native_event_ns":jump["local_ns"],
 "geometric_bracket_ns":[geometry["lo_ns"],geometry["hi_ns"]],"instantaneous_scalar_normal_ns":jn,"instantaneous_tangent_vector_ns":jt.to_array(),"instantaneous_tangent_magnitude_ns":jt.length(),
 "instantaneous_vector_error_ns":impulse.distance(physical),"geometric_impulse_ns":physical.to_array(),"rows":rows,"native_vs_prior_4096":reference_delta}),
        )?,
    )?;
    Ok(())
}
