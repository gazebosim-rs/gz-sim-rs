// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recover point, moment and total-momentum balances from the measured return trajectories.
use glam::{DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn rows(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "missing rows".into())
}
fn num(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing measurement".into())
}
fn arr<const N: usize>(v: &Value) -> Result<[f64; N]> {
    rows(v)?
        .iter()
        .map(num)
        .collect::<Result<Vec<_>>>()?
        .try_into()
        .map_err(|_| "wrong measurement length".into())
}
fn vec(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(arr(v)?))
}
fn close(x: f64, y: f64) -> Result<()> {
    if (x - y).abs() > 1e-10 * x.abs().max(y.abs()).max(1.) {
        Err(format!("mismatch {x} and {y}").into())
    } else {
        Ok(())
    }
}
fn sub(v: DVec3, w: DVec3) -> Result<()> {
    close(v.distance(w), 0.)
}
fn momentum(foot: &Value, upper: &Value) -> Result<(DVec3, DVec3)> {
    let total = vec(&foot["velocity"])? + vec(&upper["velocity"])?;
    let com = 0.5 * (vec(&foot["position"])? + vec(&upper["position"])?);
    let mut h = DVec3::ZERO;
    for r in [foot, upper] {
        let q = DQuat::from_array(arr(&r["orientation"])?).normalize();
        let spin = q.conjugate() * vec(&r["angular_velocity"])?;
        h += q * (DVec3::new(0.0201, 0.0201, 0.025) * spin)
            + (vec(&r["position"])? - com).cross(vec(&r["velocity"])? - 0.5 * total);
    }
    Ok((total, h))
}
fn receipt(t: &Value) -> Result<(DVec3, DVec3)> {
    let n = num(&t["normal_ns"])?;
    close(arr::<4>(&t["quadrant_normal_ns"])?.iter().sum(), n)?;
    Ok((
        vec(&t["normal_vector_ns"])? + vec(&t["tangent_vector_ns"])?,
        vec(&t["normal_moment_nms"])? + vec(&t["tangent_moment_nms"])?,
    ))
}
fn verify_event(r: &Value) -> Result<()> {
    if r.is_null() {
        return Ok(());
    }
    let (j, h) = receipt(&r["integrals"])?;
    let mut point_j = DVec3::ZERO;
    let mut point_h = DVec3::ZERO;
    let mut q = [0.; 4];
    for p in rows(&r["paid_points"])? {
        let f = vec(&p["normal_vector_ns"])? + vec(&p["tangent_vector_ns"])?;
        point_j += f;
        point_h += vec(&p["arm_to_assembly_com_m"])?.cross(f);
        let k = p["quadrant"]
            .as_u64()
            .filter(|v| *v < 4)
            .ok_or("wrong quadrant")? as usize;
        q[k] += num(&p["normal_ns"])?;
    }
    sub(j, point_j)?;
    sub(h, point_h)?;
    for (x, y) in q
        .into_iter()
        .zip(arr::<4>(&r["integrals"]["quadrant_normal_ns"])?)
    {
        close(x, y)?;
    }
    let solver_normal = rows(&r["solver_rows"])?
        .iter()
        .map(|p| num(&p["normal_ns"]))
        .collect::<Result<Vec<_>>>()?
        .iter()
        .sum();
    close(solver_normal, num(&r["integrals"]["normal_ns"])?)?;
    let (pi, hi) = momentum(&r["before"][0], &r["before"][1])?;
    let (pf, hf) = momentum(&r["after"][0], &r["after"][1])?;
    let gravity = DVec3::Z * (-2. * 9.81 * num(&r["native_dt_ns"])? * 1e-9);
    if (pf - pi - gravity - j).length() > 1e-6 || (hf - hi - h).length() > 1e-6 {
        return Err("native point event fails momentum balance".into());
    }
    Ok(())
}
fn compare_rows(a: &Value, b: &Value) -> Result<Value> {
    let a = rows(a)?;
    let b = rows(b)?;
    let quadrant = |p: &Value| -> Result<usize> {
        let v = vec(&p["local_foot"])?;
        Ok(2 * usize::from(v.x > 0.) + usize::from(v.y > 0.))
    };
    let qa = a.iter().map(quadrant).collect::<Result<Vec<_>>>()?;
    let qb = b.iter().map(quadrant).collect::<Result<Vec<_>>>()?;
    let mut used = vec![false; b.len()];
    let mut matched = 0usize;
    let mut max = 0.0f64;
    for p in a {
        let anchor = vec(&p["local_foot"])?;
        let mut best = None;
        for (i, x) in b.iter().enumerate() {
            if used[i] {
                continue;
            }
            let d = anchor.distance(vec(&x["local_foot"])?);
            if best.is_none_or(|(_, v)| d < v) {
                best = Some((i, d));
            }
        }
        if let Some((i, d)) = best {
            used[i] = true;
            max = max.max(d);
            matched += 1;
        }
    }
    Ok(
        json!({"retained_quadrant_order":qa,"other_quadrant_order":qb,"row_order_changed":qa!=qb,"retained_count":a.len(),"other_count":b.len(),"matched_anchors":matched,
     "largest_nearest_matched_anchor_m":if matched==a.len()&&a.len()==b.len(){Some(max)}else{None},
     "pair_order_retained":a.first().map(|p|p["foot_is_first"].clone()),"pair_order_other":b.first().map(|p|p["foot_is_first"].clone())}),
    )
}
fn main() -> Result<()> {
    let input = env::args()
        .nth(1)
        .ok_or("secondary_points PHASE/case-010")?;
    let dir = Path::new(&input);
    let source = read(&dir.join("points.json"))?;
    let mut tested = vec![];
    let mut charts = vec![];
    let mut largest_p = 0.0f64;
    let mut largest_h = 0.0f64;
    let mut public_count = 0;
    let mut sampled_events = 0;
    for donor in rows(&source["donors"])? {
        let name = donor["incoming"].as_str().ok_or("missing incoming donor")?;
        let folder = dir.join(name);
        let standalone = read(&folder.join("points.json"))?;
        if donor != &standalone {
            return Err("detached donor inconsistent".into());
        }
        let branches = rows(&donor["branches"])?;
        for b in branches {
            let tag = format!(
                "{}{}",
                b["graph"].as_str().ok_or("graph absent")?,
                b["grid"].as_u64().ok_or("grid absent")?
            );
            let tr = read(&folder.join(format!("traces/{tag}.json")))?;
            let frames = rows(&tr["frames"])?;
            if frames.len() != 11 || rows(&b["public"])?.len() != 10 {
                return Err("incomplete point record".into());
            }
            for (k, t) in rows(&b["public"])?.iter().enumerate() {
                let r = &frames[k + 1];
                if r["tick"] != t["tick"]
                    || r["native_steps"] != b["grid"]
                    || r["corner_ns"] != t["totals"]["quadrant_normal_ns"]
                {
                    return Err("public pose trace or corner totals disagree".into());
                }
                let (j, h) = receipt(&t["totals"])?;
                let (pi, hi) = momentum(&frames[k]["foot"], &frames[k]["upper"])?;
                let (pf, hf) = momentum(&r["foot"], &r["upper"])?;
                let dynamic = pf - pi + DVec3::Z * 0.00981;
                sub(dynamic, vec(&t["independent_delta_p_minus_gravity_ns"])?)?;
                let ep = (dynamic - j).length();
                let eh = (hf - hi - h).length();
                close(ep, num(&t["dp_residual_ns"])?)?;
                close(eh, num(&t["dh_residual_nms"])?)?;
                largest_p = largest_p.max(ep);
                largest_h = largest_h.max(eh);
                for field in ["first_positive", "first_after_unloaded_native_call", "peak"] {
                    verify_event(&t[field])?;
                    sampled_events += usize::from(!t[field].is_null());
                }
                public_count += 1;
            }
        }
        let mut changes = vec![];
        for n in [512, 4096] {
            let select = |graph: &str| -> Result<&Value> {
                branches
                    .iter()
                    .find(|b| b["graph"] == graph && b["grid"] == n)
                    .ok_or_else(|| "branch absent".into())
            };
            let retained = select("retained")?;
            let cold = select("cold")?;
            let fresh = select("fresh")?;
            let a = &retained["public"][0];
            let (j, h) = receipt(&a["totals"])?;
            let mut alternatives = vec![];
            for (graph, other) in [("cold", cold), ("fresh", fresh)] {
                let o = &other["public"][0];
                let (jo, ho) = receipt(&o["totals"])?;
                let signature = compare_rows(
                    &a["first_positive"]["solver_rows"],
                    &o["first_positive"]["solver_rows"],
                )?;
                let p = vec(&a["totals"]["normal_weighted_point_local_m"])?;
                let po = vec(&o["totals"]["normal_weighted_point_local_m"])?;
                alternatives.push(json!({"graph":graph,"first_local_ns":o["first_positive"]["elapsed_ns"],"signature":signature,
           "normal_change_ns":num(&o["totals"]["normal_ns"])?-num(&a["totals"]["normal_ns"])?,
           "impulse_vector_change_ns":(jo-j).to_array(),"impulse_distance_ns":jo.distance(j),
           "moment_change_nms":(ho-h).to_array(),"moment_distance_nms":ho.distance(h),
           "normal_center_local_displacement_mm":1000.*po.distance(p),"quadrants":o["totals"]["quadrant_normal_ns"]}));
            }
            let (fc, hc) = receipt(&cold["public"][0]["totals"])?;
            let (ff, hf) = receipt(&fresh["public"][0]["totals"])?;
            changes.push(json!({"grid":n,"retained_first_local_ns":a["first_positive"]["elapsed_ns"],"retained_quadrants":a["totals"]["quadrant_normal_ns"],"alternatives":alternatives,"cold_fresh_impulse_distance_ns":fc.distance(ff),"cold_fresh_moment_distance_nms":hc.distance(hf)}));
            if name == "continuous4096r1" && n == 4096 {
                for b in [retained, cold, fresh] {
                    charts.push(b["public"][0].clone());
                }
            }
        }
        tested.push(json!({"incoming":name,"experiments":changes}));
    }
    let summary = json!({"sources":tested,"verified_public_intervals":public_count,"verified_native_event_records":sampled_events,
    "largest_native_public_momentum_residual_ns":largest_p,"largest_native_public_angular_residual_nms":largest_h});
    fs::write(
        dir.join("readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    let file = dir.join("secondary-contact-points.svg");
    let root = SVGBackend::new(&file, (1250, 625)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "Which sole points carry the resumed load?",
        (28, 38),
        ("sans-serif", 27).into_font(),
    ))?;
    root.draw(&Text::new("Continuous fine incoming donor  ·  first 500 µs after the second landing  ·  4,096 native calls in each lane",(28,73),("sans-serif",18).into_font()))?;
    let panes = root.margin(98, 96, 20, 20).split_evenly((1, 3));
    let colors = [
        RGBColor(198, 99, 42),
        RGBColor(45, 139, 128),
        RGBColor(65, 105, 169),
    ];
    for (g, panel) in panes.iter().enumerate() {
        let x = &charts[g];
        let q = arr::<4>(&x["totals"]["quadrant_normal_ns"])?;
        let mut chart = ChartBuilder::on(panel)
            .caption(
                ["Retained geometry", "Rebuilt graph", "Fresh sole features"][g],
                ("sans-serif", 21),
            )
            .margin(8)
            .x_label_area_size(40)
            .y_label_area_size(45)
            .build_cartesian_2d(-190.0..194.0, -135.0..136.0)?;
        chart
            .configure_mesh()
            .x_desc("Sole-local x (mm)")
            .y_desc("y (mm)")
            .light_line_style(RGBColor(235, 238, 240))
            .draw()?;
        chart.draw_series([
            Rectangle::new(
                [(-180., -100.), (180., 100.)],
                RGBColor(240, 244, 247).filled(),
            ),
            Rectangle::new(
                [(-180., -100.), (180., 100.)],
                RGBColor(143, 155, 165).stroke_width(1),
            ),
        ])?;
        for (n, r) in rows(&x["first_positive"]["solver_rows"])?
            .iter()
            .enumerate()
        {
            let local = vec(&r["local_foot"])?;
            let i = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
            let (sx, sy) = (1000. * local.x, 1000. * local.y);
            let rad = (q[i].sqrt() * 15.).round() as i32;
            chart.draw_series([Circle::new((sx, sy), rad, colors[g].mix(0.82).filled())])?;
            chart.draw_series([Text::new(
                format!("{}  {:.3}", n + 1, q[i]),
                (sx - 25., if sy > 0. { sy - 27. } else { sy + 29. }),
                ("sans-serif", 14).into_font(),
            )])?;
        }
        let center = vec(&x["totals"]["normal_weighted_point_local_m"])?;
        chart.draw_series([Cross::new(
            (1000. * center.x, 1000. * center.y),
            8,
            BLACK.stroke_width(2),
        )])?;
    }
    root.draw(&Text::new("Circle area shows quadrant-normal impulse; each label gives native solver-row sequence and N·s.",(30,560),("sans-serif",17).into_font()))?;
    root.draw(&Text::new("Cross marks the normal-impulse-weighted location in the foot frame. Anchors are measured at the first positive native solve.",(30,590),("sans-serif",16).into_font()))?;
    root.present()?;
    Ok(())
}
