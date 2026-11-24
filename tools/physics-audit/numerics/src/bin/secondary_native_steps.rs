// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently reconstruct the first differing native event and source geometry.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn v(a: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(a.clone())?))
}
fn n(a: &Value) -> Result<f64> {
    a.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing finite value".into())
}
fn check(b: bool, e: &str) -> Result<()> {
    if b { Ok(()) } else { Err(e.into()) }
}
#[derive(Clone, Copy)]
struct Frame {
    ns: u64,
    warm: DVec3,
    cold: DVec3,
    dw: DVec3,
    dc: DVec3,
}
struct Site {
    quadrant: u8,
    foot: DVec3,
    pad: DVec3,
    normal: DVec3,
    gap: f64,
    impulse: f64,
}
fn points(entry: &Value) -> Result<Vec<Site>> {
    let mut out = Vec::new();
    for r in entry["solver_rows"]
        .as_array()
        .ok_or("missing native geometry")?
    {
        let p = v(&r["foot_m"])?;
        let q = (u8::from(p.x > 0.) << 1) | u8::from(p.y > 0.);
        out.push(Site {
            quadrant: q,
            foot: p,
            pad: v(&r["pad_m"])?,
            normal: v(&r["normal_on_foot"])?,
            gap: n(&r["observed_gap_m"])?,
            impulse: n(&r["normal_ns"])?,
        });
    }
    out.sort_by(|a, b| {
        a.quadrant
            .cmp(&b.quadrant)
            .then_with(|| a.foot.x.total_cmp(&b.foot.x))
            .then_with(|| a.foot.y.total_cmp(&b.foot.y))
    });
    Ok(out)
}
fn feature(e: &Value) -> Result<Value> {
    let a = points(&e["retained"])?;
    let b = points(&e["rebuilt"])?;
    if a.len() != b.len() || a.iter().zip(&b).any(|(p, q)| p.quadrant != q.quadrant) {
        return Ok(json!({"matching_features":false}));
    }
    let mut ds = [0f64; 4];
    let mut loads = Vec::new();
    for (p, q) in a.iter().zip(&b) {
        let deltas = [
            p.foot.distance(q.foot),
            p.pad.distance(q.pad),
            p.normal.distance(q.normal),
            (p.gap - q.gap).abs(),
        ];
        for (peak, d) in ds.iter_mut().zip(deltas) {
            *peak = peak.max(d);
        }
        loads.push(json!({"quadrant":p.quadrant,"retained_ns":p.impulse,"rebuilt_ns":q.impulse,"retained_gap_m":p.gap,"rebuilt_gap_m":q.gap}));
    }
    Ok(
        json!({"matching_features":true,"max_foot_anchor_m":ds[0],"max_pad_anchor_m":ds[1],
             "max_normal_vector":ds[2],"max_geometric_gap_delta_m":ds[3],"points":loads}),
    )
}
fn draw(path: &Path, rows: &[Frame], first: u64, focus: u64) -> Result<()> {
    let root = SVGBackend::new(path, (1120, 800)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(
            "Inside the second strike with the same solver row ordering",
            ("sans-serif", 25),
        )?
        .split_evenly((2, 2));
    let blue = RGBColor(34, 85, 145);
    let green = RGBColor(4, 127, 108);
    let orange = RGBColor(186, 86, 34);
    let norm = |r: &Frame| r.warm.distance(r.cold);
    let range = |x: Vec<f64>| {
        let hi = x.into_iter().fold(0., f64::max);
        0.0..hi.max(1e-9) * 1.18
    };
    let full = rows.last().ok_or("empty native interval")?.ns as f64 * 0.001;
    let mut c = ChartBuilder::on(&areas[0])
        .caption("Accumulated native reaction", ("sans-serif", 18))
        .margin(14)
        .x_label_area_size(48)
        .y_label_area_size(65)
        .build_cartesian_2d(
            0.0..full,
            range(
                rows.iter()
                    .flat_map(|r| [r.warm.length(), r.cold.length()])
                    .collect(),
            ),
        )?;
    c.configure_mesh()
        .max_light_lines(1)
        .x_desc("Time since incoming cut (µs)")
        .y_desc("Resultant norm (N·s)")
        .draw()?;
    for (native, color, label) in [(true, blue, "Retained"), (false, green, "Rebuilt")] {
        c.draw_series(LineSeries::new(
            std::iter::once((0., 0.)).chain(rows.iter().map(|r| {
                (
                    r.ns as f64 * 0.001,
                    if native { r.warm } else { r.cold }.length(),
                )
            })),
            color.stroke_width(2),
        ))?
        .label(label)
        .legend(move |(x, y)| PathElement::new([(x, y), (x + 16, y)], color.stroke_width(2)));
    }
    c.configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    let mut c = ChartBuilder::on(&areas[1])
        .caption("Cumulative vector disagreement", ("sans-serif", 18))
        .margin(14)
        .x_label_area_size(48)
        .y_label_area_size(65)
        .build_cartesian_2d(0.0..full, range(rows.iter().map(norm).collect()))?;
    c.configure_mesh()
        .max_light_lines(1)
        .x_desc("Time since incoming cut (µs)")
        .y_desc("Vector distance (N·s)")
        .draw()?;
    c.draw_series(LineSeries::new(
        rows.iter().map(|r| (r.ns as f64 * 0.001, norm(r))),
        orange.stroke_width(2),
    ))?;
    for (area, center, caption) in [
        (&areas[2], first, "At the first paid native solve"),
        (&areas[3], focus, "At the first difference ≥ 1 µN·s"),
    ] {
        let pts: Vec<_> = rows
            .iter()
            .filter(|r| r.ns.abs_diff(center) <= 2600)
            .collect();
        let mut c = ChartBuilder::on(area)
            .caption(caption, ("sans-serif", 18))
            .margin(14)
            .x_label_area_size(55)
            .y_label_area_size(65)
            .build_cartesian_2d(
                -2.7..2.7,
                range(
                    pts.iter()
                        .flat_map(|r| [r.dw.length(), r.dc.length()])
                        .collect(),
                ),
            )?;
        c.configure_mesh()
            .max_light_lines(1)
            .x_desc(format!(
                "Time relative to {:.3} µs (µs)",
                center as f64 * 0.001
            ))
            .y_desc("Per-solve resultant (N·s)")
            .draw()?;
        for (flag, color) in [(true, blue), (false, green)] {
            c.draw_series(pts.iter().map(|r| {
                let x = (r.ns as f64 - center as f64) * 0.001;
                let y = if flag { r.dw } else { r.dc }.length();
                Circle::new(
                    (x, y),
                    if flag { 5 } else { 3 },
                    if flag {
                        color.stroke_width(1)
                    } else {
                        color.filled()
                    },
                )
            }))?;
        }
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let dir = env::args_os()
        .nth(1)
        .ok_or("secondary_native_steps CASE_DIR")?;
    let dir = Path::new(&dir);
    let raw = read(&dir.join("steps.json"))?;
    let native = raw["elapsed_ns_and_world_impulses"]
        .as_array()
        .ok_or("missing native solves")?;
    check(
        native.len() == 4096 && raw["native_calls"] == 4096 && raw["first_public_ns"] == 500000,
        "wrong native budget",
    )?;
    let (mut warm, mut cold) = (DVec3::ZERO, DVec3::ZERO);
    let mut prev = 0;
    let mut data = Vec::new();
    let mut marks = BTreeMap::new();
    let mut peak = 0.;
    for (i, r) in native.iter().enumerate() {
        let ns = r[0].as_u64().ok_or("missing native clock")?;
        let dw = v(&r[3])?;
        let dc = v(&r[4])?;
        warm += dw;
        cold += dc;
        check(
            (122..=123).contains(&(ns - prev))
                && warm.distance(v(&r[1])?) < 1e-11
                && cold.distance(v(&r[2])?) < 1e-11,
            "invalid native integration or clock",
        )?;
        prev = ns;
        if dw != DVec3::ZERO {
            marks.entry("first_retained").or_insert(i);
        }
        if dc != DVec3::ZERO {
            marks.entry("first_rebuilt").or_insert(i);
        }
        for (value, id) in [
            (1e-9, "first_nano_ns"),
            (1e-6, "first_micro_ns"),
            (1e-3, "first_milli_ns"),
            (1e-2, "first_centins"),
        ] {
            if warm.distance(cold) >= value {
                marks.entry(id).or_insert(i);
            }
        }
        if dw.distance(dc) > peak {
            marks.insert("peak_single_solve", i);
            peak = dw.distance(dc);
        }
        data.push(Frame {
            ns,
            warm,
            cold,
            dw,
            dc,
        });
    }
    check(prev == 500_000, "incomplete actuator tick")?;
    let recorded = raw["events"].as_object().ok_or("missing events")?;
    check(
        recorded.len() == marks.len(),
        "extra or missing threshold crossings",
    )?;
    let mut summary = json!({"case":raw["case"],"native_calls_checked":data.len(),"max_native_vector_delta_ns":peak,
           "final_vector_delta_ns":cold.distance(warm),"final_delta_on_foot_ns":(cold-warm).to_array()});
    for (id, &i) in &marks {
        let x = &raw["events"][id];
        let r = &data[i];
        check(
            x["substep"] == i + 1
                && x["retained"]["ns"] == r.ns
                && x["rebuilt"]["ns"] == r.ns
                && v(&x["delta_j_ns"])?.distance(r.dc - r.dw) < 1e-11,
            "incorrect threshold event",
        )?;
        let wp = v(&x["retained"]["before"][0]["velocity"])?
            + v(&x["retained"]["before"][1]["velocity"])?;
        let cp =
            v(&x["rebuilt"]["before"][0]["velocity"])? + v(&x["rebuilt"]["before"][1]["velocity"])?;
        check(
            (cp - wp).distance(v(&x["delta_before_linear_momentum_ns"])?) < 1e-12,
            "incoming link momentum discrepancy",
        )?;
        summary["events"][id] = json!({"substep":i+1,"elapsed_ns":r.ns,"cumulative_vector_delta_ns":r.warm.distance(r.cold),"native_vector_delta_ns":r.dw.distance(r.dc),
          "incoming_linear_momentum_delta_ns":wp.distance(cp),"incoming_angular_momentum_delta_nms":v(&x["delta_before_angular_momentum_nms"])?.length(),"geometry":feature(x)?});
    }
    let first = marks["first_retained"].min(marks["first_rebuilt"]);
    let focus = *marks.get("first_micro_ns").unwrap_or(&first);
    draw(
        &dir.join("first-native-difference.svg"),
        &data,
        data[first].ns,
        data[focus].ns,
    )?;
    fs::write(
        dir.join("readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}
