// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent comparison of native returning-foot treatments with the material reference.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn check(b: bool, msg: &str) -> Result<()> {
    if b { Ok(()) } else { Err(msg.into()) }
}
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn arr(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing array".into())
}
fn vector(v: &Value) -> Result<DVec3> {
    let v = DVec3::from_array(serde_json::from_value(v.clone())?);
    check(v.is_finite(), "nonfinite vector")?;
    Ok(v)
}
fn num(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing finite number".into())
}
fn text(v: &Value) -> Result<&str> {
    v.as_str().ok_or_else(|| "missing string".into())
}
fn stats(mut v: Vec<f64>) -> Value {
    v.sort_by(f64::total_cmp);
    let q = |p: f64| (!v.is_empty()).then(|| v[((v.len() as f64 * p).ceil() as usize).max(1) - 1]);
    json!({"n":v.len(),"min":v.first(),"median":q(0.5),"p95":q(0.95),"max":v.last()})
}
fn scored(src: &Value, physical: &Value) -> Result<Vec<Value>> {
    for k in ["case", "held", "slot", "cut_tick", "incoming"] {
        check(src[k] == physical[k], "source state mismatch")?;
    }
    let end = arr(&physical["targets"])?
        .iter()
        .find(|s| s["time_ns"] == 500000)
        .ok_or("missing target time")?;
    let onset = &physical["material"];
    let analytic = &onset["response"];
    let mut keys = BTreeSet::new();
    let mut cells = vec![];
    let modes = if src["event_bounds"] == true {
        [
            "native",
            "launch",
            "subdivided",
            "directed",
            "retimed_directed",
        ]
    } else {
        ["native", "point", "retimed", "bound", "launch"]
    };
    for r in arr(&src["rows"])? {
        let c = r["calls"].as_u64().ok_or("missing budget")?;
        let m = text(&r["mode"])?;
        let g = text(&r["graph"])?;
        check(
            r["passes"] == 8
                && [8, 64].contains(&c)
                && ["retained", "cold"].contains(&g)
                && modes.contains(&m)
                && keys.insert((c, g, m)),
            "invalid or duplicate coordinate",
        )?;
        let js = arr(&r["impacts"])?;
        let spans = arr(&r["sustained"])?;
        check(
            js.len() <= 1 && (m != "native" || (js.is_empty() && spans.is_empty())),
            "unexpected primary injection",
        )?;
        let mut s = json!({"case":src["case"],"slot":src["slot"],"held":src["held"],"graph":g,"calls":c,"mode":m,"projected":!js.is_empty(),"spans":spans.len(),"first_ns":r["first_ns"],"balance_ns":r["normal_momentum_residual_ns"],"public_qualified":end["qualified"],"feature":onset["feature"]});
        let mut prior = 0u64;
        let mut span_ns = 0u64;
        let mut from_primary = js.first().and_then(|j| j["time_ns"].as_u64());
        let mut internal = vec![];
        for a in spans {
            let t = a["start_ns"].as_u64().ok_or("missing start")?;
            let h = a["duration_ns"].as_u64().ok_or("missing duration")?;
            check(
                t >= prior && h > 0 && t + h <= 500000,
                "overlapping or invalid material continuation",
            )?;
            if from_primary == Some(t) {
                from_primary = Some(t + h);
            } else {
                from_primary = None;
            }
            if let Some(parts) = a["branches"].as_array() {
                let mut at = 0.;
                for p in parts {
                    let start = num(&p["time_s"])?;
                    let dt = num(&p["span_s"])?;
                    check(
                        (start - at).abs() < 1e-12 && dt > 0.,
                        "piecewise receipt has a gap",
                    )?;
                    at = start + dt;
                    if !p["jump_ns"].is_null() {
                        internal.push((t as f64 + at * 1e9, vector(&p["jump_ns"])?));
                    }
                }
                check(
                    (at - h as f64 * 1e-9).abs() < 1e-12,
                    "internal receipt does not cover its native time",
                )?;
            }
            prior = t + h;
            span_ns += h;
            vector(&a["j_ns"])?;
            check(num(&a["normal_ns"])? >= 0., "negative pressure")?;
        }
        s["sustained_ns"] = json!(span_ns);
        s["whole_from_impact"] = json!(from_primary == Some(500000));
        s["internal_events"] = json!(internal.len());
        if from_primary == Some(500000) && end["qualified"] == true {
            let events = arr(&end["model"]["events"])?;
            s["same_event_count"] = json!(events.len() == internal.len());
            if events.len() == internal.len() && !events.is_empty() {
                let mut time = 0.0_f64;
                let mut impulse = 0.0_f64;
                for ((t, j), e) in internal.iter().zip(events) {
                    time = time.max((t - num(&e["time_s"])? * 1e9).abs());
                    impulse = impulse.max(j.distance(vector(&e["impulse_ns"])?));
                }
                s["peak_internal_time_error_ns"] = json!(time);
                s["peak_internal_jump_error_ns"] = json!(impulse);
            }
        }
        if let Some(j) = js.first() {
            check(
                onset["status"] == "unique",
                "projection on unqualified source",
            )?;
            let lag = num(&j["time_ns"])? - num(&onset["time_s"])? * 1e9;
            check(
                lag.abs() < 1.00001
                    && j["feature"] == onset["feature"]
                    && j["stick"] == analytic["sticking"],
                "registered primary time, feature or stick mode disagreement",
            )?;
            s["projected_time_error_ns"] = json!(lag);
            s["jump_vector_error_ns"] =
                json!(vector(&j["j_ns"])?.distance(vector(&analytic["impulse_ns"])?));
            s["jump_loss_error_j"] = json!((num(&j["loss_j"])? - num(&analytic["loss_j"])?).abs());
        }
        if end["qualified"] == true {
            let e = &end["model"]["end"];
            s["j_error_ns"] =
                json!(vector(&r["world_impulse_ns"])?.distance(vector(&e["total_impulse_ns"])?));
            s["com_error_um"] =
                json!(1e6 * vector(&r["end_com_m"])?.distance(vector(&e["com_m"])?));
            s["spin_error_rad_s"] =
                json!(vector(&r["end_spin_rad_s"])?.distance(vector(&e["spin_rad_s"])?));
        }
        cells.push(s);
    }
    check(keys.len() == 20, "missing treatment")?;
    for r in &mut cells {
        let same = arr(&src["rows"])?
            .iter()
            .find(|x| x["graph"] == r["graph"] && x["calls"] == r["calls"] && x["mode"] == "native")
            .unwrap();
        let own = arr(&src["rows"])?
            .iter()
            .find(|x| {
                x["graph"] == r["graph"] && x["calls"] == r["calls"] && x["mode"] == r["mode"]
            })
            .unwrap();
        let other = arr(&src["rows"])?
            .iter()
            .find(|x| {
                x["graph"] != r["graph"] && x["calls"] == r["calls"] && x["mode"] == r["mode"]
            })
            .unwrap();
        r["graph_difference_ns"] =
            json!(vector(&own["world_impulse_ns"])?.distance(vector(&other["world_impulse_ns"])?));
        if let Some(v) = r["j_error_ns"].as_f64() {
            let ordinary = vector(&same["world_impulse_ns"])?
                .distance(vector(&end["model"]["end"]["total_impulse_ns"])?);
            r["delta_error_ns"] = json!(v - ordinary);
        }
    }
    Ok(cells)
}
fn main() -> Result<()> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    check(
        args.len() == 2,
        "usage: secondary_material_policy STUDY REPORT",
    )?;
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    fs::create_dir_all(out)?;
    let base = dir
        .parent()
        .ok_or("missing studies")?
        .join("multifidelity-secondary-material-response/cases");
    let mut slots = BTreeSet::new();
    let mut rows = vec![];
    for e in fs::read_dir(dir.join("cases"))? {
        let path = e?.path();
        if path.extension().is_none_or(|x| x != "json") {
            continue;
        }
        let v = read(&path)?;
        let ph = read(&base.join(path.file_name().unwrap()))?;
        check(
            slots.insert(v["slot"].as_u64().unwrap()),
            "duplicate source slot",
        )?;
        let mut scored_rows = scored(&v, &ph)?;
        if v["event_bounds"] == true {
            let input = arr(&v["rows"])?;
            for r in &mut scored_rows {
                let same = |m: &str| {
                    input
                        .iter()
                        .find(|x| {
                            x["mode"] == m && x["calls"] == r["calls"] && x["graph"] == r["graph"]
                        })
                        .unwrap()
                };
                let b = same("subdivided");
                let own = same(text(&r["mode"])?);
                if r["mode"] == "directed" {
                    check(
                        own["impacts"] == b["impacts"],
                        "directed continuation modified primary impulse",
                    )?;
                }
                let z = arr(&ph["targets"])?
                    .iter()
                    .find(|s| s["time_ns"] == 500000)
                    .unwrap();
                if z["qualified"] == true {
                    let old = vector(&b["world_impulse_ns"])?
                        .distance(vector(&z["model"]["end"]["total_impulse_ns"])?);
                    r["original_error_ns"] = json!(old);
                    r["directed_change_ns"] = json!(num(&r["j_error_ns"])? - old);
                }
            }
        } else if v["certify_vertices"] == true {
            let prior = read(
                &dir.parent()
                    .unwrap()
                    .join("multifidelity-secondary-face-controls/cases")
                    .join(path.file_name().unwrap()),
            )?;
            check(
                prior["incoming"] == v["incoming"],
                "different unmodified certificate source",
            )?;
            let prev = scored(&prior, &ph)?;
            for r in &mut scored_rows {
                let b = prev
                    .iter()
                    .find(|x| ["calls", "graph", "mode"].iter().all(|k| x[k] == r[k]))
                    .ok_or("missing corresponding original")?;
                if let Some((x, y)) = r["j_error_ns"].as_f64().zip(b["j_error_ns"].as_f64()) {
                    r["original_error_ns"] = json!(y);
                    r["certificate_change_ns"] = json!(x - y);
                }
                r["original_projected"] = b["projected"].clone();
                r["original_spans"] = b["spans"].clone();
                if r["mode"] == "native" || b["projected"] == true {
                    for k in [
                        "projected",
                        "first_ns",
                        "j_error_ns",
                        "com_error_um",
                        "spin_error_rad_s",
                        "jump_vector_error_ns",
                        "spans",
                    ] {
                        check(
                            r[k] == b[k],
                            "certificate changed an already qualified result or ordinary control",
                        )?;
                    }
                }
            }
        }
        rows.extend(scored_rows);
    }
    rows.sort_by_key(|r| {
        (
            r["slot"].as_u64(),
            r["calls"].as_u64(),
            r["graph"].as_str().map(str::to_owned),
            r["mode"].as_str().map(str::to_owned),
        )
    });
    let mut groups = BTreeMap::<(bool, u64, String, String), Vec<&Value>>::new();
    for r in &rows {
        groups
            .entry((
                r["held"].as_bool().unwrap(),
                r["calls"].as_u64().unwrap(),
                text(&r["graph"])?.into(),
                text(&r["mode"])?.into(),
            ))
            .or_default()
            .push(r);
    }
    let mut cells = vec![];
    let event = rows.iter().any(|r| r["mode"] == "directed");
    let effect = if event {
        "directed_change_ns"
    } else if rows.iter().any(|r| r["certificate_change_ns"].is_number()) {
        "certificate_change_ns"
    } else {
        "delta_error_ns"
    };
    for ((held, calls, graph, mode), rs) in groups {
        let counts = rs
            .iter()
            .filter_map(|r| r[effect].as_f64())
            .fold([0; 3], |mut ct, d| {
                ct[if d < -1e-10 {
                    0
                } else if d > 1e-10 {
                    2
                } else {
                    1
                }] += 1;
                ct
            });
        let mut m = json!({"held":held,"graph":graph,"mode":mode,"calls":calls,"n":rs.len(),"projected":rs.iter().filter(|r|r["projected"]==true).count(),"with_sustained":rs.iter().filter(|r|r["spans"].as_u64().unwrap()>0).count(),"improved_equal_worse":counts});
        m["whole_from_impact"] =
            json!(rs.iter().filter(|r| r["whole_from_impact"] == true).count());
        m["whole_equal_event_count"] =
            json!(rs.iter().filter(|r| r["same_event_count"] == true).count());
        m["whole_unequal_event_count"] =
            json!(rs.iter().filter(|r| r["same_event_count"] == false).count());
        for key in [
            "peak_internal_time_error_ns",
            "peak_internal_jump_error_ns",
            "internal_events",
            "j_error_ns",
            "com_error_um",
            "spin_error_rad_s",
            "delta_error_ns",
            "certificate_change_ns",
            "directed_change_ns",
            "jump_vector_error_ns",
            "projected_time_error_ns",
            "jump_loss_error_j",
            "sustained_ns",
            "graph_difference_ns",
            "balance_ns",
        ] {
            m[key] = stats(rs.iter().filter_map(|r| r[key].as_f64()).collect());
        }
        cells.push(m);
    }
    let chosen = rows
        .iter()
        .filter(|r| {
            r["held"] == true
                && r["calls"] == 8
                && r["graph"] == "retained"
                && r["mode"] == if event { "directed" } else { "launch" }
                && r[effect].is_number()
        })
        .max_by(|a, b| {
            a[effect]
                .as_f64()
                .unwrap()
                .abs()
                .total_cmp(&b[effect].as_f64().unwrap().abs())
                .then(b["case"].as_u64().cmp(&a["case"].as_u64()))
        });
    let summary = json!({"complete_sources":slots.len(),"selection_metric":effect,"pending_slots":(0..50).filter(|v|!slots.contains(v)).collect::<Vec<_>>(),"selected":chosen,"cells":cells,"rows":rows});
    fs::write(out.join("policy.json"), serde_json::to_vec(&summary)?)?;
    plot(&cells, &out.join("impact-policy.svg"))?;
    Ok(())
}
fn plot(cells: &[Value], p: &Path) -> Result<()> {
    let area = SVGBackend::new(p, (1100, 420)).into_drawing_area();
    area.fill(&WHITE)?;
    let selected = cells
        .iter()
        .filter(|r| r["held"] == true && r["calls"] == 8 && r["graph"] == "retained")
        .collect::<Vec<_>>();
    let max = selected
        .iter()
        .filter_map(|r| r["j_error_ns"]["p95"].as_f64())
        .fold(0.1, f64::max);
    let mut c = ChartBuilder::on(&area)
        .caption(
            "Held-out returning-foot reaction · eight calls, eight passes",
            ("sans-serif", 24),
        )
        .margin(18)
        .x_label_area_size(50)
        .y_label_area_size(70)
        .build_cartesian_2d((0..5).into_segmented(), 0.0..max * 1.12)?;
    let event = cells.iter().any(|r| r["mode"] == "directed");
    let names = if event {
        [
            "Native",
            "Transfers",
            "Subdivided",
            "Directed",
            "Directed + retimed",
        ]
    } else {
        [
            "Native",
            "First point",
            "Retimed",
            "Bound contact",
            "Slip launch",
        ]
    };
    let ids = if event {
        [
            "native",
            "launch",
            "subdivided",
            "directed",
            "retimed_directed",
        ]
    } else {
        ["native", "point", "retimed", "bound", "launch"]
    };
    c.configure_mesh()
        .disable_x_mesh()
        .y_desc("p95 impulse-vector error (N·s)")
        .x_label_formatter(&|k| match k {
            SegmentValue::CenterOf(x) => names.get(*x as usize).copied().unwrap_or("").into(),
            _ => String::new(),
        })
        .draw()?;
    for (i, id) in ids.into_iter().enumerate() {
        if let Some(v) = selected
            .iter()
            .find(|r| r["mode"] == id)
            .and_then(|r| r["j_error_ns"]["p95"].as_f64())
        {
            c.draw_series(
                [Rectangle::new(
                    [
                        (SegmentValue::Exact(i as i32), 0.),
                        (SegmentValue::Exact(i as i32 + 1), v),
                    ],
                    RGBColor(65, 117, 153).filled(),
                )]
                .into_iter()
                .map(|mut b| {
                    b.set_margin(0, 0, 24, 24);
                    b
                }),
            )?;
        }
    }
    area.present()?;
    Ok(())
}
