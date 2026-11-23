// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare phase-shifted material sequences with their unshifted independent trajectories.
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
fn check(b: bool, s: &str) -> Result<()> {
    if b { Ok(()) } else { Err(s.into()) }
}
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn n(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|q| q.is_finite())
        .ok_or_else(|| "missing finite number".into())
}
fn a(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing array".into())
}
fn u(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing integer".into())
}
fn v(x: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(x.clone())?))
}
fn stats(rows: &[&Value], k: &str) -> Value {
    let mut x = rows
        .iter()
        .filter_map(|r| r[k].as_f64())
        .collect::<Vec<_>>();
    x.sort_by(f64::total_cmp);
    let q = |p: f64| (!x.is_empty()).then(|| x[((x.len() as f64 * p).ceil() as usize).max(1) - 1]);
    json!({"n":x.len(),"min":x.first(),"median":q(0.5),"p95":q(0.95),"max":x.last()})
}
fn figure(path: &Path, rows: &[Value], ids: &BTreeMap<u64, u64>) -> Result<()> {
    let root = SVGBackend::new(path, (1210, 1000)).into_drawing_area();
    root.fill(&WHITE)?;
    let (head, rest) = root.split_vertically(84);
    head.draw(&Text::new(
        "Physical transfers before the integer clock",
        (40, 34),
        ("sans-serif", 27).into_font().style(FontStyle::Bold),
    ))?;
    head.draw(&Text::new("64 shifts of each incoming trajectory   •   teal: verified single skew site   •   orange: another physical impact   •   red: incoming geometry alone would miss it", (40,66),("sans-serif",15)))?;
    let mut chart = ChartBuilder::on(&rest)
        .margin(15)
        .x_label_area_size(55)
        .y_label_area_size(112)
        .build_cartesian_2d(0..64u32, 0..50u32)?;
    chart
        .configure_mesh()
        .disable_mesh()
        .x_labels(9)
        .y_labels(12)
        .x_desc("First impact phase within its final nanosecond (% of native time bin)")
        .y_desc("Prespecified donor slot")
        .x_label_formatter(&|x| format!("{:.1}", f64::from(*x) * 100. / 64.))
        .draw()?;
    for r in rows {
        let Some(k) = r["phase"].as_u64() else {
            continue;
        };
        let s = u(&r["slot"])?;
        let ev = u(&r["extra_impacts"])?;
        let color = if !r["complete"].as_bool().unwrap() {
            RGBColor(91, 77, 107)
        } else if ev > 0 && r["edge_geometry"] == true {
            RGBColor(172, 45, 51)
        } else if ev > 0 {
            RGBColor(232, 131, 52)
        } else if r["skew_bound"] == true {
            RGBColor(29, 119, 125)
        } else {
            RGBColor(219, 226, 227)
        };
        chart.draw_series(std::iter::once(Rectangle::new(
            [(k as u32, s as u32), (k as u32 + 1, s as u32 + 1)],
            color.filled(),
        )))?;
    }
    check(ids.len() == 50, "wrong figure size")?;
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    check(args.len() == 2, "secondary_quantization STUDY REPORT")?;
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    let studies = dir.parent().ok_or("missing studies")?;
    let reference =
        read(&studies.join("multifidelity-secondary-material-response/report/material.json"))?;
    let mut rows = vec![];
    let mut ids = BTreeMap::new();
    let mut widest = 0_f64;
    for file in fs::read_dir(dir.join("cases"))? {
        let file = file?.path();
        let data = read(&file)?;
        let id = u(&data["case"])?;
        let slot = u(&data["slot"])?;
        let donor = a(&reference["cases"])?
            .iter()
            .find(|r| r["case"] == id)
            .ok_or("missing reference donor")?;
        check(
            ids.insert(slot, id).is_none()
                && donor["held"] == data["held"]
                && donor["cut_tick"] == data["cut_tick"],
            "source mismatch",
        )?;
        let mut phases = BTreeSet::new();
        let mat = &donor["material"];
        let origin = n(&mat["onset_s"])? * 1e9;
        for r in a(&data["rows"])? {
            let phase = r["phase"].as_u64().unwrap_or(64);
            let shift = n(&r["shift_s"])? * 1e9;
            let onset = n(&r["onset_ns"])?;
            check(
                phase <= 64
                    && phases.insert(phase)
                    && (phase == 64 || (onset.fract() - (phase as f64 + 0.5) / 64.).abs() < 1e-4)
                    && (onset + shift - origin).abs() < 1e-4,
                "phase translation mismatch",
            )?;
            let j = v(&r["initial_impulse_ns"])?;
            check(
                r["feature"] == mat["feature"] && j.distance(v(&mat["impulse_ns"])?) < 1e-10,
                "different physical first impact",
            )?;
            let width = u(&r["upper_ns"])? as f64 - onset;
            check(
                (0.0..=1.001).contains(&width),
                "invalid native time bracket",
            )?;
            let complete = r["both_meshes_agree"] == true;
            let observed = a(&r["events"])?;
            let refs = a(&donor["boundary_events"])?;
            let expected = refs
                .iter()
                .take_while(|e| e["time_s"].as_f64().unwrap() * 1e9 < origin + width)
                .collect::<Vec<_>>();
            if complete {
                check(
                    observed.len() == expected.len(),
                    "physical prefix missed a transfer",
                )?;
                for (a, b) in observed.iter().zip(expected) {
                    let dev = (n(&a["time_s"])? * 1e9 + shift - n(&b["time_s"])? * 1e9).abs();
                    widest = widest.max(dev);
                    check(
                        dev < 1e-4 && v(&a["impulse_ns"])?.distance(v(&b["impulse_ns"])?) < 1e-7,
                        "material response depends on translated start",
                    )?;
                }
            }
            let bound = r["single_outgoing_skew"]
                .as_bool()
                .ok_or("missing outgoing qualification")?;
            let geometry = r["edge_certified"] == true;
            check(
                !bound || (geometry && complete && observed.is_empty()),
                "skew bound admitted another impulse",
            )?;
            let omitted = if complete {
                let e = v(&r["end_impulse_ns"])?.distance(j);
                check(
                    (e - n(&r["first_only_discrepancy_ns"])?).abs() < 1e-13,
                    "vector reaction mismatch",
                )?;
                Some(e)
            } else {
                None
            };
            let mut record = json!({"case":id,"slot":slot,"held":donor["held"],"phase":r["phase"],"feature":r["feature"],"complete":complete,"width_after_ns":width,"extra_impacts":observed.len(),"edge_geometry":geometry,"skew_bound":bound,"geometry_only_false_single":geometry && !observed.is_empty(),"first_only_discrepancy_ns":omitted});
            if let Some(staged) = r.get("staged") {
                record["staged"] = json!(!staged.is_null());
                if !staged.is_null() {
                    check(
                        complete && staged["feature"] == r["feature"],
                        "staged an unresolved or wrong feature",
                    )?;
                    let ts = n(&staged["first_s"])? * 1e9;
                    let bracket = a(&staged["bracket_s"])?;
                    check(
                        bracket.len() == 2
                            && n(&bracket[0])? * 1e9 <= onset
                            && n(&bracket[1])? * 1e9 >= onset,
                        "physical root outside staged bracket",
                    )?;
                    let later = a(&staged["events"])?;
                    check(
                        later.len() == observed.len(),
                        "staged wrong number of reactions",
                    )?;
                    let mut te = (ts - onset).abs();
                    let mut je = v(&staged["first_impulse_ns"])?.distance(j);
                    for (x, y) in later.iter().zip(observed) {
                        te = te.max((n(&x["time_s"])? - n(&y["time_s"])?).abs() * 1e9);
                        je = je.max(v(&x["j_ns"])?.distance(v(&y["impulse_ns"])?));
                    }
                    let total = v(&staged["total_ns"])?.distance(v(&r["end_impulse_ns"])?);
                    let com = v(&staged["com_m"])?.distance(v(&r["end_reference_com_m"])?);
                    let spin =
                        v(&staged["angular_rad_s"])?.distance(v(&r["end_reference_spin_rad_s"])?);
                    check(
                        te < 1e-4 && je < 1e-7 && total < 1e-7 && com < 1e-9 && spin < 1e-6,
                        "staged path disagreed with independent mechanics",
                    )?;
                    record["event_time_error_ns"] = json!(te);
                    record["event_vector_error_ns"] = json!(je);
                    record["net_vector_error_ns"] = json!(total);
                    record["endpoint_com_error_m"] = json!(com);
                    record["endpoint_spin_error_rad_s"] = json!(spin);
                }
            }
            rows.push(record);
        }
        check(phases.len() == 65, "incomplete phase sweep")?;
    }
    check(ids.len() == 50 && rows.len() == 3250, "incomplete donors")?;
    let mut groups = vec![];
    for shifted in [false, true] {
        for held in [false, true] {
            let g = rows
                .iter()
                .filter(|r| r["phase"].is_number() == shifted && r["held"] == held)
                .collect::<Vec<_>>();
            let count = |k: &str| g.iter().filter(|r| r[k] == true).count();
            let extra = g
                .iter()
                .copied()
                .filter(|r| r["extra_impacts"].as_u64().unwrap() > 0)
                .collect::<Vec<_>>();
            groups.push(json!({"shifted":shifted,"held":held,"n":g.len(),"complete":count("complete"),"additional_impact":extra.len(),"geometric_edges":count("edge_geometry"),"geometry_only_false_single":count("geometry_only_false_single"),"material_qualified_edges":count("skew_bound"),"staged":count("staged"),"staged_time_error_ns":stats(&g,"event_time_error_ns"),"staged_impulse_error_ns":stats(&g,"net_vector_error_ns"),"staged_com_error_m":stats(&g,"endpoint_com_error_m"),"staged_spin_error_rad_s":stats(&g,"endpoint_spin_error_rad_s"),"first_only_extra_error_ns":stats(&extra,"first_only_discrepancy_ns")}));
        }
    }
    let best = rows
        .iter()
        .filter(|r| r["held"] == true && r["phase"].is_number() && r["complete"] == true)
        .max_by(|a, b| {
            a["first_only_discrepancy_ns"]
                .as_f64()
                .unwrap()
                .total_cmp(&b["first_only_discrepancy_ns"].as_f64().unwrap())
                .then_with(|| b["case"].as_u64().cmp(&a["case"].as_u64()))
                .then_with(|| b["phase"].as_u64().cmp(&a["phase"].as_u64()))
        });
    fs::create_dir_all(out)?;
    figure(&out.join("phase-transfers.svg"), &rows, &ids)?;
    fs::write(
        out.join("bracket.json"),
        serde_json::to_vec_pretty(
            &json!({"sources":ids.len(),"groups":groups,"selected":best,"largest_translated_reference_event_difference_ns":widest,"rows":rows}),
        )?,
    )?;
    Ok(())
}
