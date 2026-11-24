// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join regular Bevy/SDF landing receipts to independently integrated material trajectories.
use glam::DVec3;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn arr(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "expected array".into())
}
fn n(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "expected finite number".into())
}
fn u(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "expected count".into())
}
fn v(x: &Value) -> Result<DVec3> {
    let d = DVec3::from_array(serde_json::from_value(x.clone())?);
    if d.is_finite() {
        Ok(d)
    } else {
        Err("nonfinite vector".into())
    }
}
fn ok(c: bool, msg: &str) -> Result<()> {
    if c { Ok(()) } else { Err(msg.into()) }
}
fn stats(rows: &[&Value], key: &str) -> Value {
    let mut xs = rows
        .iter()
        .filter_map(|r| r[key].as_f64())
        .collect::<Vec<_>>();
    xs.sort_by(f64::total_cmp);
    let quant =
        |p: f64| (!xs.is_empty()).then(|| xs[((p * xs.len() as f64).ceil() as usize).max(1) - 1]);
    json!({"n":xs.len(),"median":quant(0.5),"p95":quant(0.95),"max":xs.last()})
}
fn main() -> Result<()> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    ok(args.len() == 2, "secondary_authored STUDY REPORT")?;
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    let studies = dir.parent().ok_or("no studies")?;
    let source = read(&dir.join("study.json"))?;
    let reference =
        read(&studies.join("multifidelity-secondary-material-response/report/material.json"))?;
    let mut rows = vec![];
    let mut slots = BTreeSet::new();
    for c in arr(&source["cases"])? {
        let id = u(&c["case"])?;
        let slot = u(&c["slot"])?;
        ok(slots.insert(slot), "duplicate donor")?;
        let physical = arr(&reference["cases"])?
            .iter()
            .find(|r| r["case"] == id)
            .ok_or("no physical donor")?;
        let raw = read(&studies.join(format!(
            "multifidelity-secondary-order-field/cases/case-{id:03}.json"
        )))?;
        ok(
            raw["incoming"]["cut_tick"] == c["cut_tick"]
                && raw["slot"] == slot
                && physical["cut_tick"] == c["cut_tick"]
                && physical["held"] == c["held"],
            "source coordinate mismatch",
        )?;
        let incoming = &raw["incoming"]["motion"];
        let links = arr(&incoming["links"])?;
        let inlet = v(&links[0]["velocity"])? + v(&links[1]["velocity"])?;
        let free = 2. * 0.0005 * v(&incoming["gravity"])?;
        let boundary = &physical["boundary_model"];
        let valid = physical["qualified"] == true;
        let typed = &physical["material"];
        let mut modes = BTreeSet::new();
        for r in arr(&c["rows"])? {
            let mode = r["lane"].as_str().ok_or("missing lane")?;
            ok(modes.insert(mode), "duplicate SDF treatment")?;
            let obs = &r["refined"];
            let first = if obs.is_null() {
                None
            } else {
                arr(&obs["impacts"])?.first()
            };
            let calls = u(&r["calls"])?;
            ok([1, 8].contains(&calls), "invalid public solve count")?;
            let outgoing = v(&r["foot"]["velocity"])? + v(&r["upper"]["velocity"])?;
            let from_momentum = outgoing - inlet - free;
            let receipt = (!obs.is_null())
                .then(|| v(&obs["world_impulse_ns"]))
                .transpose()?;
            let jump = first.map(|j| v(&j["j_ns"])).transpose()?;
            let jref = v(&typed["impulse_ns"])?;
            let onset = n(&typed["onset_s"])? * 1e9;
            let lag = first
                .map(|j| n(&j["time_ns"]).map(|t| t - onset))
                .transpose()?;
            if let Some(dt) = lag {
                ok(
                    (-0.0001..=1.0001).contains(&dt),
                    "first impulse outside independent ns bracket",
                )?;
            }
            let jerr = jump.map(|j| j.distance(jref));
            ok(
                jerr.is_none_or(|e| e < 1e-6),
                "SDF jump disagrees with physical law",
            )?;
            let center = (v(&r["foot"]["position"])? + v(&r["upper"]["position"])?) * 0.5;
            let end = valid
                .then(|| v(&boundary["total_impulse_ns"]))
                .transpose()?;
            let com = valid
                .then(|| v(&boundary["com_m"]).map(|x| 1e6 * x.distance(center)))
                .transpose()?;
            let spin = valid
                .then(|| {
                    v(&r["foot"]["angular_velocity"])
                        .and_then(|x| Ok(x.distance(v(&boundary["spin_rad_s"])?)))
                })
                .transpose()?;
            let observed_events = obs["internal_jumps"].as_array();
            let independently_observed = physical["boundary_events"].as_array();
            let n_obs = observed_events.map(|a| a.len());
            let complete = valid
                && observed_events.is_some_and(|xs| {
                    !xs.is_empty() && independently_observed.is_some_and(|ys| ys.len() == xs.len())
                });
            let mut dt: f64 = 0.;
            let mut dj: f64 = 0.;
            if complete {
                for (a, b) in observed_events
                    .unwrap()
                    .iter()
                    .zip(independently_observed.unwrap())
                {
                    dt = dt.max((n(&a["time_ns"])? - n(&b["time_s"])? * 1e9).abs());
                    dj = dj.max(v(&a["j_ns"])?.distance(v(&b["impulse_ns"])?));
                }
            }
            rows.push(json!({"case":id,"slot":slot,"held":c["held"],"lane":mode,"calls":calls,"geometry":typed["feature"],
            "physical_boundary":physical["boundary"],"projected":first.is_some(),"time_error_ns":lag,"first_vector_error_ns":jerr,
            "pair_vector_error_ns":end.zip(receipt).map(|(x,y)|x.distance(y)),
            "momentum_vector_error_ns":end.map(|j|j.distance(from_momentum)),
            "pair_momentum_difference_ns":receipt.map(|j|j.distance(from_momentum)),
            "com_error_um":com,"spin_error_rad_s":spin,"registered_internal_events":n_obs,
            "equal_nonempty_internal_series":complete,"matched_event_time_error_ns":complete.then_some(dt),"matched_event_vector_error_ns":complete.then_some(dj)}));
        }
        ok(modes.len() == 3, "missing SDF control")?;
    }
    ok(
        slots == (0..50).collect() && rows.len() == 150,
        "incomplete SDF campaign",
    )?;
    let mut groups = BTreeMap::<(bool, &str), Vec<&Value>>::new();
    for r in &rows {
        groups
            .entry((r["held"] == true, r["lane"].as_str().unwrap()))
            .or_default()
            .push(r);
    }
    let mut summaries = vec![];
    for ((held, lane), g) in groups {
        let mut summary = json!({"held":held,"lane":lane,"n":g.len(),"projected":g.iter().filter(|r|r["projected"]==true).count(),
            "finite_public_references":g.iter().filter(|r|r["physical_boundary"]=="complete").count(),
            "equal_nonempty_internal_series":g.iter().filter(|r|r["equal_nonempty_internal_series"]==true).count()});
        for key in [
            "time_error_ns",
            "first_vector_error_ns",
            "pair_vector_error_ns",
            "momentum_vector_error_ns",
            "pair_momentum_difference_ns",
            "com_error_um",
            "spin_error_rad_s",
            "matched_event_time_error_ns",
            "matched_event_vector_error_ns",
        ] {
            summary[key] = stats(&g, key);
        }
        summaries.push(summary);
    }
    fs::create_dir_all(out)?;
    fs::write(
        out.join("authored.json"),
        serde_json::to_vec_pretty(
            &json!({"sources":slots.len(),"rows":rows,"summaries":summaries}),
        )?,
    )?;
    Ok(())
}
