// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the independent certification and coalescing of a post-impact tail
//! in a sole-tree local world. Every run starts on a retained pre-landing graph.
use super::whole::{Method, first};
use super::*;
fn collect(f: Fixture, directed: bool) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (k, c) in cuts.into_iter().enumerate() {
        let Some(c) = c else { continue };
        let mut paths = vec![];
        for n in [2, 8, 64] {
            for (name, m) in [("fixed", Method::Subdivided), ("compact", Method::Compact)] {
                let (mut r, _) = first(f, &c, m.directed(directed), n, n, c.tick + 1)?;
                r["profile"] = json!(format!("{name}{n}"));
                paths.push(r);
            }
        }
        trials.push(json!({"stroke":k+1,"cut":snapshot(&c),"paths":paths}));
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(dir: &Path, start: usize, count: usize, directed: bool) -> Result<()> {
    survey::write_cases(
        dir,
        start,
        count,
        if directed {
            "directed-primary-tail-v1"
        } else {
            "certified-primary-tail-v1"
        },
        |f| collect(f, directed),
    )
}

/// Aggregate measured native bins at the clocks visited by both schedules.
fn common(bins: &[Value], clocks: &[u64]) -> Result<Vec<Value>> {
    if bins.first().map(|s| &s["time_ns"]) != Some(&json!(clocks[0])) {
        return Err("different origins".into());
    }
    let mut rows = vec![bins[0].clone()];
    let mut i = 1;
    for (tick, &ns) in clocks.iter().enumerate().skip(1) {
        let mut calls = 0;
        let mut normal = 0.;
        let mut j = DVec3::ZERO;
        while let Some(a) = bins.get(i) {
            let at = a["time_ns"].as_u64().ok_or("no native clock")?;
            if at > ns {
                break;
            }
            normal += a["normal_impulse_ns"]
                .as_f64()
                .ok_or("no measured reaction")?;
            j += DVec3::from_array(serde_json::from_value(a["world_impulse_ns"].clone())?);
            calls += a["native_steps"].as_u64().ok_or("no native count")?;
            i += 1;
        }
        let mut row = bins[i - 1].clone();
        if row["time_ns"] != ns || calls == 0 {
            return Err("required common clock was not executed".into());
        }
        row["tick"] = json!(tick);
        row["native_steps"] = json!(calls);
        row["normal_impulse_ns"] = json!(normal);
        row["world_impulse_ns"] = json!(j.to_array());
        rows.push(row);
    }
    if i != bins.len() {
        return Err("unaccounted native suffix".into());
    }
    Ok(rows)
}
pub(crate) fn replay(dir: &Path, k: usize, stroke: usize) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid compact donor".into());
    }
    let d: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let directed = d["format"] == "directed-primary-tail-v1";
    let t = d["result"]["trials"]
        .as_array()
        .ok_or("missing donors")?
        .iter()
        .find(|q| q["stroke"] == stroke)
        .ok_or("missing stroke")?;
    let f = survey::fixture(k);
    let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing primary")?;
    let geo =
        super::super::super::finite::geometry(super::super::super::finite::kinematics(f, &c)?.0)?;
    let folder = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    let inside = folder.join("interval");
    for p in [&folder, &inside] {
        fs::create_dir_all(p.join("traces"))?;
    }
    let save = |dir: &Path,
                name: &str,
                label: &str,
                frames: Vec<Value>,
                analytical: bool|
     -> Result<Value> {
        let file = format!("traces/{name}.json");
        write_json(
            &dir.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        Ok(json!({"label":name,"name":label,"file":file,"analytical":analytical}))
    };
    let mut lanes = vec![];
    let mut window = vec![];
    let mut measured = vec![];
    let mut initial = Value::Null;
    for (i, id, title, n, m) in [
        (2, "fixed8", "Eight native calls", 8, Method::Subdivided),
        (
            3,
            "compact8",
            "Coalesce a certified post-impact interval",
            8,
            Method::Compact,
        ),
        (
            9,
            "native8192",
            "Continuous 8,192 native calls",
            8192,
            Method::Native,
        ),
    ] {
        let mut samples = vec![];
        let (mut r, path) = super::whole::observed(
            f,
            &c,
            m.directed(directed),
            n,
            if i == 9 { 8192 } else { 512 },
            c.tick + 26,
            (i != 9).then_some(&mut samples),
        )?;
        if i != 9 {
            r["profile"] = json!(id);
            if serde_json::from_slice::<Value>(&serde_json::to_vec(&r)?)? != t["paths"][i] {
                return Err("replay first interval changed".into());
            }
            if initial.is_null() {
                initial = samples[0].clone()
            } else if initial != samples[0] {
                return Err("different pre-primary states".into());
            }
            measured.push((id, title, samples));
        }
        let mut origin = initial.clone();
        origin["tick"] = json!(c.tick);
        let mut frames = vec![origin];
        frames.extend(path.iter().map(Tick::as_json));
        lanes.push(save(&folder, id, title, frames, false)?);
    }
    let clocks = measured[1]
        .2
        .iter()
        .map(|s| s["time_ns"].as_u64().ok_or("no native time"))
        .collect::<std::result::Result<Vec<_>, _>>()?;
    for (id, title, rows) in measured {
        window.push(save(&inside, id, title, common(&rows, &clocks)?, false)?);
    }
    let exact = super::whole::physical_samples(f, &c, &geo, &clocks, initial)?;
    window.push(save(
        &inside,
        "mechanics",
        "Piecewise rigid integration through the original clock",
        exact,
        true,
    )?);
    write_json(
        &inside.join("comparison.json"),
        &json!({"title":"Certified native work at the common executed clocks","interval_samples":true,"public_dt_ns":500000,"sampled_ticks":true,"pair_totals_only":true,"clock_origin_ns":c.tick*500000,"lanes":window}),
    )?;
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"After coalescing the primary bearing interval","pair_totals_only":true,"lanes":lanes}),
    )
}
