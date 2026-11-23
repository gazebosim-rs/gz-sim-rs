// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read matched native transport campaigns and the isolated receipt hasher measurements.
use super::{Comparison, Gauge, events, median, num, read, sha};
use crate::Result;
use serde_json::Value;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
fn timed(
    name: &str,
    unit: &str,
    values: [f64; 2],
    samples: u64,
    window: [u64; 3],
) -> Result<Gauge> {
    let mut g = Gauge::new(name, unit, values, samples)?;
    g.window = Some(window);
    Ok(g)
}
pub fn digest(root: &Path, layout: &str, trace: &Path) -> Result<Comparison> {
    let mut evidence = BTreeMap::new();
    for name in [
        "campaign-8-11/pairs.jsonl",
        "campaign-8-11/summary.json",
        "micro-8.json",
    ] {
        evidence.insert(name.to_owned(), sha(&root.join(name))?);
    }
    let summary = read(&root.join("campaign-8-11/summary.json"))?;
    let micro = read(&root.join("micro-8.json"))?;
    let wire = read(
        &trace
            .parent()
            .ok_or("missing native qualification")?
            .join("packed.json"),
    )?;
    let qualified: Vec<u64> = wire["wire"]["transfers"]
        .as_array()
        .ok_or("missing qualified transfers")?
        .iter()
        .map(|g| g["ids"][0].as_u64().ok_or("missing member"))
        .collect::<std::result::Result<_, _>>()?;
    let cases = ["single_to_split", "loaded_to_active"];
    let mut rows: [[Vec<f64>; 2]; 2] = Default::default();
    let mut expected = None;
    for line in BufReader::new(File::open(root.join("campaign-8-11/pairs.jsonl"))?).lines() {
        let row: Value = serde_json::from_str(&line?)?;
        if row["wire"] != "packed" || row["layout"] != layout {
            continue;
        }
        let Some(i) = cases.iter().position(|&c| row["case"] == c) else {
            continue;
        };
        let digest = row["physics_sha256"].as_str().ok_or("missing trajectory")?;
        if expected.as_ref().is_some_and(|d| d != digest) {
            return Err("matched campaign has inconsistent physical digest".into());
        }
        expected = Some(digest.to_owned());
        if i == 0 {
            let moves = row["movements"]
                .as_array()
                .ok_or("missing trial movements")?;
            if row["barrier"] != 75
                || moves.len() != qualified.len()
                || moves
                    .iter()
                    .zip(&qualified)
                    .any(|(m, &id)| m["component"] != id || m["from"] != 0 || m["to"] != 1)
            {
                return Err("digest trial transfer differs from recorded physical run".into());
            }
        }
        rows[i][0].push(num(&row, "baseline_us")?);
        rows[i][1].push(num(&row, "candidate_us")?);
    }
    let mut gauges = vec![];
    for (i, times) in rows.iter_mut().enumerate() {
        let case = cases[i];
        let row = &summary["packed"][layout][case];
        let n = times[0].len() as u64;
        if n == 0 || row["pairs"] != n {
            return Err("missing packed digest trial pairs".into());
        }
        for (j, column) in ["median", "p95"].into_iter().enumerate() {
            let expected = times.each_mut().map(|t| {
                let med = median(t);
                if j == 0 {
                    med
                } else {
                    t[(t.len() * 95).div_ceil(100) - 1]
                }
            });
            let values = [
                num(row, &format!("baseline_{column}_us"))?,
                num(row, &format!("candidate_{column}_us"))?,
            ];
            if expected
                .iter()
                .zip(values)
                .any(|(a, b)| (a - b).abs() > 1e-8)
            {
                return Err("handoff quantile differs from raw pairs".into());
            }
            gauges.push(timed(
                &format!("packed_{case}_{column}"),
                "us",
                values,
                n,
                [0, 175, (2 * i + j) as u64],
            )?);
        }
    }
    let configs = [&micro["loaded"]["0"], &micro["loaded"]["8192"]];
    let n = configs[0]["samples"]
        .as_u64()
        .ok_or("missing isolated samples")?;
    if configs[1]["samples"] != n
        || configs[0]["body_bytes"] != configs[1]["body_bytes"]
        || configs[0]["native_bytes"] != configs[1]["native_bytes"]
    {
        return Err("different loaded digest inputs".into());
    }
    for (i, (key, name, unit)) in [
        ("calls", "loaded_sha_update_calls", "calls"),
        ("median_us", "isolated_loaded_digest_median", "us"),
        ("p95_us", "isolated_loaded_digest_p95", "us"),
    ]
    .into_iter()
    .enumerate()
    {
        gauges.push(timed(
            name,
            unit,
            [num(configs[0], key)?, num(configs[1], key)?],
            n,
            [175, 351, i as u64],
        )?);
    }
    Ok(Comparison {
        arms: ["direct", "buffered_8192"],
        gauges,
        events: events(&[
            (75, "qualified_four_group_migration_ack_retry"),
            (120, "qualified_launch_ack_retry"),
            (179, "qualified_cargo_owner_replacement"),
            (185, "qualified_whole_pool_restart"),
        ]),
        expected_physics: expected,
        evidence,
    })
}

pub fn sparse(root: &Path, layout: &str) -> Result<(Comparison, [super::Owners; 2])> {
    let dir = format!("{layout}-8");
    let name = format!("owners-{layout}.json");
    let path = root.join(&name);
    let owners = super::Owners::read(&path, layout, "", 8)?;
    if read(&path)?["width"] != 8 || owners.owner_changes.len() != 1 {
        return Err("unexpected eight-worker fixture ownership".into());
    }
    let mut evidence = BTreeMap::from([(name, sha(&path)?)]);
    for name in [format!("{dir}/pairs.jsonl"), format!("{dir}/result.json")] {
        evidence.insert(name.clone(), sha(&root.join(name))?);
    }
    let summary = read(&root.join(&dir).join("result.json"))?;
    let conditions = summary["by_tick"]
        .as_array()
        .ok_or("missing sparse conditions")?;
    let n = summary["rounds"].as_u64().ok_or("missing rounds")?;
    if n == 0 || conditions.len() != 6 || summary["width"] != 8 || summary["layout"] != layout {
        return Err("invalid sparse trial summary".into());
    }
    let mut costs = vec![[Vec::<f64>::new(), vec![]]; conditions.len()];
    for line in BufReader::new(File::open(root.join(&dir).join("pairs.jsonl"))?).lines() {
        let pair: Value = serde_json::from_str(&line?)?;
        for (arm, key) in ["reference", "candidate"].into_iter().enumerate() {
            let trial = &pair[key];
            let hits = trial["confirmations"]
                .as_array()
                .ok_or("missing confirmations")?;
            if trial["width"] != 8
                || trial["layout"] != layout
                || trial["digest"] != owners.trajectory_sha256
                || hits.len() != conditions.len()
            {
                return Err("sparse trial disagrees with physical scene".into());
            }
            for (i, (hit, row)) in hits.iter().zip(conditions).enumerate() {
                if ["tick", "missing"].iter().any(|&k| hit[k] != row[k])
                    || ["requests", "request_bytes", "reply_bytes"]
                        .iter()
                        .any(|&k| hit[k] != row[k][arm])
                {
                    return Err("confirmation conditions or traffic disagree with summary".into());
                }
                costs[i][arm].push(num(hit, "retry_us")?)
            }
        }
    }
    let mut gauges = vec![];
    let mut ev = vec![];
    for (i, row) in conditions.iter().enumerate() {
        let tick = row["tick"].as_u64().ok_or("missing confirmation tick")?;
        let until = conditions
            .get(i + 1)
            .and_then(|r| r["tick"].as_u64())
            .unwrap_or(351);
        let missing: Vec<u8> = serde_json::from_value(row["missing"].clone())?;
        if row["requests"] != serde_json::json!([8, missing.len()]) {
            return Err("sparse confirm targeted the wrong request count".into());
        }
        let ms = missing
            .iter()
            .map(u8::to_string)
            .collect::<Vec<_>>()
            .join("_");
        ev.push((tick, format!("lost_final_step_replies_from_workers_{ms}")));
        for (j, (key, unit)) in [
            ("requests", "calls"),
            ("request_bytes", "bytes"),
            ("reply_bytes", "bytes"),
            ("median_retry_us", "us"),
        ]
        .into_iter()
        .enumerate()
        {
            let values: [f64; 2] = serde_json::from_value(row[key].clone())?;
            if key == "median_retry_us"
                && costs[i]
                    .iter_mut()
                    .zip(values)
                    .any(|(v, x)| v.len() != n as usize || (median(v) - x).abs() > 1e-8)
            {
                return Err("retry median differs from individual campaigns".into());
            }
            gauges.push(timed(
                &format!("tick_{tick}_{key}"),
                unit,
                values,
                n,
                [tick, until, j as u64],
            )?);
        }
    }
    Ok((
        Comparison {
            arms: ["all_workers", "missing_workers"],
            gauges,
            events: ev,
            expected_physics: Some(owners.trajectory_sha256.clone()),
            evidence,
        },
        [owners.clone(), owners],
    ))
}
