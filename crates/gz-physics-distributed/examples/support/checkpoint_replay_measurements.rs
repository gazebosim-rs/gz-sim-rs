// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate frozen checkpoint capture/read/restore pairs and the matching independent ownership trace.
use super::{Comparison, Gauge, Owners, median, read, sha};
use crate::Result;
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
#[derive(Deserialize)]
struct Trial {
    save_us: f64,
    compact_us: f64,
    open_us: f64,
    resume_us: f64,
    request_bytes: u64,
    reply_bytes: u64,
    requests: u64,
    archive_bytes: u64,
    archive_sha256: String,
    trajectory_sha256: String,
    pair_receipts: u64,
    floor_receipts: u64,
}
#[derive(Deserialize)]
struct Pair {
    round: usize,
    trial: [Trial; 2],
}
pub fn checkpoint(root: &Path, layout: &str) -> Result<(Comparison, [Owners; 2])> {
    let qualified = format!("qualification-{layout}");
    let source = root.join(&qualified).join("owners.json");
    let map = Owners::read(&source, layout, "checkpoint", 8)?;
    let q = read(&source)?;
    let bar = &q["barrier"];
    let archive = bar["archive_sha256"]
        .as_str()
        .ok_or("missing archive hash")?;
    let stored = bar["archive_bytes"]
        .as_u64()
        .ok_or("missing persisted size")?;
    let capture = bar["reply_bytes"]
        .as_u64()
        .ok_or("missing warm communication bytes")?;
    if q["width"] != 8
        || bar["tick"] != 75
        || bar["before"] != bar["after"]
        || bar["before"]["positive_pairs"] != 54
        || bar["after"]["positive_floors"] != 8
        || map.owner_changes.len() != 2
        || map.owner_changes[1].tick != 70
        || q["member_ids"].as_array().is_none_or(|v| v.len() != 25)
    {
        return Err("incomplete warm barrier qualification".into());
    }
    let path = root.join(&qualified).join("tick-75.gzpool");
    if sha(&path)? != archive || std::fs::metadata(&path)?.len() != stored {
        return Err("qualified native image differs from its receipt".into());
    }
    let mut evidence = BTreeMap::from([
        (format!("{qualified}/owners.json"), sha(&source)?),
        (format!("{qualified}/tick-75.gzpool"), archive.to_owned()),
    ]);
    let mut times: [[Vec<f64>; 2]; 4] = Default::default();
    let mut replies = [None; 2];
    let mut n = 0;
    for lane in ["core", "alternate"] {
        let dir = format!("combined-{lane}-{layout}-8");
        let sum = read(&root.join(&dir).join("summary.json"))?;
        let expected = sum["pairs"].as_u64().ok_or("incomplete matched campaign")? as usize;
        if sum["layout"] != layout
            || sum["width"] != 8
            || sum["archive_sha256"] != archive
            || sum["trajectory_sha256"] != map.trajectory_sha256
            || sum["archive_bytes"] != stored
            || expected == 0
        {
            return Err("checkpoint summary is for another run".into());
        }
        for file in ["summary.json", "pairs.jsonl"] {
            evidence.insert(format!("{dir}/{file}"), sha(&root.join(&dir).join(file))?);
        }
        let mut count = 0;
        for (i, line) in BufReader::new(File::open(root.join(&dir).join("pairs.jsonl"))?)
            .lines()
            .enumerate()
        {
            let pair: Pair = serde_json::from_str(&line?)?;
            if pair.round != i {
                return Err("missing paired checkpoint round".into());
            }
            for (j, t) in pair.trial.into_iter().enumerate() {
                if t.archive_sha256 != archive
                    || t.archive_bytes != stored
                    || t.trajectory_sha256 != map.trajectory_sha256
                    || t.pair_receipts != q["native_pair_receipts"].as_u64().unwrap()
                    || t.floor_receipts != q["native_floor_receipts"].as_u64().unwrap()
                    || t.requests != 8
                    || t.request_bytes != 272
                    || *replies[j].get_or_insert(t.reply_bytes) != t.reply_bytes
                    || j == 1 && t.reply_bytes != capture
                {
                    return Err("checkpoint wire, native state or saved bytes changed".into());
                }
                for (k, v) in [t.save_us, t.compact_us, t.open_us, t.resume_us]
                    .into_iter()
                    .enumerate()
                {
                    if !v.is_finite() || v <= 0. {
                        return Err("invalid checkpoint latency".into());
                    }
                    times[k][j].push(v);
                }
            }
            count += 1;
        }
        if count != expected {
            return Err("truncated checkpoint pairs".into());
        }
        n += count;
    }
    let calc = times.each_mut().map(|v| {
        v.each_mut().map(|s| {
            let m = median(s);
            [m, s[(s.len() * 95).div_ceil(100) - 1]]
        })
    });
    let mut gauges = vec![];
    for (name, unit, val, start, end, row) in [
        (
            "durable_save_median",
            "us",
            calc[0].map(|x| x[0]),
            0,
            351,
            0,
        ),
        ("durable_save_p95", "us", calc[0].map(|x| x[1]), 0, 351, 1),
        (
            "capture_replies",
            "bytes",
            replies.map(|v| v.unwrap() as f64),
            0,
            351,
            2,
        ),
        (
            "native_compaction_median",
            "us",
            calc[1].map(|x| x[0]),
            0,
            75,
            3,
        ),
        ("same_immutable_file", "bytes", [stored as f64; 2], 0, 75, 4),
        (
            "validated_file_open_median",
            "us",
            calc[2].map(|x| x[0]),
            75,
            351,
            3,
        ),
        (
            "fresh_process_resume_median",
            "us",
            calc[3].map(|x| x[0]),
            75,
            351,
            4,
        ),
    ] {
        let mut g = Gauge::new(name, unit, val, n as u64)?;
        g.window = Some([start, end, row]);
        gauges.push(g);
    }
    let moved = q["member_ids"].as_array().unwrap();
    let first = moved[0].as_u64().unwrap() as usize - 1;
    let events = vec![
        (0, "eight_actual_native_workers".to_owned()),
        (
            70,
            format!(
                "handoff_25_loaded_members_{}_through_{}_w{}_to_w{}",
                moved[0],
                moved.last().unwrap(),
                map.owner_changes[0].owners[first],
                map.owner_changes[1].owners[first]
            ),
        ),
        (
            75,
            format!(
                "saved_compacted_opened_resumed_same_54_loaded_pairs_8_floors_sha_{}",
                &archive[..12]
            ),
        ),
        (
            76,
            "first_post_resume_native_step_contact_continuity_verified".to_owned(),
        ),
    ];
    Ok((
        Comparison {
            arms: ["prior", "combined_packed"],
            gauges,
            events,
            expected_physics: Some(map.trajectory_sha256.clone()),
            evidence,
        },
        [map.clone(), map],
    ))
}
