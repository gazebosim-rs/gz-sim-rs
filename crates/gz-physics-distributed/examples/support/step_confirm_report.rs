// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Validate identical stopped-worker faults and the real process fenced by each grace.
use super::{Result, Times, read};
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs, path::Path};

pub(super) fn run(base: &Path, blocks: u16) -> Result<()> {
    let mut groups = BTreeMap::<String, Times>::new();
    let mut count = 0;
    for block in 0..blocks {
        for layout in ["adjacent", "separated"] {
            for mode in ["fixed", "automatic"] {
                for peer in [0, 2] {
                    for thaw in [Some(0), Some(100), Some(250), None] {
                        let tag = thaw
                            .map(|n| n.to_string())
                            .unwrap_or_else(|| "persist".into());
                        let (a, x) = read(
                            &base
                                .join(layout)
                                .join(format!("{block}-{peer}-{mode}-{tag}-1000")),
                        )?;
                        let (b, y) = read(
                            &base
                                .join(layout)
                                .join(format!("{block}-{peer}-{mode}-{tag}-150")),
                        )?;
                        for key in [
                            "layout",
                            "native_sha256",
                            "ticks_per_run",
                            "floors_per_run",
                            "pairs_per_run",
                            "rounds",
                            "peer",
                            "thaw_after_first_timeout_ms",
                        ] {
                            if a[key] != b[key] {
                                return Err(
                                    format!("different fault or physical proof in {key}").into()
                                );
                            }
                        }
                        if a["layout"] != layout
                            || a["peer"] != peer
                            || a["thaw_after_first_timeout_ms"] != json!(thaw)
                            || a["step_confirm_ms"] != 1000
                            || b["step_confirm_ms"] != 150
                            || x.len() != 1
                            || y.len() != 1
                        {
                            return Err("incorrect endpoint or run configuration".into());
                        }
                        for (arm, e) in [&x[0], &y[0]].into_iter().enumerate() {
                            let first = e.first_failure_us.ok_or("no failed initial solve")?;
                            let replace = thaw.is_none() || (arm == 1 && thaw == Some(250));
                            if !e.first_step_retry
                                || e.tick != 75
                                || e.positive_pairs_before != 54
                                || e.elapsed_us <= first
                                || !first.is_finite()
                                || !e.elapsed_us.is_finite()
                                || (e.windows_started != 0) != (mode == "automatic")
                                || e.before_pids
                                    .iter()
                                    .zip(e.after_pids)
                                    .enumerate()
                                    .any(|(i, (&old, new))| (old != new) != (replace && i == peer))
                            {
                                return Err(
                                    "physical retry, measured PID or timing violates schedule"
                                        .into(),
                                );
                            }
                        }
                        for key in [tag.clone(), format!("{layout}/{mode}/{peer}/{tag}")] {
                            groups.entry(key).or_default().add(&x[0], &y[0]);
                        }
                        count += 1;
                    }
                }
            }
        }
    }
    let mut public = vec![];
    for width in [3, 4, 8] {
        let dir = base.join(format!("public-{width}"));
        let proof: Value = serde_json::from_slice(&fs::read(dir.join("proof.json"))?)?;
        let rows = fs::read_to_string(dir.join("episodes.jsonl"))?;
        let mut categories = BTreeMap::new();
        for line in rows.lines() {
            let row: Value = serde_json::from_str(line)?;
            let pair = row["pids"]
                .as_array()
                .filter(|a| a.len() == 2)
                .ok_or("missing public PID arrays")?;
            let old = pair[0].as_array().ok_or("old PID list missing")?;
            let new = pair[1].as_array().ok_or("new PID list missing")?;
            let owner = row["owner"].as_u64().ok_or("missing stopped owner")? as usize;
            let thaw = row["thaw_from_stop_ms"].as_u64();
            if old.len() != width
                || new.len() != width
                || owner >= width
                || row["positive_before"].as_u64().unwrap_or(0) == 0
                || old.iter().zip(new).enumerate().any(|(i, (a, b))| {
                    (a != b) != ((thaw.is_none() || thaw == Some(1400)) && i == owner)
                })
            {
                return Err("public proof has invalid contact or replacement".into());
            }
            let key = format!(
                "{}/{}",
                row["source"].as_str().ok_or("missing entry")?,
                thaw.map(|t| t.to_string())
                    .unwrap_or_else(|| "persist".into())
            );
            *categories.entry(key).or_insert(0usize) += 1;
        }
        if categories.len() != 12
            || rows.lines().count()
                != 3 * proof["rounds"].as_u64().ok_or("missing rounds")? as usize
        {
            return Err("incomplete public fault schedules".into());
        }
        public.push(json!({"width":width,"proof":proof,"episodes":rows.lines().count(),"schedules":categories}));
    }
    let results: BTreeMap<_, _> = groups.into_iter().map(|(k, g)| (k, g.report())).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"matched_raw_pairs":count,"native_ticks_per_arm":350*count,"arm_grace_ms":[1000,150],"raw":results,"public":public})
        )?
    );
    Ok(())
}
