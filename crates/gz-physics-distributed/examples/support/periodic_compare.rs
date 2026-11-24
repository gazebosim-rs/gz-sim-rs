// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep complete paired process runs together when comparing scheduler policies or builds.
use super::{Result, Round, Trial, compare, median, p95, reaction, read};
use serde_json::{Value, json};
use std::{collections::BTreeMap, path::PathBuf};

struct Block {
    reference: Vec<Round>,
    candidate: Vec<Round>,
}

fn summarize(values: Vec<f64>) -> Option<Value> {
    if values.is_empty() {
        return None;
    }
    Some(json!({"records":values.len(),"median_us":median(values.clone()),"p95_us":p95(values)}))
}
fn change_intervals(blocks: &[Block], mode: &str) -> Value {
    // Resample whole matched process-run blocks, including the same six arm orders.
    const DRAWS: usize = 4096;
    let mut rng = fastrand::Rng::with_seed(674191);
    let changes: Vec<Vec<[f64; 2]>> = blocks
        .iter()
        .map(|block| {
            block
                .reference
                .iter()
                .zip(&block.candidate)
                .map(|(r, c)| {
                    let (r, c) = (&r[mode], &c[mode]);
                    [
                        100. * (c.full_us / r.full_us - 1.),
                        100. * (c.steady_p95_us / r.steady_p95_us - 1.),
                    ]
                })
                .collect()
        })
        .collect();
    let mut samples = [Vec::with_capacity(DRAWS), Vec::with_capacity(DRAWS)];
    for _ in 0..DRAWS {
        let mut trial = [Vec::new(), Vec::new()];
        for _ in 0..blocks.len() {
            for &[total, tail] in &changes[rng.usize(..blocks.len())] {
                trial[0].push(total);
                trial[1].push(tail);
            }
        }
        for (s, t) in samples.iter_mut().zip(trial) {
            s.push(median(t));
        }
    }
    let [total, steady] = samples.map(|mut s| {
        s.sort_by(f64::total_cmp);
        [s[DRAWS.div_ceil(40) - 1], s[(39 * DRAWS).div_ceil(40) - 1]]
    });
    json!({"draws":DRAWS,"full_change_pct":total,"steady_p95_change_pct":steady})
}
pub(super) fn compare_sets(paths: &[PathBuf]) -> Result<Value> {
    if paths.is_empty() || !paths.len().is_multiple_of(2) {
        return Err("expected reference/candidate pairs".into());
    }
    let mut blocks = Vec::new();
    for pair in paths.chunks_exact(2) {
        let (mut a, reference) = read(&pair[0])?;
        let (mut b, candidate) = read(&pair[1])?;
        a.as_object_mut().ok_or("bad proof")?.remove("policy");
        b.as_object_mut().ok_or("bad proof")?.remove("policy");
        if a != b
            || reference
                .iter()
                .zip(&candidate)
                .any(|(r, c)| r["fixed"].contact_transitions != c["fixed"].contact_transitions)
        {
            return Err("paired inputs or native trajectory proofs differ".into());
        }
        blocks.push(Block {
            reference,
            candidate,
        });
    }
    let mut modes = BTreeMap::new();
    for mode in ["fixed", "observe", "automatic"] {
        let pairs: Vec<_> = blocks
            .iter()
            .flat_map(|b| b.reference.iter().zip(&b.candidate))
            .map(|(r, c)| (&r[mode], &c[mode]))
            .collect();
        let sides = [
            pairs.iter().map(|&(r, _)| r).collect::<Vec<_>>(),
            pairs.iter().map(|&(_, c)| c).collect::<Vec<_>>(),
        ];
        let states=sides.iter().map(|v|->Result<Value>{
            let sum=|metric:fn(&Trial)->u64|v.iter().map(|t|metric(t)).sum::<u64>();
            Ok(json!({"starts":sum(|t|t.windows_started),"deferred":sum(|t|t.windows_deferred),"ineligible":sum(|t|t.windows_ineligible),
                "early_wakes":sum(|t|t.windows_woken),"moves":sum(|t|t.moves.len() as u64),
                "landing":summarize(v.iter().flat_map(|t|t.landing_us).collect()),
                "close_without_move":summarize(v.iter().flat_map(|t|t.closing_without_move_us.iter().copied()).collect()),
                "open_only":summarize(v.iter().flat_map(|t|t.opening_only_us.iter().copied()).collect()),
                "next_sample_after_contact_change":reaction(v.iter().copied())?}))
        }).collect::<Result<Vec<_>>>()?;
        modes.insert(mode,json!({"paired_runs":pairs.len(),"all_350_ticks":compare(&pairs,|t|t.full_us),
            "steady_tick_p95":compare(&pairs,|t|t.steady_p95_us),"process_blocks_95_percent":change_intervals(&blocks,mode),
            "reference":states[0],"candidate":states[1]}));
    }
    Ok(json!({"directory_pairs":paths.chunks_exact(2).collect::<Vec<_>>(),"modes":modes}))
}
