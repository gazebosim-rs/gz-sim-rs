// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render a physically qualified difference at floating-point precision without amplifying any motion.
use super::{Bodies, Frame, Rendered, Result, Transform, View, capture, floor, read};
use serde_json::json;
use std::{collections::BTreeSet, path::Path};
fn impulse(f: &Frame, pair: [u64; 2]) -> Option<f64> {
    f.pairs
        .iter()
        .find(|p| [p.a, p.b] == pair)
        .map(|p| p.normal_impulse)
}
fn triplet(f: &Frame) -> Result<(usize, usize, usize)> {
    let worlds = f
        .components
        .as_array()
        .ok_or("missing actual native components")?;
    let mut groups = vec![];
    let mut occupied = 0;
    for (worker, world) in worlds.iter().enumerate() {
        let comps = world.as_array().ok_or("missing group list")?;
        occupied += usize::from(!comps.is_empty());
        for comp in comps {
            let ids = comp["ids"].as_array().ok_or("missing native IDs")?;
            if ids.iter().any(|v| {
                v.as_u64().is_none_or(|id| {
                    id == 0 || id > 62 || f.owners[id as usize - 1] as usize != worker
                })
            }) {
                return Err("native component owner mismatch".into());
            }
            groups.push(ids.len());
        }
    }
    if groups.iter().sum::<usize>() != 62 {
        return Err("native component coverage".into());
    }
    Ok((
        occupied,
        groups.len(),
        *groups.iter().max().ok_or("missing native world")?,
    ))
}
pub fn render(root: &Path, receipt: bool) -> Result<Rendered> {
    let data = read(&root.join("clip-comparison.json"))?;
    let (ordinary, _) = capture(
        &root.join("ordinary-clip/reference.jsonl"),
        62,
        &[(170, 15265)],
    )?;
    let (canonical, _) = capture(
        &root.join("reference-clip/reference.jsonl"),
        62,
        &[(170, 15265)],
    )?;
    let pair: [u64; 2] = serde_json::from_value(data["pair"].clone())?;
    let event = data["first_observed_physical_difference"]
        .as_u64()
        .ok_or("missing divergence")?;
    let move_at = data["complete_migration_tick"]
        .as_u64()
        .ok_or("missing migration")?;
    let ack_at = data["lost_ack_barrier_retry_tick"]
        .as_u64()
        .ok_or("missing retry")?;
    let proof = data["warm_replay_proof"]
        .as_array()
        .ok_or("missing direct warm replay")?;
    if proof.len() != 2
        || proof.iter().any(|x| x["exact_live_physical_bytes"] != true)
        || ordinary.len() != canonical.len()
        || ordinary.len() != 97
        || data["physical_frames_per_arm"] != ordinary.len()
    {
        return Err("unverified solver-order capture".into());
    }
    for (a, b) in ordinary.iter().zip(&canonical) {
        if a.tick != b.tick
            || b.control_sha256_sample.as_deref() != Some(&b.trajectory_sha256_sample)
            || a.control_sha256_sample != b.control_sha256_sample
            || (a.tick < event && a.trajectory_sha256_sample != b.trajectory_sha256_sample)
        {
            return Err("canonical control or physical prefix disagreement".into());
        }
    }
    let index = canonical
        .iter()
        .position(|f| f.tick == event)
        .ok_or("missing first observed difference")?;
    let impact = &canonical[index];
    for (kind, hash) in [
        ("worker", &ordinary[index].trajectory_sha256_sample),
        ("reference", &impact.trajectory_sha256_sample),
    ] {
        if !proof
            .iter()
            .any(|x| x["world"] == kind && x["sha256"] == *hash)
        {
            return Err("direct isolated native replay differs at first difference".into());
        }
    }
    if impulse(impact, pair) == impulse(&ordinary[index], pair) {
        return Err("reported pair has identical source impulses".into());
    }
    let p0 = impact.bodies[pair[0] as usize - 1].position;
    let p1 = impact.bodies[pair[1] as usize - 1].position;
    let windows: Vec<[u64; 2]> = serde_json::from_value(data["sample_windows"].clone())?;
    let [begin, end] = windows[usize::from(receipt)];
    let groups = [&ordinary, &canonical];
    let tags = ["ordinary", "canonical"];
    let selected: BTreeSet<_> = if receipt {
        pair.iter().copied().collect()
    } else {
        (1..=62).collect()
    };
    let (center, scale, offset) = if receipt {
        ([(p0[0] + p1[0]) / 2., (p0[1] + p1[1]) / 2.], 1., 2.4)
    } else {
        ([0., 0.], 0.45, 5.6)
    };
    let mut panels = [0, 1].map(|i| {
        Bodies::new(
            tags[i],
            10_000 + i as u64 * 50_000,
            Transform {
                center,
                scale,
                offset_y: if i == 0 { offset } else { -offset },
            },
            62,
        )
    });
    let mut samples = vec![];
    let mut stream = vec![];
    let mut migration = [0; 2];
    for (i, frames) in groups.iter().enumerate() {
        let j = frames
            .iter()
            .position(|f| f.tick == move_at)
            .ok_or("missing migration frame")?;
        migration[i] = frames[j]
            .owners
            .iter()
            .zip(&frames[j - 1].owners)
            .filter(|(a, b)| a != b)
            .count();
        if migration[i] != [25, 62][i] {
            return Err("unexpected number of actual transferred bodies".into());
        }
    }
    for (i, base) in ordinary
        .iter()
        .enumerate()
        .filter(|(_, f)| (begin..=end).contains(&f.tick))
    {
        let tick = base.tick;
        let revision = &canonical[i];
        let equal = base.trajectory_sha256_sample == revision.trajectory_sha256_sample;
        let mut v = View::default();
        floor(
            &mut v,
            if receipt {
                [10., 10., 0.12]
            } else {
                [26., 22., 0.12]
            },
        );
        let mut arms = vec![];
        let values = [impulse(base, pair), impulse(revision, pair)];
        for (j, f) in [base, revision].into_iter().enumerate() {
            let (owners, worlds, largest) = triplet(f)?;
            panels[j].draw(&mut v, f, None, Some(&selected))?;
            let generation = if receipt {
                f.generations[pair[0] as usize - 1]
            } else {
                *f.generations.iter().max().ok_or("missing generations")?
            };
            let bar = if receipt {
                let magnitude = values[j].unwrap_or(0.);
                let label =
                    values[j].map_or_else(|| "absent".to_owned(), |imp| format!("{imp:.17}"));
                let width = magnitude * 3.;
                (
                    format!(
                        "{}_native_tick_{tick}_pair_{}_{}_impulse_{label}_N_s",
                        tags[j], pair[0], pair[1]
                    ),
                    width,
                    -2.7,
                    [0.9, -4.2][j],
                )
            } else {
                (
                    format!(
                        "{}_native_tick_{tick}_{}_owners_{}_worlds_largest_{}_generation_{}",
                        tags[j], owners, worlds, largest, generation
                    ),
                    largest as f64 * 6.5 / 62.,
                    -3.25,
                    [1.0, -10.25][j],
                )
            };
            v.cuboid(
                200_000 + (tick - begin) * 8 + 2 * j as u64,
                &bar.0,
                [bar.2 + bar.1 / 2., bar.3, 0.24],
                [bar.1.max(0.03), 0.25, 0.30],
                [[0.37, 0.68, 0.95, 1.], [0.82, 0.71, 0.28, 1.]][j],
            );
            arms.push(json!({"owners":owners,"native_worlds":worlds,"largest":largest,"generation":generation,"impulse":values[j],"bar":bar.0,"width":bar.1.max(0.03),"sha256":f.trajectory_sha256_sample}));
        }
        let note = if receipt {
            if tick == event {
                let diff = values[1].unwrap() - values[0].unwrap();
                let h = [
                    &data["ordinary_previous_native_handles"],
                    &data["canonical_previous_native_handles"],
                ];
                format!(
                    "native_tick_{tick}_first_difference_pair_{}_{}_delta_{diff:.17e}_N_s_ordinary_handles_{}_{}_canonical_{}_{}",
                    pair[0], pair[1], h[0][0], h[0][1], h[1][0], h[1][1]
                )
            } else {
                format!(
                    "native_tick_{tick}_physical_bytes_{}",
                    if equal { "identical" } else { "different" }
                )
            }
        } else if tick >= ack_at {
            format!("native_tick_{ack_at}_step_ack_lost_and_confirmed_both_trajectories_equal")
        } else if tick >= move_at {
            format!(
                "native_tick_{move_at}_ordinary_{}_body_handoff_canonical_{}_body_handoff",
                migration[0], migration[1]
            )
        } else {
            format!(
                "native_tick_{tick}_ordinary_{}_native_worlds_canonical_{}_native_worlds",
                arms[0]["native_worlds"], arms[1]["native_worlds"]
            )
        };
        let color = if equal {
            [0.12, 0.65, 0.44, 1.]
        } else {
            [0.90, 0.45, 0.13, 1.]
        };
        v.cuboid(
            300_000 + (tick - begin) * 2,
            &note,
            [if receipt { 1.8 } else { 6. }, 0., 0.26],
            [0.42, 0.40, 0.47],
            color,
        );
        stream.push(v.finish(tick - begin));
        samples.push(json!({"tick":tick,"byte_equal":equal,"note":note,"arms":arms}));
    }
    let transforms = panels.map(|p| p.transform);
    Ok((
        stream,
        json!({"mode":if receipt{"canonical_receipt"}else{"canonical_migration"},"panels":transforms,"selected_ids":selected,"pair":pair,"summary":data,"frames":samples.len(),"samples":samples}),
    ))
}
