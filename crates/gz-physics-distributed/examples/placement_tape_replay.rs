// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Display the original measured placement and its independently executed deterministic replay.
#[path = "support/worker_scene_display.rs"]
mod display;
#[path = "support/changing_worlds.rs"]
#[allow(dead_code, unused_imports)]
mod fixture;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
use clap::{Parser, ValueEnum};
use display::{BodyLayer, Frame, OWNER_COLORS};
use gz_physics_distributed::pool::PlacementRecord;
use primitive_replay_view::View;
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    #[arg(long)]
    evidence: PathBuf,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Row {
    #[serde(flatten)]
    f: Frame,
    generations: Vec<u64>,
    components: Value,
    pids: [u32; 4],
    substeps: Option<u8>,
    refined: Option<Vec<u64>>,
    events: Vec<Value>,
    window_counters: [u64; 4],
    public_sha256: String,
    physical_sha256: String,
    committed_placement: Option<PlacementRecord>,
}
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
fn lines(p: &Path) -> Result<Vec<Value>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect()
}
fn rows(p: &Path, pads: &[u64]) -> Result<Vec<Row>> {
    lines(p)?
        .into_iter()
        .map(|mut v| {
            let pairs = v["pairs"]
                .as_array()
                .ok_or("absent finite contact receipts")?;
            let counts = pads
                .iter()
                .map(|&pad| {
                    pairs
                        .iter()
                        .filter(|p| p["a"] == pad && p["normal_impulse"].as_f64().unwrap() > 0.)
                        .count()
                })
                .collect::<Vec<_>>();
            v["loaded_pairs"] = json!(counts);
            Ok(serde_json::from_value(v)?)
        })
        .collect()
}
fn bytes(r: &Row) -> Result<Vec<u8>> {
    Ok(bincode::serialize(&(
        r.f.tick,
        &r.f.bodies,
        &r.f.pairs,
        &r.f.floors,
    ))?)
}
fn receipt_digest(s: &str) -> Result<Vec<u8>> {
    if s.len() != 64 || !s.is_ascii() {
        return Err("invalid SHA-256".into());
    }
    (0..32)
        .map(|i| Ok(u8::from_str_radix(&s[2 * i..2 * i + 2], 16)?))
        .collect()
}
fn main() -> Result<()> {
    let a = Args::parse();
    let summary = read(&a.evidence.join("summary.json"))?;
    let groups = fixture::seeds(a.layout);
    let pads = groups
        .iter()
        .filter(|g| g.len() == 25)
        .map(|g| g[0].id)
        .collect::<Vec<_>>();
    let original = rows(&a.evidence.join("recorded.jsonl"), &pads)?;
    let replay = rows(&a.evidence.join("replay.jsonl"), &pads)?;
    let tape_path = a.evidence.join("tape.jsonl");
    let tape = lines(&tape_path)?
        .into_iter()
        .map(serde_json::from_value::<PlacementRecord>)
        .collect::<std::result::Result<Vec<_>, _>>()?;
    if summary["layout"] != a.layout.to_possible_value().unwrap().get_name()
        || original.len() != 351
        || replay.len() != 351
        || tape.len() != 350
        || summary["tape_sha256"] != format!("{:x}", Sha256::digest(fs::read(tape_path)?))
    {
        return Err("incomplete or mismatched evidence".into());
    }
    let mut layer = BodyLayer::new(&groups, 0, "shared", 0.);
    let mut stream = Vec::new();
    let mut samples = Vec::new();
    let mut physical = Sha256::new();
    let mut published = Sha256::new();
    let mut owners = original[0].f.owners.clone();
    let mut generations = original[0].generations.clone();
    let mut fault = None;
    let mut restarts = Vec::new();
    let mut last_move = None;
    let mut all_events = vec![];
    let mut n_moves = 0;
    for (i, (left, right)) in original.iter().zip(&replay).enumerate() {
        let t = i as u64;
        let rec = t.checked_sub(1).map(|k| &tape[k as usize]);
        let lhs = bytes(left)?;
        let rhs = bytes(right)?;
        if t != left.f.tick
            || t != right.f.tick
            || lhs != rhs
            || format!("{:x}", Sha256::digest(&lhs)) != left.physical_sha256
            || left.physical_sha256 != right.physical_sha256
            || left.f.bodies.len() != 62
            || left
                .f
                .bodies
                .iter()
                .enumerate()
                .any(|(i, b)| b.id != i as u64 + 1)
            || left.f.owners != right.f.owners
            || left.generations != right.generations
            || left.components != right.components
            || left.substeps != right.substeps
            || left.refined != right.refined
            || left.public_sha256 != right.public_sha256
            || left.committed_placement.as_ref() != rec
            || right.committed_placement.as_ref() != rec
            || rec.is_some_and(|r| r.target_tick != t)
            || right.window_counters != [0; 4]
            || !left.events.is_empty()
        {
            return Err(
                format!("inconsistent raw tape and executed public/physical state at {t}").into(),
            );
        }
        if i > 0 {
            physical.update(&lhs);
            published.update(receipt_digest(&left.public_sha256)?);
        }
        if let Some(m) = rec.and_then(|r| r.optional_move.as_ref()) {
            if m.members.is_empty() || !m.members.windows(2).all(|x| x[0].id < x[1].id) {
                return Err("invalid moved native island".into());
            }
            for b in &m.members {
                let idx = b.id.checked_sub(1).ok_or("invalid ID")? as usize;
                if owners.get(idx) != Some(&m.source) || generations.get(idx) != Some(&b.generation)
                {
                    return Err("stale handoff source/generation".into());
                }
                owners[idx] = m.destination;
                generations[idx] = b.generation + 1;
            }
            n_moves += 1;
            last_move = Some((t, m));
        }
        if owners != left.f.owners || generations != left.generations {
            return Err("observed ownership disagrees with persisted moves".into());
        }
        let mut actual_before = i
            .checked_sub(1)
            .map(|k| replay[k].pids)
            .unwrap_or(right.pids);
        for event in &right.events {
            let prior: [u32; 4] = serde_json::from_value(event["before_pids"].clone())?;
            let after: [u32; 4] = serde_json::from_value(event["after_pids"].clone())?;
            if event["tick"] != t || prior != actual_before {
                return Err("incorrect process proof".into());
            }
            match event["kind"].as_str() {
                Some("lost_final_ack_and_recipient_death") => {
                    let dst = event["killed_recipient"].as_u64().unwrap() as usize;
                    let lost = event["lost_ack_worker"].as_u64().unwrap() as usize;
                    if fault.is_some()
                        || rec.and_then(|r| r.optional_move.as_ref()).is_none_or(|m| {
                            usize::from(m.destination) != dst || usize::from(m.source) == lost
                        })
                        || dst == lost
                        || (0..4).any(|n| (prior[n] != after[n]) != (n == dst))
                        || event["visible_tick_on_failure"] != t - 1
                        || event["changed_directive_rejected"] != true
                    {
                        return Err("unverified lost acknowledgement or recipient".into());
                    }
                    fault = Some((t, lost, dst, prior[dst], after[dst]));
                }
                Some("fresh_coordinator_restore") => {
                    if prior.iter().any(|p| after.contains(p)) {
                        return Err("fresh restore kept an old worker PID".into());
                    }
                    restarts.push(t);
                }
                _ => return Err("unknown replay event".into()),
            }
            actual_before = after;
            all_events.push(event.clone());
        }
        if actual_before != right.pids || left.pids != original[0].pids {
            return Err("unexplained process replacement".into());
        }
        let mut view = View::default();
        view.cuboid(
            2,
            "shared_ground_display",
            [0., 0., -0.08],
            [24., 19., 0.16],
            [0.28, 0.34, 0.40, 1.],
        );
        layer.draw(&mut view, &left.f, &left.f.owners)?;
        for (k, row) in [left, right].into_iter().enumerate() {
            let label = ["measured", "taped"][k];
            let center = [5.75, 6.55][k];
            let base = 2_500_000 + 80 * t + 20 * k as u64;
            let n = row.window_counters[0];
            let len = (0.13 * n as f64).max(0.08);
            view.cuboid(
                base,
                &format!("{label}_tick_{t}_timing_windows_started_{n}"),
                [-10.7 + len / 2., center, 0.22],
                [len, 0.35, 0.32],
                if k == 0 {
                    [0.39, 0.75, 0.64, 1.]
                } else {
                    [0.50, 0.56, 0.62, 1.]
                },
            );
            for (w, pid) in row.pids.iter().enumerate() {
                let count = owners.iter().filter(|&&n| usize::from(n) == w).count();
                let width = (2.5 * count as f64 / 62.).max(0.05);
                view.cuboid(
                    base + 2 + 2 * w as u64,
                    &format!("{label}_tick_{t}_worker_{w}_pid_{pid}_bodies_{count}"),
                    [-3.5 + 3.4 * w as f64 + width / 2., center, 0.22],
                    [width, 0.32, 0.32],
                    OWNER_COLORS[w],
                );
            }
        }
        let base = 3_000_000 + 20 * t;
        for (k, &pad) in pads.iter().enumerate() {
            let n = left.f.loaded_pairs[k];
            let width = (2.9 * n as f64 / 24.).max(0.05);
            view.cuboid(
                base + 2 * k as u64,
                &format!("shared_tick_{t}_pad_{pad}_positive_finite_contacts_{n}"),
                [-10.7 + 4.1 * k as f64 + width / 2., -5.9, 0.2],
                [width, 0.30, 0.28],
                [0.94, 0.77, 0.33, 1.],
            );
        }
        if let Some((at, m)) = last_move {
            let ids = m
                .members
                .iter()
                .map(|m| m.id.to_string())
                .collect::<Vec<_>>()
                .join("_");
            let width = 0.1 * m.members.len() as f64;
            view.cuboid(
                base + 4,
                &format!(
                    "last_persisted_move_at_{at}_worker_{}_to_{}_members_{ids}",
                    m.source, m.destination
                ),
                [-1.3 + width / 2., -5.9, 0.23],
                [width, 0.33, 0.35],
                OWNER_COLORS[usize::from(m.destination)],
            );
        }
        if let Some((at, lost, dst, old, new)) = fault {
            view.cuboid(base+6,&format!("tick_{at}_lost_ack_worker_{lost}_held_prior_barrier_rejected_different_directive_replaced_recipient_{dst}_pid_{old}_to_{new}"),[4.7,-5.9,0.26],[0.45,0.45,0.40],[0.96,0.40,0.17,1.]);
        }
        for (n, &at) in restarts.iter().enumerate() {
            view.cuboid(
                base + 8 + 2 * n as u64,
                &format!("all_four_replay_processes_restored_at_{at}"),
                [6.0 + 0.65 * n as f64, -5.9, 0.24],
                [0.36, 0.36, 0.35],
                [0.55, 0.73, 0.88, 1.],
            );
        }
        samples.push(json!({"tick":t,"physical_sha256":left.physical_sha256,"public_sha256":left.public_sha256,"original_started":left.window_counters[0],"replay_started":right.window_counters[0],"original_pids":left.pids,"replay_pids":right.pids,"events":right.events,"move":rec.and_then(|r|r.optional_move.as_ref()),"last_move":last_move.map(|(at,m)|json!([at,m])),"positive_pairs":left.f.loaded_pairs}));
        stream.push(view.finish(t));
    }
    if summary["public_barrier_sha256"] != format!("{:x}", published.finalize())
        || summary["native_sha256"] != format!("{:x}", physical.finalize())
        || fault.is_none()
        || restarts != [70, 160, 280]
        || n_moves == 0
        || original[350].window_counters[0] == 0
        || summary["events"] != json!(all_events)
    {
        return Err("incomplete exact-replay qualification".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    serde_json::to_writer_pretty(
        File::create(a.directory.join("index.json"))?,
        &json!({"source_summary":summary,"samples":samples}),
    )?;
    println!(
        "All 351 measured and replayed states matched the raw tape, public solver digest, physical native reference, per-body owner and generation, substeps and finite/ground receipts. {n_moves} whole-island moves, one rejected altered retry, one recipient replacement and three fresh four-peer restores."
    );
    Ok(())
}
