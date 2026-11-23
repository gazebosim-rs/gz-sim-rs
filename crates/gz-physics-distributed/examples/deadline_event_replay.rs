// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Play actual Linux worker deadline observations against their verified public physics barriers.
#[path = "support/worker_scene_display.rs"]
mod display;
#[allow(dead_code, unused_imports)]
#[path = "support/changing_worlds.rs"]
mod fixture;
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::{Parser, ValueEnum};
use display::{BodyLayer, Frame};
use primitive_replay_view::View;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
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
    /// Completed JSON lines from genuine Linux pidfd deadline episodes.
    #[arg(long)]
    episodes: PathBuf,
    /// Optional previous build for comparing one truly unavailable worker's reconstruction.
    #[arg(long)]
    reference: Option<PathBuf>,
    /// Paired process-termination benchmark directory for the eight-worker all / targeted comparison.
    #[arg(long, requires = "reference")]
    killed_campaign: Option<PathBuf>,
    /// Canonical 351-frame distributed body/contact trace of the same loaded fixture.
    #[arg(long)]
    trace: PathBuf,
    directory: PathBuf,
}
#[derive(Serialize, Deserialize, PartialEq)]
struct Move {
    tick: u64,
    source: u8,
    destination: u8,
    member_ids: Vec<u64>,
}
#[derive(Serialize, Deserialize)]
struct Deadline {
    phase: String,
    owner: u8,
    tick: u64,
    timeout_us: f64,
    retry_us: f64,
    replacements: usize,
    before_pids: Vec<u32>,
    after_pids: Vec<u32>,
}
#[derive(Serialize, Deserialize)]
struct Episode {
    number: u64,
    layout: String,
    width: usize,
    initial_owners: Vec<u8>,
    moves: Vec<Move>,
    deadline: Deadline,
    physics_sha256: String,
}
const PHASES: [&str; 3] = ["step_thaw", "step_fence", "prepare_thaw"];
fn selected(p: &Path, layout: &str) -> Result<(Vec<Episode>, BTreeMap<String, String>)> {
    let mut rows = BTreeMap::new();
    let mut hashes = BTreeMap::new();
    for line in BufReader::new(File::open(p)?).lines() {
        let line = line?;
        let e: Episode = serde_json::from_str(&line)?;
        if e.layout != layout
            || !PHASES.contains(&e.deadline.phase.as_str())
            || rows.contains_key(&e.deadline.phase)
        {
            continue;
        }
        let d = &e.deadline;
        if d.owner as usize >= e.width
            || d.before_pids.len() != e.width
            || d.after_pids.len() != e.width
            || e.initial_owners.len() != 62
            || e.initial_owners.iter().any(|&o| o as usize >= e.width)
            || d.before_pids.iter().collect::<BTreeSet<_>>().len() != e.width
            || d.after_pids.iter().collect::<BTreeSet<_>>().len() != e.width
            || d.replacements
                != d.before_pids
                    .iter()
                    .zip(&d.after_pids)
                    .filter(|(a, b)| a != b)
                    .count()
            || d.timeout_us < 800_000.
            || !d.retry_us.is_finite()
            || d.retry_us <= 0.
        {
            return Err("incomplete real deadline observation".into());
        }
        hashes.insert(
            format!("episode_{}", e.number),
            format!("{:x}", Sha256::digest(line.as_bytes())),
        );
        rows.insert(e.deadline.phase.clone(), e);
        if rows.len() == 3 {
            break;
        }
    }
    let ordered = PHASES
        .into_iter()
        .map(|p| {
            rows.remove(p)
                .ok_or_else(|| format!("missing {p} in {layout}").into())
        })
        .collect::<Result<_>>()?;
    Ok((ordered, hashes))
}
impl Episode {
    fn owners(&self, tick: u64, after_handoff: bool) -> Result<Vec<u8>> {
        let mut map = self.initial_owners.clone();
        for m in self
            .moves
            .iter()
            .filter(|m| m.tick < tick || m.tick == tick && after_handoff)
        {
            for &id in &m.member_ids {
                let owner = map
                    .get_mut(id.checked_sub(1).ok_or("invalid ID")? as usize)
                    .ok_or("unknown transferred member")?;
                if *owner != m.source || m.destination as usize >= self.width {
                    return Err("transfer disagrees with observed prior ownership".into());
                }
                *owner = m.destination;
            }
        }
        Ok(map)
    }
}
#[derive(Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Stage {
    Before,
    Stopped,
    Retry,
    Recovered,
}
#[derive(Serialize)]
struct Sample {
    episode: u64,
    stage: Stage,
    source_tick: u64,
    display_ns: u64,
}
fn schedule(e: &Episode, start: u64) -> Vec<Sample> {
    let d = &e.deadline;
    let held = d.tick - u64::from(d.phase != "prepare_thaw");
    let sample = |t, ns, stage| Sample {
        episode: e.number,
        stage,
        source_tick: t,
        display_ns: ns,
    };
    let mut frames = vec![];
    let ns_per_tick = 10_000_000;
    for i in 0..10 {
        frames.push(sample(
            held - 10 + i,
            start + i * ns_per_tick,
            Stage::Before,
        ));
    }
    let stopped = start + 10 * ns_per_tick;
    let retry = stopped + (d.timeout_us * 1_000.).round() as u64;
    let recovered = retry + (d.retry_us * 1_000.).round() as u64;
    frames.push(sample(held, stopped, Stage::Stopped));
    frames.push(sample(held, retry, Stage::Retry));
    for i in 0..25 {
        frames.push(sample(
            d.tick + i,
            recovered + i * ns_per_tick,
            Stage::Recovered,
        ));
    }
    frames
}
#[derive(Serialize)]
struct Killed {
    tick: u64,
    samples: u64,
    missing: Vec<u8>,
    us: [f64; 2],
    replaced: [u64; 2],
    requests: [u64; 2],
    source_sha256: String,
}
fn killed(root: &Path, layout: &str, digest: &str) -> Result<Killed> {
    let dir = root.join(format!("{layout}-8"));
    let doc: serde_json::Value = serde_json::from_reader(File::open(dir.join("result.json"))?)?;
    let tick = 160;
    let rows = doc["by_tick"]
        .as_array()
        .ok_or("missing process trial summary")?;
    let row = rows
        .iter()
        .find(|r| r["tick"] == tick)
        .ok_or("missing single-worker killed trial")?;
    let mut k = Killed {
        tick,
        samples: doc["rounds"].as_u64().ok_or("missing sample count")?,
        missing: serde_json::from_value(row["missing"].clone())?,
        us: serde_json::from_value(row["median_retry_us"].clone())?,
        replaced: serde_json::from_value(row["replacements"].clone())?,
        requests: serde_json::from_value(row["requests"].clone())?,
        source_sha256: String::new(),
    };
    if k.missing.len() != 1 || k.replaced != [8, 1] || doc["width"] != 8 || doc["layout"] != layout
    {
        return Err("unexpected process-termination scenario".into());
    }
    let path = dir.join("pairs.jsonl");
    let mut hash = Sha256::new();
    let mut costs: [Vec<f64>; 2] = Default::default();
    for line in BufReader::new(File::open(path)?).lines() {
        let line = line?;
        hash.update(line.as_bytes());
        hash.update(b"\n");
        let pair: serde_json::Value = serde_json::from_str(&line)?;
        for (arm, key) in ["reference", "candidate"].into_iter().enumerate() {
            let t = &pair[key];
            if t["digest"] != digest || t["width"] != 8 {
                return Err("unmatched killed-worker physics".into());
            }
            let sample = t["confirmations"]
                .as_array()
                .unwrap()
                .iter()
                .find(|v| v["tick"] == tick)
                .ok_or("missing killed process evidence")?;
            if sample["missing"] != serde_json::json!(k.missing)
                || sample["replacements"] != k.replaced[arm]
                || sample["requests"] != k.requests[arm]
            {
                return Err("raw killed endpoint evidence differs from summary".into());
            }
            costs[arm].push(sample["retry_us"].as_f64().ok_or("missing retry time")?);
        }
    }
    for (v, &median) in costs.iter_mut().zip(&k.us) {
        if v.is_empty() || v.len() != k.samples as usize {
            return Err("missing paired process-death trials".into());
        }
        v.sort_by(f64::total_cmp);
        let mid = (v[(v.len() - 1) / 2] + v[v.len() / 2]) / 2.;
        if (mid - median).abs() > 1e-8 {
            return Err("incorrect killed-worker median".into());
        }
    }
    k.source_sha256 = format!("{:x}", hash.finalize());
    Ok(k)
}
fn main() -> Result<()> {
    let args = Args::parse();
    let layout = args.layout.to_possible_value().unwrap();
    let layout = layout.get_name();
    let (cases, hashes) = selected(&args.episodes, layout)?;
    let prev = args
        .reference
        .as_ref()
        .map(|p| selected(p, layout))
        .transpose()?;
    let mut physics = Sha256::new();
    let mut frames = vec![];
    for line in BufReader::new(File::open(&args.trace)?).lines() {
        let f: Frame = serde_json::from_str(&line?)?;
        if f.tick != frames.len() as u64 || f.bodies.len() != 62 || f.owners.len() != 62 {
            return Err("invalid physical trace".into());
        }
        if f.tick > 0 {
            physics.update(bincode::serialize(&(
                f.tick, &f.bodies, &f.pairs, &f.floors,
            ))?)
        }
        frames.push(f)
    }
    let digest = format!("{:x}", physics.finalize());
    if frames.len() != 351 || cases.iter().any(|e| e.physics_sha256 != digest) {
        return Err("deadline trajectory differs from recorded physics".into());
    }
    let killed_data = args
        .killed_campaign
        .as_deref()
        .map(|root| killed(root, layout, &digest))
        .transpose()?;
    let mut layer = BodyLayer::new(&fixture::seeds(args.layout), 0, "common", 0.);
    let mut index = vec![];
    let mut stream = vec![];
    let mut start = 0;
    for (i, e) in cases.iter().enumerate() {
        let d = &e.deadline;
        let before = prev.as_ref().map(|(c, _)| &c[i]);
        if let Some(other) = before
            && (other.number != e.number
                || other.physics_sha256 != e.physics_sha256
                || other.initial_owners != e.initial_owners
                || other.moves != e.moves
                || other.deadline.owner != d.owner
                || other.deadline.tick != d.tick)
        {
            return Err("candidate episode differs from reference conditions".into());
        }
        for s in schedule(e, start) {
            let f = &frames[s.source_tick as usize];
            let mut v = View::default();
            v.cuboid(
                2,
                "display_floor",
                [0., 0., -0.07],
                [24., 17., 0.14],
                [0.31, 0.36, 0.42, 1.],
            );
            let after = s.stage == Stage::Recovered || s.source_tick != d.tick;
            let owners = e.owners(s.source_tick, after)?;
            layer.draw(&mut v, f, &owners)?;
            let base = 10_000 + i as u64 * 200;
            let stage = s.stage as u64;
            let names = ["before", "pidfd_stop", "timeout_then_retry", "recovered"];
            v.cuboid(
                base + 2 * stage,
                &format!(
                    "episode_{}_{}_{}_at_native_tick_{}",
                    e.number, d.phase, names[stage as usize], d.tick
                ),
                [0., 5.8, 0.28],
                [0.85, 0.45, 0.56],
                [0.88, 0.49, 0.20, 1.],
            );
            for (w, (&old, &new)) in d.before_pids.iter().zip(&d.after_pids).enumerate() {
                let (mark, color) = match s.stage {
                    Stage::Stopped if w == d.owner as usize => ("stopped", [0.87, 0.25, 0.29, 1.]),
                    Stage::Retry if w == d.owner as usize && d.phase == "step_fence" => {
                        ("still_stopped", [0.87, 0.25, 0.29, 1.])
                    }
                    Stage::Retry if w == d.owner as usize => ("resumed", [0.89, 0.70, 0.19, 1.]),
                    Stage::Recovered if old != new => ("replaced", [0.91, 0.40, 0.20, 1.]),
                    Stage::Recovered => ("retained", [0.10, 0.70, 0.42, 1.]),
                    _ => ("running", [0.47, 0.56, 0.67, 1.]),
                };
                let pid = if s.stage == Stage::Recovered {
                    new
                } else {
                    old
                };
                v.cuboid(
                    base + 20 + stage * 16 + w as u64 * 2,
                    &format!("episode_{}_worker_{w}_{mark}_pid_{pid}", e.number),
                    [-4.9 + w as f64 * 1.4, -4.65, 0.24],
                    [0.66, 0.40, 0.48],
                    color,
                );
            }
            let mut vals = if let Some(other) = before {
                vec![
                    ("prior_retry_us", other.deadline.retry_us),
                    ("targeted_retry_us", d.retry_us),
                    ("prior_changed_pids", other.deadline.replacements as f64),
                    ("targeted_changed_pids", d.replacements as f64),
                ]
            } else {
                vec![
                    ("timeout_us", d.timeout_us),
                    ("successful_retry_us", d.retry_us),
                ]
            };
            if let Some(k) = &killed_data {
                vals.extend([
                    ("prior_eight_worker_kill_median_us", k.us[0]),
                    ("targeted_eight_worker_kill_median_us", k.us[1]),
                    ("prior_eight_worker_kill_pid_changes", k.replaced[0] as f64),
                    (
                        "targeted_eight_worker_kill_pid_changes",
                        k.replaced[1] as f64,
                    ),
                ]);
            }
            for (j, (label, value)) in vals.iter().enumerate() {
                let pair = &vals[j / 2 * 2..j / 2 * 2 + 2];
                let scale = pair[0].1.max(pair[1].1).max(1.);
                let length = 8. * value / scale;
                let x = [-9., 1.][j % 2];
                v.cuboid(
                    base + 100 + j as u64 * 2,
                    &format!("episode_{}_{}_{value:.3}", e.number, label),
                    [x + length / 2., -5.55 - (j / 2) as f64 * 0.75, 0.26],
                    [if *value == 0. { 0.11 } else { length }, 0.30, 0.25],
                    [[0.37, 0.68, 0.95, 1.], [0.82, 0.71, 0.28, 1.]][j % 2],
                );
            }
            for (j, count) in f.loaded_pairs.iter().enumerate() {
                v.cuboid(
                    2000 + 2 * j as u64,
                    &format!("slab_{}_loaded_contacts_out_of_24", j + 1),
                    [-1. + j as f64 * 2., 0., *count as f64 / 12.],
                    [0.5, 0.35, 0.15],
                    [0.1, 0.74, 0.62, 1.],
                );
            }
            stream.push(v.finish_at(stream.len() as u64 + 1, s.display_ns));
            start = s.display_ns + 10_000_000;
            index.push(s);
        }
    }
    fs::create_dir(&args.directory)?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        args.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"episodes":cases,"reference":prev.as_ref().map(|(c,_)|c),"killed_benchmark":killed_data,"row_sha256":hashes,"reference_sha256":prev.as_ref().map(|(_,h)|h),"trajectory_sha256":digest,"frames":index}),
        )?,
    )?;
    println!(
        "Verified {} deadline samples, 3 real pidfd experiments and physical trajectory {digest}.",
        index.len()
    );
    Ok(())
}
