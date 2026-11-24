// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare real migration-stage failures using physically qualified public barriers and measured wall time.
#[path = "support/worker_scene_display.rs"]
mod display;
#[path = "support/changing_worlds.rs"]
#[allow(dead_code, unused_imports)]
mod fixture;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
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
    #[arg(long)]
    evidence: PathBuf,
    #[arg(long)]
    trace: PathBuf,
    directory: PathBuf,
}
#[derive(Clone, Deserialize, Serialize)]
struct Observation {
    policy: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    grace_ms: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    thaw_ms: Option<u64>,
    fault: String,
    endpoint: u8,
    before_pids: Vec<u32>,
    after_pids: Vec<u32>,
    elapsed_us: f64,
    requests: u64,
    request_bytes: u64,
    reply_bytes: u64,
}
impl Observation {
    fn label(&self) -> &str {
        if self.grace_ms == Some(150) {
            "confirm_150ms"
        } else if self.policy == "confirm" && matches!(self.fault.as_str(), "stopped" | "thaw") {
            "confirm_long_grace"
        } else {
            &self.policy
        }
    }
    fn replaced(&self) -> usize {
        self.before_pids
            .iter()
            .zip(&self.after_pids)
            .filter(|(a, b)| a != b)
            .count()
    }
}
#[derive(Clone, Deserialize, Serialize)]
struct Movement {
    tick: u64,
    source: u8,
    destination: u8,
    members: usize,
    withheld: String,
    member_ids: Vec<u64>,
    recovery: Observation,
}
impl Movement {
    fn same(&self, b: &Self) -> bool {
        self.tick == b.tick
            && self.source == b.source
            && self.destination == b.destination
            && self.member_ids == b.member_ids
            && self.withheld == b.withheld
            && self.recovery.endpoint == b.recovery.endpoint
            && self.recovery.fault == b.recovery.fault
    }
}
#[derive(Clone, Deserialize, Serialize)]
struct Episode {
    layout: String,
    number: u64,
    width: usize,
    initial_owners: Vec<u8>,
    moves: Vec<Movement>,
    physics_sha256: String,
}
impl Episode {
    fn owners(&self, tick: u64, committed: bool) -> Result<Vec<u8>> {
        let mut owners = self.initial_owners.clone();
        for m in self
            .moves
            .iter()
            .filter(|m| m.tick < tick || m.tick == tick && committed)
        {
            for &id in &m.member_ids {
                let w = owners
                    .get_mut(id.checked_sub(1).ok_or("zero body ID")? as usize)
                    .ok_or("unknown member")?;
                if *w != m.source {
                    return Err("inconsistent recorded source".into());
                }
                *w = m.destination;
            }
        }
        Ok(owners)
    }
}
#[derive(Serialize)]
struct Case {
    movement: usize,
    files: [String; 2],
    source_sha256: [String; 2],
    arms: [Episode; 2],
}
fn load(path: &Path, layout: &str, policy: &str) -> Result<BTreeMap<u64, (Episode, String)>> {
    let mut out = BTreeMap::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let raw = line?;
        let e: Episode = serde_json::from_str(&raw)?;
        if e.layout != layout {
            continue;
        }
        if e.initial_owners.len() != 62
            || e.initial_owners.iter().any(|&w| w as usize >= e.width)
            || e.moves.len() != 2
        {
            return Err("wrong episode geometry".into());
        }
        for m in &e.moves {
            let o = &m.recovery;
            if m.members == 0
                || m.members != m.member_ids.len()
                || m.source == m.destination
                || m.source as usize >= e.width
                || m.destination as usize >= e.width
                || m.member_ids.iter().collect::<BTreeSet<_>>().len() != m.members
                || o.policy != policy
                || ![m.source, m.destination].contains(&o.endpoint)
                || o.before_pids.len() != e.width
                || o.after_pids.len() != e.width
                || !o.elapsed_us.is_finite()
                || o.elapsed_us <= 0.
                || o.requests == 0
                || [&o.before_pids, &o.after_pids]
                    .into_iter()
                    .any(|p| p.contains(&0) || p.iter().collect::<BTreeSet<_>>().len() != e.width)
            {
                return Err("incomplete move evidence".into());
            }
        }
        e.owners(350, true)?;
        let key = e.number;
        if out
            .insert(key, (e, format!("{:x}", Sha256::digest(raw.as_bytes()))))
            .is_some()
        {
            return Err("duplicate episode".into());
        }
    }
    Ok(out)
}
fn select(
    root: &Path,
    layout: &str,
    campaign: &str,
    controls: [&str; 2],
    faults: [&str; 2],
) -> Result<Vec<Case>> {
    let files = controls.map(|p| format!("{campaign}-{p}/episodes.jsonl"));
    let a = load(&root.join(&files[0]), layout, controls[0])?;
    let b = load(&root.join(&files[1]), layout, "confirm")?;
    let mut result = vec![];
    for fault in faults {
        let mut eligible = vec![];
        for (&number, (e, _)) in &a {
            for (i, m) in e
                .moves
                .iter()
                .enumerate()
                .filter(|(_, m)| m.recovery.fault == fault)
            {
                // Prefer 25-body groups and receiver-side commands, then the first seeded episode.
                eligible.push((
                    usize::MAX - m.members,
                    m.recovery.endpoint != m.destination,
                    number,
                    i,
                ));
            }
        }
        eligible.sort();
        let (_, _, number, i) = *eligible.first().ok_or("missing required fault")?;
        let (prior, sha0) = &a[&number];
        let (candidate, sha1) = b.get(&number).ok_or("missing matched policy")?;
        if controls[1] == "short"
            && candidate
                .moves
                .iter()
                .any(|m| m.recovery.grace_ms != Some(150) || m.recovery.thaw_ms != Some(100))
        {
            return Err("unexpected bounded grace configuration".into());
        }
        if prior.initial_owners != candidate.initial_owners
            || prior.width != candidate.width
            || prior.physics_sha256 != candidate.physics_sha256
            || prior
                .moves
                .iter()
                .zip(&candidate.moves)
                .any(|(a, b)| !a.same(b))
        {
            return Err("fault inputs differ by policy".into());
        }
        result.push(Case {
            movement: i,
            files: files.clone(),
            source_sha256: [sha0.clone(), sha1.clone()],
            arms: [prior.clone(), candidate.clone()],
        });
    }
    Ok(result)
}
#[derive(Serialize)]
struct Sample {
    case: usize,
    arm: usize,
    stage: &'static str,
    tick: u64,
    display_ns: u64,
}
fn schedule(case: usize, arm: usize, m: &Movement, start: u64) -> Vec<Sample> {
    let f = |stage, tick, display_ns| Sample {
        case,
        arm,
        stage,
        tick,
        display_ns,
    };
    let dt = 10_000_000;
    let mut out: Vec<_> = (0..6)
        .map(|i| f("before", m.tick - 6 + i, start + i * dt))
        .collect();
    let held = start + 6 * dt;
    out.push(f("handoff", m.tick, held));
    let commit = held + (m.recovery.elapsed_us * 1_000.).round() as u64;
    out.extend((0..13).map(|i| f("committed", m.tick + i, commit + i * dt)));
    out
}
fn main() -> Result<()> {
    let args = Args::parse();
    let name = args.layout.to_possible_value().unwrap();
    let layout = name.get_name();
    let mut cases = select(
        &args.evidence,
        layout,
        "matched-0",
        ["recover", "confirm"],
        ["lost", "killed"],
    )?;
    cases.extend(select(
        &args.evidence,
        layout,
        "stopped-0",
        ["recover", "confirm"],
        ["thaw", "stopped"],
    )?);
    cases.extend(select(
        &args.evidence,
        layout,
        "stopped-0",
        ["confirm", "short"],
        ["thaw", "stopped"],
    )?);
    let mut frames = vec![];
    let mut digest = Sha256::new();
    for line in BufReader::new(File::open(&args.trace)?).lines() {
        let f: Frame = serde_json::from_str(&line?)?;
        if f.tick != frames.len() as u64 || f.bodies.len() != 62 || f.owners.len() != 62 {
            return Err("incomplete physical trace".into());
        }
        if f.tick != 0 {
            digest.update(bincode::serialize(&(
                f.tick, &f.bodies, &f.pairs, &f.floors,
            ))?);
        }
        frames.push(f);
    }
    let sha = format!("{:x}", digest.finalize());
    if frames.len() != 351 || cases.iter().any(|c| c.arms[0].physics_sha256 != sha) {
        return Err("recorded encounter diverges from observed public physics".into());
    }
    let seed = fixture::seeds(args.layout);
    let mut layer = BodyLayer::new(&seed, 0, "common", 0.);
    let shifts: BTreeMap<_, _> = seed
        .iter()
        .flat_map(|g| {
            let [x, y, _] = g[0].position;
            g.iter().map(move |b| (b.id, [-0.56 * x, -0.56 * y]))
        })
        .collect();
    let mut stream = vec![];
    let mut timeline = vec![];
    let mut cursor = 0;
    for (case, c) in cases.iter().enumerate() {
        let paired = c.arms.each_ref().map(|e| &e.moves[c.movement].recovery);
        for (arm, e) in c.arms.iter().enumerate() {
            let m = &e.moves[c.movement];
            let o = &m.recovery;
            for s in schedule(case, arm, m, cursor) {
                let f = &frames[s.tick as usize];
                let done = s.stage == "committed";
                let during = s.stage == "handoff";
                let mut v = View::default();
                let base = 1_000_000 + stream.len() as u64 * 200;
                v.cuboid(
                    2,
                    "display_floor",
                    [0., 0., -0.07],
                    [24., 17., 0.14],
                    [0.31, 0.36, 0.42, 1.],
                );
                layer.draw(&mut v, f, &e.owners(s.tick, done)?)?;
                for (i, &id) in m.member_ids.iter().enumerate() {
                    let b = &f.bodies[id as usize - 1];
                    let [dx, dy] = shifts[&id];
                    let [x, y, z] = b.position;
                    v.cuboid(
                        base + 2 * i as u64,
                        &format!("{}_{}_island_member_{id}", o.label(), o.fault),
                        [x + dx, y + dy, z + b.size[2] / 2. + 0.12],
                        [0.14, 0.14, 0.12],
                        [0.96, 0.88, 0.48, 1.],
                    );
                }
                for j in 0..e.width {
                    let changed = o.before_pids[j] != o.after_pids[j];
                    let (pid, color, state) = if done {
                        (
                            o.after_pids[j],
                            if changed {
                                [0.94, 0.53, 0.20, 1.]
                            } else {
                                [0.10, 0.68, 0.46, 1.]
                            },
                            if changed { "replaced" } else { "retained" },
                        )
                    } else if during && j == o.endpoint as usize {
                        (o.before_pids[j], [0.88, 0.24, 0.23, 1.], "faulted")
                    } else {
                        (o.before_pids[j], [0.32, 0.51, 0.73, 1.], "before")
                    };
                    v.cuboid(
                        base + 60 + 2 * j as u64,
                        &format!("{}_{}_worker_{j}_pid_{pid}_{state}", o.label(), o.fault),
                        [-4. + j as f64 * 1.2, 5.6, 0.31],
                        [0.66, 0.44, 0.60],
                        color,
                    );
                }
                for (row, (tag, values)) in [
                    ("elapsed_us", paired.map(|a| a.elapsed_us)),
                    ("requests", paired.map(|a| a.requests as f64)),
                    (
                        "wire_bytes",
                        paired.map(|a| (a.request_bytes + a.reply_bytes) as f64),
                    ),
                    ("replaced_pids", paired.map(|a| a.replaced() as f64)),
                ]
                .into_iter()
                .enumerate()
                {
                    for (k, x) in values.into_iter().enumerate() {
                        let width = 7. * x / values[0].max(values[1]).max(1.);
                        v.cuboid(
                            base + 90 + row as u64 * 4 + 2 * k as u64,
                            &format!("{}_{}_{tag}_{x:.3}", paired[k].label(), o.fault),
                            [[-9., 1.][k] + width / 2., -5. - row as f64 * 0.62, 0.22],
                            [if width == 0. { 0.10 } else { width }, 0.25, 0.3],
                            [[0.37, 0.68, 0.95, 1.], [0.82, 0.71, 0.28, 1.]][k],
                        );
                    }
                }
                for (i, &count) in f.loaded_pairs.iter().enumerate() {
                    v.cuboid(
                        base + 112 + 2 * i as u64,
                        &format!("pad_{}_positive_contacts_{count}", i + 1),
                        [-1. + 2. * i as f64, 0., count as f64 / 12.],
                        [0.50, 0.35, 0.15],
                        [0.10, 0.74, 0.62, 1.],
                    );
                }
                v.cuboid(
                    base + 120,
                    &format!(
                        "{}_{}_{}_{}_tick_{}_w{}_to_w{}_{}_members",
                        o.label(),
                        o.fault,
                        m.withheld,
                        s.stage,
                        m.tick,
                        m.source,
                        m.destination,
                        m.members
                    ),
                    [0., 1.8, 0.29],
                    [0.8, 0.55, 0.58],
                    [0.84, 0.41, 0.21, 1.],
                );
                cursor = s.display_ns + 10_000_000;
                stream.push(v.finish_at(stream.len() as u64 + 1, s.display_ns));
                timeline.push(s);
            }
        }
    }
    fs::create_dir(&args.directory)?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        args.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"physics_sha256":sha,"layout":layout,"cases":cases,"timeline":timeline}),
        )?,
    )?;
    println!(
        "Exported {} true source barrier and elapsed-time samples across twelve matched handoffs.",
        timeline.len()
    );
    Ok(())
}
