// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay observed signed per-worker momentum terms, explicitly distinguishing a solve from a same-tick handoff.
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
    #[arg(long)]
    seed: u64,
    #[arg(long)]
    trace: PathBuf,
    directory: PathBuf,
}
#[derive(Deserialize, Serialize)]
struct Terms {
    owner: usize,
    delta_ns: [f64; 3],
    migration_ns: [f64; 3],
    gravity_ns: [f64; 3],
    applied_ns: [f64; 3],
    floor_ns: [f64; 3],
    residual_ns: [f64; 3],
}
impl Terms {
    fn vertical(&self) -> [f64; 5] {
        [
            self.delta_ns[2],
            self.migration_ns[2],
            self.gravity_ns[2],
            self.applied_ns[2],
            self.floor_ns[2],
        ]
    }
}
#[derive(Deserialize, Serialize)]
struct Row {
    tick: u64,
    advancing: bool,
    workers: Vec<Terms>,
    global_residual_ns: [f64; 3],
}
fn lines<T: serde::de::DeserializeOwned>(p: &Path) -> Result<Vec<T>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect()
}
fn load_owner(e: &Value, tick: u64, after: bool) -> Result<Vec<u8>> {
    let mut map: Vec<u8> = serde_json::from_value(e["initial_owners"].clone())?;
    for m in e["moves"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|m| m["tick"].as_u64().unwrap() < tick || m["tick"] == tick && after)
    {
        for id in m["member_ids"].as_array().unwrap() {
            let w = map
                .get_mut(id.as_u64().unwrap() as usize - 1)
                .ok_or("unknown tracked member")?;
            if *w as u64 != m["source"].as_u64().unwrap() {
                return Err("migration has inconsistent prior owner".into());
            }
            *w = m["destination"].as_u64().unwrap() as u8;
        }
    }
    Ok(map)
}
fn pz(f: &Frame, map: &[u8], width: usize) -> Vec<f64> {
    let mut p = vec![0.; width];
    for (b, &w) in f.bodies.iter().zip(map) {
        p[w as usize] += b.mass * b.linear[2];
    }
    p
}
fn close(a: f64, b: f64) -> bool {
    a.is_finite() && b.is_finite() && (a - b).abs() < 1e-9
}
const LABELS: [&str; 5] = [
    "delta_pz",
    "transported_pz",
    "gravity_impulse",
    "applied_impulse",
    "ground_impulse",
];
fn main() -> Result<()> {
    let a = Args::parse();
    let layout = a.layout.to_possible_value().unwrap();
    let labels = layout.get_name();
    let episode = lines::<Value>(&a.evidence.join("episodes.jsonl"))?
        .into_iter()
        .find(|r| r["number"] == a.seed && r["layout"] == labels)
        .ok_or("missing audited episode")?;
    let source = a.evidence.join(format!("momentum-{}.jsonl", a.seed));
    let rows: Vec<Row> = lines(&source)?;
    let width = episode["width"].as_u64().ok_or("missing pool width")? as usize;
    let frames: Vec<Frame> = lines(&a.trace)?;
    let mut digest = Sha256::new();
    for (i, f) in frames.iter().enumerate() {
        if f.tick != i as u64 || f.bodies.len() != 62 || f.owners.len() != f.bodies.len() {
            return Err("invalid reference physics".into());
        }
        if i > 0 {
            digest.update(bincode::serialize(&(
                f.tick, &f.bodies, &f.pairs, &f.floors,
            ))?);
        }
    }
    if frames.len() != 351
        || format!("{:x}", digest.finalize()) != episode["physics_sha256"].as_str().unwrap()
        || rows.len() != 354
    {
        return Err("audit and solver evidence differ".into());
    }
    let max = rows
        .iter()
        .flat_map(|r| &r.workers)
        .flat_map(Terms::vertical)
        .map(f64::abs)
        .fold(0., f64::max);
    let groups = fixture::seeds(a.layout);
    let mut layer = BodyLayer::new(&groups, 0, "common", 0.);
    let mut previous = episode["initial_owners"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap() as u8)
        .collect::<Vec<_>>();
    let mut tick = 0;
    let mut held = 0u64;
    let mut stream = vec![];
    let mut samples = vec![];
    for (i, row) in rows.iter().enumerate() {
        if row.tick != tick + u64::from(row.advancing) || row.workers.len() != width {
            return Err("missing or unordered audit rows".into());
        }
        let f = &frames[row.tick as usize];
        let old = &frames[tick as usize];
        let owners = load_owner(&episode, row.tick, !row.advancing)?;
        let prior = pz(old, &previous, width);
        let next = pz(f, &owners, width);
        let mut flux = vec![0.; width];
        let mut mass = vec![0.; width];
        let mut ground = vec![0.; width];
        let mut pushed = vec![0.; width];
        for ((b, &w0), &w1) in old.bodies.iter().zip(&previous).zip(&owners) {
            let p = b.mass * b.linear[2];
            if w0 != w1 {
                flux[w0 as usize] -= p;
                flux[w1 as usize] += p;
            }
            if row.advancing {
                mass[w1 as usize] += b.mass;
            }
        }
        if row.advancing {
            for support in &f.floors {
                ground[owners[support.body as usize - 1] as usize] += support.normal_impulse;
            }
            for input in fixture::wrenches(row.tick, &groups) {
                pushed[owners[input.id as usize - 1] as usize] += input.force[2] * 0.01;
            }
        }
        let mut v = View::default();
        v.cuboid(
            2,
            "shared_display_ground",
            [0., -1.6, -0.07],
            [24., 19., 0.14],
            [0.31, 0.36, 0.42, 1.],
        );
        layer.draw(&mut v, f, &owners)?;
        let mut notes = vec![];
        let unit = 1.0 / max;
        let phase = if row.advancing {
            "advance"
        } else {
            "same_tick"
        };
        for (w, term) in row.workers.iter().enumerate() {
            let expected = [
                next[w] - prior[w],
                flux[w],
                fixture::GRAVITY[2] * mass[w] * 0.01,
                pushed[w],
                ground[w],
            ];
            if w != term.owner
                || term
                    .vertical()
                    .into_iter()
                    .zip(expected)
                    .any(|(a, b)| !close(a, b))
                || term.residual_ns.iter().any(|&x| !close(x, 0.))
            {
                return Err(format!(
                    "vertical physical budget disagrees on worker {w} tick {}",
                    row.tick
                )
                .into());
            }
            let base = 1_000_000 + i as u64 * 158 + w as u64 * 16;
            let x = (w as f64 - (width as f64 - 1.) / 2.) * 2.65;
            v.cuboid(
                base,
                &format!("worker_{w}_signed_origin"),
                [x, -7.4, 0.15],
                [0.025, 3., 0.12],
                display::OWNER_COLORS[w],
            );
            for (j, value) in term.vertical().into_iter().enumerate() {
                let size = (unit * value).abs().max(0.025);
                let name = format!(
                    "{phase}_tick_{}_w{w}_{}_{value:.12e}_N_s",
                    row.tick, LABELS[j]
                );
                v.cuboid(
                    base + 2 + 2 * j as u64,
                    &name,
                    [x + unit * value / 2., -6.25 - j as f64 * 0.57, 0.30],
                    [size, 0.30, 0.30],
                    [
                        [0.90, 0.82, 0.34, 1.],
                        [0.69, 0.43, 0.85, 1.],
                        [0.39, 0.65, 0.92, 1.],
                        [0.92, 0.48, 0.21, 1.],
                        [0.18, 0.70, 0.51, 1.],
                    ][j],
                );
            }
            if !row.advancing && flux[w] != 0. {
                notes.push((w, flux[w]));
            }
        }
        let label = if row.advancing {
            if pushed.iter().any(|&x| x != 0.) {
                "held_launch"
            } else {
                "physical_step"
            }
        } else if notes.is_empty() {
            if episode["restored_at"] == row.tick {
                "pool_file_resumed"
            } else {
                "worker_recovery"
            }
        } else {
            "warm_handoff"
        };
        let src = episode["moves"]
            .as_array()
            .unwrap()
            .iter()
            .find(|m| m["tick"] == row.tick);
        let annotation = if let Some(m) = src.filter(|_| !row.advancing) {
            format!(
                "native_tick_{}_warm_handoff_{}_members_w{}_to_w{}_zero_external_impulse",
                row.tick, m["members"], m["source"], m["destination"]
            )
        } else {
            format!("native_tick_{}_{label}", row.tick)
        };
        let indicators = 1_000_000 + i as u64 * 158 + 136;
        v.cuboid(
            indicators,
            &annotation,
            [0., 1.8, 0.28],
            [0.62, 0.48, 0.52],
            if row.advancing {
                [0.63, 0.66, 0.72, 1.]
            } else {
                [0.86, 0.62, 0.20, 1.]
            },
        );
        for (j, &n) in f.loaded_pairs.iter().enumerate() {
            v.cuboid(
                indicators + 2 + 2 * j as u64,
                &format!("pad_{}_positive_contacts_{n}", j + 1),
                [-1. + j as f64 * 2., 0., n as f64 / 12.],
                [0.48, 0.3, 0.14],
                [0.10, 0.74, 0.62, 1.],
            );
        }
        let display_ns = (row.tick - 1) * 10_000_000 + held;
        let display_ns = if !row.advancing {
            held += 250_000_000;
            display_ns + 1
        } else {
            display_ns
        };
        samples.push(json!({"tick":row.tick,"advancing":row.advancing,"display_ns":display_ns,"annotation":annotation,"flux":notes}));
        stream.push(v.finish_at(i as u64 + 1, display_ns));
        previous = owners;
        tick = row.tick;
    }
    if tick != 350 || held != 1_000_000_000 {
        return Err("incomplete native trajectory".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        a.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"episode":episode,"scale_ns":max,"frames":samples,"terms":LABELS,"source_sha256":format!("{:x}",Sha256::digest(fs::read(source)?))}),
        )?,
    )?;
    println!(
        "Verified all 354 native momentum rows, {} worker accounts, and independent source-derived signed vertical momentum, floor, held force, gravity and migration.",
        354 * width
    );
    Ok(())
}
