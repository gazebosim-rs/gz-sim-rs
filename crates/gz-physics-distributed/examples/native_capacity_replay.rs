// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Visualize measured native64 dense contacts and the committed barrier of a refused native32 advance.
#[path = "support/canonical_replay.rs"]
mod canonical_replay;
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[path = "support/resident_replay.rs"]
mod resident_replay;
use clap::{Parser, ValueEnum};
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport, Primitive};
use primitive_replay_view::View;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeSet,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
type Rendered = (Vec<(String, String)>, Value);
#[derive(Clone, Copy, ValueEnum)]
enum Mode {
    Dense,
    Admission,
    CanonicalMigration,
    CanonicalReceipt,
    ResidentSites,
    ResidentHandoff,
}
#[derive(Parser)]
struct Args {
    #[arg(long, value_enum)]
    mode: Mode,
    #[arg(long)]
    evidence: PathBuf,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: Vec<Body>,
    owners: Vec<u8>,
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
    trajectory_sha256_sample: String,
    #[serde(default)]
    control_sha256_sample: Option<String>,
    #[serde(default)]
    generations: Vec<u64>,
    #[serde(default)]
    components: Value,
}
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
fn capture(path: &Path, n: usize, windows: &[(u64, u64)]) -> Result<(Vec<Frame>, String)> {
    let mut frames = vec![];
    let mut sha = Sha256::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let f: Frame = serde_json::from_str(&line?)?;
        if f.bodies.len() != n
            || f.owners.len() != n
            || f.bodies
                .iter()
                .enumerate()
                .any(|(i, b)| b.id as usize != i + 1)
            || frames.last().is_some_and(|last: &Frame| {
                last.tick + 1 != f.tick
                    && !windows
                        .iter()
                        .any(|(begin, end)| last.tick == *begin && f.tick == *end)
            })
        {
            return Err("invalid capacity trace".into());
        }
        let raw = bincode::serialize(&(f.tick, &f.bodies, &f.pairs, &f.floors))?;
        if format!("{:x}", Sha256::digest(&raw)) != f.trajectory_sha256_sample {
            return Err("native sample digest mismatch".into());
        }
        sha.update(raw);
        frames.push(f);
    }
    Ok((frames, format!("{:x}", sha.finalize())))
}
#[derive(Serialize)]
struct Transform {
    center: [f64; 2],
    scale: f64,
    offset_y: f64,
}
impl Transform {
    fn pos(&self, p: [f64; 3]) -> [f64; 3] {
        [
            (p[0] - self.center[0]) * self.scale,
            (p[1] - self.center[1]) * self.scale + self.offset_y,
            p[2] * self.scale,
        ]
    }
}
struct Bodies {
    tag: &'static str,
    id: u64,
    transform: Transform,
    previous: Vec<Option<u8>>,
    generations: Vec<u64>,
}
impl Bodies {
    fn new(tag: &'static str, id: u64, transform: Transform, n: usize) -> Self {
        Self {
            tag,
            id,
            transform,
            previous: vec![None; n],
            generations: vec![0; n],
        }
    }
    fn draw(
        &mut self,
        v: &mut View,
        f: &Frame,
        highlight: Option<&BTreeSet<u64>>,
        selected: Option<&BTreeSet<u64>>,
    ) -> Result<()> {
        for (b, &owner) in f.bodies.iter().zip(&f.owners) {
            if selected.is_some_and(|ids| !ids.contains(&b.id)) {
                continue;
            }
            let i = b.id as usize - 1;
            if self.previous[i].is_some_and(|w| w != owner) {
                self.generations[i] += 1;
            }
            self.previous[i] = Some(owner);
            let size = b.size.map(|s| s * self.transform.scale);
            let shape = match b.primitive {
                Primitive::Box => json!({"kind":"box","size":size}),
                Primitive::Sphere => json!({"kind":"sphere","radius":size[0]/2.}),
                _ => return Err("unexpected fixture shape".into()),
            };
            let color = if highlight.is_some_and(|ids| !ids.contains(&b.id)) {
                [0.51, 0.55, 0.59, 1.]
            } else {
                *[
                    [0.14, 0.48, 0.86, 1.],
                    [0.98, 0.50, 0.11, 1.],
                    [0.62, 0.30, 0.76, 1.],
                    [0.09, 0.65, 0.45, 1.],
                ]
                .get(owner as usize)
                .ok_or("unexpected worker slot")?
            };
            v.add(
                self.id + 2 * b.id + 1000 * self.generations[i],
                &format!("{}_body_{}_worker_{owner}", self.tag, b.id),
                self.transform.pos(b.position),
                b.orientation,
                shape,
                color,
            );
        }
        Ok(())
    }
}
fn floor(v: &mut View, size: [f64; 3]) {
    v.cuboid(
        2,
        "shared_ground_display",
        [0., 0., -0.06],
        size,
        [0.28, 0.34, 0.40, 1.],
    );
}
fn dense(root: &Path) -> Result<Rendered> {
    let dir = root.join("dense-clip");
    let summary = read(&dir.join("summary.json"))?;
    let (frames, digest) = capture(&dir.join("frames.jsonl"), 64, &[])?;
    if summary["trajectory_sha256"] != digest
        || summary["exact_native_frames"] != frames.len()
        || summary["native_step_seconds"] != 0.01
        || summary["ground_z"] != 0
    {
        return Err("dense oracle summary mismatch".into());
    }
    let mut bodies = Bodies::new(
        "dense",
        10_000,
        Transform {
            center: [0., 0.],
            scale: 3.,
            offset_y: 0.7,
        },
        64,
    );
    let mut out = vec![];
    let mut pair_count = 0;
    let mut floor_count = 0;
    let int = |id: u64| {
        let k = id - 1;
        (2..=5).contains(&(k % 8)) && (2..=5).contains(&(k / 8))
    };
    for f in &frames {
        let mut v = View::default();
        floor(&mut v, [15., 17., 0.12]);
        bodies.draw(&mut v, f, None, None)?;
        let owner = f.owners[0];
        let generation = f.generations[0];
        if f.owners.iter().any(|w| *w != owner) || f.generations.iter().any(|g| *g != generation) {
            return Err("partial dense island ownership".into());
        }
        let base = 100_000 + f.tick * 160;
        let mut painted = 0;
        for p in f.pairs.iter().filter(|p| int(p.a) && int(p.b)) {
            let a = f.bodies[p.a as usize - 1].position;
            let b = f.bodies[p.b as usize - 1].position;
            let mut pos = bodies
                .transform
                .pos(std::array::from_fn(|i| (a[i] + b[i]) / 2.));
            pos[2] += 0.58;
            let color = if p.normal_impulse > 0. {
                [0.96, 0.80, 0.20, 1.]
            } else {
                [0.50, 0.55, 0.60, 1.]
            };
            v.cuboid(
                base + 2 * painted,
                &format!(
                    "tick_{}_central_pair_{}_{}_normal_{:.6}_N_s",
                    f.tick, p.a, p.b, p.normal_impulse
                ),
                pos,
                [0.14, 0.14, 0.13],
                color,
            );
            painted += 1;
        }
        if painted > 32 {
            return Err("dense contact detail exceeds the viewer budget".into());
        }
        let rows = [
            (
                "pair",
                f.pairs.len(),
                f.pairs.iter().filter(|p| p.normal_impulse > 0.).count(),
                112.,
            ),
            (
                "floor",
                f.floors.len(),
                f.floors.iter().filter(|p| p.normal_impulse > 0.).count(),
                64.,
            ),
        ];
        for (j, (name, n, positive, max)) in rows.into_iter().enumerate() {
            let width = 5. * positive as f64 / max;
            let origin = [-6., 1.][j];
            v.cuboid(
                base + 80 + 2 * j as u64,
                &format!(
                    "tick_{}_{}_receipts_{}_positive_{}",
                    f.tick, name, n, positive
                ),
                [origin + width / 2., -6.2, 0.24],
                [if positive == 0 { 0.1 } else { width }, 0.28, 0.32],
                [[0.96, 0.80, 0.20, 1.], [0.10, 0.73, 0.50, 1.]][j],
            );
        }
        if let Some(event) = summary["events"]
            .as_array()
            .unwrap()
            .iter()
            .rev()
            .find(|x| x["tick"].as_u64().unwrap() <= f.tick)
        {
            let at = event["tick"].as_u64().unwrap();
            let name = event["operation"].as_str().unwrap().replace(' ', "_");
            v.cuboid(
                base + 90,
                &format!("tick_{at}_{name}_owner_{owner}_generation_{generation}"),
                [0., 6.6, 0.25],
                [0.64, 0.45, 0.50],
                [0.84, 0.43, 0.18, 1.],
            );
        }
        pair_count += f.pairs.len();
        floor_count += f.floors.len();
        out.push(v.finish(f.tick));
    }
    if summary["pair_receipts"] != pair_count || summary["floor_receipts"] != floor_count {
        return Err("dense receipt totals disagree".into());
    }
    Ok((
        out,
        json!({"mode":"dense","summary":summary,"panels":[bodies.transform],"frames":frames.len()}),
    ))
}
fn admission(root: &Path) -> Result<Rendered> {
    let comparison = read(&root.join("capacity-clip-comparison.json"))?;
    let (old, _) = capture(&root.join("standard-capacity-clip/capacity.jsonl"), 62, &[])?;
    let (new, _) = capture(&root.join("adjacent-qualified/capacity.jsonl"), 62, &[])?;
    let held = old.last().ok_or("missing old publication")?;
    let at = held.tick + 1;
    if comparison["old_last_native_tick"] != held.tick
        || comparison["old_first_refused_tick"] != at
        || comparison["new_first_admitted_extended_tick"] != at
        || comparison["overlap_identical_physics_ownership_components"] != old.len()
        || old.len() >= new.len()
        || old.iter().zip(&new).any(|(a, b)| {
            a.tick != b.tick
                || a.owners != b.owners
                || a.components != b.components
                || a.trajectory_sha256_sample != b.trajectory_sha256_sample
        })
    {
        return Err("capacity prefix differs across the two measured runs".into());
    }
    let groups: Vec<Vec<u64>> =
        serde_json::from_value(comparison["native32_prospective_group_ids"].clone())?;
    let highlight: BTreeSet<_> = groups
        .iter()
        .find(|g| g.len() > 32)
        .ok_or("missing rejected component")?
        .iter()
        .copied()
        .collect();
    if highlight.len() != 33 {
        return Err("unexpected rejected group size".into());
    }
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for b in new.iter().flat_map(|f| &f.bodies) {
        for i in 0..2 {
            lo[i] = lo[i].min(b.position[i] - b.size[i] / 2.);
            hi[i] = hi[i].max(b.position[i] + b.size[i] / 2.);
        }
    }
    let center = [(lo[0] + hi[0]) / 2., (lo[1] + hi[1]) / 2.];
    let scale = (23. / (hi[0] - lo[0])).min(8. / (hi[1] - lo[1]));
    let mut panels = [(0, "native32", 5.6), (1, "native64", -5.6)].map(|(i, name, y)| {
        Bodies::new(
            name,
            10_000 + i * 50_000,
            Transform {
                center,
                scale,
                offset_y: y,
            },
            62,
        )
    });
    let mut out = vec![];
    for (i, f) in new.iter().enumerate() {
        let mut v = View::default();
        floor(&mut v, [26., 22., 0.12]);
        let prior = old.get(i).unwrap_or(held);
        for (arm, frame) in [prior, f].into_iter().enumerate() {
            panels[arm].draw(&mut v, frame, Some(&highlight), None)?;
            let n = frame
                .components
                .as_array()
                .unwrap()
                .iter()
                .flat_map(|w| w.as_array().unwrap())
                .map(|g| g["ids"].as_array().unwrap().len())
                .max()
                .unwrap();
            let label = if arm == 0 && f.tick >= at {
                "refused_33_retains_barrier"
            } else {
                "committed"
            };
            let width = 6.5 * n as f64 / 64.;
            v.cuboid(
                200_000 + i as u64 * 8 + 2 * arm as u64,
                &format!(
                    "profile_{}_{}_native_tick_{}_largest_component_{}",
                    [32, 64][arm],
                    label,
                    frame.tick,
                    n
                ),
                [-3.25 + width / 2., [0.86, -10.32][arm], 0.25],
                [width, 0.28, 0.34],
                [[0.43, 0.70, 0.94, 1.], [0.92, 0.69, 0.17, 1.]][arm],
            );
        }
        if f.tick >= at {
            v.cuboid(
                4000,
                "native32_refused_33_body_prospective_solve_native64_advanced",
                [5., 0., 0.25],
                [0.75, 0.45, 0.50],
                [0.90, 0.35, 0.14, 1.],
            );
        }
        out.push(v.finish(f.tick - new[0].tick));
    }
    let markers = panels.map(|p| p.transform);
    Ok((
        out,
        json!({"mode":"admission","summary":comparison,"panels":markers,"highlight_ids":highlight,"frames":new.len()}),
    ))
}
fn main() -> Result<()> {
    let args = Args::parse();
    let (stream, index) = match args.mode {
        Mode::ResidentSites => resident_replay::render(&args.evidence, false)?,
        Mode::ResidentHandoff => resident_replay::render(&args.evidence, true)?,
        Mode::CanonicalMigration => canonical_replay::render(&args.evidence, false)?,
        Mode::CanonicalReceipt => canonical_replay::render(&args.evidence, true)?,
        Mode::Dense => dense(&args.evidence)?,
        Mode::Admission => admission(&args.evidence)?,
    };
    fs::create_dir(&args.directory)?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        args.directory.join("index.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    println!(
        "Wrote {} native source frames for {}.",
        index["frames"], index["mode"]
    );
    Ok(())
}
