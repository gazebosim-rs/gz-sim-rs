// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare committed native island membership for the same dense encounter at the same times.
use clap::Parser;
use gz_physics_distributed::physics::{Body, Primitive};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    f64::consts::{FRAC_1_SQRT_2, FRAC_PI_8},
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// JSONL from the prior reach bound.
    prior: PathBuf,
    /// JSONL from the tighter reach bound.
    current: PathBuf,
    /// New read-only gz-web-rs recording directory.
    output: PathBuf,
}

#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: Vec<Body>,
    owners: Vec<usize>,
    components: Vec<Vec<u64>>,
}

fn read(path: &Path) -> Result<Vec<Frame>> {
    let mut frames = Vec::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let f: Frame = serde_json::from_str(&line?)?;
        let ids: BTreeSet<_> = f.bodies.iter().map(|b| b.id).collect();
        let members: Vec<_> = f.components.iter().flatten().copied().collect();
        if f.tick != frames.len() as u64
            || ids.len() != f.bodies.len()
            || !ids.contains(&1)
            || !ids.contains(&2)
            || f.owners.len() != ids.len()
            || f.owners.iter().any(|&w| w >= 4)
            || members.len() != ids.len()
            || members.iter().copied().collect::<BTreeSet<_>>() != ids
            || f.bodies
                .iter()
                .any(|b| b.primitive != Primitive::Sphere || b.shape().is_err())
        {
            return Err("expected complete four-worker sphere frames starting at zero".into());
        }
        frames.push(f);
    }
    if frames.len() < 2 {
        return Err("empty or incomplete trajectory".into());
    }
    Ok(frames)
}

// Use a rigid, constant display transform for each copy. Rotate the diagonal
// flight path onto display X so both lanes can share a single camera.
fn pose(body: &Body, center: f64, lane: f64) -> ([f64; 3], [f64; 4]) {
    let [x, y, z] = body.position;
    let [i, j, k, w] = body.orientation;
    let (s, c) = (-FRAC_PI_8).sin_cos();
    (
        [
            (x + y) * FRAC_1_SQRT_2 - center,
            (y - x) * FRAC_1_SQRT_2 + lane,
            z - 3. + lane,
        ],
        [c * i - s * j, c * j + s * i, c * k + s * w, c * w - s * k],
    )
}

fn main() -> Result<()> {
    let args = Args::parse();
    let old = read(&args.prior)?;
    let new = read(&args.current)?;
    if old.len() != new.len() || old[0].bodies != new[0].bodies {
        return Err("comparison requires identical seeds and public times".into());
    }
    let initial: BTreeMap<_, _> = old[0].bodies.iter().map(|b| (b.id, b.size)).collect();
    let (lo, hi) = new
        .iter()
        .chain(&old)
        .flat_map(|f| &f.bodies)
        .map(|b| (b.position[0] + b.position[1]) * FRAC_1_SQRT_2)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| {
            (lo.min(x), hi.max(x))
        });
    let center = (lo + hi) * 0.5;
    let mut identities = BTreeMap::new();
    let mut next_model = 2_u64;
    let mut frames = Vec::new();
    for (a, b) in old.iter().zip(&new) {
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        for (frame, lane, label) in [(a, 1.45, "prior"), (b, -1.45, "tight")] {
            let linked: BTreeSet<_> = frame
                .components
                .iter()
                .filter(|c| c.contains(&1) || c.contains(&2))
                .flatten()
                .copied()
                .collect();
            for (body, &owner) in frame.bodies.iter().zip(&frame.owners) {
                if initial.get(&body.id) != Some(&body.size) {
                    return Err("body identity or radius changed".into());
                }
                let core = body.id <= 2;
                let attached = linked.contains(&body.id);
                let color = if core {
                    [0.13, 0.69, 0.45, 1.]
                } else if attached {
                    [1., 0.43, 0.08, 1.]
                } else {
                    [0.12, 0.55, 0.98, 1.]
                };
                let state = if core {
                    "loaded_pair"
                } else if attached {
                    "linked"
                } else {
                    "separate"
                };
                let entry = identities.entry((label, body.id)).or_insert((None, 0));
                if entry.0 != Some((owner, attached)) {
                    *entry = (Some((owner, attached)), next_model);
                    next_model += 2;
                }
                let model = entry.1;
                let name = format!("default::{label}_body_{}_worker_{owner}_{state}", body.id);
                let (t, r) = pose(body, center, lane);
                visuals.push(json!({"id":model+1,"name":format!("{name}::sphere"),"translation":t,
                    "rotation":r,"shape":{"kind":"sphere","radius":body.size[0]*0.5},"color":color}));
                models.push(json!({"model_id":model,"name":name,"visual_ids":[model+1],"can_remove":false,"can_set_state":false}));
            }
        }
        let sequence = a.tick + 1;
        let scene: Value = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":a.tick/100,
            "nanoseconds":(a.tick%100)*10_000_000,"sequence":sequence,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"models":models,"templates":[]});
        frames.push((scene.to_string(), catalog.to_string()));
    }
    gz_sim_cli::export_archive(&args.output, Default::default(), frames)?;
    println!(
        "Exported {} synchronized frames of {} physical bodies per lane.",
        new.len(),
        initial.len()
    );
    Ok(())
}
