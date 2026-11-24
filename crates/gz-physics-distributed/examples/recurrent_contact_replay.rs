// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export two measured cycles of support, release and reimpact on finite physical tables.
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
#[path = "support/recurrent_placement_view.rs"]
mod recurrent_placement_view;
use clap::Parser;
use gz_physics_distributed::physics::{Body, BodyWrench, ContactImpulse, GroundSupport};
use primitive_replay_view::View;
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    evidence: PathBuf,
    /// Compare the physical window with an independent control while displaying automatic decisions.
    #[arg(long)]
    automatic_reference: Option<PathBuf>,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: Vec<Body>,
    owners: Vec<usize>,
    generations: Vec<u64>,
    components: Vec<Vec<Component>>,
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
    wrenches: Vec<BodyWrench>,
    trajectory_sha256_sample: String,
}
#[derive(Deserialize)]
struct Component {
    ids: Vec<u64>,
    contact_history: bool,
}
const OWNER: [[f64; 4]; 4] = [
    [0.14, 0.48, 0.86, 1.],
    [0.98, 0.50, 0.11, 1.],
    [0.62, 0.30, 0.76, 1.],
    [0.09, 0.65, 0.45, 1.],
];
const GOLD: [f64; 4] = [0.98, 0.74, 0.26, 1.];
fn main() -> Result<()> {
    let a = Args::parse();
    let summary: Value = serde_json::from_reader(File::open(a.evidence.join("summary.json"))?)?;
    let first = summary["first_tick"]
        .as_u64()
        .ok_or("missing initial time")?;
    let last = summary["last_tick"].as_u64().ok_or("missing final time")?;
    if last - first != 200
        || summary["loaded_pair"] != json!([1, 2])
        || summary["driven_pair"] != json!([3, 4])
    {
        return Err("unexpected two-cycle source".into());
    }
    let events = summary["events"]
        .as_array()
        .ok_or("missing recovery events")?;
    let mut placement = a
        .automatic_reference
        .as_deref()
        .map(|p| recurrent_placement_view::Placement::read(&a.evidence, p))
        .transpose()?;
    let mut stream = vec![];
    let mut timeline = vec![];
    let mut previous_owners = [None; 4];
    let mut generation = [0; 4];
    for (i, line) in BufReader::new(File::open(a.evidence.join("frames.jsonl"))?)
        .lines()
        .enumerate()
    {
        let f: Frame = serde_json::from_str(&line?)?;
        let t = f.tick;
        if t != first + i as u64
            || f.bodies.len() != 4
            || f.owners.len() != 4
            || f.generations.len() != 4
            || f.components.len() != 4
            || !f.floors.is_empty()
        {
            return Err("incomplete or unverified finite contact frame".into());
        }
        let physical = bincode::serialize(&(t, &f.bodies, &f.pairs, &f.floors))?;
        if format!("{:x}", Sha256::digest(physical)) != f.trajectory_sha256_sample {
            return Err("native physics hash differs".into());
        }
        let mut v = View::default();
        let base = 1_000_000 + 40 * i as u64;
        // Each table keeps the source geometry exactly; the two local origins are brought together.
        for (b_idx, b) in f.bodies.iter().enumerate() {
            let site = b_idx / 2;
            let id = b.id;
            let w = f.owners[b_idx];
            if id != b_idx as u64 + 1 || w >= 4 {
                return Err("incorrect physical ID or owner".into());
            }
            let comp = f.components[w]
                .iter()
                .find(|c| c.ids.contains(&id))
                .ok_or("no native component for displayed body")?;
            if previous_owners[b_idx].is_some_and(|o| o != w) {
                generation[b_idx] += 1;
            }
            previous_owners[b_idx] = Some(w);
            let x = b.position[0] + [12.8, -12.8][site];
            v.add(
                10_000 + 64 * b_idx as u64 + 2 * generation[b_idx],
                &format!(
                    "site{site}_body_{id}_owner_{w}_generation_{}",
                    f.generations[b_idx]
                ),
                [x, b.position[1], b.position[2]],
                b.orientation,
                if b_idx % 2 == 0 {
                    json!({"kind":"box","size":b.size})
                } else {
                    json!({"kind":"sphere","radius":b.size[0]/2.})
                },
                if b_idx % 2 == 0 {
                    let [r, g, b, _] = OWNER[w];
                    [r * 0.7, g * 0.7, b * 0.7, 1.]
                } else {
                    OWNER[w]
                },
            );
            if b_idx % 2 == 0 {
                let components = comp
                    .ids
                    .iter()
                    .map(u64::to_string)
                    .collect::<Vec<_>>()
                    .join("_");
                v.cuboid(
                    base + site as u64 * 2,
                    &format!(
                        "tick_{t}_site_{site}_owner_{w}_warm_{}_native_members_{components}",
                        comp.contact_history
                    ),
                    [[-2.2, 2.2][site], -1.55, 0.16],
                    [0.38 * comp.ids.len() as f64, 0.16, 0.24],
                    OWNER[w],
                );
            }
        }
        let mut receipts = vec![];
        for (site, (a, b)) in [(1, 2), (3, 4)].into_iter().enumerate() {
            let pair = f
                .pairs
                .iter()
                .find(|p| p.a == a && p.b == b)
                .map(|p| p.normal_impulse);
            let label = pair
                .map(|n| format!("{n:.8}_N_s"))
                .unwrap_or_else(|| "absent".into());
            let val = pair.unwrap_or(0.);
            let len = (val * 1.14).max(0.035);
            v.cuboid(
                base + 4 + 2 * site as u64,
                &format!("tick_{t}_pair_{a}_{b}_normal_{label}"),
                [[-3.45, 0.95][site] + len / 2., 1.78, 0.28],
                [len, 0.17, 0.30],
                match pair {
                    None => [0.4, 0.43, 0.47, 1.],
                    Some(0.) => [0.62, 0.58, 0.43, 1.],
                    Some(_) => GOLD,
                },
            );
            receipts.push(pair);
        }
        let drive = f
            .wrenches
            .iter()
            .find(|w| w.id == 4)
            .map_or(0., |w| w.force[2]);
        let gap = f.bodies[3].position[2]
            - f.bodies[3].size[2] / 2.
            - f.bodies[2].position[2]
            - f.bodies[2].size[2] / 2.;
        for (j, (value, scale, unit, color)) in [
            (drive, 0.09, "drive_N", [0.92, 0.38, 0.22, 1.]),
            (gap, 10., "clearance_m", [0.21, 0.76, 0.72, 1.]),
        ]
        .into_iter()
        .enumerate()
        {
            let len = (value * scale).abs().max(0.035);
            v.cuboid(
                base + 12 + 2 * j as u64,
                &format!("tick_{t}_right_{unit}_{value:.8}"),
                [0.95 + len / 2., 2.14 + 0.34 * j as f64, 0.25],
                [len, 0.17, 0.25],
                color,
            );
        }
        let recent = events
            .iter()
            .rev()
            .find(|ev| ev["tick"].as_u64().is_some_and(|n| n <= t));
        let event = recent
            .map(|e| {
                format!(
                    "at_{}_{}",
                    e["tick"],
                    e["operation"].as_str().unwrap().replace(' ', "_")
                )
            })
            .unwrap_or_else(|| "cycle_before_faults".into());
        v.cuboid(
            base + 20,
            &format!("native_tick_{t}_{event}"),
            [-2.2, 2.30, 0.25],
            [0.3, 0.3, 0.36],
            if recent.is_some_and(|e| e["tick"] == t) {
                [0.94, 0.53, 0.19, 1.]
            } else {
                [0.55, 0.62, 0.69, 1.]
            },
        );
        let automatic = placement
            .as_mut()
            .map(|p| p.draw(&mut v, base, i, &f))
            .transpose()?;
        timeline.push(json!({"tick":t,"automatic":automatic,"owners":f.owners,"generations":f.generations,"pair_loads":receipts,"drive_n":drive,"clearance_m":gap,"event":event,"sample_sha256":f.trajectory_sha256_sample}));
        stream.push(v.finish_at(i as u64 + 1, (t - first) * 10_000_000));
    }
    if stream.len() != 201 {
        return Err("missing final two cycles".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        a.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"summary":summary,"samples":timeline,"visual_x_offsets_m":[12.8,-12.8]}),
        )?,
    )?;
    println!(
        "Verified and exported 201 source frames with original physical tick IDs and relative replay clock, both finite pair receipts, actual force, clearance, native component memberships and ownership."
    );
    Ok(())
}
