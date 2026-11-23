// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare two measured ownership traces at identical physical times.
#[path = "support/hinge_rig.rs"]
mod hinge_rig;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;

use clap::Parser;
use gz_physics_distributed::moving::MovingObservation;
use gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec;
use hinge_rig::{BLUE, GRAY, ID, MAGENTA};
use primitive_replay_view::View;
use serde::Deserialize;
use serde_json::json;
use std::{
    collections::BTreeSet,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Run directory containing the archived whole-articulation enclosure.
    baseline: PathBuf,
    /// Run directory containing the new coordinator's observations.
    revised: PathBuf,
    /// New directory for the side-by-side replay and its measurements.
    output: PathBuf,
}

#[derive(Deserialize)]
struct Summary {
    workers: u8,
    reverse: bool,
    #[serde(default)]
    continuous: bool,
    states_compared: usize,
    exact_states: usize,
    first_split_after_contact: u64,
    last_active_tick: u64,
    state_digest: String,
    frames: Vec<u64>,
}
#[derive(Deserialize)]
struct Row {
    tick: u64,
    owners: [u8; 2],
    queued_effort_nm: Option<f64>,
    worker: MovingObservation,
    direct: MovingObservation,
}

fn read(dir: &Path) -> Result<(Summary, Vec<Row>)> {
    let summary: Summary = serde_json::from_reader(File::open(dir.join("summary.json"))?)?;
    let rows = BufReader::new(File::open(dir.join("states.jsonl"))?)
        .lines()
        .map(|line| Ok(serde_json::from_str::<Row>(&line?)?))
        .collect::<Result<Vec<_>>>()?;
    if rows.len() != summary.states_compared || rows.len() != summary.exact_states {
        return Err("trace length or standalone comparison count differs".into());
    }
    Ok((summary, rows))
}

fn draw_owners(v: &mut View, base: u64, label: &str, y: f64, width: u8, owners: [u8; 2]) {
    let slot_y = |slot: u8| y + (f64::from(width - 1) / 2. - f64::from(slot)) * 0.48;
    for slot in 0..width {
        v.cuboid(
            base + 2 * u64::from(slot),
            &format!("{label}_worker_{slot}"),
            [-2.32, slot_y(slot), 0.16],
            [0.70, 0.42, 0.16],
            GRAY,
        );
    }
    for (index, (owner, name, color)) in [
        (owners[0], "hinge_owner", BLUE),
        (owners[1], "sphere_owner", MAGENTA),
    ]
    .into_iter()
    .enumerate()
    {
        v.add(
            base + 20 + 2 * index as u64,
            &format!("{label}_{name}"),
            [-2.52 + 0.40 * index as f64, slot_y(owner), 0.40],
            ID,
            json!({"kind":"sphere","radius":0.16}),
            color,
        );
    }
}

/// Show the signed command awaiting the next step. The middle tick is zero.
fn draw_effort(v: &mut View, base: u64, label: &str, y: f64, effort: f64, full_scale: f64) {
    let x = 1.64;
    v.cuboid(
        base,
        &format!("{label}_torque_scale"),
        [x, y, 0.33],
        [0.045, 0.96, 0.045],
        GRAY,
    );
    v.cuboid(
        base + 2,
        &format!("{label}_zero_torque"),
        [x, y, 0.33],
        [0.17, 0.045, 0.045],
        GRAY,
    );
    v.add(
        base + 4,
        &format!("{label}_queued_torque"),
        [x, y + 0.48 * (effort / full_scale), 0.36],
        ID,
        json!({"kind":"sphere","radius":0.105}),
        [0.12, 0.86, 0.56, 1.],
    );
}

fn main() -> Result<()> {
    let args = Args::parse();
    let (old, old_rows) = read(&args.baseline)?;
    let (new, new_rows) = read(&args.revised)?;
    if !(2..=8).contains(&old.workers)
        || old.workers != new.workers
        || old.continuous != new.continuous
        || old.reverse != new.reverse
        || old_rows.len() != new_rows.len()
        || old.state_digest != new.state_digest
        || new.first_split_after_contact >= old.first_split_after_contact
        || old.last_active_tick != new.last_active_tick
    {
        return Err("need matching drives and trajectories with an earlier confirmed split".into());
    }
    let spec = distributed_moving_hinge_spec(
        include_str!("../../../examples/distributed_hinge_impact.sdf"),
        "distributed_hinge_impact.sdf",
        "default",
    )?;
    let mut full_scale = 0.0_f64;
    if old.continuous {
        for (a, b) in old_rows.iter().zip(&new_rows) {
            let effort = a.queued_effort_nm.ok_or("missing baseline command")?;
            let revised = b.queued_effort_nm.ok_or("missing revised command")?;
            if !effort.is_finite() || effort.to_bits() != revised.to_bits() {
                return Err(format!("queued torque differs at tick {}", a.tick).into());
            }
            full_scale = full_scale.max(effort.abs());
        }
        if full_scale == 0. {
            return Err("continuous drive has no nonzero command".into());
        }
    }
    let labels = if old.continuous {
        ["torque_fallback", "driven_sweep"]
    } else {
        ["whole_hinge", "link_sweep"]
    };
    let ticks: BTreeSet<_> = old.frames.iter().chain(&new.frames).copied().collect();
    let mut frames = Vec::new();
    let mut joined = [0_u64; 2];
    let mut different = 0;
    let mut split_frame = None;
    for (n, (a, b)) in old_rows.iter().zip(&new_rows).enumerate() {
        if a.tick != n as u64
            || a.tick != b.tick
            || a.worker.hinge.tick != a.tick
            || b.worker.hinge.tick != a.tick
            || a.owners
                .into_iter()
                .chain(b.owners)
                .any(|slot| slot >= old.workers)
            || a.worker != b.worker
            || a.direct != a.worker
            || b.direct != b.worker
        {
            return Err(format!("observation differs at tick {n}").into());
        }
        let both = |owners: [u8; 2]| owners[0] == owners[1];
        joined[0] += u64::from(both(a.owners));
        joined[1] += u64::from(both(b.owners));
        different += u64::from(both(a.owners) != both(b.owners));
        if !ticks.contains(&a.tick) {
            continue;
        }
        let ordinal = frames.len() as u64 + 1;
        if a.tick == new.first_split_after_contact {
            if !both(a.owners) || both(b.owners) {
                return Err("release frame ownership disagrees".into());
            }
            split_frame = Some(ordinal);
        }
        let mut view = View::default();
        let y = (f64::from(old.workers) * 0.48 + 0.95) / 2.;
        for (i, (label, row, offset)) in [(labels[0], a, y), (labels[1], b, -y)]
            .into_iter()
            .enumerate()
        {
            let base = 10 + 80 * i as u64;
            hinge_rig::draw(&mut view, &spec, &row.worker, label, base, offset);
            draw_owners(&mut view, base + 38, label, offset, old.workers, row.owners);
            if old.continuous {
                draw_effort(
                    &mut view,
                    base + 64,
                    label,
                    offset,
                    row.queued_effort_nm.unwrap(),
                    full_scale,
                );
            }
        }
        frames.push(view.finish_at(ordinal, a.tick * 1_000_000));
    }
    let split_frame = split_frame.ok_or("missing split frame")?;
    if frames.len() != ticks.len() {
        return Err("frame outside the common trace".into());
    }
    let gained = joined[0]
        .checked_sub(joined[1])
        .ok_or("coupled residency increased")?;
    fs::create_dir(&args.output)?;
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), frames)?;
    let mut summary = json!({"workers":old.workers,"reverse":old.reverse,"digest":old.state_digest,
        "matched_states":old_rows.len(),"last_active_tick":old.last_active_tick,
        "old_split":old.first_split_after_contact,"new_split":new.first_split_after_contact,
        "old_joined_ticks":joined[0],"new_joined_ticks":joined[1],"different_ownership_ticks":different,
        "independent_ticks_gained":gained,"release_frame":split_frame,"frames":ticks});
    if old.continuous {
        summary["continuous"] = json!(true);
        summary["effort_full_scale_nm"] = json!(full_scale);
    }
    fs::write(
        args.output.join("comparison.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}
