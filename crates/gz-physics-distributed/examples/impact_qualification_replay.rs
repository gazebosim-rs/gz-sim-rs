// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Visualize independently qualified coarse, scene-wide and island-refined planar impacts.
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
use clap::Parser;
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport};
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
    #[arg(long)]
    evidence: PathBuf,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Member {
    owner: u8,
    body: Body,
}
#[derive(Deserialize)]
struct Source {
    tick: u64,
    event: String,
    members: Vec<Member>,
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
    planar: Planar,
}
#[derive(Deserialize)]
struct Planar {
    angular_z_nms: f64,
    kinetic_horizontal_j: f64,
}
const RATES: [&str; 3] = ["coarse", "scene", "island"];
const OWNER: [[f64; 4]; 4] = [
    [0.14, 0.48, 0.86, 1.],
    [0.98, 0.50, 0.11, 1.],
    [0.62, 0.30, 0.76, 1.],
    [0.09, 0.65, 0.45, 1.],
];
fn phys(f: &Source) -> Result<Vec<u8>> {
    Ok(bincode::serialize(&(
        f.tick,
        f.members.iter().map(|m| &m.body).collect::<Vec<_>>(),
        &f.pairs,
        &f.floors,
    ))?)
}
fn source(path: &Path) -> Result<Vec<Source>> {
    let mut rows = vec![];
    let mut previous = Vec::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let f: Source = serde_json::from_str(&line?)?;
        if f.members.len() != 3
            || f.members
                .iter()
                .enumerate()
                .any(|(i, m)| m.body.id != i as u64 + 1 || m.owner >= 4)
        {
            return Err("unexpected impact membership".into());
        }
        let mut l = 0.;
        let mut e = 0.;
        for m in &f.members[..2] {
            let b = &m.body;
            let [x, y, _] = b.position;
            let [vx, vy, _] = b.linear;
            let omega = b.angular[2];
            let inertia = 0.4 * b.mass * (b.size[0] / 2.).powi(2);
            l += b.mass * (x * vy - y * vx) + inertia * omega;
            e += 0.5 * b.mass * (vx * vx + vy * vy) + 0.5 * inertia * omega * omega;
        }
        if (f.planar.angular_z_nms - l).abs() > 1e-12
            || (f.planar.kinetic_horizontal_j - e).abs() > 1e-12
        {
            return Err("source energy/angular momentum differ from sphere observations".into());
        }
        let current = phys(&f)?;
        if let Some(old) = rows.last() {
            let old: &Source = old;
            if f.tick == old.tick {
                if current != previous
                    || !matches!(
                        f.event.as_str(),
                        "file_resume" | "warm_handoff_after_prepare_death"
                    )
                {
                    return Err("recovery changed same-tick physical state".into());
                }
            } else if f.tick != old.tick + 1 || f.event != "step" {
                return Err("missing or reordered impact frame".into());
            }
        }
        previous = current;
        rows.push(f);
    }
    if rows.len() != 153 || rows[0].tick != 0 || rows[152].tick != 150 {
        return Err("incomplete impact capture".into());
    }
    Ok(rows)
}
fn main() -> Result<()> {
    let a = Args::parse();
    let mut captures = vec![];
    let mut hashes = vec![];
    let summary: Vec<Value> =
        serde_json::from_reader(File::open(a.evidence.join("summary.json"))?)?;
    if summary.len() != 3 {
        return Err("missing fidelity comparison".into());
    }
    for (i, mode) in RATES.into_iter().enumerate() {
        let p = a.evidence.join(format!("{mode}-frames.jsonl"));
        hashes.push(format!("{:x}", Sha256::digest(fs::read(&p)?)));
        let frames = source(&p)?;
        let t = summary[i]["first_positive_impact_tick"]
            .as_u64()
            .ok_or("missing impact tick")?;
        let f = frames
            .iter()
            .find(|x| x.tick == t && x.event == "step")
            .ok_or("missing actual impulse")?;
        if summary[i]["rate"] != mode
            || summary[i]["width"] != 4
            || f.pairs
                .iter()
                .find(|p| p.a == 1 && p.b == 2)
                .is_none_or(|p| summary[i]["impact_pair_impulse_ns"] != p.normal_impulse)
        {
            return Err("source hit differs from summarized impulse".into());
        }
        captures.push(frames);
    }
    let ys = [2.5, 0., -2.5];
    let mut last = [[None; 2]; 3];
    let mut generation = [[0usize; 2]; 3];
    let mut snapshots = vec![];
    let mut timeline = vec![];
    let mut held = 0;
    for (i, reference) in captures[0].iter().enumerate() {
        let tick = reference.tick;
        let event = &reference.event;
        if !matches!(event.as_str(), "initial" | "step") {
            held += 200_000_000;
        }
        let wall = tick * 10_000_000 + held;
        let mut v = View::default();
        v.cuboid(
            2,
            "three_fidelity_rows_display_floor",
            [0., 0., -0.06],
            [12., 9., 0.12],
            [0.31, 0.36, 0.42, 1.],
        );
        let mut details = vec![];
        for (j, mode) in RATES.into_iter().enumerate() {
            let f = &captures[j][i];
            let refdata = &summary[j];
            if tick != f.tick || event != &f.event {
                return Err("mode stages cannot be synchronized".into());
            }
            let base = 1_000_000 + i as u64 * 160 + j as u64 * 40;
            let y = ys[j];
            for (k, item) in f.members[..2].iter().enumerate() {
                let b = &item.body;
                let w = item.owner;
                if last[j][k].is_some_and(|old| old != w) {
                    generation[j][k] += 1;
                }
                last[j][k] = Some(w);
                let pos = [b.position[0] - 2.4, b.position[1] + y, b.position[2]];
                v.add(
                    10_000 + (j * 1000 + generation[j][k] * 16 + k * 2) as u64,
                    &format!("{mode}_body_{}_owner_{w}", b.id),
                    pos,
                    b.orientation,
                    json!({"kind":"sphere","radius":b.size[0]/2.}),
                    OWNER[w as usize],
                );
                let [vx, vy, _] = b.linear;
                let len = vx.hypot(vy) * 0.48;
                let angle = vy.atan2(vx);
                let half = angle / 2.;
                let pointer = [
                    pos[0] + 0.24 * vx,
                    pos[1] + 0.24 * vy,
                    pos[2] + b.size[2] / 2. + 0.04,
                ];
                v.add(
                    base + 2 * k as u64,
                    &format!(
                        "{mode}_tick_{tick}_sphere_{}_vx_{vx:.8}_vy_{vy:.8}_m_s",
                        b.id
                    ),
                    pointer,
                    [0., 0., half.sin(), half.cos()],
                    json!({"kind":"box","size":[len.max(0.025),0.045,0.04]}),
                    [[0.94, 0.77, 0.33, 1.], [0.80, 0.87, 0.96, 1.]][k],
                );
            }
            let measured = f
                .pairs
                .iter()
                .find(|p| p.a == 1 && p.b == 2)
                .map(|p| p.normal_impulse);
            let normal = measured.unwrap_or(0.);
            let error = refdata["max_velocity_error_m_s"]
                .as_f64()
                .ok_or("missing analytic velocity comparison")?;
            let numbers = [
                normal,
                f.planar.kinetic_horizontal_j,
                f.planar.angular_z_nms,
                error,
            ];
            let units = [
                "pair_normal_impulse_N_s",
                "planar_kinetic_J",
                "angular_about_z_N_m_s",
                "final_max_velocity_error_m_s",
            ];
            let scales = [1.4, 0.65, 1.5, 180.];
            for h in 0..4 {
                let x = numbers[h];
                let size = (x * scales[h]).abs().max(0.025);
                v.cuboid(
                    base + 8 + 2 * h as u64,
                    &format!("{mode}_tick_{tick}_{}_{x:.14e}", units[h]),
                    [0.45 + x * scales[h] / 2., y + 0.62 - h as f64 * 0.34, 0.20],
                    [size, 0.16, 0.22],
                    [
                        [0.95, 0.73, 0.29, 1.],
                        [0.15, 0.67, 0.46, 1.],
                        [0.65, 0.43, 0.80, 1.],
                        [0.39, 0.64, 0.90, 1.],
                    ][h],
                );
            }
            let continuum = refdata["continuum"]["impact_ns"].as_f64().unwrap();
            v.cuboid(
                base + 18,
                &format!("{mode}_continuum_normal_impulse_{continuum:.14e}_N_s"),
                [0.45 + 1.4 * continuum, y + 0.62, 0.25],
                [0.035, 0.30, 0.30],
                [0.88, 0.89, 0.93, 1.],
            );
            details.push(json!({"normal_impulse":measured,"angular_z":f.planar.angular_z_nms,"kinetic":f.planar.kinetic_horizontal_j,"velocity_error":error}));
        }
        v.cuboid(
            2_000_000 + i as u64 * 2,
            &format!("native_tick_{tick}_{event}"),
            [-0.25, 3.65, 0.20],
            [0.3, 0.24, 0.35],
            if event == "step" {
                [0.38, 0.49, 0.59, 1.]
            } else {
                [0.88, 0.55, 0.22, 1.]
            },
        );
        snapshots.push(v.finish_at(i as u64 + 1, wall));
        timeline.push(json!({"tick":tick,"event":event,"display_ns":wall,"rows":details}));
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), snapshots)?;
    fs::write(
        a.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"summary":summary,"source_sha256":hashes,"rows_y":ys,"velocity_display_seconds":0.48,"frames":timeline}),
        )?,
    )?;
    println!(
        "Verified and exported {} synchronized sample times across all three actual fidelity modes; velocities, planar energy, angular momentum and warm ownership receipt continuity checked.",
        timeline.len()
    );
    Ok(())
}
