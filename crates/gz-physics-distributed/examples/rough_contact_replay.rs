// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render loaded contact on a prescribed slab, keeping the public and point-control reactions distinct.
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
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    evidence: PathBuf,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Point {
    sphere_witness_world_m: [f64; 3],
    normal_impulse_on_sphere_n_s: [f64; 3],
    tangent_impulse_on_sphere_n_s: [f64; 3],
}
#[derive(Deserialize)]
struct Reaction {
    normal_n_s: f64,
    tangent_on_sphere: [f64; 3],
    points: Vec<Point>,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    time_ns: u64,
    bodies: [Body; 2],
    owners: [usize; 2],
    generations: [u64; 2],
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
    trajectory_sha256_sample: String,
    native_reaction: Reaction,
    #[serde(default)]
    distributed_horizontal_contact_impulse_n_s: Option<f64>,
    com_impulse_n_s: f64,
    slip_m_s: f64,
    relative_energy_j: f64,
    dissipated_relative_energy_j: f64,
    table_work_j: f64,
}
const OWNER: [[f64; 4]; 4] = [
    [0.14, 0.48, 0.86, 1.],
    [0.98, 0.50, 0.11, 1.],
    [0.62, 0.30, 0.76, 1.],
    [0.09, 0.65, 0.45, 1.],
];
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-10
}
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let [a, b, c] = v;
    let t = [
        2. * (y * c - z * b),
        2. * (z * a - x * c),
        2. * (x * b - y * a),
    ];
    [
        a + w * t[0] + y * t[2] - z * t[1],
        b + w * t[1] + z * t[0] - x * t[2],
        c + w * t[2] + x * t[1] - y * t[0],
    ]
}
fn rod(view: &mut View, id: u64, name: &str, witness: [f64; 3], vector: [f64; 3], color: [f64; 4]) {
    let magnitude = vector.iter().map(|x| x * x).sum::<f64>().sqrt();
    if magnitude < 1e-12 {
        return;
    }
    let [x, y, z] = vector.map(|c| c / magnitude);
    let length = 0.8 * (magnitude / 0.04).asinh();
    // Local +X is aligned with the measured outgoing vector.
    let rotation = if x <= -1. + 1e-12 {
        [0., 0., 1., 0.]
    } else {
        let w = (0.5 * (1. + x)).sqrt();
        [0., -z / (2. * w), y / (2. * w), w]
    };
    let origin = [witness[0], witness[1] - 0.42, witness[2]];
    let center = std::array::from_fn(|k| origin[k] + 0.5 * length * [x, y, z][k]);
    view.add(
        id,
        name,
        center,
        rotation,
        json!({"kind":"box","size":[length,0.022,0.022]}),
        color,
    );
}
fn main() -> Result<()> {
    let a = Args::parse();
    let summary: Value = serde_json::from_reader(File::open(a.evidence.join("summary.json"))?)?;
    let speed = summary["table_speed_m_s"]
        .as_f64()
        .ok_or("missing prescribed speed")?;
    let radius = summary["radius_m"].as_f64().ok_or("missing radius")?;
    let inertia = summary["inertia_kg_m2"]
        .as_f64()
        .ok_or("missing sphere inertia")?;
    let step = summary["step_ms"]
        .as_u64()
        .ok_or("missing native timestep")?;
    let ticks = summary["ticks"].as_u64().ok_or("missing endpoint")?;
    let events = summary["events"].as_array().ok_or("missing events")?;
    let mut hash = Sha256::new();
    let mut stream = vec![];
    let mut samples = vec![];
    let mut start = None;
    let mut last_v = None;
    let mut cumulative = 0.;
    let mut last = [None; 2];
    let mut display_generation = [0; 2];
    for (i, line) in BufReader::new(File::open(a.evidence.join("frames.jsonl"))?)
        .lines()
        .enumerate()
    {
        let f: Frame = serde_json::from_str(&line?)?;
        let t = f.tick;
        let begin = *start.get_or_insert(t);
        let base = 1_000_000 + i as u64 * 64;
        if t != begin + i as u64
            || f.time_ns != t * step * 1_000_000
            || f.bodies[0].id != 1
            || f.bodies[1].id != 2
            || f.owners.iter().any(|w| *w >= 4)
            || !f.floors.is_empty()
        {
            return Err("invalid public physical frame".into());
        }
        let bytes = bincode::serialize(&(t, &f.bodies, &f.pairs, &f.floors))?;
        if format!("{:x}", Sha256::digest(&bytes)) != f.trajectory_sha256_sample {
            return Err("invalid sample hash".into());
        }
        if t > 0 {
            hash.update(bytes);
        }
        let sphere = &f.bodies[1];
        let vx = sphere.linear[0];
        let spin = sphere.angular[1];
        let slip = vx - speed - radius * spin;
        let relative_energy =
            0.5 * sphere.mass * (vx - speed).powi(2) + 0.5 * inertia * spin.powi(2);
        let jx = sphere.mass * (vx - last_v.unwrap_or(vx)) - f.com_impulse_n_s;
        last_v = Some(vx);
        let public_n = f.pairs.first().map(|p| p.normal_impulse);
        if !close(slip, f.slip_m_s)
            || !close(relative_energy, f.relative_energy_j)
            || !close(jx, f.native_reaction.tangent_on_sphere[0])
            || f.distributed_horizontal_contact_impulse_n_s
                .is_some_and(|x| !close(jx, x))
            || !close(public_n.unwrap_or(0.), f.native_reaction.normal_n_s)
        {
            return Err(format!("independent physical comparison failed at {t}").into());
        }
        let mut v = View::default();
        let plate = &f.bodies[0];
        for j in 0..2 {
            let b = &f.bodies[j];
            let w = f.owners[j];
            if last[j].is_some_and(|o| o != w) {
                display_generation[j] += 1;
            }
            last[j] = Some(w);
            let [r, g, bl, _] = OWNER[w];
            let color = if j == 0 {
                [0.7 * r, 0.7 * g, 0.7 * bl, 1.]
            } else {
                OWNER[w]
            };
            v.add(
                10_000 + j as u64 * 200 + display_generation[j] * 2,
                &format!("body_{}_worker_{w}_generation_{}", b.id, f.generations[j]),
                b.position,
                b.orientation,
                if j == 0 {
                    json!({"kind":"box","size":b.size})
                } else {
                    json!({"kind":"sphere","radius":radius})
                },
                color,
            );
        }
        let offset = rotate(sphere.orientation, [0., -radius - 0.012, 0.]);
        let marker = std::array::from_fn(|k| sphere.position[k] + offset[k]);
        v.add(
            14_000,
            "sphere_rotation_diameter_lab_frame",
            marker,
            sphere.orientation,
            json!({"kind":"box","size":[2.*radius,0.025,0.035]}),
            [0.98, 0.86, 0.54, 1.],
        );
        v.cuboid(
            14_002,
            "slab_center_lab_frame",
            [
                plate.position[0],
                plate.position[1] - plate.size[1] / 2. - 0.02,
                plate.position[2],
            ],
            [0.05, 0.04, 0.38],
            [0.9, 0.87, 0.77, 1.],
        );
        let normal_tag = public_n.map_or("absent".to_owned(), |n| format!("{n:.9}_N_s"));
        let n = public_n.unwrap_or(0.);
        let ln = (0.8 * (n / 0.04).asinh()).max(0.035);
        v.cuboid(
            base,
            &format!("tick_{t}_distributed_public_pair_1_2_normal_{normal_tag}"),
            [-3.7 + 0.5 * ln, 1.7, 0.18],
            [ln, 0.16, 0.18],
            [0.91, 0.73, 0.27, 1.],
        );
        let scalar = [
            ("slip_m_s", slip, 0.9),
            ("spin_y_rad_s", spin, 0.36),
            (
                "cumulative_relative_loss_J",
                f.dissipated_relative_energy_j,
                0.9,
            ),
            ("moving_table_work_J", f.table_work_j, 1.4),
            ("distributed_COM_inferred_tangent_N_s", jx, 120.),
        ];
        for (k, (name, value, scale)) in scalar.into_iter().enumerate() {
            let size = (value * scale).abs().max(0.035);
            let origin = if k < 2 { -3.7 } else { 1.0 };
            let y = if k < 2 {
                2.0 + k as f64 * 0.32
            } else {
                1.7 + (k - 2) as f64 * 0.32
            };
            v.cuboid(
                base + 4 + k as u64 * 2,
                &format!("native_tick_{t}_{name}_{value:.12e}"),
                [origin + value * scale / 2., y, 0.18],
                [size, 0.16, 0.18],
                [
                    [0.78, 0.38, 0.64, 1.],
                    [0.31, 0.64, 0.91, 1.],
                    [0.84, 0.50, 0.20, 1.],
                    [0.61, 0.42, 0.75, 1.],
                    [0.18, 0.73, 0.56, 1.],
                ][k],
            );
        }
        if f.native_reaction.points.len() > 1 {
            return Err("expected single sphere-side witness".into());
        }
        for p in &f.native_reaction.points {
            let pos = p.sphere_witness_world_m;
            v.add(
                base + 18,
                &format!(
                    "tick_{t}_direct_control_sphere_witness_x_{:.8}_z_{:.8}_m",
                    pos[0], pos[2]
                ),
                pos,
                [0., 0., 0., 1.],
                json!({"kind":"sphere","radius":0.034}),
                [0.96, 0.82, 0.42, 1.],
            );
            for (k, (tag, d, color)) in [
                (
                    "normal",
                    p.normal_impulse_on_sphere_n_s,
                    [0.91, 0.73, 0.27, 1.],
                ),
                (
                    "tangent",
                    p.tangent_impulse_on_sphere_n_s,
                    [0.18, 0.73, 0.56, 1.],
                ),
            ]
            .into_iter()
            .enumerate()
            {
                rod(
                    &mut v,
                    base + 20 + k as u64 * 2,
                    &format!(
                        "tick_{t}_direct_point_Coulomb_{tag}_on_sphere_{:.8e}_{:.8e}_{:.8e}_N_s",
                        d[0], d[1], d[2]
                    ),
                    pos,
                    d,
                    color,
                );
            }
        }
        if f.com_impulse_n_s != 0. {
            v.cuboid(
                base + 26,
                &format!("tick_{t}_applied_COM_impulse_{:.2}_N_s", f.com_impulse_n_s),
                [sphere.position[0], -1.55, 0.3],
                [0.8, 0.18, 0.2],
                [0.97, 0.47, 0.21, 1.],
            );
        }
        let event = events
            .iter()
            .filter(|e| e["tick"].as_u64().is_some_and(|n| n <= t))
            .max_by_key(|e| e["tick"].as_u64());
        let label = event
            .map(|e| {
                format!(
                    "tick_{}_{}",
                    e["tick"],
                    e["operation"].as_str().unwrap().replace(' ', "_")
                )
            })
            .unwrap_or_else(|| "settling_before_drive".into());
        v.cuboid(
            base + 28,
            &label,
            [-4.2, 1.55, 0.20],
            [0.2, 0.2, 0.3],
            if event.is_some_and(|e| e["tick"] == t) {
                [0.94, 0.48, 0.20, 1.]
            } else {
                [0.51, 0.60, 0.68, 1.]
            },
        );
        cumulative += jx;
        samples.push(json!({"tick":t,"time_ns":f.time_ns,"computed_tangent_n_s":jx,"cumulative_tangent_n_s":cumulative,"public_normal_n_s":public_n,"event":label,"source_sha256":f.trajectory_sha256_sample}));
        stream.push(v.finish_at(i as u64 + 1, (t - begin) * step * 1_000_000));
    }
    if start.is_none()
        || start.unwrap() + samples.len() as u64 != ticks + 1
        || summary["trajectory_sha256"] != format!("{:x}", hash.finalize())
    {
        return Err("incomplete trajectory evidence".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        a.directory.join("index.json"),
        serde_json::to_vec_pretty(&json!({"summary":summary,"samples":samples}))?,
    )?;
    println!(
        "Verified {} raw observations and distributed physics against published SHA, point-control tangents versus COM momentum, local energy, true axial spin and relative contact slip.",
        samples.len()
    );
    Ok(())
}
