// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show actual distributed torsional braking with an independent four-point native contact inset.
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
use clap::Parser;
use gz_physics_distributed::physics::{Body, ContactImpulse};
use primitive_replay_view::View;
use rapier3d_f64::math::{Rotation, Vector};
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
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
    microstep: u8,
    point_on_block_lever_m: [f64; 3],
    normal_on_table: [f64; 3],
    normal_n_s: f64,
    radius_m: f64,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: [Body; 2],
    reference_bodies: [Body; 2],
    owner: usize,
    generations: BTreeMap<u64, u64>,
    pairs: Vec<ContactImpulse>,
    substeps: u8,
    omega_before: f64,
    omega_after: f64,
    angular_reaction_z_nms: f64,
    predicted_twist_limit_nms: f64,
    last_native_point_count: usize,
    native_points: Vec<Point>,
    events: Vec<Value>,
}
fn angular_momentum(b: &Body) -> Vector {
    let r = Rotation::from_array(b.orientation);
    let [x, y, z] = b.size;
    let inertia = Vector::new(y * y + z * z, x * x + z * z, x * x + y * y) * (b.mass / 12.);
    r * (inertia * (r.inverse() * Vector::from(b.angular)))
}
fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}
const COLORS: [[f64; 4]; 4] = [
    [0.15, 0.48, 0.82, 1.],
    [0.95, 0.49, 0.15, 1.],
    [0.59, 0.34, 0.77, 1.],
    [0.17, 0.71, 0.53, 1.],
];
fn main() -> Result<()> {
    let a = Args::parse();
    let summary: Value = serde_json::from_reader(File::open(a.evidence.join("summary.json"))?)?;
    let frames = BufReader::new(File::open(a.evidence.join("frames.jsonl"))?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect::<Result<Vec<Frame>>>()?;
    let initial: Body = serde_json::from_value(summary["box"].clone())?;
    let mu = initial.friction;
    let mut previous = initial;
    let mut owner = 0;
    let mut generation = 0;
    let mut last_event = "before_drive".to_owned();
    let mut hash = Sha256::new();
    let mut stream = Vec::new();
    let mut samples = Vec::new();
    let mut total = 0.;
    let mut actual_events = vec![];
    let mut point_count = 0;
    for (i, f) in frames.iter().enumerate() {
        let t = i as u64 + 1;
        let [table, block] = &f.bodies;
        if t != f.tick
            || f.owner >= 4
            || f.substeps == 0
            || !matches!(f.substeps, 1 | 10)
            || table.id != 1
            || block.id != 2
            || f.bodies != f.reference_bodies
            || f.omega_after.to_bits() != block.angular[2].to_bits()
            || f.native_points
                .iter()
                .any(|p| p.microstep >= f.substeps || p.normal_n_s < 0. || p.radius_m < 0.)
        {
            return Err("native bodies, impulses or fine clock differ".into());
        }
        let encoded = f.bodies.iter().collect::<Vec<_>>();
        hash.update(bincode::serialize(&(t, encoded, &f.pairs))?);
        let applied = f
            .events
            .iter()
            .find(|e| e["kind"] == "angular impulse")
            .map(|e| e["angular_nms"][2].as_f64().unwrap())
            .unwrap_or(0.);
        let momentum = angular_momentum(block);
        let reaction = momentum.z - angular_momentum(&previous).z - applied;
        let energy = 0.5 * Vector::from(block.angular).dot(momentum);
        let inertia_z = block.mass * (block.size[0].powi(2) + block.size[1].powi(2)) / 12.;
        let torque = mu * block.mass * 9.81 * 0.5 * block.size[0].hypot(block.size[1]);
        let ideal = if t >= 31 {
            (summary["angular_impulse_nms"].as_f64().unwrap().abs()
                - torque * (t - 30) as f64 * 0.01)
                .max(0.)
                .powi(2)
                / (2. * inertia_z)
        } else {
            0.
        };
        if !near(reaction, f.angular_reaction_z_nms, 1e-10) {
            return Err(format!("distinct rotating-inertia impulse at {t}").into());
        }
        let normal = f.pairs.iter().map(|p| p.normal_impulse).sum::<f64>();
        let reference_normal = f.native_points.iter().map(|p| p.normal_n_s).sum::<f64>();
        let cap = mu
            * f.native_points
                .iter()
                .map(|p| p.radius_m * p.normal_n_s)
                .sum::<f64>();
        if !near(normal, reference_normal, 1e-9) || !near(cap, f.predicted_twist_limit_nms, 1e-10) {
            return Err("native point totals disagree with actual impulses".into());
        }
        if t >= 31 {
            total += reaction
        }
        for e in &f.events {
            let k = e["kind"].as_str().ok_or("unlabeled physical operation")?;
            if k == "warm transfer" {
                if e["source"] != owner || e["members"] != json!([1, 2]) {
                    return Err("transfer member or source differs".into());
                }
                owner = e["dest"].as_u64().unwrap() as usize;
                generation += 1;
            }
            last_event = format!("{}_at_{t}", k.replace(' ', "_"));
            actual_events.push(json!({"tick":t,"event":e}));
        }
        if owner != f.owner
            || f.generations.get(&1) != Some(&generation)
            || f.generations.get(&2) != Some(&generation)
        {
            return Err("wrong ownership generation".into());
        }
        let mut v = View::default();
        let base = 1_000_000 + t * 100;
        let r = Rotation::from_array(block.orientation);
        v.add(
            10,
            "prescribed_table_lab_pose",
            table.position,
            table.orientation,
            json!({"kind":"box","size":table.size}),
            [0.33, 0.38, 0.42, 1.],
        );
        v.add(
            100 + 10 * generation,
            &format!("block_owner_{owner}_generation_{generation}"),
            block.position,
            block.orientation,
            json!({"kind":"box","size":block.size}),
            COLORS[owner],
        );
        let marker = (Vector::from(block.position)
            + r * Vector::new(block.size[0] * 0.24, 0., block.size[2] * 0.51))
        .to_array();
        v.add(
            300,
            "cuboid_positive_material_x_axis",
            marker,
            block.orientation,
            json!({"kind":"box","size":[0.46*block.size[0],0.035,0.016]}),
            [0.97, 0.80, 0.36, 1.],
        );
        // Inset coordinates are relative solver lever vectors, in world axes. This preserves their signed
        // geometry without inventing the missing absolute COM pose of an intermediate solver microstep.
        v.cuboid(
            400,
            "last_substep_native_lever_inset",
            [-2.25, -2.70, 0.01],
            [2.62, 2.58, 0.035],
            [0.25, 0.29, 0.33, 1.],
        );
        let last = f
            .native_points
            .iter()
            .filter(|p| p.microstep + 1 == f.substeps)
            .collect::<Vec<_>>();
        if last.len() != f.last_native_point_count || last.len() > 4 {
            return Err("final substep corner count differs".into());
        }
        point_count += last.len();
        for (j, p) in last.iter().enumerate() {
            let [x, y, _] = p.point_on_block_lever_m;
            let height = (p.normal_n_s * f64::from(f.substeps) / 0.01 * 0.045).max(0.025);
            let label = format!(
                "tick_{t}_native_corner_{j}_normal_{:.9e}_N_s_radius_{:.9e}_m",
                p.normal_n_s, p.radius_m
            );
            let norm = p.normal_on_table.into_iter().map(|v| v * v).sum::<f64>();
            if !near(norm, 1., 1e-7) {
                return Err("reference contact normal not a unit vector".into());
            }
            v.cuboid(
                base + 2 * j as u64,
                &label,
                [-2.25 + 2.7 * x, -2.70 + 2.7 * y, 0.048 + height / 2.],
                [0.115, 0.115, height],
                [0.97, 0.76, 0.28, 1.],
            );
        }
        let fields = [
            (
                "yaw_rate_rad_s",
                f.omega_after,
                0.28,
                [0.32, 0.70, 0.86, 1.],
            ),
            (
                "distributed_signed_reaction_N_m_s",
                reaction,
                29.,
                [0.95, 0.47, 0.30, 1.],
            ),
            (
                "native_positive_twist_limit_N_m_s",
                cap,
                29.,
                [0.83, 0.68, 0.35, 1.],
            ),
            (
                "cumulative_signed_braking_N_m_s",
                total,
                1.4,
                [0.48, 0.76, 0.60, 1.],
            ),
            (
                "distributed_spin_energy_j",
                energy,
                0.47,
                [0.48, 0.74, 0.50, 1.],
            ),
            (
                "ideal_coulomb_spin_energy_j",
                ideal,
                0.47,
                [0.70, 0.53, 0.82, 1.],
            ),
        ];
        for (j, (name, value, factor, color)) in fields.into_iter().enumerate() {
            v.cuboid(
                base + 20 + j as u64 * 2,
                &format!("tick_{t}_{name}_{value:.9e}"),
                [0.95 + value * factor / 2., -2.10 - j as f64 * 0.37, 0.17],
                [(value * factor).abs().max(0.028), 0.20, 0.24],
                color,
            );
        }
        v.cuboid(
            base + 34,
            &format!("tick_{t}_latest_{last_event}_owner_{owner}_generation_{generation}"),
            [3.34, -2.16, 0.19],
            [0.23, 0.23, 0.28],
            COLORS[owner],
        );
        samples.push(json!({"tick":t,"owner":owner,"generation":generation,"physical_sha256":format!("{:x}",Sha256::digest(bincode::serialize(&(t,f.bodies.iter().collect::<Vec<_>>(),&f.pairs))?)),"reaction":reaction,"spin_energy_j":energy,"ideal_energy_j":ideal,"limit":cap,"total":total,"normal":normal,"omega_before":f.omega_before,"omega_after":f.omega_after,"final_point_count":last.len(),"events":f.events,"last_event":last_event}));
        stream.push(v.finish_at(t, (t - 1) * 10_000_000));
        previous = block.clone();
    }
    if frames.len() != 120
        || summary["trajectory_sha256"] != format!("{:x}", hash.finalize())
        || summary["events"] != json!(actual_events)
        || point_count == 0
        || !near(summary["total_reaction_nms"].as_f64().unwrap(), total, 1e-9)
    {
        return Err("raw archive or total braking qualification failed".into());
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    serde_json::to_writer_pretty(
        File::create(a.directory.join("index.json"))?,
        &json!({"summary":summary,"samples":samples,"final_substep_native_points":point_count}),
    )?;
    println!(
        "Verified 120 distributed poses against native reference, rotated-inertia angular response, total point normal/twist bound, {point_count} displayed final-substep corner levers, both ownership transitions, recovery and shared physical hash."
    );
    Ok(())
}
