// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Side-by-side native replay at common physical times of two ordinary solver step sizes.
#[path = "support/actuated_convoy.rs"]
#[allow(dead_code)]
mod convoy;
#[path = "support/hinge_rod.rs"]
mod hinge_rod;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
use clap::Parser;
use gz_physics_distributed::multi_sphere::{MultiSphereObservation, MultiSphereSpec};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use primitive_replay_view::View;
use rapier3d_f64::math::{Rotation, Vector};
use serde::Deserialize;
use serde_json::json;
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
    seed: u64,
    /// Whole-millisecond samples from the 1 ms solver.
    coarse: PathBuf,
    /// Whole-millisecond samples from the 0.125 ms solver.
    fine: PathBuf,
    output: PathBuf,
}
#[derive(Deserialize)]
struct Sample {
    time_ms: u64,
    observation: MultiSphereObservation,
    angular_drift_nms: f64,
    interval_normal_impulses_ns: [f64; 2],
}
fn load(p: &PathBuf) -> Result<Vec<Sample>> {
    BufReader::new(File::open(p)?)
        .lines()
        .map(|s| Ok(serde_json::from_str(&s?)?))
        .collect()
}
fn row(view: &mut View, spec: &MultiSphereSpec, s: &Sample, coarse: bool, scale: f64) {
    let (base, label, offset, tag) = if coarse {
        (10, "step_1_ms", -0.85, [0.97, 0.44, 0.10, 1.])
    } else {
        (50, "step_0_125_ms", 0.85, [0.06, 0.78, 0.89, 1.])
    };
    let project = |p: Vector| (p - Vector::new(0.75, 0., 0.5)) * 2. + Vector::new(0., offset, 0.35);
    for (i, (link, body)) in spec
        .hinge
        .links
        .iter()
        .chain(&spec.spheres)
        .zip(
            s.observation
                .hinge
                .links
                .iter()
                .chain(&s.observation.spheres),
        )
        .enumerate()
    {
        let id = base + i as u64 * 6;
        let at = Vector::from(body.position);
        let name = ["base", "tool", "sphere_A", "sphere_B"][i];
        let color = [
            [0.14, 0.48, 0.83, 1.],
            [0.96, 0.43, 0.12, 1.],
            [0.82, 0.20, 0.57, 1.],
            [0.12, 0.73, 0.39, 1.],
        ][i];
        view.add(
            id,
            &format!("{label}_{name}"),
            project(at).to_array(),
            body.orientation,
            json!({"kind":"sphere","radius":2.*link.radius}),
            color,
        );
        if i < 2 {
            let arm = Rotation::from_array(link.orientation).inverse()
                * (Vector::from(spec.hinge.anchor) - Vector::from(link.position));
            if arm.length() > 0. {
                hinge_rod::rod(
                    view,
                    id + 2,
                    &format!("{label}_{name}_to_joint"),
                    project(at),
                    project(at + Rotation::from_array(body.orientation) * arm),
                    2. * arm.length(),
                    color,
                );
            }
        }
    }
    let y = offset + 0.48;
    view.cuboid(
        base + 28,
        &format!("{label}_angular_momentum_error_scale"),
        [0., y, 0.13],
        [2., 0.032, 0.032],
        [0.30, 0.33, 0.37, 1.],
    );
    view.add(
        base + 30,
        &format!("{label}_angular_momentum_error"),
        [-1. + 2. * s.angular_drift_nms / scale, y, 0.17],
        [0., 0., 0., 1.],
        json!({"kind":"sphere","radius":0.055}),
        tag,
    );
}
fn main() -> Result<()> {
    let a = Args::parse();
    let coarse = load(&a.coarse)?;
    let fine = load(&a.fine)?;
    if coarse.is_empty() || coarse.len() != fine.len() {
        return Err("unequal or empty tracks".into());
    }
    let spec = distributed_multi_sphere_spec(&convoy::geometry(a.seed), "convoy.sdf", "default")?;
    let scale = coarse
        .iter()
        .chain(&fine)
        .map(|s| s.angular_drift_nms)
        .fold(0., f64::max);
    if !scale.is_finite() || scale <= 0. {
        return Err("invalid momentum scale".into());
    }
    let mut frames = Vec::new();
    let mut index = Vec::new();
    for (i, (c, f)) in coarse.iter().zip(&fine).enumerate() {
        if c.time_ms != i as u64
            || f.time_ms != c.time_ms
            || c.observation.hinge.tick != c.time_ms
            || f.observation.hinge.tick != f.time_ms
        {
            return Err("misaligned public clocks".into());
        }
        let impact = c
            .interval_normal_impulses_ns
            .into_iter()
            .chain(f.interval_normal_impulses_ns)
            .any(|v| v > 0.);
        if i % 12 != 0 && !impact && i + 1 < coarse.len() {
            continue;
        }
        let mut v = View::default();
        row(&mut v, &spec, c, true, scale);
        row(&mut v, &spec, f, false, scale);
        frames.push(v.finish_at(frames.len() as u64 + 1, c.time_ms * 1_000_000));
        index.push(json!({"time_ms":c.time_ms,"coarse_angular_error_nms":c.angular_drift_nms,"fine_angular_error_nms":f.angular_drift_nms,"coarse_impulses_ns":c.interval_normal_impulses_ns,"fine_impulses_ns":f.interval_normal_impulses_ns}));
    }
    fs::create_dir(&a.output)?;
    gz_sim_cli::export_archive(&a.output.join("replay"), Default::default(), frames)?;
    fs::write(
        a.output.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"seed":a.seed,"rail_full_scale_nms":scale,"frames":index}),
        )?,
    )?;
    Ok(())
}
