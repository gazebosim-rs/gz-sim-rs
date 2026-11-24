// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Display independently captured local native reduced-coordinate states, with verified link kinematics.
#[path = "support/hinge_transfer_view.rs"]
mod hinge_transfer_view;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
use clap::Parser;
use primitive_replay_view::View;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{
    error::Error,
    fs::{self, File},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
type V = [f64; 3];
type Q = [f64; 4];
const BLUE: [f64; 4] = [0.04, 0.30, 0.72, 1.];
const ORANGE: [f64; 4] = [0.95, 0.25, 0.04, 1.];
const DISPLAY_SCALE: f64 = 3.;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    trace: PathBuf,
    #[arg(long)]
    reference: PathBuf,
    #[arg(long)]
    transfer_evidence: Option<PathBuf>,
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Trace {
    mode: String,
    native_step_ns: u64,
    native_source_cut_ns: u64,
    model: u64,
    public_joint: u64,
    native_joint: u64,
    pending_effort_nm: f64,
    later_effort_tick: u64,
    later_effort_nm: f64,
    frames: Vec<Frame>,
}
#[derive(Deserialize, PartialEq)]
struct Frame {
    offset: u64,
    ecs_clock_ns: u64,
    #[serde(flatten)]
    physics: State,
}
#[derive(Deserialize, PartialEq, Serialize)]
struct State {
    links: [Link; 2],
    axis: V,
    q: f64,
    qdot: f64,
    linear_momentum: V,
    angular_momentum: V,
}
#[derive(Deserialize, PartialEq, Serialize)]
struct Link {
    public_link: u64,
    public_visual: u64,
    native_body: u64,
    position: V,
    orientation_xyzw: Q,
    com: V,
    com_velocity: V,
    angular_velocity: V,
    anchor: V,
}
fn add(a: V, b: V) -> V {
    std::array::from_fn(|k| a[k] + b[k])
}
fn sub(a: V, b: V) -> V {
    std::array::from_fn(|k| a[k] - b[k])
}
fn scale(a: V, x: f64) -> V {
    a.map(|v| v * x)
}
fn dot(a: V, b: V) -> f64 {
    a.iter().zip(b).map(|(u, v)| u * v).sum()
}
fn cross([x, y, z]: V, [a, b, c]: V) -> V {
    [y * c - z * b, z * a - x * c, x * b - y * a]
}
fn rotate([x, y, z, w]: Q, p: V) -> V {
    let u = [x, y, z];
    let t = scale(cross(u, p), 2.);
    add(p, add(scale(t, w), cross(u, t)))
}
fn norm(v: V) -> f64 {
    dot(v, v).sqrt()
}
fn world(origin: V, p: V) -> V {
    add(scale(sub(p, origin), DISPLAY_SCALE), [-1.0, 0., 0.9])
}
fn segment(v: &mut View, id: u64, name: &str, a: V, b: V, thickness: f64, color: [f64; 4]) {
    let delta = sub(b, a);
    let len = norm(delta);
    assert!(len > 1e-10);
    let [x, y, z] = scale(delta, 1. / len);
    let q = if x < -1. + 1e-12 {
        [0., 0., 1., 0.]
    } else {
        let w = (0.5 * (1. + x)).sqrt();
        [0., -z / (2. * w), y / (2. * w), w]
    };
    v.add(
        id,
        name,
        scale(add(a, b), 0.5),
        q,
        // Every segment joins two rigidly related points. Keep its immutable shape
        // stable across round-off in the live world transforms.
        json!({"kind":"box","size":[(len*1e9).round()/1e9,thickness,thickness]}),
        color,
    );
}
fn read(p: &Path) -> Result<Trace> {
    Ok(serde_json::from_reader(File::open(p)?)?)
}
fn main() -> Result<()> {
    let a = Args::parse();
    let tr = read(&a.trace)?;
    let other = read(&a.reference)?;
    if tr.mode == other.mode
        || ![&tr, &other].iter().all(|r| {
            matches!(r.mode.as_str(), "source" | "restored")
                && r.frames.len() == 257
                && r.native_step_ns == 1_000_000
                && r.native_source_cut_ns == 120_000_000
                && r.model == 2
                && r.public_joint == 9
                && r.native_joint == 0
                && r.pending_effort_nm == -0.8125
                && r.later_effort_tick == 99
                && r.later_effort_nm == 0.375
        })
    {
        return Err("incompatible local articulated sources".into());
    }
    let transfer = a
        .transfer_evidence
        .as_deref()
        .map(|p| hinge_transfer_view::Transfer::read(p, &tr.mode))
        .transpose()?;
    let label = transfer.as_ref().map_or(tr.mode.as_str(), |v| v.label);
    let origin = tr.frames[0].physics.links[0].position;
    let p0 = tr.frames[0].physics.linear_momentum;
    let l0 = tr.frames[0].physics.angular_momentum;
    let mut stream = vec![];
    let mut samples = vec![];
    let mut max_gap: f64 = 0.;
    let mut p_drift: f64 = 0.;
    let mut l_drift: f64 = 0.;
    for (j, (f, oracle)) in tr.frames.iter().zip(&other.frames).enumerate() {
        let tick = j as u64;
        let time = tick * tr.native_step_ns;
        let state = &f.physics;
        let start = if tr.mode == "source" {
            tr.native_source_cut_ns
        } else {
            0
        };
        let other_start = if other.mode == "source" {
            other.native_source_cut_ns
        } else {
            0
        };
        if f.offset != tick
            || oracle.offset != tick
            || f.ecs_clock_ns != time + start
            || oracle.ecs_clock_ns != time + other_start
            || bincode::serialize(state)? != bincode::serialize(&oracle.physics)?
        {
            return Err(format!("native state or clock mismatch at {tick}").into());
        }
        let [base, tool] = &state.links;
        let [x, y, z, w] = base.orientation_xyzw;
        let [aa, bb, c, d] = tool.orientation_xyzw;
        let q = 2. * (w * c - z * d - x * bb + y * aa).atan2(w * d + x * aa + y * bb + z * c) - 0.7;
        let dq = dot(
            sub(tool.angular_velocity, base.angular_velocity),
            state.axis,
        );
        let gap = norm(sub(tool.anchor, base.anchor));
        max_gap = max_gap.max(gap);
        if (q - state.q).abs() > 1e-10 || (dq - state.qdot).abs() > 1e-10 || gap > 1e-11 {
            return Err(format!("independent hinge reconstruction differs at {tick}").into());
        }
        let mut v = View::default();
        let mut momentum = [0.; 3];
        let mut angular = [0.; 3];
        for (k, link) in state.links.iter().enumerate() {
            let name = ["base", "tool"][k];
            let radius = [0.16, 0.12][k];
            let color = [BLUE, ORANGE][k];
            let mass = [2., 0.8][k];
            let inertia = [0.12, 0.036][k];
            let com = add(
                link.position,
                rotate(
                    link.orientation_xyzw,
                    [[0.07, -0.03, 0.], [-0.04, 0.02, 0.]][k],
                ),
            );
            if norm(sub(com, link.com)) > 1e-12 {
                return Err("link COM transform mismatch".into());
            }
            let p = scale(link.com_velocity, mass);
            momentum = add(momentum, p);
            angular = add(
                angular,
                add(cross(com, p), scale(link.angular_velocity, inertia)),
            );
            let pos = world(origin, link.position);
            let anchor = world(origin, link.anchor);
            v.add(
                link.public_visual - 1,
                &format!(
                    "{}_robot_{}_{}_link_{}_visual_{}_native_body_{}",
                    label, tr.model, name, link.public_link, link.public_visual, link.native_body
                ),
                pos,
                link.orientation_xyzw,
                json!({"kind":"sphere","radius":radius*DISPLAY_SCALE}),
                color,
            );
            segment(
                &mut v,
                100 + 20 * k as u64,
                &format!("{name}_origin_to_own_hinge_anchor"),
                pos,
                anchor,
                0.082,
                color,
            );
            let c = world(origin, com);
            v.add(
                102 + 20 * k as u64,
                &format!("{name}_world_center_of_mass"),
                c,
                [0., 0., 0., 1.],
                json!({"kind":"sphere","radius":0.068}),
                [0.92, 0.86, 0.37, 1.],
            );
            // Extend the COM marker outward to make its offset distinguishable through the spherical visual.
            segment(
                &mut v,
                104 + 20 * k as u64,
                &format!("{name}_center_of_mass_direction"),
                c,
                add(pos, scale(sub(c, pos), 4.)),
                0.035,
                [0.94, 0.86, 0.41, 1.],
            );
            segment(
                &mut v,
                106 + 20 * k as u64,
                &format!("{name}_link_x_axis"),
                pos,
                add(
                    pos,
                    rotate(
                        link.orientation_xyzw,
                        [radius * DISPLAY_SCALE + 0.18, 0., 0.],
                    ),
                ),
                0.035,
                [0.85, 0.74, 0.62, 1.],
            );
        }
        if norm(sub(momentum, state.linear_momentum)) > 1e-11
            || norm(sub(angular, state.angular_momentum)) > 1e-11
        {
            return Err("whole-tree momentum mismatch".into());
        }
        let dp = norm(sub(momentum, p0));
        let dl = norm(sub(angular, l0));
        p_drift = p_drift.max(dp);
        l_drift = l_drift.max(dl);
        let pivot = world(origin, base.anchor);
        v.add(
            200,
            "hinge_public_9_native_0_world_anchor",
            pivot,
            [0., 0., 0., 1.],
            json!({"kind":"sphere","radius":0.088}),
            [0.91, 0.77, 0.30, 1.],
        );
        segment(
            &mut v,
            202,
            "hinge_positive_world_axis",
            pivot,
            add(pivot, scale(state.axis, 0.87)),
            0.05,
            [0.30, 0.77, 0.54, 1.],
        );
        let effort = if tick == 0 {
            tr.pending_effort_nm
        } else if tick == tr.later_effort_tick {
            tr.later_effort_nm
        } else {
            0.
        };
        for (k, (label, value, k_scale, col)) in [
            ("hinge_angle_rad", q, 1.2, [0.95, 0.73, 0.26, 1.]),
            ("hinge_rate_rad_s", dq, 0.44, [0.25, 0.66, 0.88, 1.]),
            (
                "effort_for_next_native_step_N_m",
                effort,
                1.7,
                [0.73, 0.41, 0.72, 1.],
            ),
        ]
        .into_iter()
        .enumerate()
        {
            let length = (value * k_scale).abs().max(0.025);
            v.cuboid(
                1_000_000 + tick * 20 + k as u64 * 2,
                &format!("offset_{tick}_{label}_{value:.12e}"),
                [-0.7 + 0.5 * value * k_scale, -1.9 + k as f64 * 0.29, 0.23],
                [length, 0.14, 0.18],
                col,
            );
        }
        let shown_clock = transfer.as_ref().map_or(f.ecs_clock_ns, |r| r.clock(j));
        v.cuboid(
            1_000_010 + tick * 20,
            &format!(
                "{}_offset_{tick}_local_ECS_clock_{}_ns_anchor_gap_{gap:.3e}_m",
                label, shown_clock
            ),
            [-2.5, -1.60, 0.25],
            [0.24, 0.24, 0.3],
            [0.40, 0.59, 0.68, 1.],
        );
        let process = transfer
            .as_ref()
            .map(|r| r.draw(&mut v, j, state))
            .transpose()?;
        samples.push(json!({"offset":tick,"process":process,"ecs_clock_ns":f.ecs_clock_ns,"anchor_gap_m":gap,"q_rad":q,"qdot_rad_s":dq,"pending_for_next_step_nm":effort,"linear_momentum_drift_ns":dp,"angular_momentum_drift_nms":dl}));
        stream.push(v.finish_at(tick + 1, time));
    }
    fs::create_dir(&a.directory)?;
    gz_sim_cli::export_archive(&a.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        a.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"mode":tr.mode,"display_label":label,"transfer_summary":transfer.as_ref().map(|r|&r.summary),"source_cut_ns":tr.native_source_cut_ns,"display_origin":origin,"display_scale":DISPLAY_SCALE,"max_anchor_gap_m":max_gap,"maximum_linear_momentum_drift_ns":p_drift,"maximum_angular_momentum_drift_nms":l_drift,"samples":samples}),
        )?,
    )?;
    println!(
        "All 257 local-native states agreed bit for bit, including both SDF-bound link states, their COMs and anchors and native joint. Independently reconstructed every angle, rate and momentum. Max anchor separation {max_gap:.4e} m."
    );
    Ok(())
}
