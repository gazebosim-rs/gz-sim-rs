// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two ordinary Bevy grippers receiving measured, simultaneous impulses.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/articulated_mobility.rs"]
mod fixture;
mod soft_replay;
use clap::Parser;
use fixture::*;
use glam::{DQuat, DVec3};
use gz_sim_scene::articulated_contact::ArticulatedImpulseReport;
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    output: PathBuf,
    /// Compare a one-finger strike and equal opposing impulses on two free, offset-COM grippers.
    #[arg(long)]
    pair: bool,
    /// Save true source positions, velocities, projected mobility and complete native states.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
const GOLD: [f64; 4] = [0.99, 0.72, 0.18, 1.];
const ROOT: [f64; 4] = [0.35, 0.43, 0.54, 1.];
const TEAL: [f64; 4] = [0.15, 0.78, 0.73, 1.];
const ORANGE: [f64; 4] = [0.90, 0.43, 0.22, 1.];

fn pulse(k: u64) -> Option<DVec3> {
    match k {
        100 => Some(DVec3::new(0.07, -0.15, 0.10)),
        270 => Some(DVec3::new(-0.09, 0.18, -0.15)),
        460 => Some(DVec3::new(-0.035, 0.08, 0.12)),
        620 => Some(DVec3::new(0.055, -0.09, -0.04)),
        _ => None,
    }
}
struct Run {
    gripper: Gripper,
    delta: [DVec3; 3],
}
fn event(report: &ArticulatedImpulseReport, impulse: [DVec3; 3]) -> Value {
    let k = (0..3)
        .map(|a| {
            (0..3)
                .map(|b| report.before.mobility(a, b).unwrap())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    json!({"dofs":report.before.dofs,"links":report.before.links,"impulse":impulse.map(|v|v.to_array()),"world_points":report.before.world_points,"before":report.before.velocities,"after":report.after_velocities,"mobility":k,"work_j":report.kinetic_energy_change})
}
fn visual(run: &Run, name: &str, shift: f64, grounded: bool, frame: &mut Frame) -> Value {
    let turn = DQuat::from_rotation_z(-0.72);
    let display = |p: DVec3| turn * p + shift * DVec3::Y;
    let states = run.gripper.states();
    let mut raw = vec![];
    for (i, s) in states.into_iter().enumerate() {
        let color = [ROOT, TEAL, ORANGE][i];
        let q = DQuat::from_array(s.orientation);
        let center = display(s.position.into());
        frame.add(
            &format!("{name}_body_{i}"),
            center,
            turn * q,
            json!({"kind":"sphere","radius":if i==0{0.14}else{0.10}}),
            color,
        );
        let point = pose(s, LOCAL_POINTS[i]);
        let start = display(point);
        frame.add(
            &format!("{name}_site_{i}"),
            start,
            turn * q,
            json!({"kind":"sphere","radius":0.026}),
            [0.96, 0.95, 0.88, 1.],
        );
        for j in 1..=4 {
            frame.add(
                &format!("{name}_delta_{i}_{j}"),
                start + turn * (2.5 * run.delta[i] * j as f64 / 4.),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":if j==4{0.035}else{0.017}}),
                GOLD,
            );
        }
        if i > 0 {
            let y = if i == 1 { 0.29 } else { -0.29 };
            let hinge = display(pose(states[0], [0.04, y, 0.]));
            frame.edge(
                &format!("{name}_shank_{i}"),
                hinge,
                center,
                0.60,
                0.052,
                color,
            );
            frame.add(
                &format!("{name}_hinge_{i}"),
                hinge,
                turn * DQuat::from_array(states[0].orientation),
                json!({"kind":"sphere","radius":0.055}),
                [0.60, 0.68, 0.74, 1.],
            );
        }
        raw.push(json!({"body":NAMES[i],"position":s.position,"orientation":s.orientation,"v":s.velocity,"w":s.angular_velocity,"site":point.to_array(),"last_delta":run.delta[i].to_array()}));
    }
    frame.edge(
        &format!("{name}_palm"),
        display(pose(states[0], [0.04, 0.29, 0.])),
        display(pose(states[0], [0.04, -0.29, 0.])),
        0.58,
        0.073,
        ROOT,
    );
    frame.add(
        &format!("{name}_table"),
        display(DVec3::new(0., 0., -0.02)),
        turn,
        json!({"kind":"box","size":[1.60,1.18,0.04]}),
        [0.22, 0.26, 0.30, 1.],
    );
    if grounded {
        frame.add(
            &format!("{name}_world_mount"),
            display(DVec3::new(-0.32, 0., 0.25)),
            turn,
            json!({"kind":"box","size":[0.075,0.075,0.50]}),
            [0.65, 0.70, 0.74, 1.],
        );
    }
    json!(raw)
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut a = Run {
        gripper: Gripper::new(
            if args.pair {
                WORLD.into()
            } else {
                centered_fixture(false)
            }
            .as_str(),
        )?,
        delta: [DVec3::ZERO; 3],
    };
    let mut b = Run {
        gripper: Gripper::new(
            if args.pair {
                WORLD.into()
            } else {
                centered_fixture(true)
            }
            .as_str(),
        )?,
        delta: [DVec3::ZERO; 3],
    };
    let mut out = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut frames = vec![];
    for tick in 0..=800 {
        let mut row = json!({"tick":tick,"time_ns":1_000_000u64*tick,"pair":args.pair});
        let mut f = Frame::default();
        for (lane, r) in [&mut a, &mut b].into_iter().enumerate() {
            if tick > 0 {
                r.gripper.sim.step_exact(1)?;
            }
            let record = if let Some(j) = pulse(tick) {
                let impulses = [
                    DVec3::ZERO,
                    j,
                    if args.pair && lane == 1 {
                        -j
                    } else {
                        DVec3::ZERO
                    },
                ];
                let before = r
                    .gripper
                    .states()
                    .map(|s| json!({"v":s.velocity,"w":s.angular_velocity}));
                let response = r.gripper.apply(impulses)?;
                r.delta = std::array::from_fn(|i| {
                    DVec3::from(response.after_velocities[i])
                        - DVec3::from(response.before.velocities[i])
                });
                let mut e = event(&response, impulses);
                e["bodies_before"] = json!(before);
                e
            } else {
                Value::Null
            };
            let name = if lane == 0 {
                "free"
            } else if args.pair {
                "balanced"
            } else {
                "grounded"
            };
            let states = visual(
                r,
                name,
                if lane == 0 { 0.85 } else { -0.85 },
                !args.pair && lane == 1,
                &mut f,
            );
            row[name] = json!({"bodies":states,"event":record});
        }
        if tick % 5 == 0 {
            frames.push(f.finish(a.gripper.sim.elapsed(), frames.len() + 1));
        }
        if let Some(w) = &mut out {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
    }
    if let Some(w) = &mut out {
        w.flush()?;
    }
    println!(
        "Recorded {} native correspondence frames and four actual instantaneous impulse barriers.",
        frames.len()
    );
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
