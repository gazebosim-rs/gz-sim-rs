// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native-footage comparison of force distribution on the soft substep clock.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/force_aware.rs"]
mod force;
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/capsule_grasp.rs"]
mod grasp;
#[path = "soft_replay/capsule_grasp.rs"]
mod grasp_view;
mod soft_replay;
use clap::Parser;
use force::*;
use glam::{DQuat, DVec3};
use gz_sim_scene::deformable::DeformableSample;
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
    /// Record the two-jaw cloth carry. The default records a fixed platen under identical loading.
    #[arg(long)]
    carry: bool,
    /// Use the same authored normal, transport and countertorque history on both free cloth grips.
    #[arg(long, requires = "carry")]
    identical: bool,
    /// Save all public states and each converged substep contact.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
const ORANGE: [f64; 4] = [0.96, 0.44, 0.16, 1.];
const CYAN: [f64; 4] = [0.10, 0.79, 0.78, 1.];
const GOLD: [f64; 4] = [1., 0.75, 0.20, 1.];
fn diagnostics(s: &DeformableSample) -> (Value, Value) {
    let samples=s.last_capsule_contact_samples().iter().map(|v|json!({"boundary":v.boundary,"vertices":v.feature.vertices(),"interval":v.interval,"first":v.first_time,"weights":v.weights,"point":v.point,"normal":v.normal,"jn":v.normal_impulse,"jt":v.tangent_impulse,"slip":v.slip_velocity})).collect::<Vec<_>>();
    let f=s.capsule_forcing().iter().map(|v|json!({"boundary":v.boundary,"used":v.used,"acceleration":v.acceleration,"start_v":v.native_start_velocity,"end_v":v.native_end_velocity,"chord_v":v.native_chord_velocity})).collect::<Vec<_>>();
    (json!(samples), json!(f))
}
fn sample_platen(run: &Platen, name: &str, dy: f64, f: &mut Frame, force: f64) -> Value {
    let s = run.state();
    let color = if name == "force_aware" { CYAN } else { ORANGE };
    let shift = dy * DVec3::Y;
    let p = DVec3::from(s.position) + shift;
    let q = DQuat::from_array(s.orientation);
    for (j, sign) in [("bottom", -1.), ("top", 1.)] {
        f.add(
            &format!("{name}_piston_{j}"),
            p + q * (sign * 0.12 * DVec3::Z),
            q,
            json!({"kind":"sphere","radius":0.08}),
            color,
        );
    }
    for i in 0..10 {
        let a = q * DQuat::from_rotation_z(i as f64 * std::f64::consts::TAU / 10.);
        f.add(
            &format!("{name}_piston_barrel_{i}"),
            p + a * (0.077 * DVec3::X),
            a,
            json!({"kind":"box","size":[0.007,0.052,0.24]}),
            color,
        );
    }
    let sample = run.cloth();
    let nodes = sample
        .particles()
        .iter()
        .map(|p| DVec3::from(p.position) + shift)
        .collect::<Vec<_>>();
    for (i, &p) in nodes.iter().enumerate() {
        f.add(
            &format!("{name}_anchor_{i}"),
            p,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.023}),
            [0.70, 0.73, 0.76, 1.],
        );
    }
    for (i, (a, b)) in [(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)]
        .into_iter()
        .enumerate()
    {
        f.edge(
            &format!("{name}_membrane_{i}"),
            nodes[a],
            nodes[b],
            nodes[a].distance(nodes[b]),
            0.012,
            [0.48, 0.56, 0.62, 1.],
        );
    }
    let mut forces = [0.; 4];
    for c in sample.last_capsule_contact_samples() {
        let i = ((0.5 * (c.interval[1] + c.interval[0])) / 0.001).floor() as usize;
        forces[i.min(3)] += c.normal_impulse / 0.001;
    }
    // White markers are the known actuator load. Colored markers are
    // independent 1 ms bins integrated from all native contact substeps.
    for (i, measured) in forces.iter().enumerate() {
        let foot = DVec3::new(-0.36 + 0.17 * i as f64, dy - 0.38, 0.06);
        f.add(
            &format!("{name}_force_scale_{i}"),
            foot + 0.27 * DVec3::Z,
            DQuat::IDENTITY,
            json!({"kind":"box","size":[0.012,0.013,0.54]}),
            [0.36, 0.40, 0.46, 1.],
        );
        f.add(
            &format!("{name}_command_{i}"),
            foot + 0.024 * force * DVec3::Z,
            DQuat::IDENTITY,
            json!({"kind":"box","size":[0.080,0.025,0.010]}),
            [0.88, 0.89, 0.85, 1.],
        );
        f.add(
            &format!("{name}_measured_{i}"),
            foot + 0.024 * measured * DVec3::Z,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.027}),
            color,
        );
    }
    f.add(
        &format!("{name}_velocity"),
        DVec3::new(0.34, dy - 0.38, 0.06 + 10. * s.velocity[2]),
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":0.026}),
        GOLD,
    );
    f.add(
        &format!("{name}_base"),
        DVec3::new(-0.04, dy, 0.025),
        DQuat::IDENTITY,
        json!({"kind":"box","size":[0.97,0.93,0.045]}),
        [0.23, 0.26, 0.29, 1.],
    );
    let (events, forcing) = diagnostics(sample);
    json!({"position":s.position,"orientation":s.orientation,"velocity":s.velocity,"command_n":force,"slot_force":forces,"samples":events,"forcing":forcing,"particles":sample.particles().iter().map(|p|p.position).collect::<Vec<_>>()})
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut frames = vec![];
    let mut out = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut save = |data: Value| -> Result<(), Box<dyn Error>> {
        if let Some(w) = &mut out {
            serde_json::to_writer(&mut *w, &data)?;
            writeln!(w)?;
        }
        Ok(())
    };
    if args.carry {
        let case = if args.identical {
            CarryInput::Balanced
        } else {
            CarryInput::Feedback
        };
        let mut chord = pair(false, 4, 0.001, case)?;
        let mut aware = pair(true, 4, 0.001, case)?;
        for k in 0..=1050 {
            let zero = [(DVec3::ZERO, DVec3::ZERO); 2];
            let (a, b) = if k > 0 {
                let a = command(case, &chord, &chord);
                let b = command(case, &chord, &aware);
                chord.step_wrenches(a)?;
                aware.step_wrenches(b)?;
                (a, b)
            } else {
                (zero, zero)
            };
            let mut f = Frame::default();
            let mut row = json!({"tick":k,"time_ns":1_000_000u64*k,"same_input":a==b});
            for (name, dy, g, cmd) in [
                ("native_chord", 0.63, &chord, a),
                ("force_aware", -0.63, &aware, b),
            ] {
                let mut v = grasp_view::sample(g, name, dy, &mut f, cmd);
                let (events, forcing) = diagnostics(g.cloth());
                v["samples"] = events;
                v["forcing"] = forcing;
                let mut w = Window::default();
                w.add(g.cloth(), 0.001, 2, grasp::SQUEEZE);
                let duty = w.duty / w.duration;
                v["duty"] = json!(duty);
                let base = DVec3::new(-0.50, dy + 0.24, 0.055);
                f.add(
                    &format!("{name}_duty_scale"),
                    base + 0.26 * DVec3::X,
                    DQuat::IDENTITY,
                    json!({"kind":"box","size":[0.52,0.030,0.025]}),
                    [0.30, 0.34, 0.38, 1.],
                );
                f.add(
                    &format!("{name}_active_load"),
                    base + 0.52 * duty * DVec3::X + DVec3::Z * 0.028,
                    DQuat::IDENTITY,
                    json!({"kind":"sphere","radius":0.032}),
                    if name == "force_aware" { CYAN } else { ORANGE },
                );
                row[name] = v;
            }
            if k % 5 == 0 {
                frames.push(f.finish(chord.sim.elapsed(), frames.len() + 1));
            }
            save(row)?;
        }
    } else {
        let mut chord = Platen::new(false, 4, 0.004)?;
        let mut aware = Platen::new(true, 4, 0.004)?;
        for k in 0..=200 {
            let fz = if k == 0 {
                0.
            } else {
                load((k - 1) as f64 * 0.004)
            };
            if k > 0 {
                chord.step(fz)?;
                aware.step(fz)?;
            }
            let mut f = Frame::default();
            let mut r = json!({"tick":k,"time_ns":k*4_000_000u64});
            for (n, dy, g) in [
                ("native_chord", 0.63, &chord),
                ("force_aware", -0.63, &aware),
            ] {
                r[n] = sample_platen(g, n, dy, &mut f, fz);
            }
            frames.push(f.finish(chord.sim.elapsed(), frames.len() + 1));
            save(r)?;
        }
    }
    if let Some(w) = &mut out {
        w.flush()?;
    }
    println!("Recorded {} actual Bevy frames.", frames.len());
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
