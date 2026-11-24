// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record ordinary and conserving 3-D rotor states, with an independent RK4 trajectory.
use clap::Parser;
#[path = "../../../examples/support/free_top.rs"]
mod free_top;
mod soft_replay;
use free_top::*;
use glam::{DQuat, DVec3};
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
    /// New replay directory for gz-web-rs.
    output: PathBuf,
    /// Apply two opposing finite world-torque pulses during the run.
    #[arg(long)]
    driven: bool,
    /// Save raw native states and reference values for every simulated tick.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
fn dot(f: &mut Frame, name: &str, p: DVec3, r: f64, color: [f64; 4]) {
    f.add(
        name,
        p,
        DQuat::IDENTITY,
        json!({"kind":"sphere","radius":r}),
        color,
    );
}
fn sample(run: &Run, index: usize, reference: Reference, frame: &mut Frame) -> Value {
    let (name, offset, color) = if index == 0 {
        ("native", 0.66, [0.96, 0.42, 0.14, 1.])
    } else {
        ("conserving", -0.66, [0.08, 0.67, 0.70, 1.])
    };
    let gray = [0.6, 0.62, 0.65, 1.];
    let white = [0.96, 0.96, 0.93, 1.];
    let shift = DVec3::Y * offset;
    let s = run.state();
    let native_center = DVec3::from(s.position);
    let center = native_center + shift;
    let q = DQuat::from_array(s.orientation);
    let (l, energy) = spatial(s);
    let dl = l.distance(reference.momentum);
    let angle = attitude_error(q, reference.attitude());
    frame.add(
        &format!("{name}_top"),
        center,
        q,
        json!({"kind":"box","size":[0.60,0.36,0.22]}),
        color,
    );
    // A fixed corner attachment shows the measured or reference attitude in 3D.
    let tip = DVec3::new(0.41, 0.19, 0.12);
    let measured = center + q * tip;
    let expected = center + reference.attitude() * tip;
    for (label, end, c, r) in [
        ("measured", measured, color, 0.034),
        ("reference", expected, white, 0.023),
    ] {
        frame.edge(
            &format!("{name}_{label}_radius"),
            center,
            end,
            tip.length(),
            0.012,
            c,
        );
        dot(frame, &format!("{name}_{label}_tip"), end, r, c);
    }
    // Each dot is an actual measured residual on a fixed metric scale.
    for (kind, z, fraction) in [
        ("momentum", 0.29, dl / 0.010),
        ("attitude", 0.17, angle / 2.3),
    ] {
        let a = shift + DVec3::new(0., 0.29, z);
        let b = a - DVec3::Y * 0.58;
        frame.edge(&format!("{name}_{kind}_scale"), a, b, 0.58, 0.008, gray);
        dot(frame, &format!("{name}_{kind}_origin"), a, 0.013, gray);
        dot(frame, &format!("{name}_{kind}_limit"), b, 0.013, gray);
        dot(
            frame,
            &format!("{name}_{kind}_residual"),
            a.lerp(b, fraction),
            0.029,
            color,
        );
    }
    json!({"position":s.position,"orientation":s.orientation,"angular_velocity":s.angular_velocity,"angular_momentum":l.to_array(),"energy":energy,"momentum_error":dl,"attitude_error":angle})
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut runs = [Run::new(false, 10)?, Run::new(true, 10)?];
    let mut reference = Reference::new(runs[0].state());
    let mut writer = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut frames = vec![];
    for tick in 0..=600 {
        let tau = if tick == 0 {
            DVec3::ZERO
        } else {
            torque((tick - 1) * 10, args.driven)
        };
        if tick > 0 {
            for r in &mut runs {
                r.advance(tau)?;
            }
            reference.advance(0.01, tau, 32);
        }
        let mut f = Frame::default();
        let mut row = json!({"tick":tick,"frame":null,"torque":tau.to_array(),"expected":{"momentum":reference.momentum.to_array(),"orientation":reference.attitude().to_array(),"spin":reference.spin().to_array(),"energy":reference.energy()}});
        for (i, r) in runs.iter().enumerate() {
            let mode = if i == 0 { "native" } else { "conserving" };
            row[mode] = sample(r, i, reference, &mut f);
        }
        if tick % 3 == 0 {
            frames.push(f.finish(runs[0].sim.elapsed(), frames.len() + 1));
            row["frame"] = json!(frames.len());
        }
        if let Some(w) = &mut writer {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
        if tick == 600 {
            println!(
                "6 s final native/conserving attitude error (rad): {} / {}. Angular momentum error (N·m·s): {} / {}.",
                row["native"]["attitude_error"],
                row["conserving"]["attitude_error"],
                row["native"]["momentum_error"],
                row["conserving"]["momentum_error"]
            );
        }
    }
    if let Some(w) = &mut writer {
        w.flush()?;
    }
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
