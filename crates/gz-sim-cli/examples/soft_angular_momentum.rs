// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare ordinary and momentum-preserving XPBD on the same free rotor.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
mod soft_replay;
use soft_replay::Frame;

#[derive(Parser)]
struct Args {
    /// New gz-web-rs archive.
    output: PathBuf,
    /// Use the 3-by-3 spinning cloth in both profiles.
    #[arg(long)]
    cloth: bool,
    /// Write every physical tick as JSON lines.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
struct Run {
    sim: Simulation,
    name: &'static str,
    shift: DVec3,
    color: [f64; 4],
    rest: Vec<DVec3>,
    initial_angular: DVec3,
}
const CENTER: DVec3 = DVec3::new(0., 0., 0.8);
impl Run {
    fn new(xml: &str, conserve: bool) -> Result<Self, Box<dyn Error>> {
        let xml = xml.replace(
            "conserve_angular_momentum=\"true\"",
            &format!("conserve_angular_momentum=\"{conserve}\""),
        );
        let sim = load_world_with_sdf_step(
            &xml,
            "spinning.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let soft = bodies(&sim).next().unwrap().1.particles();
        let rest = soft.iter().map(|p| DVec3::from(p.position)).collect();
        let initial_angular = soft
            .iter()
            .map(|p| {
                (DVec3::from(p.position) - CENTER).cross(DVec3::from(p.velocity) / p.inverse_mass)
            })
            .sum();
        let (name, offset, color) = if conserve {
            ("conserving", -0.65, [0.09, 0.78, 0.80, 1.])
        } else {
            ("ordinary", 0.65, [1., 0.48, 0.19, 1.])
        };
        Ok(Self {
            sim,
            name,
            shift: DVec3::Y * offset,
            color,
            rest,
            initial_angular,
        })
    }
    fn sample(&self, cloth: bool, f: &mut Frame) -> Value {
        let sample = bodies(&self.sim).next().unwrap().1;
        let points = sample.particles();
        let n = self.name;
        let mut angular = DVec3::ZERO;
        let mut energy = 0.;
        let mut states = vec![];
        for (i, p) in points.iter().enumerate() {
            let x = DVec3::from(p.position);
            let v = DVec3::from(p.velocity);
            let mv = v / p.inverse_mass;
            angular += (x - CENTER).cross(mv);
            energy += 0.5 * mv.dot(v);
            states.push(
                json!({"position":p.position,"velocity":p.velocity,"mass":p.inverse_mass.recip()}),
            );
            f.add(
                &format!("{n}_node_{i}"),
                x + self.shift,
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":if i==0{0.037}else{0.025}}),
                self.color,
            );
            let mut edge = |a: usize| {
                f.edge(
                    &format!("{n}_edge_{a}_{i}"),
                    DVec3::from(points[a].position) + self.shift,
                    x + self.shift,
                    self.rest[a].distance(self.rest[i]),
                    0.016,
                    self.color,
                );
            };
            if i > 0 && (!cloth || i % 3 != 0) {
                edge(i - 1);
            }
            if cloth && i >= 3 {
                edge(i - 3);
            }
        }
        let q = DQuat::from_rotation_x(6. * self.sim.elapsed().as_secs_f64());
        let expected = CENTER + q * (self.rest[0] - CENTER);
        // The pale reference sits just forward of the measured rotor in the viewer.
        let reference_shift = self.shift + DVec3::X * 0.055;
        f.edge(
            &format!("{n}_analytic_radius"),
            CENTER + reference_shift,
            expected + reference_shift,
            self.rest[0].distance(CENTER),
            0.008,
            [0.79, 0.82, 0.85, 1.],
        );
        f.add(
            &format!("{n}_analytic_tip"),
            expected + reference_shift,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.018}),
            [0.9, 0.9, 0.9, 1.],
        );
        let ratio = angular.dot(self.initial_angular) / self.initial_angular.length_squared();
        let a = DVec3::new(0., 0.34, 0.25) + self.shift;
        let b = a - DVec3::Y * 0.68;
        f.edge(
            &format!("{n}_angular_momentum_scale"),
            a,
            b,
            0.68,
            0.010,
            [0.5, 0.50, 0.52, 1.],
        );
        for (i, x) in [a, b].into_iter().enumerate() {
            f.add(
                &format!("{n}_momentum_{}pct", i * 100),
                x,
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.015}),
                [0.7, 0.71, 0.72, 1.],
            );
        }
        f.add(
            &format!("{n}_retained_angular_momentum"),
            a.lerp(b, ratio),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.034}),
            self.color,
        );
        json!({"particles":states,"angular_momentum":angular.to_array(),"fraction":ratio,"kinetic_energy":energy})
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let xml = if args.cloth {
        include_str!("../../../examples/spinning_cloth.sdf")
    } else {
        include_str!("../../../examples/spinning_cable.sdf")
    };
    let mut runs = [Run::new(xml, false)?, Run::new(xml, true)?];
    let mut data = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut frames = Vec::new();
    for tick in 0..=400 {
        let mut f = Frame::default();
        let mut row = json!({"tick":tick,"frame":null});
        for run in &mut runs {
            if tick > 0 {
                run.sim.step_exact(1)?;
            }
            row[run.name] = run.sample(args.cloth, &mut f);
        }
        if tick % 2 == 0 {
            frames.push(f.finish(runs[0].sim.elapsed(), frames.len() + 1));
            row["frame"] = json!(frames.len());
        }
        if let Some(w) = &mut data {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
        if tick == 400 {
            println!(
                "At 4 s ordinary retained {} of initial L, conserving retained {}.",
                row["ordinary"]["fraction"], row["conserving"]["fraction"]
            );
        }
    }
    if let Some(w) = &mut data {
        w.flush()?;
    }
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
