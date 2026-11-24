// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record a free, torque-driven spherical wrist and its carrier's counter-rotation.
#![allow(deprecated)]
#[path = "ball_wrist/mounted.rs"]
mod mounted;
use clap::Parser;
use glam::DVec3;
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BallJointSample, Name, PhysicsEngine, Velocity, load_world_with_sdf_step,
    queue_ball_joint_effort,
};
use std::{error::Error, path::Path};

const XML: &str = include_str!("../../../examples/worlds/local_ball_wrist.sdf");

#[derive(Parser)]
struct Options {
    /// Fresh gz-web-rs replay directory.
    archive: std::path::PathBuf,
    /// Park a hanging wrist and compare its first torque response with continuous physics.
    #[arg(long)]
    mounted: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let options = Options::parse();
    let path = options.archive;
    if options.mounted {
        return mounted::record(&path);
    }
    let sdf = sdformat::load_str(XML, "wrist.sdf", &sdformat::NoResources)?;
    let world = "default";
    let mut sim = load_world_with_sdf_step(
        XML,
        "wrist.sdf",
        world,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let parts = ["bearing", "carrier", "tool"].map(|local| {
        let name = format!("{world}::wrist::{local}");
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .map(|e| e.id())
    });
    let [Some(joint), Some(carrier), Some(tool)] = parts else {
        return Err("missing authored wrist entity".into());
    };
    let bearing = sim
        .world()
        .get::<EntityId>(joint)
        .ok_or("missing joint ID")?
        .get();
    let mut frames = vec![gz_sim_cli::primitive_frame(&sim, &sdf, world, 1)?];
    let mut max_residual = 0_f64;
    for tick in 1..=2400 {
        // Compose about different moving-frame axes, then coast freely.
        let torque = match tick {
            1..=400 => [0.15, -0.08, 0.14],
            401..=800 => [-0.14, 0.18, -0.04],
            801..=1200 => [0.03, -0.13, -0.16],
            _ => [0.; 3],
        };
        if tick <= 1200 {
            queue_ball_joint_effort(&mut sim, bearing, torque)?;
        }
        sim.step_exact(1)?;
        let spin = |entity| {
            sim.world()
                .get::<Velocity>(entity)
                .map(|v| DVec3::from_array(v.angular))
        };
        let residual = 0.04 * spin(carrier).ok_or("missing carrier velocity")?
            + 0.02 * spin(tool).ok_or("missing tool velocity")?;
        max_residual = max_residual.max(residual.length());
        if tick % 20 == 0 {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                &sdf,
                world,
                frames.len() as u64 + 1,
            )?);
        }
    }
    let sample = sim
        .world()
        .get::<BallJointSample>(joint)
        .ok_or("missing spherical state")?;
    println!(
        "Peak total angular momentum residual {max_residual:e} N·m·s; final relative angular velocity {:?} rad/s",
        sample.state.angular_velocity
    );
    gz_sim_cli::export_archive(Path::new(&path), Default::default(), frames)?;
    Ok(())
}
