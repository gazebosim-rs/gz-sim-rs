// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Generate ordinary reference cuts and repeat a loaded worker transfer on a recovered session.
use clap::{Parser, Subcommand};
use gz_physics::PhysicsBackend;
use gz_sim_scene::distributed_hinge::{
    DistributedMultiSphereBackend, MultiSpherePool, WorkerTransport, load_distributed_hinge_world,
};
use gz_sim_scene::queue_joint_effort;
use std::{error::Error, fs, path::PathBuf};

#[path = "support/continuous_hinge_drive.rs"]
mod continuous_hinge_drive;

#[derive(Parser)]
struct Args {
    /// Stock physics_worker for references or convoy_ssh_worker for deployed transfer.
    #[arg(long)]
    worker: PathBuf,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Generate {
        #[arg(long)]
        reverse: bool,
        /// Durably queue the full signed torque before copying the input cut.
        #[arg(long)]
        pending: bool,
        /// Tick immediately before or after the pair's loaded transfer to slot zero.
        #[arg(long,value_parser=clap::value_parser!(u64).range(149..=150))]
        tick: u64,
        /// New directory for the reference session and immutable inputs.
        output: PathBuf,
    },
    /// Transfer the loaded contact to the other SSH node after a successful recovery.
    Transfer { session: PathBuf },
}

fn transfer(worker: &std::path::Path, session: &std::path::Path) -> Result<(), Box<dyn Error>> {
    let mut pool =
        MultiSpherePool::resume_with_transport(worker, session, WorkerTransport::LoopbackZenoh)?;
    let before = serde_json::to_value(pool.observation())?;
    let owners = pool.owners();
    if pool.observation().hinge.tick != 150
        || pool.observation().sphere_reaction.normal_impulse <= 0.
        || owners != [0, 1, 1]
    {
        return Err("expected loaded pre-transfer cut at 150".into());
    }
    pool.transfer_pair(0)?;
    if pool.owners() != [1, 0, 0] || serde_json::to_value(pool.observation())? != before {
        return Err("ownership transfer changed a committed reaction or pose".into());
    }
    println!(
        "{}",
        serde_json::json!({"before_owners":owners,"after_owners":pool.owners(),"tick":150,"normal_impulse_ns":pool.observation().sphere_reaction.normal_impulse,"observations_unchanged":true})
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let (reverse, pending, tick, output) = match args.command {
        Command::Generate {
            reverse,
            pending,
            tick,
            output,
        } => (reverse, pending, tick, output),
        Command::Transfer { session } => return transfer(&args.worker, &session),
    };
    fs::create_dir(&output)?;
    let session = output.join("reference");
    let mut sim = load_distributed_hinge_world(
        include_str!("../../../examples/distributed_sphere_pair_island.sdf"),
        "operator.sdf",
        "default",
        &args.worker,
        &session,
        Some(3),
    )?;
    let torque = if reverse { -0.01 } else { 0.01 };
    continuous_hinge_drive::install_drive(&mut sim, torque)?;
    sim.step_exact(tick)?;
    if tick == 150 {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<DistributedMultiSphereBackend>()
            .ok_or("missing multi-sphere backend")?
            .transfer_pair(0)?;
    }
    let joint = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedMultiSphereBackend>()
        .ok_or("missing multi-sphere backend")?
        .pool()
        .spec()
        .hinge
        .joint_id;
    if pending {
        queue_joint_effort(&mut sim, joint, torque)?;
    }
    let archive = session.join("barrier.gzmultisphere");
    fs::copy(&archive, output.join("initial.gzmultisphere"))?;
    sim.step_exact(1)?;
    fs::copy(&archive, output.join("next.gzmultisphere"))?;
    if tick == 149 {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<DistributedMultiSphereBackend>()
            .ok_or("missing multi-sphere backend")?
            .transfer_pair(0)?;
        fs::copy(&archive, output.join("transferred.gzmultisphere"))?;
    }
    Ok(())
}
