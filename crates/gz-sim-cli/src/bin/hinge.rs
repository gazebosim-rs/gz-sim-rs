// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Advance or transfer a durable SDF hinge using stock local workers.
use clap::Parser;
use gz_sim_scene::distributed_hinge::{
    ArticulationPool, HingeRecipe, MovingPool, MultiSpherePool, SessionBackend,
    distributed_hinge_recipe,
};
use std::{
    error::Error,
    fs::File,
    io::{self, Read},
    path::PathBuf,
};

#[derive(Parser)]
#[command(version, about = "Run one durable, distributed SDF hinge")]
struct Args {
    /// Trusted local session directory. Reopen the last committed cut by default.
    session: PathBuf,
    /// Initialize a new session from a self-contained two-link SDF.
    #[arg(long, value_name = "SDF")]
    create: Option<PathBuf>,
    /// Local worker count for a new session (default 3 for a hinge with two free spheres, otherwise 2).
    #[arg(long, requires = "create", value_parser = clap::value_parser!(u8).range(2..=8))]
    workers: Option<u8>,
    /// Authored world name when creating a session (default "default").
    #[arg(long, requires = "create")]
    world: Option<String>,
    /// Trusted physics_worker executable (default sibling of this binary).
    #[arg(long)]
    worker: Option<PathBuf>,
    /// Queue signed torque contributions, in occurrence order, before transfer or stepping.
    #[arg(long, allow_hyphen_values = true)]
    effort: Vec<f64>,
    /// Transfer the complete tree to this zero-based slot before stepping.
    #[arg(long)]
    transfer_to: Option<u8>,
    /// Migrate the independent sphere–sphere island to this slot before stepping.
    #[arg(long, conflicts_with_all = ["transfer_to", "create"])]
    transfer_pair_to: Option<u8>,
    /// Atomically add this torque (N·m) to any queued effort and advance one 1 ms tick.
    #[arg(long, value_name = "NM", allow_hyphen_values = true, value_parser = finite_torque,
        conflicts_with_all = ["effort", "steps", "transfer_to", "transfer_pair_to"])]
    step_with_effort: Option<f64>,
    /// Require this durable epoch before an atomic effort step.
    #[arg(long, value_name = "E", requires_all = ["step_with_effort", "expect_tick"])]
    expect_epoch: Option<u64>,
    /// Require this committed tick before an atomic effort step.
    #[arg(long, value_name = "T", requires_all = ["step_with_effort", "expect_epoch"])]
    expect_tick: Option<u64>,
    /// Number of advancing 1 ms steps.
    #[arg(long, default_value_t = 0)]
    steps: u64,
}

fn finite_torque(input: &str) -> Result<f64, String> {
    let torque = input.parse::<f64>().map_err(|e| e.to_string())?;
    if !torque.is_finite() {
        return Err("torque must be finite".into());
    }
    Ok(torque)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.create.is_some()
        && args
            .expect_epoch
            .zip(args.expect_tick)
            .is_some_and(|stamp| stamp != (0, 0))
    {
        return Err("a new hinge session starts at epoch 0, tick 0".into());
    }
    let exe = match args.worker.clone() {
        Some(exe) => exe,
        None => std::env::current_exe()?.with_file_name("physics_worker"),
    };
    let recipe = if let Some(path) = &args.create {
        let mut xml = String::new();
        File::open(path)?
            .take(1024 * 1024 + 1)
            .read_to_string(&mut xml)?;
        Some(distributed_hinge_recipe(
            &xml,
            &path.to_string_lossy(),
            args.world.as_deref().unwrap_or("default"),
        )?)
    } else {
        None
    };
    if args.transfer_pair_to.is_some()
        && !args.session.join("barrier.gzmultisphere").try_exists()?
    {
        return Err("sphere-pair transfer requires a two-sphere session".into());
    }
    let width = args.workers.unwrap_or(2);
    let report = match recipe {
        Some(HingeRecipe::Tree(spec)) => tree(
            ArticulationPool::create(&exe, &args.session, width, *spec)?,
            &args,
        )?,
        Some(HingeRecipe::Moving(spec)) => moving(
            MovingPool::create(&exe, &args.session, width, *spec)?,
            &args,
        )?,
        Some(HingeRecipe::MultiSphere(spec)) => multi_sphere(
            MultiSpherePool::create(&exe, &args.session, args.workers.unwrap_or(3), *spec)?,
            &args,
        )?,
        None if args.session.join("barrier.gzmultisphere").try_exists()? => {
            multi_sphere(MultiSpherePool::resume(&exe, &args.session)?, &args)?
        }
        None if args.session.join("barrier.gzmoving").try_exists()? => {
            moving(MovingPool::resume(&exe, &args.session)?, &args)?
        }
        None => tree(ArticulationPool::resume(&exe, &args.session)?, &args)?,
    };
    serde_json::to_writer(io::stdout().lock(), &report)?;
    println!();
    Ok(())
}

fn operate(pool: &mut impl SessionBackend, args: &Args) -> Result<(), Box<dyn Error>> {
    let joint = pool.hinge_sample().joint_id;
    if let Some(effort) = args.step_with_effort {
        if let Some((epoch, tick)) = args.expect_epoch.zip(args.expect_tick) {
            pool.advance_with_effort_at(epoch, tick, joint, effort)?;
        } else {
            pool.advance_with_effort(joint, effort)?;
        }
        return Ok(());
    }
    for &torque in &args.effort {
        pool.queue_effort(joint, torque)?;
    }
    if let Some(destination) = args.transfer_pair_to {
        pool.transfer_pair(destination)?;
    } else if let Some(destination) = args.transfer_to {
        pool.transfer(destination)?;
    }
    for _ in 0..args.steps {
        pool.advance()?;
    }
    Ok(())
}

fn tree(mut pool: ArticulationPool, args: &Args) -> Result<serde_json::Value, Box<dyn Error>> {
    operate(&mut pool, args)?;
    Ok(serde_json::json!({
        "workers": pool.width(), "owner": pool.owner(), "generation": pool.generation(),
        "epoch": pool.epoch(), "state": pool.observation(), "pending_effort_nm":pool.pending_joint_effort()
    }))
}

fn moving(mut pool: MovingPool, args: &Args) -> Result<serde_json::Value, Box<dyn Error>> {
    operate(&mut pool, args)?;
    Ok(serde_json::json!({
        "workers": pool.width(), "owners": pool.owners(), "generation": pool.generation(),
        "epoch": pool.epoch(), "state": pool.observation(), "pending_effort_nm":pool.pending_joint_effort()
    }))
}

fn multi_sphere(
    mut pool: MultiSpherePool,
    args: &Args,
) -> Result<serde_json::Value, Box<dyn Error>> {
    if let Some(destination) = args.transfer_pair_to {
        let [hinge, a, b] = pool.owners();
        if a != b || a == hinge || destination >= pool.width() || destination == a {
            return Err("expected a separate sphere pair and a different valid destination".into());
        }
    }
    operate(&mut pool, args)?;
    Ok(serde_json::json!({
        "workers": pool.width(), "owners": pool.owners(), "generation": pool.generation(),
        "epoch": pool.epoch(), "state": pool.observation(), "pending_effort_nm": pool.pending_joint_effort(),
        "sphere_pair_owner": pool.pair_image().map(|_| pool.owners()[1])
    }))
}
