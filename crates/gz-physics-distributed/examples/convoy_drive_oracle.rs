// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a measured series of operator steps to an uninterrupted local Bevy world
//! and a separate uninterrupted worker pool with the same source placement.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "two_sphere_hinge/reference.rs"]
#[allow(dead_code)]
mod reference;
use authored_hinge::AuthoredHinge;
use clap::Parser;
use gz_physics_distributed::multi_sphere::{
    MultiSphereObservation, MultiSpherePool, MultiSphereSpec, inspect_published_path,
};
use gz_sim_cli::live::JointTorqueDrive;
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    sdf: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    program: PathBuf,
    /// Consecutive epoch-zero archives, beginning at the source cut. The
    /// program must also specify the drive history leading to that cut.
    #[arg(required = true)]
    cuts: Vec<PathBuf>,
}
fn main() -> Result<()> {
    let a = Args::parse();
    let xml = fs::read_to_string(&a.sdf)?;
    let world_name = a.sdf.to_string_lossy();
    let spec = distributed_multi_sphere_spec(&xml, &world_name, "default")?;
    let joint = spec.hinge.joint_id;
    let program = JointTorqueDrive::read(fs::File::open(a.program)?)?;
    let mut ordinary = AuthoredHinge::load(&xml, &world_name)?;
    let root = tempfile::tempdir()?;
    let archive = root.path().join("barrier.gzmultisphere");
    fs::copy(&a.cuts[0], &archive)?;
    let mut reference_pool = MultiSpherePool::resume(&a.worker, root.path())?;
    if reference_pool.spec() != &spec || reference_pool.epoch() != 0 {
        return Err("source cut disagrees with SDF or epoch".into());
    }
    let first = reference_pool.observation().hinge.tick;
    for tick in 0..first {
        ordinary.queue(joint, program.interval_effort(tick))?;
        ordinary.advance()?;
    }
    let mut rows = Vec::new();
    for (index, file) in a.cuts.iter().enumerate() {
        let tick = first.checked_add(index as u64).ok_or("clock overflow")?;
        if index > 0 {
            let effort = program.interval_effort(tick - 1);
            ordinary.queue(joint, effort)?;
            ordinary.advance()?;
            if let Some(pending) = reference_pool.pending_joint_effort() {
                if pending.to_bits() != effort.to_bits() {
                    return Err(format!("incompatible durable torque at tick {}", tick - 1).into());
                }
                reference_pool.advance()?;
            } else {
                reference_pool.advance_with_effort_at(0, tick - 1, joint, effort)?;
            }
        }
        let measured = inspect_published_path(file)?;
        let state = reference::observe(&ordinary, &spec, tick)?;
        if measured.cut.epoch != 0
            || measured.cut.observation.hinge.tick != tick
            || bincode::serialize(&state)? != bincode::serialize(&measured.cut.observation)?
            || fs::read(file)? != fs::read(&archive)?
        {
            return Err(format!("physics or native archive diverged at tick {tick}").into());
        }
        rows.push(json!({"tick":tick,"next_effort_nm":program.interval_effort(tick),
            "sha256":measured.sha256,"angle_rad":state.hinge.q,"angular_velocity_rad_s":state.hinge.qdot,
            "sphere_impulse_ns":state.sphere_reaction.normal_impulse}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"ordinary_observations":rows.len(),"exact_native_archives":rows.len(),"samples":rows})
        )?
    );
    Ok(())
}
