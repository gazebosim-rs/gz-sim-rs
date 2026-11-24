// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify negotiated friction profiles against an actual earlier worker and native archive.
use clap::Parser;
use gz_physics_distributed::{
    physics::{
        Body, GroundPlane, NativeFrictionModel, NativeSolverProfile, SimulationStep, fixture,
    },
    pool::{PoolNativeRecovery, WorkerPool},
};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    previous_worker: PathBuf,
    /// Repeat for historical native pools or public scene archives containing a warm pool.
    #[arg(long, required = true)]
    previous_archive: Vec<PathBuf>,
    output: PathBuf,
}
fn spawn(path: &Path, profile: NativeSolverProfile) -> Result<WorkerPool> {
    let body = Body {
        friction: 0.5,
        position: [0., 0., 0.36],
        linear: [0.; 3],
        angular: [0., 0., 4.],
        ..fixture()[0].promoted()
    };
    Ok(WorkerPool::spawn_with_profile(
        path,
        7912,
        vec![vec![body], vec![], vec![]],
        SimulationStep::Ms10,
        Some(GroundPlane::with_friction(0., 0.65)?),
        128,
        profile,
    )?)
}
fn refusal<T, E: std::fmt::Display>(result: std::result::Result<T, E>) -> Result<String> {
    let error = result
        .err()
        .ok_or("unqualified worker accepted a point-model launch")?
        .to_string();
    if !error.contains("does not offer") {
        return Err(format!("unexpected startup failure: {error}").into());
    }
    Ok(error)
}
fn sha(path: &Path) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(fs::read(path)?)))
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.output)?;
    let mut rows = Vec::new();
    let gravity = [0., 0., -9.81];
    for grouped in [
        NativeSolverProfile::Standard32,
        NativeSolverProfile::Extended64,
    ] {
        let point = grouped.with_friction(NativeFrictionModel::PerPoint);
        let cold = refusal(spawn(&args.previous_worker, point))?;
        let mut live = spawn(&args.worker, point)?;
        for _ in 0..45 {
            live.advance_adaptive_in_field(gravity, &[])?;
        }
        live.transfer_component(1, 1)?;
        let file = args
            .output
            .join(format!("point-{}.gzpool", point.max_bodies()));
        live.save_native_recovery(&file)?;
        if &fs::read(&file)?[..8] != b"GZPOOL04" {
            return Err("point envelope magic".into());
        }
        let image = PoolNativeRecovery::open(&file)?;
        if image.native_solver_profile() != point || image.frame() != live.frame() {
            return Err("point archive model or frame changed".into());
        }
        let native = refusal(image.resume(&args.previous_worker))?;
        let mut resumed = PoolNativeRecovery::open(&file)?.resume(&args.worker)?;
        let mut samples = 0;
        for _ in 0..8 {
            if resumed.advance_adaptive_in_field(gravity, &[])?
                != live.advance_adaptive_in_field(gravity, &[])?
            {
                return Err("new point-mode peer diverged across fresh restore".into());
            }
            samples += 1;
        }
        // Authenticate the real earlier binary by making a successful grouped launch
        // and by verifying the original complete grouped startup/physics response.
        let mut old = spawn(&args.previous_worker, grouped)?;
        let mut new = spawn(&args.worker, grouped)?;
        for _ in 0..45 {
            if old.advance_adaptive_in_field(gravity, &[])?
                != new.advance_adaptive_in_field(gravity, &[])?
            {
                return Err("new and previous grouped peer cold trajectories differ".into());
            }
        }
        rows.push(
            json!({"capacity":grouped.max_bodies(),"model":point,"old_cold_refusal":cold,
            "old_image_refusal":native,"new_point_image_sha256":sha(&file)?,
            "point_resume_matched_steps":samples,"grouped_previous_worker_matched_steps":45}),
        );
    }

    let historical = args
        .previous_archive
        .iter()
        .map(|path| historic(&args, path, gravity))
        .collect::<Result<Vec<_>>>()?;
    fs::write(
        args.output.join("results.json"),
        serde_json::to_vec_pretty(&json!({
            "new_worker_sha256":sha(&args.worker)?,"previous_worker_sha256":sha(&args.previous_worker)?,
            "startup_and_restore":rows,"historical":historical
        }))?,
    )?;
    Ok(())
}

fn historic(args: &Args, path: &Path, gravity: [f64; 3]) -> Result<serde_json::Value> {
    let actual = fs::read(path)?;
    let is_scene = actual.starts_with(b"GZSCN");
    let open = || -> Result<PoolNativeRecovery> {
        if is_scene {
            Ok(PoolNativeRecovery::open_scene_manifest(path)?.2)
        } else {
            Ok(PoolNativeRecovery::open(path)?)
        }
    };
    let plan = open()?;
    let profile = plan.native_solver_profile();
    let high_water = plan.population_high_water();
    if plan.native_solver_profile().friction() != NativeFrictionModel::Grouped {
        return Err("prior archive must actually have grouped friction".into());
    }
    let archived_tick = plan.frame().tick;
    let starting_supports: usize = plan
        .frame()
        .workers
        .iter()
        .map(|x| x.snapshot.supports.len() + x.snapshot.impulses.len())
        .sum();
    if starting_supports == 0 {
        return Err("historical archive must exercise warm receipts".into());
    }
    let mut new = plan.resume(&args.worker)?;
    let mut old = open()?.resume(&args.previous_worker)?;
    for _ in 0..16 {
        if old.advance_adaptive_in_field(gravity, &[])?
            != new.advance_adaptive_in_field(gravity, &[])?
        {
            return Err(
                "new and old grouped restores diverged on their shared future input".into(),
            );
        }
    }
    Ok(
        json!({"file":path,"envelope_magic":std::str::from_utf8(&actual[..8])?,
        "profile":profile,"population_high_water":high_water,"archive_sha256":format!("{:x}",Sha256::digest(&actual)),
        "cut_tick":archived_tick,"initial_contact_receipts":starting_supports,
        "resumed_grouped_through_tick":new.frame().tick,"future_gravity":gravity}),
    )
}
