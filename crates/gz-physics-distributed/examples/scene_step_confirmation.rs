// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise a stalled owner through normal Bevy stepping, loaded reset and same-build disk resume.
#[cfg(target_os = "linux")]
#[path = "../tests/private/passive_pose_proof.rs"]
mod passive_pose_proof;
#[cfg(target_os = "linux")]
mod linux {
    use clap::Parser;
    use gz_physics::PhysicsBackend;
    use gz_physics_distributed::SceneBackend;
    use gz_sim_core::Simulation;
    use gz_sim_scene::{
        PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
        passive_checkpoint::{PassivePoolScene, save_paused_runtime},
        queue_link_world_wrench, reset_loaded_world,
    };
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    use serde_json::json;
    use sha2::{Digest, Sha256};
    use std::{
        error::Error,
        fs::{self, File},
        io::{BufWriter, Write},
        path::{Path, PathBuf},
        thread,
        time::{Duration, Instant},
    };

    type Result<T> = std::result::Result<T, Box<dyn Error>>;
    const SDF: &str = include_str!("../../../examples/distributed_stack.sdf");
    #[derive(Parser)]
    struct Args {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
        workers: u8,
        /// Number of independent seeds, each exercising a cold start, durable restore and loaded reset.
        #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        output: PathBuf,
    }
    fn backend(sim: &Simulation) -> &SceneBackend {
        sim.world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .unwrap()
    }
    fn options(worker: &Path, width: u8) -> WorldLoadOptions<'_> {
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedIslands(width),
            distributed_worker: Some(worker),
            step: None,
            noise_seed: [0; 32],
        }
    }
    fn compare(a: &Simulation, b: &Simulation, digest: &mut Sha256) -> Result<()> {
        let value = super::passive_pose_proof::proof(a);
        if value != super::passive_pose_proof::proof(b) {
            return Err(format!(
                "public clocks, identities, poses, velocities, owners or receipts diverged at {:?}",
                a.elapsed()
            )
            .into());
        }
        digest.update(value);
        Ok(())
    }
    fn advance(a: &mut Simulation, b: &mut Simulation, n: u16, digest: &mut Sha256) -> Result<()> {
        for _ in 0..n {
            a.step_exact(1)?;
            b.step_exact(1)?;
            compare(a, b, digest)?;
        }
        Ok(())
    }
    pub fn run() -> Result<()> {
        let args = Args::parse();
        fs::create_dir(&args.output)?;
        let scratch = tempfile::tempdir_in(&args.output)?;
        let mut log = BufWriter::new(File::create_new(args.output.join("episodes.jsonl"))?);
        let config = WorldRuntimeOptions {
            distributed_packed_handoffs: true,
            distributed_local_helper_limit: Some(0),
            ..Default::default()
        };
        let mut all = Sha256::new();
        for round in 0..args.rounds {
            let mut tested = PassivePoolScene::load(
                SDF,
                "stack.sdf",
                "default",
                options(&args.worker, args.workers),
                config,
            )?
            .into_checkpointable_runtime();
            let mut ordinary = load_world_with_runtime(
                SDF,
                "stack.sdf",
                "default",
                &sdformat::NoResources,
                options(&args.worker, args.workers),
                config,
            )?;
            for world in [&mut tested, &mut ordinary] {
                world.set_paused(true);
                world.tick()?;
            }
            for (idx, source) in ["cold", "disk", "reset"].into_iter().enumerate() {
                if source == "reset" {
                    for world in [&mut tested, &mut ordinary] {
                        reset_loaded_world(world, |_| ())?;
                        world.set_paused(true);
                        world.tick()?;
                    }
                }
                advance(&mut tested, &mut ordinary, 75, &mut all)?;
                for world in [&mut tested, &mut ordinary] {
                    // A nonzero one-tick field must apply once on the faulted step.
                    queue_link_world_wrench(world, 7, [0.125, 0., 0.25], [0., 0.0625, 0.])?;
                }
                if source == "disk" {
                    let path = scratch.path().join(format!("cut-{round}"));
                    save_paused_runtime(&mut tested, &path)?;
                    drop(tested);
                    tested = PassivePoolScene::resume(
                        SDF,
                        "stack.sdf",
                        "default",
                        options(&args.worker, args.workers),
                        config,
                        &path,
                    )?
                    .into_checkpointable_runtime();
                }
                compare(&tested, &ordinary, &mut all)?;
                let pool = backend(&tested).pool_frame().ok_or("missing public pool")?;
                let (owner, positive) = pool
                    .workers
                    .iter()
                    .enumerate()
                    .map(|(i, w)| {
                        (
                            i,
                            w.snapshot
                                .impulses
                                .iter()
                                .filter(|p| p.normal_impulse > 0.)
                                .count(),
                        )
                    })
                    .max_by_key(|p| p.1)
                    .ok_or("empty pool")?;
                if positive == 0 {
                    return Err("expected positive finite contact before fault".into());
                }
                let before = backend(&tested).local_worker_pids().unwrap();
                let pid = before[owner];
                let fd = pidfd_open(
                    Pid::from_raw(pid.try_into()?).ok_or("invalid worker PID")?,
                    PidfdFlags::empty(),
                )?;
                pidfd_send_signal(&fd, Signal::STOP)?;
                let until = Instant::now() + Duration::from_secs(2);
                while !fs::read_to_string(format!("/proc/{pid}/status"))?
                    .lines()
                    .any(|l| l.starts_with("State:") && l.contains("T (stopped)"))
                {
                    if Instant::now() >= until {
                        return Err("worker did not stop".into());
                    }
                    thread::yield_now();
                }
                let thaw =
                    [Some(100), Some(1100), Some(1400), None][(usize::from(round) + idx) % 4];
                let replacement = thaw.is_none() || thaw == Some(1400);
                let start = Instant::now();
                let elapsed = thread::scope(|scope| -> Result<Duration> {
                    let job = thaw.map(|ms| {
                        scope.spawn(move || {
                            thread::sleep(Duration::from_millis(ms));
                            match pidfd_send_signal(&fd, Signal::CONT) {
                                Err(rustix::io::Errno::SRCH) if replacement => Ok(()),
                                other => other,
                            }
                        })
                    });
                    tested.step_exact(1)?;
                    let elapsed = start.elapsed();
                    if let Some(handle) = job {
                        handle.join().map_err(|_| "thaw panicked")??;
                    }
                    Ok(elapsed)
                })?;
                ordinary.step_exact(1)?;
                compare(&tested, &ordinary, &mut all)?;
                let after = backend(&tested).local_worker_pids().unwrap();
                if before
                    .iter()
                    .zip(&after)
                    .enumerate()
                    .any(|(i, (a, b))| (a != b) != (replacement && i == owner))
                {
                    return Err("unexpected replacement set".into());
                }
                advance(&mut tested, &mut ordinary, 24, &mut all)?;
                writeln!(
                    log,
                    "{}",
                    json!({"source":source,"round":round,"width":args.workers,"positive_before":positive,"thaw_from_stop_ms":thaw,"elapsed_ms":elapsed.as_secs_f64()*1e3,"owner":owner,"pids":[before,after],"post_fault_ticks":24})
                )?;
                log.flush()?;
            }
        }
        fs::write(
            args.output.join("proof.json"),
            serde_json::to_vec_pretty(&json!({"rounds":args.rounds,
            "checked_advancing_public_ticks":u64::from(args.rounds)*300,"trajectory_sha256":format!("{:x}",all.finalize())}))?,
        )?;
        Ok(())
    }
}
#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    linux::run()
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("Worker suspension uses Linux process descriptors.");
    std::process::exit(2);
}
