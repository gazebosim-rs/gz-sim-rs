// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a loaded nonlinear shuttle through real ownership changes, process death and reset.
use clap::Parser;
#[path = "support/curved_world.rs"]
mod curved;
use curved::{SDF, body, imu, load, native};
use gz_physics::PhysicsBackend;
use gz_sim_scene::{
    queue_link_world_velocity, queue_link_world_wrench, reset_loaded_world,
    trajectory_workers::{self, TrajectoryProcessPool},
};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::PathBuf, time::Instant};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Opt {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 0)]
    seed: u64,
    #[arg(long,default_value_t=2,value_parser=clap::value_parser!(u32).range(1..))]
    epochs: u32,
    #[arg(long,default_value_t=770,value_parser=clap::value_parser!(u32).range(320..=100000))]
    ticks: u32,
    #[arg(long)]
    view: Option<PathBuf>,
}
#[cfg(unix)]
fn kill(pid: u32) -> Result<()> {
    use rustix::process::{Pid, Signal, kill_process};
    kill_process(
        Pid::from_raw(i32::try_from(pid)?).ok_or("invalid worker PID")?,
        Signal::KILL,
    )?;
    Ok(())
}
#[cfg(not(unix))]
fn kill(_: u32) -> Result<()> {
    Err("this worker loss test requires POSIX signals".into())
}
fn main() -> Result<()> {
    let opt = Opt::parse();
    let xml=SDF.replace("<collision name=\"box\">","<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>200</update_rate><imu/></sensor><collision name=\"box\">")
        .replace("0 0 0.315",&format!("0 0 {:.17}",0.313+0.00035*(opt.seed%13) as f64));
    let xml = curved::vary(xml, opt.seed);
    let sdf = sdformat::load_str(&xml, "shuttle.sdf", &sdformat::NoResources)?;
    let mut local = load(&xml, None)?;
    let mut remote = load(&xml, Some(&opt.worker))?;
    let (cargo_id, cargo) = body(&remote, "cargo")?;
    let (deck_id, deck) = body(&remote, "platform")?;
    let mut frames = Vec::new();
    let mut evidence = Vec::new();
    let mut dense = Vec::new();
    let mut past = Vec::new();
    for epoch in 0..opt.epochs {
        if epoch > 0 {
            reset_loaded_world(&mut local, |_| ())?;
            reset_loaded_world(&mut remote, |_| ())?;
        }
        let mut totals = [0u64; 5];
        let mut times = Vec::new();
        let mut run = Sha256::new();
        for tick in 0..=opt.ticks {
            let mut killed = false;
            let mut moved = false;
            let mut rehydrated = false;
            if tick > 0 {
                let prev = remote.committed_update_info()?;
                let slot = remote.world().resource::<TrajectoryProcessPool>().owner();
                if [8, 13, 42, 154, 280, 281, 440, 441, 719, 720].contains(&tick)
                    || tick % 197 == 114
                {
                    trajectory_workers::select(&mut remote, prev.epoch, prev.iteration, 1 - slot)?;
                    moved = true;
                }
                let p = remote.world().resource::<TrajectoryProcessPool>();
                if ([10, 11, 43, 156, 282, 443, 721].contains(&tick)
                    || tick % 169 == 70 + ((opt.seed % 11) as u32))
                    && let Some(pid) = p.pid(p.owner())
                {
                    kill(pid)?;
                    killed = true;
                }
                if [36, 89, 175, 279, 442, 800].contains(&tick) {
                    for s in [&mut remote, &mut local] {
                        queue_link_world_wrench(
                            s,
                            cargo_id,
                            [0.7, -0.55, 0.35],
                            [0.004, 0., -0.002],
                        )?;
                    }
                }
                // Replace the moving deck's velocity only after one full lap and the recovery at its seam.
                if tick == 740 {
                    for s in [&mut remote, &mut local] {
                        queue_link_world_velocity(s, deck_id, [0.08, -0.03, 0.], [0.; 3])?;
                    }
                }
                local.step_exact(1)?;
                let clock = Instant::now();
                remote.step_exact(1)?;
                times.push(clock.elapsed().as_micros() as u64);
                let m = remote
                    .world()
                    .resource::<TrajectoryProcessPool>()
                    .last()
                    .ok_or("missing worker receipt")?;
                if m.restarted != killed {
                    return Err(format!("unexpected restart at {tick}").into());
                }
                rehydrated = m.transferred_source;
                totals[0] += u64::from(moved);
                totals[1] += u64::from(killed);
                totals[2] += m.source_bytes as u64;
                totals[3] += m.successor_bytes as u64;
            }
            let state = native(&remote).checkpoint_trajectory()?;
            if state != native(&local).checkpoint_trajectory()? || imu(&remote)? != imu(&local)? {
                return Err(format!(
                    "different complete native contact state or IMU at {epoch}/{tick}"
                )
                .into());
            }
            let sha = Sha256::digest(&state);
            run.update(sha);
            let (sub, hits) = native(&remote).translation_interval();
            let j = hits.iter().map(|c| c.normal_impulse).sum::<f64>();
            totals[4] += u64::from(j > 0.);
            if epoch == 0 && opt.view.is_some() && tick <= 1440 {
                let r = remote.world().resource::<PhysicsBackend>();
                let a = r.state(deck).unwrap();
                let b = r.state(cargo).unwrap();
                let row = json!({"tick":tick,"time_ns":remote.elapsed().as_nanos(),"deck":a.position,"cargo":b.position,"cargo_orientation":b.orientation,
                    "native_sha256":format!("{sha:x}"),"normal_impulse":j,"native_substeps":sub,"migrated":moved,"killed":killed,
                    "owner":remote.world().resource::<TrajectoryProcessPool>().owner(),"rehydrated":rehydrated});
                dense.push(row.clone());
                if tick % 5 == 0 || killed || moved {
                    let seq = frames.len() as u64 + 1;
                    let a = gz_sim_cli::primitive_frame(&remote, &sdf, "default", seq)?;
                    if a != gz_sim_cli::primitive_frame(&local, &sdf, "default", seq)? {
                        return Err("public geometry diverged".into());
                    }
                    evidence.push(row);
                    frames.push(a);
                }
            }
        }
        let hash = format!("{:x}", run.finalize());
        if past.first().is_some_and(|r| r != &hash) {
            return Err("loaded reset changed the sequence of complete native states".into());
        }
        past.push(hash.clone());
        times.sort_unstable();
        println!(
            "{}",
            json!({"seed":opt.seed,"epoch":epoch,"ticks":opt.ticks,"verified_native_cuts":opt.ticks+1,"imu_checks":opt.ticks+1,"loaded_intervals":totals[4],
            "migrations":totals[0],"worker_kills":totals[1],"source_bytes":totals[2],"successor_bytes":totals[3],"p50_us":times[times.len()/2],"p95_us":times[(times.len()*95).div_ceil(100)-1],"digest":hash})
        );
    }
    if let Some(dir) = opt.view {
        fs::create_dir_all(&dir)?;
        gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
        fs::write(
            dir.join("index.json"),
            serde_json::to_vec(&json!({"frames":evidence,"seed":opt.seed,"step_ns":5_000_000}))?,
        )?;
        fs::write(dir.join("dense.json"), serde_json::to_vec(&dense)?)?;
        fs::write(dir.join("world.sdf"), xml)?;
    }
    Ok(())
}
