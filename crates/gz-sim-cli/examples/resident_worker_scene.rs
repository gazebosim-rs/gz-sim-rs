// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the ordinary SDF, resize its process pool and kill a real worker around impact.
use clap::Parser;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    ImuSample, PhysicsEngine, WorldLoadOptions,
    gpu_scene::{
        PairProcessPool, SpherePairBlockCheckpoint as Cut, batch_boundary,
        resident_pair_observation, set_resident_worker_count,
    },
    load_world_on_adapter, reset_loaded_world,
};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/gpu_resident_contacts.sdf");
#[derive(Parser)]
struct Options {
    #[arg(long)]
    adapter: String,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 0)]
    seed: u64,
    #[arg(long, default_value_t=2,value_parser=clap::value_parser!(u32).range(1..))]
    epochs: u32,
    /// Stop a worker without closing its pipes at two barriers to exercise the real wall timeout.
    #[arg(long)]
    stall_workers: bool,
    #[arg(long)]
    view: Option<PathBuf>,
}
fn load(xml: &str, adapter: &str, worker: Option<&Path>) -> Result<Simulation> {
    Ok(load_world_on_adapter(
        xml,
        "gpu_resident_contacts.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: worker,
        },
        Some(adapter),
    )?)
}
fn imu(sim: &Simulation) -> Result<Option<Value>> {
    let Some(sample) = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<ImuSample>())
    else {
        return Ok(None);
    };
    let v = sample
        .reading
        .as_ref()
        .map_err(|e| format!("invalid IMU: {e:?}"))?;
    Ok(Some(
        json!({"epoch":sample.epoch,"time_ns":sample.time.as_nanos(),"q":v.orientation,"specific_force":v.specific_force,"angular_velocity":v.angular_velocity,
        "angular_covariance":v.angular_velocity_covariance,"linear_covariance":v.linear_acceleration_covariance}),
    ))
}
#[cfg(unix)]
fn lose(pid: u32, stall: bool) -> Result<()> {
    use rustix::process::{Pid, Signal, kill_process};
    kill_process(
        Pid::from_raw(i32::try_from(pid)?).ok_or("invalid child pid")?,
        if stall { Signal::STOP } else { Signal::KILL },
    )?;
    Ok(())
}
#[cfg(not(unix))]
fn lose(_: u32, _: bool) -> Result<()> {
    Err("this process-death witness needs POSIX signals".into())
}
fn main() -> Result<()> {
    let opts = Options::parse();
    // A bounded horizontal offset and closing speed vary the physical impact while the sibling
    // consistently clears by one millimetre. Both SDFs receive exactly the same doubles.
    let mut rng = opts.seed;
    let mut draw = || {
        rng = rng
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (rng >> 11) as f64 / 9007199254740992.
    };
    let speed = 0.86 + 0.18 * draw();
    let offset = 0.025 + 0.045 * draw();
    let xml = SDF
        .replace("0.95", &format!("{speed:.17}"))
        .replace("0.05 -0.3 0.58", &format!("{offset:.17} -0.3 0.58"));
    let pooled = xml.replace("<resident_block ", "<resident_block workers=\"2\" ");
    let sdf = sdformat::load_str(&pooled, "resident_workers.sdf", &sdformat::NoResources)?;
    let mut ordinary = load(&xml, &opts.adapter, None)?;
    let mut distributed = load(&pooled, &opts.adapter, Some(&opts.worker))?;
    let mut captures = Vec::new();
    let mut frames = Vec::new();
    let mut first_hit = None;
    let mut hashes = Vec::new();
    let mut total_kills = 0usize;
    let mut total_stalls = 0usize;
    for epoch in 0..opts.epochs {
        if epoch > 0 {
            reset_loaded_world(&mut ordinary, |_| ())?;
            reset_loaded_world(&mut distributed, |_| ())?;
        }
        let start = Instant::now();
        let mut digest = Sha256::new();
        let mut hits = 0;
        let mut restarted = 0;
        let mut warm = 0;
        let mut traffic = [0u64; 2];
        let mut elapsed = Vec::new();
        for i in 0..=80u64 {
            let mut killed = false;
            let stall = opts.stall_workers && [16, 34].contains(&i);
            if i > 0 {
                let at = batch_boundary(&distributed);
                if [1, 12, 24, 42, 56].contains(&i) {
                    let width = if [12, 42].contains(&i) { 1 } else { 2 };
                    set_resident_worker_count(&mut distributed, at, width)?;
                }
                let p = distributed.world().resource::<PairProcessPool>();
                if i == 26 + opts.seed % 3 || i == 27 + opts.seed % 3 || i == 58 || i == 73 || stall {
                    let mut procs = p.processes();
                    let j = ((i + u64::from(epoch) + opts.seed % 16) % procs.len() as u64) as usize;
                    lose(procs.nth(j).ok_or("missing selected worker")?, stall)?;
                    if stall {
                        total_stalls += 1;
                    } else {
                        total_kills += 1;
                        killed = true;
                    }
                }
                ordinary.step_exact(1)?;
                distributed.step_exact(1)?;
                let m = distributed
                    .world()
                    .resource::<PairProcessPool>()
                    .last()
                    .ok_or("missing metrics")?;
                if m.restarted != usize::from(killed || stall) {
                    return Err("worker death was not recovered exactly once".into());
                }
                restarted += m.restarted;
                warm += m.reused;
                traffic[0] += m.request_bytes;
                traffic[1] += m.response_bytes;
                elapsed.push(m.latency.as_micros() as u64);
            }
            let a = Cut::capture_simulation(&ordinary)?.to_bytes()?;
            let b = Cut::capture_simulation(&distributed)?.to_bytes()?;
            if a != b || imu(&ordinary)? != imu(&distributed)? {
                return Err(
                    format!("different full cut or IMU at epoch {epoch}, interval {i}").into(),
                );
            }
            let sha = Sha256::digest(&b);
            digest.update(sha);
            let mut pairs = Vec::new();
            for (id, tag) in [(7, "impact"), (41, "clear")] {
                let r = resident_pair_observation(&distributed, id)?;
                if let Some((time, _)) = r.contact_times() {
                    if id == 41 {
                        return Err("near-miss lane produced contact".into());
                    }
                    hits += 1;
                    first_hit.get_or_insert((i, (i - 1) as f64 * 0.008 + time));
                }
                if opts.view.is_some() && epoch == 0 {
                    pairs.push(json!({"id":id,"name":tag,"state":r.state,"contact_ticks":r.contact_ticks(),"contact_times":r.contact_times(),"impulse_on_b":r.impulse_on_b}));
                }
            }
            if opts.view.is_some() && epoch == 0 {
                let scene = gz_sim_cli::primitive_frame(&distributed, &sdf, "default", i + 1)?;
                let ob = gz_sim_cli::primitive_frame(&ordinary, &sdf, "default", i + 1)?;
                if scene != ob {
                    return Err("ordinary SDF geometry differs after worker handoff".into());
                }
                let pool = distributed.world().resource::<PairProcessPool>();
                captures.push(json!({"tick":i,"pairs":pairs,"cut_sha256":format!("{sha:x}"),"imu":imu(&distributed)?,
                    "workers":pool.processes().len(),"killed":killed,"stopped":stall,"restarts":pool.last().map_or(0,|m|m.restarted),
                    "reply_bytes":pool.last().map_or(0,|m|m.response_bytes),"request_bytes":pool.last().map_or(0,|m|m.request_bytes),
                    "wait_us":pool.last().map_or(0,|m|m.wait.as_micros()),"process_us":pool.last().map_or(0,|m|m.latency.as_micros())}));
                frames.push(scene);
            }
        }
        if hits != 1 {
            return Err(format!("expected one contact interval, got {hits}").into());
        }
        elapsed.sort_unstable();
        hashes.push(format!("{:x}", digest.finalize()));
        println!(
            "{}",
            json!({"seed":opts.seed,"epoch":epoch,"barriers":80,"verified_full_cuts":81,"killed":restarted-usize::from(opts.stall_workers)*2,"stopped":usize::from(opts.stall_workers)*2,"warm_range_solves":warm,
            "request_bytes":traffic[0],"reply_bytes":traffic[1],"first_impact_time_s":first_hit.map(|v|v.1),"p95_us":elapsed[75],"elapsed_s":start.elapsed().as_secs_f64(),"digest":hashes.last()})
        );
    }
    if let Some(path) = opts.view {
        fs::create_dir(&path)?;
        let (hit, time) = first_hit.ok_or("missing impact")?;
        gz_sim_cli::export_archive(&path.join("replay"), Default::default(), frames)?;
        fs::write(path.join("world.sdf"), &pooled)?;
        fs::write(
            path.join("index.json"),
            serde_json::to_vec(
                &json!({"seed":opts.seed,"step_ns":8_000_000,"first_impact_barrier":hit,"impact_s":time,"workers_killed":total_kills,"workers_stopped":total_stalls,"epochs":opts.epochs,"frames":captures}),
            )?,
        )?;
    }
    Ok(())
}
