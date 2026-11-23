// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a production worker-owned floating hinge to an uninterrupted authored SDF world.
use clap::Parser;
use gz_physics_distributed::articulation::{
    ArticulationObservation, ArticulationPool, ArticulationSpec,
};
use gz_sim_scene::distributed_hinge_spec;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::Instant,
};
#[path = "articulated_handoff/fixture.rs"]
#[allow(dead_code)]
mod control;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
#[path = "production_hinge/view.rs"]
mod view;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_articulation.sdf");
const TICKS: u64 = 384;
#[derive(Parser)]
pub struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t=4, value_parser=clap::value_parser!(u8).range(2..=8))]
    workers: u8,
    /// Reverse the entire internal-effort schedule.
    #[arg(long)]
    reverse: bool,
    /// Measure this many additional warm transfers on the identical loaded cut after tick 384.
    #[arg(long, default_value_t = 0)]
    transfer_samples: u16,
    /// Place the native session here instead of OUTPUT/ledger. Requires a fresh directory.
    #[arg(long)]
    ledger_directory: Option<PathBuf>,
    /// New directory for the native ledger, measured states and gz-web-rs recording.
    output: PathBuf,
}

fn pids(pool: &ArticulationPool, width: u8) -> Result<Vec<u32>> {
    (0..width)
        .map(|slot| {
            pool.worker_pid(slot)
                .ok_or_else(|| "expected every authoritative process to be present".into())
        })
        .collect()
}
fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn check_observation(
    tick: u64,
    spec: &ArticulationSpec,
    c: &control::Tree,
    actual: &ArticulationObservation,
) -> Result<control::Observation> {
    let reference = c.observe();
    if tick > 0 {
        c.check_publication(&reference)?;
    }
    if c.time_ns() != tick * 1_000_000
        || actual.tick != tick
        || actual.joint_id != spec.joint_id
        || actual.q.to_bits() != reference.q.to_bits()
        || actual.qdot.to_bits() != reference.qdot.to_bits()
    {
        return Err(format!("joint or clock differs from ordinary SDF at tick {tick}: actual ({:?}, {:?}), reference ({:?}, {:?})",actual.q,actual.qdot,reference.q,reference.qdot).into());
    }
    for (i, (measured, expected)) in actual.links.iter().zip(reference.links).enumerate() {
        if measured.id != spec.links[i].id
            || measured.position.map(f64::to_bits) != expected.position.map(f64::to_bits)
            || measured.orientation.map(f64::to_bits) != expected.orientation.map(f64::to_bits)
            || measured.velocity.map(f64::to_bits) != expected.velocity.map(f64::to_bits)
            || measured.angular_velocity.map(f64::to_bits)
                != expected.angular_velocity.map(f64::to_bits)
        {
            return Err(format!(
                "link {} differs from the independent SDF at tick {tick}",
                measured.id
            )
            .into());
        }
    }
    Ok(reference)
}
fn check_image(
    tick: u64,
    spec: &ArticulationSpec,
    c: &control::Tree,
    pool: &ArticulationPool,
) -> Result<()> {
    if tick > 0 && spec.warm_payload(tick, pool.image())? != c.image()? {
        return Err(format!("native coordinates or pending effort differ at tick {tick}").into());
    }
    Ok(())
}
fn effort_at(tick: u64, sign: f64) -> Vec<f64> {
    let raw: &[f64] = match tick {
        0..=41 => &[4.0],
        96 => &[-0.8125, 0.125],
        128 => &[1.375, -0.25],
        192 => &[0.75, -0.125, 0.0625],
        201..=230 => &[-3.],
        264 => &[-1.125],
        296 => &[0.875],
        320 => &[-0.34375, 0.0625],
        _ => &[],
    };
    raw.iter().map(|v| v * sign).collect()
}
#[cfg(target_os = "linux")]
fn kill_owner(pid: u32) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let fd = pidfd_open(
        Pid::from_raw(pid.try_into()?).ok_or("invalid PID")?,
        PidfdFlags::empty(),
    )?;
    pidfd_send_signal(&fd, Signal::KILL)?;
    Ok(())
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    qualify(Args::parse())
}

pub fn qualify(args: Args) -> Result<()> {
    fs::create_dir(&args.output)?;
    let ledger = args
        .ledger_directory
        .unwrap_or_else(|| args.output.join("ledger"));
    let spec = distributed_hinge_spec(SDF, "multifidelity_articulation.sdf", "default")?;
    let mut pool = ArticulationPool::create(&args.worker, &ledger, args.workers, spec)?;
    let mut reference = control::Tree::fresh()?;
    let mut trace = BufWriter::new(File::create(args.output.join("states.jsonl"))?);
    let mut trajectory = Sha256::new();
    let mut events = vec![];
    let mut stream = Vec::new();
    let mut index = vec![];
    let mut warm_comparisons = 0;
    let mut held_comparisons = 0;
    let mut waits = Vec::new();
    let mut killed: Option<(u8, Vec<u32>)> = None;
    let mut no_effort_control: Option<(u64, control::Tree)> = None;
    let mut impulses_observable = Vec::new();
    let mut peak_qdot: f64 = 0.;
    let mut largest_gap: f64 = 0.;
    for tick in 0..=TICKS {
        let mut at = vec![];
        if tick > 0 {
            let started = Instant::now();
            pool.advance()?;
            let wall_ns = u64::try_from(started.elapsed().as_nanos())?;
            reference.step(None)?;
            if let Some((slot, old)) = killed.take() {
                let new = pids(&pool, args.workers)?;
                if old
                    .iter()
                    .zip(&new)
                    .enumerate()
                    .any(|(i, (a, b))| (a != b) != (i == usize::from(slot)))
                {
                    return Err("failed owner was not exclusively replaced".into());
                }
                at.push(json!({"kind":"owner_replaced","tick":tick,"owner":slot,"before":old,"after":new,"advance_ns":wall_ns}));
            } else {
                waits.push(wall_ns);
            }
            if let Some((cut, mut other)) = no_effort_control.take() {
                other.step(None)?;
                let difference = (pool.observation().qdot - other.observe().qdot).abs();
                if difference < 1e-7 {
                    return Err(
                        format!("held effort at {cut} had no observable physical effect").into(),
                    );
                }
                impulses_observable.push(
                    json!({"queued_at":cut,"at":tick,"delta_qdot_rad_s_vs_omitted":difference}),
                );
            }
        }
        let control = check_observation(tick, &spec, &reference, pool.observation())?;
        check_image(tick, &spec, &reference, &pool)?;
        warm_comparisons += usize::from(tick > 0);
        let efforts = effort_at(tick, if args.reverse { -1. } else { 1. });
        if [96, 128, 192, 264, 296, 320].contains(&tick) {
            no_effort_control = Some((tick, control::Tree::restore(&reference.image()?)?));
        }
        for &e in &efforts {
            pool.queue_effort(spec.joint_id, e)?;
            reference.hold_effort(e)?;
            check_image(tick, &spec, &reference, &pool)?;
            held_comparisons += usize::from(tick > 0);
        }
        let cut = pool.image().to_vec();
        let owner = pool.owner();
        let generation_before = pool.generation();
        let epoch = pool.epoch();
        if [96, 264].contains(&tick) {
            let dest = if tick == 96 { args.workers - 1 } else { 0 };
            let old = pids(&pool, args.workers)?;
            let start = Instant::now();
            pool.transfer(dest)?;
            let ns = start.elapsed().as_nanos();
            if pool.owner() != dest
                || pool.generation() != generation_before + 1
                || pool.epoch() != epoch
                || pids(&pool, args.workers)? != old
                || pool.image() != cut
            {
                return Err(
                    format!("transfer did not retain the complete moving cut at {tick}").into(),
                );
            }
            at.push(json!({"kind":"transfer","tick":tick,"from":owner,"to":dest,"generation_before":generation_before,"generation_after":pool.generation(),"simulation_epoch":epoch,"pid_before":old[usize::from(owner)],"pid_after":old[usize::from(dest)],"ns":ns}));
        }
        if [192, 320].contains(&tick) {
            let old = pids(&pool, args.workers)?;
            let start = Instant::now();
            drop(pool);
            pool = ArticulationPool::resume(&args.worker, &ledger)?;
            let ns = start.elapsed().as_nanos();
            let new = pids(&pool, args.workers)?;
            if pool.owner() != owner
                || pool.generation() != generation_before
                || pool.epoch() != epoch
                || new.iter().any(|pid| old.contains(pid))
                || pool.image() != cut
            {
                return Err("fresh process restore changed motion, pending torque or owner".into());
            }
            at.push(json!({"kind":"disk_resume","tick":tick,"owner":owner,"generation_before":generation_before,"generation_after":pool.generation(),"simulation_epoch":epoch,"before":old,"after":new,"ns":ns}));
        }
        #[cfg(target_os = "linux")]
        if [128, 296].contains(&tick) {
            let old = pids(&pool, args.workers)?;
            kill_owner(old[usize::from(owner)])?;
            at.push(json!({"kind":"owner_sigkill","tick":tick,"owner":owner,"pid":old[usize::from(owner)]}));
            killed = Some((owner, old));
        }
        if [96, 128, 192, 264, 296, 320].contains(&tick) {
            let cut_dir = args.output.join("cuts").join(format!("tick-{tick}"));
            fs::create_dir_all(&cut_dir)?;
            fs::copy(
                ledger.join("barrier.gzhinge"),
                cut_dir.join("barrier.gzhinge"),
            )?;
            fs::write(cut_dir.join("worker.image"), pool.image())?;
            fs::write(cut_dir.join("direct.gzart"), reference.image()?)?;
        }
        check_observation(tick, &spec, &reference, pool.observation())?;
        check_image(tick, &spec, &reference, &pool)?;
        let state = pool.observation();
        trajectory.update(bincode::serialize(&(tick, state))?);
        peak_qdot = peak_qdot.max(state.qdot.abs());
        let row = json!({"tick":tick,"ns":tick*1_000_000,"owner":pool.owner(),"generation":pool.generation(),"simulation_epoch":pool.epoch(),"efforts_nm":efforts,"native_image_sha256":hash(pool.image()),"state":state,"direct_sdf":control,"events":at});
        serde_json::to_writer(&mut trace, &row)?;
        writeln!(trace)?;
        events.extend(at.iter().cloned());
        if tick.is_multiple_of(4) || !at.is_empty() {
            let (frame, gap) = view::draw(
                &spec,
                state,
                &control,
                view::Authority {
                    owner: pool.owner(),
                    width: args.workers,
                    events: &at,
                },
                &efforts,
                stream.len() as u64 + 1,
            )?;
            largest_gap = largest_gap.max(gap);
            index.push(json!({"ordinal":stream.len()+1,"tick":tick,"events":at}));
            stream.push(frame);
        }
    }
    trace.flush()?;
    if peak_qdot < 1. || largest_gap > 1e-9 || impulses_observable.len() != 6 {
        return Err("angular excitation, anchor closure or held-input check failed".into());
    }
    waits.sort_unstable();
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), stream)?;
    let terminal_q = pool.observation().q;
    if args.transfer_samples > 0 {
        let held = -0.53125;
        reference.hold_effort(held)?;
        pool.queue_effort(spec.joint_id, held)?;
        let cut = pool.image().to_owned();
        let before = pids(&pool, args.workers)?;
        let mut transfers = Vec::new();
        for i in 0..u32::from(args.transfer_samples) + 2 {
            let from = pool.owner();
            let destination = (from + 1) % args.workers;
            let generation = pool.generation();
            let started = Instant::now();
            pool.transfer(destination)?;
            let ns = u64::try_from(started.elapsed().as_nanos())?;
            if pool.owner() != destination
                || pool.generation() != generation + 1
                || pool.image() != cut
                || before != pids(&pool, args.workers)?
            {
                return Err(
                    "timed handoff changed the committed cut or lost a live process".into(),
                );
            }
            if i >= 2 {
                transfers.push(ns);
            }
        }
        check_image(TICKS, &spec, &reference, &pool)?;
        pool.advance()?;
        reference.step(None)?;
        check_observation(TICKS + 1, &spec, &reference, pool.observation())?;
        check_image(TICKS + 1, &spec, &reference, &pool)?;
        let mut sorted = transfers.clone();
        sorted.sort_unstable();
        let summary = json!({"workers":args.workers,"ledger":ledger,"samples":args.transfer_samples,"excluded_warmups":2,
            "cut_tick":TICKS,"pending_effort_nm":held,"cut_bytes":cut.len(),"cut_sha256":hash(&cut),
            "complete_live_transfer_ns":transfers,"p50_ns":sorted[sorted.len().div_ceil(2)-1],
            "p95_ns":sorted[(sorted.len()*95).div_ceil(100)-1],"p99_ns":sorted[(sorted.len()*99).div_ceil(100)-1],
            "worker_pids":before,"next_native_step_exact":true});
        fs::write(
            args.output.join("timing.json"),
            serde_json::to_vec_pretty(&summary)?,
        )?;
    }
    let summary = json!({"workers":args.workers,"reverse":args.reverse,"frames_checked":TICKS+1,"last_tick":TICKS,"native_steps":TICKS,"held_efforts_exact_images":held_comparisons,"exact_native_images_after_advance":warm_comparisons,"last_q_rad":terminal_q,"peak_qdot_rad_s":peak_qdot,"maximum_displayed_anchor_gap_m":largest_gap,"moving_state_sha256":format!("{:x}",trajectory.finalize()),"normal_advance_ns_p50":waits[waits.len().div_ceil(2)-1],"normal_advance_ns_p95":waits[(waits.len()*95).div_ceil(100)-1],"held_effort_controls":impulses_observable,"events":events,"frames":index});
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!(
        "{} exact authored observations, {warm_comparisons} advancing native images, {held_comparisons} held-input images, max anchor gap {largest_gap:e} m",
        TICKS + 1
    );
    Ok(())
}
