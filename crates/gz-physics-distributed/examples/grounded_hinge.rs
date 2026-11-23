// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare one worker-owned hinge against an independently loaded Bevy SDF through ground impact.
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
#[path = "grounded_hinge/reference.rs"]
mod reference;
#[path = "grounded_hinge/view.rs"]
mod view;

use clap::Parser;
use gz_physics_distributed::articulation::{
    ArticulationLinkState, ArticulationObservation, ArticulationPool, ArticulationSpec,
};
use gz_sim_scene::distributed_hinge_spec;
use reference::Reference;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOURCE: &str = "distributed_grounded_hinge.sdf";
const SDF: &str = include_str!("../../../examples/distributed_grounded_hinge.sdf");
const LAST: u64 = 640;
#[derive(Parser)]
pub struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(2..=8))]
    workers: u8,
    /// Reverse both the flight drive and each queued effort at a fault boundary.
    #[arg(long)]
    reverse: bool,
    /// An unused directory for the durable ledger. Defaults to OUTPUT/ledger.
    #[arg(long)]
    ledger_directory: Option<PathBuf>,
    /// New directory for contact measurements, native cuts and the gz-web-rs replay.
    output: PathBuf,
}

fn pids(pool: &ArticulationPool) -> Result<Vec<u32>> {
    (0..pool.width())
        .map(|i| {
            pool.worker_pid(i)
                .ok_or_else(|| "missing worker process".into())
        })
        .collect()
}

#[cfg(target_os = "linux")]
fn kill_owner(pid: u32) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let fd = pidfd_open(
        Pid::from_raw(pid.try_into()?).ok_or("invalid PID")?,
        PidfdFlags::empty(),
    )?;
    Ok(pidfd_send_signal(&fd, Signal::KILL)?)
}

fn verify(
    reference: &Reference,
    spec: &ArticulationSpec,
    pool: &ArticulationPool,
    tick: u64,
) -> Result<ArticulationObservation> {
    let measured = reference.observe(spec, tick)?;
    if &measured != pool.observation() {
        return Err(format!(
            "native observations differ at {tick}: direct={measured:?}, worker={:?}",
            pool.observation()
        )
        .into());
    }
    Ok(measured)
}

fn check_image(
    reference: &Reference,
    spec: &ArticulationSpec,
    pool: &ArticulationPool,
    tick: u64,
) -> Result<()> {
    if tick > 0 && reference.image()? != spec.warm_payload(tick, pool.image())? {
        return Err(format!("moving native image differs at tick {tick}").into());
    }
    Ok(())
}

fn save_cut(output: &Path, ledger: &Path, tick: u64) -> Result<()> {
    let dir = output.join("cuts").join(format!("tick-{tick}"));
    fs::create_dir_all(&dir)?;
    fs::copy(ledger.join("barrier.gzhinge"), dir.join("barrier.gzhinge"))?;
    Ok(())
}

#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    qualify(Args::parse())
}

pub fn qualify(args: Args) -> Result<()> {
    let dir = &args.output;
    fs::create_dir(dir)?;
    let ledger = args.ledger_directory.unwrap_or_else(|| dir.join("ledger"));
    let spec = distributed_hinge_spec(SDF, SOURCE, "default")?;
    let ground = spec.ground.ok_or("SDF must include a floor")?;
    let mut pool = ArticulationPool::create(&args.worker, &ledger, args.workers, spec)?;
    let mut reference = Reference::load(&spec)?;
    let sign = if args.reverse { -1. } else { 1. };
    let mut trace = BufWriter::new(File::create(dir.join("states.jsonl"))?);
    let mut frames = vec![];
    let mut frame_ticks = vec![];
    let mut events = vec![];
    let mut digest = Sha256::new();
    let mut history = vec![];
    let mut omitted: Option<(u64, Reference)> = None;
    let mut counterfactuals = vec![];
    let mut loaded_cuts = 0;
    let mut eligible_after = 0;
    let mut killed: Option<(u8, Vec<u32>)> = None;
    let total_mass = spec.links.iter().map(|s| s.mass).sum::<f64>();
    let weight = -ground.gravity_z * total_mass;
    let initial_momentum: [f64; 3] =
        std::array::from_fn(|axis| spec.links.iter().map(|s| s.mass * s.velocity[axis]).sum());
    let mut impulse = [0.0; 3];
    let mut max_balance_error = [0.0_f64; 3];
    let mut support_ticks = [0u64; 2];
    let mut first_impact = None;
    let mut peak_force: f64 = 0.;
    let mut warm_ticks = 0;
    let mut exact_held_images = 0;
    let mut tail_sum = 0.;
    let mut tail_begin_momentum = 0.;
    let mut tail_end_momentum = 0.;
    for tick in 0..=LAST {
        let mut event = None;
        if tick > 0 {
            pool.advance()?;
            reference.step()?;
        }
        let direct = verify(&reference, &spec, &pool, tick)?;
        check_image(&reference, &spec, &pool, tick)?;
        if let Some((from, mut no_input)) = omitted.take() {
            no_input.step()?;
            let alt = no_input.observe(&spec, tick)?;
            let delta = (direct.qdot - alt.qdot).abs();
            if delta < 1e-9 {
                return Err(format!("omitted input at {from} had no joint effect").into());
            }
            counterfactuals
                .push(json!({"queued_at":from,"tick":tick,"absolute_delta_qdot_rad_s":delta}));
        }
        if let Some((slot, old)) = killed.take() {
            let new = pids(&pool)?;
            if old
                .iter()
                .zip(&new)
                .enumerate()
                .any(|(i, (a, b))| (a != b) != (i == usize::from(slot)))
            {
                return Err("owner replacement changed wrong processes".into());
            }
            events.push(json!({"kind":"replaced","at":tick,"slot":slot,"before":old,"after":new}));
            event = Some("replaced");
        }
        let contacts = direct.ground_reactions.ok_or("missing ground data")?;
        let normal = contacts.map(|c| c.normal_impulse);
        let force = (normal[0] + normal[1]) * 1000.;
        let warm = contacts.iter().any(|r| r.warmstart_normal_impulse > 0.);
        for (i, r) in contacts.iter().enumerate() {
            if !r.normal_impulse.is_finite()
                || r.normal_impulse < 0.
                || (!r.active && r.normal_impulse > 0.)
            {
                return Err("nonphysical reaction".into());
            }
            support_ticks[i] += u64::from(r.normal_impulse > 0.);
        }
        if force > 0. {
            first_impact.get_or_insert(tick);
        }
        warm_ticks += u64::from(warm);
        peak_force = peak_force.max(force);
        let momentum_error: [f64; 3] = std::array::from_fn(|axis| {
            impulse[axis] += contacts
                .iter()
                .map(|c| c.tangent_impulse[axis])
                .sum::<f64>();
            if axis == 2 {
                impulse[axis] += normal[0] + normal[1];
            }
            let momentum: f64 = spec
                .links
                .iter()
                .zip(direct.links)
                .map(|(s, l)| s.mass * l.velocity[axis])
                .sum();
            let gravity = if axis == 2 {
                -weight * tick as f64 * 0.001
            } else {
                0.
            };
            let residual = (momentum - initial_momentum[axis]) - (impulse[axis] + gravity);
            max_balance_error[axis] = max_balance_error[axis].max(residual.abs());
            residual
        });
        if tick == LAST - 100 || tick == LAST {
            let pz = spec
                .links
                .iter()
                .zip(direct.links)
                .map(|(s, l)| s.mass * l.velocity[2])
                .sum::<f64>();
            if tick == LAST {
                tail_end_momentum = pz;
            } else {
                tail_begin_momentum = pz;
            }
        }
        if tick > LAST - 100 {
            tail_sum += force;
        }
        let fault_ready = force > 0.05 * weight
            && tick >= eligible_after
            && loaded_cuts < 6
            && tick < LAST
            && (loaded_cuts == 0 || warm);
        let mut efforts = if tick < 40 { vec![sign * 1.5] } else { vec![] };
        if fault_ready {
            omitted = Some((tick, Reference::without_next_effort(&spec, &history)?));
            let amplitude = sign * if loaded_cuts % 2 == 0 { 1.5 } else { -1.75 };
            efforts.extend([amplitude, -amplitude * 0.125]);
        }
        for &f in &efforts {
            pool.queue_effort(spec.joint_id, f)?;
            reference.queue(spec.joint_id, f)?;
            check_image(&reference, &spec, &pool, tick)?;
            exact_held_images += u64::from(tick > 0);
        }
        history.push(efforts.clone());
        if fault_ready {
            let slot = pool.owner();
            let before = pids(&pool)?;
            let generation = pool.generation();
            let stored = pool.image().to_vec();
            let kind = match loaded_cuts % 3 {
                0 => {
                    let dest = if slot == 0 { args.workers - 1 } else { 0 };
                    pool.transfer(dest)?;
                    if pool.owner() != dest
                        || pids(&pool)? != before
                        || pool.generation() != generation + 1
                    {
                        return Err("loaded transfer lost source/destination identity".into());
                    }
                    "transfer"
                }
                1 => {
                    #[cfg(not(target_os = "linux"))]
                    return Err("actual owner SIGKILL qualification requires Linux".into());
                    #[cfg(target_os = "linux")]
                    {
                        kill_owner(before[usize::from(slot)])?;
                        killed = Some((slot, before.clone()));
                        "kill"
                    }
                }
                _ => {
                    drop(pool);
                    pool = ArticulationPool::resume(&args.worker, &ledger)?;
                    if pool.owner() != slot
                        || pool.generation() != generation
                        || pids(&pool)?.iter().any(|id| before.contains(id))
                    {
                        return Err("expected a fresh pool on original committed owner".into());
                    }
                    "resume"
                }
            };
            if pool.image() != stored || pool.epoch() != 0 {
                return Err("fault changed committed native cut".into());
            }
            verify(&reference, &spec, &pool, tick)?;
            let info = json!({
                "kind":kind, "at":tick, "slot_before":slot, "slot_after":pool.owner(),
                "generation":pool.generation(), "pids_before":before, "pids_after":pids(&pool)?,
                "force_n":force, "reactions":contacts, "held_torque_nm":efforts.iter().sum::<f64>(),
                "image_bytes":pool.image().len()
            });
            events.push(info);
            event = Some(kind);
            save_cut(dir, &ledger, tick)?;
            loaded_cuts += 1;
            eligible_after = tick + 32;
        }
        let state = pool.observation();
        digest.update(bincode::serialize(state)?);
        let row = json!({"tick":tick,"owner":pool.owner(),"event":event,"efforts_nm":efforts,"direct":direct,"worker":state,"total_normal_force_n":force,"momentum_residual_ns":momentum_error});
        serde_json::to_writer(&mut trace, &row)?;
        writeln!(trace)?;
        if tick % 5 == 0 || event.is_some() || first_impact == Some(tick) {
            frame_ticks.push(tick);
            frames.push(view::draw(
                &spec,
                &direct,
                state,
                pool.owner(),
                args.workers,
                event,
                frame_ticks.len() as u64,
            )?);
        }
    }
    trace.flush()?;
    if loaded_cuts != 6
        || counterfactuals.len() != 6
        || first_impact.is_none_or(|t| t < 2)
        || support_ticks.contains(&0)
        || warm_ticks == 0
        || peak_force < weight
    {
        return Err(format!("fixture did not exercise loaded impact and warm support: {loaded_cuts} cuts, {support_ticks:?} loaded link ticks").into());
    }
    // Normalize the impulse budget by the measured free-fall momentum scale.
    let balance_limit = weight * first_impact.unwrap() as f64 * 0.001 * 0.01;
    if max_balance_error
        .iter()
        .any(|e| !e.is_finite() || *e > balance_limit)
    {
        return Err(format!(
            "ground impulse balance {max_balance_error:?} exceeds {balance_limit} N s"
        )
        .into());
    }
    let mean_from_momentum = weight + (tail_end_momentum - tail_begin_momentum) / 0.1;
    gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
    let summary = json!({
        "workers": args.workers,
        "reverse": args.reverse,
        "states_compared": LAST + 1,
        "advancing_images_exact": LAST,
        "warm_effort_images_exact": exact_held_images,
        "first_impact_ms": first_impact,
        "loaded_link_ticks": support_ticks,
        "warmstart_ticks": warm_ticks,
        "peak_normal_force_n": peak_force,
        "weight_n": weight,
        "mean_total_force_last_100_n": tail_sum / 100.,
        "mean_force_from_vertical_momentum_last_100_n": mean_from_momentum,
        "per_axis_momentum_acceptance_ns": balance_limit,
        "cumulative_ground_impulse_ns": impulse,
        "max_momentum_residual_ns": max_balance_error,
        "state_digest": format!("{:x}", digest.finalize()),
        "events": events,
        "counterfactuals": counterfactuals,
        "frames": frame_ticks
    });
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!(
        "{} exact states, {} loaded cuts, peak normal force {:.4} N, final 100-step mean {:.4} N",
        LAST + 1,
        loaded_cuts,
        peak_force,
        tail_sum / 100.
    );
    Ok(())
}
