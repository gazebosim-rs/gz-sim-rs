// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a distributed three-body impact against the uninterrupted authored world.
#[path = "support/hinge_rig.rs"]
mod hinge_rig;
#[path = "moving_hinge/measure.rs"]
mod measure;
#[path = "support/primitive_replay_view.rs"]
#[allow(dead_code)]
mod primitive_replay_view;
#[path = "moving_hinge/reference.rs"]
mod reference;
#[path = "moving_hinge/view.rs"]
mod view;

use clap::Parser;
use gz_physics_distributed::moving::{MovingObservation, MovingPool, MovingSpec};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec;
use measure::{Differences, impulse_on_hinge, momentum};
use rapier3d_f64::math::Vector;
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
const SOURCE: &str = "distributed_hinge_impact.sdf";
const SDF: &str = include_str!("../../../examples/distributed_hinge_impact.sdf");
const LAST: u64 = 1600;

#[derive(Parser)]
pub struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(2..=8))]
    workers: u8,
    /// Reverse the sign of the applied torque.
    #[arg(long)]
    reverse: bool,
    /// Apply a positive torque magnitude in N·m on every interval, including recovered intervals.
    #[arg(long, num_args = 0..=1, require_equals = true, default_missing_value = "0.01", value_name = "NM")]
    continuous: Option<f64>,
    /// Directory for the live coordinator ledger. Defaults to OUTPUT/ledger.
    #[arg(long)]
    ledger_directory: Option<PathBuf>,
    /// New directory in which to write measurements, cuts and a gz-web-rs replay.
    output: PathBuf,
}
fn pids(pool: &MovingPool) -> Result<Vec<u32>> {
    (0..pool.width())
        .map(|i| {
            pool.worker_pid(i)
                .ok_or_else(|| "missing worker PID".into())
        })
        .collect()
}
fn steps(pool: &MovingPool) -> Result<Vec<u64>> {
    (0..pool.width())
        .map(|i| {
            pool.worker_steps(i)
                .ok_or_else(|| "missing native step count".into())
        })
        .collect()
}
#[cfg(target_os = "linux")]
fn kill(pid: u32) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let fd = pidfd_open(
        Pid::from_raw(pid.try_into()?).ok_or("invalid worker PID")?,
        PidfdFlags::empty(),
    )?;
    Ok(pidfd_send_signal(&fd, Signal::KILL)?)
}
#[cfg(not(target_os = "linux"))]
fn kill(_: u32) -> Result<()> {
    Err("production process fault qualification requires Linux pidfds".into())
}

fn save_cut(dir: &Path, ledger: &Path, tick: u64) -> Result<()> {
    let output = dir.join("cuts").join(format!("tick-{tick}"));
    fs::create_dir_all(&output)?;
    fs::copy(
        ledger.join("barrier.gzmoving"),
        output.join("barrier.gzmoving"),
    )?;
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
    let spec = distributed_moving_hinge_spec(SDF, SOURCE, "default")?;
    let mut pool = MovingPool::create(&args.worker, &ledger, args.workers, spec)?;
    let mut ordinary = Reference::load(&spec)?;
    let mut without_torque = Some(Reference::load(&spec)?);
    let magnitude = args.continuous.unwrap_or(2.25);
    if !magnitude.is_finite() || magnitude <= 0. {
        return Err("continuous torque magnitude must be finite and positive".into());
    }
    let continuous = args.continuous.is_some();
    let torque = if args.reverse { -magnitude } else { magnitude };
    let mut commands = 0_u64;
    let mut differences = Differences::default();
    let mut trace = BufWriter::new(File::create(dir.join("states.jsonl"))?);
    let mut frames = vec![];
    let mut frame_ticks = vec![];
    let mut events = vec![];
    let mut digest = Sha256::new();
    let (p0, l0) = momentum(&spec, &ordinary.observe(&spec, 0)?);
    let mut applied = Vector::ZERO;
    let mut max_linear = 0.0_f64;
    let mut max_angular = 0.0_f64;
    let mut max_loose = 0.0_f64;
    let mut peak_normal = 0.0_f64;
    let mut peak_tangent = 0.0_f64;
    let mut exact = 0;
    let mut independent_intervals = 0;
    let mut joined_intervals = 0;
    let mut impacts = 0;
    let mut join_tick = None;
    let mut first_active = None;
    let mut last_active = None;
    let mut last_loaded = None;
    let mut first_loaded = None;
    let mut previous_active = false;
    let mut handoff_tick = None;
    let mut split_tick = None;
    let mut omitted_delta = None;
    let mut killed: Option<(u8, Vec<u32>)> = None;
    for tick in 0..=LAST {
        let prior = pool.owners();
        let was_joined = prior[0] == prior[1];
        let mut event = None;
        if tick > 0 {
            let counts = steps(&pool)?;
            pool.advance()?;
            ordinary.advance()?;
            if let Some(alt) = &mut without_torque {
                alt.advance()?;
            }
            let owners = pool.owners();
            let is_joined = owners[0] == owners[1];
            if let Some((slot, old)) = killed.take() {
                let new = pids(&pool)?;
                if old
                    .iter()
                    .zip(&new)
                    .enumerate()
                    .any(|(i, (a, b))| (a != b) != (i == usize::from(slot)))
                {
                    return Err("unexpected physical processes replaced".into());
                }
                events.push(
                    json!({"kind":"replaced","at":tick,"slot":slot,"before":old,"after":new}),
                );
                event = Some("replaced");
            } else if prior == owners {
                let after = steps(&pool)?;
                // Check each worker's native advance count while its role remains stable.
                for (i, (before, after)) in counts.iter().zip(after).enumerate() {
                    let expected = u64::from(owners.contains(&(i as u8)));
                    if *before + expected != after {
                        return Err(format!("wrong native step count on worker {i} at tick {tick}: from {before} to {after}, owners {owners:?}").into());
                    }
                }
                if is_joined {
                    joined_intervals += 1;
                } else {
                    independent_intervals += 1;
                }
            }
            if is_joined != was_joined {
                let kind = if is_joined { "join" } else { "split" };
                events.push(json!({"kind":kind,"at":tick,"before":prior,"after":owners}));
                event = Some(kind);
                if is_joined && join_tick.is_none() {
                    join_tick = Some(tick);
                }
                if !is_joined && handoff_tick.is_some() && split_tick.is_none() {
                    split_tick = Some(tick);
                }
            }
        }
        let direct = ordinary.observe(&spec, tick)?;
        differences.check(&direct, pool.observation())?;
        if bincode::serialize(&direct)? == bincode::serialize(pool.observation())? {
            exact += 1;
        }
        if handoff_tick.is_some_and(|t| tick == t + 1) {
            let alt = without_torque.take().unwrap().observe(&spec, tick)?;
            let delta = (direct.hinge.qdot - alt.hinge.qdot).abs();
            if delta < 1e-8 {
                return Err("counterfactual omitted torque produced no response".into());
            }
            omitted_delta = Some(delta);
        }
        let (p, l) = momentum(&spec, &direct);
        let j = impulse_on_hinge(&direct);
        applied += j;
        let loose = spec.sphere.mass
            * (Vector::from(direct.sphere.velocity) - Vector::from(spec.sphere.velocity))
            + applied;
        max_linear = max_linear.max((p - p0).length());
        max_angular = max_angular.max((l - l0).length());
        max_loose = max_loose.max(loose.length());
        let normal = direct
            .reactions
            .iter()
            .map(|r| r.normal_impulse)
            .sum::<f64>();
        peak_normal = peak_normal.max(normal);
        peak_tangent = peak_tangent.max(
            direct
                .reactions
                .iter()
                .map(|r| Vector::from(r.tangent_impulse).length())
                .sum(),
        );
        let contact_active = direct.reactions.iter().any(|r| r.active);
        if contact_active {
            first_active.get_or_insert(tick);
            last_active = Some(tick);
        }
        let changed_active = previous_active != contact_active;
        previous_active = contact_active;
        if normal > 0. {
            first_loaded.get_or_insert(tick);
            last_loaded = Some(tick);
            impacts += 1;
            if pool.owners()[0] != pool.owners()[1] {
                return Err("positive contact while still on different workers".into());
            }
        }
        let warm: f64 = direct
            .reactions
            .iter()
            .map(|r| r.warmstart_normal_impulse)
            .sum();
        // The solver can report applied impulse before the first cached impulse.
        let loaded_handoff = normal > 0. && warm > 0. && handoff_tick.is_none();
        let queue = tick < LAST && (continuous || loaded_handoff);
        let queued_effort = if queue { torque } else { 0. };
        if queue {
            pool.queue_effort(spec.hinge.joint_id, torque)?;
            ordinary.queue(spec.hinge.joint_id, torque)?;
            // This reference omits only the command at the loaded handoff.
            if !loaded_handoff && let Some(alt) = &mut without_torque {
                alt.queue(spec.hinge.joint_id, torque)?;
            }
            commands += 1;
        }
        if tick == 4 {
            if was_joined {
                return Err("expected initial independent flight".into());
            }
            let ids = pids(&pool)?;
            let slot = pool.owners()[1];
            kill(ids[usize::from(slot)])?;
            events.push(json!({"kind":"independent_owner_kill","at":tick,"slot":slot,"pids":ids}));
            killed = Some((slot, ids));
            event = Some("owner_kill");
        }
        if loaded_handoff {
            handoff_tick = Some(tick);
            let owner = pool.owners()[0];
            let dest = (owner + 1) % args.workers;
            let image = pool.image().ok_or("missing loaded graph")?.to_vec();
            if MultiFidelityBackend::moving_checkpoint_effort(&image)? != Some(torque) {
                return Err("loaded image did not preserve pending joint torque".into());
            }
            let before = pids(&pool)?;
            pool.transfer(dest)?;
            if pool.owners() != [dest; 2]
                || pids(&pool)? != before
                || pool.image() != Some(image.as_slice())
            {
                return Err("loaded transfer did not preserve graph/owners".into());
            }
            save_cut(dir, &ledger, tick)?;
            drop(pool);
            pool = MovingPool::resume(&args.worker, &ledger)?;
            let restarted = pids(&pool)?;
            if restarted.iter().any(|p| before.contains(p))
                || pool.owners() != [dest; 2]
                || pool.image() != Some(image.as_slice())
            {
                return Err("loaded restart did not restore entire graph".into());
            }
            differences.check(&direct, pool.observation())?;
            kill(restarted[usize::from(dest)])?;
            events.push(json!({"kind":"loaded_handoff","at":tick,"from":owner,"to":dest,"torque_nm":torque,"image_bytes":image.len(),"normal_impulse_ns":normal,"warmstart_normal_impulse_ns":warm,"impulse_on_hinge_ns":j.to_array(),"old_pids":before,"reopened_pids":restarted,"killed_pid":restarted[usize::from(dest)]}));
            killed = Some((dest, restarted));
            event = Some("loaded_handoff");
        }
        if split_tick == Some(tick) {
            let owners = pool.owners();
            if direct
                .reactions
                .iter()
                .any(|r| r.active || r.normal_impulse != 0.)
            {
                return Err("split before contact release".into());
            }
            save_cut(dir, &ledger, tick)?;
            let before = pids(&pool)?;
            let barrier = ledger.join("barrier.gzmoving");
            let prior_cut = fs::read(&barrier)?;
            drop(pool);
            pool = MovingPool::resume(&args.worker, &ledger)?;
            let after = pids(&pool)?;
            if before.iter().any(|pid| after.contains(pid))
                || pool.owners() != owners
                || fs::read(&barrier)? != prior_cut
            {
                return Err("split reopen changed ownership or the durable image".into());
            }
            differences.check(&direct, pool.observation())?;
            events.push(json!({"kind":"released_resume","at":tick,"queued_effort_nm":queued_effort,"archive_bytes":prior_cut.len(),"owners":owners,"old_pids":before,"reopened_pids":after}));
            event = Some("released_resume");
        }
        let state = pool.observation();
        digest.update(bincode::serialize(state)?);
        let row = json!({"tick":tick,"direct":direct,"worker":state,"owners":pool.owners(),"event":event,"queued_effort_nm":queued_effort,"impulse_on_hinge_ns":j.to_array(),
            "linear_momentum":p.to_array(),"angular_momentum":l.to_array(),"sphere_impulse_residual_ns":loose.to_array()});
        serde_json::to_writer(&mut trace, &row)?;
        writeln!(trace)?;
        if tick % 8 == 0
            || changed_active
            || event.is_some()
            || (normal > 0. && handoff_tick.is_some_and(|t| tick - t < 8))
            || tick == LAST
        {
            frame_ticks.push(tick);
            frames.push(view::draw(
                &spec,
                &direct,
                state,
                pool.owners(),
                pool.width(),
                event,
                frame_ticks.len() as u64,
            )?);
        }
    }
    trace.flush()?;
    if commands != if continuous { LAST } else { 1 } {
        return Err("incorrect number of joint commands".into());
    }
    if join_tick.is_none()
        || join_tick > first_active
        || joined_intervals == 0
        || handoff_tick.is_none()
        || (!continuous && split_tick.is_none())
        || omitted_delta.is_none()
        || independent_intervals < 10
        || peak_tangent < 1e-8
    {
        return Err(format!("missing oblique, pre-joined and released impact: {handoff_tick:?}/{split_tick:?}, tangent {peak_tangent}").into());
    }
    if exact != LAST + 1 {
        return Err(format!(
            "{} of {} physical observations were bit-identical",
            exact,
            LAST + 1
        )
        .into());
    }
    let momentum_scale = spec.sphere.mass * Vector::from(spec.sphere.velocity).length();
    let encounter_length =
        (Vector::from(spec.sphere.position) - Vector::from(spec.hinge.anchor)).length();
    if max_linear > 0.01 * momentum_scale
        || max_loose > 1e-7 * momentum_scale
        || max_angular > 0.01 * momentum_scale * encounter_length
    {
        return Err(format!(
            "momentum balance residuals P={max_linear} L={max_angular} sphere={max_loose}"
        )
        .into());
    }
    gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
    let summary = json!({"workers":args.workers,"reverse":args.reverse,"continuous":continuous,"torque_nm":torque,"queued_commands":commands,"states_compared":LAST+1,"exact_states":exact,"max_differences":differences,
        "first_joined_tick":join_tick,"first_active_tick":first_active,"last_active_tick":last_active,"first_loaded_tick":first_loaded,"loaded_handoff_tick":handoff_tick,"last_loaded_tick":last_loaded,"first_split_after_contact":split_tick,"loaded_ticks":impacts,
        "independent_steps_verified":independent_intervals,"joined_steps_verified":joined_intervals,"peak_normal_impulse_ns":peak_normal,
        "peak_tangent_impulse_ns":peak_tangent,"net_impulse_on_hinge_ns":applied.to_array(),"initial_linear_momentum":p0.to_array(),
        "incoming_momentum_ns":momentum_scale,"initial_sphere_to_hinge_m":encounter_length,"max_total_momentum_residual_ns":max_linear,"max_total_angular_momentum_residual_nms":max_angular,"max_sphere_impulse_residual_ns":max_loose,
        "omitted_torque_delta_rad_s":omitted_delta,"events":events,"frames":frame_ticks,"state_digest":format!("{:x}",digest.finalize())});
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!(
        "{} measured states, first impact {:?} ms, first released tick {:?}, peak {:.6} N·s",
        LAST + 1,
        first_loaded,
        split_tick,
        peak_normal
    );
    Ok(())
}
