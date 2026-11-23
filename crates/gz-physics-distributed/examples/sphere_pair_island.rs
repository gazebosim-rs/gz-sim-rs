// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a separately owned sphere pair that reaches a continuously driven floating hinge.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "two_sphere_hinge/reference.rs"]
mod reference;
#[path = "support/two_sphere_run.rs"]
mod run;

use authored_hinge::AuthoredHinge;
use clap::Parser;
use gz_physics_distributed::multi_sphere::{
    MultiSphereObservation, MultiSpherePool, MultiSphereSpec,
};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;

use run::{cut, kill, pids, reopen, save, validate_owners};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOURCE: &str = "distributed_sphere_pair_island.sdf";
const XML: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
#[derive(Parser)]
pub struct Qualification {
    #[command(flatten)]
    run: run::Args,
    /// Simulated milliseconds. Longer runs also capture the subsequent release of the hinge and the pair.
    #[arg(long, default_value_t=320, value_parser=clap::value_parser!(u64).range(320..))]
    ticks: u64,
}

fn pair_only(owners: [u8; 3]) -> bool {
    owners[0] != owners[1] && owners[1] == owners[2]
}

fn swap_pair(pool: &mut MultiSpherePool, torque: f64) -> Result<serde_json::Value> {
    let owners = pool.owners();
    let worker_pids = pids(pool)?;
    let pair_image = pool
        .pair_image()
        .ok_or("missing standalone sphere-pair image")?
        .to_vec();
    let hinge_image = pool.image().ok_or("missing hinge image")?.to_vec();
    let state = bincode::serialize(pool.observation())?;
    // First place the loaded pair on an idle process, then on the hinge's occupied
    // slot. The second operation must move the independently driven hinge as well.
    let spare = (0..pool.width())
        .find(|s| !owners.contains(s))
        .ok_or("no spare worker")?;
    let mut hops = Vec::new();
    for destination in [spare, owners[0]] {
        let old = pool.owners();
        pool.transfer_pair(destination)?;
        let hinge_dest = if old[0] == destination {
            old[1]
        } else {
            old[0]
        };
        if pool.owners() != [hinge_dest, destination, destination]
            || pids(pool)? != worker_pids
            || pool.pair_image() != Some(&pair_image)
            || pool.image() != Some(&hinge_image)
            || bincode::serialize(pool.observation())? != state
            || pool.pending_joint_effort() != Some(torque)
        {
            return Err(
                "pair migration changed a native image, observation, input or process".into(),
            );
        }
        hops.push(json!({"from":old,"to":pool.owners()}));
    }
    Ok(
        json!({"hops":hops,"pair_image_bytes":pair_image.len(),"pair_sha256":format!("{:x}",Sha256::digest(&pair_image))}),
    )
}

#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    qualify(Qualification::parse())
}

pub fn qualify(options: Qualification) -> Result<()> {
    let args = options.run;
    let last = options.ticks;
    let spec = distributed_multi_sphere_spec(XML, SOURCE, "default")?;
    fs::create_dir(&args.output)?;
    let ledger = args
        .ledger_directory
        .clone()
        .unwrap_or_else(|| args.output.join("ledger"));
    let mut pool = MultiSpherePool::create(&args.worker, &ledger, args.workers, spec)?;
    let mut ordinary = AuthoredHinge::load(XML, SOURCE)?;
    let mut omission = Some(AuthoredHinge::load(XML, SOURCE)?);
    let torque = if args.reverse { -0.01 } else { 0.01 };
    let mut writer = if args.summary_only {
        None
    } else {
        Some(BufWriter::new(File::create(
            args.output.join("states.jsonl"),
        )?))
    };
    let mut physical_digest = Sha256::new();
    let mut separated_pair_ticks = 0;
    let mut events = Vec::new();
    let mut restarts = Vec::new();
    let mut last_owners = pool.owners();
    if last_owners != [0, 1, 2] {
        return Err("expected three independent initial owners".into());
    }
    let mut before: Option<MultiSphereObservation> = None;
    let mut killed: Option<(u8, Vec<u32>)> = None;
    let mut counter_ticks = 0;
    let mut worst_residual = 0.0_f64;
    let mut pair_formed = None;
    let mut isolated_impact = None;
    let mut warm_merge = None;
    let mut hinge_impact = None;
    let mut omitted_delta = None;
    let mut pair_released = None;
    let mut all_separate = None;

    for tick in 0..=last {
        let mut event = None;
        if tick > 0 {
            let steps: Vec<_> = (0..pool.width())
                .map(|slot| pool.worker_steps(slot).unwrap())
                .collect();
            pool.advance()?;
            ordinary.advance()?;
            if let Some(control) = &mut omission {
                control.advance()?;
            }
            if let Some((slot, previous)) = killed.take() {
                let current = pids(&pool)?;
                if current
                    .iter()
                    .zip(&previous)
                    .enumerate()
                    .any(|(i, (a, b))| (a != b) != (i == usize::from(slot)))
                {
                    return Err("wrong process replacement after SIGKILL".into());
                }
                events.push(json!({"tick":tick,"kind":"worker_replaced","slot":slot,"before":previous,"after":current}));
            } else {
                for (slot, old) in (0..pool.width()).zip(steps) {
                    if pool.worker_steps(slot)
                        != Some(old + u64::from(pool.owners().contains(&slot)))
                    {
                        return Err(format!(
                            "incorrect advancing owner on tick {tick}, slot {slot}"
                        )
                        .into());
                    }
                }
                counter_ticks += 1;
            }
        }
        let direct = reference::observe(&ordinary, &spec, tick)?;
        let worker = *pool.observation();
        let state = bincode::serialize(&direct)?;
        let owners = pool.owners();
        if bincode::serialize(&worker)? != state {
            save(
                &args.output.join("first-difference.json"),
                &json!({"tick":tick,"owners":owners,"direct":direct,"worker":worker}),
            )?;
            return Err(
                format!("ordinary Bevy and distributed states differ at tick {tick}").into(),
            );
        }
        if pool.pending_joint_effort().is_some() {
            return Err("command survived advance".into());
        }
        validate_owners(&direct, owners)?;
        physical_digest.update(state);
        if owners != last_owners {
            events.push(json!({"tick":tick,"kind":"placement","from":last_owners,"to":owners}));
            event = Some("placement");
        }
        if let Some(previous) = before {
            worst_residual =
                worst_residual.max(reference::momentum_residual(&spec, &previous, &direct));
            if pair_only(last_owners) && owners == [owners[0]; 3] && warm_merge.is_none() {
                if !previous.sphere_reaction.active
                    || previous.sphere_reaction.warmstart_normal_impulse <= 0.0
                    || isolated_impact.is_none()
                {
                    return Err(format!(
                        "merge at {tick} lacked a previously loaded warm sphere pair: {:?}",
                        previous.sphere_reaction
                    )
                    .into());
                }
                warm_merge = Some(tick);
                events.push(json!({"tick":tick,"kind":"warm_merge","from":last_owners,"to":owners,"prior_reaction":previous.sphere_reaction,"after_reaction":direct.sphere_reaction}));
                event = Some("warm_merge");
            }
        }
        before = Some(direct);
        if pair_only(owners) {
            separated_pair_ticks += 1;
            pair_formed.get_or_insert(tick);
        }
        let pair = direct.sphere_reaction;
        let hinge_loads = direct
            .reactions
            .map(|links| links.iter().map(|r| r.normal_impulse).sum::<f64>());
        let first_pair_load = isolated_impact.is_none() && pair.normal_impulse > 0.;
        let first_hinge_load = hinge_impact.is_none() && hinge_loads.into_iter().any(|v| v > 0.);
        if tick < last {
            if tick == 3 {
                pool.drop_next_acknowledgement(owners[0])?;
            }
            pool.queue_effort(spec.hinge.joint_id, torque)?;
            ordinary.queue(spec.hinge.joint_id, torque)?;
            if !first_pair_load && let Some(control) = &mut omission {
                control.queue(spec.hinge.joint_id, torque)?;
            }
            if pool.pending_joint_effort() != Some(torque) {
                return Err("torque commit differs".into());
            }
        }
        if isolated_impact.is_some_and(|t| t + 1 == tick) {
            let alt = omission.take().ok_or("missing omitted torque control")?;
            let delta =
                (direct.hinge.qdot - reference::observe(&alt, &spec, tick)?.hinge.qdot).abs();
            if delta < 1e-8 {
                return Err("omitted torque does not distinguish the loaded handoff".into());
            }
            omitted_delta = Some(delta);
        }
        if first_pair_load {
            if !pair_only(owners)
                || pair.warmstart_normal_impulse <= 0.
                || pair.tangent_impulse == [0.; 3]
                || hinge_loads != [0.; 2]
            {
                return Err(format!("expected isolated oblique loaded sphere pair on tick {tick}, owners {owners:?}, reaction {pair:?}").into());
            }
            let migration = swap_pair(&mut pool, torque)?;
            isolated_impact = Some(tick);
            events.push(json!({"tick":tick,"kind":"pair_handoff","owners":pool.owners(),"reaction":pair,"receipt":migration}));
            event = Some("pair_handoff");
        }
        if first_hinge_load {
            if owners != [owners[0]; 3] || warm_merge.is_none() || pair.normal_impulse <= 0. {
                return Err(format!(
                    "expected simultaneous pair and hinge impulses after merge on tick {tick}"
                )
                .into());
            }
            hinge_impact = Some(tick);
            events.push(json!({"tick":tick,"kind":"coupled_impact","hinge_normal_impulses_ns":hinge_loads,"sphere_reaction":pair}));
            event = Some("coupled_impact");
        }
        if hinge_impact.is_some()
            && pair_released.is_none()
            && pair_only(owners)
            && last_owners == [last_owners[0]; 3]
        {
            pair_released = Some(tick);
            event = Some("hinge_released");
            events.push(json!({"tick":tick,"kind":event,"owners":owners,"reaction":pair}));
        }
        if pair_released.is_some()
            && all_separate.is_none()
            && pair_only(last_owners)
            && owners[0] != owners[1]
            && owners[0] != owners[2]
            && owners[1] != owners[2]
        {
            all_separate = Some(tick);
            event = Some("pair_separated");
            events.push(json!({"tick":tick,"kind":event,"owners":owners,"reaction":pair}));
        }
        if first_pair_load
            || warm_merge == Some(tick)
            || first_hinge_load
            || pair_released == Some(tick)
            || all_separate == Some(tick)
        {
            cut(&args, &ledger, tick)?;
            let receipt;
            (pool, receipt) = reopen(pool, &args.worker, &ledger)?;
            restarts.push(json!({"tick":tick,"kind":event,"receipt":receipt}));
        }
        if first_pair_load || first_hinge_load {
            let slot = pool.owners()[1];
            let previous = pids(&pool)?;
            kill(previous[usize::from(slot)])?;
            events.push(json!({"kind":"worker_kill","tick":tick,"slot":slot,"pid":previous[usize::from(slot)]}));
            killed = Some((slot, previous));
        }
        if let Some(w) = &mut writer {
            serde_json::to_writer(
                &mut *w,
                &json!({"tick":tick,"owners":pool.owners(),"event":event,"worker":worker,"direct":direct,"queued_effort_nm":if tick<last {torque} else {0.}}),
            )?;
            w.write_all(b"\n")?;
        }
        last_owners = pool.owners();
    }
    if let Some(w) = &mut writer {
        w.flush()?;
    }
    if !matches!((pair_formed,isolated_impact,warm_merge,hinge_impact),(Some(a),Some(b),Some(c),Some(d)) if a<b && b<c && c<d)
        || omitted_delta.is_none()
        || worst_residual > 1e-7
        || restarts.len()
            != 3 + usize::from(pair_released.is_some()) + usize::from(all_separate.is_some())
        || counter_ticks != last - 2
    {
        return Err("missing physical event or recovery in qualification".into());
    }
    let summary = json!({"workers":args.workers,"torque_nm":torque,"states_compared":last+1,"queued_commands":last,
        "pair_released_tick":pair_released,"all_separate_tick":all_separate,"physical_digest":format!("{:x}",physical_digest.finalize()),"pair_formed_tick":pair_formed,"pair_impact_tick":isolated_impact,
        "warm_merge_tick":warm_merge,"hinge_impact_tick":hinge_impact,"independent_pair_intervals":separated_pair_ticks,
        "max_sphere_impulse_residual_ns":worst_residual,"worker_step_counts_verified":counter_ticks,"omitted_command_delta_rad_s":omitted_delta,
        "events":events,"restarts":restarts});
    save(&args.output.join("summary.json"), &summary)?;
    println!(
        "{} exact observations, pair impact at {} ms, warm merge at {} ms, simultaneous pair and hinge loads at {} ms",
        last + 1,
        isolated_impact.unwrap(),
        warm_merge.unwrap(),
        hinge_impact.unwrap()
    );
    Ok(())
}
