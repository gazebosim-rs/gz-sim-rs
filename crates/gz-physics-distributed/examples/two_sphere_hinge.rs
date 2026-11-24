// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare independent Bevy and real local-worker trajectories during two overlapping impacts.
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
pub use run::Args;
use run::{cut, kill, pids, reopen, save, validate_owners};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOURCE: &str = "distributed_hinge_two_spheres.sdf";
const SDF: &str = include_str!("../../../examples/distributed_hinge_two_spheres.sdf");
const LAST: u64 = 1800;

#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    qualify(Args::parse())
}

pub fn qualify(args: Args) -> Result<()> {
    let spec = distributed_multi_sphere_spec(SDF, SOURCE, "default")?;
    let torque = if args.reverse { -0.01 } else { 0.01 };
    fs::create_dir(&args.output)?;
    let ledger = args
        .ledger_directory
        .clone()
        .unwrap_or_else(|| args.output.join("ledger"));
    let mut pool = MultiSpherePool::create(&args.worker, &ledger, args.workers, spec)?;
    let mut ordinary = AuthoredHinge::load(SDF, SOURCE)?;
    let mut without_command = Some(AuthoredHinge::load(SDF, SOURCE)?);
    let mut writer = if args.summary_only {
        None
    } else {
        Some(BufWriter::new(File::create(
            args.output.join("states.jsonl"),
        )?))
    };
    let mut digest = Sha256::new();
    let mut events = Vec::new();
    let mut restarts = Vec::new();
    let mut layouts = [0u64; 4]; // independent, only A with hinge, only B with hinge, both
    let mut previous_owners = pool.owners();
    if previous_owners[0] == previous_owners[1]
        || previous_owners[1] == previous_owners[2]
        || previous_owners[0] == previous_owners[2]
    {
        return Err("authored far-apart bodies must start with three independent owners".into());
    }
    let mut worker_steps_verified = 0;
    let mut initial_partial = None;
    let mut first_link = [None; 2];
    let mut last_link = [None; 2];
    let mut overlap = None;
    let mut partial = None;
    let mut separate = None;
    let mut sphere_hit = None;
    let mut omitted_delta = None;
    let mut previous = None;
    let mut max_residual = 0.0_f64;
    let mut killed = None;
    for tick in 0..=LAST {
        if tick != 0 {
            let before: Vec<_> = (0..pool.width())
                .map(|slot| pool.worker_steps(slot))
                .collect();
            pool.advance()?;
            if killed.is_none() {
                for (slot, &old) in (0..pool.width()).zip(&before) {
                    let expected = old.ok_or("missing native worker counter")?
                        + u64::from(pool.owners().contains(&slot));
                    if pool.worker_steps(slot) != Some(expected) {
                        return Err(format!(
                            "native step count differs on slot {slot} at tick {tick}"
                        )
                        .into());
                    }
                }
                worker_steps_verified += 1;
            }
            ordinary.advance()?;
            if let Some(alt) = &mut without_command {
                alt.advance()?
            }
        }
        let direct = reference::observe(&ordinary, &spec, tick)?;
        let worker = *pool.observation();
        if bincode::serialize(&direct)? != bincode::serialize(&worker)? {
            save(
                &args.output.join("first-difference.json"),
                &json!({"tick":tick,"owners":pool.owners(),"direct":direct,"worker":worker}),
            )?;
            return Err(format!("worker differs from ordinary physics at tick {tick}").into());
        }
        digest.update(bincode::serialize(&direct)?);
        if let Some(before) = previous.replace(direct) {
            max_residual = max_residual.max(reference::momentum_residual(&spec, &before, &direct));
        }
        let owners = pool.owners();
        validate_owners(&direct, owners)?;
        if pool.pending_joint_effort().is_some() {
            return Err("an advancing step retained the command".into());
        }
        let mask = usize::from(owners[0] == owners[1]) + 2 * usize::from(owners[0] == owners[2]);
        if initial_partial.is_none() && overlap.is_none() && mask == 1 {
            initial_partial = Some(tick);
        }
        if tick > 0 {
            layouts[mask] += 1
        }
        let mut event = None;
        if owners != previous_owners {
            events.push(json!({"tick":tick,"kind":"placement","from":previous_owners,"to":owners}));
            event = Some("placement");
        }
        if let Some((slot, before)) = killed.take() {
            let after = pids(&pool)?;
            let before: Vec<u32> = before;
            if before
                .iter()
                .zip(&after)
                .enumerate()
                .any(|(i, (a, b))| (a != b) != (i == slot))
            {
                return Err("wrong worker replaced after process kill".into());
            }
            events.push(
                json!({"tick":tick,"kind":"replaced","slot":slot,"before":before,"after":after}),
            );
        }
        if overlap.is_some_and(|t| tick == t + 1) {
            let alternate = without_command
                .take()
                .ok_or("missing omitted-command control")?;
            let delta =
                (reference::observe(&alternate, &spec, tick)?.hinge.qdot - direct.hinge.qdot).abs();
            if delta < 1e-8 {
                return Err("omitting the migrated command did not change the joint rate".into());
            }
            omitted_delta = Some(delta);
        }
        let loads = direct
            .reactions
            .map(|edges| edges.iter().map(|r| r.normal_impulse).sum::<f64>());
        let cached = direct.reactions.map(|edges| {
            edges
                .iter()
                .map(|r| r.warmstart_normal_impulse)
                .sum::<f64>()
        });
        for i in 0..2 {
            if loads[i] > 0. && first_link[i].is_none() {
                first_link[i] = Some(tick)
            }
            if direct.reactions[i].iter().any(|r| r.active) {
                last_link[i] = Some(tick)
            }
        }
        let double = overlap.is_none() && loads.iter().all(|v| *v > 0.);
        if tick < LAST {
            if tick == 3 {
                pool.drop_next_acknowledgement(owners[0])?;
            }
            pool.queue_effort(spec.hinge.joint_id, torque)?;
            ordinary.queue(spec.hinge.joint_id, torque)?;
            if !double && let Some(alt) = &mut without_command {
                alt.queue(spec.hinge.joint_id, torque)?
            }
            if pool.pending_joint_effort() != Some(torque) {
                return Err("pending drive did not commit once".into());
            }
        }
        if double {
            if mask != 3 {
                return Err("independent worlds at double impact".into());
            }
            let before = pids(&pool)?;
            let image = pool.image().ok_or("missing loaded native image")?.to_vec();
            let destination = (owners[0] + 1) % args.workers;
            pool.transfer(destination)?;
            if pool.image() != Some(image.as_slice())
                || pool.owners() != [destination; 3]
                || pids(&pool)? != before
                || bincode::serialize(pool.observation())? != bincode::serialize(&direct)?
                || pool.pending_joint_effort() != Some(torque)
            {
                return Err("loaded handoff changed the graph, state or torque".into());
            }
            cut(&args, &ledger, tick)?;
            let restored;
            (pool, restored) = reopen(pool, &args.worker, &ledger)?;
            restarts.push(json!({"tick":tick,"kind":"loaded","receipt":restored}));
            events.push(json!({"tick":tick,"kind":"loaded_handoff","from":owners,"to":pool.owners(),"normal_impulses_ns":loads,
                "warmstart_impulses_ns":cached,"image_bytes":image.len(),"image_sha256":format!("{:x}",Sha256::digest(&image))}));
            overlap = Some(tick);
            event = Some("loaded_handoff");
        } else if overlap.is_some() && partial.is_none() && matches!(mask, 1 | 2) {
            partial = Some(tick);
            cut(&args, &ledger, tick)?;
            let restored;
            (pool, restored) = reopen(pool, &args.worker, &ledger)?;
            restarts.push(json!({"tick":tick,"kind":"partial_release","receipt":restored}));
            event = Some("partial_release");
        } else if partial.is_some() && separate.is_none() && mask == 0 && owners[1] != owners[2] {
            separate = Some(tick);
            cut(&args, &ledger, tick)?;
            event = Some("separated");
        }
        if sphere_hit.is_none() && direct.sphere_reaction.normal_impulse > 0. {
            if separate.is_none()
                || owners[1] != owners[2]
                || owners[0] == owners[1]
                || pool.pair_image().is_none()
                || direct.reactions.iter().flatten().any(|r| r.active)
            {
                return Err("expected a later sphere-only encounter".into());
            }
            if direct.sphere_reaction.tangent_impulse == [0.; 3] {
                return Err("missing expected glancing sphere–sphere friction impulse".into());
            }
            sphere_hit = Some(tick);
            cut(&args, &ledger, tick)?;
            let restored;
            (pool, restored) = reopen(pool, &args.worker, &ledger)?;
            restarts.push(json!({"tick":tick,"kind":"sphere_collision","receipt":restored}));
            events.push(json!({"tick":tick,"kind":"sphere_collision","owners":owners,"reaction":direct.sphere_reaction}));
            event = Some("sphere_collision");
        }
        if tick == 4 || double {
            let before = pids(&pool)?;
            let slot = if double { pool.owners()[0] } else { owners[2] };
            kill(before[usize::from(slot)])?;
            killed = Some((usize::from(slot), before.clone()));
            events.push(json!({"tick":tick,"kind":"worker_kill","slot":slot,"pid":before[usize::from(slot)]}));
        }
        if let Some(w) = &mut writer {
            serde_json::to_writer(
                &mut *w,
                &json!({"tick":tick,"owners":pool.owners(),"direct":direct,"worker":worker,
                "event":event,"queued_effort_nm":if tick<LAST{torque}else{0.}}),
            )?;
            w.write_all(b"\n")?;
        }
        previous_owners = pool.owners();
    }
    if let Some(w) = &mut writer {
        w.flush()?
    }
    let summary = json!({"workers":args.workers,"torque_nm":torque,"states_compared":LAST+1,"queued_commands":LAST,
        "physical_digest":format!("{:x}",digest.finalize()),"first_hinge_impacts":first_link,"last_active_hinge_contacts":last_link,
        "first_hinge_with_sphere_a_tick":initial_partial,"worker_step_counts_verified":worker_steps_verified,"double_impact_tick":overlap,"partial_release_tick":partial,"independent_again_tick":separate,
        "sphere_impact_tick":sphere_hit,"omitted_command_delta_rad_s":omitted_delta,
        "max_sphere_impulse_residual_ns":max_residual,"intervals_by_hinge_membership":layouts,"events":events,"restarts":restarts});
    save(&args.output.join("summary.json"), &summary)?;
    if initial_partial.is_none()
        || overlap.is_none()
        || partial.is_none()
        || separate.is_none()
        || sphere_hit.is_none()
        || layouts[0] == 0
        || layouts[1] == 0
        || layouts[3] == 0
        || omitted_delta.is_none()
        || max_residual > 1e-7
    {
        return Err("trajectory failed the staged impact, independence or impulse-balance checks; see summary.json".into());
    }
    println!(
        "{} ordinary and worker states matched; shared impact {} ms, partial release {} ms, separate {} ms, sphere impact {} ms",
        LAST + 1,
        overlap.unwrap(),
        partial.unwrap(),
        separate.unwrap(),
        sphere_hit.unwrap()
    );
    Ok(())
}
