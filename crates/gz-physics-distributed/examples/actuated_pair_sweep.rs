// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replayable geometry and actuation sweep against an uninterrupted ordinary Bevy world.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "two_sphere_hinge/reference.rs"]
mod reference;
#[allow(dead_code)]
#[path = "support/two_sphere_run.rs"]
mod run;

use authored_hinge::AuthoredHinge;
use clap::Parser;
use gz_physics_distributed::multi_sphere::{
    MultiSphereObservation, MultiSpherePool, MultiSphereSpec,
};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[path = "support/actuated_convoy.rs"]
mod convoy;
use convoy::{draw, geometry};
const ARCHIVE: &str = "barrier.gzmultisphere";

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Use loopback Zenoh RPC for each resident, including replacements.
    #[arg(long)]
    zenoh: bool,
    /// Alternate transports on whole-pool recovery and compare the complete native receipt.
    #[arg(long)]
    alternate_transport: bool,
    /// Existing filesystem directory for a private, temporary, durable worker ledger.
    #[arg(long)]
    ledger_root: PathBuf,
    #[arg(long)]
    seed: u64,
    #[arg(long, default_value_t = 2048, value_parser=clap::value_parser!(u64).range(1..))]
    ticks: u64,
    #[arg(long, default_value_t = 4, value_parser=clap::value_parser!(u8).range(3..=8))]
    workers: u8,
    /// New directory. On failure, keep the input geometry, last cut and first differing sample.
    output: PathBuf,
}

fn exercise(args: &Args, xml: &str, ledger: &Path) -> Result<Value> {
    let spec = distributed_multi_sphere_spec(xml, "convoy.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    use gz_physics_distributed::moving::WorkerTransport::{LoopbackZenoh, Pipe};
    let mut transport = if args.zenoh { LoopbackZenoh } else { Pipe };
    let mut pool = MultiSpherePool::create_with_transport(
        &args.worker,
        ledger,
        args.workers,
        spec,
        transport,
    )?;
    let mut ordinary = AuthoredHinge::load(xml, "convoy.sdf")?;
    let mut prior = reference::observe(&ordinary, &spec, 0)?;
    let mut digest = Sha256::new();
    let mut residual: f64 = 0.;
    let mut max_orientation_error: f64 = 0.;
    let (mut migrations, mut reopens, mut replacements, mut queues) = (0, 0, 0, 0);
    let mut visited = std::collections::BTreeMap::new();
    let (mut pair_loads, mut hinge_loads) = (0, 0);
    let mut moved_loaded = false;
    for tick in 0..=args.ticks {
        if tick > 0 {
            // Each group of 32 ms has the same signed input. Binary fractions
            // preserve exact accumulation when a durable contribution already exists.
            let drive = convoy::drive(args.seed, tick);
            if let Some(extra) = convoy::extra(args.seed, tick) {
                pool.queue_effort(joint, extra)?;
                ordinary.queue(joint, extra)?;
                queues += 1;
            }
            if tick % 251 == (draw(args.seed, 9) % 251) {
                let slot = pool.owners()[(tick / 251 % 3) as usize];
                run::kill(pool.worker_pid(slot).ok_or("missing resident worker")?)?;
                replacements += 1;
            }
            pool.advance_with_effort(joint, drive)?;
            ordinary.queue(joint, drive)?;
            ordinary.advance()?;
        }
        let expected = reference::observe(&ordinary, &spec, tick)?;
        let actual = pool.observation();
        if bincode::serialize(actual)? != bincode::serialize(&expected)?
            || pool.pending_joint_effort().is_some()
        {
            run::save(
                &args.output.join("difference.json"),
                &json!({"tick":tick,"owners":pool.owners(),"expected":expected,"actual":actual,"pending":pool.pending_joint_effort()}),
            )?;
            return Err(
                format!("ordinary and distributed observations diverged at tick {tick}").into(),
            );
        }
        run::validate_owners(actual, pool.owners())?;
        let error = reference::momentum_residual(&spec, &prior, actual);
        if !error.is_finite() || error > 1e-8 {
            return Err(format!("sphere impulse balance {error:e} at tick {tick}").into());
        }
        residual = residual.max(error);
        for link in actual.hinge.links.iter().chain(&actual.spheres) {
            let norm_error =
                (glam::DQuat::from_array(link.orientation).length_squared() - 1.).abs();
            if !norm_error.is_finite() || norm_error > 1e-12 {
                return Err(format!("orientation norm error {norm_error:e} at tick {tick}").into());
            }
            max_orientation_error = max_orientation_error.max(norm_error);
        }
        prior = *actual;
        digest.update(bincode::serialize(actual)?);
        let [h, a, b] = pool.owners();
        let layout = if h == a && h == b {
            "together"
        } else if a == b {
            "sphere_pair"
        } else if h == a || h == b {
            "hinge_and_sphere"
        } else {
            "separate"
        };
        *visited.entry(layout).or_insert(0_u64) += 1;
        let loaded = actual.sphere_reaction.normal_impulse > 0.;
        pair_loads += u64::from(loaded);
        hinge_loads += u64::from(
            actual
                .reactions
                .iter()
                .flatten()
                .any(|r| r.normal_impulse > 0.),
        );
        if h != a && a == b && loaded && !moved_loaded {
            pool.transfer_pair(h)?;
            moved_loaded = true;
            migrations += 1;
        }
        if tick > 0 && tick % 389 == draw(args.seed, 10) % 389 {
            if args.alternate_transport {
                transport = if transport == Pipe {
                    LoopbackZenoh
                } else {
                    Pipe
                };
            }
            let (restored, _) = run::reopen_with_transport(pool, &args.worker, ledger, transport)?;
            pool = restored;
            reopens += 1;
        }
    }
    Ok(
        json!({"seed":args.seed,"initial_transport":if args.zenoh {"loopback_zenoh"} else {"pipe"},"alternating_transport":args.alternate_transport,"workers":args.workers,"states":args.ticks+1,"ticks":args.ticks,
        "layout_observations":visited,"pair_loads":pair_loads,"hinge_loads":hinge_loads,
        "loaded_transfers":migrations,"whole_pool_reopens":reopens,"worker_kills":replacements,"independently_queued":queues,
        "max_quaternion_squared_norm_error":max_orientation_error,"max_sphere_impulse_residual_ns":residual,"physical_digest":format!("{:x}",digest.finalize())}),
    )
}

fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.output)?;
    let ledger = tempfile::tempdir_in(&args.ledger_root)?;
    let start = Instant::now();
    let xml = geometry(args.seed);
    match exercise(&args, &xml, ledger.path()) {
        Ok(mut results) => {
            results["wall_seconds"] = start.elapsed().as_secs_f64().into();
            run::save(&args.output.join("summary.json"), &results)?;
            println!("{}", serde_json::to_string(&results)?);
            Ok(())
        }
        Err(e) => {
            fs::write(args.output.join("convoy.sdf"), xml)?;
            if ledger.path().join(ARCHIVE).exists() {
                fs::copy(ledger.path().join(ARCHIVE), args.output.join(ARCHIVE))?;
            }
            run::save(
                &args.output.join("failure.json"),
                &json!({"seed":args.seed,"initial_transport":if args.zenoh {"loopback_zenoh"} else {"pipe"},"alternating_transport":args.alternate_transport,"workers":args.workers,"requested_ticks":args.ticks,"error":e.to_string()}),
            )?;
            Err(e)
        }
    }
}
