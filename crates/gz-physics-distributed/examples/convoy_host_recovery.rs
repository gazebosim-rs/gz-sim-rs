// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a deployed three-slot convoy to ordinary Bevy and a local native pool.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "two_sphere_hinge/reference.rs"]
mod reference;
#[allow(dead_code)]
#[path = "support/two_sphere_run.rs"]
mod run;

use authored_hinge::AuthoredHinge;
use clap::{Parser, ValueEnum};
use gz_physics_distributed::{
    articulation::ArticulationError,
    moving::WorkerTransport,
    multi_sphere::{MultiSphereObservation, MultiSpherePool, MultiSphereSpec},
};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";
const TRANSPORT: WorkerTransport = WorkerTransport::LoopbackZenoh;

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Fault {
    Channel,
    Worker,
    /// Suspend the deployed solver, then disconnect its SSH owner.
    SuspendedWorker,
    Host,
    HeldLease,
    /// Keep the link down through fencing, restore it, then reopen the pool.
    Partition,
    /// Recover a fenced session inside the interrupted advance when the link returns.
    AutomaticReopen,
    /// Keep the cut active until the configured automatic attempts are exhausted.
    ExhaustReopen,
    /// Briefly interrupt the link and require the original session to finish the step.
    TransientLink,
    /// Leave the physics clock paused with a healthy link, then resume in place.
    HealthyPause,
}

#[derive(Parser)]
struct Args {
    /// Coordinator-side worker launcher.
    #[arg(long)]
    worker: PathBuf,
    /// Local same-build physics_worker for the fault-free native control.
    #[arg(long)]
    control_worker: PathBuf,
    #[arg(long)]
    ledger_root: PathBuf,
    /// Worker and host faults wait for the deployment operator at the published cut.
    #[arg(long, value_enum, default_value_t = Fault::Channel)]
    fault: Fault,
    #[arg(long)]
    reverse: bool,
    /// Maximum fresh-pool attempts for one failed interval (default 0).
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=8))]
    reopen_attempts: u8,
    /// Interrupt the loaded pair on its destination instead of its source.
    #[arg(long)]
    after_migration: bool,
    /// New directory for receipts and the migrated display checkpoint.
    output: PathBuf,
}

fn await_marker(path: &Path) -> Result<()> {
    let deadline = Instant::now() + Duration::from_secs(120);
    while !path.try_exists()? {
        if Instant::now() >= deadline {
            return Err(format!("timed out waiting for {}", path.display()).into());
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(())
}

fn main() -> Result<()> {
    let a = Args::parse();
    if matches!(a.fault, Fault::AutomaticReopen | Fault::ExhaustReopen) && a.reopen_attempts == 0 {
        return Err("automatic-reopen trials require --reopen-attempts in 1..=8".into());
    }
    fs::create_dir(&a.output)?;
    let scratch = tempfile::tempdir_in(&a.ledger_root)?;
    let session = scratch.path().join("subject");
    let control = scratch.path().join("control");
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    let drive = if a.reverse { -0.01 } else { 0.01 };
    let mut ordinary = AuthoredHinge::load(SDF, "convoy.sdf")?;
    let mut expected = MultiSpherePool::create(&a.control_worker, &control, 3, spec)?;
    let mut pool = MultiSpherePool::create_with_transport(&a.worker, &session, 3, spec, TRANSPORT)?;
    pool.set_reopen_attempts(a.reopen_attempts)?;
    let public = session.join(ARCHIVE);
    let baseline = control.join(ARCHIVE);
    let mut digest = Sha256::new();
    let mut residual = 0.0_f64;
    let mut before = reference::observe(&ordinary, &spec, 0)?;
    let mut recovery = None;
    let mut migrated = None;
    let mut reopened = None;
    let fault_tick = if a.after_migration { 150 } else { 149 };
    assert_eq!(fs::read(&public)?, fs::read(&baseline)?);
    for tick in 1..=1220 {
        let mut contribution = drive;
        if tick == fault_tick + 1 {
            for p in [&mut pool, &mut expected] {
                p.queue_effort(joint, drive / 2.)?;
            }
            contribution = drive / 2.;
            let saved = fs::read(&public)?;
            assert_eq!(saved, fs::read(&baseline)?);
            let [hinge, slot, other] = pool.owners();
            assert_ne!(hinge, slot);
            assert_eq!(slot, other);
            let old = pool.worker_pid(slot).ok_or("missing pair transport")?;
            if a.fault == Fault::Channel {
                run::kill(old)?;
            } else {
                run::save(
                    &a.output.join("fault-ready.json"),
                    &json!({
                        "tick":tick-1, "slot":slot, "local_transport_pid":old,
                        "owners":pool.owners(), "pending_torque_nm":pool.pending_joint_effort(),
                        "normal_impulse_ns":pool.observation().sphere_reaction.normal_impulse,
                        "session":session,"sha256":format!("{:x}",Sha256::digest(&saved))
                    }),
                )?;
                await_marker(&a.output.join("fault-applied"))?;
            }
            let start = Instant::now();
            let attempt = pool.advance_with_effort_at(0, tick - 1, joint, contribution);
            let automatic_starts = pool.reopen_attempts_used();
            let mut fenced = None;
            if matches!(
                a.fault,
                Fault::Host | Fault::HeldLease | Fault::Partition | Fault::ExhaustReopen
            ) {
                let Err(e @ ArticulationError::MultiSphereRecovery { .. }) = attempt else {
                    return Err(format!("expected restoration exhaustion, got {attempt:?}").into());
                };
                assert_eq!(fs::read(&public)?, saved);
                assert_eq!(pool.pending_joint_effort(), Some(drive / 2.));
                assert!((0..3).all(|i| pool.worker_pid(i).is_none()));
                if a.fault == Fault::ExhaustReopen {
                    assert_eq!(automatic_starts, u64::from(a.reopen_attempts));
                    // An exhausted caller cannot quietly start a new retry budget.
                    assert!(
                        pool.advance_with_effort_at(0, tick - 1, joint, contribution)
                            .is_err()
                    );
                    assert_eq!(pool.reopen_attempts_used(), automatic_starts);
                    assert_eq!(fs::read(&public)?, saved);
                }
                fenced = Some(e.to_string());
                run::save(
                    &a.output.join("fenced.json"),
                    &json!({
                        "tick":pool.observation().hinge.tick, "error":fenced,
                        "pending_torque_nm":pool.pending_joint_effort(),
                        "automatic_starts":automatic_starts,"failure_seconds":start.elapsed().as_secs_f64()
                    }),
                )?;
                drop(pool);
                let restored = match a.fault {
                    Fault::Host => "host-restored",
                    Fault::Partition | Fault::ExhaustReopen => "link-restored",
                    _ => "lease-released",
                };
                await_marker(&a.output.join(restored))?;
                pool = MultiSpherePool::resume_with_transport(&a.worker, &session, TRANSPORT)?;
                assert_eq!(pool.pending_joint_effort(), Some(drive / 2.));
                pool.advance_with_effort_at(0, tick - 1, joint, contribution)?;
            } else {
                attempt?;
                if a.fault == Fault::AutomaticReopen {
                    assert!(
                        automatic_starts > 0 && automatic_starts <= u64::from(a.reopen_attempts)
                    );
                    assert_eq!(pool.observation().hinge.tick, tick);
                    assert_eq!(pool.pending_joint_effort(), None);
                    fs::write(a.output.join("advance-completed"), [])?;
                }
            }
            let new = pool.worker_pid(slot).ok_or("missing restored transport")?;
            if matches!(a.fault, Fault::TransientLink | Fault::HealthyPause) {
                assert_eq!(
                    new, old,
                    "the interrupted interval replaced a healthy SSH transport"
                );
            } else {
                assert_ne!(new, old);
            }
            assert!(matches!(
                pool.advance_with_effort_at(0, tick - 1, joint, contribution),
                Err(ArticulationError::StaleInterval { .. })
            ));
            recovery = Some(
                json!({"committed_tick":fault_tick,"old_transport_pid":old,"new_transport_pid":new,
                "slot":slot,"fenced":fenced,"automatic_starts":automatic_starts,"elapsed_seconds":start.elapsed().as_secs_f64(),
                "checkpoint_before_fault_sha256":format!("{:x}",Sha256::digest(&saved))}),
            );
        } else {
            if tick == fault_tick + 2 {
                pool.drop_next_acknowledgement(pool.owners()[0])?;
            }
            pool.advance_with_effort_at(0, tick - 1, joint, contribution)?;
        }
        ordinary.queue(joint, drive)?;
        ordinary.advance()?;
        expected.advance_with_effort_at(0, tick - 1, joint, contribution)?;
        let current = reference::observe(&ordinary, &spec, tick)?;
        let bytes = bincode::serialize(&current)?;
        assert_eq!(bincode::serialize(pool.observation())?, bytes);
        assert_eq!(pool.pending_joint_effort(), None);
        run::validate_owners(&current, pool.owners())?;
        let error = reference::momentum_residual(&spec, &before, &current);
        assert!(error.is_finite() && error < 1e-8);
        residual = residual.max(error);
        before = current;
        digest.update(bytes);
        if tick == 150 {
            let owners = pool.owners();
            assert!(current.sphere_reaction.normal_impulse > 0.);
            for p in [&mut pool, &mut expected] {
                p.transfer_pair(owners[0])?;
            }
            assert_eq!(pool.owners(), [owners[1], owners[0], owners[0]]);
            assert_eq!(
                bincode::serialize(pool.observation())?,
                bincode::serialize(&current)?
            );
            fs::create_dir(a.output.join("loaded"))?;
            fs::copy(&public, a.output.join("loaded").join(ARCHIVE))?;
            migrated = Some(json!({"tick":tick,"before":owners,"after":pool.owners(),
                "normal_impulse_ns":current.sphere_reaction.normal_impulse}));
        }
        if tick == fault_tick + 1 {
            // An after-migration fault occurs after the loaded tick-150 archive.
            // Publish the first verified successor for inspection in the visualizer.
            let bytes = fs::read(&public)?;
            assert_eq!(bytes, fs::read(&baseline)?);
            let recovered = a.output.join("recovered");
            fs::create_dir(&recovered)?;
            fs::write(recovered.join(ARCHIVE), bytes)?;
        }
        if tick == 180 {
            let (new, receipt) = run::reopen_with_transport(pool, &a.worker, &session, TRANSPORT)?;
            pool = new;
            reopened = Some(receipt);
        }
        assert_eq!(
            fs::read(&public)?,
            fs::read(&baseline)?,
            "archive at {tick}"
        );
    }
    run::save(
        &a.output.join("summary.json"),
        &json!({
            "verified_advances":1220, "workers":3, "torque_nm":drive, "fault":format!("{:?}",a.fault),
            "reopen_attempt_budget":a.reopen_attempts,"recovery":recovery,"migration":migrated,"reopen":reopened,
            "max_impulse_residual_ns":residual,"physical_sha256":format!("{:x}",digest.finalize())
        }),
    )
}
