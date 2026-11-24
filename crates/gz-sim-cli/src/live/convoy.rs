// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::PhysicsBackend;
use gz_sim_scene::distributed_hinge::{
    ColdSelection, DistributedMultiSphereBackend, FenceReceipt, HingeRecipe, HingeWorkerOptions,
    RecoveryMonitor, RecoveryPhase, distributed_hinge_recipe, inspect_published_path,
    load_distributed_hinge_world_with_options, preview_selected_hinge_world,
    resume_fenced_hinge_world, resume_selected_hinge_world,
};
use std::path::Path;

#[derive(Clone, Copy)]
pub struct ConvoyDeployment<'a> {
    pub xml: &'a str,
    pub source: &'a str,
    pub world: &'a str,
    pub worker: &'a Path,
    pub session: &'a Path,
    pub options: HingeWorkerOptions,
}

enum Authority {
    WorkerFailure(FenceReceipt),
    Cold(Box<ColdSelection>),
}

struct Fence {
    scene: String,
    epoch: u64,
    tick: u64,
    authority: Option<Authority>,
    reason: String,
}

fn monitor(sim: &mut Simulation) -> io::Result<RecoveryMonitor> {
    sim.world_mut()
        .get_resource_mut::<PhysicsBackend>()
        .and_then(|mut backend| {
            backend
                .downcast_mut::<DistributedMultiSphereBackend>()
                .map(|backend| backend.subscribe_recovery())
        })
        .ok_or_else(|| io::Error::other("operator recovery requires a two-sphere convoy"))
}

type InstallResult = Result<(), Box<dyn Error>>;

enum Session {
    Running {
        simulation: Box<Simulation>,
        handoff: Option<String>,
    },
    Parked(Fence),
}

/// Park a terminal worker fence on the live viewer port. Each verified install
/// starts paused at the authorized cut with a fresh Bevy world and worker pool.
pub fn run_recoverable_convoy(
    deployment: ConvoyDeployment<'_>,
    port: u16,
    install_systems: impl Fn(&mut Simulation) -> InstallResult,
) -> InstallResult {
    let d = deployment;
    if d.world != "default" {
        return Err("gz-web-rs primitive operator recovery requires the default world".into());
    }
    if !matches!(
        distributed_hinge_recipe(d.xml, d.source, d.world)?,
        HingeRecipe::MultiSphere(_)
    ) {
        return Err("--operator-recovery requires a two-sphere convoy".into());
    }
    let termination = Termination::install()?;
    let mut simulation = load_distributed_hinge_world_with_options(
        d.xml, d.source, d.world, d.worker, d.session, d.options,
    )?;
    install_systems(&mut simulation)?;
    supervise(
        d,
        port,
        &install_systems,
        termination,
        Session::Running {
            simulation: Box::new(simulation),
            handoff: None,
        },
    )
}

/// Start a replacement supervisor with a reviewed package. The caller supplies
/// its digest independently. Preview starts without workers. The first operator
/// click verifies ownership and the current checkpoint again before installing.
pub fn run_selected_convoy(
    package_path: &Path,
    expected_package_sha256: &str,
    worker: &Path,
    session: &Path,
    options: HingeWorkerOptions,
    port: u16,
    install_drive: impl Fn(&mut Simulation, &ConvoyCapsule) -> InstallResult,
) -> InstallResult {
    let package = ConvoyCapsule::open(package_path, expected_package_sha256)?;
    if package.world() != "default" {
        return Err("gz-web-rs primitive operator recovery requires the default world".into());
    }
    let cut = package.cut();
    if options.width.is_some_and(|w| w != cut.cut.width) {
        return Err("worker count conflicts with the selected checkpoint".into());
    }
    if options.reopen_attempts
        > gz_sim_scene::distributed_hinge::MultiSpherePool::MAX_REOPEN_ATTEMPTS
    {
        return Err("invalid worker reopen budget".into());
    }
    let deployment = ConvoyDeployment {
        xml: package.sdf(),
        source: "supervisor-checkpoint.sdf",
        world: package.world(),
        worker,
        session,
        options: HingeWorkerOptions {
            width: Some(cut.cut.width),
            ..options
        },
    };
    let termination = Termination::install()?;
    let preview =
        preview_selected_hinge_world(deployment.xml, deployment.source, deployment.world, cut)?;
    let scene = crate::web::ConvoyView::new(&preview, deployment.world)?.encoded()?;
    drop(preview);
    let (authority, reason) = match ColdSelection::inspect(session, &cut.sha256, package.origin()) {
        Ok(selected) => (
            Some(Authority::Cold(Box::new(selected))),
            "Replacement supervisor is ready. Verify and recover the selected checkpoint."
                .to_owned(),
        ),
        Err(error) => {
            eprintln!("Supervisor checkpoint selection refused: {error}");
            (
                None,
                format!("Selected checkpoint unavailable. Inspection required. {error}")
                    .chars()
                    .take(220)
                    .collect(),
            )
        }
    };
    let state = Session::Parked(Fence {
        scene,
        epoch: cut.cut.epoch,
        tick: cut.cut.observation.hinge.tick,
        authority,
        reason,
    });
    supervise(
        deployment,
        port,
        &|sim| install_drive(sim, &package),
        termination,
        state,
    )
}

fn supervise(
    d: ConvoyDeployment<'_>,
    port: u16,
    install: &impl Fn(&mut Simulation) -> InstallResult,
    termination: Termination,
    mut state: Session,
) -> InstallResult {
    while !termination.requested() {
        state = match state {
            Session::Running {
                mut simulation,
                handoff,
            } => {
                let progress = monitor(&mut simulation)?;
                let mut view = crate::web::ConvoyView::new(&simulation, d.world)?;
                let run_options = RunOptions {
                    restored: handoff,
                    running: false,
                    iterations: 0,
                    camera_worker: None,
                    checkpoint: None,
                    viewer: ViewerOptions {
                        port: Some(port),
                        lifecycle_sdf: None,
                        recording_path: None,
                    },
                };
                let Err(error) = run_session(
                    *simulation,
                    d.world,
                    &termination,
                    run_options,
                    Some(&mut view),
                ) else {
                    return Ok(());
                };
                let Some(cut) = progress
                    .snapshot()
                    .filter(|s| s.phase == RecoveryPhase::Fenced)
                else {
                    return Err(error);
                };
                let receipt = progress.fence_receipt();
                let reason = if receipt.is_some() {
                    "Worker transport failed. The committed checkpoint is verified."
                } else {
                    "Worker session fenced. Inspect the checkpoint before continuing."
                };
                eprintln!(
                    "Worker fence at epoch {} tick {}: {error}",
                    cut.epoch, cut.tick
                );
                Session::Parked(Fence {
                    scene: view.encoded()?,
                    epoch: cut.epoch,
                    tick: cut.tick,
                    authority: receipt.map(Authority::WorkerFailure),
                    reason: reason.into(),
                })
            }
            Session::Parked(mut fence) => {
                let mut parked = crate::ParkedFence::bind(
                    port,
                    &fence.scene,
                    fence.epoch,
                    fence.tick,
                    fence.authority.is_some(),
                    &fence.reason,
                )?;
                if !parked.wait(|| termination.requested())? || termination.requested() {
                    return Ok(());
                }
                let handoff = parked.admitted_handoff()?;
                drop(parked);
                let authority = fence
                    .authority
                    .as_ref()
                    .ok_or_else(|| io::Error::other("ineligible fence"))?;
                let (restore, digest) = match authority {
                    Authority::WorkerFailure(receipt) => (
                        resume_fenced_hinge_world(
                            d.xml, d.source, d.world, d.worker, d.session, d.options, receipt,
                        ),
                        receipt.sha256(),
                    ),
                    Authority::Cold(selected) => (
                        resume_selected_hinge_world(
                            d.xml, d.source, d.world, d.worker, d.session, d.options, selected,
                        ),
                        selected.cut().sha256.as_str(),
                    ),
                };
                match restore {
                    Ok(mut simulation) => {
                        let installed = install(&mut simulation).and_then(|()| {
                            if simulation.epoch() == fence.epoch
                                && simulation.elapsed() == Duration::from_millis(fence.tick)
                                && simulation.is_paused()
                            {
                                Ok(())
                            } else {
                                Err(io::Error::other(
                                    "restored world has a different clock or run mode",
                                )
                                .into())
                            }
                        });
                        match installed {
                            Ok(()) => Session::Running {
                                simulation: Box::new(simulation),
                                handoff: Some(handoff),
                            },
                            Err(error) => {
                                eprintln!("Guarded scene installation failed: {error}");
                                fence.authority = None;
                                fence.reason =
                                    "Scene installation failed. Operator inspection required."
                                        .into();
                                Session::Parked(fence)
                            }
                        }
                    }
                    Err(error) => {
                        eprintln!("Guarded worker restore failed: {error}");
                        // Only a diagnosis. Every actual attempt verifies again under the lease.
                        let same_archive =
                            inspect_published_path(&d.session.join("barrier.gzmultisphere"))
                                .is_ok_and(|cut| cut.sha256 == digest);
                        if !same_archive || !error.retryable() {
                            fence.authority = None;
                            fence.reason = if same_archive {
                                "Worker installation rejected the checkpoint. Operator inspection required."
                            } else { "Checkpoint changed or cannot be verified. Operator inspection required." }.into();
                        } else {
                            fence.reason = "Restore failed. Check worker availability and session ownership before retrying.".into();
                        }
                        Session::Parked(fence)
                    }
                }
            }
        };
    }
    Ok(())
}
