// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Serve a continuously actuated moving hinge through the ordinary gz-web-rs live server.
use clap::Parser;
use gz_sim_cli::live::{self, ViewerOptions};
use gz_sim_scene::distributed_hinge::{
    HingeWorkerOptions, MultiSpherePool, WorkerTransport, load_distributed_hinge_world_with_options,
};
use std::{error::Error, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// Authored floating, grounded, one-sphere or two-sphere hinge scene.
    #[arg(long)]
    sdf: Option<PathBuf>,
    /// Stock physics_worker, or convoy_ssh_worker for a deployed two-sphere convoy.
    #[arg(long)]
    worker: PathBuf,
    /// Create a worker session or reopen an existing measured cut.
    #[arg(long)]
    session: PathBuf,
    /// Use Zenoh for worker requests and receipts. Required with convoy_ssh_worker.
    #[arg(long)]
    zenoh: bool,
    /// Additional reopen attempts per interrupted step for the two-sphere profile.
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(..=i64::from(MultiSpherePool::MAX_REOPEN_ATTEMPTS)))]
    reopen_attempts: u8,
    /// Keep a terminal two-sphere worker fence available for operator-directed recovery.
    #[arg(long)]
    operator_recovery: bool,
    /// Publish a new immutable package of the installed scene and ECS drive before serving.
    #[arg(long, conflicts_with = "recover_package")]
    checkpoint_package: Option<PathBuf>,
    /// Publish the installed drive and exact cut here when this process receives SIGUSR1.
    #[arg(long)]
    checkpoint_directory: Option<PathBuf>,
    /// Park a replacement supervisor at this explicitly selected checkpoint.
    #[arg(long, requires = "package_sha256", conflicts_with_all = ["sdf", "drive_program", "torque_nm"])]
    recover_package: Option<PathBuf>,
    /// Independently reviewed full SHA-256 of the selected package.
    #[arg(long, requires = "recover_package")]
    package_sha256: Option<String>,
    /// Number of workers. Defaults to four for ordinary startup or the archived width for recovery.
    #[arg(long, value_parser = clap::value_parser!(u8).range(2..=8))]
    workers: Option<u8>,
    /// Authored version-1 torque timeline in simulation ticks.
    #[arg(long, conflicts_with = "torque_nm")]
    drive_program: Option<PathBuf>,
    /// Signed constant torque in N·m, reapplied each advancing millisecond.
    #[arg(long, default_value_t = 0.01, allow_hyphen_values = true)]
    torque_nm: f64,
    #[arg(long, default_value_t = 9229, value_parser = clap::value_parser!(u16).range(1..))]
    web_viewer: u16,
}

#[path = "support/continuous_hinge_drive.rs"]
mod continuous_hinge_drive;
use continuous_hinge_drive::install_drive;

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let options = HingeWorkerOptions {
        width: args
            .workers
            .or_else(|| args.recover_package.is_none().then_some(4)),
        transport: if args.zenoh {
            WorkerTransport::LoopbackZenoh
        } else {
            WorkerTransport::Pipe
        },
        reopen_attempts: args.reopen_attempts,
    };
    let request = args
        .checkpoint_directory
        .as_deref()
        .map(live::ConvoyCaptureRequest::new)
        .transpose()?;
    if let Some(path) = &args.recover_package {
        return live::run_selected_convoy(
            path,
            args.package_sha256
                .as_deref()
                .ok_or("missing package digest")?,
            &args.worker,
            &args.session,
            options,
            args.web_viewer,
            |sim, capsule| {
                continuous_hinge_drive::install_program(sim, capsule.drive().clone())?;
                if let Some(request) = &request {
                    request.attach(sim, &args.session, capsule.sdf(), capsule.world());
                }
                Ok(())
            },
        );
    }
    let program = args
        .drive_program
        .map(|p| live::JointTorqueDrive::read(std::fs::File::open(p)?))
        .transpose()?;
    if !args.torque_nm.is_finite() {
        return Err("expected finite torque".into());
    }
    let (xml, name) = match args.sdf {
        Some(path) => (std::fs::read_to_string(&path)?, path.display().to_string()),
        None => (
            include_str!("../../../examples/distributed_hinge_impact.sdf").into(),
            "distributed_hinge_impact.sdf".into(),
        ),
    };
    if args.checkpoint_package.is_some() || request.is_some() {
        gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec(&xml, &name, "default")?;
    }
    let capture = std::cell::Cell::new(args.checkpoint_package.as_deref());
    let drive = |sim: &mut gz_sim_core::Simulation| -> Result<(), Box<dyn Error>> {
        match &program {
            Some(program) => continuous_hinge_drive::install_program(sim, program.clone())?,
            None => install_drive(sim, args.torque_nm)?,
        }
        if let Some(request) = &request {
            request.attach(sim, &args.session, &xml, "default");
        }
        if let Some(output) = capture.take() {
            let capsule = live::ConvoyCapsule::capture_installed(
                sim,
                &args.session.join("barrier.gzmultisphere"),
                &xml,
                "default",
            )?;
            let digest = capsule.publish(output)?;
            eprintln!(
                "Published supervisor checkpoint {} at epoch {} tick {} SHA-256 {digest}",
                output.display(),
                sim.epoch(),
                capsule.cut().cut.observation.hinge.tick
            );
        }
        Ok(())
    };
    if args.operator_recovery {
        return live::run_recoverable_convoy(
            live::ConvoyDeployment {
                xml: &xml,
                source: &name,
                world: "default",
                worker: &args.worker,
                session: &args.session,
                options,
            },
            args.web_viewer,
            drive,
        );
    }
    let mut sim = load_distributed_hinge_world_with_options(
        &xml,
        &name,
        "default",
        &args.worker,
        &args.session,
        options,
    )?;
    drive(&mut sim)?;
    live::run(
        sim,
        "default",
        false,
        0,
        None,
        ViewerOptions {
            port: Some(args.web_viewer),
            lifecycle_sdf: None,
            recording_path: None,
        },
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use gz_physics::PhysicsBackend;
    use gz_sim_core::{Simulation, entity_by_id};
    use gz_sim_scene::JointHandle;
    use gz_sim_scene::distributed_hinge::{
        DistributedMultiSphereBackend, load_distributed_hinge_world,
    };
    use gz_sim_scene::{JointSample, queue_joint_effort, reset_loaded_world};
    use std::{fs, path::Path};

    #[test]
    #[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock physics_worker"]
    fn live_drive_matches_durable_queue_through_warm_reopen_and_reset() -> Result<(), Box<dyn Error>>
    {
        const TORQUE: f64 = 0.01;
        let xml = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
        let worker = PathBuf::from(
            std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?,
        );
        let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
        let a = root.path().join("live");
        let b = root.path().join("reference");
        let host = |p: &Path| {
            load_distributed_hinge_world(xml, "convoy.sdf", "default", &worker, p, Some(4))
        };
        let mut live = host(&a)?;
        let mut reference = host(&b)?;
        let joint = live
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMultiSphereBackend>()
            .unwrap()
            .pool()
            .spec()
            .hinge
            .joint_id;
        let entity = entity_by_id(live.world(), joint).unwrap();
        let handle = live.world().get::<JointHandle>(entity).unwrap().id();
        let stage = |sim: &mut Simulation, value| {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<DistributedMultiSphereBackend>()
                .unwrap()
                .stage_step_effort(handle, value)
        };
        let archive = |d: &Path| fs::read(d.join("barrier.gzmultisphere"));
        // Reject a contribution whose sum with a saved torque overflows.
        queue_joint_effort(&mut live, joint, f64::MAX)?;
        let pending_max = archive(&a)?;
        assert!(stage(&mut live, f64::MAX).is_err());
        assert_eq!(archive(&a)?, pending_max);
        reset_loaded_world(&mut live, |_| ())?;
        reset_loaded_world(&mut reference, |_| ())?;
        // A durable command arriving after staging must also preserve a finite next sum.
        stage(&mut live, f64::MAX)?;
        let staged_only = archive(&a)?;
        assert!(queue_joint_effort(&mut live, joint, f64::MAX).is_err());
        assert_eq!(archive(&a)?, staged_only);
        drop(live);
        live = host(&a)?;
        // A stale guard cannot occupy the staging slot or change the saved input.
        let staged = |sim: &mut Simulation, epoch, tick, torque| {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<DistributedMultiSphereBackend>()
                .unwrap()
                .stage_step_effort_at(epoch, tick, handle, torque)
        };
        let cut = archive(&a)?;
        assert!(staged(&mut live, 0, 0, TORQUE).is_err());
        assert!(staged(&mut live, 1, 1, TORQUE).is_err());
        assert_eq!(cut, archive(&a)?);
        staged(&mut live, 1, 0, TORQUE)?;
        assert!(stage(&mut live, TORQUE).is_err());
        live.tick()?;
        assert_eq!(cut, archive(&a)?);
        queue_joint_effort(&mut reference, joint, TORQUE)?;
        live.step_exact(1)?;
        reference.step_exact(1)?;
        assert_eq!(archive(&a)?, archive(&b)?);
        reset_loaded_world(&mut live, |_| ())?;
        reset_loaded_world(&mut reference, |_| ())?;
        // A staged command survives pause and reaches disk only on an advancing step.
        let cold = archive(&a)?;
        assert!(stage(&mut live, f64::NAN).is_err());
        stage(&mut live, TORQUE)?;
        assert!(stage(&mut live, 2. * TORQUE).is_err());
        live.tick()?;
        assert_eq!(archive(&a)?, cold);
        assert_eq!(live.elapsed(), std::time::Duration::ZERO);
        // Closing this host before it steps discards its local input.
        drop(live);
        live = host(&a)?;
        live.step_exact(1)?;
        reference.step_exact(1)?;
        assert_eq!(archive(&a)?, archive(&b)?);
        // Reset discards a staged command too.
        stage(&mut live, TORQUE)?;
        reset_loaded_world(&mut live, |_| ())?;
        reset_loaded_world(&mut reference, |_| ())?;
        live.step_exact(1)?;
        reference.step_exact(1)?;
        assert_eq!(archive(&a)?, archive(&b)?);
        // Accumulate a locally staged contribution after a separately durable torque.
        queue_joint_effort(&mut live, joint, -TORQUE)?;
        queue_joint_effort(&mut reference, joint, -TORQUE)?;
        let pending = archive(&a)?;
        stage(&mut live, TORQUE / 2.)?;
        live.tick()?;
        assert_eq!(archive(&a)?, pending);
        queue_joint_effort(&mut reference, joint, TORQUE / 2.)?;
        live.step_exact(1)?;
        reference.step_exact(1)?;
        assert_eq!(archive(&a)?, archive(&b)?);
        live.step_exact(1)?;
        reference.step_exact(1)?;
        assert_eq!(archive(&a)?, archive(&b)?);
        reset_loaded_world(&mut live, |_| ())?;
        reset_loaded_world(&mut reference, |_| ())?;
        install_drive(&mut live, TORQUE)?;
        for tick in 1..=170 {
            queue_joint_effort(&mut reference, joint, TORQUE)?;
            if tick == 151 {
                queue_joint_effort(&mut live, joint, TORQUE)?;
                let saved = archive(&a)?;
                assert_eq!(saved, archive(&b)?);
                drop(live);
                live = host(&a)?;
                assert!(install_drive(&mut live, -TORQUE).is_err());
                assert_eq!(saved, archive(&a)?);
                install_drive(&mut live, TORQUE)?;
                live.tick()?;
                assert_eq!(saved, archive(&a)?);
            }
            live.step_exact(1)?;
            reference.step_exact(1)?;
            assert_eq!(archive(&a)?, archive(&b)?, "tick {tick}");
            let sample = |sim: &Simulation| {
                *sim.world()
                    .get::<JointSample>(entity_by_id(sim.world(), joint).unwrap())
                    .unwrap()
            };
            let (lhs, rhs) = (sample(&live), sample(&reference));
            assert_eq!(
                (lhs.state, lhs.time, lhs.epoch),
                (rhs.state, rhs.time, rhs.epoch)
            );
        }
        reset_loaded_world(&mut live, |_| ())?;
        reset_loaded_world(&mut reference, |_| ())?;
        live.tick()?;
        assert_eq!(archive(&a)?, archive(&b)?);
        live.step_exact(1)?;
        queue_joint_effort(&mut reference, joint, TORQUE)?;
        reference.step_exact(1)?;
        assert_eq!(archive(&a)?, archive(&b)?);
        Ok(())
    }
}

#[cfg(test)]
#[path = "driven_hinge/reopen.rs"]
mod reopen_tests;
