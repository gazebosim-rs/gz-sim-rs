// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn unsupported_reopen_settings_are_rejected_before_session_creation() -> Result<(), Box<dyn Error>>
{
    let args = ["driven_hinge", "--worker", "missing", "--session", "unused"];
    assert_eq!(Args::try_parse_from(args)?.reopen_attempts, 0);
    assert!(Args::try_parse_from(args.into_iter().chain(["--reopen-attempts", "9"])).is_err());

    let root = tempfile::tempdir()?;
    let session = root.path().join("unused");
    let worker = root.path().join("missing");
    for (xml, attempts, expected) in [
        (
            include_str!("../../../../examples/distributed_hinge_impact.sdf"),
            1,
            "requires a hinge and two free spheres",
        ),
        (
            include_str!("../../../../examples/distributed_sphere_pair_island.sdf"),
            u8::MAX,
            "exceed the multi-sphere limit",
        ),
    ] {
        let result = load_distributed_hinge_world_with_options(
            xml,
            "test.sdf",
            "default",
            &worker,
            &session,
            HingeWorkerOptions {
                reopen_attempts: attempts,
                ..Default::default()
            },
        );
        let err = result.err().expect("expected an invalid recovery setting");
        assert!(err.to_string().contains(expected), "{err}");
        assert!(!session.exists());
    }
    Ok(())
}

#[cfg(target_os = "linux")]
#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock physics_worker"]
fn driven_scene_reopens_both_staged_and_durably_queued_steps() -> Result<(), Box<dyn Error>> {
    use gz_physics::PhysicsBackend;
    use gz_sim_scene::{distributed_hinge::DistributedMultiSphereBackend, queue_joint_effort};
    use rustix::{
        event::{PollFd, PollFlags, Timespec, poll},
        process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal},
    };
    use std::{fs, path::Path, thread, time::Duration};

    let worker =
        PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?);
    let xml = include_str!("../../../../examples/distributed_sphere_pair_island.sdf");
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let dir = root.path().join("live");
    let control_dir = root.path().join("control");
    let load = |p: &Path, attempts| {
        load_distributed_hinge_world_with_options(
            xml,
            "convoy.sdf",
            "default",
            &worker,
            p,
            HingeWorkerOptions {
                width: Some(3),
                reopen_attempts: attempts,
                ..Default::default()
            },
        )
    };
    let mut live = load(&dir, 2)?;
    let mut control = load(&control_dir, 0)?;
    install_drive(&mut live, 0.01)?;
    install_drive(&mut control, 0.01)?;

    for (index, already_queued) in [false, true].into_iter().enumerate() {
        let pool = live
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMultiSphereBackend>()
            .unwrap()
            .pool();
        let joint = pool.spec().hinge.joint_id;
        let watch = |slot| {
            let pid = Pid::from_raw(pool.worker_pid(slot).unwrap() as i32).unwrap();
            pidfd_open(pid, PidfdFlags::empty())
        };
        let lost = watch(0)?;
        let sibling = watch(2)?;
        if already_queued {
            queue_joint_effort(&mut live, joint, 0.01)?;
            queue_joint_effort(&mut control, joint, 0.01)?;
        }
        pidfd_send_signal(lost, Signal::KILL)?;
        let lease = fs::File::open(dir.join("worker-0.lock"))?;
        lease.lock()?;
        // Keep the failed slot unavailable through ordinary repair. The healthy
        // sibling exits when the pool fences. Only then allow a fresh worker in.
        let holder = thread::spawn(move || {
            let mut fd = [PollFd::new(&sibling, PollFlags::IN)];
            let timeout = Timespec::try_from(Duration::from_secs(20)).unwrap();
            let exited = poll(&mut fd, Some(&timeout)).unwrap() == 1;
            drop(lease);
            exited
        });
        let advanced = live.step_exact(1);
        assert!(
            holder.join().unwrap(),
            "pool never fenced its healthy sibling"
        );
        advanced?;
        control.step_exact(1)?;
        let pool = live
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMultiSphereBackend>()
            .unwrap()
            .pool();
        assert_eq!(pool.observation().hinge.tick, (index + 1) as u64);
        assert_eq!(pool.reopen_attempts_used(), (index + 1) as u64);
        assert_eq!(pool.pending_joint_effort(), None);
        assert_eq!(live.elapsed(), control.elapsed());
        assert_eq!(
            fs::read(dir.join("barrier.gzmultisphere"))?,
            fs::read(control_dir.join("barrier.gzmultisphere"))?
        );
    }
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock physics_worker"]
fn scheduled_input_survives_pause_reopen_and_epoch_reset() -> Result<(), Box<dyn Error>> {
    use gz_physics::PhysicsBackend;
    use gz_sim_scene::distributed_hinge::{
        DistributedMultiSphereBackend, load_distributed_hinge_world,
    };
    use gz_sim_scene::{queue_joint_effort, reset_loaded_world};
    use std::fs;
    let worker =
        PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?);
    let root = tempfile::tempdir()?;
    let path = root.path().join("live");
    let reference_path = root.path().join("reference");
    let xml = include_str!("../../../../examples/distributed_sphere_pair_island.sdf");
    let load = |path: &std::path::Path| {
        load_distributed_hinge_world(xml, "drive.sdf", "default", &worker, path, Some(3))
    };
    let mut sim = load(&path)?;
    let mut reference = load(&reference_path)?;
    // Interval 1 crosses a torque reversal. After Reset interval 0 must again use -0.02.
    let drive=live::JointTorqueDrive::read(br#"{"version":1,"interpolation":"hold","end":"repeat","knots":[{"tick":0,"torque_nm":-0.02},{"tick":1,"torque_nm":0.01},{"tick":3,"torque_nm":-0.02}]}"#.as_slice())?;
    let joint = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedMultiSphereBackend>()
        .unwrap()
        .pool()
        .spec()
        .hinge
        .joint_id;
    let archive = |p: &std::path::Path| fs::read(p.join("barrier.gzmultisphere"));
    continuous_hinge_drive::install_program(&mut sim, drive.clone())?;
    sim.step_exact(1)?;
    queue_joint_effort(&mut reference, joint, -0.02)?;
    reference.step_exact(1)?;
    assert_eq!(archive(&path)?, archive(&reference_path)?);
    queue_joint_effort(&mut sim, joint, 0.01)?;
    queue_joint_effort(&mut reference, joint, 0.01)?;
    let saved = archive(&path)?;
    drop(sim);
    let mut sim = load(&path)?;
    assert!(install_drive(&mut sim, -0.02).is_err());
    continuous_hinge_drive::install_program(&mut sim, drive)?;
    sim.set_paused(true);
    assert!(sim.tick()?.dt.is_zero());
    assert_eq!(archive(&path)?, saved);
    sim.step_exact(1)?;
    reference.step_exact(1)?;
    assert_eq!(archive(&path)?, archive(&reference_path)?);
    reset_loaded_world(&mut sim, |_| ())?;
    reset_loaded_world(&mut reference, |_| ())?;
    assert_eq!(sim.epoch(), 1);
    assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
    sim.step_exact(1)?;
    queue_joint_effort(&mut reference, joint, -0.02)?;
    reference.step_exact(1)?;
    assert_eq!(archive(&path)?, archive(&reference_path)?);
    Ok(())
}
