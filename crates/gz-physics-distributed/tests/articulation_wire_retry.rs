// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::articulation::{
    ArticulationPool, ArticulationSpec, NativeArticulation,
};
use std::{error::Error, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn spec() -> Result<ArticulationSpec> {
    Ok(gz_sim_scene::distributed_hinge_spec(
        include_str!("../../../examples/multifidelity_articulation.sdf"),
        "multifidelity_articulation.sdf",
        "default",
    )?)
}
#[test]
fn lost_step_effort_prepare_and_fence_acknowledgements_keep_live_workers_and_exact_state()
-> Result<()> {
    let dir = tempfile::tempdir()?;
    let spec = spec()?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut pool = ArticulationPool::create(exe, dir.path(), 2, spec)?;
    let pids = [pool.worker_pid(0).unwrap(), pool.worker_pid(1).unwrap()];
    let mut reference = NativeArticulation::new(&spec)?;
    // The worker executes and commits each command before withholding its reply.
    pool.drop_next_acknowledgement(0)?;
    pool.queue_effort(spec.joint_id, 0.25)?;
    reference.effort(spec.joint_id, 0.25)?;
    assert_eq!(pool.image(), reference.capture()?);
    pool.drop_next_acknowledgement(0)?;
    pool.advance()?;
    reference.advance()?;
    assert_eq!(pool.image(), reference.capture()?);
    pool.queue_effort(spec.joint_id, -0.3125)?;
    reference.effort(spec.joint_id, -0.3125)?;
    let moving = reference.capture()?;
    pool.drop_next_acknowledgement(1)?; // destination stage
    pool.drop_next_acknowledgement(0)?; // source fence
    pool.transfer(1)?;
    assert_eq!((pool.owner(), pool.generation()), (1, 1));
    assert_eq!(pool.image(), moving);
    assert_eq!(
        pids,
        [pool.worker_pid(0).unwrap(), pool.worker_pid(1).unwrap()]
    );
    for _ in 0..16 {
        pool.advance()?;
        reference.advance()?;
        assert_eq!(pool.image(), reference.capture()?);
        assert_eq!(pool.observation(), &reference.observe());
    }
    drop(pool);
    let resumed = ArticulationPool::resume(exe, dir.path())?;
    assert_eq!((resumed.owner(), resumed.generation()), (1, 1));
    assert_eq!(resumed.image(), reference.capture()?);
    Ok(())
}

#[test]
fn mixed_hinge_seed_and_offer_fail_before_slot_lease_is_acquired() -> Result<()> {
    use std::{
        io::{BufRead, BufReader, Write},
        process::{Command, Stdio},
        thread,
        time::{Duration, Instant},
    };
    let dir = tempfile::tempdir()?;
    let grounded = gz_sim_scene::distributed_hinge_spec(
        include_str!("../../../examples/distributed_grounded_hinge.sdf"),
        "distributed_grounded_hinge.sdf",
        "default",
    )?;
    let lease = dir.path().join("slot");
    for (flag, offered, wrong_seed) in [
        (
            "--managed-hinge-v1",
            "GZARTICULATION/FLOATING-REVOLUTE/1",
            grounded,
        ),
        (
            "--managed-ground-hinge-v1",
            "GZARTICULATION/GROUNDED-REVOLUTE/1",
            spec()?,
        ),
    ] {
        let mut child = Command::new(env!("CARGO_BIN_EXE_physics_worker"))
            .args(["--run", "341", "--worker", "0", flag])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()?;
        let mut stdout = BufReader::new(child.stdout.take().unwrap());
        let mut offer = String::new();
        stdout.read_line(&mut offer)?;
        assert_eq!(offer.trim_end(), offered);
        let mut owner_pipe = child.stdin.take().unwrap();
        serde_json::to_writer(
            &mut owner_pipe,
            &serde_json::json!({"spec":wrong_seed, "slot_lock":lease}),
        )?;
        owner_pipe.write_all(b"\n")?;
        let deadline = Instant::now() + Duration::from_secs(5);
        while child.try_wait()?.is_none() {
            if Instant::now() > deadline {
                child.kill()?;
                child.wait()?;
                return Err("mixed seed did not terminate".into());
            }
            thread::sleep(Duration::from_millis(2));
        }
        let result = child.wait_with_output()?;
        assert!(!result.status.success());
        assert!(
            String::from_utf8(result.stderr)?.contains("seed disagrees with negotiated profile")
        );
        assert!(!lease.exists());
    }
    Ok(())
}

#[test]
fn grounded_owner_keeps_manifold_and_held_torque_across_lost_replies() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let sdf = include_str!("../../../examples/distributed_grounded_hinge.sdf");
    let spec =
        gz_sim_scene::distributed_hinge_spec(sdf, "distributed_grounded_hinge.sdf", "default")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut pool = ArticulationPool::create(exe, dir.path(), 2, spec)?;
    let original = [pool.worker_pid(0), pool.worker_pid(1)];
    let mut reference = NativeArticulation::new(&spec)?;
    for _ in 0..600 {
        pool.advance()?;
        reference.advance()?;
        assert_eq!(pool.observation(), &reference.observe());
        if reference
            .observe()
            .ground_reactions
            .unwrap()
            .iter()
            .any(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
        {
            break;
        }
    }
    let loaded = reference.observe();
    assert!(
        loaded
            .ground_reactions
            .unwrap()
            .iter()
            .any(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
    );
    pool.drop_next_acknowledgement(0)?;
    pool.queue_effort(spec.joint_id, 0.375)?;
    reference.effort(spec.joint_id, 0.375)?;
    pool.drop_next_acknowledgement(1)?;
    pool.drop_next_acknowledgement(0)?;
    pool.transfer(1)?;
    assert_eq!(pool.observation(), &loaded);
    assert_eq!(pool.image(), reference.capture()?);
    pool.drop_next_acknowledgement(1)?;
    pool.advance()?;
    reference.advance()?;
    assert_eq!(pool.image(), reference.capture()?);
    assert_eq!(original, [pool.worker_pid(0), pool.worker_pid(1)]);
    drop(pool);
    let mut pool = ArticulationPool::resume(exe, dir.path())?;
    for _ in 0..16 {
        pool.advance()?;
        reference.advance()?;
        assert_eq!(pool.observation(), &reference.observe());
        assert_eq!(pool.image(), reference.capture()?);
    }
    Ok(())
}

#[test]
#[ignore = "requires a retained physics_worker built before managed-hinge-v1"]
fn pre_hinge_worker_refuses_the_new_offer_before_the_initial_barrier() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let exe = std::env::var_os("GZ_PRE_HINGE_WORKER").ok_or("set GZ_PRE_HINGE_WORKER")?;
    let error = ArticulationPool::create(Path::new(&exe), dir.path(), 2, spec()?)
        .err()
        .ok_or("legacy worker admitted an articulation")?;
    assert!(error.to_string().contains("does not offer"), "{error}");
    assert!(!dir.path().join("barrier.gzhinge").exists());
    Ok(())
}

#[cfg(target_os = "linux")]
#[test]
fn surviving_stopped_owner_keeps_its_slot_fenced_after_coordinator_death() -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    use std::{
        fs, io,
        os::fd::OwnedFd,
        process::{Child, Command, Stdio},
        thread,
        time::{Duration, Instant},
    };
    const CHILD_ROOT: &str = "GZ_HINGE_STOPPED_OWNER_TEST";
    const NAME: &str = "surviving_stopped_owner_keeps_its_slot_fenced_after_coordinator_death";
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = spec()?;
    if let Some(root) = std::env::var_os(CHILD_ROOT) {
        let root = Path::new(&root);
        let mut pool = ArticulationPool::create(exe, &root.join("session"), 2, spec)?;
        pool.queue_effort(spec.joint_id, 0.125)?;
        pool.advance()?;
        pool.transfer(1)?;
        pool.queue_effort(spec.joint_id, -0.25)?;
        fs::write(
            root.join("pid.tmp"),
            pool.worker_pid(1).unwrap().to_string(),
        )?;
        fs::rename(root.join("pid.tmp"), root.join("pid"))?;
        thread::park_timeout(Duration::from_secs(30));
        return Err("test controller was not killed".into());
    }
    struct Controller(Child);
    impl Drop for Controller {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    struct StoppedOwner(OwnedFd);
    impl Drop for StoppedOwner {
        fn drop(&mut self) {
            let _ = pidfd_send_signal(&self.0, Signal::KILL);
        }
    }
    let root = tempfile::tempdir()?;
    let mut controller = Controller(
        Command::new(std::env::current_exe()?)
            .args(["--exact", NAME, "--nocapture"])
            .env(CHILD_ROOT, root.path())
            .stdout(Stdio::null())
            .spawn()?,
    );
    let deadline = Instant::now() + Duration::from_secs(15);
    let pid: u32 = loop {
        match fs::read_to_string(root.path().join("pid")) {
            Ok(s) => break s.parse()?,
            Err(e)
                if e.kind() == io::ErrorKind::NotFound
                    && Instant::now() < deadline
                    && controller.0.try_wait()?.is_none() =>
            {
                thread::sleep(Duration::from_millis(5))
            }
            Err(e) => return Err(e.into()),
        }
    };
    let owner = StoppedOwner(pidfd_open(
        Pid::from_raw(pid.try_into()?).ok_or("invalid PID")?,
        PidfdFlags::empty(),
    )?);
    pidfd_send_signal(&owner.0, Signal::STOP)?;
    let deadline = Instant::now() + Duration::from_secs(3);
    while !fs::read_to_string(format!("/proc/{pid}/status"))?
        .lines()
        .any(|line| line.starts_with("State:\tT"))
    {
        if Instant::now() >= deadline {
            return Err("owner did not stop".into());
        }
        thread::sleep(Duration::from_millis(1));
    }
    controller.0.kill()?;
    controller.0.wait()?;
    let session = root.path().join("session");
    let lease = fs::File::options()
        .read(true)
        .write(true)
        .open(session.join("worker-1.lock"))?;
    assert!(matches!(
        lease.try_lock(),
        Err(fs::TryLockError::WouldBlock)
    ));
    let before = fs::read(session.join("barrier.gzhinge"))?;
    let refusal = ArticulationPool::resume(exe, &session)
        .err()
        .ok_or("resumed an unfenced stopped process")?;
    assert!(refusal.to_string().contains("timed out"), "{refusal}");
    assert_eq!(before, fs::read(session.join("barrier.gzhinge"))?);
    assert!(matches!(
        lease.try_lock(),
        Err(fs::TryLockError::WouldBlock)
    ));
    // Resuming the old process lets the managed owner-pipe EOF terminate it.
    pidfd_send_signal(&owner.0, Signal::CONT)?;
    let mut recovered = ArticulationPool::resume(exe, &session)?;
    let mut direct = NativeArticulation::new(&spec)?;
    direct.effort(spec.joint_id, 0.125)?;
    direct.advance()?;
    direct.effort(spec.joint_id, -0.25)?;
    assert_eq!((recovered.owner(), recovered.generation()), (1, 1));
    assert_ne!(recovered.worker_pid(1), Some(pid));
    assert_eq!(recovered.image(), direct.capture()?);
    recovered.advance()?;
    direct.advance()?;
    assert_eq!(recovered.image(), direct.capture()?);
    Ok(())
}
