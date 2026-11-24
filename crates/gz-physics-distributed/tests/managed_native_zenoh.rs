// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The native cut is portable between both qualified local worker transports.
#![cfg(target_os = "linux")]
use gz_physics_distributed::{moving::WorkerTransport, multi_sphere::MultiSpherePool};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::Path,
    process::{Command, Stdio},
    thread,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";

fn rejected_forwarded_offer(lease: &Path, prefix: &[u8]) -> Result<String> {
    use std::io::Write;

    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let mut offer = prefix.to_vec();
    bincode::serialize_into(
        &mut offer,
        &(*b"GZ2SWK02", 0_u8, [7_u8; 16], serde_json::to_vec(&spec)?),
    )?;
    let mut hash = Sha256::new();
    std::io::copy(&mut fs::File::open(WORKER)?, &mut hash)?;
    let mut child = Command::new(WORKER)
        .args(["--managed-multisphere-ssh-v1", "0"])
        .arg(lease)
        .args(["41701", "51701"])
        .arg(format!("{:x}", hash.finalize()))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
    let mut input = child.stdin.take().unwrap();
    input.write_all(&u32::try_from(offer.len())?.to_le_bytes())?;
    input.write_all(&offer)?;
    // Keep the SSH owner channel live. The refusal must precede any ready
    // message or controller EOF.
    let deadline = Instant::now() + Duration::from_secs(6);
    while child.try_wait()?.is_none() {
        if Instant::now() >= deadline {
            child.kill()?;
            child.wait()?;
            return Err("forwarded worker failed to reject its bootstrap".into());
        }
        thread::sleep(Duration::from_millis(5));
    }
    let output = child.wait_with_output()?;
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    Ok(String::from_utf8(output.stderr)?)
}

#[test]
fn forwarded_slot_refuses_a_valid_pipe_offer_before_acquiring_its_lease() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let lease = dir.path().join("slot.lock");
    assert!(rejected_forwarded_offer(&lease, b"")?.contains("requires a Zenoh bootstrap"));
    assert!(!lease.exists());
    Ok(())
}

#[test]
fn forwarded_slot_refuses_a_fifo_lease_before_advertising_an_endpoint() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let lease = dir.path().join("slot.lock");
    rustix::fs::mkfifoat(
        rustix::fs::CWD,
        &lease,
        rustix::fs::Mode::RUSR | rustix::fs::Mode::WUSR,
    )?;
    assert!(
        rejected_forwarded_offer(&lease, b"GZPZNH01")?.contains("lease must be a regular file")
    );
    Ok(())
}

#[test]
fn forwarded_slot_checks_the_running_image_before_reading_the_offer() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let lease = dir.path().join("slot.lock");
    for (digest, expected) in [
        ("0".repeat(64), "running worker SHA-256 differs"),
        ("A".repeat(64), "64 lowercase hexadecimal digits"),
        ("a".repeat(63), "64 lowercase hexadecimal digits"),
    ] {
        let output = Command::new(WORKER)
            .args(["--managed-multisphere-ssh-v1", "0"])
            .arg(&lease)
            .args(["41701", "51701"])
            .arg(digest)
            .stdin(Stdio::null())
            .output()?;
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
        assert!(String::from_utf8(output.stderr)?.contains(expected));
        assert!(!lease.exists());
    }
    Ok(())
}

#[test]
fn forwarded_slot_rejects_preclosed_bootstrap_with_consistent_diagnostics() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let lease = dir.path().join("slot");
    let lease_path = lease.to_str().unwrap();
    let mut hash = Sha256::new();
    std::io::copy(&mut fs::File::open(WORKER)?, &mut hash)?;
    let digest = format!("{:x}", hash.finalize());
    let wrong = "0".repeat(64);
    for (args, diagnostic) in [
        (vec![], "usage:"),
        (
            vec!["9", lease_path, "1", "2", &digest],
            "multi-sphere slot",
        ),
        (vec!["0", "relative", "1", "2", &digest], "absolute lease"),
        (vec!["0", lease_path, "0", "2", &digest], "Zero"),
        (vec!["0", lease_path, "1", "65536", &digest], "PosOverflow"),
        (
            vec!["bad", "relative", "0", "0", &wrong],
            "running worker SHA-256 differs",
        ),
        (vec!["0", lease_path, "1", "2", &digest], "UnexpectedEof"),
    ] {
        let (reader, writer) = std::io::pipe()?;
        drop(writer);
        let output = Command::new(WORKER)
            .arg("--managed-multisphere-ssh-v1")
            .args(args)
            .stdin(reader)
            .output()?;
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
        let stderr = String::from_utf8(output.stderr)?;
        assert!(stderr.contains(diagnostic), "{stderr}");
        assert!(!lease.exists());
    }
    Ok(())
}

#[test]
#[ignore = "launched and killed by zenoh_parent_death_preserves_loaded_native_cut"]
fn zenoh_controller_child() -> Result<()> {
    let ledger = std::env::var("GZ_NATIVE_ZENOH_TEST_CUT")?;
    let dir = Path::new(&ledger);
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let j = spec.hinge.joint_id;
    let mut pool = MultiSpherePool::create_with_transport(
        Path::new(WORKER),
        dir,
        4,
        spec,
        WorkerTransport::LoopbackZenoh,
    )?;
    for tick in 0..150 {
        if tick == 149 {
            pool.drop_next_acknowledgement(pool.owners()[1])?;
        }
        pool.advance_with_effort(j, 0.01)?;
    }
    assert!(pool.observation().sphere_reaction.normal_impulse > 0.);
    pool.transfer_pair(pool.owners()[0])?;
    pool.queue_effort(j, -0.005)?;
    let pids: Vec<_> = (0..4).map(|slot| pool.worker_pid(slot).unwrap()).collect();
    fs::write(dir.join("ready.tmp"), serde_json::to_vec(&pids)?)?;
    fs::rename(dir.join("ready.tmp"), dir.join("ready"))?;
    loop {
        thread::park();
    }
}

#[test]
fn zenoh_parent_death_preserves_loaded_native_cut() -> Result<()> {
    let dir = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let reference = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let worker = Path::new(WORKER);
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let j = spec.hinge.joint_id;
    let mut child = Command::new(std::env::current_exe()?)
        .args([
            "--exact",
            "zenoh_controller_child",
            "--ignored",
            "--nocapture",
        ])
        .env("GZ_NATIVE_ZENOH_TEST_CUT", dir.path())
        .stdin(Stdio::null())
        .spawn()?;
    let until = Instant::now() + Duration::from_secs(40);
    while !dir.path().join("ready").exists() {
        if let Some(status) = child.try_wait()? {
            return Err(format!("controller stopped before loaded cut: {status}").into());
        }
        if Instant::now() >= until {
            child.kill()?;
            child.wait()?;
            return Err("controller did not reach loaded cut".into());
        }
        thread::sleep(Duration::from_millis(10));
    }
    let pids: Vec<u32> = serde_json::from_slice(&fs::read(dir.path().join("ready"))?)?;
    child.kill()?;
    child.wait()?;
    let until = Instant::now() + Duration::from_secs(6);
    for pid in pids {
        while fs::read_to_string(format!("/proc/{pid}/stat")).is_ok_and(|s| {
            s.rsplit_once(") ")
                .is_some_and(|(_, v)| !v.starts_with("Z "))
        }) {
            if Instant::now() >= until {
                return Err(format!(
                    "managed network worker {pid} still running after owner death"
                )
                .into());
            }
            thread::sleep(Duration::from_millis(10));
        }
    }
    let mut direct = MultiSpherePool::create(worker, reference.path(), 4, spec)?;
    for _ in 0..150 {
        direct.advance_with_effort(j, 0.01)?;
    }
    direct.transfer_pair(direct.owners()[0])?;
    direct.queue_effort(j, -0.005)?;
    let mut recovered = MultiSpherePool::resume(worker, dir.path())?;
    assert_eq!(
        fs::read(dir.path().join(ARCHIVE))?,
        fs::read(reference.path().join(ARCHIVE))?
    );
    for tick in 150..185 {
        if tick == 167 {
            drop(recovered);
            recovered = MultiSpherePool::resume_with_transport(
                worker,
                dir.path(),
                WorkerTransport::LoopbackZenoh,
            )?;
            recovered.drop_next_acknowledgement(recovered.owners()[0])?;
        }
        for pool in [&mut direct, &mut recovered] {
            pool.advance_with_effort_at(0, tick, j, 0.01)?;
        }
        assert_eq!(
            fs::read(dir.path().join(ARCHIVE))?,
            fs::read(reference.path().join(ARCHIVE))?
        );
    }
    Ok(())
}

#[test]
fn moving_sphere_reopens_on_either_transport_and_retries_a_killed_owner() -> Result<()> {
    use gz_physics_distributed::moving::MovingPool;
    use gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec;
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let xml = include_str!("../../../examples/distributed_hinge_impact.sdf");
    let spec = distributed_moving_hinge_spec(xml, "impact.sdf", "default")?;
    let worker = Path::new(WORKER);
    let fresh = || tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir());
    let (source, compare) = (fresh()?, fresh()?);
    let mut a = MovingPool::open_scene_with_transport(
        worker,
        source.path(),
        Some(3),
        spec,
        WorkerTransport::LoopbackZenoh,
    )?;
    let mut b = MovingPool::create(worker, compare.path(), 3, spec)?;
    let j = spec.hinge.joint_id;
    let mut hits = 0;
    for tick in 0..320 {
        if tick == 205 {
            assert!(
                a.observation()
                    .reactions
                    .iter()
                    .any(|r| r.normal_impulse > 0.)
            );
            for pool in [&mut a, &mut b] {
                pool.queue_effort(j, -0.005)?;
                pool.transfer((pool.owners()[0] + 1) % 3)?;
            }
            let pid = Pid::from_raw(a.worker_pid(a.owners()[0]).unwrap().try_into()?).unwrap();
            pidfd_send_signal(&pidfd_open(pid, PidfdFlags::empty())?, Signal::KILL)?;
        }
        if tick == 206 || tick == 250 {
            drop(a);
            let transport = if tick == 206 {
                WorkerTransport::Pipe
            } else {
                WorkerTransport::LoopbackZenoh
            };
            a = MovingPool::resume_with_transport(worker, source.path(), transport)?;
        }
        for pool in [&mut a, &mut b] {
            pool.advance_with_effort_at(0, tick, j, 0.01)?;
        }
        assert_eq!(a.observation(), b.observation());
        assert_eq!(
            fs::read(source.path().join("barrier.gzmoving"))?,
            fs::read(compare.path().join("barrier.gzmoving"))?
        );
        hits += u32::from(
            a.observation()
                .reactions
                .iter()
                .any(|r| r.normal_impulse > 0.),
        );
    }
    assert!(hits > 0);
    Ok(())
}
