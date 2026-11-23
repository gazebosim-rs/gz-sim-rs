// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(all(unix, feature = "distributed-physics"))]
use gz_sim_cli::live::{ConvoyCapsule, JointTorqueDrive};
use gz_sim_scene::distributed_hinge::{
    MultiSpherePool, distributed_multi_sphere_spec, load_distributed_hinge_world,
};
use std::{error::Error, fs, path::Path, process::Command};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to a stock physics_worker"]
fn snapshots_retain_origin_drive_and_chronology() -> Result<()> {
    let worker = std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?;
    let tmp = tempfile::tempdir()?;
    let source = tmp.path().join("session");
    let archive = source.join("barrier.gzmultisphere");
    let spec = distributed_multi_sphere_spec(XML, "convoy.sdf", "default")?;
    let mut pool = MultiSpherePool::create(Path::new(&worker), &source, 3, spec)?;
    let sdf = tmp.path().join("scene.sdf");
    fs::write(&sdf, XML)?;
    let initial = tmp.path().join("initial.gzcap");
    let next = tmp.path().join("next.gzcap");
    let refuse = tmp.path().join("refuse.gzcap");
    let exe = env!("CARGO_BIN_EXE_gz-sim-convoy-checkpoint");
    let make = || {
        Command::new(exe)
            .args(["create", "--session"])
            .arg(&source)
            .arg("--sdf")
            .arg(&sdf)
            .args(["--torque-nm", "0.015", "--output"])
            .arg(&initial)
            .output()
    };
    assert!(make()?.status.success());
    let src = ConvoyCapsule::inspect(&initial)?;
    let original = fs::read(&initial)?;
    assert!(!make()?.status.success());
    assert_eq!(fs::read(&initial)?, original);
    let capture = |from: &Path, sha: &str, session: &Path, out: &Path| {
        Command::new(exe)
            .args(["capture-latest", "--from"])
            .arg(from)
            .args(["--package-sha256", sha, "--session"])
            .arg(session)
            .arg("--output")
            .arg(out)
            .output()
    };
    let clean = fs::read(&archive)?;
    pool.queue_effort(spec.hinge.joint_id, 0.015)?;
    assert!(
        !capture(&initial, src.sha256(), &source, &refuse)?
            .status
            .success(),
        "different archive at same clock"
    );
    pool.advance()?;
    assert!(
        capture(&initial, src.sha256(), &source, &next)?
            .status
            .success()
    );
    let later = ConvoyCapsule::inspect(&next)?;
    assert_eq!(later.cut().cut.observation.hinge.tick, 1);
    assert_eq!(later.drive().interval_effort(1), 0.015);
    assert!(
        !capture(&initial, &"0".repeat(64), &source, &refuse)?
            .status
            .success()
    );
    let copy = tmp.path().join("other-session");
    let other = MultiSpherePool::create(Path::new(&worker), &copy, 3, spec)?;
    assert!(
        !capture(&initial, src.sha256(), &copy, &refuse)?
            .status
            .success()
    );
    drop(other);
    // An older file at the original path cannot be blessed as a newer cut.
    drop(pool);
    fs::write(&archive, clean)?;
    assert!(
        !capture(&next, later.sha256(), &source, &refuse)?
            .status
            .success()
    );
    assert!(!refuse.exists());
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to a stock physics_worker"]
fn active_bevy_capture_compares_the_retained_worker_state() -> Result<()> {
    let worker = std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?;
    let tmp = tempfile::tempdir()?;
    let session = tmp.path().join("simulation");
    let archive = session.join("barrier.gzmultisphere");
    let mut sim = load_distributed_hinge_world(
        XML,
        "scene.sdf",
        "default",
        Path::new(&worker),
        &session,
        Some(3),
    )?;
    sim.world_mut()
        .insert_resource(JointTorqueDrive::constant(0.015)?);
    let original = ConvoyCapsule::capture_installed(&sim, &archive, XML, "default")?;
    let other = tmp.path().join("alternate");
    let spec = distributed_multi_sphere_spec(XML, "scene.sdf", "default")?;
    let mut pool = MultiSpherePool::create(Path::new(&worker), &other, 3, spec)?;
    pool.queue_effort(spec.hinge.joint_id, 0.015)?;
    fs::copy(other.join("barrier.gzmultisphere"), &archive)?;
    // The external input is internally valid and matches the drive, recipe and clock.
    let selected = ConvoyCapsule::capture(
        &archive,
        XML,
        "default",
        &JointTorqueDrive::constant(0.015)?,
    )?;
    assert_ne!(selected.cut().sha256, original.cut().sha256);
    assert!(ConvoyCapsule::capture_installed(&sim, &archive, XML, "default").is_err());
    fs::write(&archive, original.archive())?;
    assert_eq!(
        ConvoyCapsule::capture_installed(&sim, &archive, XML, "default")?.sha256(),
        original.sha256()
    );
    Ok(())
}
