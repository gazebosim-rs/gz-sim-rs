// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
use serde_json::Value;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

const OPERATOR: &str = env!("CARGO_BIN_EXE_gz-sim-hinge");

fn scene(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples")
        .join(name)
}

#[test]
fn conflicting_and_nonfinite_atomic_commands_fail_before_session_creation() {
    let temp = tempfile::tempdir().unwrap();
    let session = temp.path().join("invalid");
    let create = scene("distributed_sphere_pair_island.sdf");
    for invalid in [
        vec!["--expect-epoch=0", "--expect-tick=0"],
        vec!["--step-with-effort=.01", "--expect-tick=0"],
        vec!["--step-with-effort=.01", "--expect-epoch=0"],
        vec![
            "--step-with-effort=.01",
            "--expect-epoch=1",
            "--expect-tick=0",
        ],
        vec!["--step-with-effort=NaN"],
        vec!["--step-with-effort=-inf"],
        vec!["--step-with-effort=.01", "--effort=-.02"],
        vec!["--step-with-effort=.01", "--steps=0"],
        vec!["--step-with-effort=.01", "--transfer-to=0"],
        vec!["--step-with-effort=.01", "--transfer-pair-to=2"],
    ] {
        let result = Command::new(OPERATOR)
            .arg(&session)
            .arg("--create")
            .arg(&create)
            .args(invalid)
            .output()
            .unwrap();
        assert!(!result.status.success());
        assert!(!session.exists());
    }
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock physics_worker"]
fn operator_atomic_step_accumulates_pending_torque_and_matches_split_archive()
-> Result<(), Box<dyn Error>> {
    for (source, archive) in [
        ("multifidelity_articulation.sdf", "barrier.gzhinge"),
        ("distributed_grounded_hinge.sdf", "barrier.gzhinge"),
        ("distributed_hinge_impact.sdf", "barrier.gzmoving"),
        (
            "distributed_sphere_pair_island.sdf",
            "barrier.gzmultisphere",
        ),
    ] {
        compare_operator(source, archive)?;
    }
    Ok(())
}

fn compare_operator(source: &str, archive: &str) -> Result<(), Box<dyn Error>> {
    let worker = std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?;
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let atomic = root.path().join("atomic");
    let split = root.path().join("split");
    let create = scene(source);
    let invoke = |dir: &Path, args: &[&str]| -> Result<Value, Box<dyn Error>> {
        let Output {
            status,
            stdout,
            stderr,
        } = Command::new(OPERATOR)
            .arg(dir)
            .arg("--worker")
            .arg(&worker)
            .args(args)
            .output()?;
        assert!(status.success(), "{}", String::from_utf8_lossy(&stderr));
        Ok(serde_json::from_slice(&stdout)?)
    };
    let sdf = create.to_str().unwrap();
    let mut a = invoke(
        &atomic,
        &[
            "--create",
            sdf,
            "--step-with-effort=.01",
            "--expect-epoch=0",
            "--expect-tick=0",
        ],
    )?;
    let mut b = invoke(&split, &["--create", sdf, "--effort=.01", "--steps=1"])?;
    assert_eq!(a, b);
    assert_eq!(a["state"].get("hinge").unwrap_or(&a["state"])["tick"], 1);
    assert_eq!(
        fs::read(atomic.join(archive))?,
        fs::read(split.join(archive))?
    );
    for dir in [&atomic, &split] {
        let staged = invoke(dir, &["--effort=.02"])?;
        assert_eq!(staged["pending_effort_nm"], 0.02);
    }
    let before = fs::read(atomic.join(archive))?;
    let refused = Command::new(OPERATOR)
        .arg(&atomic)
        .arg("--worker")
        .arg(&worker)
        .args(["--step-with-effort=1", "--effort=-1"])
        .output()?;
    assert!(!refused.status.success());
    assert_eq!(before, fs::read(atomic.join(archive))?);
    for stamp in [("0", "0"), ("0", "2"), ("1", "1")] {
        let refused = Command::new(OPERATOR)
            .arg(&atomic)
            .arg("--worker")
            .arg(&worker)
            .args([
                "--step-with-effort=-.03",
                "--expect-epoch",
                stamp.0,
                "--expect-tick",
                stamp.1,
            ])
            .output()?;
        assert!(!refused.status.success());
        assert!(String::from_utf8_lossy(&refused.stderr).contains("StaleInterval"));
        assert_eq!(fs::read(atomic.join(archive))?, before);
        assert_eq!(invoke(&atomic, &[])?["pending_effort_nm"], 0.02);
    }
    a = invoke(
        &atomic,
        &[
            "--step-with-effort=-.03",
            "--expect-epoch=0",
            "--expect-tick=1",
        ],
    )?;
    b = invoke(&split, &["--effort=-.03", "--steps=1"])?;
    assert_eq!(a, b);
    assert_eq!(a["state"].get("hinge").unwrap_or(&a["state"])["tick"], 2);
    assert!(a["pending_effort_nm"].is_null());
    assert_eq!(
        fs::read(atomic.join(archive))?,
        fs::read(split.join(archive))?
    );
    assert_eq!(
        invoke(&atomic, &["--steps=1"])?,
        invoke(&split, &["--steps=1"])?
    );
    Ok(())
}
