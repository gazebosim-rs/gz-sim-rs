// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{
    fs,
    process::Command,
    time::{Duration, Instant},
};

const CHILD: &str = "articulation::crash_tests::coordinator_process";

pub(super) fn gate(edge: &str) {
    if std::env::var("GZ_HINGE_CRASH_EDGE").as_deref() == Ok(edge) {
        let marker =
            PathBuf::from(std::env::var_os("GZ_HINGE_CRASH_MARKER").expect("private process gate"));
        fs::write(marker, edge).unwrap();
        std::thread::sleep(Duration::from_secs(30));
        panic!("test controller was not terminated");
    }
}

#[test]
#[ignore = "private crash harness child"]
fn coordinator_process() {
    let exe = PathBuf::from(std::env::var_os("GZ_HINGE_TEST_WORKER").unwrap());
    let directory = PathBuf::from(std::env::var_os("GZ_HINGE_TEST_SESSION").unwrap());
    let mut pool = ArticulationPool::resume(&exe, &directory).unwrap();
    match std::env::var("GZ_HINGE_OPERATION").unwrap().as_str() {
        "effort" => pool.queue_effort(pool.spec().joint_id, -0.375).unwrap(),
        "step" => pool.advance().unwrap(),
        "atomic" => pool
            .advance_with_effort(pool.spec().joint_id, -0.375)
            .unwrap(),
        "reset" => pool.reset().unwrap(),
        "transfer" => pool.transfer(1).unwrap(),
        other => panic!("unknown test operation {other}"),
    }
    panic!("crash boundary was not reached");
}

#[test]
#[ignore = "requires GZ_HINGE_TEST_WORKER pointing to the stock physics_worker binary"]
fn real_controller_loss_keeps_one_tree_and_consumes_pending_effort_once() -> Result<()> {
    verify_crash_edges(native::tests::spec())
}

#[test]
#[ignore = "requires GZ_HINGE_TEST_WORKER pointing to the stock physics_worker binary"]
fn real_controller_loss_preserves_ground_contact_history_and_effort() -> Result<()> {
    let mut spec = native::tests::spec();
    spec.ground = Some(ArticulationGroundSpec {
        link_id: 2,
        gravity_z: -9.81,
        height: 0.84,
        size: [6., 4.],
    });
    for link in &mut spec.links {
        link.velocity[2] = -0.08;
    }
    verify_crash_edges(spec)
}

fn verify_crash_edges(spec: ArticulationSpec) -> Result<()> {
    let exe =
        PathBuf::from(std::env::var_os("GZ_HINGE_TEST_WORKER").expect("stock physics_worker path"));
    for (operation, edge) in [
        ("transfer", "staged"),
        ("transfer", "fenced"),
        ("transfer", "activated"),
        ("transfer", "before-rename"),
        ("transfer", "after-rename"),
        ("effort", "before-rename"),
        ("effort", "after-rename"),
        ("step", "before-rename"),
        ("step", "after-rename"),
        ("reset", "before-rename"),
        ("reset", "after-rename"),
        ("atomic", "atomic-effort-accepted"),
        ("atomic", "atomic-step-prepared"),
        ("atomic", "before-rename"),
        ("atomic", "after-rename"),
        ("atomic", "after-sync"),
    ] {
        let dir = tempfile::tempdir()?;
        let session = dir.path().join("session");
        let marker = dir.path().join("reached");
        let mut reference = NativeArticulation::new(&spec)?;
        let mut pool = ArticulationPool::create(&exe, &session, 2, spec)?;
        for _ in 0..4 {
            pool.advance()?;
            reference.advance()?;
        }
        if spec.ground.is_some() {
            assert!(pool.observation().ground_reactions.unwrap().iter().any(
                |contact| contact.active
                    && contact.normal_impulse > 0.
                    && contact.warmstart_normal_impulse > 0.
            ));
        }
        for torque in [0.75, -0.125] {
            pool.queue_effort(spec.joint_id, torque)?;
            reference.effort(spec.joint_id, torque)?;
        }
        let epoch = pool.epoch();
        let tick = pool.observation().tick;
        drop(pool);
        let mut command = Command::new(std::env::current_exe()?);
        command
            .args(["--exact", CHILD, "--ignored"])
            .env("GZ_HINGE_TEST_SESSION", &session)
            .env("GZ_HINGE_CRASH_MARKER", &marker)
            .env("GZ_HINGE_CRASH_EDGE", edge);
        command.env("GZ_HINGE_OPERATION", operation);
        let mut child = command.spawn()?;
        let deadline = Instant::now() + Duration::from_secs(20);
        while !marker.try_exists()? {
            assert!(child.try_wait()?.is_none(), "child exited before {edge}");
            if Instant::now() >= deadline {
                child.kill()?;
                child.wait()?;
                panic!("coordinator did not reach {edge}");
            }
            std::thread::sleep(Duration::from_millis(3));
        }
        assert!(
            matches!(ArticulationPool::resume(&exe, &session), Err(ArticulationError::Io(e)) if e.kind() == io::ErrorKind::WouldBlock)
        );
        child.kill()?;
        child.wait()?;
        let mut recovered = ArticulationPool::resume(&exe, &session)?;
        let published = matches!(edge, "after-rename" | "after-sync");
        assert_eq!(
            recovered.owner(),
            u8::from(operation == "transfer" && published)
        );
        assert_eq!(
            recovered.generation(),
            u64::from(matches!(operation, "reset" | "transfer") && published)
        );
        if operation == "reset" && published {
            reference = NativeArticulation::new(&spec)?;
        }
        assert_eq!(
            recovered.epoch(),
            u64::from(operation == "reset" && published)
        );
        assert!(!session.join("barrier.pending.gzhinge").exists());
        if matches!(operation, "effort" | "atomic") && published {
            reference.effort(spec.joint_id, -0.375)?;
        }
        if matches!(operation, "step" | "atomic") && published {
            reference.advance()?;
        }
        assert_eq!(recovered.image(), reference.capture()?);
        assert_eq!(
            recovered.pending_joint_effort(),
            reference.pending_joint_effort()
        );
        println!(
            "recovered operation={operation} edge={edge} tick={} owner={} generation={} epoch={}",
            recovered.observation().tick,
            recovered.owner(),
            recovered.generation(),
            recovered.epoch()
        );
        if operation == "atomic" && published {
            assert!(matches!(
                recovered.advance_with_effort_at(epoch, tick, spec.joint_id, -0.375),
                Err(ArticulationError::StaleInterval { .. })
            ));
            assert_eq!(recovered.image(), reference.capture()?);
        }
        if operation == "atomic" && !published {
            recovered.advance_with_effort_at(epoch, tick, spec.joint_id, -0.375)?;
            reference.effort(spec.joint_id, -0.375)?;
            reference.advance()?;
            assert_eq!(recovered.image(), reference.capture()?);
        }
        for _ in 0..12 {
            recovered.advance()?;
            reference.advance()?;
            assert_eq!(recovered.observation(), &reference.observe());
            assert_eq!(recovered.image(), reference.capture()?);
        }
    }
    Ok(())
}
