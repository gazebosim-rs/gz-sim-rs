// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::floating_tree::crash_tests::terminate_probe;
use std::{error::Error, fs};

type Test = std::result::Result<(), Box<dyn Error>>;
const CHILD: &str = "grounded_robot::crash_tests::controller_at_native_page_or_commit";
fn input(p: &mut GroundedRobotPool, action: &str) -> Test {
    let joint = p.spec().joints[0].joint_id;
    match action {
        "step" => {
            let l = p.spec().links[3].id;
            let t = if p.tick().is_multiple_of(2) {
                0.45
            } else {
                -0.38
            };
            p.advance_with_inputs_at(p.epoch(), p.tick(), &[(joint, t)], &[(l, [40., 10., 0.])])?
        }
        "transfer" => p.transfer((p.owner() + 1) % p.width())?,
        "queue" => p.queue_effort(joint, -0.1)?,
        "reset" => p.reset()?,
        _ => panic!("unknown robot crash-test input"),
    }
    Ok(())
}
#[test]
#[ignore = "subprocess for real robot page and commit SIGKILL"]
fn controller_at_native_page_or_commit() -> Test {
    let w = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let action = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let mut p = GroundedRobotPool::resume_with_encoding(
        Path::new(&w),
        Path::new(&dir),
        RobotGraphEncoding::FullPages,
    )?;
    input(&mut p, &action)?;
    panic!("controller missed the requested kill boundary")
}
#[test]
#[ignore = "requires previously qualified real loaded 30-DoF robot cut and managed worker"]
fn published_high_dof_pressure_holds_through_partial_pages_source_fence_and_rename() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&exe);
    let initial = fs::read(std::env::var("GZ_ROBOT_LOADED_CUT")?)?;
    for (action, edges) in [
        (
            "step",
            &[
                "robot-receipt-page",
                "robot-receipt-third-page",
                "robot-receipt-verified",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "queue",
            &["robot-receipt-third-page", "before-rename", "after-rename"][..],
        ),
        (
            "transfer",
            &[
                "robot-upload-page",
                "robot-upload-third-page",
                "robot-destination-staged",
                "robot-source-fenced",
                "robot-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &[
                "robot-destination-staged",
                "robot-source-fenced",
                "robot-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
    ] {
        for &edge in edges {
            let test = tempfile::tempdir()?;
            let reference = tempfile::tempdir()?;
            let barrier = test.path().join(FILE);
            let original = reference.path().join(FILE);
            fs::write(&barrier, &initial)?;
            fs::write(&original, &initial)?;
            let mut direct = GroundedRobotPool::resume_with_encoding(
                exe,
                reference.path(),
                RobotGraphEncoding::FullPages,
            )?;
            assert_eq!(
                (direct.tick(), direct.observation().joints.len()),
                (1050, 30)
            );
            assert_eq!(direct.ground_patches().len(), 2);
            terminate_probe(CHILD, test.path(), action, edge, direct.tick())?;
            let already = matches!(edge, "after-rename" | "after-sync");
            if already {
                input(&mut direct, action)?;
            }
            assert_eq!(fs::read(&barrier)?, fs::read(&original)?);
            let mut p = GroundedRobotPool::resume_with_encoding(
                exe,
                test.path(),
                RobotGraphEncoding::FullPages,
            )?;
            if !already {
                input(&mut p, action)?;
                input(&mut direct, action)?;
            }
            let mut support = 0;
            for _ in 0..64 {
                input(&mut p, "step")?;
                input(&mut direct, "step")?;
                assert_eq!(fs::read(&barrier)?, fs::read(&original)?);
                if p.ground_patches().len() >= 2 {
                    support += 1;
                }
            }
            assert!(support > 35);
            eprintln!(
                "action={action} edge={edge} already_published={already} exact_native_graph_and_archive=64 bilateral_or_greater={support}"
            );
        }
    }
    Ok(())
}

const BATCH_PROBE: &str = "grounded_robot::crash_tests::loaded_robot_batch_child";
fn batch_program() -> std::result::Result<RobotPostureProgram, Box<dyn Error>> {
    let xml = include_str!("../../../../examples/humanoid_stand.sdf");
    let spec = gz_sim_scene::distributed_hinge::distributed_grounded_robot_spec(
        xml,
        "robot.sdf",
        "default",
    )?;
    let sim = gz_sim_scene::load_world_with_sdf_step(
        xml,
        "robot.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::MultiFidelity,
    )?;
    let ctl = sim
        .world()
        .resource::<gz_sim_scene::joint_posture::JointPostures>()
        .get(spec.model_id)
        .unwrap();
    let chest = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == "default::humanoid::chest")
        })
        .unwrap()
        .get::<gz_sim_core::EntityId>()
        .unwrap()
        .get();
    Ok(RobotPostureProgram {
        law: ctl.law(),
        intervals: (0..16)
            .map(|i| RobotPostureInterval {
                targets: ctl.targets(),
                forces: vec![(chest, [140., i as f64 * 0.01, 0.])],
            })
            .collect(),
    })
}
#[test]
#[ignore = "real process entry for feedback, substep and commit SIGKILL"]
fn loaded_robot_batch_child() -> Test {
    let w = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let mut p = GroundedRobotPool::resume(Path::new(&w), Path::new(&dir))?;
    let start = p.tick();
    p.advance_posture_batch_at(0, start, &batch_program()?)?;
    assert_eq!(p.tick(), start + 16);
    Ok(())
}
#[test]
#[ignore = "requires qualified 1050ms robot archive and a barrier-trace worker"]
fn actual_seventh_native_substep_and_publication_expose_an_entire_batch_or_the_original() -> Test {
    use std::{
        process::Command,
        time::{Duration, Instant},
    };
    let w = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let w = Path::new(&w);
    let original = fs::read(std::env::var("GZ_ROBOT_LOADED_CUT")?)?;
    let program = batch_program()?;
    for edge in [
        "robot-packed-receipt",
        "robot-receipt-verified",
        "robot-posture-final-solver-cut",
        "robot-posture-proof-verified",
        "before-rename",
        "after-rename",
        "after-sync",
        "seven-worker",
        "seven-controller",
    ] {
        let dir = tempfile::tempdir()?;
        let ctrl = tempfile::tempdir()?;
        for d in [&dir, &ctrl] {
            fs::write(d.path().join(FILE), &original)?;
        }
        let mut reference = GroundedRobotPool::resume(w, ctrl.path())?;
        let mut already = matches!(edge, "after-rename" | "after-sync");
        if edge.starts_with("seven-") {
            let marker = dir.path().join("native-interval-seven");
            let mut child = Command::new(std::env::current_exe()?)
                .args(["--exact", BATCH_PROBE, "--ignored"])
                .env("GZ_FLOATING_TREE_TEST_SESSION", dir.path())
                .env("GZ_ROBOT_SUBSTEP_TRACE_MARKER", &marker)
                .spawn()?;
            let deadline = Instant::now() + Duration::from_secs(12);
            let pid = loop {
                if let Ok(row) = fs::read_to_string(&marker)
                    && let Some((pid, t)) = row.trim().split_once(' ')
                {
                    assert_eq!(t, "1057");
                    break pid.to_owned();
                }
                assert!(
                    child.try_wait()?.is_none(),
                    "batch child stopped before seventh native solve"
                );
                if Instant::now() > deadline {
                    child.kill()?;
                    child.wait()?;
                    panic!("no seventh native marker; build worker with barrier-trace");
                }
                std::thread::sleep(Duration::from_millis(2));
            };
            assert_eq!(fs::read(dir.path().join(FILE))?, original);
            if edge == "seven-controller" {
                child.kill()?;
                child.wait()?;
            }
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid])
                    .status()?
                    .success()
            );
            if edge == "seven-worker" {
                assert!(child.wait()?.success());
                already = true;
            }
        } else {
            terminate_probe(BATCH_PROBE, dir.path(), "batch", edge, 1050)?;
        }
        if already {
            reference.advance_posture_batch_at(0, 1050, &program)?;
        }
        assert_eq!(
            fs::read(dir.path().join(FILE))?,
            fs::read(ctrl.path().join(FILE))?
        );
        let mut p = GroundedRobotPool::resume(w, dir.path())?;
        if !already {
            p.advance_posture_batch_at(0, 1050, &program)?;
            reference.advance_posture_batch_at(0, 1050, &program)?;
        }
        assert_eq!(p.last_posture_batch(), reference.last_posture_batch());
        for _ in 0..8 {
            let start = p.tick();
            p.advance_posture_batch_at(0, start, &program)?;
            reference.advance_posture_batch_at(0, start, &program)?;
            assert_eq!(
                fs::read(dir.path().join(FILE))?,
                fs::read(ctrl.path().join(FILE))?
            );
        }
        eprintln!(
            "batch_fault={edge} whole_command_committed={already} identical_continuation_native_intervals=128 full_feedback_receipts={}",
            p.last_posture_batch().unwrap().receipt.intervals.len()
        );
    }
    Ok(())
}
