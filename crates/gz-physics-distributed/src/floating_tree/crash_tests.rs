// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::moving::crash_tests::await_boundary;
use std::{error::Error, fs, path::Path, process::Command};
type TestResult = std::result::Result<(), Box<dyn Error>>;
const ARCHIVE: &str = "barrier.gzfloatingtree";
const DRIVER: u64 = 91;
const OTHER: u64 = 92;

fn example() -> FloatingTreeSpec {
    let link = |id, position| ArticulationLinkSpec {
        id,
        radius: 0.1,
        mass: 1.5,
        position,
        orientation: [0., 0., 0., 1.],
        velocity: [0.05, -0.03, 0.08],
        angular_velocity: [0.; 3],
        inertia_center: [0.025, -0.005, 0.015],
        inertia_orientation: [0., 0., 0., 1.],
        inertia_tensor: [0.025, 0.032, 0.041, 0., 0., 0.],
    };
    let joint = |joint_id, parent_id, child_id, anchor, axis| FloatingTreeJointSpec {
        joint_id,
        parent_id,
        child_id,
        anchor,
        axis,
    };
    FloatingTreeSpec {
        model_id: 17,
        links: vec![
            link(31, [-0.3, 0., 0.]),
            link(36, [0., 0., 0.]),
            link(38, [0.3, 0., 0.]),
            link(40, [-0.3, 0.3, 0.]),
        ],
        joints: vec![
            joint(DRIVER, 36, 31, [-0.15, 0., 0.], [0., 0., 1.]),
            joint(OTHER, 36, 38, [0.15, 0., 0.], [0., 1., 0.]),
            joint(93, 31, 40, [-0.3, 0.15, 0.], [1., 0., 0.]),
        ],
    }
}

#[test]
fn reject_invalid_topologies_and_noncanonical_construction() {
    let original = example();
    assert!(original.root().is_ok());
    let mut invalid = Vec::new();
    let mut bad = original.clone();
    bad.links.swap(0, 1);
    invalid.push(bad);
    let mut bad = original.clone();
    bad.joints.reverse();
    invalid.push(bad);
    let mut bad = original.clone();
    bad.joints[0].parent_id = 40;
    invalid.push(bad);
    let mut bad = original.clone();
    bad.joints[1].child_id = 31;
    invalid.push(bad);
    let mut bad = original.clone();
    bad.joints[1].joint_id = 38;
    invalid.push(bad);
    let mut bad = original.clone();
    bad.joints[0].parent_id = 123456;
    invalid.push(bad);
    let mut bad = original.clone();
    bad.model_id = 31;
    invalid.push(bad);
    let mut bad = original.clone();
    bad.links.truncate(2);
    bad.joints.truncate(1);
    invalid.push(bad);
    for spec in invalid {
        assert!(spec.validate().is_err());
    }
}

fn inject(p: &mut FloatingTreePool, action: &str) -> Result<()> {
    match action {
        "advance" => p.advance_with_effort_at(0, 9, DRIVER, -0.035),
        "handoff" => p.transfer(2),
        "reset" => p.reset(),
        _ => panic!("unknown fault probe"),
    }
}
#[test]
#[ignore = "child of the controller process fault test"]
fn controller_process() -> TestResult {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let action = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let mut p = FloatingTreePool::resume(Path::new(&worker), Path::new(&dir))?;
    inject(&mut p, &action)?;
    panic!("floating tree coordinator missed configured fault boundary")
}

#[test]
#[ignore = "requires the absolute GZ_FLOATING_TREE_TEST_WORKER executable"]
fn floating_tree_controller_kill_preserves_multijoint_tree_and_guarded_continuation() -> TestResult
{
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let seed = tempfile::tempdir()?;
    let mut base = FloatingTreePool::create(exe, seed.path(), 3, example())?;
    for _ in 0..9 {
        base.advance_with_effort(DRIVER, 0.06)?;
    }
    base.queue_effort(OTHER, 0.014)?;
    base.queue_effort(DRIVER, -0.009)?;
    let original = fs::read(seed.path().join(ARCHIVE))?;
    drop(base);
    for (action, points) in [
        (
            "advance",
            &[
                "tree-worker-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "handoff",
            &[
                "tree-destination-staged",
                "tree-source-fenced",
                "tree-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &["before-rename", "after-rename", "after-sync"][..],
        ),
    ] {
        for &edge in points {
            let run = tempfile::tempdir()?;
            let control = tempfile::tempdir()?;
            for dir in [&run, &control] {
                fs::write(dir.path().join(ARCHIVE), &original)?;
            }
            let marker = run.path().join("kill-point");
            let mut child = Command::new(std::env::current_exe()?)
                .args([
                    "--exact",
                    "--ignored",
                    "floating_tree::crash_tests::controller_process",
                ])
                .env("GZ_FLOATING_TREE_TEST_SESSION", run.path())
                .env("GZ_FLOATING_TREE_TEST_ACTION", action)
                .env("GZ_MOVING_TEST_MARKER", &marker)
                .env("GZ_MOVING_TEST_GATE", edge)
                .spawn()?;
            await_boundary(&mut child, &marker, edge)?;
            child.kill()?;
            child.wait()?;
            let mut recovered = FloatingTreePool::resume(exe, run.path())?;
            let mut reference = FloatingTreePool::resume(exe, control.path())?;
            let published = matches!(edge, "after-rename" | "after-sync");
            if published {
                inject(&mut reference, action)?;
            }
            assert_eq!(recovered.observation(), reference.observation());
            assert_eq!(
                fs::read(run.path().join(ARCHIVE))?,
                fs::read(control.path().join(ARCHIVE))?
            );
            if !published {
                inject(&mut recovered, action)?;
                inject(&mut reference, action)?;
            } else if action == "advance" {
                assert!(matches!(
                    inject(&mut recovered, action),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            for n in 0..4 {
                let j = if n % 2 == 0 { DRIVER } else { OTHER };
                recovered.advance_with_effort(j, 0.017)?;
                reference.queue_effort(j, 0.017)?;
                reference.advance()?;
                assert_eq!(
                    fs::read(run.path().join(ARCHIVE))?,
                    fs::read(control.path().join(ARCHIVE))?
                );
            }
            eprintln!(
                "recovered action={action} edge={edge} published={published} epoch={} tick={} owner={}",
                recovered.epoch(),
                recovered.observation().tick,
                recovered.owner()
            );
        }
    }
    Ok(())
}

#[test]
fn native_snapshots_bind_the_recipe_tick_observation_and_every_held_joint() -> TestResult {
    use super::native::TreeWorld;
    let spec = example();
    let mut native = TreeWorld::new(&spec)?;
    native.effort(OTHER, 0.)?;
    native.effort(DRIVER, -0.006)?;
    for n in 0..14 {
        let snapshot = native.capture()?;
        assert_eq!(snapshot.state.tick, n);
        let mut restored = TreeWorld::restore(&spec, &snapshot)?;
        assert!(snapshot == restored.capture()?);
        let mut wrong = snapshot.clone();
        wrong.state.links[0].velocity[1] += 0.5;
        assert!(TreeWorld::restore(&spec, &wrong).is_err());
        let mut wrong = snapshot.clone();
        wrong.digest[12] ^= 1;
        assert!(TreeWorld::restore(&spec, &wrong).is_err());
        let mut wrong = snapshot.clone();
        wrong.state.tick += 1;
        assert!(TreeWorld::restore(&spec, &wrong).is_err());
        let mut wrong = spec.clone();
        wrong.model_id += 1;
        assert!(TreeWorld::restore(&wrong, &snapshot).is_err());
        for world in [&mut native, &mut restored] {
            world.effort(93, if n % 2 == 0 { 0.03 } else { -0.022 })?;
            world.advance()?;
            world.effort(OTHER, 0.005)?;
        }
        assert!(native.capture()? == restored.capture()?);
    }
    Ok(())
}

pub(super) fn kill_partial_owner_once(pid: u32) {
    kill_owner_at_marker("GZ_TREE_VECTOR_KILL_ONCE",pid);
}
pub(super) fn kill_owner_at_marker(key:&str,pid:u32){
    let Some(marker) = std::env::var_os(key) else {
        return;
    };
    match fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(marker)
    {
        Ok(_) => {
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()
                    .unwrap()
                    .success()
            );
        }
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
        Err(e) => panic!("create owner-kill marker: {e}"),
    }
}
fn apply_vector(p: &mut FloatingTreePool) -> Result<()> {
    p.advance_with_efforts_at(0, 9, &[(OTHER, 0.012), (93, -0.022), (DRIVER, -0.035)])
}
fn apply_separately(p: &mut FloatingTreePool) -> Result<()> {
    p.queue_effort(OTHER, 0.012)?;
    p.queue_effort(93, -0.022)?;
    p.queue_effort(DRIVER, -0.035)?;
    p.advance()
}
#[test]
#[ignore = "child of the vector fault harness"]
fn vector_controller_process() -> TestResult {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let mut p = FloatingTreePool::resume(Path::new(&worker), Path::new(&dir))?;
    let original_owner = p.worker_pid(p.owner());
    apply_vector(&mut p)?;
    if std::env::var_os("GZ_TREE_VECTOR_KILL_ONCE").is_some() {
        assert_ne!(p.worker_pid(p.owner()), original_owner);
        return Ok(());
    }
    panic!("vector coordinator missed fault boundary")
}
#[test]
#[ignore = "requires current and original v1 floating tree worker executables"]
fn vector_controller_and_legacy_partial_owner_kills_recover_exactly() -> TestResult {
    let current = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let old = std::env::var("GZ_FLOATING_TREE_SCALAR_WORKER")?;
    let seed = tempfile::tempdir()?;
    let mut base = FloatingTreePool::create(Path::new(&current), seed.path(), 3, example())?;
    for _ in 0..9 {
        base.advance_with_effort(OTHER, 0.009)?;
    }
    base.queue_effort(DRIVER, -0.015)?;
    base.queue_effort(93, 0.0)?;
    base.transfer(1)?;
    let bytes = fs::read(seed.path().join(ARCHIVE))?;
    drop(base);
    let reference = tempfile::tempdir()?;
    fs::write(reference.path().join(ARCHIVE), &bytes)?;
    let mut control = FloatingTreePool::resume(Path::new(&current), reference.path())?;
    apply_separately(&mut control)?;
    let advanced = fs::read(reference.path().join(ARCHIVE))?;
    control.advance_with_effort(OTHER, -0.002)?;
    let continued = fs::read(reference.path().join(ARCHIVE))?;
    let legacy_extra = ["tree-vector-partial", "legacy-owner-died"];
    for (exe, profile, extra) in [
        (&current, FloatingTreeWorkerProfile::VectorV2, &[][..]),
        (&old, FloatingTreeWorkerProfile::ScalarV1, &legacy_extra[..]),
    ] {
        for &edge in [
            "tree-worker-prepared",
            "before-rename",
            "after-rename",
            "after-sync",
        ]
        .iter()
        .chain(extra)
        {
            let dir = tempfile::tempdir()?;
            fs::write(dir.path().join(ARCHIVE), &bytes)?;
            let marker = dir.path().join("fault-marker");
            let worker_death = edge == "legacy-owner-died";
            let mut launch = Command::new(std::env::current_exe()?);
            launch
                .args([
                    "--exact",
                    "--ignored",
                    "floating_tree::crash_tests::vector_controller_process",
                ])
                .env("GZ_FLOATING_TREE_TEST_WORKER", exe)
                .env("GZ_FLOATING_TREE_TEST_SESSION", dir.path());
            if worker_death {
                assert!(
                    launch
                        .env("GZ_TREE_VECTOR_KILL_ONCE", &marker)
                        .status()?
                        .success()
                );
                assert!(marker.is_file());
            } else {
                let mut child = launch
                    .env("GZ_MOVING_TEST_GATE", edge)
                    .env("GZ_MOVING_TEST_MARKER", &marker)
                    .spawn()?;
                await_boundary(&mut child, &marker, edge)?;
                child.kill()?;
                child.wait()?;
            }
            let published = worker_death || matches!(edge, "after-rename" | "after-sync");
            let mut recovered = FloatingTreePool::resume(Path::new(exe), dir.path())?;
            assert_eq!(recovered.worker_profile(), profile);
            assert_eq!(
                fs::read(dir.path().join(ARCHIVE))?,
                if published { &advanced } else { &bytes }.as_slice()
            );
            if published {
                assert!(matches!(
                    apply_vector(&mut recovered),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            } else {
                apply_vector(&mut recovered)?;
            }
            assert_eq!(fs::read(dir.path().join(ARCHIVE))?, advanced);
            recovered.advance_with_effort(OTHER, -0.002)?;
            assert_eq!(fs::read(dir.path().join(ARCHIVE))?, continued);
            eprintln!(
                "recovered profile={profile:?} edge={edge} worker_kill={worker_death} committed={published}"
            );
        }
    }
    Ok(())
}

fn stopped_example() -> StoppedTreeSpec {
    StoppedTreeSpec {
        tree: example(),
        limits: vec![
            FloatingTreeJointLimit {
                joint_id: DRIVER,
                lower: -0.031,
                upper: 0.049,
            },
            FloatingTreeJointLimit {
                joint_id: 93,
                lower: -0.,
                upper: 0.,
            },
        ],
    }
}
#[test]
fn stopped_recipes_refuse_unknown_or_duplicated_limits_and_accept_a_locked_axis() -> TestResult {
    use super::native::TreeWorld;
    let spec = stopped_example();
    spec.validate()?;
    for limits in [
        vec![],
        vec![FloatingTreeJointLimit {
            joint_id: DRIVER,
            lower: 0.1,
            upper: 0.5,
        }],
        vec![FloatingTreeJointLimit {
            joint_id: OTHER,
            lower: f64::NEG_INFINITY,
            upper: 0.5,
        }],
        vec![FloatingTreeJointLimit {
            joint_id: 42,
            lower: 0.,
            upper: 0.3,
        }],
        vec![spec.limits[1], spec.limits[0]],
        vec![spec.limits[0], spec.limits[0]],
    ] {
        assert!(
            StoppedTreeSpec {
                tree: spec.tree.clone(),
                limits
            }
            .validate()
            .is_err()
        );
    }
    let mut native = TreeWorld::new(&spec)?;
    for _ in 0..64 {
        native.effort(93, 0.32)?;
        native.advance()?;
    }
    let snap = native.capture()?;
    assert!(
        snap.state
            .joints
            .iter()
            .find(|j| j.joint_id == 93)
            .unwrap()
            .q
            .abs()
            < 1e-5
    );
    assert!(TreeWorld::restore(&spec.tree, &snap).is_err());
    let mut altered = spec.clone();
    altered.limits[0].lower -= 0.001;
    assert!(TreeWorld::restore(&altered, &snap).is_err());
    assert!(snap == TreeWorld::restore(&spec, &snap)?.capture()?);
    Ok(())
}
fn stopped_operation(p: &mut StoppedTreePool, action: &str, expected_tick: u64) -> Result<()> {
    match action {
        "reverse" => p.advance_with_efforts_at(0, expected_tick, &[(DRIVER, -2.1), (OTHER, 0.017)]),
        "transfer" => p.transfer(2),
        "reset" => p.reset(),
        _ => panic!("unknown stopped-tree fault operation"),
    }
}
#[test]
#[ignore = "child of the stopped-tree controller fault harness"]
fn stopped_controller_process() -> TestResult {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_STOP_TEST_TICK")?.parse()?;
    let mut p = StoppedTreePool::resume(Path::new(&exe), Path::new(&dir))?;
    stopped_operation(&mut p, &op, tick)?;
    panic!("stopped-tree coordinator missed its configured crash window")
}
#[test]
#[ignore = "requires a worker implementing GZFSWK01"]
fn stopped_tree_controller_kills_retain_loaded_stop_and_continue_through_release() -> TestResult {
    let path = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&path);
    let seed = tempfile::tempdir()?;
    let spec = stopped_example();
    let mut initial = StoppedTreePool::create(exe, seed.path(), 3, spec.clone())?;
    let goal = spec.limits[0].upper;
    let mut held = 0;
    for _ in 0..400 {
        initial.advance_with_effort(DRIVER, 2.5)?;
        let s = initial
            .observation()
            .joints
            .iter()
            .find(|j| j.joint_id == DRIVER)
            .unwrap();
        if (s.q - goal).abs() < 0.0005 && s.qdot.abs() < 0.015 {
            held += 1;
        } else {
            held = 0;
        }
        if held >= 6 {
            break;
        }
    }
    assert!(held >= 6, "could not seed a loaded upper stop");
    initial.queue_effort(DRIVER, 0.035)?;
    initial.queue_effort(93, 0.0)?;
    let tick = initial.observation().tick;
    let archive = "barrier.gzstoppedtree";
    let original = fs::read(seed.path().join(archive))?;
    drop(initial);
    assert!(FloatingTreePool::resume(exe, seed.path()).is_err());
    for (op, points) in [
        (
            "reverse",
            &[
                "tree-worker-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "transfer",
            &[
                "tree-destination-staged",
                "tree-source-fenced",
                "tree-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &["before-rename", "after-rename", "after-sync"][..],
        ),
    ] {
        for &edge in points {
            let a = tempfile::tempdir()?;
            let b = tempfile::tempdir()?;
            for dir in [&a, &b] {
                fs::write(dir.path().join(archive), &original)?;
            }
            let marker = a.path().join("stop-marker");
            let mut child = Command::new(std::env::current_exe()?)
                .args([
                    "--exact",
                    "--ignored",
                    "floating_tree::crash_tests::stopped_controller_process",
                ])
                .env("GZ_FLOATING_TREE_TEST_SESSION", a.path())
                .env("GZ_FLOATING_TREE_TEST_ACTION", op)
                .env("GZ_STOP_TEST_TICK", tick.to_string())
                .env("GZ_MOVING_TEST_GATE", edge)
                .env("GZ_MOVING_TEST_MARKER", &marker)
                .spawn()?;
            await_boundary(&mut child, &marker, edge)?;
            child.kill()?;
            child.wait()?;
            let mut recovered = StoppedTreePool::resume(exe, a.path())?;
            let mut control = StoppedTreePool::resume(exe, b.path())?;
            let published = matches!(edge, "after-rename" | "after-sync");
            if published {
                stopped_operation(&mut control, op, tick)?;
            }
            assert_eq!(
                fs::read(a.path().join(archive))?,
                fs::read(b.path().join(archive))?
            );
            if !published {
                stopped_operation(&mut recovered, op, tick)?;
                stopped_operation(&mut control, op, tick)?;
            } else if op == "reverse" {
                assert!(matches!(
                    stopped_operation(&mut recovered, op, tick),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            for _ in 0..60 {
                recovered.advance_with_efforts(&[(DRIVER, -2.4), (OTHER, 0.1)])?;
                control.queue_effort(DRIVER, -2.4)?;
                control.queue_effort(OTHER, 0.1)?;
                control.advance()?;
                assert_eq!(
                    fs::read(a.path().join(archive))?,
                    fs::read(b.path().join(archive))?
                );
            }
            let angle = recovered
                .observation()
                .joints
                .iter()
                .find(|j| j.joint_id == DRIVER)
                .unwrap()
                .q;
            assert!(angle < goal - 0.01);
            eprintln!(
                "recovered stop op={op} edge={edge} published={published} seed_tick={tick} continuation_angle={angle}"
            );
        }
    }
    Ok(())
}

fn motor_operation(p: &mut MotorTreePool, op: &str) -> Result<()> {
    use gz_physics::JointVelocityMotor;
    match op {
        "set" => p.set_velocity_motor(
            DRIVER,
            Some(JointVelocityMotor {
                target_velocity: -1.5,
                max_force: 0.6,
            }),
        ),
        "clear" => p.set_velocity_motor(OTHER, None),
        "transfer" => p.transfer(2),
        "advance" => p.advance_with_efforts_at(0, 12, &[(DRIVER, 0.031), (OTHER, -0.002)]),
        "reset" => p.reset(),
        _ => panic!("unknown motor fault probe"),
    }
}
#[test]
#[ignore = "child of the persistent motor fault harness"]
fn motor_controller_process() -> TestResult {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let mut p = MotorTreePool::resume(Path::new(&exe), Path::new(&dir))?;
    motor_operation(&mut p, &op)?;
    panic!("motor coordinator missed configured fault boundary")
}
#[test]
#[ignore = "requires the motor-aware worker executable"]
fn motorized_tree_controller_kills_preserve_persistent_set_clear_and_actuation() -> TestResult {
    use gz_physics::JointVelocityMotor;
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let seed = tempfile::tempdir()?;
    let stopped = stopped_example();
    let spec = MotorTreeSpec {
        tree: stopped.tree,
        limits: stopped.limits,
    };
    let mut base = MotorTreePool::create(exe, seed.path(), 3, spec)?;
    base.set_velocity_motor(
        DRIVER,
        Some(JointVelocityMotor {
            target_velocity: 2.05,
            max_force: 1.1,
        }),
    )?;
    base.set_velocity_motor(
        OTHER,
        Some(JointVelocityMotor {
            target_velocity: -0.5,
            max_force: 0.45,
        }),
    )?;
    for _ in 0..12 {
        base.advance()?;
    }
    base.queue_effort(DRIVER, 0.019)?;
    let archive = "barrier.gzmotorizedtree";
    let original = fs::read(seed.path().join(archive))?;
    drop(base);
    assert!(StoppedTreePool::resume(exe, seed.path()).is_err());
    for (op, points) in [
        (
            "set",
            &[
                "tree-motor-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "clear",
            &[
                "tree-motor-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "transfer",
            &[
                "tree-destination-staged",
                "tree-source-fenced",
                "tree-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "advance",
            &[
                "tree-worker-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &["before-rename", "after-rename", "after-sync"][..],
        ),
    ] {
        for &edge in points {
            let a = tempfile::tempdir()?;
            let b = tempfile::tempdir()?;
            for dir in [&a, &b] {
                fs::write(dir.path().join(archive), &original)?;
            }
            let marker = a.path().join("motor-probe");
            let mut child = Command::new(std::env::current_exe()?)
                .args([
                    "--ignored",
                    "--exact",
                    "floating_tree::crash_tests::motor_controller_process",
                ])
                .env("GZ_FLOATING_TREE_TEST_SESSION", a.path())
                .env("GZ_FLOATING_TREE_TEST_ACTION", op)
                .env("GZ_MOVING_TEST_GATE", edge)
                .env("GZ_MOVING_TEST_MARKER", &marker)
                .spawn()?;
            await_boundary(&mut child, &marker, edge)?;
            child.kill()?;
            child.wait()?;
            let published = matches!(edge, "after-rename" | "after-sync");
            let mut p = MotorTreePool::resume(exe, a.path())?;
            let mut control = MotorTreePool::resume(exe, b.path())?;
            if published {
                motor_operation(&mut control, op)?;
            }
            for j in [DRIVER, OTHER, 93] {
                assert_eq!(p.velocity_motor(j), control.velocity_motor(j));
            }
            assert_eq!(
                fs::read(a.path().join(archive))?,
                fs::read(b.path().join(archive))?
            );
            if !published {
                motor_operation(&mut p, op)?;
                motor_operation(&mut control, op)?;
            } else if matches!(op, "set" | "clear") {
                // A repeated replacement does not advance the clock or apply a second impulse.
                motor_operation(&mut p, op)?;
                assert_eq!(
                    fs::read(a.path().join(archive))?,
                    fs::read(b.path().join(archive))?
                );
            } else if op == "advance" {
                assert!(matches!(
                    motor_operation(&mut p, op),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            for _ in 0..8 {
                p.advance_with_efforts(&[(OTHER, -0.08), (DRIVER, 0.006)])?;
                control.queue_effort(OTHER, -0.08)?;
                control.queue_effort(DRIVER, 0.006)?;
                control.advance()?;
                assert_eq!(
                    fs::read(a.path().join(archive))?,
                    fs::read(b.path().join(archive))?
                );
            }
            eprintln!("recovered motor op={op} edge={edge} published={published}");
        }
    }
    Ok(())
}

fn grounded_example() -> GroundedTreeSpec {
    let stopped = stopped_example();
    GroundedTreeSpec {
        tree: stopped.tree,
        limits: stopped.limits,
        ground: crate::articulation::ArticulationGroundSpec {
            link_id: 4,
            height: -0.101,
            size: [20.; 2],
            gravity_z: -9.81,
        },
    }
}

#[test]
fn grounded_tree_rejects_foreign_bounds_and_reports_a_cold_floor() -> TestResult {
    use super::native::TreeWorld;
    let spec = grounded_example();
    let native = TreeWorld::new(&spec)?;
    let cold = native.capture()?;
    let reactions = cold.image.reactions().unwrap();
    assert_eq!(reactions.len(), spec.tree.links.len());
    assert!(reactions.iter().all(|r| !r.active
        && r.normal_impulse.to_bits() == 0
        && r.warmstart_normal_impulse.to_bits() == 0));
    for g in [
        crate::articulation::ArticulationGroundSpec {
            link_id: spec.tree.links[1].id,
            ..spec.ground
        },
        crate::articulation::ArticulationGroundSpec {
            link_id: spec.tree.links[1].id + 1,
            ..spec.ground
        },
        crate::articulation::ArticulationGroundSpec {
            gravity_z: 0.,
            ..spec.ground
        },
        crate::articulation::ArticulationGroundSpec {
            height: f64::NAN,
            ..spec.ground
        },
        crate::articulation::ArticulationGroundSpec {
            size: [0., 1.],
            ..spec.ground
        },
    ] {
        assert!(
            GroundedTreeSpec {
                ground: g,
                ..spec.clone()
            }
            .validate()
            .is_err()
        );
    }
    let mut alternate = spec.clone();
    alternate.ground.height += 0.001;
    assert!(TreeWorld::restore(&alternate, &cold).is_err());
    assert!(TreeWorld::restore(&spec.tree, &cold).is_err());
    let mut oversized = spec;
    while oversized.tree.links.len() < 9 {
        let old = oversized.tree.links.last().unwrap().id;
        oversized.tree.links.push(ArticulationLinkSpec {
            id: old + 1,
            ..*oversized.tree.links.last().unwrap()
        });
        oversized.tree.joints.push(FloatingTreeJointSpec {
            joint_id: 1000 + old,
            parent_id: old,
            child_id: old + 1,
            anchor: [0.; 3],
            axis: [0., 0., 1.],
        });
    }
    assert!(oversized.validate().is_err());
    Ok(())
}
fn ground_operation(p: &mut GroundedTreePool, op: &str, tick: u64) -> Result<()> {
    match op {
        "advance" => p.advance_with_efforts_at(0, tick, &[(DRIVER, 0.73), (OTHER, -0.55)]),
        "transfer" => p.transfer(2),
        "reset" => p.reset(),
        _ => panic!("unknown grounded-tree operation"),
    }
}
#[test]
#[ignore = "child of the grounded-tree fault harness"]
fn grounded_controller_process() -> TestResult {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = GroundedTreePool::resume(Path::new(&exe), Path::new(&dir))?;
    ground_operation(&mut p, &op, tick)?;
    panic!("grounded coordinator missed fault boundary")
}
pub(crate) fn terminate_probe(name: &str, dir: &Path, op: &str, edge: &str, tick: u64) -> TestResult {
    let marker = dir.join("controller-probe");
    let mut c = Command::new(std::env::current_exe()?)
        .args(["--exact", "--ignored", name])
        .env("GZ_FLOATING_TREE_TEST_SESSION", dir)
        .env("GZ_FLOATING_TREE_TEST_ACTION", op)
        .env("GZ_TREE_PROBE_TICK", tick.to_string())
        .env("GZ_MOVING_TEST_GATE", edge)
        .env("GZ_MOVING_TEST_MARKER", &marker)
        .spawn()?;
    await_boundary(&mut c, &marker, edge)?;
    c.kill()?;
    c.wait()?;
    Ok(())
}
#[test]
#[ignore = "requires GZFGWK01 worker implementing full grounded native tree checkpoints"]
fn grounded_tree_controller_kills_restore_warm_contact_graph_and_exact_continuation() -> TestResult
{
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let seed = tempfile::tempdir()?;
    let mut p = GroundedTreePool::create(exe, seed.path(), 3, grounded_example())?;
    let mut loaded = 0;
    for _ in 0..380 {
        p.advance()?;
        loaded = p
            .ground_reactions()
            .iter()
            .filter(|r| r.active && r.warmstart_normal_impulse > 0. && r.normal_impulse > 0.)
            .count();
        if loaded >= 3 {
            break;
        }
    }
    assert!(loaded >= 3);
    p.queue_effort(DRIVER, 0.11)?;
    p.queue_effort(93, -0.06)?;
    let archive = "barrier.gzgroundedtree";
    let original = fs::read(seed.path().join(archive))?;
    let tick = p.observation().tick;
    drop(p);
    assert!(StoppedTreePool::resume(exe, seed.path()).is_err());
    assert!(MotorTreePool::resume(exe, seed.path()).is_err());
    external_fault_matrix::<GroundedTreeSpec>(
        exe,
        archive,
        &original,
        tick,
        loaded,
        "floating_tree::crash_tests::grounded_controller_process",
        ground_operation,
        &[(DRIVER, 0.63), (OTHER, 0.23)],
        TreePool::advance_with_efforts,
    )
}

fn payload_drive(spec: &PayloadTreeSpec, direction: f64) -> Vec<(u64, f64)> {
    spec.tree
        .joints
        .iter()
        .map(|j| {
            let child = spec.tree.links.iter().find(|l| l.id == j.child_id).unwrap();
            (j.joint_id, -child.position[1].signum() * direction)
        })
        .collect()
}
fn payload_operation(p: &mut PayloadTreePool, op: &str, tick: u64) -> Result<()> {
    match op {
        "advance" => p.advance_with_efforts_at(0, tick, &payload_drive(p.spec(), -0.8)),
        "transfer" => p.transfer(2),
        "reset" => p.reset(),
        _ => panic!("unknown payload tree operation"),
    }
}
#[test]
#[ignore = "child of the payload tree crash harness"]
fn payload_controller_process() -> TestResult {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = PayloadTreePool::resume(Path::new(&exe), Path::new(&dir))?;
    payload_operation(&mut p, &op, tick)?;
    panic!("payload controller missed configured fault boundary")
}
#[test]
#[ignore = "requires the managed payload worker executable"]
fn payload_controller_kills_restore_two_simultaneous_warm_contacts_and_exact_release() -> TestResult
{
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let xml = include_str!("../../../../examples/distributed_gripper_payload.sdf");
    let authored = gz_sim_scene::distributed_payload_tree_spec(
        xml,
        "distributed_gripper_payload.sdf",
        "default",
    )?;
    let spec: PayloadTreeSpec = serde_json::from_value(serde_json::to_value(&authored)?)?;
    let seed = tempfile::tempdir()?;
    let mut p = PayloadTreePool::create(exe, seed.path(), 3, spec.clone())?;
    let closing = payload_drive(&spec, 0.8);
    let opening = payload_drive(&spec, -0.8);
    let mut loaded = 0;
    for _ in 0..450 {
        p.advance_with_efforts(&closing)?;
        loaded = p
            .payload_reactions()
            .iter()
            .filter(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
            .count();
        if loaded >= 2 {
            break;
        }
    }
    assert!(loaded >= 2);
    p.queue_effort(closing[0].0, 0.037)?;
    p.queue_effort(closing[1].0, -0.005)?;
    let archive = "barrier.gzpayloadtree";
    let original = fs::read(seed.path().join(archive))?;
    let tick = p.observation().tick;
    drop(p);
    assert!(GroundedTreePool::resume(exe, seed.path()).is_err());
    external_fault_matrix::<PayloadTreeSpec>(
        exe,
        archive,
        &original,
        tick,
        loaded,
        "floating_tree::crash_tests::payload_controller_process",
        payload_operation,
        &opening,
        TreePool::advance_with_efforts,
    )
}

#[allow(clippy::too_many_arguments)]
fn external_fault_matrix<S: TreeConstruction>(
    exe: &Path,
    archive: &str,
    original: &[u8],
    tick: u64,
    loaded: usize,
    child: &str,
    apply: fn(&mut TreePool<S>, &str, u64) -> Result<()>,
    continuation: &[(u64, f64)],
    advance: impl Fn(&mut TreePool<S>, &[(u64, f64)]) -> Result<()>,
) -> TestResult {
    let mut cases = vec![
        (
            "advance",
            &[
                "tree-worker-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "transfer",
            &[
                "tree-destination-staged",
                "tree-source-fenced",
                "tree-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &["before-rename", "after-rename", "after-sync"][..],
        ),
    ];
    if S::MOTORIZED {
        for op in ["set", "clear"] {
            cases.push((
                op,
                &[
                    "tree-motor-prepared",
                    "before-rename",
                    "after-rename",
                    "after-sync",
                ][..],
            ));
        }
    }
    for (op, edges) in cases {
        for &edge in edges {
            let a = tempfile::tempdir()?;
            let b = tempfile::tempdir()?;
            for d in [&a, &b] {
                fs::write(d.path().join(archive), original)?;
            }
            terminate_probe(child, a.path(), op, edge, tick)?;
            let published = matches!(edge, "after-rename" | "after-sync");
            let mut recovered = TreePool::<S>::resume(exe, a.path())?;
            let mut control = TreePool::<S>::resume(exe, b.path())?;
            if published {
                apply(&mut control, op, tick)?;
            }
            assert_eq!(recovered.observation(), control.observation());
            assert_eq!(recovered.image(), control.image());
            assert_eq!(
                fs::read(a.path().join(archive))?,
                fs::read(b.path().join(archive))?
            );
            if !published {
                apply(&mut recovered, op, tick)?;
                apply(&mut control, op, tick)?;
            } else if op == "advance" {
                assert!(matches!(
                    apply(&mut recovered, op, tick),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            for _ in 0..32 {
                advance(&mut recovered, continuation)?;
                for &(id, effort) in continuation {
                    control.queue_effort(id, effort)?;
                }
                advance(&mut control, &[])?;
                assert_eq!(recovered.observation(), control.observation());
                assert_eq!(recovered.image(), control.image());
                assert_eq!(
                    fs::read(a.path().join(archive))?,
                    fs::read(b.path().join(archive))?
                );
            }
            eprintln!(
                "recovered {archive} op={op} edge={edge} published={published} loaded_seed={loaded} tick={tick}"
            );
        }
    }
    Ok(())
}

fn grasp_operation(p: &mut GraspTreePool, op: &str, tick: u64) -> Result<()> {
    let id = p.topology().joints[0].joint_id;
    match op {
        "advance" => p.advance_with_payload_force_at(0, tick, &[(id, -0.1)], [0.15, -0.2, 1.2]),
        "set" => p.set_velocity_motor(
            id,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: -6.,
                max_force: 2.2,
            }),
        ),
        "clear" => p.set_velocity_motor(id, None),
        "transfer" => p.transfer(2),
        "reset" => p.reset(),
        _ => panic!("unrecognized grasp operation"),
    }
}
#[test]
#[ignore = "child of the loaded grasp controller fault harness"]
fn grasp_controller_process() -> TestResult {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = GraspTreePool::resume(Path::new(&exe), Path::new(&dir))?;
    grasp_operation(&mut p, &op, tick)?;
    panic!("grasp controller missed configured process boundary")
}
#[test]
#[ignore = "requires the GZFRWK01 native grasp worker"]
fn grasp_controller_kills_restore_an_externally_loaded_part_and_capped_motors() -> TestResult {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let xml = include_str!("../../../../examples/distributed_gripper_payload.sdf");
    let external = gz_sim_scene::distributed_payload_tree_spec(
        xml,
        "distributed_gripper_payload.sdf",
        "default",
    )?;
    let payload: PayloadTreeSpec = serde_json::from_value(serde_json::to_value(&external)?)?;
    let pdir = tempfile::tempdir()?;
    let mut p = GraspTreePool::create(
        exe,
        pdir.path(),
        3,
        GraspTreeSpec {
            payload: payload.clone(),
        },
    )?;
    for (id, velocity) in payload_drive(&payload, 6.) {
        p.set_velocity_motor(
            id,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: velocity,
                max_force: 4.,
            }),
        )?;
    }
    for tick in 0..675 {
        let force = if tick >= 650 { [0., 0., 1.2] } else { [0.; 3] };
        p.advance_with_payload_force(&[], force)?;
    }
    let loaded = p
        .payload_reactions()
        .iter()
        .filter(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
        .count();
    assert_eq!(loaded, 2);
    let inputs = payload_drive(&payload, 0.012);
    for (id, e) in &inputs {
        p.queue_effort(*id, *e)?;
    }
    let archive = "barrier.gzgrasptree";
    let original = fs::read(pdir.path().join(archive))?;
    let tick = p.observation().tick;
    drop(p);
    assert!(PayloadTreePool::resume(exe, pdir.path()).is_err());
    external_fault_matrix::<GraspTreeSpec>(
        exe,
        archive,
        &original,
        tick,
        loaded,
        "floating_tree::crash_tests::grasp_controller_process",
        grasp_operation,
        &inputs,
        |p, commands| p.advance_with_payload_force(commands, [0., 0., 1.2]),
    )
}
