// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::tests::native;
use super::*;
use gz_physics::DynamicBackend;
use gz_sim_scene::distributed_hinge::GroundedRobotPool;
use std::{fs, path::Path, process::Command};

#[path = "humanoid_return_cli.rs"]
mod returned_cli;

fn verify_native(a: &Simulation, b: &Simulation, stand: &Stand) -> Result<()> {
    let here = a.world().resource::<PhysicsBackend>();
    let other = b.world().resource::<PhysicsBackend>();
    assert_eq!(
        a.committed_update_info()?.iteration,
        b.committed_update_info()?.iteration
    );
    assert_eq!(
        a.committed_update_info()?.epoch,
        b.committed_update_info()?.epoch
    );
    for &id in &stand.segments {
        assert_eq!(here.state(id), other.state(id));
        assert_eq!(
            native(a).ground_reaction(id, stand.floor)?,
            native(b).ground_reaction(id, stand.floor)?
        );
    }
    for axis in &stand.axes {
        assert_eq!(here.joint_state(axis.joint), other.joint_state(axis.joint));
        assert_eq!(
            native(a).pending_joint_effort(axis.joint),
            native(b).pending_joint_effort(axis.joint)
        );
    }
    assert_eq!(
        native(a).fixed_ground_patches(stand.floor)?,
        native(b).fixed_ground_patches(stand.floor)?
    );
    assert_eq!(
        native(a).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
        native(b).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
    );
    Ok(())
}

#[test]
#[ignore = "requires the real grounded robot process"]
fn loaded_robot_returns_to_native_fences_workers_and_recovers_held_input() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing real worker")?);
    let args = Args::try_parse_from([
        "humanoid_stand",
        "--authored-footsteps",
        "--author-disturbance",
        "--recovery-landing",
        "--motion",
        "step",
        "--push-x",
        "0",
        "--push-y=-60",
        "--push-at",
        "3380",
        "--push-ticks",
        "80",
    ])?;
    let xml = footstep_sdf(&args)?;
    let ordinary = || {
        gz_sim_scene::load_world_with_sdf_step(
            &xml,
            "robot.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    let reopen = |dir: &Path| load_returned_grounded_robot_world(&xml, "robot.sdf", "default", dir);
    let mut local = ordinary()?;
    let mut moved = ordinary()?;
    let stand = Stand::resolve(&mut local, &args)?;
    let returned = Stand::resolve(&mut moved, &args)?;
    moved.world_mut().insert_resource(returned);
    let mut frames = std::env::var_os("GZ_ROBOT_RETURN_TRACE")
        .map(File::create)
        .transpose()?
        .map(BufWriter::new);
    let mut record = |sim: &Simulation, tick: u64| -> Result<()> {
        if let Some(out) = &mut frames {
            let poses = tree_visuals::poses(sim)?.into_values().collect::<Vec<_>>();
            serde_json::to_writer(&mut *out, &serde_json::json!({"tick":tick,"visuals":poses}))?;
            writeln!(out)?;
        }
        Ok(())
    };
    // Start in a nonzero epoch and retain the reset reference for the physical viewer.
    gz_sim_scene::reset_loaded_world(&mut local, |_| ())?;
    gz_sim_scene::reset_loaded_world(&mut moved, |_| ())?;
    record(&local, 0)?;
    for tick in 1..=160 {
        local.step_exact(1)?;
        record(&local, tick)?;
    }
    local.step_exact(3440 - 160)?;
    moved.step_exact(3440)?;
    let dir = tempfile::tempdir()?;
    handoff_grounded_robot_to_workers(
        &mut moved,
        &xml,
        "robot.sdf",
        "default",
        &exe,
        dir.path(),
        3,
    )?;
    {
        let mut be = moved.world_mut().resource_mut::<PhysicsBackend>();
        be.downcast_mut::<DistributedGroundedRobot>()
            .unwrap()
            .transfer(2)?;
    }
    local.step_exact(157)?;
    moved.step_exact(157)?;
    record(&local, 3597)?;
    let directory = dir.path();
    let barrier = directory.join("barrier.gzgroundedrobot");
    let changed = xml.replace("end_tick=\"3460\"", "end_tick=\"3461\"");
    assert_ne!(xml, changed);
    let previous = fs::read(&barrier)?;
    assert!(return_grounded_robot_to_local(&mut moved, &changed, "robot.sdf", "default").is_err());
    assert_eq!(fs::read(&barrier)?, previous);
    assert!(reopen(directory).is_err());

    // One torque has already reached the managed ledger. One COM force is staged on Bevy.
    // Both must survive release and a restart before the next physical step.
    for sim in [&mut local, &mut moved] {
        let mut be = sim.world_mut().resource_mut::<PhysicsBackend>();
        be.add_joint_effort(stand.axes[5].joint, -0.125)?;
        be.add_world_wrench(stand.chest, [4.75, -17.5, 1.25], [0.; 3])?;
    }
    let process = {
        let p = moved
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedGroundedRobot>()
            .unwrap()
            .pool();
        p.worker_pid(p.owner()).unwrap()
    };
    assert!(
        Command::new("kill")
            .args(["-KILL", &process.to_string()])
            .status()?
            .success()
    );
    return_grounded_robot_to_local(&mut moved, &xml, "robot.sdf", "default")?;
    assert!(moved.world().contains_resource::<Stand>());
    let lease = moved.world().resource::<LocalGroundedRobotSession>();
    assert_eq!(
        (
            lease.durable_epoch(),
            lease.durable_tick(),
            lease.generation()
        ),
        (1, 3597, 3)
    );
    let released = fs::read(&barrier)?;
    assert_eq!(&released[..8], b"GZRBLC01");
    assert!(GroundedRobotPool::resume(&exe, directory).is_err());
    assert!(reopen(directory).is_err());
    let copy = tempfile::tempdir()?;
    fs::write(copy.path().join("barrier.gzgroundedrobot"), &released)?;
    if let Some(path) = std::env::var_os("GZ_ROBOT_RETURN_WITNESS") {
        fs::write(path, &released)?;
    }
    assert!(GroundedRobotPool::resume(&exe, copy.path()).is_err());
    assert!(
        load_returned_grounded_robot_world(&changed, "robot.sdf", "default", copy.path()).is_err()
    );
    let cli = std::env::var_os("GZ_RETURN_LOCAL_CLI")
        .map(|exe| returned_cli::RobotCli::new(exe, &released, &xml, &changed))
        .transpose()?;
    if let Some(cli) = &cli {
        cli.check(&local, 0)?;
    }
    let mut restored = reopen(copy.path())?;
    for tick in 3598..=4130 {
        for sim in [&mut local, &mut moved, &mut restored] {
            sim.step_exact(1)?;
        }
        record(&local, tick)?;
        if let Some(cli) = &cli
            && matches!(tick, 3598 | 3617 | 3820)
        {
            cli.check(&local, tick - 3597)?;
        }
        verify_native(&local, &moved, &stand)?;
        verify_native(&local, &restored, &stand)?;
        if tick == 3820 {
            assert_eq!(fs::read(&barrier)?, released);
            checkpoint_returned_grounded_robot(&mut moved)?;
            assert_eq!(
                moved
                    .world()
                    .resource::<LocalGroundedRobotSession>()
                    .durable_tick(),
                3820
            );
            assert!(checkpoint_returned_grounded_robot(&mut moved).is_err());
            let snapshot = tempfile::tempdir()?;
            fs::copy(&barrier, snapshot.path().join("barrier.gzgroundedrobot"))?;
            let again = reopen(snapshot.path())?;
            verify_native(&local, &again, &stand)?;
            if let Some(path) = std::env::var_os("GZ_ROBOT_RETURN_CHECKPOINT") {
                fs::copy(&barrier, path)?;
            }
        }
    }
    drop(restored);
    assert!(GroundedRobotPool::resume(&exe, copy.path()).is_err());
    for sim in [&mut local, &mut moved] {
        gz_sim_scene::reset_loaded_world(sim, |_| ())?;
        sim.step_exact(100)?;
    }
    verify_native(&local, &moved, &stand)?;
    checkpoint_returned_grounded_robot(&mut moved)?;
    drop(moved);
    let recovered = reopen(directory)?;
    verify_native(&local, &recovered, &stand)?;
    assert_eq!(
        recovered
            .world()
            .resource::<LocalGroundedRobotSession>()
            .durable_epoch(),
        2
    );
    println!(
        "epoch 1, handoff 3440, migration, source process {process} killed at loaded t3597, fenced local generation 3, staged -0.125 Nm and [4.75,-17.5,1.25] N. 533 continuous local/returned/reopened whole graph comparisons, optional durable t3820 and epoch 2/reset t100, lock and wrong-program controls."
    );
    Ok(())
}

fn verify_managed(a: &Simulation, b: &Simulation, stand: &Stand) -> Result<()> {
    let original = native(a);
    let backend = b.world().resource::<PhysicsBackend>();
    let moved = backend
        .downcast_ref::<DistributedGroundedRobot>()
        .ok_or("expected managed owner")?;
    assert_eq!(
        (a.epoch(), a.committed_update_info()?.iteration),
        (moved.pool().epoch(), moved.pool().tick())
    );
    for &id in &stand.segments {
        assert_eq!(original.state(id), backend.state(id));
        assert_eq!(
            original.ground_reaction(id, stand.floor)?,
            moved.ground_reaction(id, stand.floor)?
        );
    }
    for axis in &stand.axes {
        assert_eq!(
            original.joint_state(axis.joint),
            backend.joint_state(axis.joint)
        );
    }
    assert_eq!(
        original.fixed_ground_patches(stand.floor)?,
        moved.fixed_ground_patches(stand.floor)?
    );
    assert_eq!(
        original
            .checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
            .as_slice(),
        moved.pool().native_image().unwrap()
    );
    Ok(())
}

#[test]
#[ignore = "requires a verified physical returned-local humanoid cut and a real CPU worker"]
fn same_ledger_rejoins_only_the_last_checkpoint_and_preserves_two_ownership_cycles() -> Result<()> {
    let exe =
        PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing real robot worker")?);
    let seed = std::env::var_os("GZ_ROBOT_RETURN_SOURCE").ok_or("missing loaded return witness")?;
    let args = Args::try_parse_from([
        "humanoid_stand",
        "--authored-footsteps",
        "--author-disturbance",
        "--recovery-landing",
        "--motion",
        "step",
        "--push-x",
        "0",
        "--push-y=-60",
        "--push-at",
        "3380",
        "--push-ticks",
        "80",
    ])?;
    let xml = footstep_sdf(&args)?;
    let mut physical = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "robot.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let stand = Stand::resolve(&mut physical, &args)?;
    gz_sim_scene::reset_loaded_world(&mut physical, |_| ())?;
    let mut frames = std::env::var_os("GZ_ROBOT_REJOIN_TRACE")
        .map(File::create)
        .transpose()?
        .map(BufWriter::new);
    let mut record = |sim: &Simulation, tick: u64| -> Result<()> {
        if let Some(out) = &mut frames {
            let poses = tree_visuals::poses(sim)?.into_values().collect::<Vec<_>>();
            serde_json::to_writer(&mut *out, &serde_json::json!({"tick":tick,"visuals":poses}))?;
            writeln!(out)?;
        }
        Ok(())
    };
    record(&physical, 0)?;
    for tick in 1..=160 {
        physical.step_exact(1)?;
        record(&physical, tick)?;
    }
    physical.step_exact(3597 - 160)?;
    {
        let mut be = physical.world_mut().resource_mut::<PhysicsBackend>();
        be.add_joint_effort(stand.axes[5].joint, -0.125)?;
        be.add_world_wrench(stand.chest, [4.75, -17.5, 1.25], [0.; 3])?;
    }
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("barrier.gzgroundedrobot");
    fs::copy(seed, &file)?;
    let mut moved = load_returned_grounded_robot_world(&xml, "robot.sdf", "default", dir.path())?;
    let open_manager = || {
        load_distributed_grounded_robot_world(
            &xml,
            "robot.sdf",
            "default",
            &exe,
            dir.path(),
            Some(3),
        )
    };
    let mut generations = Vec::new();
    for tick in 3598..=4310 {
        physical.step_exact(1)?;
        moved.step_exact(1)?;
        if moved
            .world()
            .contains_resource::<LocalGroundedRobotSession>()
        {
            verify_native(&physical, &moved, &stand)?;
        } else {
            verify_managed(&physical, &moved, &stand)?;
        }
        if (3660..=3930).contains(&tick) {
            record(&physical, tick)?;
        }
        if tick == 3650 {
            let prior = fs::read(&file)?;
            assert!(rejoin_grounded_robot_workers(&mut moved, &exe).is_err());
            assert_eq!(fs::read(&file)?, prior);
        }
        if tick == 3660 || tick == 4200 {
            for w in [&mut moved, &mut physical] {
                w.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_joint_effort(stand.axes[8].joint, 0.072)?;
            }
            checkpoint_returned_grounded_robot(&mut moved)?;
            let prior = fs::read(&file)?;
            assert_eq!(&prior[..8], b"GZRBLC01");
            assert!(
                rejoin_grounded_robot_workers(&mut moved, &dir.path().join("absent-worker"))
                    .is_err()
            );
            assert_eq!(fs::read(&file)?, prior);
            rejoin_grounded_robot_workers(&mut moved, &exe)?;
            verify_managed(&physical, &moved, &stand)?;
            let p = moved
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<DistributedGroundedRobot>()
                .unwrap()
                .pool();
            assert!(
                p.observation()
                    .joints
                    .iter()
                    .any(|j| j.pending_effort == Some(0.072))
            );
            generations.push(p.generation());
            assert_eq!(&fs::read(&file)?[..8], b"GZRBCU01");
            if tick == 3660
                && let Some(dst) = std::env::var_os("GZ_ROBOT_REJOIN_WITNESS")
            {
                fs::copy(&file, dst)?;
            }
            println!(
                "rejoined worker at epoch 1 tick {tick} generation {} with held 0.072 Nm",
                p.generation()
            );
        }
        if tick == 3700 {
            let mut be = moved.world_mut().resource_mut::<PhysicsBackend>();
            let r = be.downcast_mut::<DistributedGroundedRobot>().unwrap();
            let old = r.pool().owner();
            let pid = r.pool().worker_pid(old).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            r.transfer((old + 1) % r.pool().width())?;
            generations.push(r.pool().generation());
            println!("killed real rejoined owner {pid} at tick {tick}");
        }
        if tick == 3750 || tick == 4210 {
            drop(moved);
            assert!(
                load_returned_grounded_robot_world(&xml, "robot.sdf", "default", dir.path())
                    .is_err()
            );
            moved = open_manager()?;
            verify_managed(&physical, &moved, &stand)?;
        }
        if tick == 4100 {
            return_grounded_robot_to_local(&mut moved, &xml, "robot.sdf", "default")?;
            generations.push(
                moved
                    .world()
                    .resource::<LocalGroundedRobotSession>()
                    .generation(),
            );
            verify_native(&physical, &moved, &stand)?;
        }
    }
    assert_eq!(generations, [4, 5, 6, 7]);
    for sim in [&mut physical, &mut moved] {
        gz_sim_scene::reset_loaded_world(sim, |_| ())?;
        sim.step_exact(100)?;
    }
    verify_managed(&physical, &moved, &stand)?;
    assert_eq!(moved.epoch(), 2);
    println!(
        "713 complete native/public/sole comparisons, stale clock and missing destination recovery, two checked local checkpoints, generations 4/5/6/7, managed disk reopen twice and original epoch-2 reset."
    );
    Ok(())
}

#[test]
#[ignore = "requires a loaded ordinary robot cut and real worker"]
fn failed_robot_publication_fences_the_live_scene_until_disk_reopen() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let source = std::env::var_os("GZ_ROBOT_RETURN_SOURCE").ok_or("missing ordinary cut")?;
    let args = Args::try_parse_from([
        "humanoid_stand",
        "--authored-footsteps",
        "--author-disturbance",
        "--recovery-landing",
        "--motion",
        "step",
        "--push-x",
        "0",
        "--push-y=-60",
        "--push-at",
        "3380",
        "--push-ticks",
        "80",
    ])?;
    let program = footstep_sdf(&args)?;
    let xml = program.as_str();
    for operation in ["checkpoint", "rejoin", "return"] {
        let dir = tempfile::tempdir()?;
        let file = dir.path().join("barrier.gzgroundedrobot");
        fs::copy(&source, &file)?;
        let open_local =
            || load_returned_grounded_robot_world(xml, "robot.sdf", "default", dir.path());
        let mut sim = open_local()?;
        sim.step_exact(1)?;
        checkpoint_returned_grounded_robot(&mut sim)?;
        if operation == "return" {
            rejoin_grounded_robot_workers(&mut sim, &exe)?;
        } else if operation == "checkpoint" {
            sim.step_exact(1)?;
        }
        let previous = fs::read(&file)?;
        // Occupy the reserved staging name after the real directory lease is held.
        // Publication now receives a kernel I/O error after destination preparation.
        let staging = dir.path().join("barrier.pending.gzhinge");
        fs::create_dir(&staging)?;
        let result = match operation {
            "checkpoint" => checkpoint_returned_grounded_robot(&mut sim),
            "rejoin" => rejoin_grounded_robot_workers(&mut sim, &exe),
            "return" => return_grounded_robot_to_local(&mut sim, xml, "robot.sdf", "default"),
            _ => unreachable!(),
        };
        assert!(result.is_err());
        assert!(
            sim.is_faulted(),
            "{operation} did not fence the live process"
        );
        assert!(sim.step_exact(1).is_err());
        assert_eq!(fs::read(&file)?, previous);
        fs::remove_dir(&staging)?;
        assert!(checkpoint_returned_grounded_robot(&mut sim).is_err());
        drop(sim);
        let mut restored = if operation == "return" {
            load_distributed_grounded_robot_world(
                xml,
                "robot.sdf",
                "default",
                &exe,
                dir.path(),
                None,
            )?
        } else {
            open_local()?
        };
        assert_eq!(restored.committed_update_info()?.iteration, 3598);
        restored.step_exact(1)?;
        println!(
            "{operation}: physical publication error fenced stepping; recovered unchanged cut 3598 and stepped to 3599"
        );
    }
    Ok(())
}

#[test]
#[ignore = "requires a loaded wider walking cut, its exact SDF and the real physics worker"]
fn rejoined_wide_stride_survives_both_airborne_and_touchdown_worker_losses() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let source = std::env::var_os("GZ_ROBOT_WIDE_STRIDE_CUT").ok_or("missing wide-stride cut")?;
    let xml = fs::read_to_string(
        std::env::var_os("GZ_ROBOT_WIDE_STRIDE_SDF").ok_or("missing physical program")?,
    )?;
    let args = Args::try_parse_from([
        "humanoid_stand",
        "--authored-footsteps",
        "--support-relative",
        "--motion",
        "walk",
        "--steps",
        "64",
        "--step-m",
        ".115",
        "--push-x",
        "0",
        "--lateral-correction-limit-m",
        ".075",
    ])?;
    let direct_dir = tempfile::tempdir()?;
    let mixed_dir = tempfile::tempdir()?;
    for dir in [&direct_dir, &mixed_dir] {
        fs::copy(&source, dir.path().join("barrier.gzgroundedrobot"))?;
    }
    let open = |path: &Path| load_returned_grounded_robot_world(&xml, "walk.sdf", "default", path);
    let mut physical = open(direct_dir.path())?;
    let mut mixed = open(mixed_dir.path())?;
    let stand = Stand::resolve(&mut physical, &args)?;
    assert_eq!(physical.committed_update_info()?.iteration, 304700);
    let checks = |world: &Simulation| -> Result<[f64; 2]> {
        let be = native(world);
        Ok([
            be.ground_reaction(stand.feet[0], stand.floor)?
                .normal_impulse,
            be.ground_reaction(stand.feet[1], stand.floor)?
                .normal_impulse,
        ])
    };
    let starting = checks(&physical)?;
    let swing = usize::from(starting[0] > 0.001);
    assert_eq!(starting[swing], 0.);
    assert!(starting[1 - swing] > 0.25);
    let mut kills = 0;
    let mut landed = None;
    for tick in 304701..=306700 {
        physical.step_exact(1)?;
        mixed.step_exact(1)?;
        if tick == 304701 {
            checkpoint_returned_grounded_robot(&mut mixed)?;
            rejoin_grounded_robot_workers(&mut mixed, &exe)?;
        }
        verify_managed(&physical, &mixed, &stand)?;
        let impulses = checks(&physical)?;
        let first_contact = landed.is_none() && impulses[swing] > 0.001;
        if first_contact {
            landed = Some(tick);
        }
        if first_contact || tick == 304720 {
            if tick == 304720 {
                assert_eq!(impulses[swing], 0.);
            }
            let mut be = mixed.world_mut().resource_mut::<PhysicsBackend>();
            let robot = be.downcast_mut::<DistributedGroundedRobot>().unwrap();
            let from = robot.pool().owner();
            let pid = robot
                .pool()
                .worker_pid(from)
                .ok_or("missing actual owner")?;
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            robot.transfer((from + 1) % robot.pool().width())?;
            kills += 1;
            println!(
                "tick={tick} oscillator_swing={swing} sole_impulses_ns={impulses:?} killed={pid} new_owner={}",
                robot.pool().owner()
            );
        }
    }
    assert_eq!(kills, 2);
    assert!(landed.is_some());
    println!(
        "matched=2000 native_graphs=2000 complete link/joint/force/sole comparisons first_contact={landed:?}"
    );
    Ok(())
}
