// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a clock-preserving transfer from an ordinary Bevy world into real CPU workers.
use super::tests::native;
use super::*;

#[test]
#[ignore = "requires the production robot CPU process via GZ_MULTI_TEST_WORKER"]
fn live_local_robot_adoption_preserves_clock_held_torque_loaded_floor_and_original_reset()
-> Result<()> {
    use gz_sim_scene::distributed_hinge::GroundedRobotPool;
    use gz_sim_scene::distributed_hinge::distributed_grounded_robot_spec;
    use std::{fs, path::Path, process::Command};
    let exe = PathBuf::from(
        std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing GZ_MULTI_TEST_WORKER")?,
    );
    let args = Args::try_parse_from([
        "humanoid_stand",
        "--motion",
        "step",
        "--recovery-landing",
        "--authored-footsteps",
        "--author-disturbance",
        "--push-x",
        "0",
        "--push-y=-60",
        "--push-at",
        "3380",
        "--push-ticks",
        "80",
    ])?;
    let xml = footstep_sdf(&args)?;
    let create = || -> Result<(Simulation, Stand)> {
        let mut sim = gz_sim_scene::load_world_with_sdf_step(
            &xml,
            "robot.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        let stand = Stand::resolve(&mut sim, &args)?;
        sim.world_mut().insert_resource(stand.clone());
        Ok((sim, stand))
    };
    let (mut physical, stand) = create()?;
    let (mut moved, _) = create()?;
    let dir = tempfile::tempdir()?;
    let rejected = tempfile::tempdir()?;
    let adopt = |s: &mut Simulation, xml: &str, path: &Path| {
        handoff_grounded_robot_to_workers(s, xml, "robot.sdf", "default", &exe, path, 3)
    };
    assert!(adopt(&mut moved, &xml, rejected.path()).is_err());
    for sim in [&mut physical, &mut moved] {
        sim.step_exact(3597)?;
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_joint_effort(stand.axes[5].joint, -0.125)?;
    }
    let before = native(&moved).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?;
    let spec = distributed_grounded_robot_spec(&xml, "robot.sdf", "default")?;
    let mut bad = before.clone();
    bad[0] ^= 1;
    for bytes in [bad.as_slice(), &before[..before.len() - 1]] {
        assert!(
            GroundedRobotPool::create_from_native(
                &exe,
                rejected.path(),
                3,
                spec.clone(),
                1,
                3597,
                bytes
            )
            .is_err()
        );
    }
    let other = xml.replace("10.5000</mass>", "10.6000</mass>");
    assert_ne!(other, xml);
    assert!(adopt(&mut moved, &other, rejected.path()).is_err());
    assert!(
        adopt(
            &mut moved,
            &xml.replace("right_foot", "foreign_foot"),
            rejected.path()
        )
        .is_err()
    );
    assert!(!rejected.path().join("barrier.gzgroundedrobot").exists());
    assert_eq!(
        native(&moved).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?,
        before
    );
    let original_pelvis = moved
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<BodyHandle>()
                .is_some_and(|h| h.id() == stand.pelvis)
        })
        .unwrap()
        .id();
    adopt(&mut moved, &xml, dir.path())?;
    assert!(moved.world().get::<BodyHandle>(original_pelvis).is_some());
    assert!(moved.world().contains_resource::<Stand>());
    let cut = fs::read(dir.path().join("barrier.gzgroundedrobot"))?;
    assert!(adopt(&mut physical, &xml, dir.path()).is_err());
    assert_eq!(fs::read(dir.path().join("barrier.gzgroundedrobot"))?, cut);
    let relation = |local: &Simulation, remote: &Simulation| -> Result<()> {
        let native = native(local);
        let be = remote.world().resource::<PhysicsBackend>();
        let managed = be.downcast_ref::<DistributedGroundedRobot>().unwrap();
        let clock = remote.committed_update_info()?;
        assert_eq!(
            (managed.pool().epoch(), managed.pool().tick()),
            (clock.epoch, clock.iteration)
        );
        for &b in &stand.segments {
            assert_eq!(
                local.world().resource::<PhysicsBackend>().state(b),
                be.state(b)
            );
            assert_eq!(
                native.ground_reaction(b, stand.floor)?,
                managed.ground_reaction(b, stand.floor)?
            );
        }
        for a in &stand.axes {
            assert_eq!(
                local
                    .world()
                    .resource::<PhysicsBackend>()
                    .joint_state(a.joint),
                be.joint_state(a.joint)
            );
        }
        assert_eq!(
            native.fixed_ground_patches(stand.floor)?,
            managed.fixed_ground_patches(stand.floor)?
        );
        assert_eq!(
            native
                .checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
                .as_slice(),
            managed.pool().native_image().unwrap()
        );
        Ok(())
    };
    relation(&physical, &moved)?;
    let pool = moved
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedGroundedRobot>()
        .unwrap()
        .pool();
    assert_eq!((pool.epoch(), pool.generation(), pool.tick()), (1, 1, 3597));
    assert!(
        pool.observation()
            .joints
            .iter()
            .any(|j| j.pending_effort == Some(-0.125))
    );
    if let Some(dst) = std::env::var_os("GZ_ROBOT_IMPORT_WITNESS") {
        fs::write(dst, &cut)?;
    }
    for t in 3598..=4120 {
        if t == 3700 {
            let mut be = moved.world_mut().resource_mut::<PhysicsBackend>();
            let robot = be.downcast_mut::<DistributedGroundedRobot>().unwrap();
            let old = robot.pool().owner();
            let pid = robot
                .pool()
                .worker_pid(old)
                .ok_or("missing real CPU owner")?;
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            robot.transfer((old + 1) % robot.pool().width())?;
            println!("killed actual imported owner PID {pid} at t{}", t - 1);
        }
        physical.step_exact(1)?;
        moved.step_exact(1)?;
        relation(&physical, &moved)?;
    }
    gz_sim_scene::reset_loaded_world(&mut physical, |_| ())?;
    gz_sim_scene::reset_loaded_world(&mut moved, |_| ())?;
    assert_eq!(moved.committed_update_info()?.epoch, 2);
    for _ in 0..140 {
        physical.step_exact(1)?;
        moved.step_exact(1)?;
        relation(&physical, &moved)?;
    }
    println!(
        "Adopted exact epoch 1 / t3597 graph including pending -0.125 Nm and first loaded recovery contact; 523 post-handoff and 140 post-reset exact native images, four rejection controls, and real worker death."
    );
    Ok(())
}

#[test]
#[ignore = "227 s physical walk, requires the production robot process via GZ_MULTI_TEST_WORKER"]
fn thirty_second_step_import_replays_the_real_landing_after_process_loss() -> Result<()> {
    use gz_sim_scene::distributed_hinge::GroundedRobotPool;
    use gz_sim_scene::distributed_hinge::distributed_grounded_robot_spec;
    use std::{fs, process::Command};
    const SWING: u64 = 220700;
    const END: u64 = 224500;
    let exe = PathBuf::from(
        std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing GZ_MULTI_TEST_WORKER")?,
    );
    let args = Args::try_parse_from([
        "humanoid_stand",
        "--motion",
        "walk",
        "--steps",
        "32",
        "--support-relative",
        "--authored-footsteps",
        "--push-x",
        "0",
    ])?;
    let xml = footstep_sdf(&args)?;
    let mut physical = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "long-walk.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let stand = Stand::resolve(&mut physical, &args)?;
    let spec = distributed_grounded_robot_spec(&xml, "long-walk.sdf", "default")?;
    let dir = tempfile::tempdir()?;
    let witness = std::env::var_os("GZ_LATE_WITNESS_DIR").map(PathBuf::from);
    if let Some(path) = &witness {
        fs::create_dir_all(path)?;
        fs::write(path.join("walk-32.sdf"), &xml)?;
    }
    let mut packet = witness
        .as_ref()
        .map(|p| File::create(p.join("ordinary-frames.jsonl")))
        .transpose()?
        .map(BufWriter::new);
    let frame_out =
        |sim: &Simulation, tick: u64, out: &mut Option<BufWriter<File>>| -> Result<()> {
            if let Some(file) = out {
                let poses = tree_visuals::poses(sim)?.into_values().collect::<Vec<_>>();
                serde_json::to_writer(
                    &mut *file,
                    &serde_json::json!({"tick":tick,"visuals":poses}),
                )?;
                writeln!(file)?;
            }
            Ok(())
        };
    frame_out(&physical, 0, &mut packet)?;
    for tick in 1..=SWING {
        physical.step_exact(1)?;
        if tick <= 160 || tick == SWING {
            frame_out(&physical, tick, &mut packet)?;
        }
    }
    let source = native(&physical).checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?;
    let make = Instant::now();
    let pool = GroundedRobotPool::create_from_native(&exe, dir.path(), 3, spec, 0, SWING, &source)?;
    let source_us = make.elapsed().as_micros();
    let save = |name: &str, tick: u64| -> Result<()> {
        if let Some(path) = &witness {
            fs::copy(
                dir.path().join("barrier.gzgroundedrobot"),
                path.join(format!("{name}.gzgroundedrobot")),
            )?;
            fs::write(
                path.join(format!("{name}.json")),
                serde_json::to_vec_pretty(
                    &serde_json::json!({"tick":tick,"input_us":source_us,"native_bytes":source.len()}),
                )?,
            )?;
        }
        Ok(())
    };
    save("final-swing", SWING)?;
    drop(pool);
    let open = || {
        load_distributed_grounded_robot_world(
            &xml,
            "long-walk.sdf",
            "default",
            &exe,
            dir.path(),
            Some(3),
        )
    };
    let mut moved = open()?;
    let mut first_touch = None;
    let mut lost = 0;
    let mut bilateral = 0;
    for tick in SWING..=END {
        if tick > SWING {
            physical.step_exact(1)?;
            moved.step_exact(1)?;
        }
        let local = native(&physical);
        let actual = moved.world().resource::<PhysicsBackend>();
        let robot = actual.downcast_ref::<DistributedGroundedRobot>().unwrap();
        for &body in &stand.segments {
            assert_eq!(
                physical.world().resource::<PhysicsBackend>().state(body),
                actual.state(body),
                "link {tick}"
            );
            assert_eq!(
                local.ground_reaction(body, stand.floor)?,
                robot.ground_reaction(body, stand.floor)?
            );
        }
        for axis in &stand.axes {
            assert_eq!(
                physical
                    .world()
                    .resource::<PhysicsBackend>()
                    .joint_state(axis.joint),
                actual.joint_state(axis.joint)
            );
        }
        if tick > SWING {
            assert_eq!(
                stand.controller(&physical).last_commands(),
                stand.controller(&moved).last_commands()
            );
        }
        assert_eq!(
            local.fixed_ground_patches(stand.floor)?,
            robot.fixed_ground_patches(stand.floor)?
        );
        let left = robot
            .ground_reaction(stand.feet[0], stand.floor)?
            .normal_impulse;
        let right = robot
            .ground_reaction(stand.feet[1], stand.floor)?
            .normal_impulse;
        if left > 0.001 && right > 0.001 {
            bilateral += 1;
        }
        let touch = first_touch.is_none() && left > 0.001;
        if touch {
            first_touch = Some(tick);
            save("final-touchdown", tick)?;
        }
        if tick > SWING
            && (tick <= SWING + 160 || first_touch.is_some_and(|t| tick >= t && tick <= t + 160))
        {
            frame_out(&physical, tick, &mut packet)?;
        }
        if tick == SWING + 1 || touch || tick % 71 == 0 || tick == END {
            assert_eq!(
                local
                    .checkpoint_grounded_robot_tree(stand.pelvis, stand.floor)?
                    .as_slice(),
                robot.pool().native_image().unwrap()
            );
        }
        if tick == SWING + 1 || touch {
            let old = robot.pool().owner();
            let pid = robot.pool().worker_pid(old).ok_or("missing owner")?;
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            lost += 1;
            drop(moved);
            moved = open()?;
            let mut b = moved.world_mut().resource_mut::<PhysicsBackend>();
            let r = b.downcast_mut::<DistributedGroundedRobot>().unwrap();
            r.transfer((old + 1) % r.pool().width())?;
            println!(
                "Killed actual CPU worker {pid}, resumed and transferred t{tick} with left {left}, right {right} Ns."
            );
        }
    }
    if let Some(file) = &mut packet {
        file.flush()?;
    }
    assert_eq!(lost, 2);
    assert!(first_touch.is_some_and(|t| (221400..222000).contains(&t)));
    assert!(
        bilateral > 600,
        "only {bilateral} bilateral intervals by {END}"
    );
    assert!(frame(&physical, &stand, END)?.tilt_rad < 0.1);
    println!(
        "Local accepted t{SWING} in final left swing imported in {source_us} us, {} native bytes; {} exact public intervals through t{END}, {bilateral} bilateral, first touchdown {first_touch:?}.",
        source.len(),
        END - SWING
    );
    Ok(())
}
