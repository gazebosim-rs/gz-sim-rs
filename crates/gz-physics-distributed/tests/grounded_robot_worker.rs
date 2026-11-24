// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_distributed::grounded_robot::{GroundedRobotPool, RobotGraphEncoding};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, distributed_hinge::distributed_grounded_robot_spec,
    joint_posture::JointPostures, load_world_with_sdf_step,
};
use std::{error::Error, fs, path::Path, process::Command};
const HUMAN: &str = include_str!("../../../examples/humanoid_stand.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn named(sim: &Simulation, path: &str) -> (u64, BodyId) {
    sim.world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == path)
                .then(|| Some((e.get::<EntityId>()?.get(), e.get::<BodyHandle>()?.id())))?
        })
        .unwrap()
}
fn halt(pid: u32) -> Result<()> {
    assert!(
        Command::new("kill")
            .args(["-KILL", &pid.to_string()])
            .status()?
            .success()
    );
    Ok(())
}
#[test]
fn standing_bilateral_corners_hold_through_paged_transfer_kill_and_disturbance() -> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_grounded_robot_spec(HUMAN, "human.sdf", "default")?;
    let mut sim = load_world_with_sdf_step(
        HUMAN,
        "human.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let root = named(&sim, "default::humanoid::pelvis").1;
    let floor = named(&sim, "default::floor::ground").1;
    let (chest_public, chest) = named(&sim, "default::humanoid::chest");
    let bodies: Vec<_> = spec
        .links
        .iter()
        .map(|l| {
            (
                l.id,
                sim.world()
                    .get::<BodyHandle>(entity_by_id(sim.world(), l.id).unwrap())
                    .unwrap()
                    .id(),
            )
        })
        .collect();
    let dir = tempfile::tempdir()?;
    let mut pool = GroundedRobotPool::create_with_encoding(
        exe,
        dir.path(),
        3,
        spec.clone(),
        RobotGraphEncoding::FullPages,
    )?;
    let archive = dir.path().join("barrier.gzgroundedrobot");
    let mut supported = 0;
    let mut transferred = 0;
    let mut kills = 0;
    let mut reopens = 0;
    let mut peak_pages = 0;
    let mut last_support = 0.;
    let mut max_bytes = 0;
    let mut p_hist = Vec::new();
    let mut transfer_us = Vec::new();
    let mut backup = Vec::new();
    for t in 0..1800u64 {
        if [5, 680, 1049, 1060, 1550].contains(&t) {
            let now = std::time::Instant::now();
            pool.transfer((pool.owner() + 1) % pool.width())?;
            transfer_us.push(now.elapsed().as_micros());
            transferred += 1;
        }
        if [5, 680, 1049, 1054, 1550].contains(&t) {
            halt(pool.worker_pid(pool.owner()).unwrap())?;
            kills += 1;
        }
        if [58, 692, 1050, 1555].contains(&t) {
            drop(pool);
            pool = GroundedRobotPool::resume_with_encoding(
                exe,
                dir.path(),
                RobotGraphEncoding::FullPages,
            )?;
            reopens += 1;
        }
        if [681, 1051].contains(&t) {
            pool.drop_next_acknowledgement(pool.owner())?;
        }
        if t == 1000 {
            let prior = fs::read(&archive)?;
            let j = spec.joints[0].joint_id;
            assert!(pool.advance_with_inputs_at(1, t, &[], &[]).is_err());
            assert!(
                pool.advance_with_inputs_at(0, t, &[(j, 1.), (j, 2.)], &[])
                    .is_err()
            );
            assert!(
                pool.advance_with_inputs_at(0, t, &[], &[(chest_public, [f64::INFINITY; 3])])
                    .is_err()
            );
            assert_eq!(fs::read(&archive)?, prior);
        }
        let loads = if (1000..1080).contains(&t) {
            let f = [140., 0., 0.];
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(chest, f, [0.; 3])?;
            vec![(chest_public, f)]
        } else {
            vec![]
        };
        sim.step_exact(1)?;
        let ctl = sim
            .world()
            .resource::<JointPostures>()
            .get(spec.model_id)
            .unwrap();
        let efforts: Vec<_> = ctl
            .axes()
            .iter()
            .zip(ctl.last_commands())
            .map(|(a, c)| {
                assert_eq!(a.joint, c.joint);
                (a.entity_id, c.effort_nm)
            })
            .collect();
        let prior_pages = pool.traffic().downloaded_pages;
        let now = std::time::Instant::now();
        pool.advance_with_inputs_at(0, t, &efforts, &loads)?;
        p_hist.push(now.elapsed().as_micros());
        peak_pages = peak_pages.max(pool.traffic().downloaded_pages - prior_pages);
        let be = sim.world().resource::<PhysicsBackend>();
        let n = be.downcast_ref::<MultiFidelityBackend>().unwrap();
        let image = n.checkpoint_grounded_robot_tree(root, floor)?;
        assert_eq!(
            pool.native_image().unwrap().len(),
            image.len(),
            "robot native image length differs at {t}"
        );
        let mismatch = pool
            .native_image()
            .unwrap()
            .iter()
            .zip(&image)
            .position(|(a, b)| a != b);
        assert_eq!(
            mismatch,
            None,
            "robot native image differs at tick {}",
            t + 1
        );
        max_bytes = max_bytes.max(image.len());
        let state = pool.observation();
        assert_eq!(state.tick, t + 1);
        for (i, &(id, body)) in bodies.iter().enumerate() {
            let expected = be.state(body).unwrap();
            let s = &state.links[i];
            assert_eq!(s.id, id);
            for (a, b) in s
                .position
                .iter()
                .chain(&s.orientation)
                .chain(&s.velocity)
                .chain(&s.angular_velocity)
                .zip(
                    expected
                        .position
                        .iter()
                        .chain(&expected.orientation)
                        .chain(&expected.velocity)
                        .chain(&expected.angular_velocity),
                )
            {
                assert_eq!(a.to_bits(), b.to_bits(), "robot body {id} at {}", t + 1);
            }
            assert_eq!(
                bincode::serialize(&pool.ground_reactions()[i])?,
                bincode::serialize(&n.ground_reaction(body, floor)?)?
            );
        }
        for a in ctl.axes() {
            let j = be.joint_state(a.joint).unwrap();
            let s = state
                .joints
                .iter()
                .find(|s| s.joint_id == a.entity_id)
                .unwrap();
            assert_eq!(s.q.to_bits(), j.position.to_bits());
            assert_eq!(s.qdot.to_bits(), j.velocity.to_bits());
            assert_eq!(s.pending_effort, None);
        }
        let mut normal = 0.;
        let ordinary = n.fixed_ground_patches(floor)?;
        assert_eq!(pool.ground_patches().len(), ordinary.len());
        for patch in &ordinary {
            let id = bodies.iter().find(|(_, b)| *b == patch.body).unwrap().0;
            let p = pool
                .ground_patches()
                .iter()
                .find(|p| p.link_id == id)
                .unwrap();
            assert_eq!(
                p.normal_on_body.map(f64::to_bits),
                patch.normal_on_body.map(f64::to_bits)
            );
            assert_eq!(
                bincode::serialize(&p.points)?,
                bincode::serialize(&patch.points)?
            );
            normal += p.points.iter().map(|r| r.normal_impulse).sum::<f64>();
        }
        if ordinary.len() == 2
            && ordinary
                .iter()
                .all(|p| p.points.iter().filter(|q| q.normal_impulse > 0.).count() >= 3)
        {
            supported += 1;
        }
        if t >= 1700 {
            last_support += normal * 10.;
        }
        if t == 1049 {
            backup = fs::read(&archive)?;
        }
    }
    assert_eq!(pool.tick(), 1800);
    assert!(supported >= 1700);
    assert!((last_support - 735.75).abs() < 1.);
    assert!(max_bytes > 192 * 1024 && peak_pages >= 4);
    if let Some(path) = std::env::var_os("GZ_ROBOT_WITNESS") {
        let path = Path::new(&path);
        fs::create_dir_all(path)?;
        fs::write(path.join("push.gzgroundedrobot"), backup)?;
        fs::copy(&archive, path.join("settled.gzgroundedrobot"))?;
        fs::write(
            path.join("steps-us.csv"),
            p_hist
                .iter()
                .enumerate()
                .map(|(i, t)| format!("{},{}\n", i + 1, t))
                .collect::<String>(),
        )?;
    }
    p_hist.sort_unstable();
    println!(
        "robot joints={} links={} exact_ordinary_native_steps=1800 bilateral_three_loaded_corners={} max_native_bytes={} max_download_pages={} transfers={} worker_kills={} reopens={} average_settled_support_N={:.6} worker_step_p50_p95_p99_us={:?} transfer_us={:?}",
        spec.joints.len(),
        spec.links.len(),
        supported,
        max_bytes,
        peak_pages,
        transferred,
        kills,
        reopens,
        last_support,
        [p_hist[900], p_hist[1710], p_hist[1782]],
        transfer_us
    );
    pool.reset()?;
    assert_eq!((pool.epoch(), pool.tick(), pool.owner()), (1, 0, 0));
    assert!(pool.native_image().is_none());
    Ok(())
}

#[test]
fn paired_packed_and_original_paged_workers_match_every_durable_byte() -> Result<()> {
    use std::{fmt::Write, time::Instant};
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_grounded_robot_spec(HUMAN, "human.sdf", "default")?;
    let mut sim = load_world_with_sdf_step(
        HUMAN,
        "human.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let (public, chest) = named(&sim, "default::humanoid::chest");
    let root = named(&sim, "default::humanoid::pelvis").1;
    let floor = named(&sim, "default::floor::ground").1;
    let dirs = [tempfile::tempdir()?, tempfile::tempdir()?];
    let mut pools = [
        GroundedRobotPool::create_with_encoding(
            exe,
            dirs[0].path(),
            3,
            spec.clone(),
            RobotGraphEncoding::FullPages,
        )?,
        GroundedRobotPool::create(exe, dirs[1].path(), 3, spec.clone())?,
    ];
    assert_eq!(pools[1].encoding(), RobotGraphEncoding::Packed);
    let mut timing = [Vec::new(), Vec::new()];
    let mut transfers = [Vec::new(), Vec::new()];
    let mut wire = [0u64; 2];
    let mut record =
        String::from("tick,first,paged_us,packed_us,paged_graph_bytes,packed_graph_bytes\n");
    let mut pages = 0;
    let mut packets = 0;
    let mut warm = 0;
    for t in 0..1800u64 {
        if [350, 680, 1049, 1210, 1550].contains(&t) {
            for i in [((t / 10) % 2) as usize, 1 - ((t / 10) % 2) as usize] {
                let p = &mut pools[i];
                let start = Instant::now();
                p.transfer((p.owner() + 1) % p.width())?;
                transfers[i].push(start.elapsed().as_micros());
            }
        }
        if [682, 1055, 1560].contains(&t) {
            for p in &pools {
                halt(p.worker_pid(p.owner()).unwrap())?;
            }
        }
        let force = if (1000..1080).contains(&t) {
            let f = [140., 0., 0.];
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(chest, f, [0.; 3])?;
            vec![(public, f)]
        } else {
            vec![]
        };
        sim.step_exact(1)?;
        let ctl = sim
            .world()
            .resource::<JointPostures>()
            .get(spec.model_id)
            .unwrap();
        let efforts: Vec<_> = ctl
            .axes()
            .iter()
            .zip(ctl.last_commands())
            .map(|(a, c)| (a.entity_id, c.effort_nm))
            .collect();
        let mut bytes = [0; 2];
        for i in [(t % 2) as usize, 1 - (t % 2) as usize] {
            let p = &mut pools[i];
            let before = p.traffic();
            let start = Instant::now();
            p.advance_with_inputs_at(0, t, &efforts, &force)?;
            timing[i].push(start.elapsed().as_micros());
            let after = p.traffic();
            bytes[i] = after.downloaded_bytes - before.downloaded_bytes;
            wire[i] += bytes[i];
            if i == 0 {
                pages += after.downloaded_pages - before.downloaded_pages;
            } else {
                packets += after.packed_downloads - before.packed_downloads;
            }
        }
        writeln!(
            &mut record,
            "{},{},{},{},{},{}",
            t + 1,
            t % 2,
            timing[0].last().unwrap(),
            timing[1].last().unwrap(),
            bytes[0],
            bytes[1]
        )?;
        let file = |i: usize| fs::read(dirs[i].path().join("barrier.gzgroundedrobot"));
        let a = file(0)?;
        let b = file(1)?;
        assert!(a == b, "whole archive differs at {}", t + 1);
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        assert_eq!(
            pools[1].native_image().unwrap(),
            native
                .checkpoint_grounded_robot_tree(root, floor)?
                .as_slice(),
            "native ordinary at {}",
            t + 1
        );
        warm += usize::from(pools[1].ground_patches().len() == 2);
    }
    assert_eq!(packets, 1800);
    assert!(pages >= 7200 && warm > 1700);
    assert!(wire[1] * 2 < wire[0]);
    if let Some(d) = std::env::var_os("GZ_ROBOT_WIRE_OUT") {
        fs::create_dir_all(&d)?;
        fs::write(Path::new(&d).join("paired-us.csv"), record)?;
    }
    for (i, r) in timing.iter_mut().enumerate() {
        r.sort_unstable();
        println!(
            "encoding={:?} exact_ordinary_and_archived_intervals=1800 p50_p95_p99_us={:?} received_graph_bytes={} transfer_us={:?} bilateral_foot_ticks={warm}",
            pools[i].encoding(),
            [r[900], r[1710], r[1782]],
            wire[i],
            transfers[i]
        );
    }
    Ok(())
}

#[test]
fn worker_evaluates_each_posture_interval_and_recovers_the_complete_batch() -> Result<()> {
    use gz_physics::posture::PostureMode;
    use gz_physics_distributed::grounded_robot::{RobotPostureInterval, RobotPostureProgram};
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_grounded_robot_spec(HUMAN, "human.sdf", "default")?;
    let mut sim = load_world_with_sdf_step(
        HUMAN,
        "human.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let (chest_public, chest) = named(&sim, "default::humanoid::chest");
    let root = named(&sim, "default::humanoid::pelvis").1;
    let floor = named(&sim, "default::floor::ground").1;
    let ctl = sim
        .world()
        .resource::<JointPostures>()
        .get(spec.model_id)
        .unwrap();
    let law = ctl.law();
    let driven = ctl.axes().last().unwrap().joint;
    let home = law.axes.last().unwrap().reference_rad;
    let dir = tempfile::tempdir()?;
    let mut p = GroundedRobotPool::create(exe, dir.path(), 3, spec.clone())?;
    let file = dir.path().join("barrier.gzgroundedrobot");
    let mut boundaries = 0;
    let mut ticks = 0;
    let mut count_torques = 0;
    let mut both = 0;
    let mut varied = 0;
    let mut deaths = 0;
    let mut migrations = 0;
    let mut reopens = 0;
    while ticks < 1800u64 {
        let start = ticks;
        let n = [1u64, 2, 4, 8, 16][boundaries % 5].min(1800 - start);
        let initial = fs::read(&file)?;
        let mut intervals = Vec::new();
        let mut commands = Vec::new();
        let mut centers = Vec::new();
        for t in start..start + n {
            {
                let mut all = sim.world_mut().resource_mut::<JointPostures>();
                let ctl = all.get_mut(spec.model_id).unwrap();
                let mode = if t == 412 {
                    PostureMode::Passive
                } else if (517..519).contains(&t) {
                    PostureMode::Posture
                } else {
                    PostureMode::Balance
                };
                ctl.set_mode(mode);
                ctl.set_balance_offset(if (541..581).contains(&t) {
                    [0.0001, -0.0002]
                } else {
                    [0.; 2]
                })?;
                ctl.set_target(
                    driven,
                    if (541..561).contains(&t) {
                        home + 0.003
                    } else {
                        home
                    },
                )?;
                ctl.set_feedforward(driven, if (562..590).contains(&t) { 0.024 } else { 0. })?;
                ctl.set_direct_effort(driven, (612..615).contains(&t).then_some(-0.013))?;
            }
            let ctl = sim
                .world()
                .resource::<JointPostures>()
                .get(spec.model_id)
                .unwrap();
            let targets = ctl.targets();
            centers.push(if targets.mode == PostureMode::Balance {
                let (x, v) = ctl.center_of_mass(sim.world().resource::<PhysicsBackend>())?;
                Some((x.to_array(), v.to_array()))
            } else {
                None
            });
            let forces = if (1000..1080).contains(&t) {
                let f = [140., 0., 0.];
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(chest, f, [0.; 3])?;
                vec![(chest_public, f)]
            } else {
                vec![]
            };
            intervals.push(RobotPostureInterval { targets, forces });
            sim.step_exact(1)?;
            let c = sim
                .world()
                .resource::<JointPostures>()
                .get(spec.model_id)
                .unwrap();
            commands.push(
                c.axes()
                    .iter()
                    .zip(c.last_commands())
                    .map(|(a, v)| (a.entity_id, v.target_rad, v.effort_nm))
                    .collect::<Vec<_>>(),
            );
            let be = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            both += usize::from(
                be.fixed_ground_patches(floor)?
                    .iter()
                    .filter(|p| p.points.iter().filter(|p| p.normal_impulse > 0.).count() >= 3)
                    .count()
                    == 2,
            );
        }
        let program = RobotPostureProgram {
            law: law.clone(),
            intervals,
        };
        if boundaries % 50 == 29 {
            let mut bad = program.clone();
            bad.intervals
                .last_mut()
                .unwrap()
                .forces
                .push((u64::MAX, [1., 0., 0.]));
            assert!(p.advance_posture_batch_at(p.epoch(), start, &bad).is_err());
            assert_eq!(fs::read(&file)?, initial);
            assert!(
                p.advance_posture_batch_at(p.epoch() + 1, start, &program)
                    .is_err()
            );
        }
        if boundaries % 34 == 8 {
            let owner = p.owner();
            p.transfer((owner + 1) % p.width())?;
            migrations += 1;
        }
        if boundaries % 38 == 13 {
            halt(p.worker_pid(p.owner()).unwrap())?;
            deaths += 1;
        }
        if boundaries % 48 == 11 {
            p.drop_next_acknowledgement(p.owner())?;
        }
        let accepted = p.advance_posture_batch_at(0, start, &program)?.clone();
        ticks = start + n;
        assert_eq!(
            (accepted.receipt.start, accepted.receipt.end),
            (start, ticks)
        );
        assert_eq!(accepted.program, program);
        for (i, (row, ordinary)) in accepted.receipt.intervals.iter().zip(&commands).enumerate() {
            assert_eq!(row.tick, start + i as u64);
            assert_eq!(
                bincode::serialize(&row.center_of_mass)?,
                bincode::serialize(&centers[i])?,
                "center of mass before tick {}",
                row.tick
            );
            assert_eq!(row.torques.len(), ordinary.len());
            for (j, &(id, t, e)) in row.torques.iter().zip(ordinary) {
                assert_eq!(
                    (j.joint_id, j.target_rad.to_bits(), j.effort_nm.to_bits()),
                    (id, t.to_bits(), e.to_bits()),
                    "actual feedback at input {}",
                    row.tick
                );
                count_torques += 1;
            }
        }
        if accepted
            .receipt
            .intervals
            .windows(2)
            .any(|w| w[0].torques != w[1].torques)
        {
            varied += 1;
        }
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let graph = native.checkpoint_grounded_robot_tree(root, floor)?;
        assert!(
            p.native_image() == Some(&graph),
            "ordinary warm native graph differs at {ticks}"
        );
        if boundaries % 30 == 14 {
            let stored = fs::read(&file)?;
            assert_eq!(&stored[..8], b"GZRBCU02");
            drop(p);
            p = GroundedRobotPool::resume(exe, dir.path())?;
            reopens += 1;
            assert_eq!(p.last_posture_batch(), Some(&accepted));
            assert_eq!(fs::read(&file)?, stored);
        }
        boundaries += 1;
    }
    assert!(count_torques > 53000 && varied > 220 && both > 1700);
    println!(
        "feedback_intervals={ticks} actual_joint_torques={count_torques} full_ordinary_native_endpoint_images={boundaries} batches_with_changed_feedback={varied} bilateral_three_corners={both} real_owner_deaths={deaths} migrations={migrations} durable_feedback_reopens={reopens}"
    );
    p.reset()?;
    assert!(p.last_posture_batch().is_none());
    assert_eq!(&fs::read(&file)?[..8], b"GZRBCU01");
    Ok(())
}

#[test]
#[ignore = "paired physical/per-commit throughput from a real warm 1050ms robot archive"]
fn profile_identical_stance_with_external_and_one_to_sixteen_worker_feedback_intervals()
-> Result<()> {
    use gz_physics_distributed::grounded_robot::{RobotPostureInterval, RobotPostureProgram};
    use sha2::{Digest, Sha256};
    use std::{fmt::Write, time::Instant};
    const BEGIN: u64 = 1050;
    const N: usize = 1024;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let original = fs::read(std::env::var("GZ_ROBOT_LOADED_CUT")?)?;
    let out = std::env::var("GZ_ROBOT_BATCH_BENCH")?;
    fs::create_dir_all(&out)?;
    let spec = distributed_grounded_robot_spec(HUMAN, "human.sdf", "default")?;
    let mut sim = load_world_with_sdf_step(
        HUMAN,
        "human.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let (chest_public, chest) = named(&sim, "default::humanoid::chest");
    let root = named(&sim, "default::humanoid::pelvis").1;
    let floor = named(&sim, "default::floor::ground").1;
    let ctl = sim
        .world()
        .resource::<JointPostures>()
        .get(spec.model_id)
        .unwrap();
    let law = ctl.law();
    let targets = ctl.targets();
    let get_image = |s: &Simulation| -> Result<Vec<u8>> {
        Ok(s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .checkpoint_grounded_robot_tree(root, floor)?)
    };
    let mut frames = Vec::with_capacity(N);
    let mut torques = Vec::with_capacity(N);
    let mut raw = Vec::with_capacity(N);
    let mut hashes = Vec::with_capacity(N);
    let mut first = Vec::new();
    for t in 0..BEGIN + N as u64 {
        if t == BEGIN {
            first = get_image(&sim)?;
        }
        let forces = if (1000..1080).contains(&t) {
            let f = [140., 0., 0.];
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(chest, f, [0.; 3])?;
            vec![(chest_public, f)]
        } else {
            vec![]
        };
        let start = Instant::now();
        sim.step_exact(1)?;
        let native_us = start.elapsed().as_micros();
        if t < BEGIN {
            continue;
        }
        let c = sim
            .world()
            .resource::<JointPostures>()
            .get(spec.model_id)
            .unwrap();
        frames.push(RobotPostureInterval {
            targets: targets.clone(),
            forces,
        });
        torques.push(
            c.axes()
                .iter()
                .zip(c.last_commands())
                .map(|(a, c)| (a.entity_id, c.effort_nm))
                .collect::<Vec<_>>(),
        );
        raw.push(native_us);
        hashes.push(<[u8; 32]>::from(Sha256::digest(get_image(&sim)?)));
    }
    let widths = [1, 1, 2, 4, 8, 16];
    let names = [
        "external", "local1", "local2", "local4", "local8", "local16",
    ];
    let mut dirs = Vec::new();
    let mut pools = Vec::new();
    for _ in widths {
        let dir = tempfile::tempdir()?;
        fs::write(dir.path().join("barrier.gzgroundedrobot"), &original)?;
        let p = GroundedRobotPool::resume(exe, dir.path())?;
        assert!(p.native_image() == Some(&first));
        pools.push(p);
        dirs.push(dir);
    }
    // Explicitly test *every* prefix of one loaded 16 ms batch against its physical ordinary cut.
    for width in 1..=16 {
        let dir = tempfile::tempdir()?;
        fs::write(dir.path().join("barrier.gzgroundedrobot"), &original)?;
        let mut p = GroundedRobotPool::resume(exe, dir.path())?;
        p.advance_posture_batch_at(
            0,
            BEGIN,
            &RobotPostureProgram {
                law: law.clone(),
                intervals: frames[..width].to_vec(),
            },
        )?;
        assert_eq!(
            <[u8; 32]>::from(Sha256::digest(p.native_image().unwrap())),
            hashes[width - 1]
        );
    }
    let mut csv = String::from("profile,start_tick,intervals,wall_us,received_graph_bytes\n");
    let mut times = vec![Vec::<u128>::new(); 6];
    let mut wire = [0u64; 6];
    for block in 0..N / 16 {
        for turn in 0..6 {
            let i = (block + turn) % 6;
            let p = &mut pools[i];
            let span = widths[i];
            for j in (block * 16..(block + 1) * 16).step_by(span) {
                let t = BEGIN + j as u64;
                let program = RobotPostureProgram {
                    law: law.clone(),
                    intervals: frames[j..j + span].to_vec(),
                };
                let bytes = p.traffic().downloaded_bytes;
                let now = Instant::now();
                if i == 0 {
                    p.advance_with_inputs_at(0, t, &torques[j], &frames[j].forces)?;
                } else {
                    p.advance_posture_batch_at(0, t, &program)?;
                }
                let us = now.elapsed().as_micros();
                let bytes = p.traffic().downloaded_bytes - bytes;
                times[i].push(us);
                wire[i] += bytes;
                writeln!(&mut csv, "{},{},{span},{us},{bytes}", names[i], t)?;
                assert_eq!(
                    <[u8; 32]>::from(Sha256::digest(p.native_image().unwrap())),
                    hashes[j + span - 1],
                    "{} native endpoint {}",
                    names[i],
                    j + span
                );
                if let Some(record) = p.last_posture_batch() {
                    for (row, expected) in
                        record.receipt.intervals.iter().zip(&torques[j..j + span])
                    {
                        for (a, b) in row.torques.iter().zip(expected) {
                            assert_eq!((a.joint_id, a.effort_nm.to_bits()), (b.0, b.1.to_bits()));
                        }
                    }
                }
            }
        }
    }
    fs::write(Path::new(&out).join("batches.csv"), csv)?;
    raw.sort_unstable();
    println!(
        "ordinary_feedback_and_solver_no_checkpoint_samples={} p50_p95_p99_us={:?}",
        raw.len(),
        [raw[N / 2], raw[95 * N / 100], raw[99 * N / 100]]
    );
    for (i, a) in times.iter_mut().enumerate() {
        a.sort_unstable();
        let n = a.len();
        let total: u128 = a.iter().sum();
        println!(
            "profile={} native_intervals_per_commit={} measured_commits={n} actual_commit_p50_p95_p99_us={:?} wall_us_per_native_interval={:.3} managed_real_time_factor={:.4} graph_bytes={} checked_native_endpoint_digests={n}",
            names[i],
            widths[i],
            [a[n / 2], a[n * 95 / 100], a[n * 99 / 100]],
            total as f64 / N as f64,
            N as f64 * 1000. / total as f64,
            wire[i]
        );
    }
    Ok(())
}
