// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    articulation::ArticulationError,
    floating_tree::{FloatingTreePool, FloatingTreeWorkerProfile},
};
use gz_sim_scene::distributed_floating_tree_spec;
use std::{error::Error, fs, path::Path, process::Command, time::Instant};
const EXE: &str = env!("CARGO_BIN_EXE_physics_worker");
const BRANCHED: &str = include_str!("../../../examples/distributed_branched_tree.sdf");
const SERIAL: &str = include_str!("../../../examples/distributed_serial_tree.sdf");
const ARCHIVE: &str = "barrier.gzfloatingtree";
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[test]
fn invalid_vectors_keep_all_held_inputs_and_a_valid_vector_matches_independent_queues() -> Result<()>
{
    let worker = Path::new(EXE);
    let spec = distributed_floating_tree_spec(BRANCHED, "branch.sdf", "default")?;
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut p = FloatingTreePool::create(worker, a.path(), 3, spec.clone())?;
    let mut control = FloatingTreePool::create(worker, b.path(), 3, spec.clone())?;
    assert_eq!(p.worker_profile(), FloatingTreeWorkerProfile::VectorV2);
    let joints = spec.joints.iter().map(|j| j.joint_id).collect::<Vec<_>>();
    let (a0, a1, a2) = (joints[0], joints[1], joints[2]);
    for pool in [&mut p, &mut control] {
        pool.queue_effort(a0, f64::MAX)?;
        pool.queue_effort(a1, -0.007)?;
        pool.queue_effort(a2, 0.)?;
    }
    let before = fs::read(a.path().join(ARCHIVE))?;
    for bad in [
        vec![(a1, 0.3), (a0, f64::MAX)],
        vec![(a2, 0.04), (a2, -0.04)],
        vec![(a2, 0.02), (u64::MAX, 0.03)],
        vec![(a2, 0.), (a1, f64::NAN)],
        vec![(a0, 0.01); 64],
    ] {
        assert!(p.advance_with_efforts_at(0, 0, &bad).is_err());
        assert_eq!(fs::read(a.path().join(ARCHIVE))?, before);
        assert_eq!(p.pending_joint_effort(a0), Some(f64::MAX));
        assert_eq!(p.pending_joint_effort(a1), Some(-0.007));
        assert_eq!(p.pending_joint_effort(a2), Some(0.));
    }
    let first = [(a1, 0.016), (a0, -f64::MAX)];
    for &(j, e) in &first {
        control.queue_effort(j, e)?;
    }
    control.advance()?;
    p.advance_with_efforts_at(0, 0, &first)?;
    assert_eq!(
        fs::read(a.path().join(ARCHIVE))?,
        fs::read(b.path().join(ARCHIVE))?
    );
    // Queue order is independent for distinct joints, including omissions and empty input.
    for tick in 1..=180 {
        let mut efforts = joints
            .iter()
            .enumerate()
            .filter(|(i, _)| (tick + *i) % 4 != 0)
            .map(|(i, j)| (*j, (i as f64 - 1.) * 0.015))
            .collect::<Vec<_>>();
        if tick == 33 || tick == 121 {
            let next = (p.owner() + 1) % p.width();
            for pool in [&mut p, &mut control] {
                pool.transfer(next)?;
            }
            drop(p);
            p = FloatingTreePool::resume(worker, a.path())?;
        }
        if tick == 78 {
            let pid = p.worker_pid(p.owner()).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
        }
        if tick == 96 {
            p.drop_next_acknowledgement(p.owner())?;
        }
        if tick % 17 == 0 {
            efforts.clear();
        }
        for &(j, e) in &efforts {
            control.queue_effort(j, e)?;
        }
        efforts.reverse();
        p.advance_with_efforts_at(0, tick as u64, &efforts)?;
        control.advance()?;
        assert!(matches!(
            p.advance_with_efforts_at(0, tick as u64, &efforts),
            Err(ArticulationError::StaleInterval { .. })
        ));
        assert_eq!(
            fs::read(a.path().join(ARCHIVE))?,
            fs::read(b.path().join(ARCHIVE))?
        );
    }
    Ok(())
}
fn percentile(mut samples: Vec<u128>, quantile: usize) -> u128 {
    samples.sort_unstable();
    samples[(quantile * samples.len()).div_ceil(100) - 1]
}

#[test]
#[ignore = "release benchmark; supply original scalar worker, output CSV and benchmark storage directory"]
fn compare_independent_publications_one_publication_and_vector_worker_call() -> Result<()> {
    let scalar = std::env::var("GZ_FLOATING_TREE_SCALAR_WORKER")?;
    let storage = std::env::var("GZ_TREE_BENCH_STORAGE")?;
    let output = std::env::var("GZ_TREE_BENCH_CSV")?;
    let mut csv = String::from("scene,joints,tick,mode,elapsed_ns\n");
    for (scene, xml) in [("branched", BRANCHED), ("serial", SERIAL)] {
        let spec = distributed_floating_tree_spec(xml, scene, "default")?;
        let dirs = (0..3)
            .map(|_| tempfile::tempdir_in(&storage))
            .collect::<std::io::Result<Vec<_>>>()?;
        let mut pools = Vec::new();
        for (i, exe) in [EXE, scalar.as_str(), EXE].iter().enumerate() {
            pools.push(FloatingTreePool::create(
                Path::new(exe),
                dirs[i].path(),
                2,
                spec.clone(),
            )?);
        }
        assert_eq!(
            pools[1].worker_profile(),
            FloatingTreeWorkerProfile::ScalarV1
        );
        assert_eq!(
            pools[2].worker_profile(),
            FloatingTreeWorkerProfile::VectorV2
        );
        let modes = ["separate", "scalar_v1_batch", "vector_v2_batch"];
        let mut times: [Vec<u128>; 3] = Default::default();
        for tick in 0..216 {
            let torques = spec
                .joints
                .iter()
                .enumerate()
                .map(|(j, s)| (s.joint_id, (((j + tick) % 7) as f64 - 3.) * 0.004))
                .collect::<Vec<_>>();
            for offset in 0..3 {
                // Rotate and reverse lane order to avoid assigning a fixed position.
                let i = (tick + if tick % 2 == 0 { offset } else { 2 - offset }) % 3;
                let start = Instant::now();
                if i == 0 {
                    for &(id, e) in &torques {
                        pools[i].queue_effort(id, e)?;
                    }
                    pools[i].advance()?;
                } else {
                    pools[i].advance_with_efforts_at(0, tick as u64, &torques)?;
                }
                let elapsed = start.elapsed().as_nanos();
                if tick >= 24 {
                    times[i].push(elapsed);
                    use std::fmt::Write;
                    writeln!(
                        csv,
                        "{scene},{},{tick},{},{elapsed}",
                        spec.joints.len(),
                        modes[i]
                    )?;
                }
            }
            let expected = fs::read(dirs[0].path().join(ARCHIVE))?;
            for dir in &dirs[1..] {
                assert_eq!(fs::read(dir.path().join(ARCHIVE))?, expected);
            }
        }
        for (i, samples) in times.into_iter().enumerate() {
            let p50 = percentile(samples.clone(), 50);
            let p95 = percentile(samples.clone(), 95);
            let p99 = percentile(samples, 99);
            eprintln!(
                "scene={scene} mode={} measured_ticks=192 p50_ns={p50} p95_ns={p95} p99_ns={p99}",
                modes[i]
            );
        }
    }
    fs::write(output, csv)?;
    Ok(())
}

fn competing_vector(p: &FloatingTreePool, id: usize) -> Vec<(u64, f64)> {
    let sign = if id == 0 { 1. } else { -1. };
    p.spec()
        .joints
        .iter()
        .enumerate()
        .map(|(i, j)| (j.joint_id, sign * (i + 1) as f64 * 0.007))
        .collect()
}

#[test]
#[ignore = "child of the simultaneous vector client regression"]
fn competing_vector_process() -> Result<()> {
    use std::{io, thread, time::Duration};
    let dir = std::env::var("GZ_VECTOR_RACE_DIR")?;
    let id: usize = std::env::var("GZ_VECTOR_RACE_LANE")?.parse()?;
    let dir = Path::new(&dir);
    let worker = Path::new(EXE);
    assert!(
        matches!(FloatingTreePool::resume(worker,dir),Err(ArticulationError::Io(e)) if e.kind()==io::ErrorKind::WouldBlock)
    );
    fs::write(dir.join(format!("ready-{id}")), [])?;
    let deadline = Instant::now() + Duration::from_secs(15);
    while !dir.join("go").is_file() {
        assert!(Instant::now() < deadline, "vector race did not start");
        thread::sleep(Duration::from_millis(1));
    }
    let mut p = loop {
        match FloatingTreePool::resume(worker, dir) {
            Ok(p) => break p,
            Err(ArticulationError::Io(e))
                if e.kind() == io::ErrorKind::WouldBlock && Instant::now() < deadline =>
            {
                thread::sleep(Duration::from_millis(1))
            }
            Err(e) => return Err(e.into()),
        }
    };
    let contributions = competing_vector(&p, id);
    let won = match p.advance_with_efforts_at(1, 0, &contributions) {
        Ok(()) => true,
        Err(ArticulationError::StaleInterval {
            epoch: 1, tick: 1, ..
        }) => false,
        Err(e) => return Err(e.into()),
    };
    fs::write(
        dir.join(format!("applied-{id}")),
        if won { "yes" } else { "no" },
    )?;
    Ok(())
}

#[test]
fn two_independent_vector_clients_admit_one_whole_set_after_reset() -> Result<()> {
    use std::{thread, time::Duration};
    let worker = Path::new(EXE);
    let history = tempfile::tempdir()?;
    let mut p = FloatingTreePool::create(
        worker,
        history.path(),
        2,
        distributed_floating_tree_spec(BRANCHED, "branch.sdf", "default")?,
    )?;
    p.advance()?;
    p.reset()?;
    let joint = p.spec().joints[0].joint_id;
    p.queue_effort(joint, 0.082)?;
    let vector = competing_vector(&p, 0);
    assert!(matches!(
        p.advance_with_efforts_at(0, 0, &vector),
        Err(ArticulationError::StaleInterval {
            epoch: 1,
            tick: 0,
            ..
        })
    ));
    let control_dir = tempfile::tempdir()?;
    fs::copy(
        history.path().join(ARCHIVE),
        control_dir.path().join(ARCHIVE),
    )?;
    let mut children = Vec::new();
    for id in 0..2 {
        children.push(
            Command::new(std::env::current_exe()?)
                .args(["--exact", "--ignored", "competing_vector_process"])
                .env("GZ_VECTOR_RACE_DIR", history.path())
                .env("GZ_VECTOR_RACE_LANE", id.to_string())
                .spawn()?,
        );
    }
    let deadline = Instant::now() + Duration::from_secs(15);
    while !(0..2).all(|id| history.path().join(format!("ready-{id}")).is_file()) {
        assert!(
            Instant::now() < deadline,
            "a vector client could not reach the lease"
        );
        for child in &mut children {
            assert!(child.try_wait()?.is_none());
        }
        thread::sleep(Duration::from_millis(1));
    }
    drop(p);
    fs::write(history.path().join("go"), [])?;
    for child in &mut children {
        assert!(child.wait()?.success());
    }
    let mut winners = Vec::new();
    for id in 0..2 {
        if fs::read(history.path().join(format!("applied-{id}")))? == b"yes" {
            winners.push(id);
        }
    }
    assert_eq!(winners.len(), 1);
    let mut control = FloatingTreePool::resume(worker, control_dir.path())?;
    for (j, e) in competing_vector(&control, winners[0]) {
        control.queue_effort(j, e)?;
    }
    control.advance()?;
    assert_eq!(
        fs::read(history.path().join(ARCHIVE))?,
        fs::read(control_dir.path().join(ARCHIVE))?
    );
    Ok(())
}
