// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Error,
    physics::{
        Body, GroundPlane, MAX_PARTITION_BODIES, Primitive, SimulationStep, Snapshot, StepField,
        Tier,
    },
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
    protocol::{Command, Observation, Outcome, Request, Worker},
};
use sha2::{Digest, Sha256};
use std::{fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const RUN: u64 = 0x706f70756c;
const GRAVITY: [f64; 3] = [0., 0., -9.81];
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn sphere(id: u64, x: f64, z: f64) -> Body {
    Body {
        id,
        tier: Tier::Dynamic,
        primitive: Primitive::Sphere,
        size: [0.6; 3],
        mass: 1.,
        friction: 0.,
        position: [x, 0., z],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    }
}
fn pool(seeds: Vec<Vec<Body>>) -> Result<WorkerPool> {
    Ok(WorkerPool::spawn(
        worker(),
        RUN,
        seeds,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        256,
    )?)
}
fn issue(reference: &mut Worker, seq: &mut u64, cmd: Command) -> Result<Observation> {
    *seq += 1;
    let Outcome::State(o) = reference
        .handle(Request {
            run_id: RUN,
            worker_id: 0,
            sequence: *seq,
            command: cmd,
        })?
        .outcome
    else {
        return Err("expected native observation".into());
    };
    Ok(o)
}
fn equal_physics(pool: &PoolFrame, reference: &Observation) {
    let mut bodies: Vec<_> = pool
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .collect();
    let mut pairs: Vec<_> = pool
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    let mut supports: Vec<_> = pool
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .collect();
    bodies.sort_by_key(|b| b.id);
    pairs.sort_by_key(|p| (p.a, p.b));
    supports.sort_by_key(|s| s.body);
    let expected = &reference.snapshot;
    assert_eq!(
        bincode::serialize(&(pool.tick, bodies, pairs, supports)).unwrap(),
        bincode::serialize(&(
            expected.tick,
            &expected.bodies,
            &expected.impulses,
            &expected.supports
        ))
        .unwrap()
    );
}
fn advance(pool: &mut WorkerPool, reference: &mut Worker, seq: &mut u64) -> Result<()> {
    pool.advance_islands_in_field(GRAVITY, &[])?;
    let cmd = Command::StepIslands {
        tick: pool.frame().tick - 1,
        field: StepField {
            gravity: GRAVITY,
            wrenches: vec![],
        },
        refined: pool.last_island_refined_bodies().unwrap().to_vec(),
    };
    equal_physics(pool.frame(), &issue(reference, seq, cmd)?);
    Ok(())
}

#[test]
fn loaded_retirement_releases_every_historical_owner_and_only_the_selected_inputs() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let a = sphere(1, 0., 0.305);
    let b = sphere(2, 0., 0.97);
    let far = sphere(51, 20., 0.305);
    let mut p = pool(vec![
        vec![a.clone(), b.clone()],
        vec![far.clone()],
        vec![],
        vec![],
    ])?;
    let mut reference = Worker::new(
        RUN,
        0,
        Snapshot {
            bodies: vec![a, b, far],
            ground: Some(GroundPlane::new(0.)?),
            step: SimulationStep::Ms10,
            ..Default::default()
        },
    )?;
    let mut seq = 0;
    for _ in 0..75 {
        advance(&mut p, &mut reference, &mut seq)?;
    }
    assert!(
        p.frame().workers[0]
            .snapshot
            .impulses
            .iter()
            .any(|j| j.a == 1 && j.b == 2 && j.normal_impulse > 0.)
    );
    assert!(
        p.frame().workers[0]
            .snapshot
            .supports
            .iter()
            .any(|s| s.body == 1 && s.normal_impulse > 0.)
    );
    assert!(p.last_island_refined_bodies().unwrap().contains(&2));
    let old = dir.path().join("immutable.native");
    p.save_native_recovery(&old)?;
    assert_eq!(&fs::read(&old)?[..8], b"GZPOOL01");
    drop(p);
    let mut p = PoolNativeRecovery::open(&old)?.resume(worker())?;
    assert!(p.insert_body(sphere(40, 40., 4.)).is_err());
    for owner in [1, 2, 3] {
        p.transfer_component(1, owner)?;
    }
    assert_eq!(p.frame().workers[3].generations[&2], 3);
    p.insert_body(sphere(99, 50., 4.))?;
    p.retire(99)?;
    for (id, v) in [
        (1, [0.007, 0., 0.]),
        (2, [0., 0., 0.01]),
        (51, [0.05, 0., -0.]),
        (1, [0.01, -0., 0.]),
    ] {
        p.queue_impulse(id, v)?;
        issue(
            &mut reference,
            &mut seq,
            Command::QueueImpulse {
                tick: 75,
                id,
                impulse: v,
            },
        )?;
    }
    p.compact()?;
    let before = p.frame().clone();
    let saved = dir.path().join("before.native");
    p.save_native_recovery(&saved)?;
    let durable = fs::read(&saved)?;
    assert_eq!(&durable[..8], b"GZPOOL02");
    for owner in 0..4 {
        let history = p.retained_entries();
        p.drop_next_acknowledgement(owner)?;
        assert!(p.retire(2).is_err());
        assert_eq!(p.frame(), &before);
        assert_eq!(p.retained_entries(), history);
        let fenced = dir.path().join(format!("fenced-{owner}"));
        p.save_native_recovery(&fenced)?;
        assert_eq!(fs::read(&fenced)?, durable);
        let mut restored = PoolNativeRecovery::open(&fenced)?.resume(worker())?;
        assert!(restored.frame().body(99).is_none());
        assert!(restored.insert_body(sphere(99, 50., 4.)).is_err());
        assert_eq!(restored.frame().body(2).unwrap().0, 3);
    }
    // All surviving normal contact and ground receipts remain exactly those of a
    // single native partition that never changed its owner's location.
    p.retire(2)?;
    let expected = issue(
        &mut reference,
        &mut seq,
        Command::RetireBody {
            tick: 75,
            id: 2,
            owner: 0,
            generation: 0,
        },
    )?;
    equal_physics(p.frame(), &expected);
    let after = dir.path().join("after.native");
    p.save_native_recovery(&after)?;
    let payload: serde_json::Value = serde_json::from_slice(&fs::read(&after)?[40..])?;
    assert_eq!(payload["population_high_water"], 99);
    for image in payload["images"].as_array().unwrap() {
        assert!(image["retired"].get("2").is_none());
    }
    assert!(
        payload["queued"]
            .as_array()
            .unwrap()
            .iter()
            .all(|p| p["id"] != 2)
    );
    assert_eq!(payload["queued"].as_array().unwrap().len(), 3);
    for owner in 0..4 {
        p.recover(owner)?;
    }
    let mut restored = PoolNativeRecovery::open(after)?.resume(worker())?;
    for _ in 0..30 {
        advance(&mut p, &mut reference, &mut seq)?;
        assert_eq!(restored.advance_islands_in_field(GRAVITY, &[])?, p.frame());
    }
    assert!(restored.insert_body(sphere(98, 50., 4.)).is_err());
    restored.insert_body(sphere(100, 50., 4.))?;
    Ok(())
}

#[test]
fn birth_checks_global_clearance_restores_empty_archives_and_never_reuses_ids() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let mut p = pool(vec![vec![], vec![], vec![]])?;
    let empty = dir.path().join("empty-v1");
    p.save_native_recovery(&empty)?;
    let mut p = PoolNativeRecovery::open(&empty)?.resume(worker())?;
    let zero = sphere(0, 5., 2.);
    p.drop_next_acknowledgement(0)?;
    assert!(p.insert_body(zero.clone()).is_err());
    assert!(p.frame().body(0).is_none());
    p.insert_body(zero.clone())?;
    let before = p.frame().clone();
    let used: Vec<_> = p.traffic().iter().map(|t| t.requests).collect();
    assert!(p.insert_body(sphere(1, 5., 2.)).is_err());
    assert!(
        p.insert_body_in_field(sphere(1, 30., 4.), [f64::NAN, 0., 0.], &[])
            .is_err()
    );
    assert!(p.insert_body(zero.clone()).is_err());
    assert_eq!(p.frame(), &before);
    assert_eq!(
        p.traffic().iter().map(|t| t.requests).collect::<Vec<_>>(),
        used
    );
    p.retire(0)?;
    assert!(p.retire(0).is_err());
    let edited = dir.path().join("empty-v2");
    p.save_native_recovery(&edited)?;
    let mut p = PoolNativeRecovery::open(&edited)?.resume(worker())?;
    assert!(p.insert_body(zero).is_err());
    p.insert_body(sphere(u64::MAX, 5., 2.))?;
    p.retire(u64::MAX)?;
    let final_file = dir.path().join("exhausted");
    p.save_native_recovery(&final_file)?;
    let mut p = PoolNativeRecovery::open(&final_file)?.resume(worker())?;
    assert!(p.insert_body(sphere(1, 0., 3.)).is_err());
    p.advance()?;
    // Missing allocator in edited v2, or an allocator attached to legacy data.
    for (index, (magic, edit)) in [(b"GZPOOL02", None), (b"GZPOOL01", Some(0))]
        .into_iter()
        .enumerate()
    {
        let mut value: serde_json::Value = serde_json::from_slice(&fs::read(&edited)?[40..])?;
        if let Some(v) = edit {
            value["population_high_water"] = v.into();
        } else {
            value
                .as_object_mut()
                .unwrap()
                .remove("population_high_water");
        }
        let body = serde_json::to_vec(&value)?;
        let mut bytes = magic.to_vec();
        bytes.extend(Sha256::digest(&body));
        bytes.extend(body);
        let path = dir.path().join(format!("malformed-{index}"));
        fs::write(&path, bytes)?;
        assert!(PoolNativeRecovery::open(path).is_err());
    }
    Ok(())
}

#[test]
fn admission_reserves_retired_capacity_on_all_owners_then_releases_it_on_permanent_retirement()
-> Result<()> {
    let n = MAX_PARTITION_BODIES;
    let mut seeds = vec![vec![], vec![], vec![]];
    for (w, shard) in seeds.iter_mut().enumerate() {
        let count = if w == 1 { n - 1 } else { n };
        for i in 0..count {
            let id = (w * n + i) as u64;
            shard.push(sphere(id, 2. * id as f64, 3.));
        }
    }
    let mut p = pool(seeds)?;
    p.transfer_component(0, 1)?;
    let fresh = sphere((3 * n) as u64, 1e4, 3.);
    let before = p.frame().clone();
    assert!(matches!(p.insert_body(fresh.clone()), Err(Error::Capacity)));
    assert_eq!(p.frame(), &before);
    p.retire(0)?;
    p.insert_body(fresh.clone())?;
    assert_eq!(p.frame().body(fresh.id).unwrap().0, 0);
    Ok(())
}

#[test]
fn a_fast_birth_uses_the_projected_routes_and_next_barrier_command_capacity() -> Result<()> {
    let first = sphere(1, 0., 3.);
    let mut moving = sphere(2, 0.67, 3.);
    moving.linear = [-10., 0., 0.];
    for (limit, initial_owner) in [(4, 1), (3, 0)] {
        let mut p = WorkerPool::spawn(
            worker(),
            RUN,
            vec![vec![first.clone()], vec![], vec![]],
            SimulationStep::Ms10,
            None,
            limit,
        )?;
        p.insert_body_in_field(moving.clone(), [0.; 3], &[])?;
        assert_eq!(p.frame().body(2).unwrap().0, initial_owner);
        for _ in 0..3 {
            p.advance()?;
        }
        assert_eq!(p.frame().body(1).unwrap().0, p.frame().body(2).unwrap().0);
    }
    Ok(())
}

#[cfg(unix)]
#[test]
#[ignore = "child coordinator for the population SIGKILL integration test"]
fn population_crash_child() {
    let dir = std::path::PathBuf::from(std::env::var_os("GZ_POPULATION_CRASH_DIR").unwrap());
    let committed = std::env::var_os("GZ_POPULATION_COMMITTED").is_some();
    let mut p = pool(vec![
        vec![sphere(1, 0., 3.)],
        vec![],
        vec![sphere(51, 15., 3.)],
        vec![],
    ])
    .unwrap();
    p.advance().unwrap();
    p.insert_body(sphere(99, 25., 3.)).unwrap();
    p.retire(99).unwrap();
    p.transfer_component(1, 1).unwrap();
    p.transfer_component(1, 3).unwrap();
    p.queue_impulse(51, [0.4, 0., 0.]).unwrap();
    if committed {
        p.retire(1).unwrap();
    }
    p.save_native_recovery(dir.join("last.native")).unwrap();
    // The last participant's retirement may already have been applied, but its
    // missing acknowledgement leaves the saved barrier as the only authority.
    p.drop_next_acknowledgement(3).unwrap();
    assert!(p.retire(if committed { 51 } else { 1 }).is_err());
    let endpoints: Vec<_> = (0..4).map(|w| p.worker_endpoint(w).unwrap()).collect();
    fs::write(dir.join("pending"), serde_json::to_vec(&endpoints).unwrap()).unwrap();
    fs::rename(dir.join("pending"), dir.join("ready")).unwrap();
    loop {
        std::thread::park();
    }
}

#[cfg(unix)]
#[test]
fn new_coordinator_restores_single_authority_and_the_issued_ceiling_after_sigkill() -> Result<()> {
    use std::{
        net::{SocketAddr, TcpStream},
        process::{Child, Command as Process},
        thread,
        time::{Duration, Instant},
    };
    struct ChildGuard(Child);
    impl Drop for ChildGuard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    for committed in [false, true] {
        let dir = tempfile::tempdir()?;
        let mut cmd = Process::new(std::env::current_exe()?);
        cmd.args([
            "--exact",
            "--ignored",
            "population_crash_child",
            "--nocapture",
        ])
        .env("GZ_POPULATION_CRASH_DIR", dir.path());
        if committed {
            cmd.env("GZ_POPULATION_COMMITTED", "1");
        } else {
            cmd.env_remove("GZ_POPULATION_COMMITTED");
        }
        let mut child = ChildGuard(cmd.spawn()?);
        let deadline = Instant::now() + Duration::from_secs(25);
        while !dir.path().join("ready").exists() {
            if let Some(status) = child.0.try_wait()? {
                return Err(format!("coordinator exited early: {status}").into());
            }
            if Instant::now() > deadline {
                return Err("coordinator did not reach its ambiguous retirement".into());
            }
            thread::sleep(Duration::from_millis(5));
        }
        let endpoints: Vec<String> = serde_json::from_slice(&fs::read(dir.path().join("ready"))?)?;
        child.0.kill()?;
        child.0.wait()?;
        for address in endpoints {
            let endpoint: SocketAddr = address
                .strip_prefix("tcp/")
                .ok_or("non-TCP local worker")?
                .parse()?;
            while TcpStream::connect_timeout(&endpoint, Duration::from_millis(50)).is_ok() {
                if Instant::now() > deadline {
                    return Err("old worker still accepts connections".into());
                }
                thread::sleep(Duration::from_millis(5));
            }
        }
        let mut restored =
            PoolNativeRecovery::open(dir.path().join("last.native"))?.resume(worker())?;
        assert!(restored.frame().body(99).is_none());
        assert!(restored.insert_body(sphere(99, 25., 3.)).is_err());
        assert_eq!(
            restored.frame().body(1).map(|(owner, _)| owner),
            if committed { None } else { Some(3) }
        );
        if !committed {
            assert_eq!(restored.frame().workers[3].generations[&1], 2);
        }
        assert!(restored.frame().body(51).is_some());
        restored.advance()?;
        assert_eq!(restored.frame().body(51).unwrap().1.linear[0], 0.4);
        restored.advance()?;
        assert_eq!(restored.frame().body(51).unwrap().1.linear[0], 0.4);
        restored.insert_body(sphere(100, 25., 3.))?;
    }
    Ok(())
}

// A restored allocator must dominate both live IDs and the latest refinement
// membership, which can still name a body just retired at this public barrier.
#[test]
fn edited_allocator_cannot_trail_live_or_just_retired_refined_body() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let mut p = pool(vec![vec![sphere(3, 0., 0.42)], vec![], vec![]])?;
    p.insert_body(sphere(17, 6., 0.42))?;
    for _ in 0..40 {
        p.advance_islands_in_field(GRAVITY, &[])?;
    }
    assert!(
        p.last_island_refined_bodies()
            .is_some_and(|ids| ids.contains(&17))
    );
    for retired in [false, true] {
        if retired {
            p.retire(17)?;
        }
        let path = dir.path().join(format!("valid-{retired}"));
        p.save_native_recovery(&path)?;
        let (mut data, tag) = {
            let bytes = fs::read(&path)?;
            (
                serde_json::from_slice::<serde_json::Value>(&bytes[40..])?,
                bytes[..8].to_vec(),
            )
        };
        assert_eq!(data["population_high_water"], 17);
        data["population_high_water"] = 16.into();
        let data = serde_json::to_vec(&data)?;
        let mut bytes = tag;
        bytes.extend(Sha256::digest(&data));
        bytes.extend(data);
        let wrong = dir.path().join(format!("backwards-{retired}"));
        fs::write(&wrong, bytes)?;
        assert!(matches!(
            PoolNativeRecovery::open(&wrong),
            Err(gz_physics_distributed::coordinator::RecoveryError::Format(
                "pool population allocator behind worker state"
            ))
        ));
        assert!(PoolNativeRecovery::open(&path).is_ok());
    }
    Ok(())
}
