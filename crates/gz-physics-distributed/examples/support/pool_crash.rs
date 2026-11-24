// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One coordinator is killed after three shards may have advanced beyond a saved barrier.
use gz_physics_distributed::{
    physics::{Body, GroundPlane, Primitive, SimulationStep, fixture},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{
    error::Error,
    fs::{self, File},
    io,
    net::{SocketAddr, TcpStream},
    path::Path,
    process::{Command, Stdio},
    thread,
    time::{Duration, Instant},
};
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;
const BARRIER: u64 = 140;
const END: u64 = 230;
// Gravity presses each cargo back onto its advancing pusher after impact.
#[derive(Clone, Copy, PartialEq)]
pub enum Profile {
    Coarse,
    Adaptive,
    CurvedGround,
    Islands,
    DiagonalLanes,
}

impl Profile {
    fn has_ground(self) -> bool {
        matches!(self, Self::CurvedGround) || self.island_scope()
    }

    fn island_scope(self) -> bool {
        matches!(self, Self::Islands | Self::DiagonalLanes)
    }

    fn check_island_receipt(self, pool: &WorkerPool) {
        if self.island_scope() {
            assert_eq!(pool.last_island_refined_bodies(), Some(&[1, 2, 11, 12][..]));
            assert!(pool.frame().body(3).is_some());
        }
        if self == Self::DiagonalLanes {
            let frame = pool.frame();
            let mut fragments: Vec<_> = frame
                .workers
                .iter()
                .flat_map(|w| w.components.iter().map(|c| c.ids.as_slice()))
                .collect();
            fragments.sort_unstable();
            assert_eq!(
                fragments,
                [&[1, 2][..], &[3], &[11, 12]],
                "barrier {}",
                frame.tick
            );

            for (pusher, cargo) in [(1, 2), (11, 12)] {
                let (owner, _) = frame.body(cargo).unwrap();
                let snapshot = &frame.workers[usize::from(owner)].snapshot;
                assert!(
                    snapshot
                        .impulses
                        .iter()
                        .any(|p| (p.a, p.b) == (pusher, cargo) && p.normal_impulse > 0.),
                    "missing pair load at {} on body {cargo}",
                    frame.tick
                );
                assert!(
                    snapshot
                        .supports
                        .iter()
                        .any(|p| p.body == cargo && p.normal_impulse > 0.),
                    "missing ground load at {} on body {cargo}",
                    frame.tick
                );
            }

            // Both loaded cargoes can deflect. The old axis-aligned travel cubes
            // overlap, but even their maximum native COM travel cannot bridge
            // the diagonal gap in this barrier.
            let (_, a) = frame.body(2).unwrap();
            let (_, b) = frame.body(12).unwrap();
            let delta: [f64; 3] = std::array::from_fn(|i| a.position[i] - b.position[i]);
            let dt = frame.step().duration().as_secs_f64();
            let reach = (a.size[0] + b.size[0]) * 0.5
                + 2. * gz_physics_multifidelity::partition_linear_speed_limit() * dt
                + gz_physics_multifidelity::partition_prediction_distance();
            assert!(delta.iter().all(|d| d.abs() < reach));
            assert!(delta[0].hypot(delta[1]).hypot(delta[2]) > reach);
        }
    }

    fn gravity(self) -> [f64; 3] {
        [-1.5, 0., if self.has_ground() { -9.81 } else { 0. }]
    }
}
const SAVE: &str = "pool.native";
const READY: &str = "ready.json";

#[derive(Serialize, Deserialize)]
struct Original {
    coordinator: u32,
    endpoints: Vec<String>,
    pids: Vec<u32>,
    frames: Vec<(String, String)>,
}

fn seeds(profile: Profile) -> Vec<Vec<Body>> {
    let pair = fixture();
    let mut workers: Vec<_> = (0..4)
        .map(|i| {
            let mut b = pair[i % 2].clone();
            b.id += 10 * (i as u64 / 2);
            b.position[1] = if i < 2 { -6. } else { 6. };
            if profile == Profile::DiagonalLanes {
                let offset = if i < 2 { -3.75 } else { 3.75 };
                b.position[0] += offset;
                b.position[1] = offset;
            }
            if profile.has_ground() {
                b.primitive = Primitive::Sphere;
                b.position[2] = 0.305;
            }
            vec![b]
        })
        .collect();
    if profile.island_scope() {
        let mut free = pair[1].clone();
        free.id = 3;
        free.position = [0., workers[1][0].position[1], 40.];
        // Shares a worker with the loaded pair at the checkpoint and stays in free flight.
        workers[1].push(free);
    }
    workers
}
fn spawn(worker: &Path, profile: Profile) -> Result<WorkerPool> {
    Ok(WorkerPool::spawn(
        worker,
        2026111914,
        seeds(profile),
        SimulationStep::Ms10,
        profile
            .has_ground()
            .then(|| GroundPlane::new(0.))
            .transpose()?,
        29,
    )?)
}
fn prepare(pool: &mut WorkerPool, profile: Profile) -> Result<()> {
    // The first loaded pair visits two other workers. Source and destination
    // have different transfer serials from the undisturbed second pair.
    for destination in [3, 1] {
        pool.transfer_component(1, destination)?;
    }
    assert_ne!(
        pool.frame().body(2).unwrap().0,
        pool.frame().body(12).unwrap().0
    );
    // Preserve global command order even though owners differ at this barrier.
    for (id, z) in [(2, 0.04), (12, -0.06), (2, -0.01), (12, 0.02)] {
        pool.queue_impulse_in_gravity(id, [0., 0., z], profile.gravity())?;
    }
    Ok(())
}
fn sample(frame: &PoolFrame, sequence: u64, profile: Profile) -> (String, String) {
    let mut visuals = Vec::new();
    let mut models = Vec::new();
    for (i, id) in [1, 2, 11, 12].into_iter().enumerate() {
        let (owner, b) = frame.body(id).unwrap();
        let [x, physical_y, z] = b.position;
        // Translate the complete lane for display, leaving every relative motion intact.
        let y = if profile == Profile::DiagonalLanes {
            physical_y
        } else {
            physical_y - if id < 10 { -5.15 } else { 5.15 }
        };
        let core = 100 + i as u64 * 10;
        let name = format!("body_{id}");
        let make = |id: u64,
                    part: &str,
                    p: [f64; 3],
                    q: [f64; 4],
                    size: [f64; 3],
                    color: [f64; 4]| json!({"id":id,"name":format!("default::{name}::{part}"),"translation":p,"rotation":q,"shape":{"kind":"box","size":size},"color":color});
        let mut body_visual = make(
            core + 1,
            "body",
            [x, y, z],
            b.orientation,
            b.size,
            if i % 2 == 0 {
                [0.93, 0.36, 0.08, 1.]
            } else {
                [0.10, 0.43, 0.89, 1.]
            },
        );
        if b.primitive == Primitive::Sphere {
            body_visual["shape"] = json!({"kind":"sphere","radius":b.size[0]*0.5});
        }
        visuals.push(body_visual);
        let mut ids = vec![core + 1];
        for (w, rgb) in [
            [0.96, 0.84, 0.18],
            [0.19, 0.9, 0.65],
            [0.8, 0.42, 0.93],
            [0.98, 0.59, 0.70],
        ]
        .iter()
        .enumerate()
        {
            let marker = core + 2 + w as u64;
            let h = if w == owner as usize { 0.41 } else { 0. };
            visuals.push(make(
                marker,
                &format!("worker_{w}"),
                [x, y, z + h],
                [0., 0., 0., 1.],
                [0.17; 3],
                [rgb[0], rgb[1], rgb[2], 1.],
            ));
            ids.push(marker);
        }
        models.push(json!({"model_id":core,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
    }
    let ns = frame.tick * 10_000_000;
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,"paused":false,"visuals":visuals});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"models":models,"templates":[]});
    (scene.to_string(), catalog.to_string())
}

fn advance(pool: &mut WorkerPool, profile: Profile) -> Result<&PoolFrame> {
    Ok(match profile {
        Profile::Islands | Profile::DiagonalLanes => {
            pool.advance_islands_in_field(profile.gravity(), &[])?
        }
        Profile::Coarse => pool.advance_in_gravity(profile.gravity())?,
        Profile::Adaptive | Profile::CurvedGround => {
            pool.advance_adaptive_in_field(profile.gravity(), &[])?
        }
    })
}

pub fn original(worker: &Path, dir: &Path, profile: Profile) -> Result<()> {
    let mut pool = spawn(worker, profile)?;
    let mut frames = vec![sample(pool.frame(), 1, profile)];
    for n in 1..=BARRIER {
        advance(&mut pool, profile)?;
        if n.is_multiple_of(2) {
            frames.push(sample(pool.frame(), frames.len() as u64 + 1, profile));
        }
    }
    assert_eq!(
        pool.frame()
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .filter(|p| p.normal_impulse > 0.)
            .count(),
        2
    );
    assert_eq!(
        pool.last_substeps(),
        Some(if profile != Profile::Coarse { 10 } else { 1 })
    );
    let support_count = |p: &WorkerPool| {
        p.frame()
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .filter(|j| j.normal_impulse > 0.)
            .count()
    };
    if profile.has_ground() {
        assert_eq!(support_count(&pool), 2);
    }
    prepare(&mut pool, profile)?;
    if profile.has_ground() {
        assert_eq!(support_count(&pool), 2);
    }
    *frames.last_mut().unwrap() = sample(pool.frame(), BARRIER / 2 + 1, profile);
    profile.check_island_receipt(&pool);
    let entries = pool.retained_entries();
    pool.save_native_recovery(dir.join(SAVE))?;
    assert_eq!(entries, pool.retained_entries());
    assert!(pool.save_native_recovery(dir.join(SAVE)).is_err());
    let evidence = Original {
        coordinator: std::process::id(),
        endpoints: (0..4)
            .map(|w| pool.worker_endpoint(w).unwrap().to_owned())
            .collect(),
        pids: (0..4).map(|w| pool.worker_pid(w).unwrap()).collect(),
        frames,
    };
    // Abort after actual remote execution on a shard, before publishing its reply.
    pool.drop_next_acknowledgement(2)?;
    assert!(advance(&mut pool, profile).is_err());
    assert_eq!(pool.frame().tick, BARRIER);
    let mut tmp = tempfile::NamedTempFile::new_in(dir)?;
    serde_json::to_writer(tmp.as_file_mut(), &evidence)?;
    tmp.persist_noclobber(dir.join(READY))?;
    io::copy(&mut io::stdin().lock(), &mut io::sink())?;
    Ok(())
}

pub fn replacement(worker: &Path, dir: &Path, profile: Profile) -> Result<()> {
    let old: Original = serde_json::from_reader(File::open(dir.join(READY))?)?;
    let mut uninterrupted = spawn(worker, profile)?;
    for _ in 0..BARRIER {
        advance(&mut uninterrupted, profile)?;
    }
    prepare(&mut uninterrupted, profile)?;
    let recovery = PoolNativeRecovery::open(dir.join(SAVE))?;
    assert_eq!(recovery.frame(), uninterrupted.frame());
    let mut current = recovery.resume(worker)?;
    profile.check_island_receipt(&current);
    assert!(current.retained_entries().iter().all(|&n| n == 0));
    assert_eq!(current.last_substeps(), uninterrupted.last_substeps());
    let mut refined = 0;
    let mut frames = old.frames;
    let mut moving_impulse = 0_f64;
    let mut supported_ticks = 0;
    for step in BARRIER + 1..=END {
        // The staged inputs were initially on different workers. A fresh post-
        // restore transfer must merge their relative order without reapplying them.
        if step == BARRIER + 1 {
            for pool in [&mut current, &mut uninterrupted] {
                pool.transfer_component(11, 1)?;
            }
        }
        if step == 180 {
            current.compact()?;
            current.recover(1)?;
        }
        let a = advance(&mut current, profile)?;
        let b = advance(&mut uninterrupted, profile)?;
        assert_eq!(
            a, b,
            "complete worker state, caches' observable impulses and ownership at {step}"
        );
        supported_ticks += u64::from(
            a.workers
                .iter()
                .any(|w| w.snapshot.supports.iter().any(|j| j.normal_impulse > 0.)),
        );
        for w in &a.workers {
            for j in &w.snapshot.impulses {
                moving_impulse = moving_impulse.max(j.normal_impulse);
            }
        }
        if step.is_multiple_of(2) {
            frames.push(sample(a, frames.len() as u64 + 1, profile));
        }
        profile.check_island_receipt(&current);
        assert_eq!(
            current.last_island_refined_bodies(),
            uninterrupted.last_island_refined_bodies()
        );
        assert_eq!(current.last_substeps(), uninterrupted.last_substeps());
        refined += usize::from(current.last_substeps() == Some(10));
    }
    assert!(moving_impulse > 0.);
    assert_eq!(refined > 0, profile != Profile::Coarse);
    if profile.has_ground() {
        assert_eq!(supported_ticks, END - BARRIER);
    }
    let new_pids: Vec<_> = (0..4).map(|w| current.worker_pid(w).unwrap()).collect();
    assert!(new_pids.iter().all(|pid| !old.pids.contains(pid)));
    let count = frames.len();
    gz_sim_cli::export_archive(&dir.join("replay"), Default::default(), frames)?;
    let mut results = json!({"width":4,"adaptive_contacts":profile!=Profile::Coarse,"curved_ground":profile.has_ground(),"island_scope":profile.island_scope(),"last_refined_ids":current.last_island_refined_bodies(),"resumed_ground_support_ticks":supported_ticks,"resumed_refined":refined,"barrier":BARRIER,"resumed_exact":END-BARRIER,"matched_body_frames":current.frame().workers.iter().map(|w|w.snapshot.bodies.len() as u64).sum::<u64>()*(END-BARRIER),"native_bytes":fs::metadata(dir.join(SAVE))?.len(),"frames":count,"old_coordinator":old.coordinator,"new_coordinator":std::process::id(),"prior_workers":old.pids,"replacement_workers":new_pids,"largest_resumed_impulse_ns":moving_impulse});
    if profile == Profile::DiagonalLanes {
        results["diagonal_lane_offset_m"] = json!([7.5, 7.5, 0.]);
        results["verified_separate_pair_barriers"] = json!(1 + END - BARRIER);
    }
    fs::write(
        dir.join("result.json"),
        serde_json::to_vec_pretty(&results)?,
    )?;
    println!("{results}");
    Ok(())
}
pub fn supervise(mut original: Command, mut replacement: Command, dir: &Path) -> Result<()> {
    fs::create_dir_all(dir)?;
    if [SAVE, READY, "result.json", "replay"]
        .iter()
        .any(|name| dir.join(name).exists())
    {
        return Err("checkpoint demonstration needs a fresh output directory".into());
    }
    let mut child = original.stdin(Stdio::piped()).spawn()?;
    let wait = (|| -> Result<Original> {
        let deadline = Instant::now() + Duration::from_secs(45);
        while !dir.join(READY).exists() {
            if let Some(status) = child.try_wait()? {
                return Err(format!("original coordinator exited: {status}").into());
            }
            if Instant::now() > deadline {
                return Err("checkpoint deadline".into());
            }
            thread::sleep(Duration::from_millis(20));
        }
        Ok(serde_json::from_reader(File::open(dir.join(READY))?)?)
    })();
    if child.try_wait()?.is_none() {
        child.kill()?;
    }
    let status = child.wait()?;
    let evidence = wait?;
    #[cfg(unix)]
    {
        use std::os::unix::process::ExitStatusExt;
        assert_eq!(status.signal(), Some(9));
    }
    let until = Instant::now() + Duration::from_secs(8);
    for endpoint in evidence.endpoints {
        let addr: SocketAddr = endpoint
            .strip_prefix("tcp/")
            .ok_or("unexpected worker endpoint")?
            .parse()?;
        loop {
            match TcpStream::connect_timeout(&addr, Duration::from_millis(100)) {
                Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => break,
                // A listener can reset the probe between connect and close. Wait
                // for a definite refusal before allowing the replacement to bind.
                Err(e)
                    if !matches!(
                        e.kind(),
                        io::ErrorKind::ConnectionReset
                            | io::ErrorKind::ConnectionAborted
                            | io::ErrorKind::TimedOut
                            | io::ErrorKind::Interrupted
                    ) =>
                {
                    return Err(e.into());
                }
                _ if Instant::now() >= until => {
                    return Err("former worker endpoint did not close".into());
                }
                _ => thread::sleep(Duration::from_millis(20)),
            }
        }
    }
    let status = replacement.status()?;
    if !status.success() {
        return Err(format!("replacement coordinator exited: {status}").into());
    }
    Ok(())
}
