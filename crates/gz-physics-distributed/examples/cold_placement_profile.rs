// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare distributed SDF cold owners through the normal Bevy scene loader.
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;

use clap::{Parser, Subcommand};
use fixture::Layout;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, Primitive},
    protocol::Observation,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fmt::Write as _,
    fs::File,
    io::{BufReader, BufWriter, Write as _},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Generate the public SDF geometry used by run and compare.
    Sdf {
        #[arg(long, value_enum, default_value = "adjacent")]
        layout: Layout,
        output: PathBuf,
    },
    /// Run one complete trajectory with ordinary scene commands and optional world reset.
    Run {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, value_enum, default_value = "adjacent")]
        layout: Layout,
        #[arg(long, default_value_t=4,value_parser=clap::value_parser!(u8).range(2..=8))]
        workers: u8,
        #[arg(long)]
        contact_placement: bool,
        #[arg(long)]
        verify_reset: bool,
        /// Withhold a worker's next command reply before the specified public tick.
        #[arg(long, value_parser=clap::value_parser!(u64).range(1..=fixture::END))]
        lose_ack_at: Option<u64>,
        /// Write observed owner changes keyed by the canonical, one-based fixture ID.
        #[arg(long)]
        owner_trace: Option<PathBuf>,
    },
    /// Reuse one loaded Bevy simulation through exact public resets, forces and worker faults.
    Endurance(EnduranceArgs),
    /// Compare the public ECS step that internally recovers a lost worker reply.
    RetryTrials {
        #[arg(long)]
        reference: PathBuf,
        #[arg(long)]
        candidate: PathBuf,
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, value_enum)]
        layout: Layout,
        #[arg(long, default_value_t=2,value_parser=clap::value_parser!(u8).range(2..=8))]
        workers: u8,
        #[arg(long, default_value_t=16,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        output: PathBuf,
    },
    /// Compare two binaries through ordinary SDF construction, reset and the full 350-tick trajectory.
    StartTrials {
        #[arg(long)]
        reference: PathBuf,
        #[arg(long)]
        candidate: PathBuf,
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, value_enum)]
        layout: Layout,
        #[arg(long, default_value_t=4,value_parser=clap::value_parser!(u8).range(2..=8))]
        workers: u8,
        #[arg(long, default_value_t=24,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        output: PathBuf,
    },
    /// Summarize the paired observations written by compare.
    Summary { input: PathBuf },
    /// Alternate baseline and contact placement; verify all hashes before reporting.
    Compare {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, value_enum)]
        layout: Layout,
        #[arg(long, default_value_t=4,value_parser=clap::value_parser!(u8).range(2..=8))]
        workers: u8,
        #[arg(long, default_value_t=8,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        output: PathBuf,
    },
}
#[derive(clap::Args)]
struct EnduranceArgs {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: Layout,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(2..=8))]
    workers: u8,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u32).range(1..))]
    epochs: u32,
    #[arg(long,conflicts_with="epochs",value_parser=clap::value_parser!(u32).range(1..))]
    seconds: Option<u32>,
    #[arg(long, default_value_t = 0)]
    pace_ms: u16,
    output: PathBuf,
}
const SURFACE: &str = "<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>";
fn xml(groups: &[Vec<Body>]) -> String {
    let [gx, gy, gz] = fixture::GRAVITY;
    let mut out = format!(
        "<?xml version=\"1.0\"?>\n<!-- Copyright (C) 2026 Open Source Robotics Foundation -->\n<!-- SPDX-License-Identifier: Apache-2.0 -->\n<sdf version=\"1.12\"><world name=\"default\"><gravity>{gx} {gy} {gz}</gravity>\n<physics name=\"ten_ms\" type=\"ode\"><max_step_size>0.01</max_step_size></physics>\n<model name=\"ground\"><static>true</static><link name=\"floor\"><collision name=\"floor\"><geometry><plane><normal>0 0 1</normal><size>64 32</size></plane></geometry>{SURFACE}</collision><visual name=\"ground\"><pose>0 0 -0.025 0 0 0</pose><geometry><box><size>64 32 0.05</size></box></geometry><material><diffuse>0.3 0.34 0.38 1</diffuse></material></visual></link></model>\n"
    );
    for b in groups.iter().flatten() {
        let [x, y, z] = b.position;
        let [sx, sy, sz] = b.size;
        let m = b.mass;
        let (geometry, inset, color) = match b.primitive {
            Primitive::Box => (
                format!("<box><size>{sx} {sy} {sz}</size></box>"),
                [
                    (sy * sy + sz * sz) * m / 12.,
                    (sx * sx + sz * sz) * m / 12.,
                    (sx * sx + sy * sy) * m / 12.,
                ],
                "0.19 0.48 0.67",
            ),
            Primitive::Sphere => (
                format!("<sphere><radius>{}</radius></sphere>", sx / 2.),
                [0.4 * m * (sx / 2.).powi(2); 3],
                "0.93 0.57 0.18",
            ),
            _ => unreachable!("fixture uses only slabs and spheres"),
        };
        let [ix, iy, iz] = inset;
        writeln!(out,"<model name=\"body_{:03}\"><pose>{x} {y} {z} 0 0 0</pose><link name=\"link\"><inertial><mass>{m}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz></inertia></inertial><collision name=\"shape\"><geometry>{geometry}</geometry>{SURFACE}</collision><visual name=\"paint\"><geometry>{geometry}</geometry><material><diffuse>{color} 1</diffuse></material></visual></link></model>",b.id).unwrap();
    }
    out.push_str("</world></sdf>\n");
    out
}
fn workers(sim: &Simulation) -> &[Observation] {
    let engine = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    if let Some(pool) = engine.pool_frame() {
        &pool.workers
    } else {
        &engine.frame().unwrap().workers
    }
}

#[derive(Serialize, Deserialize)]
struct Timing {
    placement: String,
    layout: Layout,
    workers: u8,
    load_us: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    reset_us: Option<f64>,
    first_step_us: f64,
    measured_elapsed_ms: f64,
    measured_p95_us: f64,
    initial_counts: Vec<usize>,
    initial_pad_owners: [Vec<usize>; 2],
    first_step_moved_bodies: usize,
    total_moved_bodies: usize,
    pair_receipts: usize,
    floor_receipts: usize,
    physics_sha256: String,
    ecs_poses_sha256: String,
    #[serde(default)]
    public_ids_sha256: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    fault: Option<Fault>,
}
#[derive(Serialize, Deserialize)]
struct Fault {
    tick: u64,
    worker: u8,
    public_step_us: f64,
    replaced_workers: usize,
}

fn owners(sim: &Simulation, count: usize) -> Vec<usize> {
    let mut ids = vec![usize::MAX; count];
    for (worker, obs) in workers(sim).iter().enumerate() {
        for body in &obs.snapshot.bodies {
            ids[body.id as usize] = worker;
        }
    }
    ids
}
struct SceneCase {
    groups: Vec<Vec<Body>>,
    layout: Layout,
    width: u8,
    contact: bool,
}
impl SceneCase {
    fn new(layout: Layout, width: u8, contact: bool) -> Self {
        Self {
            groups: fixture::seeds(layout),
            layout,
            width,
            contact,
        }
    }
    fn load(&self, worker: &Path) -> Result<(Simulation, f64)> {
        let sdf = xml(&self.groups);
        let at = Instant::now();
        let sim = load_world_with_runtime(
            &sdf,
            "cold-placement.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::DistributedIslands(self.width),
                step: None,
                noise_seed: [0; 32],
                distributed_worker: Some(worker),
            },
            WorldRuntimeOptions {
                distributed_contact_placement: self.contact,
                ..Default::default()
            },
        )?;
        Ok((sim, at.elapsed().as_secs_f64() * 1e6))
    }
}
#[derive(Default)]
struct TrajectoryOptions<'a> {
    verify_reset: bool,
    owner_trace: Option<&'a Path>,
    lose_ack_at: Option<u64>,
    kill: bool,
    pace: Duration,
}
#[cfg(target_os = "linux")]
fn kill_worker(pid: u32) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let fd = pidfd_open(
        Pid::from_raw(i32::try_from(pid)?).ok_or("invalid process ID")?,
        PidfdFlags::empty(),
    )?;
    pidfd_send_signal(&fd, Signal::KILL)?;
    Ok(())
}
#[cfg(not(target_os = "linux"))]
fn kill_worker(_: u32) -> Result<()> {
    Err("process-kill qualification requires a Linux pidfd".into())
}

fn run(
    worker: &Path,
    layout: Layout,
    width: u8,
    contact: bool,
    verify_reset: bool,
    owner_trace: Option<&Path>,
    lose_ack_at: Option<u64>,
) -> Result<Timing> {
    let case = SceneCase::new(layout, width, contact);
    let (mut sim, load_us) = case.load(worker)?;
    trajectory(
        &case,
        &mut sim,
        load_us,
        TrajectoryOptions {
            verify_reset,
            owner_trace,
            lose_ack_at,
            ..Default::default()
        },
    )
}
fn trajectory(
    case: &SceneCase,
    sim: &mut Simulation,
    load_us: f64,
    options: TrajectoryOptions<'_>,
) -> Result<Timing> {
    let TrajectoryOptions {
        verify_reset,
        owner_trace,
        lose_ack_at,
        kill,
        pace,
    } = options;
    let (groups, layout, width, contact) = (&case.groups, case.layout, case.width, case.contact);
    let count = groups.iter().map(Vec::len).sum::<usize>();
    let mut initial = owners(sim, count);
    if initial.contains(&usize::MAX) {
        return Err("invalid initial SDF owner map".into());
    }
    let reset_us = if verify_reset {
        sim.step_exact(3)?;
        let at = Instant::now();
        reset_loaded_world(sim, |_| ())?;
        let elapsed = at.elapsed().as_secs_f64() * 1e6;
        if owners(sim, count) != initial || !sim.elapsed().is_zero() {
            return Err("reset changed initial ownership or clock".into());
        }
        Some(elapsed)
    } else {
        None
    };
    let mut owner_changes = Vec::new();
    if owner_trace.is_some() {
        owner_changes.push(serde_json::json!({"tick":0,"owners":initial}));
    }
    let initial_counts = workers(sim)
        .iter()
        .map(|o| o.snapshot.bodies.len())
        .collect();
    let mut pads = groups.iter().filter(|g| g.len() == 25).map(|g| {
        let mut owners: Vec<_> = g.iter().map(|b| initial[b.id as usize - 1]).collect();
        owners.sort_unstable();
        owners.dedup();
        owners
    });
    let initial_pad_owners = [pads.next().unwrap(), pads.next().unwrap()];
    let mut links = Vec::new();
    for e in sim.world().iter_entities() {
        if let (Some(handle), Some(id)) = (e.get::<BodyHandle>(), e.get::<EntityId>())
            && handle.id().raw() < count as u64
        {
            links.push((handle.id().raw(), id.get(), e.id()));
        }
    }
    links.sort_unstable_by_key(|(id, _, _)| *id);
    if links.iter().map(|(id, _, _)| *id).ne(0..count as u64) {
        return Err("wrong mobile link handles".into());
    }
    let mut public_ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|id| id.get()))
        .collect();
    public_ids.sort_unstable();
    let bindings = links
        .iter()
        .map(|(body, public, _)| (body, public))
        .collect::<Vec<_>>();
    let public_ids_sha256 = format!(
        "{:x}",
        Sha256::digest(bincode::serialize(&(public_ids, bindings))?)
    );
    let mut physics = Sha256::new();
    let mut ecs = Sha256::new();
    let mut times = Vec::new();
    let (mut first_step_us, mut first_step_moved_bodies, mut total_moved_bodies) = (0., 0, 0);
    let (mut pair_receipts, mut floor_receipts) = (0, 0);
    let mut fault = None;
    for tick in 1..=fixture::END {
        let began = Instant::now();
        for w in fixture::wrenches(tick, groups) {
            queue_link_world_wrench(sim, links[w.id as usize - 1].1, w.force, w.torque)?;
        }
        let interrupted = if lose_ack_at == Some(tick) {
            let mut handle = sim.world_mut().resource_mut::<PhysicsBackend>();
            let backend = handle
                .downcast_mut::<SceneBackend>()
                .ok_or("missing distributed backend")?;
            let pids = backend.local_worker_pids().ok_or("workers unavailable")?;
            let worker = initial[0] as u8;
            if kill {
                kill_worker(pids[usize::from(worker)])?;
            } else {
                backend.drop_next_acknowledgement(worker)?;
            }
            Some((worker, pids))
        } else {
            None
        };
        let at = Instant::now();
        sim.step_exact(1)?;
        let step_us = at.elapsed().as_secs_f64() * 1e6;
        if let Some((worker, prior)) = interrupted {
            let pids = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<SceneBackend>()
                .and_then(SceneBackend::local_worker_pids)
                .ok_or("workers unavailable after retry")?;
            let replaced_workers = pids
                .iter()
                .zip(prior)
                .filter(|(pid, old)| **pid != *old)
                .count();
            fault = Some(Fault {
                tick,
                worker,
                public_step_us: step_us,
                replaced_workers,
            });
        }
        let current = owners(sim, count);
        let changed = initial.iter().zip(&current).filter(|(a, b)| a != b).count();
        if changed > 0 && owner_trace.is_some() {
            owner_changes.push(serde_json::json!({"tick":tick,"owners":current}));
        }
        total_moved_bodies += changed;
        if tick == 1 {
            first_step_us = step_us;
            first_step_moved_bodies = changed;
        }
        if tick > fixture::SETTLED {
            times.push(step_us);
        }
        initial = current;
        let mut bodies = Vec::new();
        let mut pairs = Vec::new();
        let mut floors = Vec::new();
        for obs in workers(sim) {
            let s = &obs.snapshot;
            if s.tick != tick {
                return Err("incomplete SDF barrier".into());
            }
            bodies.extend(s.bodies.iter().cloned());
            pairs.extend(s.impulses.iter().cloned());
            floors.extend(s.supports.iter().cloned());
        }
        bodies.sort_by_key(|b| b.id);
        pairs.sort_by_key(|p| (p.a, p.b));
        floors.sort_by_key(|p| p.body);
        for (b, (id, _, entity)) in bodies.iter_mut().zip(&links) {
            if b.id != *id {
                return Err("unexpected SDF body identity".into());
            }
            let pose = sim
                .world()
                .get::<WorldPose>(*entity)
                .ok_or("missing public link pose")?;
            if b.position
                .iter()
                .zip(pose.translation)
                .any(|(a, b)| (a - b).abs() > 1e-12)
                || b.orientation
                    .iter()
                    .zip(pose.rotation)
                    .any(|(a, b)| (a - b).abs() > 1e-12)
            {
                return Err(format!("unsynchronized SDF body {} at {tick}", id).into());
            }
            ecs.update(bincode::serialize(&(
                tick,
                id,
                pose.translation,
                pose.rotation,
            ))?);
            // The direct native fixture uses one-based IDs. Normalize SDF handles for its digest.
            b.id += 1;
        }
        for p in &mut pairs {
            p.a += 1;
            p.b += 1;
        }
        for p in &mut floors {
            p.body += 1;
        }
        pair_receipts += pairs.len();
        floor_receipts += floors.len();
        physics.update(bincode::serialize(&(tick, &bodies, &pairs, &floors))?);
        if sim.elapsed() != Duration::from_millis(10 * tick) {
            return Err("public clock differs from committed tick".into());
        }
        if let Some(rest) = pace.checked_sub(began.elapsed()) {
            std::thread::sleep(rest);
        }
    }
    let measured_elapsed_ms = times.iter().sum::<f64>() / 1000.;
    times.sort_by(f64::total_cmp);
    let measured_p95_us = times[(95 * times.len()).div_ceil(100) - 1];
    let timing = Timing {
        placement: if contact { "contact" } else { "spatial" }.into(),
        layout,
        workers: width,
        load_us,
        reset_us,
        first_step_us,
        measured_elapsed_ms,
        measured_p95_us,
        initial_counts,
        initial_pad_owners,
        first_step_moved_bodies,
        total_moved_bodies,
        pair_receipts,
        floor_receipts,
        physics_sha256: format!("{:x}", physics.finalize()),
        public_ids_sha256,
        ecs_poses_sha256: format!("{:x}", ecs.finalize()),
        fault,
    };
    if let Some(path) = owner_trace {
        serde_json::to_writer_pretty(
            File::create_new(path)?,
            &serde_json::json!({
                "layout": layout,
                "placement": timing.placement,
                "first_body_id": 1,
                "last_tick": fixture::END,
                "trajectory_sha256": timing.physics_sha256,
                "owner_changes": owner_changes,
            }),
        )?;
    }
    Ok(timing)
}
#[derive(Serialize, Deserialize)]
struct Trial {
    round: u16,
    spatial_first: bool,
    spatial: Timing,
    contact: Timing,
}

fn median(mut samples: Vec<f64>) -> f64 {
    samples.sort_by(f64::total_cmp);
    let middle = samples.len() / 2;
    if samples.len().is_multiple_of(2) {
        (samples[middle - 1] + samples[middle]) / 2.
    } else {
        samples[middle]
    }
}

fn summary(input: &Path) -> Result<serde_json::Value> {
    let trials: Vec<_> = serde_json::Deserializer::from_reader(BufReader::new(File::open(input)?))
        .into_iter::<Trial>()
        .collect::<std::result::Result<_, _>>()?;
    let first = trials.first().ok_or("empty comparison")?;
    for (i, pair) in trials.iter().enumerate() {
        if usize::from(pair.round) != i + 1 || pair.spatial_first != pair.round.is_multiple_of(2) {
            return Err("out-of-order comparison".into());
        }
        for t in [&pair.spatial, &pair.contact] {
            if t.layout != first.spatial.layout
                || t.workers != first.spatial.workers
                || t.physics_sha256 != first.spatial.physics_sha256
                || t.ecs_poses_sha256 != first.spatial.ecs_poses_sha256
                || t.floor_receipts != first.spatial.floor_receipts
                || t.pair_receipts != first.spatial.pair_receipts
            {
                return Err("mixed trajectories in comparison".into());
            }
        }
    }
    let metric = |get: fn(&Timing) -> f64| {
        let spatial: Vec<_> = trials.iter().map(|t| get(&t.spatial)).collect();
        let contact: Vec<_> = trials.iter().map(|t| get(&t.contact)).collect();
        let delta: Vec<_> = spatial
            .iter()
            .zip(&contact)
            .map(|(s, c)| (c / s - 1.) * 100.)
            .collect();
        serde_json::json!({
            "contact_faster_pairs": delta.iter().filter(|v| **v < 0.).count(),
            "median_paired_change_pct": median(delta),
            "spatial_median": median(spatial),
            "contact_median": median(contact)
        })
    };
    Ok(serde_json::json!({
        "pairs": trials.len(),
        "layout": first.spatial.layout,
        "workers": first.spatial.workers,
        "first_step_us": metric(|t| t.first_step_us),
        "load_us": metric(|t| t.load_us),
        "load_plus_first_step_us": metric(|t| t.load_us + t.first_step_us),
        "measured_elapsed_ms": metric(|t| t.measured_elapsed_ms),
        "measured_p95_us": metric(|t| t.measured_p95_us),
        "physics_sha256": first.spatial.physics_sha256,
        "ecs_poses_sha256": first.spatial.ecs_poses_sha256,
    }))
}

fn start_trials(
    reference: &Path,
    candidate: &Path,
    worker: &Path,
    layout: Layout,
    width: u8,
    rounds: u16,
    out: &Path,
) -> Result<()> {
    std::fs::create_dir(out)?;
    let mut log = BufWriter::new(File::create_new(out.join("pairs.jsonl"))?);
    let mut expected = None;
    let mut samples = [vec![], vec![], vec![], vec![]];
    for index in 0..rounds {
        let order = if index.is_multiple_of(2) {
            [0, 1]
        } else {
            [1, 0]
        };
        let mut pair = [None, None];
        for arm in order {
            let binary = [reference, candidate][arm];
            let output = std::process::Command::new(binary)
                .arg("run")
                .arg("--worker")
                .arg(worker)
                .args([
                    "--workers",
                    &width.to_string(),
                    "--layout",
                    match layout {
                        Layout::Adjacent => "adjacent",
                        Layout::Separated => "separated",
                    },
                    "--contact-placement",
                    "--verify-reset",
                ])
                .output()?;
            if !output.status.success() {
                return Err(format!(
                    "{}: {}",
                    binary.display(),
                    String::from_utf8_lossy(&output.stderr)
                )
                .into());
            }
            let t: Timing = serde_json::from_slice(&output.stdout)?;
            let proof = (
                t.physics_sha256.clone(),
                t.ecs_poses_sha256.clone(),
                t.public_ids_sha256.clone(),
                t.pair_receipts,
                t.floor_receipts,
                t.initial_counts.clone(),
                t.total_moved_bodies,
            );
            if expected.get_or_insert_with(|| proof.clone()) != &proof {
                return Err("SDF trajectory or reset identity changed across trials".into());
            }
            samples[arm].push(t.load_us);
            samples[2 + arm].push(t.reset_us.ok_or("missing timed public reset")?);
            pair[arm] = Some(t);
        }
        writeln!(
            log,
            "{}",
            serde_json::json!({"reference_first":order[0]==0,"samples":pair})
        )?;
        log.flush()?;
    }
    let timing = samples.map(|mut a| {
        a.sort_by(f64::total_cmp);
        let n = a.len();
        serde_json::json!({"median_us":(a[(n-1)/2]+a[n/2])/2.,"p95_us":a[(95*n).div_ceil(100)-1]})
    });
    std::fs::write(
        out.join("summary.json"),
        serde_json::to_vec_pretty(
            &serde_json::json!({"width":width,"layout":layout,"pairs":rounds,
        "load":{"serial":timing[0],"paired":timing[1]},"reset":{"serial":timing[2],"paired":timing[3]},"verified":expected}),
        )?,
    )?;
    Ok(())
}

fn retry_trials(
    reference: &Path,
    candidate: &Path,
    worker: &Path,
    layout: Layout,
    width: u8,
    rounds: u16,
    out: &Path,
) -> Result<()> {
    let run_child = |binary: &Path, tick: Option<u64>| -> Result<Timing> {
        let mut cmd = std::process::Command::new(binary);
        cmd.arg("run").arg("--worker").arg(worker).args([
            "--workers",
            &width.to_string(),
            "--contact-placement",
            "--layout",
            match layout {
                Layout::Adjacent => "adjacent",
                Layout::Separated => "separated",
            },
        ]);
        if let Some(t) = tick {
            cmd.args(["--lose-ack-at", &t.to_string()]);
        }
        let output = cmd.output()?;
        if !output.status.success() {
            return Err(format!(
                "{} failed: {}",
                binary.display(),
                String::from_utf8_lossy(&output.stderr)
            )
            .into());
        }
        Ok(serde_json::from_slice(&output.stdout)?)
    };
    std::fs::create_dir(out)?;
    let control = run_child(reference, None)?;
    std::fs::write(
        out.join("control.json"),
        serde_json::to_vec_pretty(&control)?,
    )?;
    let mut log = BufWriter::new(File::create_new(out.join("pairs.jsonl"))?);
    let mut costs = [Vec::new(), Vec::new()];
    let mut replaced = [0; 2];
    for index in 0..rounds {
        let tick = [80, 140, 160, 220][usize::from(index) % 4];
        // Reverse order on the next pass over the four fault ticks.
        let reference_first = (index + index / 4).is_multiple_of(2);
        let (a, b) = if reference_first {
            (reference, candidate)
        } else {
            (candidate, reference)
        };
        let a = run_child(a, Some(tick))?;
        let b = run_child(b, Some(tick))?;
        let pair = if reference_first { [a, b] } else { [b, a] };
        for (i, t) in pair.iter().enumerate() {
            let fault = t.fault.as_ref().ok_or("missing scene fault")?;
            if t.physics_sha256 != control.physics_sha256
                || t.ecs_poses_sha256 != control.ecs_poses_sha256
                || (t.pair_receipts, t.floor_receipts)
                    != (control.pair_receipts, control.floor_receipts)
                || t.initial_counts != control.initial_counts
                || t.total_moved_bodies != control.total_moved_bodies
                || fault.tick != tick
            {
                return Err(
                    format!("scene or public ECS trajectory differed at trial {index}").into(),
                );
            }
            costs[i].push(fault.public_step_us);
            replaced[i] += fault.replaced_workers;
        }
        writeln!(
            log,
            "{}",
            serde_json::json!({"reference_first":reference_first,"reference":pair[0],"candidate":pair[1]})
        )?;
        log.flush()?;
    }
    let [r, c] = costs;
    let summary = serde_json::json!({"layout":layout,"workers":width,"matched_pairs":rounds,
        "reference_public_step_median_us":median(r),"candidate_public_step_median_us":median(c),"replaced_workers":replaced,
        "physics_sha256":control.physics_sha256,"ecs_poses_sha256":control.ecs_poses_sha256});
    std::fs::write(
        out.join("result.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}

fn endurance(args: EnduranceArgs) -> Result<()> {
    std::fs::create_dir(&args.output)?;
    let case = SceneCase::new(args.layout, args.workers, true);
    let (mut sim, load_us) = case.load(&args.worker)?;
    let control = trajectory(&case, &mut sim, load_us, TrajectoryOptions::default())?;
    let expected = match args.layout {
        Layout::Adjacent => "67d7c57f7ba387bd45a26d0135a69aa071b8d88905fbc6c90a8dc9d88da3e688",
        Layout::Separated => "7688e5d08e422d3c50a2a827687955a3dd36e1e32c6024578c279f512ad9dfc1",
    };
    if control.physics_sha256 != expected {
        return Err("initial public trajectory differs from qualified native oracle".into());
    }
    std::fs::write(
        args.output.join("control.json"),
        serde_json::to_vec_pretty(&control)?,
    )?;
    let mut file = BufWriter::new(File::create_new(args.output.join("epochs.jsonl"))?);
    let begin = Instant::now();
    let mut kills = 0u64;
    for epoch in 1..=u32::MAX {
        reset_loaded_world(&mut sim, |_| ())?;
        if !sim.elapsed().is_zero() {
            return Err("loaded reset failed to reset public time".into());
        }
        let kill = epoch.is_multiple_of(2);
        let case_index = (epoch - 1) % 4;
        let tick = [160, 140, 240, 220][case_index as usize];
        let t = trajectory(
            &case,
            &mut sim,
            0.,
            TrajectoryOptions {
                lose_ack_at: Some(tick),
                kill,
                pace: Duration::from_millis(u64::from(args.pace_ms)),
                ..Default::default()
            },
        )?;
        let f = t.fault.as_ref().ok_or("missing fault report")?;
        if t.public_ids_sha256 != control.public_ids_sha256
            || t.ecs_poses_sha256 != control.ecs_poses_sha256
            || t.physics_sha256 != expected
            || (t.pair_receipts, t.floor_receipts)
                != (control.pair_receipts, control.floor_receipts)
            || t.initial_counts != control.initial_counts
            || t.total_moved_bodies != control.total_moved_bodies
            || f.replaced_workers != usize::from(kill)
        {
            return Err(
                format!("public reset or supervised recovery differed on epoch {epoch}").into(),
            );
        }
        kills += u64::from(kill);
        let row = serde_json::json!({"epoch":epoch,"pid_killed":kill,"elapsed_s":begin.elapsed().as_secs_f64(),"trajectory":t});
        writeln!(file, "{row}")?;
        file.flush()?;
        if args.seconds.map_or(epoch >= args.epochs, |s| {
            begin.elapsed() >= Duration::from_secs(u64::from(s))
        }) {
            std::fs::write(
                args.output.join("result.json"),
                serde_json::to_vec_pretty(
                    &serde_json::json!({"epochs":epoch,"public_resets":epoch,
                "ticks":u64::from(epoch)*fixture::END,"kills":kills,"lost_acks":u64::from(epoch)-kills,"native_sha256":expected,"ecs_sha256":control.ecs_poses_sha256,"public_ids_sha256":control.public_ids_sha256}),
                )?,
            )?;
            break;
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    match Args::parse().command {
        Command::Sdf { layout, output } => {
            std::fs::write(output, xml(&fixture::seeds(layout)))?;
        }
        Command::Run {
            worker,
            layout,
            workers,
            contact_placement,
            verify_reset,
            owner_trace,
            lose_ack_at,
        } => {
            println!(
                "{}",
                serde_json::to_string(&run(
                    &worker,
                    layout,
                    workers,
                    contact_placement,
                    verify_reset,
                    owner_trace.as_deref(),
                    lose_ack_at
                )?)?
            );
        }
        Command::Endurance(args) => endurance(args)?,
        Command::StartTrials {
            reference,
            candidate,
            worker,
            layout,
            workers,
            rounds,
            output,
        } => start_trials(
            &reference, &candidate, &worker, layout, workers, rounds, &output,
        )?,
        Command::RetryTrials {
            reference,
            candidate,
            worker,
            layout,
            workers,
            rounds,
            output,
        } => {
            retry_trials(
                &reference, &candidate, &worker, layout, workers, rounds, &output,
            )?;
        }
        Command::Summary { input } => {
            println!("{}", serde_json::to_string_pretty(&summary(&input)?)?);
        }
        Command::Compare {
            worker,
            layout,
            workers,
            rounds,
            output,
        } => {
            let mut out = BufWriter::new(File::create_new(output)?);
            let mut digests = None;
            for round in 0..=rounds {
                let spatial_first = round % 2 == 0;
                let a = run(&worker, layout, workers, !spatial_first, false, None, None)?;
                let b = run(&worker, layout, workers, spatial_first, false, None, None)?;
                let (spatial, contact) = if spatial_first { (a, b) } else { (b, a) };
                for t in [&spatial, &contact] {
                    let hash = (
                        &t.physics_sha256,
                        &t.ecs_poses_sha256,
                        t.pair_receipts,
                        t.floor_receipts,
                    );
                    let expected = digests
                        .get_or_insert_with(|| (hash.0.clone(), hash.1.clone(), hash.2, hash.3));
                    if (&expected.0, &expected.1, expected.2, expected.3) != hash {
                        return Err(format!("trajectory differs on round {round}").into());
                    }
                }
                if round != 0 {
                    serde_json::to_writer(
                        &mut out,
                        &Trial {
                            round,
                            spatial_first,
                            spatial,
                            contact,
                        },
                    )?;
                    writeln!(out)?;
                    out.flush()?;
                }
            }
        }
    }
    Ok(())
}
