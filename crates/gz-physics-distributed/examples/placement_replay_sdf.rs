// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record and replay one ordinary Bevy/SDF trajectory with a warm process-pool restore.
#[allow(dead_code)]
#[path = "support/changing_oracle.rs"]
mod control;
#[path = "../tests/private/passive_pose_proof.rs"]
mod ecs;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
use clap::Parser;
use control::Oracle;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{SceneBackend, pool::PlacementRecord};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BodyHandle, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    passive_checkpoint::PassivePoolScene,
};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader, BufWriter, Write},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    #[arg(long)]
    sdf: PathBuf,
    out: PathBuf,
}
fn backend(s: &PassivePoolScene) -> &SceneBackend {
    s.simulation()
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .expect("loaded distributed engine")
}
fn links(s: &PassivePoolScene) -> BTreeMap<u64, u64> {
    s.simulation()
        .world()
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<BodyHandle>()?.id().raw() + 1,
                e.get::<EntityId>()?.get(),
            ))
        })
        .collect()
}
fn native(s: &PassivePoolScene, oracle: &Oracle) -> Result<()> {
    let frame = backend(s).pool_frame().unwrap();
    let mut bodies: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| w.snapshot.bodies.iter().cloned())
        .collect();
    let mut pairs: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| w.snapshot.impulses.iter().cloned())
        .collect();
    let mut floors: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| w.snapshot.supports.iter().cloned())
        .collect();
    bodies.sort_by_key(|b| b.id);
    pairs.sort_by_key(|p| (p.a, p.b));
    floors.sort_by_key(|p| p.body);
    for b in &mut bodies {
        b.id += 1;
    }
    for p in &mut pairs {
        p.a += 1;
        p.b += 1;
    }
    for p in &mut floors {
        p.body += 1;
    }
    if bincode::serialize(&(frame.tick, bodies, pairs, floors))?
        != oracle.ticks[frame.tick as usize - 1]
    {
        return Err(format!("SDF native trajectory diverged at {}", frame.tick).into());
    }
    Ok(())
}
fn apply(
    s: &mut PassivePoolScene,
    ids: &BTreeMap<u64, u64>,
    oracle: &Oracle,
    t: u64,
) -> Result<()> {
    for w in fixture::wrenches(t, &oracle.groups) {
        s.queue_wrench(ids[&w.id], w.force, w.torque)?;
    }
    Ok(())
}
fn main() -> Result<()> {
    let a = Args::parse();
    fs::create_dir_all(&a.out)?;
    let xml = fs::read_to_string(&a.sdf)?;
    let oracle = Oracle::new(a.layout)?;
    let options = WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(4),
        step: None,
        distributed_worker: Some(&a.worker),
        noise_seed: [0; 32],
    };
    let recorded_runtime = WorldRuntimeOptions {
        distributed_periodic_placement: Some("8,40,24,0".parse()?),
        distributed_local_helper_limit: Some(0),
        ..Default::default()
    };
    let replay_runtime = WorldRuntimeOptions {
        distributed_packed_handoffs: true,
        distributed_periodic_placement: Some("2,2,64,0".parse()?),
        distributed_local_helper_limit: Some(1),
        ..Default::default()
    };
    let mut source =
        PassivePoolScene::load(&xml, "placement.sdf", "default", options, recorded_runtime)?;
    let recorded_ids = links(&source);
    let mut raw = Vec::new();
    let mut physics = vec![ecs::proof(source.simulation())];
    let path = a.out.join("sdf-tape.jsonl");
    let mut tape = BufWriter::new(File::create_new(&path)?);
    let mut decisions = vec![];
    for t in 1..=fixture::END {
        apply(&mut source, &recorded_ids, &oracle, t)?;
        source.step_exact(1)?;
        let scene = backend(&source);
        let record = scene
            .last_island_placement()
            .ok_or("missing authored receipt")?;
        if record.target_tick != t {
            return Err("incorrect receipt tick".into());
        }
        if record.optional_move.is_some() {
            decisions.push(record.clone());
        }
        serde_json::to_writer(&mut tape, record)?;
        writeln!(tape)?;
        native(&source, &oracle)?;
        raw.push((
            scene.last_substeps(),
            scene.last_island_refined_bodies().map(<[_]>::to_vec),
        ));
        physics.push(ecs::proof(source.simulation()));
    }
    tape.flush()?;
    if decisions.is_empty() {
        return Err("no measured SDF transfer; requires at least two host lanes".into());
    }
    let status = backend(&source).placement_status().unwrap().clone();
    drop(source);
    let mut target =
        PassivePoolScene::load(&xml, "placement.sdf", "default", options, replay_runtime)?;
    if physics[0] != ecs::proof(target.simulation()) || recorded_ids != links(&target) {
        return Err("cold SDF replay mismatch".into());
    }
    let temp = tempfile::tempdir_in(&a.out)?;
    let mut hash = Sha256::new();
    let mut events = vec![];
    for (i, line) in BufReader::new(File::open(&path)?).lines().enumerate() {
        let t = i as u64 + 1;
        let record: PlacementRecord = serde_json::from_str(&line?)?;
        apply(&mut target, &recorded_ids, &oracle, t)?;
        if [1, 80, 160, 280].contains(&t) {
            let bad = PlacementRecord {
                target_tick: t - 1,
                optional_move: None,
            };
            let before = ecs::proof(target.simulation());
            let held = backend(&target).held_scene_wrenches().to_vec();
            if target.step_with_placement(&bad).is_ok()
                || target.simulation().is_faulted()
                || ecs::proof(target.simulation()) != before
                || backend(&target).held_scene_wrenches() != held
            {
                return Err("invalid record consumed public state or pending input".into());
            }
        }
        // Capture a complete public barrier with a nonempty just-admitted force,
        // then resume through the purpose-built passive SDF entry point.
        if [80, 160, 280].contains(&t) {
            let checkpoint = temp.path().join(format!("sdf-{t}.gzscn"));
            let held = backend(&target).held_scene_wrenches().to_vec();
            let previous = ecs::proof(target.simulation());
            let old = backend(&target).local_worker_pids().unwrap();
            if held.len() != 24 {
                return Err("checkpoint lacks loaded launch field".into());
            }
            target.save(&checkpoint)?;
            drop(target);
            target = PassivePoolScene::resume(
                &xml,
                "relocated.sdf",
                "default",
                options,
                replay_runtime,
                &checkpoint,
            )?;
            let pids = backend(&target).local_worker_pids().unwrap();
            if held != backend(&target).held_scene_wrenches()
                || previous != ecs::proof(target.simulation())
                || pids.iter().zip(&old).any(|(a, b)| a == b)
            {
                return Err("SDF fresh restore changed inputs, ECS or workers".into());
            }
            events.push(serde_json::json!({"kind":"fresh_scene_restore","committed_tick":t-1,"next_force_tick":t,"held":held.len(),"before_pids":old,"after_pids":pids}));
        }
        // Simulate real PID death underneath the next recorded transfer.
        #[cfg(target_os = "linux")]
        if t == decisions[0].target_tick {
            use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
            let m = record.optional_move.as_ref().unwrap();
            let old = backend(&target).local_worker_pids().unwrap()[usize::from(m.destination)];
            let fd = pidfd_open(
                Pid::from_raw(i32::try_from(old)?).ok_or("bad pid")?,
                PidfdFlags::empty(),
            )?;
            pidfd_send_signal(&fd, Signal::KILL)?;
            target.step_with_placement(&record)?;
            let new = backend(&target).local_worker_pids().unwrap()[usize::from(m.destination)];
            if new == old {
                return Err("recipient was not replaced".into());
            }
            events.push(serde_json::json!({"kind":"recipient_death","tick":t,"worker":m.destination,"old":old,"new":new}));
        } else {
            target.step_with_placement(&record)?;
        }
        #[cfg(not(target_os = "linux"))]
        target.step_with_placement(&record)?;
        let scene = backend(&target);
        let proof = ecs::proof(target.simulation());
        if scene.last_island_placement() != Some(&record)
            || (
                scene.last_substeps(),
                scene.last_island_refined_bodies().map(<[_]>::to_vec),
            ) != raw[i]
            || physics[i + 1] != proof
            || scene.placement_status().unwrap().windows_started != 0
        {
            return Err(format!("SDF replay differs at {t}").into());
        }
        native(&target, &oracle)?;
        hash.update(proof);
    }
    let summary = serde_json::json!({"layout":a.layout,"ticks":fixture::END,"sdf_sha256":format!("{:x}",Sha256::digest(xml.as_bytes())),
        "native_sha256":oracle.digest,"native_pairs":oracle.pairs,"native_floors":oracle.floors,"ecs_and_native_sha256":format!("{:x}",hash.finalize()),
        "recorded_windows_started":status.windows_started,"recorded_windows_completed":status.windows_completed,"recorded_moves":decisions,
        "replay_windows_started":0,"events":events,"tape_sha256":format!("{:x}",Sha256::digest(fs::read(path)?))});
    serde_json::to_writer_pretty(File::create_new(a.out.join("sdf-summary.json"))?, &summary)?;
    Ok(())
}
