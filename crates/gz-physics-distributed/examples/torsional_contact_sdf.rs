// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Inspect an authored SDF's four-point twist patch before and after active ownership changes.
use clap::Parser;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_distributed::{
    SceneBackend,
    physics::{NativeFrictionModel, NativeSolverProfile},
    pool::{PlacementMember, PlacementMove, PlacementRecord, PoolNativeRecovery},
};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime,
    passive_checkpoint::{PassivePoolScene, save_paused_runtime},
    step_with_placement,
};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_torsional_contact.sdf");
#[derive(Parser)]
pub(crate) struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    workers: u8,
    #[arg(long)]
    reverse: bool,
    /// Add a 4.8 N s linear impulse at the same instant as the yaw impulse.
    #[arg(long)]
    slide: bool,
    /// Negotiate one Coulomb constraint per contact point on every worker.
    #[arg(long)]
    per_point: bool,
    output: PathBuf,
}
fn options(worker: &Path, width: u8) -> WorldLoadOptions<'_> {
    WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(width),
        distributed_worker: Some(worker),
        step: None,
        noise_seed: [0; 32],
    }
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn link(sim: &Simulation, name: &str) -> Result<(u64, BodyId)> {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .ok_or("authored link missing")?;
    Ok((
        e.get::<EntityId>().ok_or("missing public ID")?.get(),
        e.get::<BodyHandle>().ok_or("missing native body")?.id(),
    ))
}
fn physical(sim: &Simulation, links: &[(u64, BodyId); 2]) -> Result<Vec<u8>> {
    let f = backend(sim).pool_frame().ok_or("missing native pool")?;
    let mut state = vec![];
    let mut pairs = vec![];
    for (pubid, native) in links {
        let (_, b) = f.body(native.raw()).ok_or("lost body")?;
        let e = entity_by_id(sim.world(), *pubid).ok_or("lost ECS link")?;
        let pose = sim.world().get::<WorldPose>(e).ok_or("lost ECS pose")?;
        let twist = sim.world().get::<Velocity>(e).ok_or("lost ECS velocity")?;
        if pose.translation != b.position
            || pose.rotation != b.orientation
            || twist.linear != b.linear
            || twist.angular != b.angular
        {
            return Err("ECS and worker differ".into());
        }
        state.push(b);
    }
    for w in &f.workers {
        pairs.extend(w.snapshot.impulses.iter());
        if !w.snapshot.supports.is_empty() {
            return Err("unexpected infinite plane".into());
        }
    }
    pairs.sort_by_key(|p| (p.a, p.b));
    Ok(bincode::serialize(&(
        f.tick,
        sim.elapsed().as_nanos(),
        state,
        pairs,
    ))?)
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    run(Args::parse())
}

pub(crate) fn run(args: Args) -> Result<()> {
    fs::create_dir(&args.output)?;
    let opts = options(&args.worker, args.workers);
    let runtime = WorldRuntimeOptions {
        rapier_per_point_friction: args.per_point,
        distributed_contact_placement: true,
        distributed_packed_handoffs: true,
        ..Default::default()
    };
    let mut candidate = PassivePoolScene::load(SDF, "yaw.sdf", "default", opts, runtime)?
        .into_checkpointable_runtime();
    let mut reference = load_world_with_runtime(
        SDF,
        "yaw.sdf",
        "default",
        &sdformat::NoResources,
        opts,
        runtime,
    )?;
    for sim in [&mut candidate, &mut reference] {
        sim.set_paused(true);
        sim.tick()?;
    }
    let links = [
        link(&candidate, "default::table::surface")?,
        link(&candidate, "default::yaw_block::body")?,
    ];
    let mut hash = Sha256::new();
    let mut events = vec![];
    let mut points = vec![];
    let ticks = if args.slide { 180 } else { 120 };
    for tick in 1..=ticks {
        if tick == 31 {
            let kick = if args.reverse { -0.96 } else { 0.96 };
            let slide = if args.slide { 4.8 } else { 0. };
            for sim in [&mut candidate, &mut reference] {
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .apply_impulse(links[1].1, [slide, 0., 0.], [0., 0., kick])?;
            }
        }
        let f = backend(&candidate).pool_frame().unwrap();
        let owner = f.body(links[1].1.raw()).unwrap().0;
        if [36, 95].contains(&tick) {
            let w = &f.workers[usize::from(owner)];
            let members = w
                .components
                .iter()
                .find(|c| c.ids.contains(&links[1].1.raw()))
                .ok_or("missing resident island")?
                .ids
                .iter()
                .map(|id| PlacementMember {
                    id: *id,
                    generation: w.generations[id],
                })
                .collect::<Vec<_>>();
            if members.len() != 2 || !w.snapshot.impulses.iter().any(|p| p.normal_impulse > 0.) {
                return Err("finite support expected before transfer".into());
            }
            let destination = (owner + 1) % args.workers;
            step_with_placement(
                &mut candidate,
                &PlacementRecord {
                    target_tick: tick,
                    optional_move: Some(PlacementMove {
                        source: owner,
                        destination,
                        members: members.clone(),
                    }),
                },
            )?;
            let after = backend(&candidate).pool_frame().unwrap();
            if members.iter().any(|m| {
                after.body(m.id).unwrap().0 != destination
                    || after.workers[usize::from(destination)].generations[&m.id]
                        != m.generation + 1
            }) {
                return Err("public migration did not commit whole warm group".into());
            }
            events.push(json!({"move":tick,"owners":[owner,destination],"members":members}));
        } else {
            let mut pid = None;
            #[cfg(target_os = "linux")]
            if [37, 96].contains(&tick) {
                use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
                let old = backend(&candidate).local_worker_pids().unwrap();
                let fd = pidfd_open(
                    Pid::from_raw(old[usize::from(owner)] as i32).ok_or("invalid PID")?,
                    PidfdFlags::empty(),
                )?;
                pidfd_send_signal(&fd, Signal::KILL)?;
                pid = Some(old);
            }
            if [35, 94].contains(&tick) {
                candidate
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .downcast_mut::<SceneBackend>()
                    .unwrap()
                    .drop_next_acknowledgement(owner)?;
            }
            candidate.step_exact(1)?;
            if let Some(old) = pid {
                let new = backend(&candidate).local_worker_pids().unwrap();
                if old
                    .iter()
                    .zip(&new)
                    .enumerate()
                    .any(|(i, (a, b))| (a != b) != (i == usize::from(owner)))
                {
                    return Err("unexpected public PID change".into());
                }
                events.push(json!({"replacement":tick,"old":old,"new":new}));
            }
        }
        reference.step_exact(1)?;
        let p = physical(&candidate, &links)?;
        if p != physical(&reference, &links)? {
            return Err(format!("ordinary SDF control differs at {tick}").into());
        }
        hash.update(&p);
        if [38, 97].contains(&tick) {
            let file = args.output.join(format!("public-{tick}.gzscene"));
            save_paused_runtime(&mut candidate, &file)?;
            let (_, _, image) = PoolNativeRecovery::open_scene_manifest(&file)?;
            let model = if args.per_point {
                NativeFrictionModel::PerPoint
            } else {
                NativeFrictionModel::Grouped
            };
            if image.native_solver_profile() != NativeSolverProfile::Standard32.with_friction(model)
            {
                return Err("saved public scene has the wrong native friction model".into());
            }
            let rows = image.finite_contact_patches()?;
            let flat = rows.iter().flatten().collect::<Vec<_>>();
            if flat.len() != 1
                || flat[0].points.len() != 4
                || flat[0].a != links[0].1.raw()
                || flat[0].b != links[1].1.raw()
            {
                return Err("saved public contact patch differs from authored finite pair".into());
            }
            points.push(json!({"cut":tick,"owners":rows}));
            drop(candidate);
            candidate = PassivePoolScene::resume(SDF, "yaw.sdf", "default", opts, runtime, &file)?
                .into_checkpointable_runtime();
            if physical(&candidate, &links)? != p {
                return Err("public scene resume changed state".into());
            }
        }
    }
    let final_body = backend(&candidate)
        .pool_frame()
        .unwrap()
        .body(links[1].1.raw())
        .unwrap()
        .1;
    let final_speed = final_body.angular[2];
    if final_speed.abs() > 0.01 {
        return Err(format!(
            "resumed SDF block still spinning at tick {ticks}: {final_speed} rad/s"
        )
        .into());
    }
    let linear_speed = final_body.linear.iter().map(|v| v * v).sum::<f64>().sqrt();
    if args.slide && linear_speed > 0.001 {
        return Err(
            format!("resumed SDF block still moving at tick {ticks}: {linear_speed} m/s").into(),
        );
    }
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(
            &json!({"width":args.workers,"ticks":ticks,"per_point":args.per_point,"coupled":args.slide,"final_position_m":final_body.position,"terminal_speed_m_s":linear_speed,"terminal_spin_rad_s":final_speed,"trajectory_sha256":format!("{:x}",hash.finalize()),"events":events,"native_patches":points}),
        )?,
    )?;
    Ok(())
}
