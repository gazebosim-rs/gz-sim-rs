// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify shared-plane contact geometry from an authored scene against one independent native world.
#[allow(dead_code)]
#[path = "support/native_contact_control.rs"]
mod native;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, BodyWrench, GroundContactPatch, GroundSupport},
    pool::{PlacementMember, PlacementMove, PlacementRecord, PoolNativeRecovery},
};
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    passive_checkpoint::PassivePoolScene,
};
use rapier3d_f64::prelude::Vector;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_ground_torsion.sdf");
const NORMAL: [f64; 3] = [0., -0.6, 0.8];
#[derive(Clone, Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    workers: u8,
    #[arg(long)]
    packed: bool,
    /// Run widths three through eight with both handoff encodings and require the same physical trace.
    #[arg(long, conflicts_with_all = ["packed", "workers"])]
    matrix: bool,
    output: PathBuf,
}
fn backend(scene: &PassivePoolScene) -> &SceneBackend {
    scene
        .simulation()
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn read_image(file: &Path) -> Result<PoolNativeRecovery> {
    Ok(PoolNativeRecovery::open_scene_manifest(file)?.2)
}
fn state(scene: &PassivePoolScene, public: u64, id: u64) -> Result<(Body, Vec<GroundSupport>)> {
    let frame = backend(scene)
        .pool_frame()
        .ok_or("missing public barrier")?;
    let (owner, body) = frame.body(id).ok_or("missing native body")?;
    let world = scene.simulation().world();
    let e = entity_by_id(world, public).ok_or("lost public body")?;
    let pose = world.get::<WorldPose>(e).ok_or("missing ECS pose")?;
    let v = world.get::<Velocity>(e).ok_or("missing ECS twist")?;
    if body.position != pose.translation
        || body.orientation != pose.rotation
        || body.linear != v.linear
        || body.angular != v.angular
    {
        return Err("native and ECS pose/twist differ".into());
    }
    Ok((
        body.clone(),
        frame.workers[usize::from(owner)].snapshot.supports.clone(),
    ))
}
#[cfg(target_os = "linux")]
fn kill_owner(scene: &PassivePoolScene, owner: u8) -> Result<Vec<u32>> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let old = backend(scene)
        .local_worker_pids()
        .ok_or("missing child workers")?;
    let pid = Pid::from_raw(old[usize::from(owner)] as i32).ok_or("invalid worker PID")?;
    pidfd_send_signal(&pidfd_open(pid, PidfdFlags::empty())?, Signal::KILL)?;
    Ok(old)
}
#[cfg(not(target_os = "linux"))]
fn kill_owner(_: &PassivePoolScene, _: u8) -> Result<Vec<u32>> {
    Err("the PID death check in this example requires Linux pidfds".into())
}
fn run(args: &Args) -> Result<()> {
    fs::create_dir(&args.output)?;
    let opt = WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(args.workers),
        distributed_worker: Some(&args.worker),
        step: None,
        noise_seed: [0; 32],
    };
    let runtime = WorldRuntimeOptions {
        distributed_contact_placement: true,
        distributed_packed_handoffs: args.packed,
        ..Default::default()
    };
    let source = "distributed_ground_torsion.sdf";
    let mut scene = PassivePoolScene::load(SDF, source, "default", opt, runtime)?;
    scene.set_paused(true);
    scene.tick()?;
    let link = scene
        .simulation()
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::yaw_block::body")
        })
        .ok_or("missing SDF block")?;
    let public = link.get::<EntityId>().ok_or("missing public ID")?.get();
    let id = link
        .get::<BodyHandle>()
        .ok_or("missing native block")?
        .id()
        .raw();
    let cold = args.output.join("public-0.gzscene");
    scene.save(&cold)?;
    let seed = read_image(&cold)?;
    let (seed_owner, body) = seed.frame().body(id).ok_or("seed body missing")?;
    let ground = seed.frame().workers[usize::from(seed_owner)]
        .snapshot
        .ground
        .ok_or("missing authored plane")?;
    let normal = Vector::from(NORMAL);
    if (Vector::from(ground.normal()) - normal).length() > 1e-12
        || ground.point() != [2.25, -1.75, 0.85]
        || ground.friction() != 0.65
        || body.size != [0.8, 0.4, 0.2]
        || body.mass != 2.
        || body.friction != 0.35
        || !seed.ground_contact_patches()?.iter().all(Vec::is_empty)
    {
        return Err(format!("unexpected cold SDF geometry: {ground:?} {body:?}").into());
    }
    let mut direct = native::Control::new(
        std::slice::from_ref(body),
        Some(ground),
        [0., 5.886, -7.848],
    )?;
    let drive = BodyWrench {
        id,
        force: [0.; 3],
        torque: (-96. * normal).to_array(),
    };
    let mut hash = Sha256::new();
    let mut events = Vec::new();
    let mut spin_at_drive = 0.;
    let mut stop = None;
    for tick in 1..=120 {
        let mut event = None;
        if tick == 31 {
            scene.queue_wrench(public, drive.force, drive.torque)?;
            let file = args.output.join("held-torque-30.gzscene");
            scene.save(&file)?;
            drop(scene);
            scene = PassivePoolScene::resume(SDF, source, "default", opt, runtime, &file)?;
            events
                .push(json!({"input_saved_at":30,"consumed_at":31,"world_torque_nm":drive.torque}));
        }
        let f = backend(&scene).pool_frame().unwrap();
        let owner = f.body(id).unwrap().0;
        if [36, 95].contains(&tick) {
            let w = &f.workers[usize::from(owner)];
            if !w
                .snapshot
                .supports
                .iter()
                .any(|x| x.body == id && x.normal_impulse > 0.)
            {
                return Err("loaded public support required before move".into());
            }
            let generation = w.generations[&id];
            let destination = (owner + 1) % args.workers;
            scene.step_with_placement(&PlacementRecord {
                target_tick: tick,
                optional_move: Some(PlacementMove {
                    source: owner,
                    destination,
                    members: vec![PlacementMember { id, generation }],
                }),
            })?;
            let corrected = backend(&scene).pool_frame().unwrap();
            if corrected.body(id).unwrap().0 != destination
                || corrected.workers[usize::from(destination)].generations[&id] != generation + 1
            {
                return Err("public migration failed to advance generation".into());
            }
            event = Some(
                json!({"kind":"loaded_move","owners":[owner,destination],"generation":generation+1}),
            );
        } else {
            let killed = [37, 96]
                .contains(&tick)
                .then(|| kill_owner(&scene, owner))
                .transpose()?;
            scene.step_exact(1)?;
            if let Some(old) = killed {
                let new = backend(&scene).local_worker_pids().unwrap();
                if old
                    .iter()
                    .zip(&new)
                    .enumerate()
                    .any(|(i, (a, b))| (a != b) != (i == usize::from(owner)))
                {
                    return Err("unexpected PID change".into());
                }
                event = Some(json!({"kind":"owner_death","old":old,"new":new}));
            }
        }
        let actual = state(&scene, public, id)?;
        let rate = backend(&scene).last_substeps().ok_or("no resolved rate")?;
        let mut patches = vec![];
        let loads = if tick == 31 {
            std::slice::from_ref(&drive)
        } else {
            &[]
        };
        let (pairs, supports) = direct.advance_observing(rate, loads, &[], |engine| {
            patches = engine
                .partition_ground_patches()
                .into_iter()
                .map(|p| GroundContactPatch {
                    body: id,
                    normal_on_body: p.normal_on_body,
                    points: p.points,
                })
                .collect();
        })?;
        if !pairs.is_empty()
            || bincode::serialize(&actual)? != bincode::serialize(&(&direct.bodies[0], &supports))?
        {
            return Err(format!("public body or normal receipts differ from direct native at tick {tick} (rate {rate}): {:?} / {:?}, {:?} / {:?}",actual.0,direct.bodies[0],actual.1,supports).into());
        }
        hash.update(bincode::serialize(&(tick, &actual, &patches))?);
        let omega = Vector::from(actual.0.angular).dot(normal);
        if tick == 31 {
            spin_at_drive = omega;
        }
        if tick > 31 && omega.abs() < 0.01 && stop.is_none() {
            stop = Some(tick);
        }
        if [38, 97].contains(&tick) {
            event = Some(json!({"kind":"disk_resume"}));
        }
        if let Some(event) = event {
            let file = args.output.join(format!("public-{tick}.gzscene"));
            scene.save(&file)?;
            let image = read_image(&file)?;
            let rows = image.ground_contact_patches()?;
            let owner = image.frame().body(id).unwrap().0;
            if rows.len() != usize::from(args.workers)
                || rows.iter().flatten().count() != 1
                || bincode::serialize(&rows[usize::from(owner)])? != bincode::serialize(&patches)?
                || patches.len() != 1
                || (Vector::from(patches[0].normal_on_body) - normal).length() > 1e-12
                || patches[0].points.len() != 4
                || patches[0].points.iter().any(|p| p.normal_impulse <= 0.)
                || (supports[0].normal_impulse - 0.1962).abs() > 0.002
            {
                return Err(format!(
                    "event native ground patch or public support mismatch at {tick}"
                )
                .into());
            }
            if [38, 97].contains(&tick) {
                let old = backend(&scene).local_worker_pids().unwrap();
                drop(scene);
                scene = PassivePoolScene::resume(SDF, source, "default", opt, runtime, &file)?;
                if backend(&scene)
                    .local_worker_pids()
                    .unwrap()
                    .iter()
                    .zip(&old)
                    .any(|(a, b)| a == b)
                    || state(&scene, public, id)? != actual
                {
                    return Err("public disk resume changed body or reused worker PID".into());
                }
            }
            events.push(json!({"tick":tick,"event":event,"owner":owner,"final_native_points":patches[0],"substeps":rate,"public_normal_ns":supports[0].normal_impulse}));
        }
    }
    let predicted_s = 0.96 / (0.35 * 2. * 9.81 * (0.4_f64.hypot(0.2)));
    let elapsed_s = (stop.ok_or("public block did not stop")? - 30) as f64 * 0.01;
    if spin_at_drive >= -6. || (elapsed_s - predicted_s).abs() > 0.02 {
        return Err(format!(
            "public braking gate: spin {spin_at_drive}, stop {elapsed_s}, expected {predicted_s}"
        )
        .into());
    }
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&json!({
            "width":args.workers,"packed_handoffs":args.packed,"ticks":120,"public_link":public,"native_body":id,"ground":ground,
            "spin_after_held_torque_rad_s":spin_at_drive,"stop_s_from_drive_start":elapsed_s,"ideal_stop_s":predicted_s,
            "trajectory_and_patch_sha256":format!("{:x}",hash.finalize()),"events":events
        }))?,
    )?;
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    if !args.matrix {
        return run(&args);
    }
    fs::create_dir(&args.output)?;
    let mut rows = Vec::new();
    let mut first = None;
    for workers in 3..=8 {
        for packed in [false, true] {
            let arm = Args {
                workers,
                packed,
                matrix: false,
                output: args.output.join(format!(
                    "sdf-{workers}-{}",
                    if packed { "packed" } else { "json" }
                )),
                ..args.clone()
            };
            run(&arm)?;
            let result: serde_json::Value =
                serde_json::from_slice(&fs::read(arm.output.join("summary.json"))?)?;
            let digest = result["trajectory_and_patch_sha256"]
                .as_str()
                .ok_or("missing SDF trajectory digest")?;
            let expected = first.get_or_insert_with(|| digest.to_owned());
            if digest != expected {
                return Err(format!(
                    "body, support or native patch changed at width {workers}, packed {packed}"
                )
                .into());
            }
            rows.push(
                json!({"workers":workers,"packed_handoffs":packed,"ticks":result["ticks"],
                "stop_s":result["stop_s_from_drive_start"],"trajectory_and_patch_sha256":digest}),
            );
        }
    }
    fs::write(
        args.output.join("matrix.json"),
        serde_json::to_vec_pretty(&rows)?,
    )?;
    Ok(())
}
