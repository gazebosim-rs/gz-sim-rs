// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure independently durable versus atomic drive and check each tick against ordinary Bevy.
#[path = "support/continuous_hinge_drive.rs"]
mod continuous_hinge_drive;

use clap::{Parser, ValueEnum};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointSample, PhysicsEngine, WorldPose,
    distributed_hinge::{
        ArticulationSpec, DistributedHingeBackend, DistributedMovingHingeBackend, HingeRecipe,
        distributed_hinge_recipe, load_distributed_hinge_world,
    },
    load_world_with_sdf_step, queue_joint_effort,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, ValueEnum)]
enum Mode {
    Split,
    Atomic,
}

#[derive(Parser)]
struct Args {
    sdf: PathBuf,
    /// Stock physics_worker executable.
    #[arg(long)]
    worker: PathBuf,
    /// Filesystem for the temporary durable session.
    #[arg(long)]
    storage: PathBuf,
    #[arg(long, value_enum)]
    mode: Mode,
    #[arg(long, default_value_t = 240, value_parser=clap::value_parser!(u32).range(1..))]
    ticks: u32,
    #[arg(long, allow_hyphen_values = true, default_value_t = 0.01)]
    torque_nm: f64,
    /// Save ordinary poses, joint state and impulses; also copy a loaded checkpoint for gz-web-rs.
    #[arg(long)]
    record: bool,
    /// New result directory.
    output: PathBuf,
}
struct Profile {
    hinge: ArticulationSpec,
    sphere: Option<u64>,
}
impl Profile {
    fn native_body(&self, sim: &Simulation, id: u64) -> Result<(BodyId, Value)> {
        let world = sim.world();
        let entity = entity_by_id(world, id).ok_or("missing body")?;
        let body = world
            .get::<BodyHandle>(entity)
            .ok_or("missing body handle")?
            .id();
        let state = world
            .resource::<PhysicsBackend>()
            .state(body)
            .ok_or("missing native body")?;
        if !sim.elapsed().is_zero() {
            let ecs = world
                .get::<WorldPose>(entity)
                .ok_or("missing published pose")?;
            if (ecs.translation, ecs.rotation) != (state.position, state.orientation) {
                return Err("published and native body pose disagree".into());
            }
        }
        Ok((
            body,
            json!({"id":id,"position":state.position,"velocity":state.velocity,"orientation":state.orientation,"angular_velocity":state.angular_velocity}),
        ))
    }
    fn reference(&self, sim: &Simulation, tick: u32) -> Result<Value> {
        let [(left, l), (right, r)] = [
            self.native_body(sim, self.hinge.links[0].id)?,
            self.native_body(sim, self.hinge.links[1].id)?,
        ];
        let world = sim.world();
        let sample = world
            .get::<JointSample>(entity_by_id(world, self.hinge.joint_id).ok_or("missing joint")?)
            .ok_or("missing ECS joint")?;
        if sample.time != sim.elapsed() || sim.elapsed().as_millis() != u128::from(tick) {
            return Err("ordinary clock mismatch".into());
        }
        let mut hinge = json!({"joint_id":self.hinge.joint_id,"tick":tick,"links":[l,r],"q":sample.state.position,"qdot":sample.state.velocity});
        let native = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("wrong ordinary solver")?;
        if let Some(ground) = self.hinge.ground {
            let floor = self.native_body(sim, ground.link_id)?.0;
            hinge["ground_reactions"] = json!([
                native.ground_reaction(left, floor)?,
                native.ground_reaction(right, floor)?
            ]);
        }
        if let Some(id) = self.sphere {
            let (body, sphere) = self.native_body(sim, id)?;
            return Ok(
                json!({"hinge":hinge,"sphere":sphere,"reactions":[native.moving_reaction(left,body)?,native.moving_reaction(right,body)?]}),
            );
        }
        Ok(hinge)
    }
    fn distributed(&self, sim: &Simulation) -> Result<Value> {
        let backend = sim.world().resource::<PhysicsBackend>();
        if let Some(pool) = backend.downcast_ref::<DistributedMovingHingeBackend>() {
            Ok(serde_json::to_value(pool.pool().observation())?)
        } else {
            let tree = backend
                .downcast_ref::<DistributedHingeBackend>()
                .ok_or("wrong distributed solver")?;
            Ok(serde_json::to_value(tree.pool().observation())?)
        }
    }
    fn load(&self, row: &Value) -> f64 {
        let reactions = row
            .get("reactions")
            .or_else(|| row.get("ground_reactions"))
            .and_then(Value::as_array);
        reactions
            .into_iter()
            .flatten()
            .filter_map(|r| r["normal_impulse"].as_f64())
            .sum()
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    if !args.torque_nm.is_finite() {
        return Err("nonfinite drive".into());
    }
    let xml = fs::read_to_string(&args.sdf)?;
    let name = args.sdf.to_string_lossy();
    let profile = match distributed_hinge_recipe(&xml, &name, "default")? {
        HingeRecipe::Tree(h) => Profile {
            hinge: *h,
            sphere: None,
        },
        HingeRecipe::Moving(m) => Profile {
            hinge: m.hinge,
            sphere: Some(m.sphere.id),
        },
        HingeRecipe::MultiSphere(_) => {
            return Err("use two_sphere_step_cost for the two-companion profile".into());
        }
    };
    let ledger = tempfile::tempdir_in(&args.storage)?;
    let mut ordinary = load_world_with_sdf_step(
        &xml,
        &name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut host =
        load_distributed_hinge_world(&xml, &name, "default", &args.worker, ledger.path(), Some(4))?;
    let atomic = matches!(args.mode, Mode::Atomic);
    if atomic {
        continuous_hinge_drive::install_drive(&mut host, args.torque_nm)?;
    }
    fs::create_dir(&args.output)?;
    let mut frames = args
        .record
        .then(|| File::create(args.output.join("states.jsonl")).map(BufWriter::new))
        .transpose()?;
    let mut samples = Vec::new();
    let mut cut = None;
    let archive_name = if profile.sphere.is_some() {
        "barrier.gzmoving"
    } else {
        "barrier.gzhinge"
    };
    for tick in 0..=args.ticks {
        if tick > 0 {
            queue_joint_effort(&mut ordinary, profile.hinge.joint_id, args.torque_nm)?;
            ordinary.step_exact(1)?;
            let start = Instant::now();
            if !atomic && cut != Some(tick - 1) {
                queue_joint_effort(&mut host, profile.hinge.joint_id, args.torque_nm)?;
            }
            host.step_exact(1)?;
            samples.push(u64::try_from(start.elapsed().as_nanos())?);
        }
        let reference = profile.reference(&ordinary, tick)?;
        let measured = profile.distributed(&host)?;
        if serde_json::to_vec(&reference)? != serde_json::to_vec(&measured)? {
            return Err(format!("different worker physics at tick {tick}").into());
        }
        let joint_entity =
            entity_by_id(host.world(), profile.hinge.joint_id).ok_or("missing worker joint")?;
        let sample = host
            .world()
            .get::<JointSample>(joint_entity)
            .ok_or("missing worker sample")?;
        let joint = reference.get("hinge").unwrap_or(&reference);
        if joint["q"].as_f64() != Some(sample.state.position)
            || joint["qdot"].as_f64() != Some(sample.state.velocity)
            || sample.time != ordinary.elapsed()
        {
            return Err("worker ECS sample differs".into());
        }
        if let Some(frames) = frames.as_mut() {
            let impulse = profile.load(&reference);
            let mut row = json!({"tick":tick});
            if profile.hinge.ground.is_none() && profile.sphere.is_none() {
                row["direct_sdf"] = reference;
            } else {
                row["direct"] = reference;
            }
            if profile.hinge.ground.is_some() {
                row["total_normal_force_n"] = json!(impulse * 1000.);
            }
            serde_json::to_writer(&mut *frames, &row)?;
            writeln!(frames)?;
            if cut.is_none()
                && ((profile.hinge.ground.is_none() && profile.sphere.is_none() && tick == 192)
                    || impulse > 0.)
            {
                queue_joint_effort(&mut host, profile.hinge.joint_id, args.torque_nm)?;
                let dir = args.output.join("cut");
                fs::create_dir(&dir)?;
                fs::copy(ledger.path().join(archive_name), dir.join(archive_name))?;
                cut = Some(tick);
            }
        }
    }
    if let Some(mut frames) = frames {
        frames.flush()?;
        if cut.is_none() {
            return Err("missing display cut".into());
        }
    }
    let path = Path::new(&args.output).join("samples.json");
    serde_json::to_writer(File::create(path)?, &samples)?;
    samples.sort_unstable();
    let summary = json!({"mode":if atomic{"atomic"}else{"split"},"sdf":name,"torque_nm":args.torque_nm,
        "ticks":args.ticks,"states_compared":args.ticks+1,"display_tick":cut,"p50_ns":samples[(samples.len()-1)/2],
        "p95_ns":samples[(samples.len()*95).div_ceil(100)-1],"total_ns":samples.iter().sum::<u64>()});
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
