// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a distributed multi-revolute Bevy scene to an uninterrupted local reference every tick.
#[allow(dead_code)]
#[path = "support/floating_tree_drive.rs"]
mod floating_tree_drive;
#[path = "support/grasp_drive.rs"]
mod grasp_drive;
#[allow(dead_code)]
#[path = "support/tree_motor_drive.rs"]
mod tree_motor_drive;
#[path = "support/tree_visuals.rs"]
mod tree_visuals;
use clap::{Parser, ValueEnum};
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::distributed_hinge::{DistributedGroundedTreeBackend, GroundedTreeSpec};
use gz_sim_scene::distributed_hinge::{
    DistributedSplitGraspBackend, GraspPlacement, GraspTreeSpec, PayloadTreeSpec,
    load_distributed_grasp_tree_world, load_distributed_split_grasp_world,
};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, PhysicsEngine, Velocity, WorldPose,
    distributed_hinge::{
        DistributedTreeBackend, FloatingTreeRecipe, FloatingTreeSpec, MotorTreeSpec,
        StoppedTreeSpec, TreeConstruction, distributed_tree_recipe,
        load_distributed_motor_tree_world, load_distributed_tree_world,
    },
    load_world_with_sdf_step, queue_joint_effort,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::Instant,
};
use tree_visuals::poses;

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, ValueEnum, PartialEq)]
enum Mode {
    Split,
    Vector,
}
#[derive(Parser)]
struct Args {
    sdf: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    storage: PathBuf,
    #[arg(long, value_enum)]
    mode: Mode,
    #[arg(long,default_value_t=240,value_parser=clap::value_parser!(u32).range(1..))]
    ticks: u32,
    /// Apply persistent native velocity motors, including a one-shot torque at a recorded cut.
    #[arg(long)]
    velocity_motors: bool,
    /// Compare the independently moving payload under the calibrated motor/load/release exercise.
    #[arg(long,value_enum,conflicts_with_all=["velocity_motors","reverse_every"])]
    grasp: Option<grasp_drive::Grip>,
    /// Keep the gripper coupled, or separate its free payload after N clear intervals.
    #[arg(long,requires="grasp",value_parser=gz_sim_cli::parse_grasp_placement)]
    placement: Option<GraspPlacement>,
    /// Reverse all inputs every N advancing intervals.
    #[arg(long)]
    reverse_every: Option<std::num::NonZeroU64>,
    /// Record ordinary poses for gz-web-rs and retain a migrating cut at this tick.
    #[arg(long,value_parser=clap::value_parser!(u32).range(2..))]
    record_at: Option<u32>,
    output: PathBuf,
}

fn backend<S: TreeConstruction + Send + Sync + 'static>(
    sim: &Simulation,
) -> &DistributedTreeBackend<S> {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .expect("owned tree session")
}

fn native(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .expect("ordinary native world")
}

fn verify<S: TreeConstruction + Send + Sync + 'static>(
    host: &Simulation,
    regular: &Simulation,
    tick: u32,
    include_visuals: bool,
) -> Result<Option<Value>> {
    let be = host.world().resource::<PhysicsBackend>();
    let split = be
        .downcast_ref::<DistributedSplitGraspBackend>()
        .map(|b| b.pool());
    let (observation, payload, settings, image, owners, generation) = if let Some(p) = split {
        (
            p.observation(),
            Some((p.payload_state(), p.payload_reactions())),
            Some(
                p.observation()
                    .joints
                    .iter()
                    .map(|j| p.velocity_motor(j.joint_id))
                    .collect::<Vec<_>>(),
            ),
            p.image(),
            p.owners(),
            p.generation(),
        )
    } else {
        let p = backend::<S>(host).pool();
        (
            p.observation(),
            p.observed_payload(),
            p.observed_velocity_motors().map(|m| m.collect::<Vec<_>>()),
            p.image(),
            [p.owner(); 2],
            p.generation(),
        )
    };
    if observation.tick != u64::from(tick)
        || regular.elapsed() != host.elapsed()
        || regular.elapsed().as_millis() != u128::from(tick)
    {
        return Err("host or worker clock differs from ordinary Bevy".into());
    }
    let normal = regular.world().resource::<PhysicsBackend>();
    let external = payload.map(|(p, _)| p);
    for row in observation.links.iter().chain(external) {
        let e = entity_by_id(regular.world(), row.id).ok_or("missing ordinary link")?;
        let h = entity_by_id(host.world(), row.id).ok_or("missing worker link")?;
        let state = normal
            .state(
                regular
                    .world()
                    .get::<BodyHandle>(e)
                    .ok_or("missing ordinary body handle")?
                    .id(),
            )
            .ok_or("missing native body")?;
        let ordinary = regular
            .world()
            .get::<WorldPose>(e)
            .ok_or("missing ordinary ECS pose")?;
        let published = host
            .world()
            .get::<WorldPose>(h)
            .ok_or("missing worker ECS pose")?;
        let v = regular
            .world()
            .get::<Velocity>(e)
            .ok_or("missing ordinary velocity")?;
        let w = host
            .world()
            .get::<Velocity>(h)
            .ok_or("missing published velocity")?;
        if (ordinary.translation, ordinary.rotation, v.linear, v.angular)
            != (
                published.translation,
                published.rotation,
                w.linear,
                w.angular,
            )
            || (
                state.position,
                state.orientation,
                state.velocity,
                state.angular_velocity,
            ) != (
                row.position,
                row.orientation,
                row.velocity,
                row.angular_velocity,
            )
        {
            return Err(format!("worker body {} disagrees at tick {tick}", row.id).into());
        }
    }
    let mut motor_settings = settings.map(Vec::into_iter);
    let ground_host = host
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedGroundedTreeBackend>();
    let mut reactions = Vec::new();
    if let Some(ground) = ground_host {
        let p = ground.pool();
        let body = |id| {
            regular
                .world()
                .get::<BodyHandle>(
                    entity_by_id(regular.world(), id).ok_or("missing reaction link")?,
                )
                .map(BodyHandle::id)
                .ok_or("missing ground body")
        };
        let floor = body(p.spec().ground.link_id)?;
        for (link, r) in p.observation().links.iter().zip(p.ground_reactions()) {
            if native(regular).ground_reaction(body(link.id)?, floor)? != *r {
                return Err(
                    format!("ground reaction on link {} differs at {tick}", link.id).into(),
                );
            }
            if include_visuals {
                reactions.push(json!({"link_id":link.id,"reaction":r}));
            }
        }
    }
    let mut payload_reactions = Vec::new();
    if let Some((state, contacts)) = payload {
        let body = |id| {
            regular
                .world()
                .get::<BodyHandle>(
                    entity_by_id(regular.world(), id).ok_or("missing ordinary payload identity")?,
                )
                .map(BodyHandle::id)
                .ok_or("missing ordinary payload body")
        };
        let payload = body(state.id)?;
        for (link, r) in observation.links.iter().zip(contacts) {
            if native(regular).payload_tree_reaction(body(link.id)?, payload)? != *r {
                return Err(
                    format!("payload reaction on link {} differs at {tick}", link.id).into(),
                );
            }
            if include_visuals {
                payload_reactions.push(json!({"link_id":link.id,"reaction":r}));
            }
        }
    }
    let mut motors = Vec::new();
    for joint in &observation.joints {
        let ordinary =
            entity_by_id(regular.world(), joint.joint_id).ok_or("missing ordinary joint")?;
        let published = entity_by_id(host.world(), joint.joint_id).ok_or("missing worker joint")?;
        let id = regular
            .world()
            .get::<JointHandle>(ordinary)
            .ok_or("missing ordinary joint handle")?
            .id();
        let a = regular
            .world()
            .get::<JointSample>(ordinary)
            .ok_or("missing ordinary joint sample")?;
        let b = host
            .world()
            .get::<JointSample>(published)
            .ok_or("missing published joint sample")?;
        if let Some(settings) = &mut motor_settings {
            let m = settings.next().ok_or("missing solver motor observation")?;
            if m != native(regular).motor_revolute_velocity(id)? {
                return Err(format!("worker motor {} differs at {tick}", joint.joint_id).into());
            }
            if include_visuals {
                motors.push(json!({"joint_id":joint.joint_id,"target_velocity":m.map(|m|m.target_velocity),"max_force":m.map(|m|m.max_force)}));
            }
        }
        if a.state != b.state
            || a.time != b.time
            || a.epoch != b.epoch
            || (a.state.position, a.state.velocity) != (joint.q, joint.qdot)
            || native(regular).pending_joint_effort(id).map(f64::to_bits)
                != joint.pending_effort.map(f64::to_bits)
        {
            return Err(format!("worker joint {} disagrees at tick {tick}", joint.joint_id).into());
        }
    }
    if tick > 0 {
        let expected = if owners[0] != owners[1] {
            let id = external
                .ok_or("split layout missing its independent payload")?
                .id;
            let entity = entity_by_id(regular.world(), id).ok_or("missing payload entity")?;
            let body = regular
                .world()
                .get::<BodyHandle>(entity)
                .ok_or("missing independent native body")?
                .id();
            native(regular).detached_grasp_tree_image(body)?
        } else if payload.is_some() && motor_settings.is_some() {
            native(regular).checkpoint_grasp_revolute_tree()?
        } else if payload.is_some() {
            native(regular).checkpoint_payload_revolute_tree()?
        } else if ground_host.is_some() {
            native(regular).checkpoint_grounded_revolute_tree()?
        } else if motor_settings.is_some() {
            native(regular).checkpoint_motor_revolute_tree()?
        } else {
            native(regular).checkpoint_floating_revolute_tree()?
        };
        // A joined graph rebuilt from a compact image has a different dormant BVH history.
        // Its exposed body/joint/solver receipts were compared with the uninterrupted graph above.
        if owners[0] != owners[1] || split.is_none_or(|p| p.stats().joins == 0) {
            if image != Some(expected.as_slice()) {
                return Err(format!("native bytes differ at tick {tick}").into());
            }
        }
    }
    if include_visuals {
        let a = poses(regular)?;
        if a != poses(host)? {
            return Err(format!("published visual poses differ at tick {tick}").into());
        }
        Ok(Some(
            json!({"tick":tick,"owner":owners[0],"owners":owners,"placement":split.map(|p|p.placement()),"placement_stats":split.map(|p|p.stats()),"generation":generation,"state":observation,"velocity_motors":motors,"ground_reactions":reactions,"payload":external,"payload_reactions":payload_reactions,"visuals":a.into_values().collect::<Vec<_>>() }),
        ))
    } else {
        Ok(None)
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    if args.record_at.is_some_and(|t| t >= args.ticks)
        || args.record_at.is_some() && args.mode != Mode::Vector
    {
        return Err("record a vector run with at least one tick after the display cut".into());
    }
    let xml = fs::read_to_string(&args.sdf)?;
    let source = args.sdf.to_string_lossy();
    if args.grasp.is_some() {
        if args.mode != Mode::Vector {
            return Err("loaded grasp uses the atomic vector operator".into());
        }
        return run::<GraspTreeSpec>(
            &xml,
            &args,
            if args.placement.is_some() {
                "barrier.gzsplitgrasp"
            } else {
                "barrier.gzgrasptree"
            },
        );
    }
    if args.velocity_motors {
        if args.mode != Mode::Vector {
            return Err("velocity motors use the vector operator".into());
        }
        return run::<MotorTreeSpec>(&xml, &args, "barrier.gzmotorizedtree");
    }
    match distributed_tree_recipe(&xml, &source, "default")? {
        FloatingTreeRecipe::Unlimited(_) => {
            run::<FloatingTreeSpec>(&xml, &args, "barrier.gzfloatingtree")
        }
        FloatingTreeRecipe::Payload(_) => {
            run::<PayloadTreeSpec>(&xml, &args, "barrier.gzpayloadtree")
        }
        FloatingTreeRecipe::Grounded(_) => {
            run::<GroundedTreeSpec>(&xml, &args, "barrier.gzgroundedtree")
        }
        FloatingTreeRecipe::Stopped(_) => {
            run::<StoppedTreeSpec>(&xml, &args, "barrier.gzstoppedtree")
        }
    }
}
fn run<S: TreeConstruction + Send + Sync + 'static>(
    xml: &str,
    args: &Args,
    archive: &str,
) -> Result<()> {
    let source = args.sdf.to_string_lossy();
    let mut regular = load_world_with_sdf_step(
        xml,
        &source,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let temp = tempfile::tempdir_in(&args.storage)?;
    let load = if args.grasp.is_some() {
        load_distributed_grasp_tree_world
    } else if args.velocity_motors {
        load_distributed_motor_tree_world
    } else {
        load_distributed_tree_world
    };
    let open = || match args.placement {
        Some(p) => load_distributed_split_grasp_world(
            xml,
            &source,
            "default",
            &args.worker,
            temp.path(),
            Some(4),
            p,
        ),
        None => load(xml, &source, "default", &args.worker, temp.path(), Some(4)),
    };
    let mut host = open()?;
    let motors = args
        .velocity_motors
        .then(|| tree_motor_drive::MotorDrive::resolve(&regular, &[], args.reverse_every))
        .transpose()?;
    let grasp = args
        .grasp
        .map(|mode| -> Result<_> {
            let be = host.world().resource::<PhysicsBackend>();
            let state = if let Some(s) = be.downcast_ref::<DistributedSplitGraspBackend>() {
                s.pool().payload_state()
            } else {
                backend::<S>(&host)
                    .pool()
                    .observed_payload()
                    .ok_or("missing independent grasp payload")?
                    .0
            };
            grasp_drive::GraspDrive::resolve(&regular, mode, state.id)
        })
        .transpose()?;
    let install = |host: &mut Simulation| -> Result<()> {
        if let Some(mode) = args.grasp {
            grasp_drive::GraspDrive::install(host, mode)?;
        } else if let Some(m) = &motors {
            m.clone().install(host)?;
        } else if args.mode == Mode::Vector {
            floating_tree_drive::install(host, &[], args.reverse_every)?;
        }
        Ok(())
    };
    install(&mut host)?;
    let drive = if args.velocity_motors || args.grasp.is_some() {
        floating_tree_drive::Drive(Vec::new(), None)
    } else {
        floating_tree_drive::resolve(&regular, &[], args.reverse_every)?
    };
    fs::create_dir(&args.output)?;
    let mut states = args
        .record_at
        .map(|_| File::create(args.output.join("states.jsonl")).map(BufWriter::new))
        .transpose()?;
    let mut samples = Vec::with_capacity(args.ticks as usize);
    let mut cut_image_bytes = None;
    for tick in 0..=args.ticks {
        if tick > 0 {
            if let Some(g) = &grasp {
                g.apply(
                    &mut regular.world_mut().resource_mut::<PhysicsBackend>(),
                    u64::from(tick - 1),
                )?;
            }
            if let Some(m) = &motors {
                m.configure(
                    &mut regular.world_mut().resource_mut::<PhysicsBackend>(),
                    u64::from(tick - 1),
                )?;
            }
            for (index, &(public, _, effort)) in drive.0.iter().enumerate() {
                if args.record_at != Some(tick - 1) || !index.is_multiple_of(2) {
                    queue_joint_effort(
                        &mut regular,
                        public,
                        drive.torque(u64::from(tick - 1), effort),
                    )?;
                }
            }
            regular.step_exact(1)?;
            let start = Instant::now();
            if args.mode == Mode::Split {
                for &(public, _, effort) in &drive.0 {
                    queue_joint_effort(
                        &mut host,
                        public,
                        drive.torque(u64::from(tick - 1), effort),
                    )?;
                }
            }
            host.step_exact(1)
                .map_err(|error| gz_sim_cli::with_fault(&host, error))?;
            samples.push(u64::try_from(start.elapsed().as_nanos())?);
        }
        if args.record_at == Some(tick) {
            if let Some(id) = motors
                .as_ref()
                .map(|m| m.inputs[0].0)
                .or_else(|| grasp.as_ref().map(|g| g.first_joint()))
            {
                queue_joint_effort(&mut host, id, -0.037)?;
                queue_joint_effort(&mut regular, id, -0.037)?;
            }
            for &(public, _, effort) in drive.0.iter().step_by(2) {
                let effort = drive.torque(u64::from(tick), effort);
                queue_joint_effort(&mut host, public, effort)?;
                queue_joint_effort(&mut regular, public, effort)?;
            }
            verify::<S>(&host, &regular, tick, false)?;
            let mut be = host.world_mut().resource_mut::<PhysicsBackend>();
            cut_image_bytes = if let Some(tree) = be.downcast_mut::<DistributedSplitGraspBackend>()
            {
                let owners = tree.pool().owners();
                let width = tree.pool().width();
                let dest = (0..width)
                    .find(|s| !owners.contains(s))
                    .ok_or("no spare owner")?;
                tree.transfer(dest)?;
                tree.replace_worker(dest)?;
                if owners[0] != owners[1] {
                    tree.transfer_payload(owners[0])?;
                    tree.replace_worker(owners[0])?;
                }
                tree.pool().image().map(<[u8]>::len)
            } else {
                let tree = be
                    .downcast_mut::<DistributedTreeBackend<S>>()
                    .ok_or("missing distributed tree")?;
                tree.transfer((tree.pool().owner() + 1) % tree.pool().width())?;
                tree.replace_worker(tree.pool().owner())?;
                tree.pool().image().map(<[u8]>::len)
            };
            let dir = args.output.join("cut");
            fs::create_dir(&dir)?;
            fs::copy(temp.path().join(archive), dir.join(archive))?;
            drop(host);
            host = open()?;
            install(&mut host)?;
        }
        if let Some(row) = verify::<S>(&host, &regular, tick, states.is_some())? {
            let file = states.as_mut().unwrap();
            serde_json::to_writer(&mut *file, &row)?;
            writeln!(file)?;
        }
    }
    if let Some(mut f) = states {
        f.flush()?;
    }
    fs::write(
        args.output.join("samples.json"),
        serde_json::to_vec(&samples)?,
    )?;
    samples.sort_unstable();
    let be = host.world().resource::<PhysicsBackend>();
    let split = be
        .downcast_ref::<DistributedSplitGraspBackend>()
        .map(|b| b.pool());
    let (state, motors, limits) = if let Some(p) = split {
        (p.observation(), true, p.joint_limits())
    } else {
        let p = backend::<S>(&host).pool();
        (
            p.observation(),
            p.observed_velocity_motors().is_some(),
            p.joint_limits(),
        )
    };
    let summary = json!({
        "sdf":source, "mode":if args.mode==Mode::Vector {"vector"} else {"split"},
        "ticks":args.ticks, "states":args.ticks+1, "links":state.links.len(),
        "joints":state.joints.len(), "velocity_motors":motors, "placement":args.placement,"placement_stats":split.map(|p|p.stats()),
        "grasp":args.grasp.map(|m|format!("{m:?}").to_ascii_lowercase()),
        "cut_tick":args.record_at, "cut_image_bytes":cut_image_bytes, "joint_limits":limits,
        "reverse_every":args.reverse_every, "p50_ns":samples[(samples.len()-1)/2],
        "p95_ns":samples[(samples.len()*95).div_ceil(100)-1], "total_ns":samples.iter().sum::<u64>()
    });
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
