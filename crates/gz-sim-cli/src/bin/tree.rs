// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Inspect and control a durable multi-revolute Gazebo session.
use clap::{Parser, Subcommand};
use gz_sim_scene::distributed_hinge::{
    FloatingTreePool, FloatingTreeRecipe, GroundedTreePool, MotorTreePool, StoppedTreePool,
    TreeConstruction, TreePool, distributed_tree_recipe,
};
use gz_sim_scene::distributed_hinge::{
    GraspFleetLayout, GraspFleetSpec, GraspPlacement, GraspTreePool, GraspTreeSpec,
    ManagedGraspFleet, PayloadTreePool, SplitGraspPool, TreeSceneSession,
    distributed_grasp_fleet_specs,
};
use serde_json::json;
use std::{
    error::Error,
    fs::File,
    io::{self, Read},
    path::PathBuf,
};

#[derive(Parser)]
#[command(
    version,
    about = "Drive a floating tree of three to 64 spherical links on local physics workers"
)]
struct Args {
    /// Trusted local session directory.
    session: PathBuf,
    /// Stock physics_worker executable, normally next to this binary.
    #[arg(long, global = true)]
    worker: Option<PathBuf>,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Initialize the session and print the public IDs of the links and joints.
    Create {
        sdf: PathBuf,
        /// Admit persistent bounded native velocity motors in this session.
        #[arg(long)]
        motorized: bool,
        /// Select separate ownership after N consecutive geometrically clear intervals.
        #[arg(long,requires="motorized",value_name="together|adaptive=N",value_parser=gz_sim_cli::parse_grasp_placement)]
        placement: Option<GraspPlacement>,
        /// Distribute the SDF's named gripper/payload pairs as one atomic fleet.
        #[arg(long,conflicts_with_all=["motorized","placement"],value_name="packed|round-robin|coupled|adaptive|warm|replay=INTERVALS,EVENTS",value_parser=gz_sim_cli::parse_grasp_fleet_placement)]
        fleet: Option<GraspFleetLayout>,
        #[arg(long, default_value = "default")]
        world: String,
        #[arg(long,default_value_t=2,value_parser=clap::value_parser!(u8).range(2..=8))]
        workers: u8,
    },
    /// Read the last committed coordinates, velocities, held effort and ownership.
    Inspect,
    /// Durably add signed torque to one public joint for the next advancing interval.
    Queue {
        joint: u64,
        #[arg(allow_hyphen_values=true,value_parser=finite)]
        torque_nm: f64,
    },
    /// Replace one persistent velocity motor without advancing the clock.
    Motor {
        joint: u64,
        #[arg(allow_hyphen_values=true,value_parser=finite)]
        velocity_rad_s: f64,
        #[arg(value_parser=nonnegative)]
        max_torque_nm: f64,
    },
    /// Disable one motor and preserve separately queued joint effort.
    MotorOff { joint: u64 },
    /// Atomically advance one millisecond with optional torques on distinct joints.
    Step {
        #[arg(long,value_name="ID=NM",value_parser=parse_contribution)]
        effort: Vec<(u64, f64)>,
        /// One-interval world-frame force at the independently moving payload COM, in newtons.
        #[arg(long,value_name="X,Y,Z",allow_hyphen_values=true,value_parser=parse_force)]
        payload_force: Option<[f64; 3]>,
        /// World force at the specified fleet payload, applied for this interval. Repeat for distinct payloads.
        #[arg(long,conflicts_with="payload_force",value_name="PAYLOAD_ID=X,Y,Z",value_parser=parse_load)]
        load: Vec<(u64, [f64; 3])>,
        #[arg(long, requires = "expect_tick")]
        expect_epoch: Option<u64>,
        #[arg(long, requires = "expect_epoch")]
        expect_tick: Option<u64>,
    },
    /// Transfer the complete articulated tree at the current committed clock.
    Transfer {
        slot: u8,
        #[arg(long)]
        model: Option<u64>,
    },
    /// Move the independently placed payload to a worker at the current barrier.
    TransferPayload { slot: u8 },
    /// Replace one worker process and reinstall its last accepted state.
    Recover { slot: u8 },
    /// Restore authored state and start a new epoch.
    Reset,
}

fn finite(text: &str) -> Result<f64, String> {
    let v = text.parse::<f64>().map_err(|e| e.to_string())?;
    if !v.is_finite() {
        return Err("value must be finite".into());
    }
    Ok(v)
}
fn nonnegative(text: &str) -> Result<f64, String> {
    let v = finite(text)?;
    if v < 0. {
        return Err("maximum torque must be nonnegative".into());
    }
    Ok(v)
}
fn parse_contribution(text: &str) -> Result<(u64, f64), String> {
    let (id, torque) = text
        .split_once('=')
        .ok_or("expected public_joint_id=torque_nm")?;
    Ok((
        id.parse::<u64>().map_err(|e| e.to_string())?,
        finite(torque)?,
    ))
}
fn parse_force(text: &str) -> Result<[f64; 3], String> {
    let mut entries = text.split(',');
    let mut force = [0.; 3];
    for value in &mut force {
        *value = finite(entries.next().ok_or("expected force components X,Y,Z")?)?;
    }
    if entries.next().is_some() {
        return Err("expected force components X,Y,Z".into());
    }
    Ok(force)
}
fn parse_load(text: &str) -> Result<(u64, [f64; 3]), String> {
    let (id, force) = text.split_once('=').ok_or("expected PAYLOAD_ID=X,Y,Z")?;
    Ok((
        id.parse::<u64>().map_err(|e| e.to_string())?,
        parse_force(force)?,
    ))
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let worker = match args.worker {
        Some(w) => w,
        None => std::env::current_exe()?.with_file_name("physics_worker"),
    };
    if let Command::Create {
        sdf,
        world,
        workers,
        motorized,
        placement,
        fleet,
    } = args.command
    {
        let mut xml = String::new();
        File::open(&sdf)?
            .take(1024 * 1024 + 1)
            .read_to_string(&mut xml)?;
        if let Some(placement) = fleet {
            let assemblies = distributed_grasp_fleet_specs(&xml, &sdf.to_string_lossy(), &world)?;
            let pool = ManagedGraspFleet::create(
                &worker,
                &args.session,
                workers,
                GraspFleetSpec { assemblies },
                placement,
            )?;
            return operate_fleet(pool, Command::Inspect);
        }
        let recipe = distributed_tree_recipe(&xml, &sdf.to_string_lossy(), &world)?;
        if let Some(policy) = placement {
            let FloatingTreeRecipe::Payload(payload) = recipe else {
                return Err("grasp placement requires a free payload".into());
            };
            let pool = SplitGraspPool::create(
                &worker,
                &args.session,
                workers,
                GraspTreeSpec { payload },
                policy,
            )?;
            return operate_split(pool, Command::Inspect);
        }
        if motorized {
            if let FloatingTreeRecipe::Payload(payload) = recipe {
                return operate(
                    GraspTreePool::create(
                        &worker,
                        &args.session,
                        workers,
                        GraspTreeSpec { payload },
                    )?,
                    Command::Inspect,
                );
            }
            return operate(
                MotorTreePool::create(&worker, &args.session, workers, recipe.into_motor(&world)?)?,
                Command::Inspect,
            );
        }
        match recipe {
            FloatingTreeRecipe::Unlimited(spec) => {
                return operate(
                    FloatingTreePool::create(&worker, &args.session, workers, spec)?,
                    Command::Inspect,
                );
            }
            FloatingTreeRecipe::Payload(spec) => {
                return operate(
                    PayloadTreePool::create(&worker, &args.session, workers, spec)?,
                    Command::Inspect,
                );
            }
            FloatingTreeRecipe::Grounded(spec) => {
                return operate(
                    GroundedTreePool::create(&worker, &args.session, workers, spec)?,
                    Command::Inspect,
                );
            }
            FloatingTreeRecipe::Stopped(spec) => {
                return operate(
                    StoppedTreePool::create(&worker, &args.session, workers, spec)?,
                    Command::Inspect,
                );
            }
        }
    }
    // Typed resume locks and validates the complete ledger before admission.
    if args
        .session
        .join(GraspFleetLayout::Adaptive.archive())
        .try_exists()?
        || args
            .session
            .join(GraspFleetLayout::Packed.archive())
            .try_exists()?
        || args
            .session
            .join(GraspFleetLayout::Coupled.archive())
            .try_exists()?
    {
        return operate_fleet(
            ManagedGraspFleet::resume(&worker, &args.session)?,
            args.command,
        );
    }
    if args.session.join("barrier.gzsplitgrasp").try_exists()? {
        return operate_split(
            SplitGraspPool::resume(&worker, &args.session)?,
            args.command,
        );
    }
    if args.session.join("barrier.gzgrasptree").try_exists()? {
        return operate(GraspTreePool::resume(&worker, &args.session)?, args.command);
    }
    if args.session.join("barrier.gzpayloadtree").try_exists()? {
        return operate(
            PayloadTreePool::resume(&worker, &args.session)?,
            args.command,
        );
    }
    if args.session.join("barrier.gzgroundedtree").try_exists()? {
        return operate(
            GroundedTreePool::resume(&worker, &args.session)?,
            args.command,
        );
    }
    if args.session.join("barrier.gzmotorizedtree").try_exists()? {
        return operate(MotorTreePool::resume(&worker, &args.session)?, args.command);
    }
    if args.session.join("barrier.gzstoppedtree").try_exists()? {
        operate(
            StoppedTreePool::resume(&worker, &args.session)?,
            args.command,
        )
    } else {
        operate(
            FloatingTreePool::resume(&worker, &args.session)?,
            args.command,
        )
    }
}

fn execute<P: TreeSceneSession>(pool: &mut P, command: Command) -> Result<(), Box<dyn Error>> {
    match command {
        Command::Inspect => {}
        Command::Motor {
            joint,
            velocity_rad_s,
            max_torque_nm,
        } => {
            pool.set_motor(
                joint,
                Some(gz_physics::JointVelocityMotor {
                    target_velocity: velocity_rad_s,
                    max_force: max_torque_nm,
                }),
            )?;
        }
        Command::MotorOff { joint } => pool.set_motor(joint, None)?,

        Command::Queue { joint, torque_nm } => pool.queue_effort(joint, torque_nm)?,
        Command::Step {
            effort,
            payload_force,
            load,
            expect_epoch,
            expect_tick,
        } => {
            if !load.is_empty() {
                return Err("addressed payload loads require a grasp fleet".into());
            }
            if payload_force.is_some() && !pool.supports_payload_force() {
                return Err(
                    "payload forces require a motorized gripper and independent payload".into(),
                );
            }
            let clock = expect_epoch
                .zip(expect_tick)
                .unwrap_or((pool.epoch(), pool.observation().tick));
            pool.advance_commands(Some(clock), &effort, payload_force.unwrap_or([0.; 3]))?;
        }
        Command::TransferPayload { slot } => {
            let p = (pool as &mut dyn std::any::Any)
                .downcast_mut::<SplitGraspPool>()
                .ok_or("moving the payload separately requires a placement session")?;
            p.transfer_payload(slot)?;
        }
        Command::Transfer { slot, model: None } => pool.transfer(slot)?,
        Command::Transfer { model: Some(_), .. } => {
            return Err("named model transfer requires a grasp fleet".into());
        }
        Command::Recover { slot } => pool.replace_worker(slot)?,
        Command::Reset => pool.reset()?,
        Command::Create { .. } => unreachable!(),
    }
    Ok(())
}
fn operate<S: TreeConstruction + Send + Sync + 'static>(
    mut pool: TreePool<S>,
    command: Command,
) -> Result<(), Box<dyn Error>> {
    execute(&mut pool, command)?;
    let ids = || pool.observation().links.iter().map(|l| l.id);
    let payload = pool
        .observed_payload()
        .map(|(s, r)| json!({"state":s,"reactions":contact_rows(ids(),r)}));
    let ground = (&pool as &dyn std::any::Any)
        .downcast_ref::<GroundedTreePool>()
        .map(|p| {
            json!({"link_id":p.spec().ground.link_id,"reactions":contact_rows(ids(),p.ground_reactions())})
        });
    let motors = pool.observed_velocity_motors().map(|settings| {
        motor_rows(
            pool.observation()
                .joints
                .iter()
                .map(|j| j.joint_id)
                .zip(settings),
        )
    });
    serde_json::to_writer(
        io::stdout().lock(),
        &json!({
            "epoch": pool.epoch(), "generation": pool.generation(), "owner": pool.owner(),
            "workers": pool.width(), "model_id": pool.topology().model_id,
            "joint_topology": pool.topology().joints, "joint_limits": pool.joint_limits(),
            "velocity_motors": motors, "ground": ground, "payload": payload, "state": pool.observation()
        }),
    )?;
    println!();
    Ok(())
}

fn contact_rows(
    ids: impl Iterator<Item = u64>,
    reactions: &[impl serde::Serialize],
) -> Vec<serde_json::Value> {
    ids.zip(reactions)
        .map(|(id, r)| json!({"link_id":id,"reaction":r}))
        .collect()
}
fn motor_rows(
    settings: impl Iterator<Item = (u64, Option<gz_physics::JointVelocityMotor>)>,
) -> Vec<serde_json::Value> {
    settings.map(|(id,m)|json!({
        "joint_id":id, "target_velocity":m.map(|v|v.target_velocity), "max_force":m.map(|v|v.max_force)
    })).collect()
}
fn operate_split(mut pool: SplitGraspPool, command: Command) -> Result<(), Box<dyn Error>> {
    execute(&mut pool, command)?;
    let state = pool.observation();
    let reactions = contact_rows(state.links.iter().map(|l| l.id), pool.payload_reactions());
    let motors = motor_rows(
        state
            .joints
            .iter()
            .map(|j| (j.joint_id, pool.velocity_motor(j.joint_id))),
    );
    serde_json::to_writer(
        io::stdout().lock(),
        &json!({
            "epoch":pool.epoch(),"generation":pool.generation(),"owner":pool.owners()[0],"owners":pool.owners(),
            "workers":pool.width(),"model_id":pool.topology().model_id,"joint_topology":pool.topology().joints,
            "joint_limits":pool.joint_limits(),"velocity_motors":motors,"ground":null,
            "payload":{"state":pool.payload_state(),"reactions":reactions},"state":state,
            "placement":pool.placement(),"placement_stats":pool.stats()
        }),
    )?;
    println!();
    Ok(())
}

fn operate_fleet(mut pool: ManagedGraspFleet, command: Command) -> Result<(), Box<dyn Error>> {
    match command {
        Command::Inspect => {}
        Command::Queue { joint, torque_nm } => pool.queue_effort(joint, torque_nm)?,
        Command::Motor {
            joint,
            velocity_rad_s,
            max_torque_nm,
        } => pool.set_velocity_motor(
            joint,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: velocity_rad_s,
                max_force: max_torque_nm,
            }),
        )?,
        Command::MotorOff { joint } => pool.set_velocity_motor(joint, None)?,
        Command::Step {
            effort,
            payload_force,
            load,
            expect_epoch,
            expect_tick,
        } => {
            if payload_force.is_some() {
                return Err("specify the fleet payload with --load ID=X,Y,Z".into());
            }
            let (e, t) = expect_epoch
                .zip(expect_tick)
                .unwrap_or((pool.epoch(), pool.tick()));
            pool.advance_with_inputs_at(e, t, &effort, &load)?;
        }
        Command::Transfer { slot, model } => pool.transfer(model, slot)?,
        Command::TransferPayload { .. } => {
            return Err("fleet placement keeps each gripper with its assigned payload".into());
        }
        Command::Recover { slot } => pool.replace_worker(slot)?,
        Command::Reset => pool.reset()?,
        Command::Create { .. } => unreachable!(),
    }
    let mut assemblies = Vec::with_capacity(pool.owners().len());
    for (a, &owner) in pool.spec().assemblies.iter().zip(pool.owners()) {
        let tree = &a.payload.tree;
        let state = pool
            .observation(tree.model_id)
            .ok_or("missing fleet group")?;
        let reactions = pool
            .payload_reactions(tree.model_id)
            .ok_or("missing fleet reactions")?;
        let motors = motor_rows(
            state
                .joints
                .iter()
                .map(|j| (j.joint_id, pool.velocity_motor(j.joint_id))),
        );
        assemblies.push(json!({"owner":owner,"model_id":tree.model_id,"state":state,"joint_topology":tree.joints,
            "joint_limits":a.payload.limits,"velocity_motors":motors,"payload":{"state":pool.payload_state(tree.model_id),"reactions":contact_rows(state.links.iter().map(|j|j.id),reactions)}}));
    }
    serde_json::to_writer(
        io::stdout().lock(),
        &json!({"epoch":pool.epoch(),"tick":pool.tick(),"generation":pool.generation(),"owners":pool.owners(),
        "placement":pool.layout(),"coupled":pool.is_coupled(),"joined_at":pool.joined_at(),"workers":pool.width(),"assemblies":assemblies,"cross_reactions":pool.cross_reactions()}),
    )?;
    println!();
    Ok(())
}
