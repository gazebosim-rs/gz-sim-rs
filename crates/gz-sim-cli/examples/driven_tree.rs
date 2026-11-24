// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Serve a worker-owned multi-joint Gazebo articulation using the standard live gz-web-rs UI.
#[path = "support/floating_tree_drive.rs"]
mod floating_tree_drive;
#[allow(dead_code)]
#[path = "support/grasp_drive.rs"]
mod grasp_drive;
#[path = "support/grasp_fleet_drive.rs"]
mod grasp_fleet_drive;
#[path = "support/tree_motor_drive.rs"]
mod tree_motor_drive;
use clap::Parser;
use gz_sim_cli::live::{self, ViewerOptions};
use gz_sim_scene::distributed_hinge::{
    GraspFleetLayout, GraspPlacement, load_distributed_grasp_fleet_world,
    load_distributed_grasp_tree_world, load_distributed_motor_tree_world,
    load_distributed_split_grasp_world, load_distributed_tree_world,
};
use std::{error::Error, fs, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// Isolated, switchable, spherical multi-revolute SDF.
    #[arg(long)]
    sdf: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    session: PathBuf,
    /// New sessions default to two workers. Omit to adopt a saved session width.
    #[arg(long,value_parser=clap::value_parser!(u8).range(2..=8))]
    workers: Option<u8>,
    /// Use the default alternating signed drive when omitted. Repeat for multiple joints.
    #[arg(long,value_name="JOINT_NAME=NM",value_parser=floating_tree_drive::parse)]
    drive: Vec<(String, f64)>,
    /// Use native velocity motors in place of the default continuous torque drive.
    #[arg(long, conflicts_with = "drive")]
    velocity_motors: bool,
    /// Close both fingers, apply a transverse 1.2 N load and release at 1.100 s.
    #[arg(long,value_enum,conflicts_with_all=["velocity_motors","drive","reverse_every"])]
    grasp: Option<grasp_drive::Grip>,
    /// Route each named gripper and its assigned payload in an authored GraspFleet world.
    #[arg(long,conflicts_with_all=["grasp","drive","velocity_motors","reverse_every"],value_name="packed|round-robin|coupled|adaptive|warm|replay=INTERVALS,EVENTS",value_parser=gz_sim_cli::parse_grasp_fleet_placement)]
    fleet: Option<GraspFleetLayout>,
    /// Physical exercise for the named gripper fleet.
    #[arg(long, requires = "fleet", value_enum)]
    fleet_schedule: Option<grasp_fleet_drive::FleetSchedule>,
    /// Worker placement for a gripper. Adaptive dwell counts consecutive clear intervals.
    #[arg(long,requires="grasp",value_name="together|adaptive=N",value_parser=gz_sim_cli::parse_grasp_placement)]
    placement: Option<GraspPlacement>,
    /// Select named motors. Repeat for simultaneous native motors.
    #[arg(long,value_name="JOINT=RAD_PER_SEC,MAX_NM",value_parser=tree_motor_drive::parse,requires="velocity_motors")]
    motor: Vec<(String, gz_physics::JointVelocityMotor)>,
    /// Reverse all selected inputs at this many 1 ms intervals.
    #[arg(long)]
    reverse_every: Option<std::num::NonZeroU64>,
    #[arg(long,default_value_t=9250,value_parser=clap::value_parser!(u16).range(1..))]
    web_viewer: u16,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let xml = fs::read_to_string(&args.sdf)?;
    let load = if args.grasp.is_some() {
        load_distributed_grasp_tree_world
    } else if args.velocity_motors {
        load_distributed_motor_tree_world
    } else {
        load_distributed_tree_world
    };
    let source = args.sdf.to_string_lossy();
    let mut sim = if let Some(p) = args.fleet {
        load_distributed_grasp_fleet_world(
            &xml,
            &source,
            "default",
            &args.worker,
            &args.session,
            args.workers,
            p,
        )?
    } else if let Some(policy) = args.placement {
        load_distributed_split_grasp_world(
            &xml,
            &source,
            "default",
            &args.worker,
            &args.session,
            args.workers,
            policy,
        )?
    } else {
        load(
            &xml,
            &source,
            "default",
            &args.worker,
            &args.session,
            args.workers,
        )?
    };
    if args.fleet.is_some() {
        grasp_fleet_drive::FleetDrive::install(
            &mut sim,
            args.fleet_schedule
                .unwrap_or(grasp_fleet_drive::FleetSchedule::Bilateral),
        )?;
    } else if let Some(mode) = args.grasp {
        grasp_drive::GraspDrive::install(&mut sim, mode)?;
    } else if args.velocity_motors {
        tree_motor_drive::MotorDrive::resolve(&sim, &args.motor, args.reverse_every)?
            .install(&mut sim)?;
    } else {
        floating_tree_drive::install(&mut sim, &args.drive, args.reverse_every)?;
    }
    live::run(
        sim,
        "default",
        false,
        0,
        None,
        ViewerOptions {
            port: Some(args.web_viewer),
            lifecycle_sdf: None,
            recording_path: None,
        },
    )?;
    Ok(())
}
