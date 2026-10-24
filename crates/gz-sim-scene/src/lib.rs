// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Atomic construction of a supported SDF world and its Bevy simulation systems.

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics_rapier::{
    BodyId, BodySpec, Inertia, PhysicsError, PhysicsReset, RapierBackend, RotationSpec, Shape,
};
use gz_sensors::{ImuInput, ImuReading, InvalidImuInput};
use gz_sim_core::{Phase, Simulation, StepError, UpdateInfo};
use sdformat::{Geometry, Model, NoiseKind, ResourceResolver, Sdf, Transform};
use std::time::Duration;

// Semantic frame composition introduces rounding even for coincident frames.
const POSE_TOLERANCE: f64 = 1e-12;

fn centered(translation: [f64; 3]) -> bool {
    translation.into_iter().all(|v| v.abs() <= POSE_TOLERANCE)
}

fn identity(pose: Transform) -> bool {
    centered(pose.translation)
        && centered([pose.rotation[0], pose.rotation[1], pose.rotation[2]])
        && (pose.rotation[3].abs() - 1.0).abs() <= POSE_TOLERANCE
}

#[derive(Component, Debug)]
pub struct Name(pub String);

#[derive(Component, Debug, Clone, Copy)]
pub struct WorldPose {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

#[derive(Component, Debug, Clone, Copy)]
pub struct Velocity {
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

#[derive(Component)]
pub struct BodyHandle(BodyId);

impl BodyHandle {
    pub fn id(&self) -> BodyId {
        self.0
    }
}

#[derive(Component)]
struct Imu {
    body: BodyId,
    rotation: DQuat,
    reference: DQuat,
    previous_velocity: DVec3,
}

#[derive(Component, Debug)]
pub struct ImuSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<ImuReading, InvalidImuInput>,
}

#[derive(Resource)]
struct Gravity([f64; 3]);

#[derive(Debug, thiserror::Error)]
pub enum LoadError {
    #[error("{0}")]
    Sdf(#[from] sdformat::Diagnostic),
    #[error("world {0} was not found")]
    MissingWorld(String),
    #[error("{entity}: unsupported {feature}")]
    Unsupported {
        entity: String,
        feature: &'static str,
    },
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Clock(#[from] StepError),
}

fn require(supported: bool, entity: &str, feature: &'static str) -> Result<(), LoadError> {
    if supported {
        Ok(())
    } else {
        Err(LoadError::Unsupported {
            entity: entity.into(),
            feature,
        })
    }
}

/// Creates a fresh world or returns an error without changing any existing simulation.
/// The current backend accepts unjointed models with one centered box/sphere per link.
pub fn load_world(
    xml: &str,
    source: &str,
    world_name: &str,
    step: Duration,
    resolver: &dyn ResourceResolver,
) -> Result<Simulation, LoadError> {
    let sdf = sdformat::load_str(xml, source, resolver)?;
    sdf.validate_non_frame_poses()?;
    let world = sdf
        .worlds
        .iter()
        .find(|world| world.name == world_name)
        .ok_or_else(|| LoadError::MissingWorld(world_name.into()))?;
    require(world.plugins.is_empty(), world_name, "world plugins")?;
    let mut sim = Simulation::new(step)?;
    let mut backend = RapierBackend::new(world.gravity)?;
    for model in &world.models {
        build_model(&sdf, model, world_name, false, step, &mut sim, &mut backend)?;
    }
    sim.world_mut().insert_resource(Gravity(world.gravity));
    gz_physics_rapier::install(&mut sim, backend)?;
    sim.add_systems(Phase::PhysicsCommit, sync_bodies);
    sim.add_systems(Phase::PostStep, sample_imus);
    sim.add_systems(
        Phase::Reset,
        (sync_bodies, reset_imus).chain().after(PhysicsReset),
    );
    Ok(sim)
}

fn build_model(
    sdf: &Sdf,
    model: &Model,
    world_name: &str,
    parent_static: bool,
    step: Duration,
    sim: &mut Simulation,
    backend: &mut RapierBackend,
) -> Result<(), LoadError> {
    require(model.joints.is_empty(), &model.scoped_name, "joints")?;
    require(
        model.plugins.is_empty(),
        &model.scoped_name,
        "model plugins",
    )?;
    let is_static = parent_static || model.is_static;
    for link in &model.links {
        let name = &link.scoped_name;
        require(link.plugins.is_empty(), name, "link plugins")?;
        require(
            link.visuals.iter().all(|visual| visual.plugins.is_empty()),
            name,
            "visual plugins",
        )?;
        require(
            link.collisions.len() == 1,
            name,
            "collision count (expected one)",
        )?;
        let collision = &link.collisions[0];
        let local = sdf.resolve_pose(&collision.pose, &model.scoped_name, name, name)?;
        require(identity(local), name, "offset collision pose")?;
        let shape = match collision.geometry {
            Geometry::Sphere { radius } => Shape::Sphere { radius },
            Geometry::Box { size } => Shape::Box {
                half_extents: size.map(|v| v * 0.5),
            },
            _ => {
                return Err(LoadError::Unsupported {
                    entity: name.clone(),
                    feature: "collision geometry",
                });
            }
        };
        let pose = sdf.pose_of(name, world_name)?;
        let inertial_pose =
            sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, name, name)?;
        require(
            centered(inertial_pose.translation),
            name,
            "offset center of mass",
        )?;
        let id = backend.spawn_link(
            BodySpec {
                shape,
                position: pose.translation,
                velocity: [0.0; 3],
                mass: (!is_static).then_some(link.inertial.mass),
            },
            RotationSpec {
                orientation: pose.rotation,
                ..Default::default()
            },
            Some(Inertia {
                center: inertial_pose.translation,
                orientation: inertial_pose.rotation,
                tensor: link.inertial.inertia,
            }),
        )?;
        sim.world_mut().spawn((
            Name(name.clone()),
            BodyHandle(id),
            WorldPose {
                translation: pose.translation,
                rotation: pose.rotation,
            },
            Velocity {
                linear: [0.0; 3],
                angular: [0.0; 3],
            },
        ));
        for sensor in &link.sensors {
            let name = format!("{}::{}", link.scoped_name, sensor.name);
            let config = &sensor.imu;
            require(
                config
                    .angular_velocity
                    .iter()
                    .chain(&config.linear_acceleration)
                    .all(|noise| noise.kind == NoiseKind::None),
                &name,
                "IMU noise",
            )?;
            require(
                config.enable_orientation
                    && config.localization == "CUSTOM"
                    && config.custom_rpy == [0.0; 3]
                    && config.custom_rpy_parent_frame.is_empty(),
                &name,
                "IMU orientation configuration",
            )?;
            require(
                sensor.update_rate == 0.0
                    || (sensor.update_rate * step.as_secs_f64() - 1.0).abs() < 1e-12,
                &name,
                "sensor rate differing from physics rate",
            )?;
            let local = sdf.resolve_pose(
                &sensor.pose,
                &model.scoped_name,
                &link.scoped_name,
                &link.scoped_name,
            )?;
            require(centered(local.translation), &name, "offset IMU position")?;
            sim.world_mut().spawn((
                Name(name),
                Imu {
                    body: id,
                    rotation: DQuat::from_array(local.rotation),
                    reference: DQuat::from_array(pose.rotation) * DQuat::from_array(local.rotation),
                    previous_velocity: DVec3::ZERO,
                },
            ));
        }
    }
    for child in &model.models {
        build_model(sdf, child, world_name, is_static, step, sim, backend)?;
    }
    Ok(())
}

fn sync_bodies(
    backend: Res<RapierBackend>,
    mut bodies: Query<(&BodyHandle, &mut WorldPose, &mut Velocity)>,
) {
    for (body, mut pose, mut velocity) in &mut bodies {
        let state = backend.state(body.0).expect("scene owns body lifecycle");
        pose.translation = state.position;
        pose.rotation = state.orientation;
        velocity.linear = state.velocity;
        velocity.angular = state.angular_velocity;
    }
}

fn sample_imus(
    time: Res<UpdateInfo>,
    gravity: Res<Gravity>,
    backend: Res<RapierBackend>,
    mut sensors: Query<(Entity, &mut Imu)>,
    mut commands: Commands,
) {
    if time.dt.is_zero() {
        return;
    }
    for (entity, mut sensor) in &mut sensors {
        let state = backend
            .state(sensor.body)
            .expect("scene owns body lifecycle");
        let velocity = DVec3::from_array(state.velocity);
        let acceleration = (velocity - sensor.previous_velocity) / time.dt.as_secs_f64();
        sensor.previous_velocity = velocity;
        let rotation = DQuat::from_array(state.orientation) * sensor.rotation;
        commands.entity(entity).insert(ImuSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading: gz_sensors::imu(ImuInput {
                world_orientation: rotation.to_array(),
                reference_orientation: sensor.reference.to_array(),
                acceleration_sensor: (rotation.inverse() * acceleration).to_array(),
                angular_velocity_sensor: (rotation.inverse()
                    * DVec3::from_array(state.angular_velocity))
                .to_array(),
                gravity_world: gravity.0,
            }),
        });
    }
}

fn reset_imus(mut sensors: Query<(Entity, &mut Imu)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.previous_velocity = DVec3::ZERO;
        commands.entity(entity).remove::<ImuSample>();
    }
}
