// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Atomic construction of a supported SDF world and its Bevy simulation systems.

mod lifecycle;
pub use lifecycle::{RemoveError, remove_model};
mod initial;
pub use initial::{
    PendingResetRemoval, SceneResetError, SceneResetInfo, initial_imu_publications,
    reset_loaded_world,
};

use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics_rapier::{
    BodyId, BodySpec, Inertia, PhysicsError, PhysicsReset, RapierBackend, RotationSpec, Shape,
};
use gz_sensors::{ImuInput, ImuReading, InvalidImuInput};
use gz_sim_core::{IdentityError, Phase, Simulation, StepError, UpdateInfo, assign_id};
use sdformat::{Geometry, Model, NoiseKind, ResourceResolver, Sdf, Transform};
use std::{collections::BTreeMap, time::Duration};

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

/// The ownership hierarchy uses Bevy ChildOf/Children, separately from SDF frames.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    World,
    Model,
    Link,
    Collision,
    Visual,
    Sensor,
}

#[derive(Component, Debug)]
pub struct LocalName(pub String);

#[derive(Component, Debug, Clone, Copy)]
pub struct LocalPose(pub WorldPose);

#[derive(Component, Debug, Clone, Copy)]
pub struct WorldPose {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

impl From<Transform> for WorldPose {
    fn from(value: Transform) -> Self {
        Self {
            translation: value.translation,
            rotation: value.rotation,
        }
    }
}

impl WorldPose {
    fn compose(self, child: Self) -> Self {
        let rotation = DQuat::from_array(self.rotation);
        Self {
            translation: (DVec3::from_array(self.translation)
                + rotation * DVec3::from_array(child.translation))
            .to_array(),
            rotation: (rotation * DQuat::from_array(child.rotation)).to_array(),
        }
    }

    fn inverse(self) -> Self {
        let rotation = DQuat::from_array(self.rotation).conjugate();
        Self {
            translation: (rotation * -DVec3::from_array(self.translation)).to_array(),
            rotation: rotation.to_array(),
        }
    }
}

#[derive(Component, Clone)]
struct ModelAnchor {
    body: BodyId,
    link_to_model: WorldPose,
}

#[derive(Component)]
struct AttachedPose;

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

#[derive(Component, Clone)]
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

#[derive(Component, Debug, Clone)]
pub struct ImuPublication {
    pub entity_name: String,
    pub frame_id: String,
    /// Requested topic; the transport boundary applies Gazebo name repair.
    pub topic: String,
}

struct SceneContext<'a> {
    world: &'a str,
    step: Duration,
    bodies: BTreeMap<String, BodyId>,
}

struct ParentModel {
    entity: Entity,
    is_static: bool,
    topic: String,
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
    #[error(transparent)]
    Identity(#[from] IdentityError),
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
    let mut context = SceneContext {
        world: world_name,
        step,
        bodies: BTreeMap::new(),
    };
    let world_entity = spawn_node(
        &mut sim,
        None,
        world_name,
        EntityKind::World,
        Transform::default(),
        Transform::default(),
    )?;
    let parent = ParentModel {
        entity: world_entity,
        is_static: false,
        topic: format!("/world/{world_name}"),
    };
    for model in &world.models {
        build_model(&sdf, model, &mut context, &parent, &mut sim, &mut backend)?;
    }
    sim.world_mut().insert_resource(Gravity(world.gravity));
    gz_physics_rapier::install(&mut sim, backend)?;
    sim.add_systems(
        Phase::PhysicsCommit,
        (
            sync_bodies,
            sync_models,
            sync_local_poses,
            sync_attached_poses,
        )
            .chain(),
    );
    sim.add_systems(Phase::PostStep, sample_imus);
    sim.add_systems(
        Phase::Reset,
        (
            sync_bodies,
            sync_models,
            sync_local_poses,
            sync_attached_poses,
            reset_imus,
        )
            .chain()
            .after(PhysicsReset),
    );
    initial::capture(&mut sim);
    Ok(sim)
}

fn build_model(
    sdf: &Sdf,
    model: &Model,
    context: &mut SceneContext<'_>,
    parent: &ParentModel,
    sim: &mut Simulation,
    backend: &mut RapierBackend,
) -> Result<(), LoadError> {
    require(model.joints.is_empty(), &model.scoped_name, "joints")?;
    require(
        model.plugins.is_empty(),
        &model.scoped_name,
        "model plugins",
    )?;
    let is_static = parent.is_static || model.is_static;
    let model_topic = format!("{}/model/{}", parent.topic, model.name);
    let parent_name = &sim
        .world()
        .get::<Name>(parent.entity)
        .expect("scene owns hierarchy")
        .0;
    let local = sdf.pose_of(&model.scoped_name, parent_name)?;
    let pose = sdf.pose_of(&model.scoped_name, context.world)?;
    let model_entity = spawn_node(
        sim,
        Some(parent.entity),
        &model.scoped_name,
        EntityKind::Model,
        local,
        pose,
    )?;
    let parent = ParentModel {
        entity: model_entity,
        is_static,
        topic: model_topic.clone(),
    };
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
        require(
            link.inertial.fluid_added_mass.is_none(),
            name,
            "fluid added mass",
        )?;
        let collision = &link.collisions[0];
        require(
            collision.surface.source.is_none(),
            name,
            "collision surface parameters",
        )?;
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
        let pose = sdf.pose_of(name, context.world)?;
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
        context.bodies.insert(name.clone(), id);
        let local = sdf.pose_of(name, &model.scoped_name)?;
        let link_entity = spawn_node(sim, Some(model_entity), name, EntityKind::Link, local, pose)?;
        sim.world_mut().entity_mut(link_entity).insert((
            BodyHandle(id),
            Velocity {
                linear: [0.0; 3],
                angular: [0.0; 3],
            },
        ));
        for (child_name, child_pose, kind) in link
            .collisions
            .iter()
            .map(|child| (&child.name, &child.pose, EntityKind::Collision))
            .chain(
                link.visuals
                    .iter()
                    .map(|child| (&child.name, &child.pose, EntityKind::Visual)),
            )
        {
            let local = sdf.resolve_pose(child_pose, &model.scoped_name, name, name)?;
            let world = sdf.resolve_pose(child_pose, &model.scoped_name, name, context.world)?;
            let entity = spawn_node(
                sim,
                Some(link_entity),
                &format!("{name}::{child_name}"),
                kind,
                local,
                world,
            )?;
            sim.world_mut().entity_mut(entity).insert(AttachedPose);
        }
        for sensor in &link.sensors {
            let name = format!("{}::{}", link.scoped_name, sensor.name);
            require(!sensor.enable_metrics, &name, "sensor metrics")?;
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
                    || (sensor.update_rate * context.step.as_secs_f64() - 1.0).abs() < 1e-12,
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
            let entity_name = name
                .strip_prefix(&format!("{}::", context.world))
                .expect("scene names include their world scope")
                .to_owned();
            let topic = if sensor.topic.is_empty() {
                format!(
                    "{model_topic}/link/{}/sensor/{}/imu",
                    link.name, sensor.name
                )
            } else {
                sensor.topic.clone()
            };
            let world = sdf.resolve_pose(
                &sensor.pose,
                &model.scoped_name,
                &link.scoped_name,
                context.world,
            )?;
            let sensor_entity = spawn_node(
                sim,
                Some(link_entity),
                &name,
                EntityKind::Sensor,
                local,
                world,
            )?;
            sim.world_mut().entity_mut(sensor_entity).insert((
                AttachedPose,
                ImuPublication {
                    frame_id: if sensor.frame_id.is_empty() {
                        sensor
                            .gz_frame_id
                            .clone()
                            .unwrap_or_else(|| entity_name.clone())
                    } else {
                        sensor.frame_id.clone()
                    },
                    entity_name,
                    topic,
                },
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
        build_model(sdf, child, context, &parent, sim, backend)?;
    }
    if !is_static {
        let (canonical, _) = model.canonical_link_and_relative_name();
        let canonical = canonical.ok_or_else(|| LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "dynamic model without a canonical link",
        })?;
        let body = context
            .bodies
            .get(&canonical.scoped_name)
            .copied()
            .expect("scene constructs every canonical link before its model anchor");
        let model_to_link =
            WorldPose::from(sdf.pose_of(&canonical.scoped_name, &model.scoped_name)?);
        sim.world_mut()
            .entity_mut(model_entity)
            .insert(ModelAnchor {
                body,
                link_to_model: model_to_link.inverse(),
            });
    }
    Ok(())
}

fn spawn_node(
    sim: &mut Simulation,
    parent: Option<Entity>,
    name: &str,
    kind: EntityKind,
    local: Transform,
    world: Transform,
) -> Result<Entity, LoadError> {
    let entity = sim
        .world_mut()
        .spawn((
            Name(name.into()),
            LocalName(
                name.rsplit("::")
                    .next()
                    .expect("name contains at least one segment")
                    .into(),
            ),
            kind,
            LocalPose(local.into()),
            WorldPose::from(world),
        ))
        .id();
    if let Some(parent) = parent {
        sim.world_mut().entity_mut(entity).insert(ChildOf(parent));
    }
    assign_id(sim.world_mut(), entity)?;
    Ok(entity)
}

fn sync_models(backend: Res<RapierBackend>, mut models: Query<(&ModelAnchor, &mut WorldPose)>) {
    for (anchor, mut pose) in &mut models {
        let state = backend
            .state(anchor.body)
            .expect("scene owns canonical link lifecycle");
        *pose = WorldPose {
            translation: state.position,
            rotation: state.orientation,
        }
        .compose(anchor.link_to_model);
    }
}

type PoseUpdateFilter = (Without<AttachedPose>, Without<PendingResetRemoval>);

fn sync_local_poses(
    parents: Query<&WorldPose>,
    mut nodes: Query<(&ChildOf, &WorldPose, &mut LocalPose), PoseUpdateFilter>,
) {
    for (parent, world, mut local) in &mut nodes {
        local.0 = parents
            .get(parent.parent())
            .expect("scene owns hierarchy")
            .inverse()
            .compose(*world);
    }
}

fn sync_attached_poses(
    parents: Query<&WorldPose, Without<AttachedPose>>,
    mut nodes: Query<(&ChildOf, &LocalPose, &mut WorldPose), With<AttachedPose>>,
) {
    for (parent, local, mut world) in &mut nodes {
        *world = parents
            .get(parent.parent())
            .expect("attached poses have link parents")
            .compose(local.0);
    }
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
