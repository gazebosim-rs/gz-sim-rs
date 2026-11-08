// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::resource::IsResource;
use bevy_ecs::{observer::Observer, system::SystemIdMarker};
use gz_physics::{InitialPhysicsState, PhysicsBackend, RestoredPhysics};
use gz_sim_core::{EntityId, ReservedId, entity_by_id, reserve_id, restore_id};
use std::{collections::BTreeSet, sync::Arc};

/// Runtime additions remain observable during reset, then retire after observation.
#[derive(Component)]
pub struct PendingResetRemoval;

#[derive(Resource, Debug)]
pub struct SceneResetInfo {
    pub previous_time: Duration,
    pub epoch: u64,
    pub restored_ids: Vec<u64>,
    pub removed_ids: Vec<u64>,
}

#[derive(Debug, thiserror::Error)]
pub enum SceneResetError {
    #[error("world has no loaded-scene initial state")]
    MissingInitialState,
    #[error("scene physics backend is missing")]
    MissingPhysics,
    #[error("a reset handler failed after scene reconstruction; the run is invalid")]
    CommittedReset,
    #[error("runtime physics bindings cannot be restored by this loader")]
    RuntimePhysics,
    #[error("resources and Bevy infrastructure must not belong to the scene hierarchy")]
    InfrastructureOwnership,
    #[error("initial TPE scene belongs to a different backend")]
    ForeignPhysics,
    #[error(transparent)]
    Identity(#[from] IdentityError),
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Clock(#[from] StepError),
}

struct Node {
    id: ReservedId,
    parent: Option<u64>,
    name: String,
    local_name: String,
    kind: EntityKind,
    local: LocalPose,
    pose: WorldPose,
    body: Option<BodyId>,
    joint: Option<JointHandle>,
    joint_definition: Option<JointDefinition>,
    joint_force: Option<JointForceConfiguration>,
    drive: Option<crate::diff_drive::Drive>,
    controller: Option<crate::joint_controller::Controller>,
    controller_configuration: Option<JointControllerConfiguration>,
    drive_configuration: Option<DiffDriveConfiguration>,
    joint_sample: Option<JointSample>,
    motion: Option<gz_physics_tpe::Motion>,
    tpe_link: Option<tpe::TpeLink>,
    tpe_bounds: Option<gz_physics_tpe::ModelBounds>,
    velocity: Option<Velocity>,
    center: Option<LinkCenterOfMass>,
    anchor: Option<ModelAnchor>,
    attached: bool,
    imu: Option<Imu>,
    lidar: Option<crate::lidar::Lidar>,
    lidar_publication: Option<LidarPublication>,
    altimeter: Option<crate::altimeter::Altimeter>,
    altimeter_publication: Option<AltimeterPublication>,
    air_pressure: Option<crate::air_pressure::AirPressure>,
    air_pressure_publication: Option<AirPressurePublication>,
    magnetometer: Option<crate::magnetometer::Magnetometer>,
    magnetometer_publication: Option<MagnetometerPublication>,
    joint_state_publisher: Option<crate::joint_state_publisher::Publisher>,
    joint_state_publication: Option<JointStatePublication>,
    publication: Option<ImuPublication>,
    visual: Option<VisualProperties>,
    material: Option<VisualMaterial>,
    visual_geometry: Option<VisualGeometry>,
    link_properties: Option<LinkProperties>,
    collision_density: Option<CollisionDensity>,
    rendering_sensors: bool,
    world_light: Option<WorldLight>,
}

enum InitialPhysics {
    Dynamic(InitialPhysicsState),
    Tpe(Arc<()>),
}

struct Blueprint {
    nodes: Vec<Node>,
    cameras: Vec<(u64, CameraSensor)>,
    physics: InitialPhysics,
    gravity: [f64; 3],
    world_physics: Option<crate::WorldPhysics>,
}

#[derive(Resource)]
pub(super) struct InitialScene(Arc<Blueprint>);
impl InitialScene {
    pub(super) fn owns_backend(&self, backend: &PhysicsBackend) -> bool {
        matches!(&self.0.physics, InitialPhysics::Dynamic(initial) if backend.owns_initial_state(initial))
    }
}

pub(super) fn matches_physics_owner(world: &World) -> bool {
    let Some(scene) = world.get_resource::<InitialScene>() else {
        return false;
    };
    match &scene.0.physics {
        InitialPhysics::Dynamic(initial) => world
            .get_resource::<PhysicsBackend>()
            .is_some_and(|backend| backend.owns_initial_state(initial)),
        InitialPhysics::Tpe(owner) => world
            .get_resource::<tpe::TpeWorld>()
            .is_some_and(|current| Arc::ptr_eq(owner, &current.0)),
    }
}

// A live handle is not sufficient: swapping two valid handles must not redirect
// an entity-state edit to another loaded body's native state.
pub(super) fn owns_loaded_body(world: &World, link_id: u64, body: BodyId) -> bool {
    world.get_resource::<InitialScene>().is_some_and(|scene| {
        scene
            .0
            .nodes
            .iter()
            .any(|node| node.id.get() == link_id && node.body == Some(body))
    })
}

pub(super) fn capture(sim: &mut Simulation) {
    let world = sim.world();
    let mut nodes = Vec::new();
    let mut cameras = Vec::new();
    for entity in world.iter_entities().filter(|e| e.contains::<EntityKind>()) {
        let id = reserve_id(world, entity.id()).expect("loader assigned every scene identity");
        if let Some(camera) = entity.get::<CameraSensor>() {
            cameras.push((id.get(), camera.clone()));
        }
        nodes.push(Node {
            id,
            parent: entity.get::<ChildOf>().map(|parent| {
                world
                    .get::<EntityId>(parent.parent())
                    .expect("loader owns every parent")
                    .get()
            }),
            name: entity
                .get::<Name>()
                .expect("loader named every node")
                .0
                .clone(),
            local_name: entity
                .get::<LocalName>()
                .expect("loader named every node")
                .0
                .clone(),
            kind: *entity.get::<EntityKind>().expect("selected scene node"),
            local: *entity
                .get::<LocalPose>()
                .expect("loader provided local poses"),
            pose: *entity
                .get::<WorldPose>()
                .expect("loader provided world poses"),
            body: entity.get::<BodyHandle>().map(BodyHandle::id),
            joint: entity.get::<JointHandle>().copied(),
            joint_definition: entity.get::<JointDefinition>().cloned(),
            joint_force: entity.get::<JointForceConfiguration>().cloned(),
            drive: entity.get::<crate::diff_drive::Drive>().cloned(),
            controller: entity.get::<crate::joint_controller::Controller>().cloned(),
            controller_configuration: entity.get::<JointControllerConfiguration>().cloned(),
            drive_configuration: entity.get::<DiffDriveConfiguration>().cloned(),
            joint_sample: entity.get::<JointSample>().copied(),
            motion: entity.get::<gz_physics_tpe::Motion>().cloned(),
            tpe_link: entity.get::<tpe::TpeLink>().cloned(),
            tpe_bounds: entity.get::<gz_physics_tpe::ModelBounds>().cloned(),
            velocity: entity.get::<Velocity>().copied(),
            center: entity.get::<LinkCenterOfMass>().copied(),
            anchor: entity.get::<ModelAnchor>().cloned(),
            attached: entity.contains::<AttachedPose>(),
            imu: entity.get::<Imu>().cloned(),
            lidar: entity.get::<crate::lidar::Lidar>().cloned(),
            lidar_publication: entity.get::<LidarPublication>().cloned(),
            altimeter: entity.get::<crate::altimeter::Altimeter>().cloned(),
            altimeter_publication: entity.get::<AltimeterPublication>().cloned(),
            air_pressure: entity.get::<crate::air_pressure::AirPressure>().cloned(),
            air_pressure_publication: entity.get::<AirPressurePublication>().cloned(),
            magnetometer: entity.get::<crate::magnetometer::Magnetometer>().cloned(),
            magnetometer_publication: entity.get::<MagnetometerPublication>().cloned(),
            joint_state_publisher: entity
                .get::<crate::joint_state_publisher::Publisher>()
                .cloned(),
            joint_state_publication: entity.get::<JointStatePublication>().cloned(),
            publication: entity.get::<ImuPublication>().cloned(),
            visual: entity.get::<VisualProperties>().cloned(),
            material: entity.get::<VisualMaterial>().cloned(),
            visual_geometry: entity.get::<VisualGeometry>().cloned(),
            link_properties: entity.get::<LinkProperties>().copied(),
            collision_density: entity.get::<CollisionDensity>().copied(),
            rendering_sensors: entity.contains::<RenderingSensorsConfiguration>(),
            world_light: entity.get::<WorldLight>().cloned(),
        });
    }
    nodes.sort_by_key(|node| node.id.get());
    let initial = Blueprint {
        nodes,
        cameras,
        physics: if let Some(backend) = world.get_resource::<PhysicsBackend>() {
            InitialPhysics::Dynamic(backend.initial_state())
        } else {
            InitialPhysics::Tpe(world.resource::<tpe::TpeWorld>().0.clone())
        },
        gravity: world.resource::<Gravity>().0,
        world_physics: world.get_resource::<crate::WorldPhysics>().cloned(),
    };
    sim.world_mut()
        .insert_resource(InitialScene(Arc::new(initial)));
}

/// Original configured controllers, including retired entities restored by a loaded reset.
/// The iterator borrows the immutable loader blueprint; it performs no IO.
pub fn initial_joint_force_configurations(
    sim: &Simulation,
) -> Result<impl Iterator<Item = (u64, &JointForceConfiguration)>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial.0.nodes.iter().filter_map(|node| {
        node.joint_force
            .as_ref()
            .map(|configuration| (node.id.get(), configuration))
    }))
}

/// Original configured drive streams, including removed loaded models.
pub fn initial_diff_drive_configurations(
    sim: &Simulation,
) -> Result<impl Iterator<Item = &DiffDriveConfiguration>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial
        .0
        .nodes
        .iter()
        .filter_map(|node| node.drive_configuration.as_ref()))
}

/// Initial configured controllers paired with their stable loaded joint IDs.
pub fn initial_joint_controller_configurations(
    sim: &Simulation,
) -> Result<impl Iterator<Item = (u64, &JointControllerConfiguration)>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial.0.nodes.iter().filter_map(|node| {
        node.controller_configuration
            .as_ref()
            .map(|configuration| (node.id.get(), configuration))
    }))
}

/// Initial stream configuration, including sensors whose runtime entities were removed.
/// The iterator borrows the immutable loader blueprint; it performs no IO.
pub fn initial_imu_publications(
    sim: &Simulation,
) -> Result<impl Iterator<Item = &ImuPublication>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial
        .0
        .nodes
        .iter()
        .filter_map(|node| node.publication.as_ref()))
}

/// Initial configured lidar streams, including removed original entities.
pub fn initial_lidar_publications(
    sim: &Simulation,
) -> Result<impl Iterator<Item = &LidarPublication>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial
        .0
        .nodes
        .iter()
        .filter_map(|node| node.lidar_publication.as_ref()))
}

/// Initial loaded altimeter metadata, including removed original entities.
pub fn initial_altimeter_publications(
    sim: &Simulation,
) -> Result<impl Iterator<Item = &AltimeterPublication>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial
        .0
        .nodes
        .iter()
        .filter_map(|node| node.altimeter_publication.as_ref()))
}

/// Initial loaded pressure metadata, including removed original entities.
pub fn initial_air_pressure_publications(
    sim: &Simulation,
) -> Result<impl Iterator<Item = &AirPressurePublication>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial
        .0
        .nodes
        .iter()
        .filter_map(|node| node.air_pressure_publication.as_ref()))
}

/// Initial loaded magnetic streams, including removed original sensors.
pub fn initial_magnetometer_publications(
    sim: &Simulation,
) -> Result<impl Iterator<Item = &MagnetometerPublication>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial
        .0
        .nodes
        .iter()
        .filter_map(|node| node.magnetometer_publication.as_ref()))
}

/// Initial joint-state configurations paired with stable loaded model IDs.
pub fn initial_joint_state_publications(
    sim: &Simulation,
) -> Result<impl Iterator<Item = (u64, &JointStatePublication)>, SceneResetError> {
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?;
    Ok(initial.0.nodes.iter().filter_map(|node| {
        node.joint_state_publication
            .as_ref()
            .map(|publication| (node.id.get(), publication))
    }))
}

/// Restores the loaded scene in the same Bevy World, then invokes reset handlers.
/// The immutable observer runs before runtime additions retire, matching native
/// reset-time removal observation. Its result is returned after cleanup.
/// Preflight failures precede mutation. A reported reset fault is terminal after
/// reconstruction: pending nodes retire and observation is skipped.
/// A panicking handler/observer invalidates the run.
/// Custom resources survive; custom system state requires an explicit Reset handler.
pub fn reset_loaded_world<R>(
    sim: &mut Simulation,
    observe: impl FnOnce(&Simulation) -> R,
) -> Result<R, SceneResetError> {
    let epoch = sim.validate_reset()?;
    let initial = sim
        .world()
        .get_resource::<InitialScene>()
        .ok_or(SceneResetError::MissingInitialState)?
        .0
        .clone();
    let initial_ids: BTreeSet<_> = initial.nodes.iter().map(|node| node.id.get()).collect();
    for node in &initial.nodes {
        node.id.validate(sim.world())?;
    }
    let mut current = Vec::new();
    let mut extra = Vec::new();
    let mut parents = Vec::new();
    let mut removed_ids = Vec::new();
    let runtime_nodes = crate::world_spawn::reset_nodes(sim.world())
        .map_err(|_| SceneResetError::RuntimePhysics)?;
    if runtime_nodes.iter().any(|entity| {
        sim.world()
            .get::<EntityId>(*entity)
            .is_none_or(|id| initial_ids.contains(&id.get()))
    }) {
        return Err(SceneResetError::RuntimePhysics);
    }
    for entity in sim.world().iter_entities() {
        let id = entity.get::<EntityId>().map(EntityId::get);
        if let Some(id) = id
            && entity_by_id(sim.world(), id) != Some(entity.id())
        {
            return Err(IdentityError::MovedIdentity.into());
        }
        if entity.contains::<IsResource>()
            || entity.contains::<SystemIdMarker>()
            || entity.contains::<Observer>()
        {
            if entity.contains::<ChildOf>() || id.is_some_and(|id| initial_ids.contains(&id)) {
                return Err(SceneResetError::InfrastructureOwnership);
            }
            continue;
        }
        current.push(entity.id());
        if id.is_none_or(|id| !initial_ids.contains(&id)) {
            if entity.contains::<gz_physics_tpe::Motion>()
                || entity.contains::<tpe::TpeLink>()
                || entity.contains::<gz_physics_tpe::ModelBounds>()
                || entity.contains::<JointHandle>()
                || entity.contains::<Imu>()
                || (!runtime_nodes.contains(&entity.id())
                    && (entity.contains::<BodyHandle>()
                        || entity.contains::<LinkCenterOfMass>()
                        || entity.contains::<ModelAnchor>()
                        || entity.contains::<AttachedPose>()))
            {
                return Err(SceneResetError::RuntimePhysics);
            }
            let parent = entity.get::<ChildOf>().map(ChildOf::parent);
            let parent_id = parent
                .and_then(|parent| sim.world().get::<EntityId>(parent))
                .map(EntityId::get);
            parents.push((entity.id(), parent, parent_id));
            extra.push(entity.id());
            removed_ids.extend(id);
        }
    }
    let prepared = match &initial.physics {
        InitialPhysics::Dynamic(initial) => {
            if sim.world().contains_resource::<tpe::TpeWorld>() {
                return Err(SceneResetError::ForeignPhysics);
            }
            Some(
                sim.world()
                    .get_resource::<PhysicsBackend>()
                    .ok_or(SceneResetError::MissingPhysics)?
                    .prepare_initial(initial)?,
            )
        }
        InitialPhysics::Tpe(owner) => {
            let current = sim
                .world()
                .get_resource::<tpe::TpeWorld>()
                .ok_or(SceneResetError::MissingPhysics)?;
            if !Arc::ptr_eq(owner, &current.0) || sim.world().contains_resource::<PhysicsBackend>()
            {
                return Err(SceneResetError::ForeignPhysics);
            }
            None
        }
    };
    let previous_time = sim.elapsed();
    let world = sim.world_mut();
    // Detach first so replacing original parents cannot recursively delete additions.
    for entity in &current {
        world.entity_mut(*entity).remove::<ChildOf>();
    }
    // Native restoration removes runtime bodies. Keep their public geometry and
    // final poses for PendingResetRemoval observation, but retire physics/sync
    // bindings before the reset schedule can query the restored backend.
    for entity in &runtime_nodes {
        world
            .entity_mut(*entity)
            .remove::<(BodyHandle, LinkCenterOfMass, ModelAnchor, AttachedPose)>();
    }
    let extras: BTreeSet<_> = extra.iter().copied().collect();
    for entity in current {
        if !extras.contains(&entity) {
            world.entity_mut(entity).despawn();
        }
    }
    let mut restored = BTreeMap::new();
    for node in &initial.nodes {
        let mut entity = world.spawn((
            Name(node.name.clone()),
            LocalName(node.local_name.clone()),
            node.kind,
            node.local,
            node.pose,
        ));
        if let Some(motion) = &node.motion {
            entity.insert(motion.clone());
        }
        if let Some(link) = &node.tpe_link {
            entity.insert(link.clone());
        }
        if let Some(bounds) = &node.tpe_bounds {
            entity.insert(bounds.clone());
        }
        if let Some(joint) = node.joint {
            entity.insert((joint, joints::JointNode));
        }
        if let Some(definition) = &node.joint_definition {
            entity.insert(definition.clone());
        }
        if let Some(configuration) = &node.joint_force {
            entity.insert((configuration.clone(), JointForceCommand::default()));
        }
        if let Some(controller) = &node.controller {
            entity.insert(controller.clone());
        }
        if let Some(configuration) = &node.controller_configuration {
            entity.insert(configuration.clone());
        }
        if let Some(drive) = &node.drive {
            entity.insert(drive.clone());
        }
        if let Some(configuration) = &node.drive_configuration {
            entity.insert(configuration.clone());
        }
        if let Some(sample) = node.joint_sample {
            entity.insert(sample);
        }
        if let Some(body) = node.body {
            entity.insert(BodyHandle(body));
        }
        if let Some(velocity) = node.velocity {
            entity.insert(velocity);
        }
        if let Some(center) = node.center {
            entity.insert(center);
        }
        if let Some(anchor) = &node.anchor {
            entity.insert(anchor.clone());
        }
        if node.attached {
            entity.insert(AttachedPose);
        }
        if let Some(lidar) = &node.lidar {
            entity.insert(lidar.clone());
        }
        if let Some(publication) = &node.lidar_publication {
            entity.insert(publication.clone());
        }
        if let Some(altimeter) = &node.altimeter {
            entity.insert(altimeter.clone());
        }
        if let Some(publication) = &node.altimeter_publication {
            entity.insert(publication.clone());
        }
        if let Some(air_pressure) = &node.air_pressure {
            entity.insert(air_pressure.clone());
        }
        if let Some(publication) = &node.air_pressure_publication {
            entity.insert(publication.clone());
        }
        if let Some(sensor) = &node.magnetometer {
            entity.insert(sensor.clone());
        }
        if let Some(publication) = &node.magnetometer_publication {
            entity.insert(publication.clone());
        }
        if let Some(publisher) = &node.joint_state_publisher {
            entity.insert(publisher.clone());
        }
        if let Some(publication) = &node.joint_state_publication {
            entity.insert(publication.clone());
        }
        if let Some(imu) = &node.imu {
            entity.insert(imu.clone());
        }
        if let Some(visual) = &node.visual {
            entity.insert(visual.clone());
        }
        if let Some(material) = &node.material {
            entity.insert(material.clone());
        }
        if let Some(geometry) = &node.visual_geometry {
            entity.insert(geometry.clone());
        }
        if let Some(properties) = node.link_properties {
            entity.insert(properties);
        }
        if let Some(density) = node.collision_density {
            entity.insert(density);
        }
        if node.rendering_sensors {
            entity.insert(RenderingSensorsConfiguration);
        }
        if let Some(light) = &node.world_light {
            entity.insert(light.clone());
        }
        if let Some(publication) = &node.publication {
            entity.insert(publication.clone());
        }
        let entity = entity.id();
        restore_id(world, entity, &node.id)
            .expect("same allocator and vacant identities were preflighted");
        restored.insert(node.id.get(), entity);
    }
    for (id, camera) in &initial.cameras {
        world.entity_mut(restored[id]).insert(camera.clone());
    }
    for node in &initial.nodes {
        if let Some(parent) = node.parent {
            world
                .entity_mut(restored[&node.id.get()])
                .insert(ChildOf(restored[&parent]));
        }
    }
    for (entity, parent, parent_id) in parents {
        let parent = parent_id
            .and_then(|id| restored.get(&id).copied())
            .or(parent);
        if let Some(parent) = parent {
            world.entity_mut(entity).insert(ChildOf(parent));
        }
        world.entity_mut(entity).insert(PendingResetRemoval);
    }
    if let Some(prepared) = prepared {
        world
            .resource_mut::<PhysicsBackend>()
            .apply_initial(prepared)
            .expect("exclusive transaction retained the preflighted backend");
    }
    world.insert_resource(Gravity(initial.gravity));
    if let Some(metadata) = &initial.world_physics {
        world.insert_resource(metadata.clone());
    } else {
        world.remove_resource::<crate::WorldPhysics>();
    }
    removed_ids.sort_unstable();
    world.insert_resource(SceneResetInfo {
        previous_time,
        epoch,
        restored_ids: initial_ids.into_iter().collect(),
        removed_ids,
    });
    world.insert_resource(RestoredPhysics { epoch });
    crate::imu_lifecycle::install(sim);
    crate::lidar::install(sim);
    crate::altimeter::install(sim);
    crate::air_pressure::install(sim);
    crate::magnetometer::install(sim);
    crate::joint_lifecycle::install(sim);
    crate::joint_force_lifecycle::install(sim);
    crate::diff_drive::install(sim);
    crate::joint_controller::install(sim);
    crate::joint_state_publisher::install(sim);
    let reset = sim.reset();
    sim.world_mut().remove_resource::<RestoredPhysics>();
    let result = reset
        .map_err(|_| SceneResetError::CommittedReset)
        .map(|_| observe(sim));
    for entity in extra {
        if sim.world().get_entity(entity).is_ok() {
            sim.world_mut().entity_mut(entity).despawn();
        }
    }
    if let Err(error) = crate::lifecycle::retire_unused_systems(sim) {
        sim.world().resource::<StepFault>().report(error);
        return Err(SceneResetError::CommittedReset);
    }
    result
}

#[cfg(test)]
#[path = "../tests/private/initial.rs"]
mod tests;
