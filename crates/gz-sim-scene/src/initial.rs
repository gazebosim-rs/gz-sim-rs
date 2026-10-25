// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::resource::IsResource;
use bevy_ecs::{observer::Observer, system::SystemIdMarker};
use gz_physics_rapier::{InitialPhysicsState, RestoredPhysics};
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
    joint_sample: Option<JointSample>,
    motion: Option<gz_physics_tpe::Motion>,
    tpe_link: Option<tpe::TpeLink>,
    tpe_bounds: Option<gz_physics_tpe::ModelBounds>,
    velocity: Option<Velocity>,
    anchor: Option<ModelAnchor>,
    attached: bool,
    imu: Option<Imu>,
    publication: Option<ImuPublication>,
    visual: Option<VisualProperties>,
    material: Option<VisualMaterial>,
    link_properties: Option<LinkProperties>,
}

enum InitialPhysics {
    Rapier(Box<InitialPhysicsState>),
    Tpe(Arc<()>),
}

struct Blueprint {
    nodes: Vec<Node>,
    physics: InitialPhysics,
    gravity: [f64; 3],
    world_physics: Option<crate::WorldPhysics>,
}

#[derive(Resource)]
pub(super) struct InitialScene(Arc<Blueprint>);
impl InitialScene {
    pub(super) fn owns_rapier(&self, backend: &RapierBackend) -> bool {
        matches!(&self.0.physics, InitialPhysics::Rapier(initial) if backend.owns_initial_state(initial))
    }
}

pub(super) fn matches_physics_owner(world: &World) -> bool {
    let Some(scene) = world.get_resource::<InitialScene>() else {
        return false;
    };
    match &scene.0.physics {
        InitialPhysics::Rapier(initial) => world
            .get_resource::<RapierBackend>()
            .is_some_and(|backend| backend.owns_initial_state(initial)),
        InitialPhysics::Tpe(owner) => world
            .get_resource::<tpe::TpeWorld>()
            .is_some_and(|current| Arc::ptr_eq(owner, &current.0)),
    }
}

pub(super) fn capture(sim: &mut Simulation) {
    let world = sim.world();
    let mut nodes = Vec::new();
    for entity in world.iter_entities().filter(|e| e.contains::<EntityKind>()) {
        nodes.push(Node {
            id: reserve_id(world, entity.id()).expect("loader assigned every scene identity"),
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
            joint_sample: entity.get::<JointSample>().copied(),
            motion: entity.get::<gz_physics_tpe::Motion>().cloned(),
            tpe_link: entity.get::<tpe::TpeLink>().cloned(),
            tpe_bounds: entity.get::<gz_physics_tpe::ModelBounds>().cloned(),
            velocity: entity.get::<Velocity>().copied(),
            anchor: entity.get::<ModelAnchor>().cloned(),
            attached: entity.contains::<AttachedPose>(),
            imu: entity.get::<Imu>().cloned(),
            publication: entity.get::<ImuPublication>().cloned(),
            visual: entity.get::<VisualProperties>().cloned(),
            material: entity.get::<VisualMaterial>().cloned(),
            link_properties: entity.get::<LinkProperties>().copied(),
        });
    }
    nodes.sort_by_key(|node| node.id.get());
    let initial = Blueprint {
        nodes,
        physics: if let Some(backend) = world.get_resource::<RapierBackend>() {
            InitialPhysics::Rapier(Box::new(backend.initial_state()))
        } else {
            InitialPhysics::Tpe(world.resource::<tpe::TpeWorld>().0.clone())
        },
        gravity: world.resource::<Gravity>().0,
        world_physics: world.get_resource::<crate::WorldPhysics>().cloned(),
    };
    sim.world_mut()
        .insert_resource(InitialScene(Arc::new(initial)));
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
                || entity.contains::<BodyHandle>()
                || entity.contains::<ModelAnchor>()
                || entity.contains::<Imu>()
                || entity.contains::<AttachedPose>()
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
        InitialPhysics::Rapier(initial) => {
            if sim.world().contains_resource::<tpe::TpeWorld>() {
                return Err(SceneResetError::ForeignPhysics);
            }
            Some(
                sim.world()
                    .get_resource::<RapierBackend>()
                    .ok_or(SceneResetError::MissingPhysics)?
                    .prepare_initial(initial)?,
            )
        }
        InitialPhysics::Tpe(owner) => {
            let current = sim
                .world()
                .get_resource::<tpe::TpeWorld>()
                .ok_or(SceneResetError::MissingPhysics)?;
            if !Arc::ptr_eq(owner, &current.0) || sim.world().contains_resource::<RapierBackend>() {
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
        if let Some(sample) = node.joint_sample {
            entity.insert(sample);
        }
        if let Some(body) = node.body {
            entity.insert(BodyHandle(body));
        }
        if let Some(velocity) = node.velocity {
            entity.insert(velocity);
        }
        if let Some(anchor) = &node.anchor {
            entity.insert(anchor.clone());
        }
        if node.attached {
            entity.insert(AttachedPose);
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
        if let Some(properties) = node.link_properties {
            entity.insert(properties);
        }
        if let Some(publication) = &node.publication {
            entity.insert(publication.clone());
        }
        let entity = entity.id();
        restore_id(world, entity, &node.id)
            .expect("same allocator and vacant identities were preflighted");
        restored.insert(node.id.get(), entity);
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
            .resource_mut::<RapierBackend>()
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
    result
}

#[cfg(test)]
#[path = "../tests/private/initial.rs"]
mod tests;
