// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit lifecycle-boundary edits, separate from physical actuator inputs.
use super::*;
use gz_physics::BodyState;
use gz_sim_core::entity_by_id;

/// World-axis state at the model/canonical-link origin. Quaternion order is xyzw;
/// lengths and linear speeds are metres and metres/second, angular speed is rad/s.
/// Unlike the native `BodyState`, linear velocity is NOT measured at the COM.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelState {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub linear_velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
}

#[derive(Debug, thiserror::Error)]
pub enum WorldEditError {
    #[error("model identity is missing or is outside the admitted world-edit domain")]
    UnsupportedModel,
    #[error("world-edit physics or scene bindings are inconsistent")]
    InvalidScene,
    #[error("world-edit state cannot be represented by the admitted scene")]
    InvalidState,
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Identity(#[from] IdentityError),
}

struct Binding {
    model: Entity,
    link: Entity,
    parent: Entity,
    body: BodyId,
    center: LinkCenterOfMass,
    attached: Vec<(Entity, WorldPose)>,
}

// Deliberately a small complete hierarchy: one world-owned model, one coincident
// link, one collision and one visual. Sensors, nested models and joint endpoints
// are excluded even when an actuator is currently inactive. No callbacks are run.
fn binding(world: &World, id: u64) -> Result<Binding, WorldEditError> {
    let unsupported = || WorldEditError::UnsupportedModel;
    let model = entity_by_id(world, id).ok_or_else(unsupported)?;
    let parent = world
        .get::<ChildOf>(model)
        .ok_or_else(unsupported)?
        .parent();
    if world.get::<EntityKind>(model) != Some(&EntityKind::Model)
        || world.get::<EntityKind>(parent) != Some(&EntityKind::World)
    {
        return Err(unsupported());
    }
    let children = world.get::<Children>(model).ok_or_else(unsupported)?;
    if children.len() != 1 {
        return Err(unsupported());
    }
    let link = children[0];
    let local = world.get::<LocalPose>(link).ok_or_else(unsupported)?.0;
    if world.get::<EntityKind>(link) != Some(&EntityKind::Link)
        || !identity(Transform {
            translation: local.translation,
            rotation: local.rotation,
        })
        || world.get::<ChildOf>(link).map(ChildOf::parent) != Some(model)
    {
        return Err(unsupported());
    }
    if let Some(anchor) = world.get::<ModelAnchor>(model)
        && (entity_by_id(world, anchor.link) != Some(link)
            || anchor.link_to_model.translation != [0.0; 3]
            || anchor.link_to_model.rotation != [0.0, 0.0, 0.0, 1.0])
    {
        return Err(unsupported());
    }
    let descendants = world.get::<Children>(link).ok_or_else(unsupported)?;
    if descendants.len() != 2 {
        return Err(unsupported());
    }
    let mut collision = false;
    let mut visual = false;
    let mut attached = Vec::with_capacity(2);
    for entity in descendants.iter() {
        match world.get::<EntityKind>(entity) {
            Some(EntityKind::Collision) if !collision => collision = true,
            Some(EntityKind::Visual) if !visual => {
                visual = true;
                let geometry = world
                    .get::<VisualGeometry>(entity)
                    .ok_or_else(unsupported)?;
                if !matches!(
                    geometry.0.as_ref(),
                    Geometry::Box { .. } | Geometry::Sphere { .. } | Geometry::Mesh(_)
                ) {
                    return Err(unsupported());
                }
            }
            _ => return Err(unsupported()),
        }
        if world.get::<ChildOf>(entity).map(ChildOf::parent) != Some(link)
            || world
                .get::<Children>(entity)
                .is_some_and(|children| !children.is_empty())
            || !world.entity(entity).contains::<AttachedPose>()
        {
            return Err(unsupported());
        }
        let identity = world
            .get::<EntityId>(entity)
            .ok_or(WorldEditError::InvalidScene)?
            .get();
        if entity_by_id(world, identity) != Some(entity) || world.get::<WorldPose>(entity).is_none()
        {
            return Err(WorldEditError::InvalidScene);
        }
        attached.push((
            entity,
            world
                .get::<LocalPose>(entity)
                .ok_or(WorldEditError::InvalidScene)?
                .0,
        ));
    }
    if world.get::<LocalPose>(model).is_none()
        || world.get::<WorldPose>(model).is_none()
        || world.get::<WorldPose>(parent).is_none()
        || world.get::<WorldPose>(link).is_none()
        || world.get::<Velocity>(link).is_none()
        || !initial::matches_physics_owner(world)
    {
        return Err(WorldEditError::InvalidScene);
    }
    let body = world
        .get::<BodyHandle>(link)
        .ok_or(WorldEditError::InvalidScene)?
        .id();
    let link_id = world
        .get::<EntityId>(link)
        .ok_or(WorldEditError::InvalidScene)?
        .get();
    if entity_by_id(world, link_id) != Some(link)
        || !(initial::owns_loaded_body(world, link_id, body)
            || world
                .get::<super::world_spawn::RuntimeBodyOwner>(link)
                .is_some_and(|owner| owner.matches(link_id, body)))
    {
        return Err(WorldEditError::InvalidScene);
    }
    let backend = world
        .get_resource::<PhysicsBackend>()
        .ok_or(WorldEditError::InvalidScene)?;
    if backend.engine() != "rapier" || backend.is_jointed(body) != Some(false) {
        return Err(unsupported());
    }
    if backend.is_dynamic(body) == Some(true) && world.get::<ModelAnchor>(model).is_none() {
        return Err(WorldEditError::InvalidScene);
    }
    if world.iter_entities().any(|entity| {
        entity.id() != link
            && entity
                .get::<BodyHandle>()
                .is_some_and(|handle| handle.id() == body)
    }) {
        return Err(WorldEditError::InvalidScene);
    }
    Ok(Binding {
        model,
        link,
        parent,
        body,
        center: world
            .get::<LinkCenterOfMass>(link)
            .copied()
            .unwrap_or(LinkCenterOfMass([0.0; 3])),
        attached,
    })
}

pub(super) fn admitted_entities(world: &World, id: u64) -> Result<[Entity; 4], WorldEditError> {
    let target = binding(world, id)?;
    Ok([
        target.model,
        target.link,
        target.attached[0].0,
        target.attached[1].0,
    ])
}

/// Project the admitted model's visual for bounded presentation preflight.
/// This does not replace native state or validate actuator/velocity parameters.
pub fn model_visual_pose(
    sim: &Simulation,
    id: u64,
    pose: WorldPose,
) -> Result<(u64, WorldPose), WorldEditError> {
    let target = binding(sim.world(), id)?;
    let (entity, local) = target
        .attached
        .into_iter()
        .find(|(entity, _)| sim.world().get::<EntityKind>(*entity) == Some(&EntityKind::Visual))
        .expect("admitted model owns one visual");
    let projected = pose.compose(local);
    if !representable(pose) || !representable(projected) {
        return Err(WorldEditError::InvalidState);
    }
    Ok((
        sim.world()
            .get::<EntityId>(entity)
            .expect("preflighted visual identity")
            .get(),
        projected,
    ))
}

/// Reads fresh native state without stepping or refreshing collider caches.
pub fn model_state(sim: &Simulation, model_id: u64) -> Result<ModelState, WorldEditError> {
    let target = binding(sim.world(), model_id)?;
    let backend = sim.world().resource::<PhysicsBackend>();
    let state = backend
        .state(target.body)
        .ok_or(WorldEditError::InvalidScene)?;
    backend.validate_body_state(target.body, state)?;
    let result = ModelState {
        translation: state.position,
        rotation: state.orientation,
        linear_velocity: (DVec3::from_array(state.velocity)
            - target
                .center
                .velocity_shift(state.orientation, state.angular_velocity))
        .to_array(),
        angular_velocity: state.angular_velocity,
    };
    if !representable(WorldPose {
        translation: result.translation,
        rotation: result.rotation,
    }) || !result
        .linear_velocity
        .into_iter()
        .chain(result.angular_velocity)
        .all(f64::is_finite)
    {
        return Err(WorldEditError::InvalidState);
    }
    Ok(result)
}

pub(super) fn representable(pose: WorldPose) -> bool {
    pose.translation
        .into_iter()
        .chain(pose.rotation)
        .all(|value| value.is_finite() && (value as f32).is_finite())
        && (DQuat::from_array(pose.rotation).length_squared() - 1.0).abs() <= POSE_TOLERANCE
}

/// Replaces one admitted model's state outside schedule execution. Every ordinary
/// refusal precedes native/ECS mutation. Unrelated bodies, clock, IDs and initial
/// reset records are retained. This explicit world edit is not an actuator.
pub fn set_model_state(
    sim: &mut Simulation,
    model_id: u64,
    state: ModelState,
) -> Result<ModelState, WorldEditError> {
    let target = binding(sim.world(), model_id)?;
    let pose = WorldPose {
        translation: state.translation,
        rotation: state.rotation,
    };
    let local = sim
        .world()
        .get::<WorldPose>(target.parent)
        .expect("preflighted parent")
        .inverse()
        .compose(pose);
    let attached: Vec<_> = target
        .attached
        .iter()
        .map(|(entity, local)| (*entity, pose.compose(*local)))
        .collect();
    if !representable(pose)
        || !representable(local)
        || attached.iter().any(|(_, pose)| !representable(*pose))
    {
        return Err(WorldEditError::InvalidState);
    }
    let native = BodyState {
        position: state.translation,
        orientation: state.rotation,
        velocity: (DVec3::from_array(state.linear_velocity)
            + target
                .center
                .velocity_shift(state.rotation, state.angular_velocity))
        .to_array(),
        angular_velocity: state.angular_velocity,
    };
    // No observer, schedule initialization or await intervenes after validation.
    sim.world()
        .resource::<PhysicsBackend>()
        .validate_body_state(target.body, native)?;
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_body_state(target.body, native)?;
    // Loaded fixed models have no moving anchor. Once explicitly edited they
    // need one so the existing ordinary-reset schedule also restores the model
    // origin from its native construction pose. Loaded reset reconstructs the
    // original blueprint and discards this runtime-only anchor.
    if sim.world().get::<ModelAnchor>(target.model).is_none() {
        let link = sim
            .world()
            .get::<EntityId>(target.link)
            .expect("preflighted link identity")
            .get();
        sim.world_mut()
            .entity_mut(target.model)
            .insert(ModelAnchor {
                link,
                link_to_model: WorldPose {
                    translation: [0.0; 3],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                },
            });
    }
    *sim.world_mut()
        .get_mut::<WorldPose>(target.model)
        .expect("preflighted model") = pose;
    *sim.world_mut()
        .get_mut::<LocalPose>(target.model)
        .expect("preflighted local pose") = LocalPose(local);
    *sim.world_mut()
        .get_mut::<WorldPose>(target.link)
        .expect("preflighted link") = pose;
    *sim.world_mut()
        .get_mut::<Velocity>(target.link)
        .expect("preflighted velocity") = Velocity {
        linear: state.linear_velocity,
        angular: state.angular_velocity,
    };
    for (entity, pose) in attached {
        *sim.world_mut()
            .get_mut::<WorldPose>(entity)
            .expect("preflighted attached pose") = pose;
    }
    // The validated native setter has no fallible stage after mutation. Read its
    // committed value directly rather than returning an ordinary admission error
    // after commit. Exclusive world access preserves these bindings throughout.
    let actual = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(target.body)
        .expect("committed body remains owned");
    Ok(ModelState {
        translation: actual.position,
        rotation: actual.orientation,
        linear_velocity: (DVec3::from_array(actual.velocity)
            - target
                .center
                .velocity_shift(actual.orientation, actual.angular_velocity))
        .to_array(),
        angular_velocity: actual.angular_velocity,
    })
}

#[cfg(test)]
#[path = "../tests/private/world_edit.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/world_edit_boundaries.rs"]
mod boundary_tests;
