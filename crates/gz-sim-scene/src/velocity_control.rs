// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One loaded model-frame velocity target; ordinary engine stepping owns motion.
use super::*;
mod config;
pub(super) use config::{Plan, is_plugin, parse};

#[derive(Component, Clone, Debug, PartialEq)]
#[component(immutable)]
pub struct VelocityControlConfiguration {
    pub model_name: String,
    pub link: u64,
    pub topic: String,
    pub initial_linear: [f64; 3],
    pub initial_angular: [f64; 3],
}

#[derive(Component, Clone)]
pub(super) struct Controller {
    owner: u64,
    configuration: VelocityControlConfiguration,
    committed: ([f64; 3], [f64; 3]),
    pending: Option<([f64; 3], [f64; 3])>,
    center: Option<[f64; 3]>,
}

#[derive(Debug, thiserror::Error)]
pub enum VelocityControlCommandError {
    #[error("model {0} does not own an intact loaded velocity controller")]
    MissingController(u64),
    #[error("velocity controller physics binding is missing or foreign")]
    ForeignPhysics,
    #[error("velocity command is not finite and representable")]
    InvalidCommand,
    #[error(transparent)]
    Physics(#[from] PhysicsError),
}

fn world_vectors(
    rotation: [f64; 4],
    linear: [f64; 3],
    angular: [f64; 3],
) -> Option<([f64; 3], [f64; 3])> {
    let rotation = DQuat::from_array(rotation);
    if !rotation.is_finite()
        || (rotation.length_squared() - 1.).abs() > POSE_TOLERANCE
        || !linear.into_iter().chain(angular).all(f64::is_finite)
    {
        return None;
    }
    let linear = (rotation * DVec3::from_array(linear)).to_array();
    let angular = (rotation * DVec3::from_array(angular)).to_array();
    linear
        .into_iter()
        .chain(angular)
        .all(f64::is_finite)
        .then_some((linear, angular))
}

pub(super) fn build(
    sim: &mut Simulation,
    model: &Model,
    entity: Entity,
    plan: &Plan,
) -> Result<(), LoadError> {
    let link_name = &model.links[0].scoped_name;
    let link_entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| &n.0 == link_name))
        .map(|e| e.id())
        .ok_or_else(|| LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "VelocityControl link identity",
        })?;
    let link = sim
        .world()
        .get::<EntityId>(link_entity)
        .expect("loader assigned link ID")
        .get();
    let center = sim
        .world()
        .get::<LinkCenterOfMass>(link_entity)
        .map(|c| c.0);
    let owner = sim
        .world()
        .get::<EntityId>(entity)
        .expect("loader assigned model ID")
        .get();
    let configuration = VelocityControlConfiguration {
        model_name: model.scoped_name.clone(),
        link,
        topic: plan.topic.clone(),
        initial_linear: plan.linear,
        initial_angular: plan.angular,
    };
    sim.world_mut().entity_mut(entity).insert((
        configuration.clone(),
        Controller {
            owner,
            configuration,
            committed: (plan.linear, plan.angular),
            pending: None,
            center,
        },
    ));
    Ok(())
}

fn pose_matches(world: &World, model: Entity, position: [f64; 3], orientation: [f64; 4]) -> bool {
    let Some(pose) = world.get::<WorldPose>(model) else {
        return false;
    };
    position
        .into_iter()
        .zip(pose.translation)
        .all(|(a, b)| a.is_finite() && b.is_finite() && (a - b).abs() <= POSE_TOLERANCE)
        && [1., -1.].into_iter().any(|sign| {
            orientation.into_iter().zip(pose.rotation).all(|(a, b)| {
                a.is_finite() && b.is_finite() && (a - sign * b).abs() <= POSE_TOLERANCE
            })
        })
}

pub(super) fn binding_matches(world: &World, id: u64) -> bool {
    if !initial::matches_physics_owner(world)
        || world.contains_resource::<PhysicsBackend>() == world.contains_resource::<tpe::TpeWorld>()
    {
        return false;
    }
    let Some(entity) = gz_sim_core::entity_by_id(world, id) else {
        return false;
    };
    let Some(controller) = world.get::<Controller>(entity) else {
        return false;
    };
    let c = &controller.configuration;
    let initial = world.resource::<initial::InitialScene>();
    let expected_parent = initial
        .velocity_control_identities()
        .find(|row| row.id == id)
        .and_then(|row| row.parent);
    let actual_parent = world
        .get::<ChildOf>(entity)
        .and_then(|p| world.get::<EntityId>(p.parent()))
        .map(EntityId::get);
    if expected_parent.is_none() || expected_parent != actual_parent {
        return false;
    }
    let Some(blueprint) = world
        .get_resource::<initial::InitialScene>()
        .and_then(|i| i.velocity_control_blueprint(id))
    else {
        return false;
    };
    let Some(link) = gz_sim_core::entity_by_id(world, c.link) else {
        return false;
    };
    if controller.owner != id
        || c != &blueprint.configuration
        || controller.center != blueprint.center
        || world.get::<LinkCenterOfMass>(link).map(|c| c.0) != controller.center
        || world.get::<VelocityControlConfiguration>(entity) != Some(c)
        || world.get::<EntityKind>(entity) != Some(&EntityKind::Model)
        || world
            .get::<Name>(entity)
            .is_none_or(|n| n.0 != c.model_name)
        || world
            .get::<ChildOf>(entity)
            .and_then(|p| world.get::<EntityKind>(p.parent()))
            != Some(&EntityKind::World)
        || world.get::<EntityKind>(link) != Some(&EntityKind::Link)
        || world.get::<ChildOf>(link).map(ChildOf::parent) != Some(entity)
        || world.get::<LocalPose>(link).is_none_or(|p| {
            !identity(Transform {
                translation: p.0.translation,
                rotation: p.0.rotation,
            })
        })
        || world
            .get::<Children>(entity)
            .into_iter()
            .flat_map(|c| c.iter())
            .filter(|e| world.get::<EntityKind>(*e) == Some(&EntityKind::Link))
            .count()
            != 1
        || world.get::<LinkCenterOfMass>(link).is_some_and(|center| {
            center
                .0
                .into_iter()
                .any(|x| !x.is_finite() || x.abs() > POSE_TOLERANCE)
        })
        || world.get::<PendingResetRemoval>(entity).is_some()
    {
        return false;
    }
    if let Some(backend) = world.get_resource::<PhysicsBackend>() {
        let Some(body) = world.get::<BodyHandle>(link) else {
            return false;
        };
        initial::owns_loaded_body(world, c.link, body.id())
            && backend.is_dynamic(body.id()) == Some(true)
            && backend.is_jointed(body.id()) == Some(false)
            && backend
                .state(body.id())
                .is_some_and(|state| pose_matches(world, entity, state.position, state.orientation))
    } else {
        world
            .get::<gz_physics_tpe::Motion>(entity)
            .is_some_and(|m| pose_matches(world, entity, m.state().position, m.state().orientation))
            && world
                .get::<gz_physics_tpe::ModelBounds>(entity)
                .is_some_and(|b| !b.is_static())
            && world.get::<tpe::TpeLink>(link).is_some()
    }
}

/// Checks complete live loaded ownership without mutating or creating IO.
pub fn velocity_control_binding_matches(sim: &Simulation, id: u64) -> bool {
    binding_matches(sim.world(), id)
}

/// Checks the exact immutable configuration in addition to loaded body identity.
pub fn velocity_control_binding_matches_configuration(
    sim: &Simulation,
    id: u64,
    c: &VelocityControlConfiguration,
) -> bool {
    velocity_control_binding_matches(sim, id)
        && gz_sim_core::entity_by_id(sim.world(), id)
            .and_then(|e| sim.world().get::<VelocityControlConfiguration>(e))
            == Some(c)
}

/// Store the latest model-frame target. Paused calls retain pending input. Only
/// an advancing PostStep commits it; the next advancing PreStep applies it.
/// Invalid input or binding drift preserves the preceding pending/committed target.
pub fn set_velocity_control_command(
    sim: &mut Simulation,
    id: u64,
    linear: [f64; 3],
    angular: [f64; 3],
) -> Result<(), VelocityControlCommandError> {
    if !linear.into_iter().chain(angular).all(f64::is_finite) {
        return Err(VelocityControlCommandError::InvalidCommand);
    }
    if !initial::matches_physics_owner(sim.world()) {
        return Err(VelocityControlCommandError::ForeignPhysics);
    }
    if !binding_matches(sim.world(), id) {
        return Err(VelocityControlCommandError::MissingController(id));
    }
    let entity = gz_sim_core::entity_by_id(sim.world(), id).expect("validated controller ID");
    let rotation = sim
        .world()
        .get::<WorldPose>(entity)
        .ok_or(VelocityControlCommandError::MissingController(id))?
        .rotation;
    if world_vectors(rotation, linear, angular).is_none() {
        return Err(VelocityControlCommandError::InvalidCommand);
    }
    sim.world_mut()
        .get_mut::<Controller>(entity)
        .expect("validated controller")
        .pending = Some((linear, angular));
    Ok(())
}

// Original identity reservations keep ownership even if public metadata is
// removed or changed. Removal itself retires the entity, not its saved blueprint.
pub(super) fn owns_model(world: &World, id: u64) -> bool {
    world
        .get_resource::<initial::InitialScene>()
        .is_some_and(|i| i.velocity_control_blueprint(id).is_some())
        || gz_sim_core::entity_by_id(world, id)
            .is_some_and(|e| world.get::<Controller>(e).is_some())
}
pub(super) fn owns_link(world: &World, id: u64) -> bool {
    world
        .iter_entities()
        .filter_map(|e| e.get::<Controller>())
        .any(|c| c.configuration.link == id)
        || world
            .get_resource::<initial::InitialScene>()
            .is_some_and(|i| {
                i.velocity_control_identities()
                    .any(|c| c.configuration.link == id)
            })
}

#[derive(Resource)]
pub(super) struct Installed;
pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<Controller>())
    {
        return;
    }
    sim.add_systems(Phase::PreStep, apply);
    sim.add_systems(Phase::PostStep, latch);
    sim.add_systems(Phase::Reset, reset.after(sync_attached_poses));
    sim.world_mut().insert_resource(Installed);
}
pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>()
        || sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<Controller>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PreStep, apply)?;
    sim.remove_systems_in_set(Phase::PostStep, latch)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}

fn active_controllers(world: &World) -> Result<Vec<(Entity, Controller)>, InputError> {
    let initial = world
        .get_resource::<initial::InitialScene>()
        .ok_or(InputError::InvalidPhysics)?;
    let expected: std::collections::BTreeSet<_> = initial
        .velocity_control_identities()
        .filter(|row| gz_sim_core::entity_by_id(world, row.id).is_some())
        .map(|row| row.id)
        .collect();
    let rows: Vec<_> = world
        .iter_entities()
        .filter_map(|e| e.get::<Controller>().map(|c| (e.id(), c.clone())))
        .collect();
    if rows.len() != expected.len()
        || rows.iter().any(|(entity, c)| {
            !expected.contains(&c.owner)
                || gz_sim_core::entity_by_id(world, c.owner) != Some(*entity)
                || !binding_matches(world, c.owner)
        })
    {
        return Err(InputError::InvalidPhysics);
    }
    Ok(rows)
}

fn apply(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero()
        || world.resource::<StepFault>().error().is_some()
    {
        return;
    }
    let rows = match active_controllers(world) {
        Ok(rows) => rows,
        Err(error) => {
            world.resource::<StepFault>().report(error);
            return;
        }
    };
    for (entity, controller) in rows {
        let result = (|| -> Result<(), InputError> {
            if !binding_matches(world, controller.owner) {
                return Err(InputError::InvalidPhysics);
            }
            let pose = world
                .get::<WorldPose>(entity)
                .ok_or(InputError::InvalidPhysics)?;
            let (linear, angular) = world_vectors(
                pose.rotation,
                controller.committed.0,
                controller.committed.1,
            )
            .ok_or(InputError::InvalidValue)?;
            let link = gz_sim_core::entity_by_id(world, controller.configuration.link)
                .ok_or(InputError::InvalidPhysics)?;
            inputs::apply_velocity(world, link, linear, angular)
        })();
        if let Err(error) = result {
            world.resource::<StepFault>().report(error);
            return;
        }
    }
}
fn latch(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero()
        || world.resource::<StepFault>().error().is_some()
    {
        return;
    }
    let rows = match active_controllers(world) {
        Ok(rows) => rows,
        Err(error) => {
            world.resource::<StepFault>().report(error);
            return;
        }
    };
    for (_, saved) in rows {
        let id = saved.owner;
        if !binding_matches(world, id) {
            world
                .resource::<StepFault>()
                .report(InputError::InvalidPhysics);
            return;
        }
        let entity = gz_sim_core::entity_by_id(world, id).expect("validated controller ID");
        let mut controller = world
            .get_mut::<Controller>(entity)
            .expect("validated controller");
        if let Some(target) = controller.pending.take() {
            controller.committed = target;
        }
    }
}
fn reset(world: &mut World) {
    let rows = match active_controllers(world) {
        Ok(rows) => rows,
        Err(error) => {
            world.resource::<StepFault>().report(error);
            return;
        }
    };
    for (_, saved) in rows {
        let id = saved.owner;
        if !binding_matches(world, id) {
            world
                .resource::<StepFault>()
                .report(InputError::InvalidPhysics);
            return;
        }
        let entity = gz_sim_core::entity_by_id(world, id).expect("validated controller ID");
        let mut controller = world
            .get_mut::<Controller>(entity)
            .expect("validated controller");
        controller.committed = (
            controller.configuration.initial_linear,
            controller.configuration.initial_angular,
        );
        controller.pending = None;
    }
}

#[cfg(test)]
#[path = "../tests/private/velocity_control.rs"]
mod tests;
