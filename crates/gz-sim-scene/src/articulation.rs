// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public tick-boundary commands for loaded, switchable articulated models.
use super::*;
pub use gz_physics_multifidelity::ArticulationTier;
use gz_sim_core::entity_by_id;
use sdformat::Plugin;

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::SwitchableArticulation"
        && plugin.filename == "gz-sim-switchable-articulation-system"
}

/// The split/join profile refreshes speculative contact geometry each step.
pub(super) fn moving_contact_profile(world: &sdformat::World) -> bool {
    let [hinge, balls @ ..] = world.models.as_slice() else {
        return false;
    };
    (1..=2).contains(&balls.len())
        && world.gravity == [0.; 3]
        && hinge.joints.len() == 1
        && hinge.links.len() == 2
        && hinge.plugins.iter().any(is_plugin)
        && balls.iter().all(|ball| {
            ball.links.len() == 1
                && ball.joints.is_empty()
                && ball.plugins.iter().all(prescribed_motion::is_initial_velocity)
        })
        && world.models.iter().all(|model| {
            !model.is_static
                && !model.allow_auto_disable
                && model.models.is_empty()
                && model.links.iter().all(|link| {
                    !link.properties.kinematic
                        && matches!(link.collisions.as_slice(), [collision] if matches!(collision.geometry, Geometry::Sphere { .. }))
                })
        })
}

#[derive(Clone, Copy)]
pub(super) struct ArticulationOptions {
    pub exact_revolute_locks: bool,
    pub additional_pgs_iterations: Option<u8>,
}

pub(super) fn validate_marker(
    model: &Model,
    engine: Option<&str>,
) -> Result<Option<ArticulationOptions>, LoadError> {
    let mut markers = model.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(marker) = markers.next() else {
        return Ok(None);
    };
    require(
        engine == Some("multifidelity"),
        &model.scoped_name,
        "SwitchableArticulation requires the multifidelity engine",
    )?;
    require(
        !model.joints.is_empty(),
        &model.scoped_name,
        "SwitchableArticulation requires a revolute tree",
    )?;
    let invalid = || LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature: "SwitchableArticulation configuration",
    };
    let doc = roxmltree::Document::parse(&marker.xml).map_err(|_| invalid())?;
    let root = doc.root_element();
    if markers.next().is_some()
        || root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root.attributes().len() != 2
        || root.attribute("name") != Some(&marker.name)
        || root.attribute("filename") != Some(&marker.filename)
    {
        return Err(invalid());
    }
    let mut exact = None;
    let mut additional_pgs_iterations = None;
    for child in root.children() {
        if child.is_comment() || child.is_text() && child.text().unwrap_or("").trim().is_empty() {
            continue;
        }
        if !child.is_element()
            || child.namespaces().next().is_some()
            || child.attributes().len() != 0
            || child.children().any(|n| !n.is_text())
        {
            return Err(invalid());
        }
        let value = child.text().unwrap_or("").trim();
        match child.tag_name().name() {
            "exact_revolute_locks" if exact.is_none() => {
                exact = Some(match value {
                    "true" | "1" => true,
                    "false" | "0" => false,
                    _ => return Err(invalid()),
                });
            }
            "additional_pgs_iterations" if additional_pgs_iterations.is_none() => {
                let count = value.parse::<u8>().map_err(|_| invalid())?;
                if count > 128 {
                    return Err(invalid());
                }
                additional_pgs_iterations = Some(count);
            }
            _ => return Err(invalid()),
        }
    }
    Ok(Some(ArticulationOptions {
        exact_revolute_locks: exact.unwrap_or(false),
        additional_pgs_iterations,
    }))
}

#[derive(Debug, thiserror::Error)]
pub enum FidelityError {
    #[error("public entity {0} does not exist")]
    MissingModel(u64),
    #[error("public entity {0} is not a loaded obstacle link")]
    MissingObstacle(u64),
    #[error("public entity {0} is not a switchable articulation")]
    NotSwitchable(u64),
    #[error("loaded articulation bindings do not match their owner")]
    InvalidBinding,
    #[error("the articulation has an active joint force command")]
    PendingCommand,
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Clock(#[from] StepError),
}

#[derive(Component, Clone)]
#[component(immutable)]
pub(super) struct SwitchableArticulation {
    model: u64,
    root: u64,
    pub(super) root_body: BodyId,
    links: Arc<[(u64, BodyId)]>,
    joints: Arc<[(u64, gz_physics::JointId)]>,
}

pub(super) fn bind(sim: &mut Simulation, model: Entity, root: u64) {
    let world = sim.world();
    let children = world
        .get::<Children>(model)
        .expect("constructed revolute tree");
    let mut links = Vec::new();
    let mut joints = Vec::new();
    for entity in children.iter() {
        let id = world
            .get::<EntityId>(entity)
            .expect("loaded child identity")
            .get();
        if let Some(body) = world.get::<BodyHandle>(entity) {
            links.push((id, body.id()));
        }
        if let Some(joint) = world.get::<JointHandle>(entity) {
            joints.push((id, joint.id()));
        }
    }
    let root_link = entity_by_id(world, root).expect("loaded tree root");
    let marker = SwitchableArticulation {
        model: world
            .get::<EntityId>(model)
            .expect("loaded model identity")
            .get(),
        root,
        root_body: world
            .get::<BodyHandle>(root_link)
            .expect("loaded root body")
            .id(),
        links: links.into(),
        joints: joints.into(),
    };
    crate::support_loss_promotion::bind(sim, marker.model, marker.root_body);
    sim.world_mut().entity_mut(model).insert(marker);
}

pub(super) struct Binding<'a> {
    model: Entity,
    assembly: &'a SwitchableArticulation,
    solver: &'a MultiFidelityBackend,
}

pub(super) fn binding(world: &World, id: u64) -> Result<Binding<'_>, FidelityError> {
    let model = entity_by_id(world, id).ok_or(FidelityError::MissingModel(id))?;
    let marker = world
        .get::<SwitchableArticulation>(model)
        .ok_or(FidelityError::NotSwitchable(id))?;
    let parent = world.get::<ChildOf>(model).map(ChildOf::parent);
    let root = entity_by_id(world, marker.root).ok_or(FidelityError::InvalidBinding)?;
    if id != marker.model
        || world.get::<EntityKind>(model) != Some(&EntityKind::Model)
        || parent.and_then(|entity| world.get::<EntityKind>(entity)) != Some(&EntityKind::World)
        || world.get::<ChildOf>(root).map(ChildOf::parent) != Some(model)
        || world.get::<BodyHandle>(root).map(BodyHandle::id) != Some(marker.root_body)
        || world.contains_resource::<tpe::TpeWorld>()
        || !initial::matches_physics_owner(world)
    {
        return Err(FidelityError::InvalidBinding);
    }
    let solver = world
        .get_resource::<PhysicsBackend>()
        .and_then(PhysicsBackend::downcast_ref::<MultiFidelityBackend>)
        .ok_or(FidelityError::InvalidBinding)?;
    if solver.articulation_link_count(marker.root_body).ok() != Some(marker.links.len()) {
        return Err(FidelityError::InvalidBinding);
    }
    Ok(Binding {
        model,
        assembly: marker,
        solver,
    })
}

pub(super) fn wrench_body(
    world: &World,
    entity: Entity,
    id: u64,
) -> Result<Option<BodyId>, FidelityError> {
    let Some(marker) = world
        .get::<ChildOf>(entity)
        .and_then(|parent| world.get::<SwitchableArticulation>(parent.parent()))
    else {
        return Ok(None);
    };
    if world.get::<EntityKind>(entity) != Some(&EntityKind::Link) {
        return Ok(None);
    }
    let bound = binding(world, marker.model)?;
    let body = world
        .get::<BodyHandle>(entity)
        .ok_or(FidelityError::InvalidBinding)?
        .id();
    if world.get::<ChildOf>(entity).map(ChildOf::parent) != Some(bound.model)
        || !bound.assembly.links.contains(&(id, body))
        || !bound
            .solver
            .articulation_contains(bound.assembly.root_body, body)
    {
        return Err(FidelityError::InvalidBinding);
    }
    Ok(Some(body))
}

pub(super) fn tier(world: &World, id: u64) -> Result<ArticulationTier, FidelityError> {
    binding(world, id)?.tier()
}

impl Binding<'_> {
    pub(super) fn tier(&self) -> Result<ArticulationTier, FidelityError> {
        Ok(self.solver.articulation_tier(self.assembly.root_body)?)
    }

    /// Validate every selected obstacle even when an earlier one predicts contact.
    pub(super) fn first_contact(
        &self,
        world: &World,
        obstacles: &[u64],
        dt: Duration,
    ) -> Result<Option<u64>, FidelityError> {
        Self::first_contact_with(world, obstacles, |body| {
            self.solver
                .predicts_articulation_contact(self.assembly.root_body, body, dt)
        })
    }

    fn first_contact_with(
        world: &World,
        obstacles: &[u64],
        intersects: impl Fn(BodyId) -> Result<bool, PhysicsError>,
    ) -> Result<Option<u64>, FidelityError> {
        let mut first = None;
        for &obstacle in obstacles {
            let entity =
                entity_by_id(world, obstacle).ok_or(FidelityError::MissingObstacle(obstacle))?;
            let body = world
                .get::<BodyHandle>(entity)
                .filter(|_| world.get::<EntityKind>(entity) == Some(&EntityKind::Link))
                .ok_or(FidelityError::MissingObstacle(obstacle))?;
            if intersects(body.id())? {
                first.get_or_insert(obstacle);
            }
        }
        Ok(first)
    }
}

pub(super) fn root_pose(world: &World, id: u64) -> Result<WorldPose, FidelityError> {
    let bound = binding(world, id)?;
    let root = entity_by_id(world, bound.assembly.root).ok_or(FidelityError::InvalidBinding)?;
    world
        .get::<WorldPose>(root)
        .copied()
        .ok_or(FidelityError::InvalidBinding)
}

pub(super) fn switch(
    world: &mut World,
    id: u64,
    target: ArticulationTier,
    dt: Duration,
) -> Result<(), FidelityError> {
    switch_checked(world, id, target, dt, false)
}

fn switch_checked(
    world: &mut World,
    id: u64,
    target: ArticulationTier,
    dt: Duration,
    free_coast: bool,
) -> Result<(), FidelityError> {
    let bound = binding(world, id)?;
    let root = bound.assembly.root_body;
    if !free_coast && bound.tier()? == target {
        return Ok(());
    }
    validate_change(world, id, &bound, target)?;
    let mut backend = world.resource_mut::<PhysicsBackend>();
    let solver = backend
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or(FidelityError::InvalidBinding)?;
    if free_coast {
        solver.begin_articulation_free_coast(root, dt)?;
    } else {
        solver.switch_articulation(root, target, dt)?;
    }
    Ok(())
}

/// Validate the normal ECS identities and pending-input gates before the native
/// stationary-cohort proof. Obstacle policies retain their ordered predictive path.
pub(super) fn parked_zone_root(world: &World, id: u64) -> Result<Option<BodyId>, FidelityError> {
    let bound = binding(world, id)?;
    if bound.tier()? != ArticulationTier::Dynamic || crate::contact_promotion::enabled(world, id) {
        return Ok(None);
    }
    validate_change(world, id, &bound, ArticulationTier::Prescribed)?;
    Ok(Some(bound.assembly.root_body))
}
pub(super) fn validate_demotion(world: &World, id: u64, dt: Duration) -> Result<(), FidelityError> {
    let bound = binding(world, id)?;
    validate_change(world, id, &bound, ArticulationTier::Prescribed)?;
    let root = bound.assembly.root_body;
    if let Some(obstacles) = crate::contact_promotion::targets(world, id) {
        let query = bound.solver.demotion_contact_prediction(root, dt)?;
        if Binding::first_contact_with(world, obstacles, query)?.is_some() {
            return Err(PhysicsError::TransitionDeferred {
                reason: "contact guard predicts an impact on the upcoming solve",
            }
            .into());
        }
        return Ok(());
    }
    Ok(bound.solver.validate_articulation_demotion(root, dt)?)
}

fn validate_change(
    world: &World,
    id: u64,
    bound: &Binding<'_>,
    target: ArticulationTier,
) -> Result<(), FidelityError> {
    let model = bound.model;
    let solver = bound.solver;
    let assembly = bound.assembly;
    let root = assembly.root_body;
    if target == ArticulationTier::Prescribed && crate::contact_promotion::enabled(world, id) {
        solver.validate_prediction_motion(root)?;
    }
    for &(id, expected) in assembly.links.iter() {
        let entity = entity_by_id(world, id).ok_or(FidelityError::InvalidBinding)?;
        if world.get::<ChildOf>(entity).map(ChildOf::parent) != Some(model)
            || world.get::<BodyHandle>(entity).map(BodyHandle::id) != Some(expected)
            || !solver.articulation_contains(root, expected)
        {
            return Err(FidelityError::InvalidBinding);
        }
    }
    for &(id, expected) in assembly.joints.iter() {
        let entity = entity_by_id(world, id).ok_or(FidelityError::InvalidBinding)?;
        if world.get::<ChildOf>(entity).map(ChildOf::parent) != Some(model)
            || joints::input_binding(world, entity) != Some(expected)
        {
            return Err(FidelityError::InvalidBinding);
        }
        if target == ArticulationTier::Prescribed
            && world
                .get::<JointForceCommand>(entity)
                .is_some_and(|command| command.torque() != 0.0)
        {
            return Err(FidelityError::PendingCommand);
        }
    }
    Ok(())
}

/// The current solver representation of the complete model, addressed by its public model ID.
pub fn model_fidelity(sim: &Simulation, id: u64) -> Result<ArticulationTier, FidelityError> {
    tier(sim.world(), id)
}

/// Committed world pose of the physical root link, which may differ from the model's canonical link.
pub fn articulation_root_pose(sim: &Simulation, id: u64) -> Result<WorldPose, FidelityError> {
    root_pose(sim.world(), id)
}

/// Changes the model's representation at the current tick boundary without advancing the clock.
/// A world-mounted mixed robot can park only at an unloaded stationary equilibrium.
/// Its native constraints and finite collision masses remain available on the first resumed solve.
/// A refused transition preserves the current representation and pending inputs.
pub fn set_model_fidelity(
    sim: &mut Simulation,
    id: u64,
    target: ArticulationTier,
) -> Result<(), FidelityError> {
    sim.validate_steps(0)?;
    let dt = sim.step_duration();
    switch(sim.world_mut(), id, target, dt)
}

/// At this tick boundary, begin torque-free rigid transport of a passive,
/// isolated assembly in zero gravity. A later body wrench or added rigid body
/// restores dynamics before the next advancing solve. Explicitly switch back
/// to Dynamic before setting a joint actuator. Paused ticks leave the tier and
/// queued wrench unchanged.
pub fn begin_model_free_coast(sim: &mut Simulation, id: u64) -> Result<(), FidelityError> {
    sim.validate_steps(0)?;
    if crate::primary_contact::installed(sim.world()) {
        return Err(PhysicsError::TransitionDeferred {reason: "free coast requires an independent native clock and cannot combine with primary contact refinement"}.into());
    }
    if crate::deformable::bodies(sim).next().is_some() {
        return Err(PhysicsError::TransitionDeferred {
            reason: "free coast requires a scene without deformable coupling",
        }
        .into());
    }
    let dt = sim.step_duration();
    switch_checked(sim.world_mut(), id, ArticulationTier::Prescribed, dt, true)
}
