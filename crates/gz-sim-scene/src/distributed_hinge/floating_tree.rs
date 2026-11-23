// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::floating_tree::{
    FloatingTreeJointLimit, FloatingTreeJointSpec, FloatingTreeSpec, StoppedTreeSpec,
};
use gz_physics_multifidelity::MAX_FLOATING_TREE_LINKS;
use gz_sim_core::entity_by_id;

/// Resolve 3–64 spherical links and their floating tree of unlimited revolutes. Native
/// construction order comes from ordinary Bevy joint handles, including sibling branches.
pub fn distributed_floating_tree_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<FloatingTreeSpec, LoadError> {
    match distributed_tree_recipe(xml, source, world_name)? {
        FloatingTreeRecipe::Unlimited(spec) => Ok(spec),
        _ => Err(LoadError::Unsupported {
            entity: world_name.into(),
            feature: "expected unlimited revolutes",
        }),
    }
}

/// Construction profile selected from ordinary SDF. A stopped tree may include unlimited joints.
pub enum FloatingTreeRecipe {
    Unlimited(FloatingTreeSpec),
    Stopped(StoppedTreeSpec),
    Grounded(gz_physics_distributed::floating_tree::GroundedTreeSpec),
    Payload(gz_physics_distributed::floating_tree::PayloadTreeSpec),
}

pub fn distributed_tree_recipe(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<FloatingTreeRecipe, LoadError> {
    Ok(tree_and_sim(xml, source, world_name)?.0)
}

/// Resolve a tree with at least one finite, two-sided angle stop containing the construction pose.
pub fn distributed_stopped_tree_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<StoppedTreeSpec, LoadError> {
    match distributed_tree_recipe(xml, source, world_name)? {
        FloatingTreeRecipe::Stopped(spec) => Ok(spec),
        _ => Err(LoadError::Unsupported {
            entity: world_name.into(),
            feature: "expected finite revolute stops",
        }),
    }
}

pub fn distributed_grounded_tree_spec(
    xml: &str,
    source: &str,
    world: &str,
) -> Result<gz_physics_distributed::floating_tree::GroundedTreeSpec, LoadError> {
    match distributed_tree_recipe(xml, source, world)? {
        FloatingTreeRecipe::Grounded(spec) => Ok(spec),
        _ => Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "expected an external fixed plane",
        }),
    }
}
pub(crate) fn tree_and_sim(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<(FloatingTreeRecipe, Simulation), LoadError> {
    tree_and_sim_for_step(xml, source, world_name, Duration::from_millis(1))
}
/// Validate identical rigid construction against the public step selected by
/// the owning session. Historical floating-tree profiles still select 1 ms.
pub(crate) fn tree_and_sim_for_step(
    xml: &str,
    source: &str,
    world_name: &str,
    expected_step: Duration,
) -> Result<(FloatingTreeRecipe, Simulation), LoadError> {
    construction::validate_xml(xml, world_name, true)?;
    let sdf = sdformat::load_str(xml, source, &sdformat::NoResources)?;
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.into(),
        feature,
    };
    let [world] = sdf.worlds.as_slice() else {
        return Err(invalid("expected one floating-tree world"));
    };
    let (model, other) = match world.models.as_slice() {
        [tree] => (tree, None),
        [tree, other] if tree.joints.len() >= 2 && other.joints.is_empty() => (tree, Some(other)),
        [other, tree] if tree.joints.len() >= 2 && other.joints.is_empty() => (tree, Some(other)),
        _ => {
            return Err(invalid(
                "expected one articulated tree and at most one independent body",
            ));
        }
    };
    let floor = other.filter(|m| m.is_static);
    let part = other.filter(|m| !m.is_static);
    require(
        world.name == world_name
            && (floor.is_some() || world.gravity == [0.; 3])
            && world.plugins.is_empty()
            && world.joints.is_empty(),
        world_name,
        "expected an isolated articulation world without unrelated joints or systems",
    )?;
    let sim = load_world_with_sdf_step(
        xml,
        source,
        world_name,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    require(
        sim.step_duration() == expected_step,
        world_name,
        "floating-tree step differs from the session profile",
    )?;
    let recipe = resolve_recipe(&sdf, world, model, floor, part, world_name, &sim)?;
    Ok((recipe, sim))
}

pub(super) fn resolve_recipe(
    sdf: &Sdf,
    world: &sdformat::World,
    model: &Model,
    floor: Option<&Model>,
    part: Option<&Model>,
    world_name: &str,
    sim: &Simulation,
) -> Result<FloatingTreeRecipe, LoadError> {
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.into(),
        feature,
    };
    require(
        !model.is_static
            && !model.self_collide
            && model.models.is_empty()
            && model.plugins.len() == 1
            && articulation::is_plugin(&model.plugins[0])
            && (3..=MAX_FLOATING_TREE_LINKS).contains(&model.links.len())
            && model.joints.len() + 1 == model.links.len()
            && model
                .joints
                .iter()
                .all(|j| j.kind == sdformat::JointKind::Revolute),
        &model.scoped_name,
        "expected one switchable floating tree with 3–64 dynamic spherical links and self-collision excluded",
    )?;
    let ground = floor
        .map(|f| construction::ground(sdf, world, f, world_name, sim))
        .transpose()?;
    let public = |name: &str| construction::public_id(sim, name);
    let mut links = model
        .links
        .iter()
        .map(|link| construction::spherical_link(sdf, model, &link.name, world_name, sim))
        .collect::<Result<Vec<_>, _>>()?;
    links.sort_by_key(|l| l.id);
    let (joints, limits, exact_revolute_locks) = joint_recipe(sdf, model, world_name, sim)?;
    let spec = FloatingTreeSpec {
        model_id: public(&model.scoped_name)?,
        links,
        joints,
    };
    if let Some(part) = part {
        let [link] = part.links.as_slice() else {
            return Err(invalid("expected one dynamic payload link"));
        };
        require(
            part.models.is_empty()
                && part.plugins.is_empty()
                && part.joints.is_empty()
                && !link.properties.kinematic,
            &part.scoped_name,
            "expected one independent dynamic payload",
        )?;
        let payload = construction::spherical_link(sdf, part, &link.name, world_name, sim)?;
        let spec = gz_physics_distributed::floating_tree::PayloadTreeSpec {
            tree: spec,
            limits,
            payload,
            payload_can_sleep: part.allow_auto_disable,
            exact_revolute_locks,
        };
        spec.validate()
            .map_err(|_| invalid("unrepresentable dynamic payload or articulated tree"))?;
        return Ok(FloatingTreeRecipe::Payload(spec));
    }
    require(
        !exact_revolute_locks,
        world_name,
        "distributed exact revolute locks require an independent payload",
    )?;
    if let Some(ground) = ground {
        let spec = gz_physics_distributed::floating_tree::GroundedTreeSpec {
            tree: spec,
            limits,
            ground,
        };
        spec.validate()
            .map_err(|_| invalid("unrepresentable grounded revolute tree"))?;
        return Ok(FloatingTreeRecipe::Grounded(spec));
    }
    if limits.is_empty() {
        spec.validate()
            .map_err(|_| invalid("unrepresentable native floating-tree construction"))?;
        return Ok(FloatingTreeRecipe::Unlimited(spec));
    }
    let spec = StoppedTreeSpec { tree: spec, limits };
    spec.validate()
        .map_err(|_| invalid("unrepresentable finite revolute stops"))?;
    Ok(FloatingTreeRecipe::Stopped(spec))
}

pub(super) fn joint_recipe(
    sdf: &Sdf,
    model: &Model,
    world_name: &str,
    sim: &Simulation,
) -> Result<
    (
        Vec<FloatingTreeJointSpec>,
        Vec<FloatingTreeJointLimit>,
        bool,
    ),
    LoadError,
> {
    let invalid = |feature| LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    };
    let public = |name: &str| construction::public_id(sim, name);
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .expect("constructed multifidelity tree");
    let mut exact_revolute_locks = false;
    let mut joints = Vec::with_capacity(model.joints.len());
    let mut limits = Vec::new();
    for joint in &model.joints {
        let name = format!("{}::{}", model.scoped_name, joint.name);
        let axis = joint
            .axis
            .as_ref()
            .ok_or_else(|| invalid("missing floating-tree revolute axis"))?;
        let id = public(&name)?;
        if (axis.lower, axis.upper) != (f64::NEG_INFINITY, f64::INFINITY) {
            limits.push(FloatingTreeJointLimit {
                joint_id: id,
                lower: axis.lower,
                upper: axis.upper,
            });
        }

        let native = entity_by_id(sim.world(), id)
            .and_then(|e| sim.world().get::<JointHandle>(e))
            .ok_or_else(|| invalid("missing native revolute handle"))?;
        exact_revolute_locks |= backend.is_exact_revolute_lock(native.id())?;
        joints.push((
            native.id().raw(),
            FloatingTreeJointSpec {
                joint_id: id,
                parent_id: public(&sdf.resolve_name(&model.scoped_name, &joint.parent)?)?,
                child_id: public(&sdf.resolve_name(&model.scoped_name, &joint.child)?)?,
                anchor: sdf.pose_of(&name, world_name)?.translation,
                axis: sdf.resolve_axis(axis, &name, world_name)?,
            },
        ));
    }
    joints.sort_by_key(|(native, _)| *native);
    limits.sort_by_key(|l| l.joint_id);
    Ok((
        joints.into_iter().map(|(_, j)| j).collect(),
        limits,
        exact_revolute_locks,
    ))
}

/// Explicitly select persistent velocity motors on the same link, inertia and limit recipe.
pub fn distributed_motor_tree_spec(
    xml: &str,
    source: &str,
    world: &str,
) -> Result<gz_physics_distributed::floating_tree::MotorTreeSpec, LoadError> {
    distributed_tree_recipe(xml, source, world)?.into_motor(world)
}
impl FloatingTreeRecipe {
    pub fn into_motor(
        self,
        world: &str,
    ) -> Result<gz_physics_distributed::floating_tree::MotorTreeSpec, LoadError> {
        let (tree, limits) = match self {
            Self::Unlimited(tree) => (tree, Vec::new()),
            Self::Stopped(stopped) => (stopped.tree, stopped.limits),
            Self::Grounded(_) | Self::Payload(_) => {
                return Err(LoadError::Unsupported {
                    entity: world.into(),
                    feature: "external contacts on velocity-motor worker",
                });
            }
        };
        Ok(gz_physics_distributed::floating_tree::MotorTreeSpec { tree, limits })
    }
}

/// Resolve an articulated model and a separate, movable spherical payload from ordinary SDF.
pub fn distributed_payload_tree_spec(
    xml: &str,
    source: &str,
    world: &str,
) -> Result<gz_physics_distributed::floating_tree::PayloadTreeSpec, LoadError> {
    match distributed_tree_recipe(xml, source, world)? {
        FloatingTreeRecipe::Payload(spec) => Ok(spec),
        _ => Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "expected a separate dynamic payload",
        }),
    }
}
