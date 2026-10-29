// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, JointId, JointState, PhysicsBackend};
use gz_sim_core::entity_by_id;
use petgraph::{algo::toposort, graphmap::DiGraphMap};
use std::collections::{BTreeMap, BTreeSet};

/// Owned SDF construction metadata; editing it does not reconfigure the solver.
#[derive(Component, Clone, Debug)]
pub struct JointDefinition(pub Box<sdformat::Joint>);

#[derive(Component)]
pub(super) struct JointNode;

/// A solver constraint and its public child link identity, owned by the loaded scene.
#[derive(Component, Clone, Copy)]
pub struct JointHandle {
    id: JointId,
    child: u64,
    child_pose: Option<WorldPose>,
    parent: Option<(u64, BodyId)>,
    model: u64,
}
impl JointHandle {
    pub fn id(&self) -> JointId {
        self.id
    }
    pub fn child(&self) -> u64 {
        self.child
    }
}

/// Continuous joint position and axis velocity at a committed simulation instant.
#[derive(Component, Clone, Copy, Debug)]
pub struct JointSample {
    pub state: JointState,
    pub time: Duration,
    pub epoch: u64,
}

pub(super) fn validate_model(
    sdf: &Sdf,
    model: &Model,
    world: &str,
    top_level: bool,
    engine: Option<&str>,
    is_static: bool,
) -> Result<(), LoadError> {
    for joint in &model.joints {
        require(
            joint.sensors.is_empty(),
            &format!("{}::{}", model.scoped_name, joint.name),
            "joint sensors",
        )?;
    }
    if model.joints.is_empty() {
        return Ok(());
    }
    require(engine.is_some(), &model.scoped_name, "TPE joints")?;
    let rapier = engine == Some("rapier");
    require(
        top_level
            && !is_static
            && model.models.is_empty()
            && (rapier
                || ((model.links.len() == 1 || model.links.len() == 2) && model.joints.len() == 1)),
        &model.scoped_name,
        "joint topology (expected one dynamic world-anchored link)",
    )?;
    let links: BTreeMap<_, _> = model
        .links
        .iter()
        .enumerate()
        .map(|(index, link)| (link.scoped_name.as_str(), index))
        .collect();
    let mut graph = DiGraphMap::<usize, ()>::new();
    let mut children = BTreeSet::new();
    for joint in &model.joints {
        let name = format!("{}::{}", model.scoped_name, joint.name);
        require(
            joint.kind == sdformat::JointKind::Revolute && joint.axis2.is_none(),
            &name,
            "joints (unsupported type)",
        )?;
        let parent = sdf.resolve_name(&model.scoped_name, &joint.parent)?;
        let child = sdf.resolve_name(&model.scoped_name, &joint.child)?;
        let parent_index = if parent == world {
            Some(model.links.len())
        } else {
            links.get(parent.as_str()).copied()
        };
        let child_index = links.get(child.as_str()).copied();
        require(
            parent_index.is_some()
                && child_index.is_some()
                && parent != child
                && (rapier
                    || ((model.links.len() == 1 && parent == world)
                        || (model.links.len() == 2 && parent != world))),
            &name,
            "joint endpoints (expected world and sole link)",
        )?;
        let child_index = child_index.expect("validated local child");
        require(
            children.insert(child_index),
            &name,
            "multiple parent joints",
        )?;
        graph.add_edge(parent_index.expect("validated parent"), child_index, ());
        validate_axis(joint, &name)?;
    }
    require(
        toposort(&graph, None).is_ok(),
        &model.scoped_name,
        "closed joint loops",
    )
}

fn validate_axis(joint: &sdformat::Joint, name: &str) -> Result<(), LoadError> {
    let axis = joint.axis.as_ref().ok_or_else(|| LoadError::Unsupported {
        entity: name.to_owned(),
        feature: "joint without axis",
    })?;
    require(
        axis.lower == f64::NEG_INFINITY
            && axis.upper == f64::INFINITY
            && axis.effort == f64::INFINITY
            && axis.velocity == f64::INFINITY
            && axis.stiffness == 1e8
            && axis.dissipation == 1.0,
        name,
        "joint limits",
    )?;
    let dynamics = &axis.dynamics;
    require(
        dynamics.damping == 0.0
            && dynamics.friction == 0.0
            && dynamics.spring_reference == 0.0
            && dynamics.spring_stiffness == 0.0,
        name,
        "joint dynamics",
    )
}

pub(super) fn build(
    sdf: &Sdf,
    model: &Model,
    context: &SceneContext<'_>,
    model_entity: Entity,
    sim: &mut Simulation,
    backend: &mut PhysicsBackend,
) -> Result<(), LoadError> {
    for joint in &model.joints {
        let name = format!("{}::{}", model.scoped_name, joint.name);
        let child_name = sdf.resolve_name(&model.scoped_name, &joint.child)?;
        let child = context.links[&child_name];
        let entity = entity_by_id(sim.world(), child).expect("loader constructed joint child");
        let body = sim
            .world()
            .get::<BodyHandle>(entity)
            .expect("Rapier link owns body")
            .id();
        let pose = sdf.pose_of(&name, context.world)?;
        let axis = sdf.resolve_axis(
            joint.axis.as_ref().expect("validated revolute axis"),
            &name,
            context.world,
        )?;
        let parent_name = sdf.resolve_name(&model.scoped_name, &joint.parent)?;
        let (id, parent) = if parent_name == context.world {
            (
                backend.add_world_revolute(body, pose.translation, axis)?,
                None,
            )
        } else {
            let parent = entity_by_id(sim.world(), context.links[&parent_name])
                .expect("loader constructed joint parent");
            let parent_body = sim
                .world()
                .get::<BodyHandle>(parent)
                .expect("dynamic-engine link owns body")
                .id();
            (
                backend.add_revolute(parent_body, body, pose.translation, axis)?,
                Some((context.links[&parent_name], parent_body)),
            )
        };
        let local = sdf.pose_of(&name, &model.scoped_name)?;
        let entity = spawn_node(
            sim,
            Some(model_entity),
            &name,
            EntityKind::Joint,
            local,
            pose,
        )?;
        let model_id = sim
            .world()
            .get::<EntityId>(model_entity)
            .expect("loader constructed joint model")
            .get();
        sim.world_mut().entity_mut(entity).insert((
            AttachedPose,
            JointNode,
            JointDefinition(Box::new(joint.clone())),
            JointHandle {
                id,
                child,
                parent,
                model: model_id,
                child_pose: (parent_name != context.world || model.links.len() > 1)
                    .then(|| sdf.pose_of(&name, &child_name).map(WorldPose::from))
                    .transpose()?,
            },
            JointSample {
                state: backend.joint_state(id).expect("constructed joint"),
                time: Duration::ZERO,
                epoch: 0,
            },
        ));
    }
    Ok(())
}

#[derive(Debug, thiserror::Error)]
#[error("loaded joint bindings are incomplete or belong to a different backend")]
struct JointBindingFault;

pub(super) fn bindings_valid(world: &World) -> bool {
    let joints: Vec<_> = world
        .iter_entities()
        .filter(|e| {
            e.get::<EntityKind>() == Some(&EntityKind::Joint) || e.contains::<JointHandle>()
        })
        .collect();
    if joints.is_empty() {
        return true;
    }
    let Some(backend) = world.get_resource::<PhysicsBackend>() else {
        return false;
    };
    if !initial::matches_physics_owner(world) {
        return false;
    }
    let mut seen = BTreeSet::new();
    joints.into_iter().all(|entity| {
        let Some(handle) = entity.get::<JointHandle>() else {
            return false;
        };
        entity.get::<EntityKind>() == Some(&EntityKind::Joint)
            && entity.contains::<JointNode>()
            && entity.contains::<JointSample>()
            && entity.contains::<LocalPose>()
            && entity.contains::<WorldPose>()
            && entity.get::<ChildOf>().is_some_and(|parent| {
                let model = world.entity(parent.parent());
                model
                    .get::<EntityId>()
                    .is_some_and(|id| id.get() == handle.model)
                    && model.get::<EntityKind>() == Some(&EntityKind::Model)
                    && model.contains::<ModelAnchor>()
                    && model.contains::<WorldPose>()
                    && !model.contains::<JointNode>()
                    && !model.contains::<AttachedPose>()
            })
            && handle.parent.is_none_or(|(id, expected)| {
                entity_by_id(world, id).is_some_and(|parent| {
                    world
                        .get::<BodyHandle>(parent)
                        .is_some_and(|body| body.id() == expected)
                        && world.get::<WorldPose>(parent).is_some()
                        && world.get::<Velocity>(parent).is_some()
                        && world.get::<ChildOf>(parent).map(ChildOf::parent)
                            == entity.get::<ChildOf>().map(ChildOf::parent)
                })
            })
            && seen.insert(handle.id)
            && entity_by_id(world, handle.child).is_some_and(|child| {
                world.get::<ChildOf>(child).map(ChildOf::parent)
                    == entity.get::<ChildOf>().map(ChildOf::parent)
                    && world.get::<WorldPose>(child).is_some()
                    && world.get::<Velocity>(child).is_some()
                    && world
                        .get::<BodyHandle>(child)
                        .is_some_and(|body| backend.joint_child(handle.id) == Some(body.id()))
            })
    })
}

type JointBindings<'w, 's> = Query<
    'w,
    's,
    (
        Option<&'static EntityKind>,
        Option<&'static JointHandle>,
        Option<&'static ChildOf>,
        Has<JointNode>,
        Has<JointSample>,
        Has<LocalPose>,
        Has<WorldPose>,
    ),
    Or<(With<JointNode>, With<JointHandle>)>,
>;

type JointModels<'w, 's> = Query<
    'w,
    's,
    (
        &'static EntityId,
        &'static EntityKind,
        &'static WorldPose,
        &'static ModelAnchor,
    ),
    (Without<JointNode>, Without<AttachedPose>),
>;

pub(super) fn preflight(
    initial: Option<Res<initial::InitialScene>>,
    backend: Option<Res<PhysicsBackend>>,
    fault: Res<StepFault>,
    identities: EntityLookup,
    joints: JointBindings,
    children: Query<(&BodyHandle, &ChildOf, &WorldPose, &Velocity)>,
    models: JointModels,
) {
    if joints.is_empty() {
        return;
    }
    let Some(backend) = backend else {
        fault.report(JointBindingFault);
        return;
    };
    let mut seen = BTreeSet::new();
    let valid = initial.is_some_and(|initial| initial.owns_backend(&backend))
        && joints
            .iter()
            .all(|(kind, handle, parent, node, sample, local, pose)| {
                let (Some(handle), Some(parent)) = (handle, parent) else {
                    return false;
                };
                kind == Some(&EntityKind::Joint)
                    && node
                    && sample
                    && local
                    && pose
                    && models.get(parent.parent()).is_ok_and(|(id, kind, _, _)| {
                        id.get() == handle.model && *kind == EntityKind::Model
                    })
                    && handle.parent.is_none_or(|(id, expected)| {
                        identities
                            .get(id)
                            .and_then(|parent| children.get(parent).ok())
                            .is_some_and(|(body, endpoint_parent, _, _)| {
                                body.id() == expected && endpoint_parent.parent() == parent.parent()
                            })
                    })
                    && seen.insert(handle.id)
                    && identities
                        .get(handle.child)
                        .and_then(|child| children.get(child).ok())
                        .is_some_and(|(body, child_parent, _, _)| {
                            child_parent.parent() == parent.parent()
                                && backend.joint_child(handle.id) == Some(body.id())
                        })
            });
    if !valid {
        fault.report(JointBindingFault);
    }
}

pub(super) fn sample(
    time: Res<UpdateInfo>,
    backend: Res<PhysicsBackend>,
    identities: EntityLookup,
    poses: Query<&WorldPose, Without<JointNode>>,
    mut joints: Query<
        (
            &JointHandle,
            &mut JointSample,
            &ChildOf,
            &mut LocalPose,
            &mut WorldPose,
        ),
        With<JointNode>,
    >,
) {
    for (handle, mut sample, model, mut local, mut pose) in &mut joints {
        if let Some(child_pose) = handle.child_pose {
            let child = identities.get(handle.child).expect("validated joint child");
            *pose = poses
                .get(child)
                .expect("validated child pose")
                .compose(child_pose);
            local.0 = poses
                .get(model.parent())
                .expect("scene owns joint model")
                .inverse()
                .compose(*pose);
        }
        sample.state = backend
            .joint_state(handle.id)
            .expect("joint preflight precedes stepping");
        sample.time = time.sim_time;
        sample.epoch = time.epoch;
    }
}
