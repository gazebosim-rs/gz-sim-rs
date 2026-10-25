// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_rapier::{JointId, JointState};
use gz_sim_core::entity_by_id;
use std::collections::BTreeSet;

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
    rapier: bool,
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
    require(rapier, &model.scoped_name, "TPE joints")?;
    require(
        top_level
            && !is_static
            && model.models.is_empty()
            && model.links.len() == 1
            && model.joints.len() == 1,
        &model.scoped_name,
        "joint topology (expected one dynamic world-anchored link)",
    )?;
    let joint = &model.joints[0];
    let name = format!("{}::{}", model.scoped_name, joint.name);
    require(
        joint.kind == sdformat::JointKind::Revolute && joint.axis2.is_none(),
        &name,
        "joints (unsupported type)",
    )?;
    require(
        sdf.resolve_name(&model.scoped_name, &joint.parent)? == world
            && sdf.resolve_name(&model.scoped_name, &joint.child)? == model.links[0].scoped_name,
        &name,
        "joint endpoints (expected world and sole link)",
    )?;
    let axis = joint.axis.as_ref().ok_or_else(|| LoadError::Unsupported {
        entity: name.clone(),
        feature: "joint without axis",
    })?;
    require(
        axis.lower == f64::NEG_INFINITY
            && axis.upper == f64::INFINITY
            && axis.effort == f64::INFINITY
            && axis.velocity == f64::INFINITY
            && axis.stiffness == 1e8
            && axis.dissipation == 1.0,
        &name,
        "joint limits",
    )?;
    let dynamics = &axis.dynamics;
    require(
        dynamics.damping == 0.0
            && dynamics.friction == 0.0
            && dynamics.spring_reference == 0.0
            && dynamics.spring_stiffness == 0.0,
        &name,
        "joint dynamics",
    )
}

pub(super) fn build(
    sdf: &Sdf,
    model: &Model,
    context: &SceneContext<'_>,
    model_entity: Entity,
    sim: &mut Simulation,
    backend: &mut RapierBackend,
) -> Result<(), LoadError> {
    for joint in &model.joints {
        let name = format!("{}::{}", model.scoped_name, joint.name);
        let child = context.links[&model.links[0].scoped_name];
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
        let id = backend.add_world_revolute(body, pose.translation, axis)?;
        let local = sdf.pose_of(&name, &model.scoped_name)?;
        let entity = spawn_node(
            sim,
            Some(model_entity),
            &name,
            EntityKind::Joint,
            local,
            pose,
        )?;
        sim.world_mut().entity_mut(entity).insert((
            AttachedPose,
            JointNode,
            JointDefinition(Box::new(joint.clone())),
            JointHandle { id, child },
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
    let Some(backend) = world.get_resource::<RapierBackend>() else {
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
            && seen.insert(handle.id)
            && entity_by_id(world, handle.child).is_some_and(|child| {
                world.get::<ChildOf>(child).map(ChildOf::parent)
                    == entity.get::<ChildOf>().map(ChildOf::parent)
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
    ),
    Or<(With<JointNode>, With<JointHandle>)>,
>;

pub(super) fn preflight(
    initial: Option<Res<initial::InitialScene>>,
    backend: Option<Res<RapierBackend>>,
    fault: Res<StepFault>,
    identities: EntityLookup,
    joints: JointBindings,
    children: Query<(&BodyHandle, &ChildOf)>,
) {
    if joints.is_empty() {
        return;
    }
    let Some(backend) = backend else {
        fault.report(JointBindingFault);
        return;
    };
    let mut seen = BTreeSet::new();
    let valid = initial.is_some_and(|initial| initial.owns_rapier(&backend))
        && joints.iter().all(|(kind, handle, parent)| {
            let (Some(handle), Some(parent)) = (handle, parent) else {
                return false;
            };
            kind == Some(&EntityKind::Joint)
                && seen.insert(handle.id)
                && identities
                    .get(handle.child)
                    .and_then(|child| children.get(child).ok())
                    .is_some_and(|(body, child_parent)| {
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
    backend: Res<RapierBackend>,
    mut joints: Query<(&JointHandle, &mut JointSample)>,
) {
    for (handle, mut sample) in &mut joints {
        sample.state = backend
            .joint_state(handle.id)
            .expect("joint preflight precedes stepping");
        sample.time = time.sim_time;
        sample.epoch = time.epoch;
    }
}
