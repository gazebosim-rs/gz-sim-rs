// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, JointId, JointState, PhysicsBackend};
use gz_physics_multifidelity::{BallJointState, RobotJoint, UniversalJointState};
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
    entity: u64,
    child: u64,
    sample_kind: SampleKind,
    /// SDF axes relative to the parent, independent of scalar constraint axes.
    pub(super) gravity_axes: Option<[f64; 4]>,
    child_pose: Option<WorldPose>,
    parent: Option<(u64, BodyId)>,
    model: u64,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum SampleKind {
    Scalar,
    Ball,
    Universal,
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

/// Rotation and angular velocity in the parent joint frame at a committed instant.
#[derive(Component, Clone, Copy, Debug)]
pub struct BallJointSample {
    pub state: BallJointState,
    pub time: Duration,
    pub epoch: u64,
}

/// Two continuous joint coordinates at a committed simulation instant.
#[derive(Component, Clone, Copy, Debug)]
pub struct UniversalJointSample {
    pub state: UniversalJointState,
    pub time: Duration,
    pub epoch: u64,
}

fn mixed_robot(model: &Model) -> bool {
    model.joints.iter().any(|j| {
        matches!(
            j.kind,
            sdformat::JointKind::Universal
                | sdformat::JointKind::Revolute2
                | sdformat::JointKind::Ball
                | sdformat::JointKind::Prismatic
                | sdformat::JointKind::Screw
        )
    })
}

pub(super) fn is_scalar_tree_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::ReducedScalarJoints"
        && plugin.filename == "gz-sim-reduced-scalar-joints-system"
}

/// Scalar native joints can combine a bounded slider with a public effort command.
fn scalar_tree(model: &Model, engine: Option<&str>) -> Result<bool, LoadError> {
    let mut markers = model
        .plugins
        .iter()
        .filter(|plugin| is_scalar_tree_plugin(plugin));
    let Some(marker) = markers.next() else {
        return Ok(false);
    };
    let invalid = || LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature: "ReducedScalarJoints requires one world-mounted prismatic/unbounded revolute tree without spring or motor ownership",
    };
    let xml = roxmltree::Document::parse(&marker.xml).map_err(|_| invalid())?;
    let root = xml.root_element();
    if markers.next().is_some()
        || engine != Some("multifidelity")
        || !mixed_robot(model)
        || !model.joints.iter().all(|joint| {
            matches!(
                joint.kind,
                sdformat::JointKind::Prismatic | sdformat::JointKind::Revolute
            ) && joint.axis.as_ref().is_some_and(|axis| {
                (joint.kind == sdformat::JointKind::Prismatic
                    || axis.lower == f64::NEG_INFINITY && axis.upper == f64::INFINITY)
                    && axis.dynamics.damping == 0.
                    && axis.dynamics.spring_stiffness == 0.
            })
        })
        || !model.plugins.iter().all(|p| {
            is_scalar_tree_plugin(p)
                || contact_material::is_plugin(p)
                || joint_state_publisher::is_plugin(p)
                || joint_forces::is_plugin(p)
        })
        || root.attributes().len() != 2
        || root.namespaces().next().is_some()
        || !root
            .children()
            .all(|n| n.is_comment() || n.is_text() && n.text().unwrap_or("").trim().is_empty())
    {
        return Err(invalid());
    }
    Ok(true)
}

#[derive(Debug)]
pub(super) struct ReducedPlan {
    pub switchable_root: Option<usize>,
    pub scalar_robot: bool,
    pub exact_revolute_locks: bool,
    pub additional_pgs_iterations: Option<u8>,
    pub joint_order: Vec<usize>,
}

pub(super) enum JointConstruction<'a> {
    Impulse,
    Controller,
    Reduced(&'a ReducedPlan),
}

#[allow(clippy::too_many_arguments)]
pub(super) fn validate_model(
    sdf: &Sdf,
    model: &Model,
    world: &str,
    top_level: bool,
    engine: Option<&str>,
    is_static: bool,
    actuated: &[&str],
    coated_tree: bool,
) -> Result<Option<ReducedPlan>, LoadError> {
    let scalar_robot = scalar_tree(model, engine)?;
    let options = articulation::validate_marker(model, engine)?;
    let switchable = options.is_some();
    for joint in &model.joints {
        require(
            joint.sensors.is_empty(),
            &format!("{}::{}", model.scoped_name, joint.name),
            "joint sensors",
        )?;
    }
    if model.joints.is_empty() {
        return Ok(None);
    }
    require(engine.is_some(), &model.scoped_name, "TPE joints")?;
    require(
        top_level && !is_static && model.models.is_empty(),
        &model.scoped_name,
        "joint topology (expected one dynamic world-anchored link)",
    )?;
    let links: BTreeMap<_, _> = model
        .links
        .iter()
        .enumerate()
        .map(|(index, link)| (link.scoped_name.as_str(), index))
        .collect();
    let welded = engine == Some("multifidelity")
        && !switchable
        && actuated.is_empty()
        && model
            .joints
            .iter()
            .all(|j| j.kind == sdformat::JointKind::Fixed);
    let robot = engine == Some("multifidelity") && mixed_robot(model);
    require(
        robot || options.is_none_or(|o| o.additional_pgs_iterations.is_none()),
        &model.scoped_name,
        "additional PGS iterations require a rooted mixed robot",
    )?;
    let mut graph = DiGraphMap::<usize, usize>::new();
    let mut children = BTreeSet::new();
    let mut bounded = false;
    for (index, joint) in model.joints.iter().enumerate() {
        let name = format!("{}::{}", model.scoped_name, joint.name);
        require(
            (welded
                || joint.kind == sdformat::JointKind::Revolute
                || robot
                    && matches!(
                        joint.kind,
                        sdformat::JointKind::Fixed
                            | sdformat::JointKind::Universal
                            | sdformat::JointKind::Revolute2
                            | sdformat::JointKind::Ball
                            | sdformat::JointKind::Prismatic
                            | sdformat::JointKind::Screw
                    ))
                && (joint.axis2.is_some()
                    == (matches!(
                        joint.kind,
                        sdformat::JointKind::Universal | sdformat::JointKind::Revolute2
                    ))),
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
            parent_index.is_some() && child_index.is_some() && parent != child,
            &name,
            "joint endpoints (expected world and sole link)",
        )?;
        let child_index = child_index.expect("validated local child");
        require(
            children.insert(child_index),
            &name,
            "multiple parent joints",
        )?;
        let parent_index = parent_index.expect("validated parent");
        graph.add_edge(parent_index, child_index, index);
        if matches!(
            joint.kind,
            sdformat::JointKind::Ball | sdformat::JointKind::Fixed
        ) {
            require(
                joint.axis.is_none() && joint.physics_xml.is_none(),
                &name,
                "fixed constraint axis or physics overrides",
            )?;
            continue;
        }
        if joint.kind == sdformat::JointKind::Screw {
            let lead = joint.screw_thread_pitch / std::f64::consts::TAU;
            require(
                (joint.screw_thread_pitch == 0. || lead.is_normal()) && (lead * lead).is_finite(),
                &name,
                "invalid screw thread pitch",
            )?;
        }
        if matches!(
            joint.kind,
            sdformat::JointKind::Universal | sdformat::JointKind::Revolute2
        ) {
            validate_axis(&joint.kind, joint.axis2.as_ref(), &name, false, true, robot)?;
        }
        bounded |= validate_axis(
            &joint.kind,
            joint.axis.as_ref(),
            &name,
            actuated.contains(&joint.name.as_str())
                && !matches!(
                    joint.kind,
                    sdformat::JointKind::Universal | sdformat::JointKind::Revolute2
                ),
            engine == Some("multifidelity"),
            robot
                && matches!(
                    joint.kind,
                    sdformat::JointKind::Revolute
                        | sdformat::JointKind::Prismatic
                        | sdformat::JointKind::Screw
                        | sdformat::JointKind::Universal
                        | sdformat::JointKind::Revolute2
                ),
        )?;
    }
    let ordered = toposort(&graph, None).map_err(|_| LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature: "closed joint loops",
    })?;
    if scalar_robot {
        require(
            graph.neighbors(model.links.len()).count() == 1,
            &model.scoped_name,
            "ReducedScalarJoints requires a single world mount",
        )?;
    }
    if robot && switchable {
        require(
            graph.neighbors(model.links.len()).count() == 1,
            &model.scoped_name,
            "switchable mixed robot requires a single world mount",
        )?;
        require(
            !options.is_some_and(|o| o.exact_revolute_locks),
            &model.scoped_name,
            "exact revolute locks require a floating revolute tree",
        )?;
    }
    if welded || robot {
        let nodes = model.links.len() + usize::from(graph.contains_node(model.links.len()));
        require(
            graph.node_count() == nodes && model.joints.len() + 1 == nodes,
            &model.scoped_name,
            if robot {
                "robot joints require one connected tree"
            } else {
                "fixed joints require one connected tree"
            },
        )?;
    }
    if !switchable && !bounded && !welded && !robot && !coated_tree {
        return Ok(None);
    }
    // Unique parents and acyclicity were validated above. A connected floating
    // tree therefore has exactly one root and one incoming joint on each other link.
    require(
        !switchable
            || robot
            || (!graph.contains_node(model.links.len())
                && model.joints.len() + 1 == model.links.len()),
        &model.scoped_name,
        "SwitchableArticulation requires one floating connected revolute tree",
    )?;
    let joint_order = ordered
        .iter()
        .flat_map(|&link| {
            graph
                .edges_directed(link, petgraph::Direction::Incoming)
                .map(|(_, _, joint)| *joint)
        })
        .collect();
    Ok(Some(ReducedPlan {
        scalar_robot,
        switchable_root: switchable.then(|| if robot { ordered[1] } else { ordered[0] }),
        exact_revolute_locks: options.is_some_and(|o| o.exact_revolute_locks),
        additional_pgs_iterations: options.and_then(|o| o.additional_pgs_iterations),
        joint_order,
    }))
}

fn validate_axis(
    kind: &sdformat::JointKind,
    settings: Option<&sdformat::Axis>,
    name: &str,
    driven: bool,
    allow_stops: bool,
    allow_spring: bool,
) -> Result<bool, LoadError> {
    let axis = settings.ok_or_else(|| LoadError::Unsupported {
        entity: name.to_owned(),
        feature: "joint without axis",
    })?;
    let bounded = allow_stops
        && axis.lower <= 0.
        && axis.upper >= 0.
        && if matches!(
            kind,
            sdformat::JointKind::Universal
                | sdformat::JointKind::Revolute2
                | sdformat::JointKind::Prismatic
                | sdformat::JointKind::Screw
        ) {
            axis.lower.is_finite() || axis.upper.is_finite()
        } else {
            axis.lower.is_finite() && axis.upper.is_finite()
        };
    require(
        (bounded || (axis.lower == f64::NEG_INFINITY && axis.upper == f64::INFINITY))
            && (if driven {
                (allow_stops || axis.effort.is_finite())
                    && axis.effort > 0.0
                    && (allow_stops || axis.velocity.is_finite())
                    && axis.velocity > 0.0
            } else {
                (axis.effort == f64::INFINITY
                    || matches!(
                        kind,
                        sdformat::JointKind::Screw
                            | sdformat::JointKind::Universal
                            | sdformat::JointKind::Revolute2
                    ) && axis.effort >= 0.)
                    && (axis.velocity == f64::INFINITY
                        || matches!(
                            kind,
                            sdformat::JointKind::Universal
                                | sdformat::JointKind::Revolute2
                                | sdformat::JointKind::Screw
                        ) && axis.velocity >= 0.)
            })
            && axis.stiffness == 1e8
            && axis.dissipation == 1.0,
        name,
        "joint limits",
    )?;
    let dynamics = &axis.dynamics;
    require(
        dynamics.friction == 0.0
            && if allow_spring {
                dynamics.damping.is_finite()
                    && dynamics.damping >= 0.
                    && dynamics.spring_stiffness.is_finite()
                    && dynamics.spring_stiffness >= 0.
                    && dynamics.spring_reference.is_finite()
                    && (dynamics.spring_stiffness > 0. || dynamics.spring_reference == 0.)
            } else {
                dynamics.damping == 0.
                    && dynamics.spring_reference == 0.
                    && dynamics.spring_stiffness == 0.
            },
        name,
        "joint dynamics",
    )?;
    Ok(bounded)
}

pub(super) fn build(
    sdf: &Sdf,
    model: &Model,
    context: &SceneContext<'_>,
    model_entity: Entity,
    sim: &mut Simulation,
    backend: &mut PhysicsBackend,
    construction: JointConstruction<'_>,
) -> Result<(), LoadError> {
    let robot = mixed_robot(model);
    let mut order: Vec<_> = (0..model.joints.len()).collect();
    let scalar_robot =
        matches!(construction, JointConstruction::Reduced(plan) if plan.scalar_robot);
    if let JointConstruction::Reduced(plan) = construction {
        order.copy_from_slice(&plan.joint_order);
    } else if matches!(construction, JointConstruction::Controller) && model.joints.len() == 2 {
        let endpoints: Vec<_> = model
            .joints
            .iter()
            .map(|joint| {
                Ok((
                    sdf.resolve_name(&model.scoped_name, &joint.parent)?,
                    sdf.resolve_name(&model.scoped_name, &joint.child)?,
                ))
            })
            .collect::<Result<_, LoadError>>()?;
        let root = endpoints
            .iter()
            .position(|(parent, _)| {
                parent == context.world || !endpoints.iter().any(|(_, child)| child == parent)
            })
            .expect("validated serial chain has one root joint");
        order.swap(0, root);
    }
    let mut built: Vec<_> = (0..model.joints.len()).map(|_| None).collect();
    for index in order {
        let joint = &model.joints[index];
        let name = format!("{}::{}", model.scoped_name, joint.name);
        let child_name = sdf.resolve_name(&model.scoped_name, &joint.child)?;
        let child = context.links[&child_name];
        let entity = entity_by_id(sim.world(), child).expect("loader constructed joint child");
        let body = sim
            .world()
            .get::<BodyHandle>(entity)
            .expect("dynamic-engine link owns body")
            .id();
        let pose = sdf.pose_of(&name, context.world)?;
        let parent_name = sdf.resolve_name(&model.scoped_name, &joint.parent)?;
        let parent = if parent_name == context.world {
            None
        } else {
            let public_id = context.links[&parent_name];
            let entity =
                entity_by_id(sim.world(), public_id).expect("loader constructed joint parent");
            let handle = sim
                .world()
                .get::<BodyHandle>(entity)
                .expect("dynamic-engine link owns body");
            Some((public_id, handle.id()))
        };
        let parent_body = parent.map(|(_, body)| body);
        let id = if robot {
            let motion = match joint.kind {
                sdformat::JointKind::Universal | sdformat::JointKind::Revolute2 => {
                    let settings = [
                        joint.axis.as_ref().expect("validated axis"),
                        joint.axis2.as_ref().expect("validated axis2"),
                    ];
                    let [first, second] =
                        settings.map(|a| sdf.resolve_axis(a, &name, context.world));
                    RobotJoint::Universal {
                        axes: [first?, second?],
                        effort_limits: settings.map(|a| a.effort),
                        limits: settings.map(|a| {
                            (a.lower.is_finite() || a.upper.is_finite())
                                .then_some([a.lower, a.upper])
                        }),
                    }
                }
                sdformat::JointKind::Ball => RobotJoint::Ball {
                    orientation: pose.rotation,
                },
                sdformat::JointKind::Fixed => RobotJoint::Fixed,
                _ => {
                    let settings = joint.axis.as_ref().expect("validated scalar axis");
                    let axis = sdf.resolve_axis(settings, &name, context.world)?;
                    let limits = (settings.lower.is_finite() || settings.upper.is_finite())
                        .then_some([settings.lower, settings.upper]);
                    if joint.kind == sdformat::JointKind::Screw {
                        RobotJoint::Screw {
                            axis,
                            pitch: joint.screw_thread_pitch,
                            limits,
                            effort_limit: settings.effort,
                        }
                    } else if joint.kind == sdformat::JointKind::Prismatic {
                        RobotJoint::Prismatic { axis, limits }
                    } else {
                        RobotJoint::Revolute { axis, limits }
                    }
                }
            };
            let local = backend
                .downcast_mut::<MultiFidelityBackend>()
                .expect("validated local robot");
            let springs = [joint.axis.as_ref(), joint.axis2.as_ref()].map(|axis| {
                let dynamics = &axis?.dynamics;
                (dynamics.damping != 0. || dynamics.spring_stiffness != 0.).then_some(
                    gz_physics_multifidelity::RobotSpring {
                        stiffness: dynamics.spring_stiffness,
                        damping: dynamics.damping,
                        reference: dynamics.spring_reference,
                    },
                )
            });
            let id = if scalar_robot {
                local.add_reduced_scalar_robot_joint(parent_body, body, pose.translation, motion)?
            } else if matches!(motion, RobotJoint::Universal { .. }) {
                local.add_universal_joint_with_springs(
                    parent_body,
                    body,
                    pose.translation,
                    motion,
                    springs,
                )?
            } else if let Some(spring) = springs[0] {
                local.add_robot_joint_with_spring(
                    parent_body,
                    body,
                    pose.translation,
                    motion,
                    spring,
                )?
            } else {
                local.add_robot_joint(parent_body, body, pose.translation, motion)?
            };
            let limits = [joint.axis.as_ref(), joint.axis2.as_ref()]
                .map(|axis| axis.map(|a| a.velocity).filter(|v| v.is_finite()));
            if matches!(
                motion,
                RobotJoint::Universal { .. } | RobotJoint::Screw { .. }
            ) && limits.iter().any(Option::is_some)
            {
                local.set_joint_angular_speed_limits(id, limits)?;
            }
            id
        } else if joint.kind == sdformat::JointKind::Fixed {
            backend
                .downcast_mut::<MultiFidelityBackend>()
                .expect("fixed joints use the multifidelity solver")
                .add_fixed_joint(parent_body, body, pose.translation)?
        } else {
            let axis = sdf.resolve_axis(
                joint.axis.as_ref().expect("validated revolute axis"),
                &name,
                context.world,
            )?;
            let settings = joint.axis.as_ref().expect("validated revolute axis");
            if settings.lower.is_finite() {
                backend
                    .downcast_mut::<MultiFidelityBackend>()
                    .expect("bounded revolutes use the multi-fidelity solver")
                    .add_limited_revolute(
                        parent_body,
                        body,
                        pose.translation,
                        axis,
                        [settings.lower, settings.upper],
                    )?
            } else if !matches!(construction, JointConstruction::Impulse) {
                backend.add_reduced_revolute(parent_body, body, pose.translation, axis)?
            } else if let Some(parent) = parent_body {
                backend.add_revolute(parent, body, pose.translation, axis)?
            } else {
                backend.add_world_revolute(body, pose.translation, axis)?
            }
        };
        built[index] = Some((id, parent, child, child_name, parent_name, pose));
    }
    // Solver construction order must not change public declaration identities.
    for (index, joint) in model.joints.iter().enumerate() {
        let (id, parent, child, child_name, parent_name, pose) = built[index]
            .take()
            .expect("each declared joint was constructed once");
        let name = format!("{}::{}", model.scoped_name, joint.name);
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
        let public_id = sim
            .world()
            .get::<EntityId>(entity)
            .expect("loader constructed joint identity")
            .get();
        sim.world_mut().entity_mut(entity).insert((
            AttachedPose,
            JointNode,
            JointDefinition(Box::new(joint.clone())),
            JointHandle {
                id,
                entity: public_id,
                sample_kind: match joint.kind {
                    sdformat::JointKind::Universal | sdformat::JointKind::Revolute2 => {
                        SampleKind::Universal
                    }
                    sdformat::JointKind::Ball => SampleKind::Ball,
                    _ => SampleKind::Scalar,
                },
                gravity_axes: matches!(backend.engine(), "multifidelity" | "rapier")
                    .then(|| sdf.pose_of(&name, &parent_name).map(|p| p.rotation))
                    .transpose()?,
                child,
                parent,
                model: model_id,
                child_pose: (robot || parent_name != context.world || model.links.len() > 1)
                    .then(|| sdf.pose_of(&name, &child_name).map(WorldPose::from))
                    .transpose()?,
            },
        ));
        if matches!(
            joint.kind,
            sdformat::JointKind::Universal | sdformat::JointKind::Revolute2
        ) {
            let state = backend
                .downcast_ref::<MultiFidelityBackend>()
                .and_then(|b| b.universal_joint_state(id))
                .expect("constructed universal joint");
            sim.world_mut()
                .entity_mut(entity)
                .insert(UniversalJointSample {
                    state,
                    time: Duration::ZERO,
                    epoch: 0,
                });
        } else if joint.kind == sdformat::JointKind::Ball {
            let state = backend
                .downcast_ref::<MultiFidelityBackend>()
                .and_then(|b| b.ball_joint_state(id))
                .expect("constructed ball joint");
            sim.world_mut().entity_mut(entity).insert(BallJointSample {
                state,
                time: Duration::ZERO,
                epoch: 0,
            });
        } else {
            let state = backend.joint_state(id).expect("constructed scalar joint");
            sim.world_mut().entity_mut(entity).insert(JointSample {
                state,
                time: Duration::ZERO,
                epoch: 0,
            });
        }
    }
    Ok(())
}

#[derive(Debug, thiserror::Error)]
#[error("loaded joint bindings are incomplete or belong to a different backend")]
struct JointBindingFault;

pub(super) fn removal_order(world: &World, bodies: &BTreeSet<BodyId>) -> Option<Vec<BodyId>> {
    let mut graph = DiGraphMap::<BodyId, ()>::new();
    for body in bodies {
        graph.add_node(*body);
    }
    for entity in world.iter_entities() {
        if let Some(handle) = entity.get::<JointHandle>()
            && let Some((_, parent)) = handle.parent
        {
            let child = world
                .get::<BodyHandle>(entity_by_id(world, handle.child)?)?
                .id();
            if bodies.contains(&parent) && bodies.contains(&child) {
                graph.add_edge(parent, child, ());
            }
        }
    }
    // Removing a parent first may temporarily re-enable contacts with its child.
    let mut order = toposort(&graph, None).ok()?;
    order.reverse();
    Some(order)
}

pub(super) fn bindings_valid(world: &World) -> bool {
    #[cfg(feature = "gpu-batch")]
    if crate::gpu_scene::parallel_gripper::registered(world) {
        return crate::gpu_scene::parallel_gripper::valid(world);
    }
    #[cfg(feature = "gpu-batch")]
    if let Some(valid) = crate::gpu_scene::planar_tool::joint_bindings_valid(world) {
        return valid;
    }
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
        valid_binding(world, backend, entity)
            && entity
                .get::<JointHandle>()
                .is_some_and(|handle| seen.insert(handle.id))
    })
}

fn valid_binding(world: &World, backend: &PhysicsBackend, entity: EntityRef<'_>) -> bool {
    let Some(handle) = entity.get::<JointHandle>() else {
        return false;
    };
    entity.get::<EntityKind>() == Some(&EntityKind::Joint)
        && entity_by_id(world, handle.entity) == Some(entity.id())
        && entity.contains::<JointNode>()
        && (entity.contains::<JointSample>() == (handle.sample_kind == SampleKind::Scalar))
        && (entity.contains::<UniversalJointSample>()
            == (handle.sample_kind == SampleKind::Universal))
        && (entity.contains::<BallJointSample>() == (handle.sample_kind == SampleKind::Ball))
        && entity.contains::<LocalPose>()
        && entity.contains::<WorldPose>()
        && entity.get::<ChildOf>().is_some_and(|parent| {
            world.get_entity(parent.parent()).is_ok_and(|model| {
                model
                    .get::<EntityId>()
                    .is_some_and(|id| id.get() == handle.model)
                    && model.get::<EntityKind>() == Some(&EntityKind::Model)
                    && model.contains::<ModelAnchor>()
                    && model.contains::<WorldPose>()
                    && !model.contains::<JointNode>()
                    && !model.contains::<AttachedPose>()
            })
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
        && entity_by_id(world, handle.child).is_some_and(|child| {
            world.get::<ChildOf>(child).map(ChildOf::parent)
                == entity.get::<ChildOf>().map(ChildOf::parent)
                && world.get::<WorldPose>(child).is_some()
                && world.get::<Velocity>(child).is_some()
                && world
                    .get::<BodyHandle>(child)
                    .is_some_and(|body| backend.joint_child(handle.id) == Some(body.id()))
        })
}

pub(super) fn input_binding(world: &World, entity: Entity) -> Option<JointId> {
    if !initial::matches_physics_owner(world) {
        return None;
    }
    let backend = world.get_resource::<PhysicsBackend>()?;
    let entity = world.get_entity(entity).ok()?;
    if valid_binding(world, backend, entity) {
        entity.get::<JointHandle>().map(|handle| handle.id)
    } else {
        None
    }
}

type JointBindings<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        Option<&'static EntityKind>,
        Option<&'static JointHandle>,
        Option<&'static ChildOf>,
        Has<JointNode>,
        Has<JointSample>,
        Has<BallJointSample>,
        Has<UniversalJointSample>,
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
        && joints.iter().all(
            |(
                entity,
                kind,
                handle,
                parent,
                node,
                sample,
                ball_sample,
                universal_sample,
                local,
                pose,
            )| {
                let (Some(handle), Some(parent)) = (handle, parent) else {
                    return false;
                };
                identities.get(handle.entity) == Some(entity)
                    && kind == Some(&EntityKind::Joint)
                    && node
                    && sample == (handle.sample_kind == SampleKind::Scalar)
                    && universal_sample == (handle.sample_kind == SampleKind::Universal)
                    && ball_sample == (handle.sample_kind == SampleKind::Ball)
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
            },
        );
    if !valid {
        fault.report(JointBindingFault);
    }
}

type JointSamples<'w, 's> = Query<
    'w,
    's,
    (
        &'static JointHandle,
        Option<&'static mut JointSample>,
        Option<&'static mut BallJointSample>,
        Option<&'static mut UniversalJointSample>,
        &'static ChildOf,
        &'static mut LocalPose,
        &'static mut WorldPose,
    ),
    With<JointNode>,
>;

pub(super) fn sample(
    time: Res<UpdateInfo>,
    backend: Res<PhysicsBackend>,
    identities: EntityLookup,
    poses: Query<&WorldPose, Without<JointNode>>,
    mut joints: JointSamples,
) {
    for (handle, sample, ball, universal, model, mut local, mut pose) in &mut joints {
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
        if let Some(mut sample) = sample {
            sample.state = backend
                .joint_state(handle.id)
                .expect("validated scalar joint");
            sample.time = time.sim_time;
            sample.epoch = time.epoch;
        }
        if let Some(mut sample) = universal {
            sample.state = backend
                .downcast_ref::<MultiFidelityBackend>()
                .and_then(|b| b.universal_joint_state(handle.id))
                .expect("validated universal joint");
            sample.time = time.sim_time;
            sample.epoch = time.epoch;
        }
        if let Some(mut sample) = ball {
            sample.state = backend
                .downcast_ref::<MultiFidelityBackend>()
                .and_then(|b| b.ball_joint_state(handle.id))
                .expect("validated ball joint");
            sample.time = time.sim_time;
            sample.epoch = time.epoch;
        }
    }
}
