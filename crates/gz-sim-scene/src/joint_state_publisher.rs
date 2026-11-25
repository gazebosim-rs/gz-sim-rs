// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded built-in JointStatePublisher, following gz-sim 71c60db3.
//! No force measurement is available: the canonical consumer must omit force.
use super::*;
use std::collections::BTreeSet;

/// Immutable loaded publication metadata; mutation does not reconfigure a system.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct JointStatePublication {
    pub model_name: String,
    pub topic: String,
    pub joints: Vec<JointStateJointPublication>,
}

/// Physical joint type used to acquire state for the standard topic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointStateKind {
    Revolute,
    Prismatic,
    Screw,
    Universal,
    Revolute2,
    Ball,
}
impl JointStateKind {
    /// Whether the native message carries two independent angular measurements.
    pub fn has_second_axis(self) -> bool {
        matches!(self, Self::Universal | Self::Revolute2 | Self::Ball)
    }
}

/// Native joint pose is construction pose relative to the child, not world pose.
#[derive(Clone, Debug, PartialEq)]
pub struct JointStateJointPublication {
    pub kind: JointStateKind,
    pub id: u64,
    pub name: String,
    pub parent: String,
    pub child: String,
    pub pose: Transform,
    /// SDF primary axis. Ball joints have no axis metadata and use zeros here and below.
    pub axis: [f64; 3],
    pub lower: f64,
    pub upper: f64,
    pub damping: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointStateValue {
    pub id: u64,
    pub position: f64,
    pub velocity: f64,
}

/// Revision changes on equal-time publications too; it is not a wire sequence.
#[derive(Component, Clone, Debug)]
pub struct JointStateSample {
    pub time: Duration,
    pub epoch: u64,
    pub revision: u64,
    pub pose: WorldPose,
    pub joints: Vec<JointStateValue>,
    /// Additional measured axes, in the same ascending joint-ID order as `joints`.
    pub second_axes: Vec<JointStateValue>,
}

#[derive(Component, Clone)]
pub(super) struct Publisher {
    owner: u64,
    publication: JointStatePublication,
    period: Duration,
    last: Option<Duration>,
    revision: u64,
}

#[derive(Resource)]
pub(super) struct Installed;

pub(super) struct Plan {
    names: BTreeSet<String>,
    topic: String,
    period: Duration,
}

pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::JointStatePublisher"
        && plugin.filename == "gz-sim-joint-state-publisher-system"
}

fn invalid(model: &Model, feature: &'static str) -> LoadError {
    LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    }
}

fn period(rate: f64) -> Option<Duration> {
    if !rate.is_finite() || rate < 0.0 {
        return None;
    }
    if rate == 0.0 {
        return Some(Duration::ZERO);
    }
    // Native duration_cast truncates to integral nanoseconds. Bound reciprocal
    // signed steady-clock range before the float-to-integer cast; sub-nanosecond periods become zero.
    let nanos = (1.0 / rate) * 1e9;
    (nanos.is_finite() && nanos < i64::MAX as f64).then(|| Duration::from_nanos(nanos as u64))
}

fn valid_topic(topic: &str) -> bool {
    topic.starts_with('/')
        && topic.len() > 1
        && topic.len() <= 65535
        && !topic
            .bytes()
            .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
        && !["~", "//", "@", ":="]
            .iter()
            .any(|part| topic.contains(part))
}

pub(super) fn parse(model: &Model) -> Result<Option<Plan>, LoadError> {
    let mut plugins = model.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(plugin) = plugins.next() else {
        return Ok(None);
    };
    require(
        plugins.next().is_none(),
        &model.scoped_name,
        "multiple joint-state publishers",
    )?;
    require(
        plugin.xml.len() <= 65536,
        &model.scoped_name,
        "joint-state configuration size",
    )?;
    let bad = || invalid(model, "joint-state configuration");
    let document = roxmltree::Document::parse(&plugin.xml).map_err(|_| bad())?;
    let root = document.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root
            .attributes()
            .any(|a| a.namespace().is_some() || !matches!(a.name(), "name" | "filename"))
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return Err(bad());
    }
    let mut names = BTreeSet::new();
    let mut topic = None;
    let mut rate = None;
    for child in root.children() {
        if child.is_comment()
            || child.is_text() && child.text().unwrap_or_default().trim().is_empty()
        {
            continue;
        }
        if !child.is_element()
            || child.namespaces().next().is_some()
            || child.attributes().len() != 0
            || child.children().any(|n| !n.is_text())
            || plugin.xml[child.range()].contains("<![CDATA[")
        {
            return Err(bad());
        }
        let text = child.text().unwrap_or_default().trim();
        match child.tag_name().name() {
            "joint_name" => {
                if text.is_empty() || !model.joints.iter().any(|j| j.name == text) {
                    return Err(bad());
                }
                names.insert(text.to_owned());
            }
            "topic" if topic.is_none() => {
                if !valid_topic(text) {
                    return Err(bad());
                }
                topic = Some(text.to_owned());
            }
            "update_rate" if rate.is_none() => {
                rate = Some(period(text.parse().map_err(|_| bad())?).ok_or_else(bad)?);
            }
            _ => return Err(bad()),
        }
    }
    if names.is_empty() {
        names.extend(model.joints.iter().map(|joint| joint.name.clone()));
    }
    require(
        !names.is_empty() && names.len() <= 128,
        &model.scoped_name,
        "joint-state joint count",
    )?;
    let (world, local) = model.scoped_name.split_once("::").ok_or_else(bad)?;
    require(
        !local.contains("::"),
        &model.scoped_name,
        "nested joint-state model",
    )?;
    let topic = topic.unwrap_or_else(|| format!("/world/{world}/model/{}/joint_state", model.name));
    require(valid_topic(&topic), &model.scoped_name, "joint-state topic")?;
    Ok(Some(Plan {
        names,
        topic,
        period: rate.unwrap_or(Duration::ZERO),
    }))
}

impl Plan {
    pub(super) fn validate(
        &self,
        model: &Model,
        sim: &Simulation,
        engine: Option<&str>,
        placement: (bool, bool),
    ) -> Result<(), LoadError> {
        require(
            matches!(
                engine,
                Some("rapier" | "multifidelity" | "gpu-planar" | "gpu-gripper")
            ) && placement.0
                && !placement.1
                && model.models.is_empty()
                && model.raw_namespace.is_none(),
            &model.scoped_name,
            "joint-state requires a supported top-level dynamic model",
        )?;
        require(
            engine == Some("gpu-gripper")
                || sim
                    .world()
                    .iter_entities()
                    .filter(|e| e.contains::<Publisher>())
                    .count()
                    < 8,
            &model.scoped_name,
            "multiple joint-state models",
        )?;
        require(
            model
                .joints
                .iter()
                .filter(|j| self.names.contains(&j.name))
                .all(|j| {
                    use sdformat::JointKind::{
                        Ball, Prismatic, Revolute, Revolute2, Screw, Universal,
                    };
                    let supported = match engine {
                        Some("multifidelity") => {
                            matches!(
                                j.kind,
                                Revolute | Prismatic | Screw | Universal | Revolute2 | Ball
                            )
                        }
                        Some("gpu-gripper") => j.kind == Prismatic,
                        _ => j.kind == Revolute,
                    };
                    supported
                        && (j.axis.is_some() == (j.kind != Ball))
                        && j.axis2.is_some() == matches!(j.kind, Universal | Revolute2)
                        && (j.parent == "world"
                            || model.links.iter().any(|link| link.name == j.parent))
                        && model.links.iter().any(|link| link.name == j.child)
                }),
            &model.scoped_name,
            "joint-state publisher requires supported measurement axes",
        )
    }
}

pub(super) fn build(
    sdf: &Sdf,
    sim: &mut Simulation,
    model: &Model,
    model_entity: Entity,
    backend: Option<&PhysicsBackend>,
    plan: &Plan,
) -> Result<(), LoadError> {
    let mut metadata = Vec::with_capacity(plan.names.len());
    for name in &plan.names {
        let scoped = format!("{}::{name}", model.scoped_name);
        let node = sim
            .world()
            .get::<Children>(model_entity)
            .into_iter()
            .flat_map(|children| children.iter())
            .filter_map(|child| sim.world().get_entity(child).ok())
            .find(|node| node.get::<Name>().is_some_and(|n| n.0 == scoped))
            .ok_or_else(|| invalid(model, "missing joint-state joint"))?;
        require(
            node.get::<ChildOf>().map(ChildOf::parent) == Some(model_entity)
                && node.get::<EntityKind>() == Some(&EntityKind::Joint),
            &scoped,
            "joint-state ownership",
        )?;
        let joint = model
            .joints
            .iter()
            .find(|j| &j.name == name)
            .expect("plan checked joint name");
        let kind = match joint.kind {
            sdformat::JointKind::Prismatic => JointStateKind::Prismatic,
            sdformat::JointKind::Screw => JointStateKind::Screw,
            sdformat::JointKind::Universal => JointStateKind::Universal,
            sdformat::JointKind::Revolute2 => JointStateKind::Revolute2,
            sdformat::JointKind::Ball => JointStateKind::Ball,
            _ => JointStateKind::Revolute,
        };
        let finite = if kind == JointStateKind::Ball {
            let state = backend
                .and_then(|b| b.downcast_ref::<MultiFidelityBackend>())
                .and_then(|b| b.ball_joint_state(node.get::<JointHandle>()?.id()))
                .ok_or_else(|| invalid(model, "missing joint-state value"))?;
            ball_coordinates(state).is_some()
        } else if kind.has_second_axis() {
            let native = backend
                .and_then(|b| b.downcast_ref::<MultiFidelityBackend>())
                .and_then(|b| b.universal_joint_state(node.get::<JointHandle>()?.id()))
                .ok_or_else(|| invalid(model, "missing joint-state value"))?;
            native
                .position
                .into_iter()
                .chain(native.velocity)
                .all(f64::is_finite)
        } else {
            let state = if let Some(backend) = backend {
                let handle = node
                    .get::<JointHandle>()
                    .ok_or_else(|| invalid(model, "missing joint-state binding"))?;
                backend.joint_state(handle.id())
            } else {
                node.get::<JointSample>().map(|s| s.state)
            }
            .ok_or_else(|| invalid(model, "missing joint-state value"))?;
            state.position.is_finite() && state.velocity.is_finite()
        };
        require(finite, &scoped, "finite joint-state value")?;
        let (axis, lower, upper, damping) = if let Some(axis) = &joint.axis {
            (
                sdf.resolve_axis(axis, &scoped, &scoped)?,
                axis.lower,
                axis.upper,
                axis.dynamics.damping,
            )
        } else {
            ([0.0; 3], 0.0, 0.0, 0.0)
        };
        metadata.push(JointStateJointPublication {
            kind,
            id: node.get::<EntityId>().expect("loaded joint identity").get(),
            name: name.clone(),
            parent: joint.parent.clone(),
            child: joint.child.clone(),
            pose: sdf.joint_pose(&scoped, "")?,
            axis,
            lower,
            upper,
            damping,
        });
    }
    metadata.sort_by_key(|entry| entry.id);
    let owner = sim
        .world()
        .get::<EntityId>(model_entity)
        .expect("loaded model identity")
        .get();
    let publication = JointStatePublication {
        model_name: model.name.clone(),
        topic: plan.topic.clone(),
        joints: metadata,
    };
    sim.world_mut().entity_mut(model_entity).insert((
        publication.clone(),
        Publisher {
            owner,
            publication,
            period: plan.period,
            last: Some(Duration::ZERO),
            revision: 0,
        },
    ));
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<Publisher>())
    {
        return;
    }
    sim.add_systems(Phase::PostStep, sample);
    sim.add_systems(Phase::Reset, reset);
    sim.world_mut().insert_resource(Installed);
}

pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>()
        || sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<Publisher>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PostStep, sample)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}

#[derive(Debug, thiserror::Error)]
#[error("joint-state publication ownership, values or revision are invalid")]
struct PublicationFault;

fn finite_pose(pose: WorldPose) -> bool {
    pose.translation
        .iter()
        .chain(pose.rotation.iter())
        .all(|v| v.is_finite())
        && (DQuat::from_array(pose.rotation).length_squared() - 1.0).abs() < 1e-9
}

/// Native ball coordinates are the SO(3) logarithm and angular velocity in the
/// moving joint frame. The publisher writes only components 0 and 1 of each.
fn ball_coordinates(state: gz_physics_multifidelity::BallJointState) -> Option<(DVec3, DVec3)> {
    let rotation = DQuat::from_array(state.orientation);
    let omega = DVec3::from_array(state.angular_velocity);
    if !rotation.is_finite() || (rotation.length_squared() - 1.0).abs() > 1e-9 || !omega.is_finite()
    {
        return None;
    }
    // q and -q encode the same pose. Choose the principal rotation, including
    // a stable sign for a half turn, and retain sub-nanoradian displacements.
    let rotation = if rotation.w < 0.0
        || (rotation.w == 0.0
            && state.orientation[..3]
                .iter()
                .find(|&&v| v != 0.0)
                .is_some_and(|&v| v < 0.0))
    {
        -rotation
    } else {
        rotation
    };
    let imaginary = rotation.xyz();
    let length = imaginary.length();
    let factor = if length < f64::EPSILON {
        2.0
    } else {
        2.0 * length.atan2(rotation.w) / length
    };
    let velocity = rotation.conjugate() * omega;
    velocity
        .is_finite()
        .then_some((imaginary * factor, velocity))
}

fn reading(
    world: &World,
    entity: Entity,
    publisher: &Publisher,
    time: UpdateInfo,
) -> Option<JointStateSample> {
    if world.get::<EntityId>(entity)?.get() != publisher.owner
        || world.get::<EntityKind>(entity) != Some(&EntityKind::Model)
        || world.get::<JointStatePublication>(entity)? != &publisher.publication
    {
        return None;
    }
    let pose = world.get::<LocalPose>(entity)?.0;
    if !finite_pose(pose) {
        return None;
    }
    let mut values = Vec::with_capacity(publisher.publication.joints.len());
    let mut second_axes = Vec::new();
    for joint in &publisher.publication.joints {
        let id = gz_sim_core::entity_by_id(world, joint.id)?;
        if world.get::<ChildOf>(id)?.parent() != entity
            || world.get::<EntityKind>(id) != Some(&EntityKind::Joint)
        {
            return None;
        }
        let measurement = |position: f64, velocity: f64| {
            (position.is_finite() && velocity.is_finite()).then_some(JointStateValue {
                id: joint.id,
                position,
                velocity,
            })
        };
        let (stamp, epoch) = if joint.kind == JointStateKind::Ball {
            let sample = world.get::<BallJointSample>(id)?;
            let (positions, velocities) = ball_coordinates(sample.state)?;
            values.push(measurement(positions.x, velocities.x)?);
            second_axes.push(measurement(positions.y, velocities.y)?);
            (sample.time, sample.epoch)
        } else if joint.kind.has_second_axis() {
            let sample = world.get::<UniversalJointSample>(id)?;
            values.push(measurement(
                sample.state.position[0],
                sample.state.velocity[0],
            )?);
            second_axes.push(measurement(
                sample.state.position[1],
                sample.state.velocity[1],
            )?);
            (sample.time, sample.epoch)
        } else {
            let sample = world.get::<JointSample>(id)?;
            values.push(measurement(sample.state.position, sample.state.velocity)?);
            (sample.time, sample.epoch)
        };
        if stamp != time.sim_time || epoch != time.epoch {
            return None;
        }
    }
    Some(JointStateSample {
        time: time.sim_time,
        epoch: time.epoch,
        revision: publisher.revision.checked_add(1)?,
        pose,
        joints: values,
        second_axes,
    })
}

fn sample(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    let models: Vec<_> = world
        .iter_entities()
        .filter(|e| e.contains::<Publisher>())
        .map(|e| e.id())
        .collect();
    let mut checked_bindings = None;
    for entity in models {
        let publisher = world.get::<Publisher>(entity).expect("selected publisher");
        let suppressed = publisher
            .last
            .and_then(|last| time.sim_time.checked_sub(last))
            .is_some_and(|elapsed| !elapsed.is_zero() && elapsed < publisher.period);
        if suppressed {
            continue;
        }
        let value = if *checked_bindings.get_or_insert_with(|| joints::bindings_valid(world)) {
            reading(world, entity, publisher, time)
        } else {
            None
        };
        let Some(value) = value else {
            world.entity_mut(entity).remove::<JointStateSample>();
            world.resource::<StepFault>().report(PublicationFault);
            return;
        };
        let mut publisher = world
            .get_mut::<Publisher>(entity)
            .expect("selected publisher");
        publisher.last = Some(time.sim_time);
        publisher.revision = value.revision;
        world.entity_mut(entity).insert(value);
    }
}

fn reset(mut commands: Commands, publishers: Query<Entity, With<Publisher>>) {
    for entity in &publishers {
        commands.entity(entity).remove::<JointStateSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/joint_state_publisher.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_state_publishers.rs"]
mod multiple_joint_state_publishers;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_state_publishers_native.rs"]
mod multiple_joint_state_publishers_native_tests;
