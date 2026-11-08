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

/// Native joint pose is construction pose relative to the child, not world pose.
#[derive(Clone, Debug, PartialEq)]
pub struct JointStateJointPublication {
    pub id: u64,
    pub name: String,
    pub parent: String,
    pub child: String,
    pub pose: Transform,
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
            engine == Some("rapier")
                && placement.0
                && !placement.1
                && model.models.is_empty()
                && model.raw_namespace.is_none(),
            &model.scoped_name,
            "joint-state requires a top-level dynamic Rapier model",
        )?;
        require(
            !sim.world()
                .iter_entities()
                .any(|e| e.contains::<Publisher>()),
            &model.scoped_name,
            "multiple joint-state models",
        )?;
        require(
            model.joints.iter().all(|j| {
                j.kind == sdformat::JointKind::Revolute
                    && j.axis.is_some()
                    && j.axis2.is_none()
                    && (j.parent == "world" || model.links.iter().any(|link| link.name == j.parent))
                    && model.links.iter().any(|link| link.name == j.child)
            }),
            &model.scoped_name,
            "joint-state single-axis revolute joints",
        )
    }
}

pub(super) fn build(
    sdf: &Sdf,
    sim: &mut Simulation,
    model: &Model,
    model_entity: Entity,
    backend: &PhysicsBackend,
    plan: &Plan,
) -> Result<(), LoadError> {
    let mut metadata = Vec::with_capacity(plan.names.len());
    for name in &plan.names {
        let scoped = format!("{}::{name}", model.scoped_name);
        let node = sim
            .world()
            .iter_entities()
            .find(|node| node.get::<Name>().is_some_and(|n| n.0 == scoped))
            .ok_or_else(|| invalid(model, "missing joint-state joint"))?;
        require(
            node.get::<ChildOf>().map(ChildOf::parent) == Some(model_entity)
                && node.get::<EntityKind>() == Some(&EntityKind::Joint),
            &scoped,
            "joint-state ownership",
        )?;
        let handle = node
            .get::<JointHandle>()
            .ok_or_else(|| invalid(model, "missing joint-state binding"))?;
        let state = backend
            .joint_state(handle.id())
            .ok_or_else(|| invalid(model, "missing joint-state value"))?;
        require(
            state.position.is_finite() && state.velocity.is_finite(),
            &scoped,
            "finite joint-state value",
        )?;
        let joint = model
            .joints
            .iter()
            .find(|j| &j.name == name)
            .expect("plan checked joint name");
        let axis = joint.axis.as_ref().expect("validated revolute axis");
        metadata.push(JointStateJointPublication {
            id: node.get::<EntityId>().expect("loaded joint identity").get(),
            name: name.clone(),
            parent: joint.parent.clone(),
            child: joint.child.clone(),
            pose: sdf.joint_pose(&scoped, "")?,
            axis: sdf.resolve_axis(axis, &scoped, &scoped)?,
            lower: axis.lower,
            upper: axis.upper,
            damping: axis.dynamics.damping,
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
    for joint in &publisher.publication.joints {
        let id = gz_sim_core::entity_by_id(world, joint.id)?;
        if world.get::<ChildOf>(id)?.parent() != entity
            || world.get::<EntityKind>(id) != Some(&EntityKind::Joint)
        {
            return None;
        }
        let sample = world.get::<JointSample>(id)?;
        if sample.time != time.sim_time
            || sample.epoch != time.epoch
            || !sample.state.position.is_finite()
            || !sample.state.velocity.is_finite()
        {
            return None;
        }
        values.push(JointStateValue {
            id: joint.id,
            position: sample.state.position,
            velocity: sample.state.velocity,
        });
    }
    Some(JointStateSample {
        time: time.sim_time,
        epoch: time.epoch,
        revision: publisher.revision.checked_add(1)?,
        pose,
        joints: values,
    })
}

fn sample(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    let models: Vec<_> = world
        .iter_entities()
        .filter(|e| e.contains::<Publisher>())
        .map(|e| e.id())
        .collect();
    for entity in models {
        let publisher = world.get::<Publisher>(entity).expect("selected publisher");
        let suppressed = publisher
            .last
            .and_then(|last| time.sim_time.checked_sub(last))
            .is_some_and(|elapsed| !elapsed.is_zero() && elapsed < publisher.period);
        if suppressed {
            continue;
        }
        let value = if joints::bindings_valid(world) {
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
