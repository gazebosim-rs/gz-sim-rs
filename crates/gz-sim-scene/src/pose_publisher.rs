// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded flat-model PosePublisher; local transforms and unpaused native cadence.
use super::*;
use std::collections::BTreeSet;

#[derive(Component, Clone, Debug, PartialEq)]
pub struct PosePublication {
    pub model_name: String,
    pub topic: String,
    /// Stable lexical child-frame order is a Rust choice, not native hash order.
    pub poses: Vec<PoseFramePublication>,
}
#[derive(Clone, Debug, PartialEq)]
pub struct PoseFramePublication {
    /// Internal ownership identity; canonical Pose wire ID remains zero.
    pub identity: u64,
    pub frame_id: String,
    pub child_frame_id: String,
}
#[derive(Clone, Copy, Debug)]
pub struct PoseFrameValue {
    pub identity: u64,
    pub pose: WorldPose,
}
#[derive(Component, Clone, Debug)]
pub struct PoseSample {
    pub time: Duration,
    pub epoch: u64,
    pub revision: u64,
    pub poses: Vec<PoseFrameValue>,
}
#[derive(Component, Clone)]
pub(super) struct Publisher {
    owner: u64,
    publication: PosePublication,
    period: Duration,
    last: Duration,
    revision: u64,
}
#[derive(Resource)]
pub(super) struct Installed;
pub(super) struct Plan {
    model: bool,
    links: bool,
    topic: String,
    period: Duration,
}
pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::PosePublisher"
        && plugin.filename == "gz-sim-pose-publisher-system"
}
fn invalid(model: &Model, feature: &'static str) -> LoadError {
    LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    }
}
fn period(rate: f64) -> Option<Duration> {
    if !rate.is_finite() {
        return None;
    }
    if rate <= 0.0 {
        return Some(Duration::ZERO);
    }
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
    let mut plugins = model.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(None);
    };
    let bad = || invalid(model, "pose-publisher configuration");
    if plugins.next().is_some() || plugin.xml.len() > 65536 {
        return Err(bad());
    }
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
    let mut seen = BTreeSet::new();
    let mut model_pose = false;
    let mut links = true;
    let mut vector = false;
    let mut topic = None;
    let mut frequency = Duration::ZERO;
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
        let name = child.tag_name().name();
        if !seen.insert(name) {
            return Err(bad());
        }
        let text = child.text().unwrap_or_default().trim();
        match name {
            "topic" => {
                if !valid_topic(text) {
                    return Err(bad());
                }
                topic = Some(text.to_owned());
            }
            "update_frequency" => {
                frequency = period(text.parse().map_err(|_| bad())?).ok_or_else(bad)?
            }
            "publish_model_pose"
            | "publish_link_pose"
            | "use_pose_vector_msg"
            | "publish_nested_model_pose"
            | "publish_visual_pose"
            | "publish_collision_pose"
            | "publish_sensor_pose"
            | "static_publisher" => {
                let value = match text {
                    "true" => true,
                    "false" => false,
                    _ => return Err(bad()),
                };
                match name {
                    "publish_model_pose" => model_pose = value,
                    "publish_link_pose" => links = value,
                    "use_pose_vector_msg" => vector = value,
                    _ if value => return Err(bad()),
                    _ => {}
                }
            }
            _ => return Err(bad()),
        }
    }
    let (_, local) = model.scoped_name.split_once("::").ok_or_else(bad)?;
    let count = usize::from(model_pose) + if links { model.links.len() } else { 0 };
    if !vector || local.contains("::") || !(1..=128).contains(&count) {
        return Err(bad());
    }
    let topic = topic.unwrap_or_else(|| format!("/model/{}/pose", model.name));
    if !valid_topic(&topic) {
        return Err(bad());
    }
    Ok(Some(Plan {
        model: model_pose,
        links,
        topic,
        period: frequency,
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
                && model.models.is_empty()
                && model.raw_namespace.is_none(),
            &model.scoped_name,
            "pose-publisher requires a top-level flat Rapier model",
        )?;
        require(
            !sim.world()
                .iter_entities()
                .any(|e| e.contains::<Publisher>()),
            &model.scoped_name,
            "multiple pose-publisher models",
        )
    }
}
pub(super) fn build(
    _sdf: &Sdf,
    sim: &mut Simulation,
    model: &Model,
    entity: Entity,
    _backend: &PhysicsBackend,
    plan: &Plan,
) -> Result<(), LoadError> {
    let owner = sim
        .world()
        .get::<EntityId>(entity)
        .ok_or_else(|| invalid(model, "missing pose-publisher model"))?
        .get();
    let (world, local) = model
        .scoped_name
        .split_once("::")
        .ok_or_else(|| invalid(model, "pose-publisher model scope"))?;
    let mut poses = Vec::new();
    if plan.model {
        poses.push(PoseFramePublication {
            identity: owner,
            frame_id: world.into(),
            child_frame_id: local.into(),
        });
    }
    if plan.links {
        for link in &model.links {
            let node = sim
                .world()
                .iter_entities()
                .find(|node| node.get::<Name>().is_some_and(|n| n.0 == link.scoped_name))
                .ok_or_else(|| invalid(model, "missing pose-publisher link"))?;
            require(
                node.get::<EntityKind>() == Some(&EntityKind::Link)
                    && node.get::<ChildOf>().map(ChildOf::parent) == Some(entity),
                &link.scoped_name,
                "pose-publisher link ownership",
            )?;
            let identity = node
                .get::<EntityId>()
                .ok_or_else(|| invalid(model, "missing pose-publisher identity"))?
                .get();
            poses.push(PoseFramePublication {
                identity,
                frame_id: local.into(),
                child_frame_id: format!("{local}::{}", link.name),
            });
        }
    }
    poses.sort_by(|a, b| a.child_frame_id.cmp(&b.child_frame_id));
    let publication = PosePublication {
        model_name: local.into(),
        topic: plan.topic.clone(),
        poses,
    };
    sim.world_mut().entity_mut(entity).insert((
        publication.clone(),
        Publisher {
            owner,
            publication,
            period: plan.period,
            last: Duration::ZERO,
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
#[error("pose-publisher ownership, local transform or revision is invalid")]
struct PublicationFault;
fn reading(
    world: &World,
    entity: Entity,
    publisher: &Publisher,
    time: UpdateInfo,
) -> Option<PoseSample> {
    if world.get::<EntityId>(entity)?.get() != publisher.owner
        || world.get::<EntityKind>(entity) != Some(&EntityKind::Model)
        || world.get::<PosePublication>(entity)? != &publisher.publication
    {
        return None;
    }
    let mut poses = Vec::with_capacity(publisher.publication.poses.len());
    for frame in &publisher.publication.poses {
        let node = gz_sim_core::entity_by_id(world, frame.identity)?;
        if frame.identity == publisher.owner {
            if node != entity {
                return None;
            }
        } else if world.get::<EntityKind>(node) != Some(&EntityKind::Link)
            || world.get::<ChildOf>(node)?.parent() != entity
        {
            return None;
        }
        let pose = world.get::<LocalPose>(node)?.0;
        if !pose
            .translation
            .iter()
            .chain(pose.rotation.iter())
            .all(|v| v.is_finite())
            || (DQuat::from_array(pose.rotation).length_squared() - 1.0).abs() > 1e-12
        {
            return None;
        }
        poses.push(PoseFrameValue {
            identity: frame.identity,
            pose,
        });
    }
    Some(PoseSample {
        time: time.sim_time,
        epoch: time.epoch,
        revision: publisher.revision.checked_add(1)?,
        poses,
    })
}
fn sample(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.paused {
        return;
    }
    let entities: Vec<_> = world
        .iter_entities()
        .filter(|e| e.contains::<Publisher>())
        .map(|e| e.id())
        .collect();
    for entity in entities {
        let publisher = world.get::<Publisher>(entity).expect("selected publisher");
        if time
            .sim_time
            .checked_sub(publisher.last)
            .is_some_and(|d| !d.is_zero() && d < publisher.period)
        {
            continue;
        }
        let Some(value) = reading(world, entity, publisher, time) else {
            world.entity_mut(entity).remove::<PoseSample>();
            world.resource::<StepFault>().report(PublicationFault);
            return;
        };
        let mut publisher = world
            .get_mut::<Publisher>(entity)
            .expect("selected publisher");
        publisher.last = time.sim_time;
        publisher.revision = value.revision;
        world.entity_mut(entity).insert(value);
    }
}
fn reset(mut commands: Commands, publishers: Query<Entity, With<Publisher>>) {
    for entity in &publishers {
        commands.entity(entity).remove::<PoseSample>();
    }
}
#[cfg(test)]
#[path = "../tests/private/pose_publisher.rs"]
mod tests;
