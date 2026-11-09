// Copyright (C) 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One bounded, loaded model-origin LogicalCamera. Native logical sensor/system
//! sources: gz-sensors8ddf4c17 and gz-sim71c60db3 (OSRF, Apache-2.0).
use super::*;
use gz_sensors::logical_camera::{
    InvalidLogicalCameraInput, LogicalCameraConfig, LogicalCameraModel, LogicalCameraPose,
    LogicalCameraReading, logical_camera, valid_name, valid_pose,
};
use std::collections::BTreeSet;

#[derive(Component, Debug, Clone, PartialEq)]
pub struct LogicalCameraPublication {
    pub entity_name: String,
    pub frame_id: String,
    pub topic: String,
    pub frustum_topic: String,
    pub config: LogicalCameraConfig,
}
#[derive(Component, Debug)]
pub struct LogicalCameraSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<LogicalCameraReading, InvalidLogicalCameraInput>,
}
#[derive(Component, Clone)]
pub(super) struct LogicalCamera {
    schedule: SampleSchedule,
    owner: u64,
    link: u64,
    world: String,
    publication: LogicalCameraPublication,
}
#[derive(Resource)]
pub(super) struct Installed;
#[derive(Resource, Default)]
struct Interest(Vec<u64>);

pub fn set_logical_camera_interest(
    sim: &mut Simulation,
    connected: impl IntoIterator<Item = Entity>,
) {
    let world = sim.world_mut();
    if !world.contains_resource::<Installed>() {
        world.remove_resource::<Interest>();
        return;
    }
    world.init_resource::<Interest>();
    world.resource_scope(|world, mut interest: Mut<Interest>| {
        interest.0.clear();
        interest.0.extend(
            connected
                .into_iter()
                .filter_map(|e| world.get::<EntityId>(e).map(EntityId::get)),
        );
        interest.0.sort_unstable();
        interest.0.dedup();
        let stale: Vec<_> = world
            .iter_entities()
            .filter(|e| {
                e.contains::<LogicalCameraSample>()
                    && e.get::<EntityId>()
                        .is_none_or(|id| interest.0.binary_search(&id.get()).is_err())
            })
            .map(|e| e.id())
            .collect();
        for e in stale {
            world.entity_mut(e).remove::<LogicalCameraSample>();
        }
    });
}
pub fn clear_logical_camera_interest(sim: &mut Simulation) {
    sim.world_mut().remove_resource::<Interest>();
}
fn selected(sensor: &sdformat::Sensor) -> bool {
    sensor.kind == "logical_camera"
        || sensor.effective_type() == Some(sdformat::SensorType::LogicalCamera)
}
fn topic_valid(topic: &str) -> bool {
    topic.starts_with('/')
        && topic.len() > 1
        && topic.len() <= 65535
        && !topic
            .bytes()
            .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
        && !["~", "//", "@", ":="].iter().any(|v| topic.contains(v))
}
fn config(sensor: &sdformat::Sensor) -> Result<LogicalCameraConfig, LoadError> {
    let fail = || LoadError::Unsupported {
        entity: sensor.name.clone(),
        feature: "bounded logical_camera configuration",
    };
    if sensor.kind != "logical_camera"
        || sensor.effective_type() != Some(sdformat::SensorType::LogicalCamera)
        || !matches!(&sensor.config, sdformat::SensorConfig::Unconfigured)
        || sensor.configuration_xml.len() > 65536
        || sensor.configuration_xml.contains("<![CDATA[")
    {
        return Err(fail());
    }
    let doc = roxmltree::Document::parse(&sensor.configuration_xml).map_err(|_| fail())?;
    let root = doc.root_element();
    if root.tag_name().name() != "logical_camera"
        || root.tag_name().namespace().is_some()
        || root.attributes().len() != 0
        || root.namespaces().len() != 0
    {
        return Err(fail());
    }
    let mut values = BTreeMap::new();
    for node in root.children() {
        if node.is_text() {
            if node.text().is_some_and(|s| !s.trim().is_empty()) {
                return Err(fail());
            }
            continue;
        }
        if node.is_comment() {
            continue;
        }
        if !node.is_element()
            || node.tag_name().namespace().is_some()
            || node.attributes().len() != 0
            || node.namespaces().len() != 0
            || node.children().any(|n| !n.is_text())
        {
            return Err(fail());
        }
        let key = node.tag_name().name();
        if !["near", "far", "horizontal_fov", "aspect_ratio"].contains(&key) {
            return Err(fail());
        }
        let raw = node.text().ok_or_else(fail)?.trim();
        let number = raw.parse::<f64>().map_err(|_| fail())?;
        if !number.is_finite() || values.insert(key, number).is_some() {
            return Err(fail());
        }
    }
    if values.len() != 4 {
        return Err(fail());
    }
    let config = LogicalCameraConfig {
        near: values["near"],
        far: values["far"],
        horizontal_fov: values["horizontal_fov"],
        aspect_ratio: values["aspect_ratio"],
    };
    config.validate().map_err(|_| fail())?;
    Ok(config)
}
fn camera_pose(pose: WorldPose) -> LogicalCameraPose {
    LogicalCameraPose {
        translation: pose.translation,
        rotation: pose.rotation,
    }
}
fn transform_pose(pose: sdformat::Transform) -> LogicalCameraPose {
    LogicalCameraPose {
        translation: pose.translation,
        rotation: pose.rotation,
    }
}
pub(super) struct Parent<'a> {
    pub link: &'a sdformat::Link,
    pub entity: Entity,
    pub topic: &'a str,
    pub id: u64,
}
struct Prepared {
    config: LogicalCameraConfig,
    schedule: SampleSchedule,
    local: sdformat::Transform,
    world: sdformat::Transform,
}
fn prepare(
    sdf: &Sdf,
    model: &Model,
    link: &sdformat::Link,
    sensor: &sdformat::Sensor,
    world: &str,
    rapier: bool,
) -> Result<Prepared, LoadError> {
    let name = format!("{}::{}", link.scoped_name, sensor.name);
    require(rapier, &name, "Rapier ideal logical_camera")?;
    require(
        !sensor.visualize
            && !sensor.enable_metrics
            && sensor.plugins.is_empty()
            && sensor.extensions.is_empty(),
        &name,
        "logical_camera metadata",
    )?;
    let base = if sensor.topic.is_empty() {
        format!(
            "/world/{world}/model/{}/link/{}/sensor/{}/logical_camera",
            model.name, link.name, sensor.name
        )
    } else {
        sensor.topic.clone()
    };
    require(
        topic_valid(&base) && topic_valid(&format!("{base}/frustum")),
        &name,
        "logical_camera two valid bounded topics",
    )?;
    let frame = if sensor.frame_id.is_empty() {
        sensor.gz_frame_id.as_deref().unwrap_or(&name)
    } else {
        &sensor.frame_id
    };
    require(
        !frame.is_empty() && frame.len() <= 65535,
        &name,
        "logical_camera frame id",
    )?;
    let config = config(sensor)?;
    let schedule = SampleSchedule::new(sensor.update_rate).map_err(|_| LoadError::Unsupported {
        entity: name.clone(),
        feature: "unrepresentable sensor rate",
    })?;
    let local = sdf.resolve_pose(
        &sensor.pose,
        &model.scoped_name,
        &link.scoped_name,
        &link.scoped_name,
    )?;
    let pose = sdf.resolve_pose(&sensor.pose, &model.scoped_name, &link.scoped_name, world)?;
    require(
        logical_camera(config, transform_pose(pose), &[]).is_ok(),
        &name,
        "logical_camera finite pose and derived planes",
    )?;
    Ok(Prepared {
        config,
        schedule,
        local,
        world: pose,
    })
}
pub(super) fn preflight(
    sdf: &Sdf,
    world: &sdformat::World,
    engine: PhysicsEngine,
) -> Result<(), LoadError> {
    fn count(model: &Model) -> usize {
        model
            .links
            .iter()
            .map(|l| l.sensors.iter().filter(|s| selected(s)).count())
            .sum::<usize>()
            + model
                .joints
                .iter()
                .map(|j| j.sensors.iter().filter(|s| selected(s)).count())
                .sum::<usize>()
            + model.models.iter().map(count).sum::<usize>()
    }
    let cameras = world.models.iter().map(count).sum::<usize>();
    if cameras == 0 {
        return Ok(());
    }
    require(cameras == 1, &world.name, "one loaded logical_camera")?;
    require(
        world.models.len() <= 128,
        &world.name,
        "logical_camera model count",
    )?;
    let mut names = BTreeSet::new();
    let mut found = false;
    for model in &world.models {
        require(
            model.models.is_empty()
                && valid_name(&model.name)
                && names.insert(model.name.clone())
                && model.scoped_name == format!("{}::{}", world.name, model.name),
            &model.scoped_name,
            "logical_camera flat unique model names",
        )?;
        require(
            valid_pose(transform_pose(
                sdf.pose_of(&model.scoped_name, &world.name)?,
            )),
            &model.scoped_name,
            "logical_camera finite model pose",
        )?;
        require(
            !model.joints.iter().any(|j| j.sensors.iter().any(selected)),
            &model.scoped_name,
            "logical_camera direct-link sensor",
        )?;
        for link in &model.links {
            for sensor in &link.sensors {
                if selected(sensor) {
                    prepare(
                        sdf,
                        model,
                        link,
                        sensor,
                        &world.name,
                        engine == PhysicsEngine::Rapier,
                    )?;
                    found = true;
                }
            }
        }
    }
    require(
        found,
        &world.name,
        "logical_camera top-level direct-link sensor",
    )
}
pub(super) fn spawn(
    sdf: &Sdf,
    model: &Model,
    sensor: &sdformat::Sensor,
    context: &SceneContext<'_>,
    sim: &mut Simulation,
    backend: Option<&PhysicsBackend>,
    parent: Parent<'_>,
) -> Result<(), LoadError> {
    let name = format!("{}::{}", parent.link.scoped_name, sensor.name);
    require(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<LogicalCamera>()),
        &name,
        "one loaded logical_camera",
    )?;
    require(
        sim.world()
            .get::<EntityId>(parent.entity)
            .is_some_and(|id| id.get() == parent.id),
        &name,
        "logical_camera parent identity",
    )?;
    let prepared = prepare(
        sdf,
        model,
        parent.link,
        sensor,
        context.world,
        backend.is_some_and(|b| b.engine() == "rapier"),
    )?;
    let entity_name = name
        .strip_prefix(&format!("{}::", context.world))
        .expect("world-scoped name")
        .to_owned();
    let topic = if sensor.topic.is_empty() {
        format!(
            "{}/link/{}/sensor/{}/logical_camera",
            parent.topic, parent.link.name, sensor.name
        )
    } else {
        sensor.topic.clone()
    };
    let frustum_topic = format!("{topic}/frustum");
    require(
        topic_valid(&topic) && topic_valid(&frustum_topic),
        &name,
        "logical_camera two valid bounded topics",
    )?;
    let publication = LogicalCameraPublication {
        entity_name: entity_name.clone(),
        frame_id: if sensor.frame_id.is_empty() {
            sensor.gz_frame_id.clone().unwrap_or(entity_name)
        } else {
            sensor.frame_id.clone()
        },
        topic,
        frustum_topic,
        config: prepared.config,
    };
    require(
        !publication.frame_id.is_empty() && publication.frame_id.len() <= 65535,
        &name,
        "logical_camera frame id",
    )?;
    let entity = spawn_node(
        sim,
        Some(parent.entity),
        &name,
        EntityKind::Sensor,
        prepared.local,
        prepared.world,
    )?;
    let owner = sim
        .world()
        .get::<EntityId>(entity)
        .expect("spawn_node identity")
        .get();
    sim.world_mut().entity_mut(entity).insert((
        AttachedPose,
        publication.clone(),
        LogicalCamera {
            schedule: prepared.schedule,
            owner,
            link: parent.id,
            world: context.world.into(),
            publication,
        },
    ));
    Ok(())
}
pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<LogicalCamera>())
    {
        return;
    }
    sim.add_systems(Phase::PostStep, sample);
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
            .any(|e| e.contains::<LogicalCamera>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PostStep, sample)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<Installed>();
    sim.world_mut().remove_resource::<Interest>();
    Ok(())
}
#[derive(Debug, thiserror::Error)]
#[error("logical_camera {entity:?} sampling deadline: {source}")]
struct LogicalCameraScheduleFault {
    entity: Entity,
    source: schedule::ScheduleError,
}
fn reading(
    world: &World,
    entity: Entity,
    camera: &LogicalCamera,
) -> Result<LogicalCameraReading, InvalidLogicalCameraInput> {
    let fail = InvalidLogicalCameraInput;
    if world.get::<EntityId>(entity).map(EntityId::get) != Some(camera.owner)
        || gz_sim_core::entity_by_id(world, camera.owner) != Some(entity)
        || world.get::<EntityKind>(entity) != Some(&EntityKind::Sensor)
        || world.get::<LogicalCameraPublication>(entity) != Some(&camera.publication)
    {
        return Err(fail);
    }
    let parent = gz_sim_core::entity_by_id(world, camera.link).ok_or(fail)?;
    if world.get::<ChildOf>(entity).map(ChildOf::parent) != Some(parent)
        || world.get::<EntityKind>(parent) != Some(&EntityKind::Link)
    {
        return Err(fail);
    }
    let pose = camera_pose(*world.get::<WorldPose>(entity).ok_or(fail)?);
    let mut models = Vec::new();
    for e in world.iter_entities().filter(|e| {
        e.get::<EntityKind>() == Some(&EntityKind::Model) && !e.contains::<PendingResetRemoval>()
    }) {
        if models.len() >= 128 {
            return Err(fail);
        }
        let id = e.get::<EntityId>().ok_or(fail)?.get();
        if gz_sim_core::entity_by_id(world, id) != Some(e.id()) {
            return Err(fail);
        }
        let parent = e.get::<ChildOf>().ok_or(fail)?.parent();
        if world.get::<EntityKind>(parent) != Some(&EntityKind::World)
            || world.get::<Name>(parent).map(|n| n.0.as_str()) != Some(camera.world.as_str())
        {
            return Err(fail);
        }
        let name = e
            .get::<Name>()
            .ok_or(fail)?
            .0
            .strip_prefix(&(camera.world.clone() + "::"))
            .ok_or(fail)?;
        models.push(LogicalCameraModel {
            name: name.into(),
            pose: camera_pose(*e.get::<WorldPose>().ok_or(fail)?),
        });
    }
    logical_camera(camera.publication.config, pose, &models)
}
fn sample(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.dt.is_zero() {
        return;
    }
    let cameras: Vec<_> = world
        .iter_entities()
        .filter(|e| e.contains::<LogicalCamera>())
        .map(|e| e.id())
        .collect();
    for entity in cameras {
        let id = world
            .get::<LogicalCamera>(entity)
            .expect("selected camera")
            .owner;
        if world
            .get_resource::<Interest>()
            .is_some_and(|i| i.0.binary_search(&id).is_err())
        {
            continue;
        }
        let due = world
            .get_mut::<LogicalCamera>(entity)
            .expect("selected camera")
            .schedule
            .take_sample(time.sim_time, false);
        match due {
            Ok(false) => continue,
            Err(source) => {
                world.entity_mut(entity).remove::<LogicalCameraSample>();
                world
                    .resource::<StepFault>()
                    .report(LogicalCameraScheduleFault { entity, source });
                return;
            }
            Ok(true) => {}
        }
        let value = reading(
            world,
            entity,
            world.get::<LogicalCamera>(entity).expect("selected camera"),
        );
        world.entity_mut(entity).insert(LogicalCameraSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading: value,
        });
    }
}
fn reset(mut cameras: Query<(Entity, &mut LogicalCamera)>, mut commands: Commands) {
    for (entity, mut camera) in &mut cameras {
        camera.schedule.reset();
        commands.entity(entity).remove::<LogicalCameraSample>();
    }
}
#[cfg(test)]
#[path = "../tests/private/logical_camera.rs"]
mod tests;
