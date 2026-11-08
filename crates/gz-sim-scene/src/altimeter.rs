// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One loaded, ideal Rapier altimeter; no transport or native library ownership.
use super::*;
use gz_sensors::altimeter::{AltimeterInput, AltimeterReading, InvalidAltimeterInput};

#[derive(Component, Clone)]
pub(super) struct Altimeter {
    link: u64,
    offset: DVec3,
    reference: f64,
    schedule: SampleSchedule,
}

#[derive(Component, Debug)]
pub struct AltimeterSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<AltimeterReading, InvalidAltimeterInput>,
}

#[derive(Component, Debug, Clone, PartialEq)]
pub struct AltimeterPublication {
    pub entity_name: String,
    pub frame_id: String,
    pub topic: String,
    pub vertical_reference: f64,
}

#[derive(Resource)]
pub(super) struct Installed;
#[derive(Resource, Default)]
struct Interest(Vec<u64>);

/// No interest resource selects local/manual acquisition; an explicit empty
/// selection owns no sample and advances no sensor deadline.
pub fn set_altimeter_interest(sim: &mut Simulation, connected: impl IntoIterator<Item = Entity>) {
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
                .filter_map(|entity| world.get::<EntityId>(entity).map(EntityId::get)),
        );
        interest.0.sort_unstable();
        interest.0.dedup();
        let stale: Vec<_> = world
            .iter_entities()
            .filter(|entity| {
                entity.contains::<AltimeterSample>()
                    && entity
                        .get::<EntityId>()
                        .is_none_or(|id| interest.0.binary_search(&id.get()).is_err())
            })
            .map(|entity| entity.id())
            .collect();
        for entity in stale {
            world.entity_mut(entity).remove::<AltimeterSample>();
        }
    });
}

pub fn clear_altimeter_interest(sim: &mut Simulation) {
    sim.world_mut().remove_resource::<Interest>();
}

pub(super) struct Parent<'a> {
    pub link: &'a sdformat::Link,
    pub entity: Entity,
    pub id: u64,
    pub topic: &'a str,
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
        sensor.effective_type() == Some(sdformat::SensorType::Altimeter)
            && backend.is_some_and(|backend| backend.engine() == "rapier"),
        &name,
        "Rapier ideal altimeter",
    )?;
    require(
        !sim.world()
            .iter_entities()
            .any(|entity| entity.contains::<Altimeter>()),
        &name,
        "multiple altimeters",
    )?;
    require(
        !sensor.visualize
            && !sensor.enable_metrics
            && sensor.plugins.is_empty()
            && sensor.extensions.is_empty(),
        &name,
        "altimeter metadata",
    )?;
    let config = sensor
        .altimeter()
        .expect("caller selected altimeter configuration");
    require(
        config.vertical_position == sdformat::Noise::default()
            && config.vertical_velocity == sdformat::Noise::default(),
        &name,
        "altimeter noise",
    )?;
    let schedule = SampleSchedule::new(sensor.update_rate).map_err(|_| LoadError::Unsupported {
        entity: name.clone(),
        feature: "unrepresentable sensor rate",
    })?;
    let local = sdf.resolve_pose(
        &sensor.pose,
        &model.scoped_name,
        &parent.link.scoped_name,
        &parent.link.scoped_name,
    )?;
    let world = sdf.resolve_pose(
        &sensor.pose,
        &model.scoped_name,
        &parent.link.scoped_name,
        context.world,
    )?;
    let entity_name = name
        .strip_prefix(&format!("{}::", context.world))
        .expect("scene names include their world scope")
        .to_owned();
    let publication = AltimeterPublication {
        frame_id: if sensor.frame_id.is_empty() {
            sensor
                .gz_frame_id
                .clone()
                .unwrap_or_else(|| entity_name.clone())
        } else {
            sensor.frame_id.clone()
        },
        entity_name,
        topic: if sensor.topic.is_empty() {
            format!(
                "{}/link/{}/sensor/{}/altimeter",
                parent.topic, parent.link.name, sensor.name
            )
        } else {
            sensor.topic.clone()
        },
        vertical_reference: world.translation[2],
    };
    let entity = spawn_node(
        sim,
        Some(parent.entity),
        &name,
        EntityKind::Sensor,
        local,
        world,
    )?;
    sim.world_mut().entity_mut(entity).insert((
        AttachedPose,
        publication,
        Altimeter {
            link: parent.id,
            offset: DVec3::from_array(local.translation),
            reference: world.translation[2],
            schedule,
        },
    ));
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Altimeter>())
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
            .any(|entity| entity.contains::<Altimeter>())
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
#[error("altimeter {entity:?} sampling deadline: {source}")]
struct AltimeterScheduleFault {
    entity: Entity,
    source: schedule::ScheduleError,
}

fn sample(
    (time, interest): (Res<UpdateInfo>, Option<Res<Interest>>),
    fault: Res<StepFault>,
    identities: EntityLookup,
    bodies: Query<(&WorldPose, &Velocity)>,
    mut sensors: Query<(Entity, &EntityId, &WorldPose, &mut Altimeter)>,
    mut commands: Commands,
) {
    if time.dt.is_zero() {
        return;
    }
    for (entity, id, pose, mut sensor) in &mut sensors {
        if interest
            .as_ref()
            .is_some_and(|interest| interest.0.binary_search(&id.get()).is_err())
        {
            continue;
        }
        match sensor.schedule.take_sample(time.sim_time, false) {
            Ok(true) => {}
            Ok(false) => continue,
            Err(source) => {
                fault.report(AltimeterScheduleFault { entity, source });
                return;
            }
        }
        let parent = identities
            .get(sensor.link)
            .expect("scene owns altimeter link identity");
        let (link_pose, velocity) = bodies.get(parent).expect("scene owns altimeter link state");
        // Velocity is at the link origin (COM shift already removed by sync_bodies).
        // Native Physics::LinkFrameDataAtOffset adds omega cross rotated offset.
        let offset = DQuat::from_array(link_pose.rotation) * sensor.offset;
        let point_velocity =
            DVec3::from_array(velocity.linear) + DVec3::from_array(velocity.angular).cross(offset);
        let reading = gz_sensors::altimeter::altimeter(AltimeterInput {
            world_z: pose.translation[2],
            world_velocity_z: point_velocity.z,
            vertical_reference: sensor.reference,
        });
        commands.entity(entity).insert(AltimeterSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading,
        });
    }
}

fn reset(mut sensors: Query<(Entity, &mut Altimeter)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.schedule.reset();
        commands.entity(entity).remove::<AltimeterSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/altimeter.rs"]
mod tests;
