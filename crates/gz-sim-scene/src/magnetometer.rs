// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One loaded, ideal Rapier magnetometer; no transport or native library ownership.
use super::*;
use gz_sensors::magnetometer::{InvalidMagnetometerInput, MagnetometerInput, MagnetometerReading};

#[derive(Component, Clone)]
pub(super) struct Magnetometer {
    schedule: SampleSchedule,
    field_world: [f64; 3],
}

#[derive(Component, Debug)]
pub struct MagnetometerSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<MagnetometerReading, InvalidMagnetometerInput>,
}

#[derive(Component, Debug, Clone, PartialEq)]
pub struct MagnetometerPublication {
    pub entity_name: String,
    pub frame_id: String,
    pub topic: String,
}

#[derive(Resource)]
pub(super) struct Installed;
#[derive(Resource, Default)]
struct Interest(Vec<u64>);

/// No interest resource selects local/manual acquisition; an explicit empty
/// selection owns no sample and advances no sensor deadline.
pub fn set_magnetometer_interest(
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
                .filter_map(|entity| world.get::<EntityId>(entity).map(EntityId::get)),
        );
        interest.0.sort_unstable();
        interest.0.dedup();
        let stale: Vec<_> = world
            .iter_entities()
            .filter(|entity| {
                entity.contains::<MagnetometerSample>()
                    && entity
                        .get::<EntityId>()
                        .is_none_or(|id| interest.0.binary_search(&id.get()).is_err())
            })
            .map(|entity| entity.id())
            .collect();
        for entity in stale {
            world.entity_mut(entity).remove::<MagnetometerSample>();
        }
    });
}

pub fn clear_magnetometer_interest(sim: &mut Simulation) {
    sim.world_mut().remove_resource::<Interest>();
}

pub(super) struct Parent<'a> {
    pub link: &'a sdformat::Link,
    pub entity: Entity,
    pub topic: &'a str,
    pub field_world: [f64; 3],
}

struct Prepared {
    schedule: SampleSchedule,
    local: sdformat::Transform,
    world: sdformat::Transform,
}

// Shared by pre-Simulation admission and spawning; a typed consumer never
// authorizes a nondefault world field on the strength of its family alone.
fn prepare(
    sdf: &Sdf,
    model: &Model,
    link: &sdformat::Link,
    sensor: &sdformat::Sensor,
    world_name: &str,
    field_world: [f64; 3],
    rapier: bool,
) -> Result<Prepared, LoadError> {
    let name = format!("{}::{}", link.scoped_name, sensor.name);
    require(
        sensor.magnetometer().is_some()
            && sensor.effective_type() == Some(sdformat::SensorType::Magnetometer)
            && rapier,
        &name,
        "Rapier ideal magnetometer",
    )?;
    require(
        !sensor.visualize
            && !sensor.enable_metrics
            && sensor.plugins.is_empty()
            && sensor.extensions.is_empty(),
        &name,
        "magnetometer metadata",
    )?;
    let config = sensor.magnetometer().expect("typed magnetometer checked");
    require(
        config
            .noise
            .iter()
            .all(|noise| *noise == sdformat::Noise::default()),
        &name,
        "magnetometer noise",
    )?;
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
    let world = sdf.resolve_pose(
        &sensor.pose,
        &model.scoped_name,
        &link.scoped_name,
        world_name,
    )?;
    require(
        gz_sensors::magnetometer::magnetometer(MagnetometerInput {
            world_orientation: world.rotation,
            field_world,
        })
        .is_ok(),
        &name,
        "magnetometer field domain",
    )?;
    Ok(Prepared {
        schedule,
        local,
        world,
    })
}

pub(super) fn preflight(
    sdf: &Sdf,
    world: &sdformat::World,
    engine: PhysicsEngine,
) -> Result<bool, LoadError> {
    fn visit(
        sdf: &Sdf,
        world: &sdformat::World,
        model: &Model,
        engine: PhysicsEngine,
        count: &mut usize,
    ) -> Result<(), LoadError> {
        for link in &model.links {
            for sensor in &link.sensors {
                if sensor.magnetometer().is_none()
                    && sensor.effective_type() != Some(sdformat::SensorType::Magnetometer)
                {
                    continue;
                }
                *count += 1;
                require(*count == 1, &world.name, "multiple magnetometers")?;
                require(
                    world.spherical_coordinates.is_none(),
                    &world.name,
                    "magnetometer spherical coordinates",
                )?;
                prepare(
                    sdf,
                    model,
                    link,
                    sensor,
                    &world.name,
                    world.magnetic_field,
                    engine == PhysicsEngine::Rapier,
                )?;
            }
        }
        for child in &model.models {
            visit(sdf, world, child, engine, count)?;
        }
        Ok(())
    }
    let mut count = 0;
    for model in &world.models {
        visit(sdf, world, model, engine, &mut count)?;
    }
    Ok(count == 1)
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
            .any(|entity| entity.contains::<Magnetometer>()),
        &name,
        "multiple magnetometers",
    )?;
    let Prepared {
        schedule,
        local,
        world,
    } = prepare(
        sdf,
        model,
        parent.link,
        sensor,
        context.world,
        parent.field_world,
        backend.is_some_and(|backend| backend.engine() == "rapier"),
    )?;
    let entity_name = name
        .strip_prefix(&format!("{}::", context.world))
        .expect("scene names include their world scope")
        .to_owned();
    let publication = MagnetometerPublication {
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
                "{}/link/{}/sensor/{}/magnetometer",
                parent.topic, parent.link.name, sensor.name
            )
        } else {
            sensor.topic.clone()
        },
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
        Magnetometer {
            schedule,
            field_world: parent.field_world,
        },
    ));
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Magnetometer>())
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
            .any(|entity| entity.contains::<Magnetometer>())
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
#[error("magnetometer {entity:?} sampling deadline: {source}")]
struct MagnetometerScheduleFault {
    entity: Entity,
    source: schedule::ScheduleError,
}

fn sample(
    (time, interest): (Res<UpdateInfo>, Option<Res<Interest>>),
    fault: Res<StepFault>,
    mut sensors: Query<(Entity, &EntityId, &WorldPose, &mut Magnetometer)>,
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
                fault.report(MagnetometerScheduleFault { entity, source });
                return;
            }
        }
        let reading = gz_sensors::magnetometer::magnetometer(MagnetometerInput {
            world_orientation: pose.rotation,
            field_world: sensor.field_world,
        });
        commands.entity(entity).insert(MagnetometerSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading,
        });
    }
}

fn reset(mut sensors: Query<(Entity, &mut Magnetometer)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.schedule.reset();
        commands.entity(entity).remove::<MagnetometerSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/magnetometer.rs"]
mod tests;
