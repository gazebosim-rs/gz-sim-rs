// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One loaded Rapier air_pressure with optional native scalar Gaussian noise.
use super::*;
use gz_sensors::air_pressure::{AirPressureInput, AirPressureReading, InvalidAirPressureInput};
use gz_sensors::{GaussianNoise, GaussianParameters, InvalidNoise, NoiseStream};

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum AirPressureSampleError {
    #[error("invalid ideal air pressure: {0}")]
    Ideal(InvalidAirPressureInput),
    #[error("invalid air-pressure noise: {0}")]
    Noise(InvalidNoise),
    #[error("noisy air pressure must remain positive")]
    NonpositiveNoisyPressure,
}

#[derive(Clone)]
struct PressureNoise {
    channel: GaussianNoise,
    stream: NoiseStream,
    seed: [u8; 32],
    identity: u64,
    variance: f64,
}

impl PressureNoise {
    fn new(
        config: &sdformat::Noise,
        seed: [u8; 32],
        identity: u64,
    ) -> Result<Option<Box<Self>>, InvalidNoise> {
        if config.kind == sdformat::NoiseKind::None {
            return if *config == sdformat::Noise::default() {
                Ok(None)
            } else {
                Err(InvalidNoise)
            };
        }
        if [
            config.mean,
            config.stddev,
            config.bias_mean,
            config.bias_stddev,
            config.dynamic_bias_stddev,
            config.dynamic_bias_correlation_time,
            config.precision,
        ]
        .iter()
        .any(|value| !value.is_finite())
            || config.stddev < 0.0
            || config.bias_stddev < 0.0
            || config.dynamic_bias_stddev < 0.0
            || config.precision < 0.0
        {
            return Err(InvalidNoise);
        }
        let mut stream = NoiseStream::new(seed, identity);
        let channel = GaussianNoise::new(
            GaussianParameters {
                mean: config.mean,
                stddev: config.stddev,
                bias_mean: config.bias_mean,
                bias_stddev: config.bias_stddev,
                dynamic_bias_stddev: config.dynamic_bias_stddev,
                dynamic_bias_correlation_time: config.dynamic_bias_correlation_time,
                precision: config.precision,
            },
            &mut stream,
        )?;
        Ok(Some(Box::new(Self {
            channel,
            stream,
            seed,
            identity,
            // Pinned AirPressureSensor publishes sqrt(StdDev), not stddev squared.
            variance: config.stddev.sqrt(),
        })))
    }

    fn apply(
        &mut self,
        mut reading: AirPressureReading,
    ) -> Result<AirPressureReading, AirPressureSampleError> {
        // Native AirPressureSensor calls Apply(pressure) with its default dt=0.
        reading.pressure = self
            .channel
            .apply(reading.pressure, 0.0, &mut self.stream)
            .map_err(AirPressureSampleError::Noise)?;
        if reading.pressure <= 0.0 {
            return Err(AirPressureSampleError::NonpositiveNoisyPressure);
        }
        reading.variance = self.variance;
        Ok(reading)
    }

    fn reset(&mut self) {
        self.stream = NoiseStream::new(self.seed, self.identity);
        self.channel = GaussianNoise::new(self.channel.parameters(), &mut self.stream)
            .expect("same validated parameters and seed reproduce the finite initial bias");
    }
}

#[derive(Component, Clone)]
pub(super) struct AirPressure {
    schedule: SampleSchedule,
    noise: Option<Box<PressureNoise>>,
}

#[derive(Component, Debug)]
pub struct AirPressureSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<AirPressureReading, AirPressureSampleError>,
}

#[derive(Component, Debug, Clone, PartialEq)]
pub struct AirPressurePublication {
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
pub fn set_air_pressure_interest(
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
                entity.contains::<AirPressureSample>()
                    && entity
                        .get::<EntityId>()
                        .is_none_or(|id| interest.0.binary_search(&id.get()).is_err())
            })
            .map(|entity| entity.id())
            .collect();
        for entity in stale {
            world.entity_mut(entity).remove::<AirPressureSample>();
        }
    });
}

pub fn clear_air_pressure_interest(sim: &mut Simulation) {
    sim.world_mut().remove_resource::<Interest>();
}

pub(super) struct Parent<'a> {
    pub link: &'a sdformat::Link,
    pub entity: Entity,
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
        sensor.effective_type() == Some(sdformat::SensorType::AirPressure)
            && backend.is_some_and(|backend| backend.engine() == "rapier"),
        &name,
        "Rapier ideal air_pressure",
    )?;
    require(
        !sim.world()
            .iter_entities()
            .any(|entity| entity.contains::<AirPressure>()),
        &name,
        "multiple air_pressures",
    )?;
    require(
        !sensor.visualize
            && !sensor.enable_metrics
            && sensor.plugins.is_empty()
            && sensor.extensions.is_empty(),
        &name,
        "air_pressure metadata",
    )?;
    let config = sensor
        .air_pressure()
        .expect("caller selected air_pressure configuration");
    require(
        config.reference_altitude == 0.0,
        &name,
        "air_pressure reference altitude",
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
    require(
        gz_sensors::air_pressure::air_pressure(AirPressureInput {
            world_z: world.translation[2],
        })
        .is_ok(),
        &name,
        "air_pressure atmosphere domain",
    )?;
    let entity_name = name
        .strip_prefix(&format!("{}::", context.world))
        .expect("scene names include their world scope")
        .to_owned();
    let publication = AirPressurePublication {
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
                "{}/link/{}/sensor/{}/air_pressure",
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
    let identity = sim
        .world()
        .get::<EntityId>(entity)
        .expect("spawn_node assigns a public sensor identity")
        .get();
    let noise =
        PressureNoise::new(&config.pressure, context.noise_seed, identity).map_err(|_| {
            LoadError::Unsupported {
                entity: name.clone(),
                feature: "unrepresentable air_pressure noise",
            }
        })?;
    sim.world_mut().entity_mut(entity).insert((
        AttachedPose,
        publication,
        AirPressure { schedule, noise },
    ));
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<AirPressure>())
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
            .any(|entity| entity.contains::<AirPressure>())
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
#[error("air_pressure {entity:?} sampling deadline: {source}")]
struct AirPressureScheduleFault {
    entity: Entity,
    source: schedule::ScheduleError,
}

fn sample(
    (time, interest): (Res<UpdateInfo>, Option<Res<Interest>>),
    fault: Res<StepFault>,
    mut sensors: Query<(Entity, &EntityId, &WorldPose, &mut AirPressure)>,
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
                fault.report(AirPressureScheduleFault { entity, source });
                return;
            }
        }
        let mut reading = gz_sensors::air_pressure::air_pressure(AirPressureInput {
            world_z: pose.translation[2],
        })
        .map_err(AirPressureSampleError::Ideal);
        if let (Ok(ideal), Some(noise)) = (&reading, &mut sensor.noise) {
            reading = noise.apply(*ideal);
        }
        commands.entity(entity).insert(AirPressureSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading,
        });
    }
}

fn reset(mut sensors: Query<(Entity, &mut AirPressure)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.schedule.reset();
        if let Some(noise) = &mut sensor.noise {
            noise.reset();
        }
        commands.entity(entity).remove::<AirPressureSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/air_pressure.rs"]
mod tests;
