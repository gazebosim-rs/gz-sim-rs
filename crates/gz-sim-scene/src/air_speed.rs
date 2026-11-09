// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One loaded Rapier air-speed sensor with optional pre-publication pressure noise.
use super::*;
use gz_sensors::air_speed::{
    AirSpeedInput, AirSpeedReading, InvalidAirSpeedInput, air_speed_unscaled, scale_air_speed,
};
use gz_sensors::{GaussianNoise, GaussianParameters, InvalidNoise, NoiseStream};

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum AirSpeedSampleError {
    #[error("invalid ideal air speed: {0}")]
    Ideal(InvalidAirSpeedInput),
    #[error("invalid air-speed pressure noise: {0}")]
    Noise(InvalidNoise),
}

#[derive(Clone)]
struct PressureNoise {
    channel: GaussianNoise,
    stream: NoiseStream,
    seed: [u8; 32],
    identity: u64,
}
impl PressureNoise {
    fn validate(config: &sdformat::Noise) -> Result<(), InvalidNoise> {
        if config.kind == sdformat::NoiseKind::None {
            return if *config == sdformat::Noise::default() {
                Ok(())
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
        .any(|x| !x.is_finite())
            || config.stddev < 0.0
            || config.bias_stddev < 0.0
            || config.dynamic_bias_stddev < 0.0
            || config.precision < 0.0
        {
            return Err(InvalidNoise);
        }
        Ok(())
    }
    fn new(
        config: &sdformat::Noise,
        seed: [u8; 32],
        identity: u64,
    ) -> Result<Option<Box<Self>>, InvalidNoise> {
        Self::validate(config)?;
        if config.kind == sdformat::NoiseKind::None {
            return Ok(None);
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
        })))
    }
    fn apply(&mut self, input: AirSpeedInput) -> Result<AirSpeedReading, AirSpeedSampleError> {
        let (pressure, temperature) =
            air_speed_unscaled(input).map_err(AirSpeedSampleError::Ideal)?;
        // Native Apply(double) defaults to dt=0, then assigns back to float
        // BEFORE multiplying by100. Neither bias nor quantization uses wire units.
        let noisy = self
            .channel
            .apply(f64::from(pressure), 0.0, &mut self.stream)
            .map_err(AirSpeedSampleError::Noise)?;
        scale_air_speed(noisy as f32, temperature)
            .map_err(|_| AirSpeedSampleError::Noise(InvalidNoise))
    }
    fn reset(&mut self) {
        self.stream = NoiseStream::new(self.seed, self.identity);
        self.channel = GaussianNoise::new(self.channel.parameters(), &mut self.stream)
            .expect("validated parameters and seed reproduce the finite initial bias");
    }
}

#[derive(Component, Clone)]
pub(super) struct AirSpeed {
    schedule: SampleSchedule,
    noise: Option<Box<PressureNoise>>,
    link: u64,
    offset: DVec3,
}

#[derive(Component, Debug)]
pub struct AirSpeedSample {
    pub time: Duration,
    pub epoch: u64,
    pub reading: Result<AirSpeedReading, AirSpeedSampleError>,
}

#[derive(Component, Debug, Clone, PartialEq)]
pub struct AirSpeedPublication {
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
pub fn set_air_speed_interest(sim: &mut Simulation, connected: impl IntoIterator<Item = Entity>) {
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
                entity.contains::<AirSpeedSample>()
                    && entity
                        .get::<EntityId>()
                        .is_none_or(|id| interest.0.binary_search(&id.get()).is_err())
            })
            .map(|entity| entity.id())
            .collect();
        for entity in stale {
            world.entity_mut(entity).remove::<AirSpeedSample>();
        }
    });
}

pub fn clear_air_speed_interest(sim: &mut Simulation) {
    sim.world_mut().remove_resource::<Interest>();
}

pub(super) struct Parent<'a> {
    pub link: &'a sdformat::Link,
    pub entity: Entity,
    pub topic: &'a str,
    pub id: u64,
}

struct Prepared {
    schedule: SampleSchedule,
    local: sdformat::Transform,
    world: sdformat::Transform,
}

fn prepare(
    sdf: &Sdf,
    model: &Model,
    link: &sdformat::Link,
    sensor: &sdformat::Sensor,
    world_name: &str,
    rapier: bool,
) -> Result<Prepared, LoadError> {
    let name = format!("{}::{}", link.scoped_name, sensor.name);
    require(
        sensor.air_speed().is_some()
            && sensor.effective_type() == Some(sdformat::SensorType::AirSpeed)
            && rapier,
        &name,
        "Rapier ideal air_speed",
    )?;
    require(
        !model.enable_wind && !link.properties.enable_wind,
        &name,
        "air_speed wind",
    )?;
    require(
        !sensor.visualize
            && !sensor.enable_metrics
            && sensor.plugins.is_empty()
            && sensor.extensions.is_empty(),
        &name,
        "air_speed metadata",
    )?;
    require(
        PressureNoise::validate(
            &sensor
                .air_speed()
                .expect("typed configuration checked")
                .pressure,
        )
        .is_ok(),
        &name,
        "air_speed pressure noise configuration",
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
        gz_sensors::air_speed::air_speed(AirSpeedInput {
            world_z: world.translation[2],
            velocity_sensor: [0.0; 3],
        })
        .is_ok(),
        &name,
        "air_speed atmosphere domain",
    )?;
    Ok(Prepared {
        schedule,
        local,
        world,
    })
}

/// Full typed family admission before Simulation construction. A nested or
/// joint-owned AirSpeed cannot hide behind a valid top-level family member.
pub(super) fn preflight(
    sdf: &Sdf,
    world: &sdformat::World,
    engine: PhysicsEngine,
) -> Result<(), LoadError> {
    fn selected(sensor: &sdformat::Sensor) -> bool {
        sensor.air_speed().is_some()
            || sensor.effective_type() == Some(sdformat::SensorType::AirSpeed)
    }
    fn visit(
        sdf: &Sdf,
        world: &sdformat::World,
        model: &Model,
        engine: PhysicsEngine,
        top: bool,
        count: &mut usize,
    ) -> Result<(), LoadError> {
        for joint in &model.joints {
            require(
                !joint.sensors.iter().any(selected),
                &model.scoped_name,
                "air_speed direct-link sensor",
            )?;
        }
        for link in &model.links {
            for sensor in &link.sensors {
                if !selected(sensor) {
                    continue;
                }
                *count += 1;
                require(*count == 1, &world.name, "multiple air_speeds")?;
                require(top, &model.scoped_name, "air_speed top-level model")?;
                require(
                    world.wind_linear_velocity == [0.0; 3],
                    &world.name,
                    "air_speed zero world wind",
                )?;
                let expected = sdformat::Atmosphere::default();
                let at = &world.atmosphere;
                require(
                    at.atmosphere_type == expected.atmosphere_type
                        && at.temperature_kelvin == expected.temperature_kelvin
                        && at.temperature_gradient == expected.temperature_gradient
                        && at.pressure == expected.pressure
                        && at.source.is_none()
                        && at.xml.is_none(),
                    &world.name,
                    "air_speed default atmosphere",
                )?;
                prepare(
                    sdf,
                    model,
                    link,
                    sensor,
                    &world.name,
                    engine == PhysicsEngine::Rapier,
                )?;
            }
        }
        for child in &model.models {
            visit(sdf, world, child, engine, false, count)?;
        }
        Ok(())
    }
    let mut count = 0;
    for model in &world.models {
        visit(sdf, world, model, engine, true, &mut count)?;
    }
    Ok(())
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
            .any(|entity| entity.contains::<AirSpeed>()),
        &name,
        "multiple air_speeds",
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
        backend.is_some_and(|backend| backend.engine() == "rapier"),
    )?;
    let entity_name = name
        .strip_prefix(&format!("{}::", context.world))
        .expect("scene names include their world scope")
        .to_owned();
    let publication = AirSpeedPublication {
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
                "{}/link/{}/sensor/{}/air_speed",
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
        .expect("spawned sensor owns a public identity")
        .get();
    let noise = PressureNoise::new(
        &sensor
            .air_speed()
            .expect("typed configuration checked")
            .pressure,
        context.noise_seed,
        identity,
    )
    .map_err(|_| LoadError::Unsupported {
        entity: name.clone(),
        feature: "unrepresentable air_speed noise",
    })?;
    sim.world_mut().entity_mut(entity).insert((
        AttachedPose,
        publication,
        AirSpeed {
            schedule,
            noise,
            link: parent.id,
            offset: DVec3::from_array(local.translation),
        },
    ));
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<AirSpeed>())
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
            .any(|entity| entity.contains::<AirSpeed>())
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
#[error("air_speed {entity:?} sampling deadline: {source}")]
struct AirSpeedScheduleFault {
    entity: Entity,
    source: schedule::ScheduleError,
}

fn sample(
    (time, interest): (Res<UpdateInfo>, Option<Res<Interest>>),
    fault: Res<StepFault>,
    identities: EntityLookup,
    bodies: Query<(&WorldPose, &Velocity)>,
    mut sensors: Query<(Entity, &EntityId, &WorldPose, &mut AirSpeed)>,
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
                fault.report(AirSpeedScheduleFault { entity, source });
                return;
            }
        }
        let parent = identities
            .get(sensor.link)
            .expect("scene owns air-speed parent identity");
        let (link_pose, velocity) = bodies.get(parent).expect("scene owns air-speed link state");
        let rotation = DQuat::from_array(pose.rotation);
        let link_rotation = DQuat::from_array(link_pose.rotation);
        let point = DVec3::from_array(velocity.linear)
            + DVec3::from_array(velocity.angular).cross(link_rotation * sensor.offset);
        let reading = if rotation.is_finite()
            && link_rotation.is_finite()
            && (rotation.length_squared() - 1.0).abs() <= 1e-12
            && (link_rotation.length_squared() - 1.0).abs() <= 1e-12
        {
            let input = AirSpeedInput {
                world_z: pose.translation[2],
                velocity_sensor: (rotation.inverse() * point).to_array(),
            };
            match &mut sensor.noise {
                Some(noise) => noise.apply(input),
                None => gz_sensors::air_speed::air_speed(input).map_err(AirSpeedSampleError::Ideal),
            }
        } else {
            Err(AirSpeedSampleError::Ideal(InvalidAirSpeedInput))
        };
        commands.entity(entity).insert(AirSpeedSample {
            time: time.sim_time,
            epoch: time.epoch,
            reading,
        });
    }
}

fn reset(mut sensors: Query<(Entity, &mut AirSpeed)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.schedule.reset();
        if let Some(noise) = &mut sensor.noise {
            noise.reset();
        }
        commands.entity(entity).remove::<AirSpeedSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/air_speed.rs"]
mod tests;
