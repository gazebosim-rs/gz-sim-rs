// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded, demand-selected planar CPU lidar. No renderer or transport ownership.
use super::*;
use gz_sensors::lidar::{PlanarLidar, PlanarLidarConfig};
use gz_sensors::{GaussianNoise, GaussianParameters, InvalidNoise, NoiseStream};

#[derive(Clone)]
struct LidarNoise {
    channel: GaussianNoise,
    stream: NoiseStream,
    seed: [u8; 32],
    identity: u64,
}

impl LidarNoise {
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
        if config.kind != sdformat::NoiseKind::Gaussian
            || !config.mean.is_finite()
            || !config.stddev.is_finite()
            || config.stddev < 0.0
            || config.bias_mean != 0.0
            || config.bias_stddev != 0.0
            || config.dynamic_bias_stddev != 0.0
            || config.dynamic_bias_correlation_time != 0.0
            || config.precision != 0.0
        {
            return Err(InvalidNoise);
        }
        // CpuLidarSensor uses math::equal's inclusive 1e-6 activation boundary.
        if config.mean.abs() <= 1e-6 && config.stddev <= 1e-6 {
            return Ok(None);
        }
        let mut stream = NoiseStream::new(seed, identity);
        let channel = GaussianNoise::new(
            GaussianParameters {
                mean: config.mean,
                stddev: config.stddev,
                ..Default::default()
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

    fn range(&mut self, hit: Option<f64>, maximum: f64) -> Result<f64, String> {
        let Some(hit) = hit else {
            return Ok(f64::INFINITY);
        };
        if !hit.is_finite() || hit < 0.0 || hit > maximum {
            return Err("invalid raw CPU-lidar hit".into());
        }
        // Native finite beams consume this one stream in index order, dt=0.
        // GaussianNoise rejects nonfinite output BEFORE any clipping can hide it.
        let noisy = self
            .channel
            .apply(hit, 0.0, &mut self.stream)
            .map_err(|error| error.to_string())?;
        Ok(noisy.clamp(0.0, maximum))
    }

    fn reset(&mut self) {
        self.stream = NoiseStream::new(self.seed, self.identity);
        self.channel = GaussianNoise::new(self.channel.parameters(), &mut self.stream)
            .expect("same seed and validated parameters reconstruct the finite initial bias");
    }
}

#[derive(Component, Clone)]
pub(super) struct Lidar {
    link: u64,
    model: PlanarLidar,
    schedule: SampleSchedule,
    noise: Option<Box<LidarNoise>>,
}

#[derive(Component, Debug)]
pub struct LidarSample {
    pub time: Duration,
    pub epoch: u64,
    /// Actual acquisition frame, not the later publication frame.
    pub world_pose: WorldPose,
    /// Continuous metres, positive infinity for a ray with no return.
    pub ranges: Vec<f64>,
}

#[derive(Component, Debug, Clone, PartialEq)]
pub struct LidarPublication {
    pub entity_name: String,
    pub frame_id: String,
    pub topic: String,
    pub config: PlanarLidarConfig,
    pub angle_step: f64,
}

#[derive(Resource)]
pub(super) struct Installed;
#[derive(Resource, Default)]
struct Interest(Vec<u64>);

/// Select connected consumers. No selection resource means local/manual acquisition;
/// an explicitly empty selection performs no ray queries and owns no stale scan.
pub fn set_lidar_interest(sim: &mut Simulation, connected: impl IntoIterator<Item = Entity>) {
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
                entity.contains::<LidarSample>()
                    && entity
                        .get::<EntityId>()
                        .is_none_or(|id| interest.0.binary_search(&id.get()).is_err())
            })
            .map(|entity| entity.id())
            .collect();
        for entity in stale {
            world.entity_mut(entity).remove::<LidarSample>();
        }
    });
}

pub fn clear_lidar_interest(sim: &mut Simulation) {
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
        sensor.effective_type() == Some(sdformat::SensorType::Lidar),
        &name,
        "CPU lidar type",
    )?;
    require(
        backend.is_some_and(|backend| backend.engine() == "rapier"),
        &name,
        "non-IMU sensors",
    )?;
    require(
        !sim.world()
            .iter_entities()
            .any(|entity| entity.contains::<Lidar>()),
        &name,
        "multiple CPU lidars",
    )?;
    let config = sensor.lidar().expect("caller selected lidar configuration");
    require(
        config.horizontal.resolution == 1.0
            && config.vertical.samples == 1
            && config.vertical.resolution == 1.0
            && config.vertical.min_angle == 0.0
            && config.vertical.max_angle == 0.0,
        &name,
        "non-planar or interpolated lidar",
    )?;
    require(
        config.range_resolution.is_finite()
            && config.range_resolution >= 0.0
            && config.visibility_mask == u32::MAX,
        &name,
        "lidar resolution, noise or visibility mask",
    )?;
    require(
        config.noise.kind == sdformat::NoiseKind::None || config.range_min == 0.0,
        &name,
        "Gaussian CPU lidar requires zero minimum range",
    )?;
    let ray_model = PlanarLidar::new(PlanarLidarConfig {
        samples: config.horizontal.samples as usize,
        angle_min: config.horizontal.min_angle,
        angle_max: config.horizontal.max_angle,
        range_min: config.range_min,
        range_max: config.range_max,
    })
    .map_err(|_| LoadError::Unsupported {
        entity: name.clone(),
        feature: "invalid bounded planar lidar",
    })?;
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
        .expect("loaded scene scope")
        .to_owned();
    let frame_id = if sensor.frame_id.is_empty() {
        sensor
            .gz_frame_id
            .clone()
            .unwrap_or_else(|| entity_name.clone())
    } else {
        sensor.frame_id.clone()
    };
    let topic = if sensor.topic.is_empty() {
        format!(
            "{}/link/{}/sensor/{}/scan",
            parent.topic, parent.link.name, sensor.name
        )
    } else {
        sensor.topic.clone()
    };
    let publication = LidarPublication {
        entity_name,
        frame_id,
        topic,
        config: *ray_model.config(),
        angle_step: ray_model.angle_step(),
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
        .expect("spawn_node assigns public sensor identity")
        .get();
    let noise = LidarNoise::new(&config.noise, context.noise_seed, identity).map_err(|_| {
        LoadError::Unsupported {
            entity: name.clone(),
            feature: "unsupported CPU lidar noise",
        }
    })?;
    sim.world_mut().entity_mut(entity).insert((
        AttachedPose,
        publication,
        Lidar {
            link: parent.id,
            model: ray_model,
            schedule,
            noise,
        },
    ));
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Lidar>())
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
            .any(|entity| entity.contains::<Lidar>())
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
#[error("lidar {entity:?}: {message}")]
struct LidarFault {
    entity: Entity,
    message: String,
}

fn sample(
    (time, interest): (Res<UpdateInfo>, Option<Res<Interest>>),
    backend: Res<PhysicsBackend>,
    fault: Res<StepFault>,
    identities: EntityLookup,
    bodies: Query<&BodyHandle>,
    mut sensors: Query<(Entity, &EntityId, &WorldPose, &mut Lidar)>,
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
        let result = (|| -> Result<Option<Vec<f64>>, String> {
            if !sensor
                .schedule
                .take_sample(time.sim_time, false)
                .map_err(|error| error.to_string())?
            {
                return Ok(None);
            }
            let parent = identities.get(sensor.link).ok_or("missing lidar link")?;
            let body = bodies.get(parent).map_err(|error| error.to_string())?;
            let rotation = DQuat::from_array(pose.rotation);
            let Lidar { model, noise, .. } = &mut *sensor;
            let mut ranges = Vec::with_capacity(model.config().samples);
            for direction in model.directions() {
                let direction = (rotation * DVec3::from_array(*direction)).to_array();
                let hit = backend
                    .cast_ray(
                        pose.translation,
                        direction,
                        model.config().range_max,
                        Some(body.id()),
                    )
                    .map_err(|error| error.to_string())?;
                let range = match noise.as_mut() {
                    Some(noise) => noise.range(hit, model.config().range_max)?,
                    None => model.range(hit).map_err(|error| error.to_string())?,
                };
                ranges.push(range);
            }
            Ok(Some(ranges))
        })();
        match result {
            Ok(Some(ranges)) => {
                commands.entity(entity).insert(LidarSample {
                    time: time.sim_time,
                    epoch: time.epoch,
                    world_pose: *pose,
                    ranges,
                });
            }
            Ok(None) => {}
            Err(message) => {
                commands.entity(entity).remove::<LidarSample>();
                fault.report(LidarFault { entity, message });
                return;
            }
        }
    }
}

fn reset(mut sensors: Query<(Entity, &mut Lidar)>, mut commands: Commands) {
    for (entity, mut sensor) in &mut sensors {
        sensor.schedule.reset();
        if let Some(noise) = &mut sensor.noise {
            noise.reset();
        }
        commands.entity(entity).remove::<LidarSample>();
    }
}

#[cfg(test)]
#[path = "../tests/private/lidar.rs"]
mod tests;
