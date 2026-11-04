// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Additive opt-in measured scan; scene geometry and control clocks stay separate.
use super::*;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{EntityKind, LidarPublication, LidarSample};

#[derive(Serialize)]
struct Envelope {
    version: u8,
    kind: &'static str,
    world_id: u32,
    epoch: u64,
    sim_time_ns: u64,
    scan: Option<Scan>,
}
#[derive(Serialize)]
struct Scan {
    sensor_id: u32,
    acquired_ns: u64,
    sensor_position: [f64; 3],
    sensor_orientation: [f64; 4],
    angle_min: f64,
    angle_max: f64,
    angle_step: f64,
    range_min: f64,
    range_max: f64,
    ranges: Vec<Option<f64>>,
}
fn invalid() -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidData,
        "invalid bounded measured lidar scan",
    )
}
fn id(value: u64) -> io::Result<u32> {
    u32::try_from(value)
        .ok()
        .filter(|value| *value != 0)
        .ok_or_else(invalid)
}

pub(super) fn encode(sim: &Simulation) -> io::Result<String> {
    let mut worlds = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.get::<EntityKind>() == Some(&EntityKind::World));
    let world = worlds.next().ok_or_else(invalid)?;
    if worlds.next().is_some() || sim.is_faulted() {
        return Err(invalid());
    }
    let world_id = id(world
        .get::<EntityId>()
        .filter(|id| entity_by_id(sim.world(), id.get()) == Some(world.id()))
        .ok_or_else(invalid)?
        .get())?;
    let now = u64::try_from(sim.elapsed().as_nanos()).map_err(|_| invalid())?;
    let mut sensors = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.contains::<LidarPublication>());
    let mut scan = None;
    if let Some(entity) = sensors.next() {
        if sensors.next().is_some() {
            return Err(invalid());
        }
        let config = entity
            .get::<LidarPublication>()
            .expect("selected lidar publication");
        if let Some(sample) = entity
            .get::<LidarSample>()
            .filter(|sample| sample.epoch == sim.epoch())
        {
            let acquired_ns = u64::try_from(sample.time.as_nanos()).map_err(|_| invalid())?;
            let norm: f64 = sample
                .world_pose
                .rotation
                .iter()
                .map(|value| value * value)
                .sum();
            let metadata = &config.config;
            if acquired_ns > now
                || !(2..=128).contains(&sample.ranges.len())
                || sample.ranges.len() != metadata.samples
                || !sample
                    .world_pose
                    .translation
                    .iter()
                    .chain(sample.world_pose.rotation.iter())
                    .all(|value| value.is_finite())
                || (norm - 1.0).abs() > 1e-10
                || ![
                    metadata.angle_min,
                    metadata.angle_max,
                    config.angle_step,
                    metadata.range_min,
                    metadata.range_max,
                ]
                .iter()
                .all(|value| value.is_finite())
                || metadata.angle_max <= metadata.angle_min
                || metadata.angle_max - metadata.angle_min > std::f64::consts::TAU
                || config.angle_step <= 0.0
                || metadata.range_min < 0.0
                || metadata.range_max <= metadata.range_min
            {
                return Err(invalid());
            }
            let expected_step =
                (metadata.angle_max - metadata.angle_min) / (metadata.samples - 1) as f64;
            if !expected_step.is_finite()
                || expected_step <= 0.0
                || (config.angle_step - expected_step).abs()
                    > 4.0 * f64::EPSILON * expected_step.abs()
                || sample
                    .world_pose
                    .translation
                    .iter()
                    .any(|value| value.abs() + metadata.range_max > f64::from(f32::MAX))
            {
                return Err(invalid());
            }
            let mut previous = None;
            for index in 0..metadata.samples {
                let angle = if index + 1 == metadata.samples {
                    metadata.angle_max
                } else {
                    metadata.angle_min + index as f64 * config.angle_step
                };
                if !angle.is_finite() || previous.is_some_and(|last| angle <= last) {
                    return Err(invalid());
                }
                previous = Some(angle);
            }
            let ranges = sample
                .ranges
                .iter()
                .map(|&value| {
                    if value == f64::INFINITY {
                        Ok(None)
                    } else if value.is_finite()
                        && value >= metadata.range_min
                        && value <= metadata.range_max
                    {
                        Ok(Some(value))
                    } else {
                        Err(invalid())
                    }
                })
                .collect::<io::Result<Vec<_>>>()?;
            scan = Some(Scan {
                sensor_id: id(entity
                    .get::<EntityId>()
                    .filter(|id| entity_by_id(sim.world(), id.get()) == Some(entity.id()))
                    .ok_or_else(invalid)?
                    .get())?,
                acquired_ns,
                sensor_position: sample.world_pose.translation,
                sensor_orientation: sample.world_pose.rotation,
                angle_min: metadata.angle_min,
                angle_max: metadata.angle_max,
                angle_step: config.angle_step,
                range_min: metadata.range_min,
                range_max: metadata.range_max,
                ranges,
            });
        }
    }
    let encoded = serde_json::to_string(&Envelope {
        version: 1,
        kind: "scan",
        world_id,
        epoch: sim.epoch(),
        sim_time_ns: now,
        scan,
    })
    .map_err(io::Error::other)?;
    if encoded.len() > 16384 {
        return Err(invalid());
    }
    Ok(encoded)
}
