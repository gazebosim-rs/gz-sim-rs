// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Error, time};
use bevy_ecs::world::World;
use gz_msgs::{
    msgs::{Header, Pose, PoseV},
    quaternion, vector3,
};
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{EntityKind, LocalName, LocalPose, WorldPose};
use std::time::{Duration, Instant};

pub(super) struct PublicationRate {
    period: Duration,
    last_attempt: Option<Instant>,
}

impl Default for PublicationRate {
    fn default() -> Self {
        // Native compares integer elapsed ns against double 1e9/60, requiring this ceiling.
        Self {
            period: Duration::from_nanos(16_666_667),
            last_attempt: None,
        }
    }
}

impl PublicationRate {
    pub(super) fn period(&self) -> Duration {
        self.period
    }

    pub(super) fn set_period(&mut self, period: Duration) {
        self.period = period;
        self.last_attempt = None;
    }

    pub(super) fn ready(&self, now: Instant) -> bool {
        self.last_attempt
            .is_none_or(|last| now.saturating_duration_since(last) >= self.period)
    }

    pub(super) fn attempted(&mut self, now: Instant) {
        self.last_attempt = Some(now);
    }
}

fn record(id: u64, name: &str, pose: WorldPose) -> Result<Pose, Error> {
    let id = u32::try_from(id).map_err(|_| Error::PoseIdRange)?;
    if id == 0 {
        return Err(Error::PoseIdRange);
    }
    let norm: f64 = pose.rotation.iter().map(|x| x * x).sum();
    if pose
        .translation
        .iter()
        .chain(&pose.rotation)
        .any(|x| !x.is_finite())
        || (norm - 1.0).abs() > 1e-12
    {
        return Err(Error::PoseData);
    }
    Ok(Pose {
        id,
        name: name.into(),
        position: Some(vector3(pose.translation)),
        orientation: Some(quaternion(pose.rotation)),
        ..Default::default()
    })
}

pub(crate) fn packet(world: &World, stamp: Duration) -> Result<PoseV, Error> {
    let mut records = Vec::new();
    for entity in world.iter_entities() {
        let order = match entity.get::<EntityKind>() {
            Some(EntityKind::Model) => 0,
            Some(EntityKind::Link) => 1,
            Some(EntityKind::Visual) => 2,
            _ => continue,
        };
        let id = entity.get::<EntityId>().ok_or(Error::PoseData)?.get();
        if entity_by_id(world, id) != Some(entity.id()) {
            return Err(Error::PoseData);
        }
        let name = &entity.get::<LocalName>().ok_or(Error::PoseData)?.0;
        let pose = entity.get::<LocalPose>().ok_or(Error::PoseData)?.0;
        records.push((order, record(id, name, pose)?));
    }
    // Native packets group models, links and visuals. Stable IDs order each group.
    records.sort_unstable_by_key(|(order, pose)| (*order, pose.id));
    Ok(PoseV {
        header: Some(Header {
            stamp: Some(time(stamp.as_nanos() as i128)?),
            data: Vec::new(),
        }),
        pose: records.into_iter().map(|(_, pose)| pose).collect(),
    })
}

#[cfg(test)]
#[path = "../tests/private/poses.rs"]
mod tests;
