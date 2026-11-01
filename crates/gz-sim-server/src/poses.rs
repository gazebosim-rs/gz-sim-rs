// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Error, quaternion, time, vector3};
use bevy_ecs::{
    hierarchy::{ChildOf, Children},
    world::{EntityRef, World},
};
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_proto::msgs::{Header, Pose, PoseV};
use gz_sim_scene::{EntityKind, LocalName, LocalPose, VisualProperties, WorldLight, WorldPose};
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

fn native_light_visual(world: &World, visual: &EntityRef<'_>) -> bool {
    if visual.get::<EntityKind>() != Some(&EntityKind::LightVisual) {
        return false;
    }
    let Some(parent) = visual.get::<ChildOf>() else {
        return false;
    };
    let Ok(light) = world.get_entity(parent.parent()) else {
        return false;
    };
    if light.get::<EntityKind>() != Some(&EntityKind::Light) || !light.contains::<WorldLight>() {
        return false;
    }
    let Some(name) = light.get::<LocalName>() else {
        return false;
    };
    let Some(child_name) = visual.get::<LocalName>() else {
        return false;
    };
    let Some(local) = visual.get::<LocalPose>() else {
        return false;
    };
    let Some(properties) = visual.get::<VisualProperties>() else {
        return false;
    };
    child_name.0.strip_suffix("Visual") == Some(name.0.as_str())
        && local.0.translation == [0.0; 3]
        && local.0.rotation == [0.0, 0.0, 0.0, 1.0]
        && !properties.0.cast_shadows
        && properties.0.transparency == 0.0
        && light
            .get::<Children>()
            .is_some_and(|children| children.len() == 1 && children[0] == visual.id())
}

fn complete_light(world: &World, light: &EntityRef<'_>) -> bool {
    let Some(parent) = light.get::<ChildOf>() else {
        return false;
    };
    if !world
        .get_entity(parent.parent())
        .is_ok_and(|parent| parent.get::<EntityKind>() == Some(&EntityKind::World))
    {
        return false;
    }
    let Some(children) = light.get::<Children>() else {
        return false;
    };
    children.len() == 1
        && world
            .get_entity(children[0])
            .is_ok_and(|child| native_light_visual(world, &child))
}

pub(crate) fn packet(world: &World, stamp: Duration) -> Result<PoseV, Error> {
    let mut records = Vec::new();
    for entity in world.iter_entities() {
        let kind = entity.get::<EntityKind>();
        if entity.contains::<WorldLight>() != (kind == Some(&EntityKind::Light)) {
            return Err(Error::PoseLighting);
        }
        if (kind == Some(&EntityKind::Light) && !complete_light(world, &entity))
            || (kind == Some(&EntityKind::LightVisual) && !native_light_visual(world, &entity))
        {
            return Err(Error::PoseLighting);
        }
        let order = match kind {
            Some(EntityKind::Model) => 0,
            Some(EntityKind::Link) => 1,
            Some(EntityKind::Visual | EntityKind::LightVisual) => 2,
            Some(EntityKind::Light) => 3,
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
    // Native groups models, links, visuals and lights. Within-group ID ordering
    // is our deterministic choice, not a native traversal-order guarantee.
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
