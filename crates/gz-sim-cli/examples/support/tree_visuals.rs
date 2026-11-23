// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{EntityKind, Name, WorldPose};
use serde_json::{Value, json};
use std::collections::BTreeMap;
use std::error::Error;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub fn poses(sim: &Simulation) -> Result<BTreeMap<u64, Value>> {
    let mut poses = BTreeMap::new();
    for entity in sim
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Visual))
    {
        let id = entity
            .get::<EntityId>()
            .ok_or("missing visual identity")?
            .get();
        let name = &entity.get::<Name>().ok_or("missing visual name")?.0;
        let world = entity.get::<WorldPose>().ok_or("missing visual pose")?;
        poses.insert(
            id,
            json!({"id":id,"name":name,"position":world.translation,"orientation":world.rotation}),
        );
    }
    Ok(poses)
}
