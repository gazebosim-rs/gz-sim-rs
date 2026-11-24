// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use gz_sim_core::{EntityId, UpdateInfo};
use gz_sim_scene::{BodyHandle, EntityKind, LocalPose, Name, Velocity, WorldPose};
use sha2::{Digest, Sha256};

pub fn proof(sim: &gz_sim_core::Simulation) -> [u8; 32] {
    let info = sim.world().resource::<UpdateInfo>();
    let mut hash = Sha256::new();
    for d in [info.start_time, info.sim_time, info.dt] {
        hash.update(d.as_nanos().to_le_bytes());
    }
    for n in [info.iteration, info.epoch] {
        hash.update(n.to_le_bytes());
    }
    hash.update([u8::from(info.paused), u8::from(sim.is_paused())]);
    let mut entities = sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<EntityKind>())
        .collect::<Vec<_>>();
    entities.sort_unstable_by_key(|e| e.get::<EntityId>().unwrap().get());
    for e in entities {
        hash.update(e.get::<EntityId>().unwrap().get().to_le_bytes());
        hash.update(e.get::<Name>().unwrap().0.as_bytes());
        if let Some(body) = e.get::<BodyHandle>() {
            hash.update(body.id().raw().to_le_bytes());
        }
        for pose in [
            *e.get::<WorldPose>().unwrap(),
            e.get::<LocalPose>().unwrap().0,
        ] {
            for value in pose.translation.into_iter().chain(pose.rotation) {
                hash.update(value.to_le_bytes());
            }
        }
        if let Some(v) = e.get::<Velocity>() {
            for value in v.linear.into_iter().chain(v.angular) {
                hash.update(value.to_le_bytes());
            }
        }
    }
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    for worker in &backend.pool_frame().unwrap().workers {
        hash.update(serde_json::to_vec(worker).unwrap());
    }
    hash.finalize().into()
}
