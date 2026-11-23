// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions,
    gpu_scene::{
        self, GpuError, GpuSceneObservations, NativeHandoffRefusal,
        PairSolver::{self, NativeRapier, SpecializedGpu},
        batch_boundary,
    },
    load_world_on_adapter,
};
use std::error::Error;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub const SDF: &str = include_str!("../../../../examples/native_gpu_support.sdf");
pub const IDS: [u64; 3] = [9, 31, 80];
pub const END: u64 = 1100;
pub const LAUNCH: u64 = 400;
pub const KICK: [f64; 3] = [0.31, -0.06, 3.0];
pub fn load(adapter: &str) -> Result<Simulation> {
    Ok(load_world_on_adapter(
        SDF,
        "native_gpu_support.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(adapter),
    )?)
}
#[derive(Debug, Clone, Copy)]
pub enum Event {
    Kick(u64),
    Moved(u64, PairSolver),
    Refused(u64, NativeHandoffRefusal),
}
pub struct Drive {
    links: [u64; 3],
    switched: [bool; 2],
}
impl Drive {
    pub fn new(sim: &Simulation) -> Self {
        Self {
            links: IDS.map(|id| {
                let name = format!("default::ball_{id}::body");
                sim.world()
                    .iter_entities()
                    .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                    .unwrap()
                    .get::<EntityId>()
                    .unwrap()
                    .get()
            }),
            switched: [false; 2],
        }
    }
    pub fn boundary(&mut self, sim: &mut Simulation) -> Result<Vec<Event>> {
        let at = batch_boundary(sim);
        let t = at.tick;
        if !matches!(t, 32 | 85 | 235 | LAUNCH | 940)
            && !(t > LAUNCH && self.switched.contains(&false))
        {
            return Ok(vec![]);
        }
        let before = sim
            .world()
            .resource::<GpuSceneObservations>()
            .environments()
            .to_vec();
        let mut events = vec![];
        for (i, id) in IDS.into_iter().enumerate() {
            if t == LAUNCH {
                gpu_scene::queue_link_impulse(sim, at, self.links[i], KICK)?;
                events.push(Event::Kick(id));
            }
            let loaded = matches!(t, 235 | LAUNCH | 940);
            let after_release = t > LAUNCH && i < 2 && !self.switched[i];
            if !loaded && (i == 2 || !matches!(t, 32 | 85) && !after_release) {
                continue;
            }
            let target = match gpu_scene::pair_solver(sim, id)? {
                NativeRapier => SpecializedGpu,
                SpecializedGpu => NativeRapier,
            };
            match gpu_scene::set_pair_solver(sim, at, &[id], target) {
                Ok(change) => {
                    assert!(!loaded, "loaded support was released at tick {t}");
                    assert_eq!(change.moved, 1);
                    if after_release {
                        self.switched[i] = true;
                    }
                    events.push(Event::Moved(id, target));
                }
                Err(GpuError::NativeHandoff {
                    reason:
                        reason @ (NativeHandoffRefusal::ContactHistory
                        | NativeHandoffRefusal::SweptClearance),
                    ..
                }) if loaded || after_release => events.push(Event::Refused(id, reason)),
                Err(e) => return Err(e.into()),
            }
        }
        let after = sim
            .world()
            .resource::<GpuSceneObservations>()
            .environments();
        for (a, b) in before.iter().zip(after) {
            assert_eq!(a.id, b.id);
            assert!(
                a.state.values().iter().map(|v| v.to_bits()).eq(b
                    .state
                    .values()
                    .iter()
                    .map(|v| v.to_bits()))
            );
        }
        assert_eq!(batch_boundary(sim), at);
        Ok(events)
    }
}
