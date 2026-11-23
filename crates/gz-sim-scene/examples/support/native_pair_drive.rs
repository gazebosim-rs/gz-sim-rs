// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared normal-scene controls for the native/GPU handoff qualification and recording.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions,
    gpu_scene::{
        self, EnvironmentState, GpuSceneObservations, NativeHandoffRefusal, PairSolver,
        PairSolver::{NativeRapier, SpecializedGpu},
        batch_boundary,
    },
    load_world_on_adapter,
};
use std::error::Error;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub const SDF: &str = include_str!("../../../../examples/native_gpu_spheres.sdf");
pub const IDS: [u64; 3] = [9, 31, 80];
pub const INPUT: [f64; 3] = [0.015, -0.05, 0.02];
pub fn load(adapter: &str) -> Result<Simulation> {
    Ok(load_world_on_adapter(
        SDF,
        "native_gpu_spheres.sdf",
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Impulse(u64),
    Moved(u64, PairSolver),
    ContactRefusal(u64),
    ClearanceRefusal(u64),
}
pub struct Drive {
    a: [u64; 3],
    pub impact: [Option<u64>; 3],
    after: [bool; 2],
}
impl Drive {
    pub fn new(sim: &Simulation) -> Self {
        Self {
            a: IDS.map(|id| {
                let name = format!("default::pair_{id}_a::body");
                sim.world()
                    .iter_entities()
                    .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                    .unwrap()
                    .get::<EntityId>()
                    .unwrap()
                    .get()
            }),
            impact: [None; 3],
            after: [false; 2],
        }
    }
    pub fn boundary(&mut self, sim: &mut Simulation) -> Result<Vec<Event>> {
        let at = batch_boundary(sim);
        let tick = at.tick;
        let mut events = vec![];
        let before = sim
            .world()
            .resource::<GpuSceneObservations>()
            .environments()
            .to_vec();
        if tick == 130 {
            for (i, id) in IDS.iter().copied().enumerate() {
                gpu_scene::queue_link_impulse(sim, at, self.a[i], INPUT)?;
                events.push(Event::Impulse(id));
            }
        }
        if matches!(tick, 40 | 130 | 565) {
            let targets: &[_] = match tick {
                40 => &[(9, SpecializedGpu), (31, NativeRapier)],
                130 => &[(9, NativeRapier), (31, SpecializedGpu)],
                _ => &[(9, NativeRapier), (31, SpecializedGpu)],
            };
            for &(id, target) in targets {
                assert_eq!(gpu_scene::set_pair_solver(sim, at, &[id], target)?.moved, 1);
                events.push(Event::Moved(id, target));
            }
        }
        for (i, id) in IDS.iter().copied().enumerate() {
            let EnvironmentState::SpherePair(p) = before[i].state else {
                unreachable!()
            };
            if p.normal_impulse > 1e-8 && self.impact[i].is_none() {
                self.impact[i] = Some(tick);
                let source = gpu_scene::pair_solver(sim, id)?;
                let target = if source == NativeRapier {
                    SpecializedGpu
                } else {
                    NativeRapier
                };
                match gpu_scene::set_pair_solver(sim, at, &[id], target) {
                    Err(gpu_scene::GpuError::NativeHandoff {
                        reason: NativeHandoffRefusal::ContactHistory,
                        ..
                    }) => events.push(Event::ContactRefusal(id)),
                    other => {
                        return Err(format!(
                            "expected actual contact history refusal at {tick} for {id}: {other:?}"
                        )
                        .into());
                    }
                }
            }
            if i < 2 && self.impact[i].is_some_and(|t| tick > t) && !self.after[i] {
                let target = if i == 0 { SpecializedGpu } else { NativeRapier };
                match gpu_scene::set_pair_solver(sim, at, &[id], target) {
                    Ok(change) => {
                        assert_eq!(change.moved, 1);
                        self.after[i] = true;
                        events.push(Event::Moved(id, target));
                    }
                    Err(gpu_scene::GpuError::NativeHandoff {
                        reason: NativeHandoffRefusal::ContactHistory,
                        ..
                    }) => events.push(Event::ContactRefusal(id)),
                    Err(gpu_scene::GpuError::NativeHandoff {
                        reason: NativeHandoffRefusal::SweptClearance,
                        ..
                    }) => events.push(Event::ClearanceRefusal(id)),
                    Err(e) => return Err(e.into()),
                }
            }
        }
        let after = sim
            .world()
            .resource::<GpuSceneObservations>()
            .environments();
        same_frame_bits(&before, after);
        assert_eq!(batch_boundary(sim), at);
        Ok(events)
    }
}

pub fn same_frame_bits(a: &[gpu_scene::Environment], b: &[gpu_scene::Environment]) {
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(b) {
        assert_eq!(a.id, b.id);
        assert_eq!(a.state.profile(), b.state.profile());
        assert!(
            a.state.values().iter().map(|v| v.to_bits()).eq(b
                .state
                .values()
                .iter()
                .map(|v| v.to_bits())),
            "differing physical record for environment {}",
            a.id
        );
    }
}
