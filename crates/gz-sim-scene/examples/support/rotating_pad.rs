// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, deformable, load_world_with_sdf_step};
use std::error::Error;
pub const CABLE: &str = include_str!("../../../../examples/rotating_pad_cable.sdf");
pub const CLOTH: &str = include_str!("../../../../examples/rotating_pad_cloth.sdf");
pub struct Rotor {
    pub sim: Simulation,
    pub body: BodyId,
    pub link: u64,
}
impl Rotor {
    pub fn new(xml: &str) -> Result<Self, Box<dyn Error>> {
        let sim = load_world_with_sdf_step(
            xml,
            "rotating_pad.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let e = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::rotor::pad")
            })
            .ok_or("missing rotor")?;
        let body = e.get::<BodyHandle>().ok_or("missing body")?.id();
        let link = e.get::<EntityId>().ok_or("missing public ID")?.get();
        Ok(Self { sim, body, link })
    }
    pub fn state(&self) -> BodyState {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(self.body)
            .unwrap()
    }
    pub fn material(&self) -> &deformable::DeformableSample {
        deformable::bodies(&self.sim).next().unwrap().1
    }
    pub fn advance(&mut self, n: u64) -> Result<(), Box<dyn Error>> {
        self.sim
            .step_exact(n)
            .map_err(|e| format!("{e} at {:?}: {:?}", self.sim.elapsed(), self.sim.fault()).into())
    }
}
