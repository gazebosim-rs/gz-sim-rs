// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "../support/authored_hinge.rs"]
mod authored_hinge;
use authored_hinge::AuthoredHinge;

pub(super) struct Reference(AuthoredHinge);

impl Reference {
    pub(super) fn load(spec: &MovingSpec) -> Result<Self> {
        let world = Self(AuthoredHinge::load(SDF, SOURCE)?);
        world.observe(spec, 0)?;
        Ok(world)
    }
    pub(super) fn advance(&mut self) -> Result<()> {
        self.0.advance()
    }
    pub(super) fn queue(&mut self, id: u64, effort: f64) -> Result<()> {
        self.0.queue(id, effort)
    }
    pub(super) fn observe(&self, spec: &MovingSpec, tick: u64) -> Result<MovingObservation> {
        let ([parent, child], hinge) = self.0.hinge(&spec.hinge, tick)?;
        let (ball, sphere) = self.0.body(spec.sphere.id, tick)?;
        let native = self.0.native()?;
        Ok(MovingObservation {
            hinge,
            sphere,
            reactions: [
                native.moving_reaction(parent, ball)?,
                native.moving_reaction(child, ball)?,
            ],
        })
    }
}
