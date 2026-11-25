// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Watch the receiving shelf that actually bears the delivered parcel.
use super::*;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(super) struct Loss {
    pub at_ns: u128,
    pub shelf_link: u64,
    pub last_upward_impulse_ns: f64,
    pub retired: bool,
}

pub(super) struct Monitor {
    receiver: Part,
    shelf: Part,
    parcel_body: BodyId,
    shelf_body: BodyId,
    remove: bool,
    last_impulse: f64,
    pub loss: Option<Loss>,
}

impl Monitor {
    /// Bind only to the loaded receiver after the placement controller has confirmed handoff.
    pub(super) fn new(sim: &Simulation, remove: bool) -> Result<Self> {
        let shelf = part(sim, "receiver::shelf")?;
        let mut monitor = Self {
            receiver: part(sim, "receiver")?,
            shelf,
            parcel_body: part(sim, "parcel::box")?.body(sim)?,
            shelf_body: shelf.body(sim)?,
            remove,
            last_impulse: 0.,
            loss: None,
        };
        monitor.last_impulse = monitor.upward_impulse(sim)?;
        if monitor.last_impulse <= 0. {
            return Err("standby requires the receiving shelf to bear the parcel".into());
        }
        Ok(monitor)
    }

    fn upward_impulse(&self, sim: &Simulation) -> Result<f64> {
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local solver")?;
        Ok(native
            .normal_reaction_on(self.parcel_body, self.shelf_body)?
            .filter(|r| r.loaded_at_end)
            .map_or(0., |r| r.impulse[2].max(0.)))
    }

    /// Called at the committed boundary, before advancing either the arm or the parcel.
    pub(super) fn before_idle_step(
        &mut self,
        sim: &mut Simulation,
        idle_tick: usize,
    ) -> Result<bool> {
        if self.loss.is_some() {
            return Ok(false);
        }
        let live = sim
            .world()
            .get::<BodyHandle>(self.shelf.entity)
            .is_some_and(|handle| handle.id() == self.shelf_body);
        let mut retired = !live;
        if live {
            let impulse = self.upward_impulse(sim)?;
            if impulse > 0. {
                self.last_impulse = impulse;
            }
            if self.remove && idle_tick == 251 {
                let paused = sim.is_paused();
                sim.set_paused(true);
                let edit = super::adaptive::retire(sim, self.receiver.id);
                sim.set_paused(paused);
                edit?;
                retired = true;
            } else if impulse > 0. {
                return Ok(false);
            }
        }
        self.loss = Some(Loss {
            at_ns: sim.elapsed().as_nanos(),
            shelf_link: self.shelf.id,
            last_upward_impulse_ns: self.last_impulse,
            retired,
        });
        Ok(true)
    }
}

#[cfg(test)]
#[path = "adaptive_support/tests.rs"]
mod tests;
