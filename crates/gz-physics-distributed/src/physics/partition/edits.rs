// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl ComponentInfo {
    pub(crate) fn admit_free_edit(&self, tier: Tier) -> Result<(), PhysicsError> {
        if tier == Tier::Fixed {
            return Err(PhysicsError::Unsupported {
                engine: "distributed",
                capability: "pool edits require a moving body",
            });
        }
        if self.ids.len() != 1 || self.contact_history {
            return Err(PhysicsError::TransitionDeferred {
                reason: "body belongs to a grouped or previously contacting native solver",
            });
        }
        Ok(())
    }
}

impl Partition {
    pub(crate) fn admit_free_edit(&self, id: u64) -> Result<(), WorldError> {
        let component = &self.components[self.owner(id)?];
        let index = self
            .committed
            .bodies
            .binary_search_by_key(&id, |b| b.id)
            .map_err(|_| WorldError::Missing(id))?;
        Ok(component.admit_free_edit(self.committed.bodies[index].tier)?)
    }

    pub(crate) fn set_velocity(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), WorldError> {
        self.healthy()?;
        let i = self.owner(id)?;
        let solver = &mut self.solvers[i];
        let world = solver.sim.world_mut();
        let native = world.get::<Binding>(solver.index[&id]).unwrap().native;
        let mut backend = world.resource_mut::<PhysicsBackend>();
        if backend.is_dynamic(native) == Some(false) && backend.is_kinematic(native) == Some(false)
        {
            return Err(PhysicsError::Unsupported {
                engine: "distributed",
                capability: "velocity command on a fixed body",
            }
            .into());
        }
        if backend.is_kinematic(native) == Some(true) {
            backend.validate_kinematic_velocity(linear, angular, self.committed.step.duration())?;
        }
        let mut body = self
            .committed
            .bodies
            .iter()
            .find(|b| b.id == id)
            .unwrap()
            .clone();
        body.linear = linear;
        body.angular = angular;
        validate_queued_for(&body, &self.pending)?;
        backend.set_world_velocity(native, linear, angular)?;
        solver.refresh(self.committed.tick);
        self.publish(self.committed.tick);
        Ok(())
    }

    pub(crate) fn preview_impulse(
        &self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<([f64; 3], [f64; 3]), WorldError> {
        self.healthy()?;
        let i = self.owner(id)?;
        self.solvers[i].preview_impulse(id, linear, angular, &self.pending)
    }

    pub(crate) fn impulse_now(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), WorldError> {
        self.healthy()?;
        let i = self.owner(id)?;
        self.solvers[i].impulse_now(id, linear, angular, &self.pending)?;
        self.publish(self.committed.tick);
        Ok(())
    }

    /// Permanently remove a body. Surviving bodies keep their native world,
    /// bindings and contact history. Pending commands for the erased body expire.
    pub(crate) fn retire(&mut self, id: u64) -> Result<(), WorldError> {
        self.healthy()?;
        let i = self.owner(id)?;
        let solver = &mut self.solvers[i];
        if solver.index.len() == 1 {
            self.solvers.remove(i);
        } else {
            let e = solver.index[&id];
            let world = solver.sim.world_mut();
            let native = world.get::<Binding>(e).unwrap().native;
            if !world.resource_mut::<PhysicsBackend>().remove(native) {
                return Err(WorldError::Missing(id));
            }
            solver.index.remove(&id);
            world.despawn(e);
            solver.committed.bodies.retain(|b| b.id != id);
            solver.prefix.retain(|member| member != id);
            solver.refresh(self.committed.tick);
        }
        self.pending.retain(|p| p.id != id);
        self.publish(self.committed.tick);
        Ok(())
    }
}

impl Island {
    fn preview_impulse(
        &self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
        pending: &[PendingImpulse],
    ) -> Result<([f64; 3], [f64; 3]), WorldError> {
        self.healthy()?;
        let e = *self.index.get(&id).ok_or(WorldError::Missing(id))?;
        let world = self.sim.world();
        let native = world.get::<Binding>(e).unwrap().native;
        let backend = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        if backend.is_dynamic(native) != Some(true) {
            return Err(PhysicsError::Unsupported {
                engine: "distributed",
                capability: "spatial impulse on a nondynamic body",
            }
            .into());
        }
        let (v, w) = backend.velocity_after_impulse(native, linear, angular)?;
        let mut body = self
            .committed
            .bodies
            .iter()
            .find(|b| b.id == id)
            .unwrap()
            .clone();
        body.linear = v;
        body.angular = w;
        validate_queued_for(&body, pending)?;
        Ok((v, w))
    }
    pub(crate) fn impulse_now(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
        pending: &[PendingImpulse],
    ) -> Result<(), WorldError> {
        self.preview_impulse(id, linear, angular, pending)?;
        let world = self.sim.world_mut();
        let native = world.get::<Binding>(self.index[&id]).unwrap().native;
        let mut resource = world.resource_mut::<PhysicsBackend>();
        let backend = resource.downcast_mut::<MultiFidelityBackend>().unwrap();
        backend.apply_impulse(native, linear, angular)?;
        self.refresh(self.committed.tick);
        Ok(())
    }
}

impl Partition {
    pub(crate) fn replace_state(&mut self, body: &Body) -> Result<(), WorldError> {
        self.healthy()?;
        let index = self.owner(body.id)?;
        let old = self
            .committed
            .bodies
            .iter()
            .find(|b| b.id == body.id)
            .unwrap();
        admit_edit(
            old,
            body,
            self.committed.bodies.iter(),
            &self.pending,
            self.committed.step.duration(),
            self.committed.ground,
        )?;
        let solver = &mut self.solvers[index];
        let world = solver.sim.world_mut();
        let handle = world.get::<Binding>(solver.index[&body.id]).unwrap().native;
        world
            .resource_mut::<PhysicsBackend>()
            .set_body_state(handle, body.state())?;
        solver.refresh(self.committed.tick);
        self.publish(self.committed.tick);
        Ok(())
    }
}
