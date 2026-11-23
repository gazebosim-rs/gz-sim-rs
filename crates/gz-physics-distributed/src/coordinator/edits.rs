// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{
    physics::{Tier, validate_queued_for},
    protocol::VelocityEdit,
};

impl<P: Peer> Coordinator<P> {
    /// Replace an owned box's world twist at a published tick. Prospective
    /// cross-worker contacts are routed before the next advancing barrier.
    pub fn set_velocity(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<&Frame, Error> {
        if linear.into_iter().chain(angular).any(|v| !v.is_finite()) {
            return Err(Error::Invalid("nonfinite body twist"));
        }
        let (w, _) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown velocity body"))?;
        let w = usize::from(w);
        let mut expected = self.frame.workers[w].clone();
        let b = expected
            .snapshot
            .bodies
            .iter_mut()
            .find(|b| b.id == id)
            .unwrap();
        if b.tier == Tier::Fixed {
            return Err(Error::Invalid("fixed body velocity command"));
        }
        b.linear = linear;
        b.angular = angular;
        if b.tier == Tier::Prescribed {
            use gz_physics::DynamicBackend;
            gz_physics_multifidelity::MultiFidelityBackend::new([0.; 3])
                .and_then(|native| {
                    native.validate_kinematic_velocity(
                        linear,
                        angular,
                        self.frame.step().duration(),
                    )
                })
                .map_err(|_| Error::Invalid("invalid prescribed twist"))?;
        }
        validate_queued_for(b, &self.queued)
            .map_err(|_| Error::Invalid("velocity invalidates queued impulse"))?;
        let edit = VelocityEdit {
            id,
            linear,
            angular,
        };
        let mut counts = [0; 2];
        counts[w] = 1;
        let mut cursor = self.begin(Action::Velocity(edit.clone()), counts)?;
        let obs = observation(self.call(
            &mut cursor,
            w,
            Command::SetVelocity {
                tick: self.frame.tick,
                edit,
            },
        )?)?;
        if obs != expected {
            return Err(Error::Invalid(
                "velocity acknowledgement changed unrelated state",
            ));
        }
        self.frame.workers[w] = obs;
        self.commit_action();
        Ok(&self.frame)
    }

    /// Apply an immediate world spatial impulse to one authoritative native
    /// body. The resident inertia may have rotated since the previous barrier.
    pub fn impulse_now(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<&Frame, Error> {
        self.impulse_checked(id, linear, angular, None)
    }

    pub(crate) fn impulse_checked(
        &mut self,
        id: u64,
        linear: [f64; 3],
        angular: [f64; 3],
        field: Option<([f64; 3], &[crate::physics::BodyWrench])>,
    ) -> Result<&Frame, Error> {
        let (owner, body) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown impulse body"))?;
        if body.tier != Tier::Dynamic || linear.into_iter().chain(angular).any(|v| !v.is_finite()) {
            return Err(Error::Invalid("invalid spatial impulse"));
        }
        let w = usize::from(owner);
        let mut expected = self.frame.workers[w].clone();
        let impulse = crate::protocol::SpatialImpulse {
            id,
            linear,
            angular,
        };
        let mut counts = [0, 0];
        counts[w] = 1;
        let mut cursor = self.begin(Action::ImpulseNow(impulse.clone()), counts)?;
        let obs = observation(self.call(
            &mut cursor,
            w,
            Command::ImpulseNow {
                tick: self.frame.tick,
                impulse,
            },
        )?)?;
        let changed = obs
            .snapshot
            .bodies
            .iter()
            .find(|b| b.id == id)
            .ok_or(Error::Invalid("missing impulse body"))?;
        if changed
            .linear
            .into_iter()
            .chain(changed.angular)
            .any(|v| !v.is_finite())
        {
            return Err(Error::Invalid("nonfinite impulse result"));
        }
        let target = expected
            .snapshot
            .bodies
            .iter_mut()
            .find(|b| b.id == id)
            .unwrap();
        target.linear = changed.linear;
        target.angular = changed.angular;
        validate_queued_for(target, &self.queued)
            .map_err(|_| Error::Invalid("impulse invalidates queue"))?;
        if obs != expected {
            return Err(Error::Invalid("impulse changed unrelated state"));
        }
        // Use the resident body's exact inverse inertia. If the resulting state
        // is inadmissible, supervision fences and restores the acknowledged prefix.
        if let Some((gravity, inputs)) = field {
            self.validate_edited_owner(w, obs.clone(), gravity, inputs)?;
        }
        self.frame.workers[w] = obs;
        self.commit_action();
        Ok(&self.frame)
    }

    /// Split a native solver only across separated swept groups with no native
    /// contact cache joining them. This same-tick operation preserves all
    /// published body states, contact impulses, queued inputs and ownership.
    pub fn split_component(&mut self, member: u64) -> Result<&Frame, Error> {
        let (owner, _) = self
            .frame
            .body(member)
            .ok_or(Error::Invalid("unknown split member"))?;
        let w = usize::from(owner);
        let mut expected = self.frame.workers[w].clone();
        let i = expected
            .components
            .iter()
            .position(|g| g.ids.binary_search(&member).is_ok())
            .ok_or(Error::Invalid("missing native component"))?;
        let groups = crate::physics::separated_groups(
            &expected.snapshot,
            &expected.components[i],
            &self.queued,
        )
        .map_err(|_| Error::Colocation)?;
        if groups.len() == 1 {
            self.require_committed()?;
            return Ok(&self.frame);
        }
        expected.components.remove(i);
        expected.components.extend(groups);
        expected.components.sort_by_key(|g| g.ids[0]);
        let mut count = [0; 2];
        count[w] = 1;
        let mut cursor = self.begin(Action::Split(member), count)?;
        let obs = observation(self.call(
            &mut cursor,
            w,
            Command::SplitComponent {
                tick: self.frame.tick,
                member,
            },
        )?)?;
        if obs != expected {
            return Err(Error::Invalid("split changed physics or ownership"));
        }
        self.frame.workers[w] = obs;
        self.commit_action();
        Ok(&self.frame)
    }

    /// Permanently retire one ID at a same-tick two-peer barrier. Forget its
    /// released generation and commands only after both acknowledgements.
    pub fn retire(&mut self, id: u64) -> Result<&Frame, Error> {
        let (owner, _) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown retired body"))?;
        let w = usize::from(owner);
        let generation = self.frame.workers[w].generations[&id];
        let mut expected = self.frame.workers.clone();
        expected[w].project_retirement(id);
        let mut cursor = self.begin(Action::Retire(id), [1, 1])?;
        let cmd = Command::RetireBody {
            tick: self.frame.tick,
            id,
            owner,
            generation,
        };
        for (i, expected) in expected.iter().enumerate() {
            if observation(self.call(&mut cursor, i, cmd.clone())?)? != *expected {
                return Err(Error::Invalid(
                    "retirement acknowledgement changed surviving bodies",
                ));
            }
        }
        self.frame.workers = expected;
        self.queued.retain(|p| p.id != id);
        self.commit_action();
        Ok(&self.frame)
    }
}

impl Observation {
    pub(crate) fn project_retirement(&mut self, id: u64) {
        self.generations.remove(&id);
        self.snapshot.bodies.retain(|b| b.id != id);
        self.snapshot.supports.retain(|p| p.body != id);
        self.snapshot.impulses.retain(|p| p.a != id && p.b != id);
        self.snapshot.contacts = self.snapshot.impulses.len();
        for c in &mut self.components {
            c.ids.retain(|&m| m != id);
        }
        self.components.retain(|c| !c.ids.is_empty());
        self.components.sort_by_key(|c| c.ids[0]);
        self.snapshot.contact_history = self.components.iter().any(|c| c.contact_history);
    }
}
