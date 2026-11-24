// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fixed native identities allow contacts on one companion to survive moving the other.
use super::*;
mod island;

pub(super) const MAGIC: &[u8; 8] = b"GZAMV002";

impl RapierBackend {
    /// Admit a four-body, zero-gravity graph with the hinge authored first.
    pub fn configure_pair_revolute(&mut self) -> Result<(), PhysicsError> {
        if !self.joints.open {
            return Err(unsupported());
        }
        let mut free = self
            .bodies
            .iter()
            .filter(|(_, entry)| entry.1.articulation.is_none())
            .map(|(&id, _)| id);
        let spheres = [
            free.next().ok_or_else(unsupported)?,
            free.next().ok_or_else(unsupported)?,
        ];
        if free.next().is_some() {
            return Err(unsupported());
        }
        // Complete all admission checks before changing sleep or contact settings.
        // Ordinary Bevy worlds may match the broad SDF shape but use a limited
        // revolute or a free body's non-isotropic inertia.
        let links = self
            .inspect_revolute_layout(Some(Companion::Pair(spheres)), true)?
            .links;
        for id in links.into_iter().chain(spheres) {
            self.validate_moving_body(id)?;
        }
        for id in links.into_iter().chain(spheres) {
            self.enable_moving_body(id);
        }
        self.pair_observer = Some(PairObserver {
            island_construction: None,
            spheres,
            links,
            initial_velocity: self.world.bodies[self.bodies[&spheres[0]].0]
                .linvel()
                .to_array(),
        });
        Ok(())
    }

    /// Save the solver graph including its current ownership mask and warm contacts.
    pub fn checkpoint_pair_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_companion(
            Companion::Pair(self.pair_observer.ok_or_else(unsupported)?.spheres),
            MAGIC,
        )
    }

    pub fn restore_pair_revolute(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_companion(
            bytes,
            Companion::Pair(self.pair_observer.ok_or_else(unsupported)?.spheres),
            MAGIC,
        )
    }

    pub fn pair_sphere_active(&self, id: BodyId) -> Result<bool, PhysicsError> {
        if !self
            .pair_observer
            .ok_or_else(unsupported)?
            .spheres
            .contains(&id)
        {
            return Err(unsupported());
        }
        Ok(self.world.bodies[self.bodies[&id].0].is_enabled())
    }

    /// Stop integrating a drained companion, or reattach it at its authoritative pose.
    /// The other companion's body, collider and contact solver caches stay resident.
    pub fn set_pair_sphere_active(
        &mut self,
        id: BodyId,
        active: bool,
        state: Option<BodyState>,
    ) -> Result<(), PhysicsError> {
        let was_active = self.pair_sphere_active(id)?;
        if was_active == active || state.is_some() != active {
            return Err(unsupported());
        }
        self.require_drained_sphere(id)?;
        let h = self.bodies[&id].0;
        let collider = self.world.bodies[h].colliders()[0];
        if let Some(state) = state {
            self.set_body_state(id, state)?;
            self.world
                .bodies
                .propagate_modified_body_positions_to_colliders(&mut self.world.colliders);
        } else if !self.joints.open {
            let center_radius = |c: &Collider| -> Result<_, PhysicsError> {
                Ok((
                    c.position().translation,
                    c.shape().as_ball().ok_or_else(unsupported)?.radius,
                ))
            };
            let (center, radius) = center_radius(&self.world.colliders[collider])?;
            for (&other, (handle, ..)) in &self.bodies {
                let body = &self.world.bodies[*handle];
                if other != id && body.is_enabled() {
                    let (point, r) = center_radius(&self.world.colliders[body.colliders()[0]])?;
                    if center.distance(point)
                        <= radius + r + self.world.integration_parameters.prediction_distance()
                    {
                        return Err(unsupported());
                    }
                }
            }
        }
        self.world.bodies[h].set_enabled(active);
        self.world.colliders[collider].set_enabled(active);
        Ok(())
    }

    /// Reaction on the first body from the second. Each pair is one native ball manifold.
    pub fn pair_reaction(
        &self,
        first: BodyId,
        second: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        let group = self.pair_observer.ok_or_else(unsupported)?;
        if self.bodies.len() != 4
            || first == second
            || !self.bodies.contains_key(&first)
            || !self.bodies.contains_key(&second)
            || !(group.spheres.contains(&first) || group.spheres.contains(&second))
        {
            return Err(unsupported());
        }
        let mut reaction = self.contact_reaction(first, second)?;
        if !reaction.active
            && reaction.normal_impulse == 0.
            && reaction.warmstart_normal_impulse == 0.
            && reaction.tangent_impulse == [0.; 3]
        {
            return Ok(MovingReaction::default());
        }
        if group.spheres.contains(&first)
            && group.spheres.contains(&second)
            && reaction.normal_impulse > 0.
        {
            let sphere = group.spheres[0];
            let rb = &self.world.bodies[self.bodies[&sphere].0];
            // In this loaded profile gravity, damping and free-sphere forcing are zero.
            // Subtract both hinge impulses to isolate the grouped rigid friction impulse.
            let mut impulse = (rb.linvel() - Vector::from(group.initial_velocity)) * rb.mass();
            for link in group.links {
                let r = self.contact_reaction(link, sphere)?;
                impulse +=
                    Vector::from(r.normal) * r.normal_impulse + Vector::from(r.tangent_impulse);
            }
            if first != sphere {
                impulse = -impulse;
            }
            let n = Vector::from(reaction.normal);
            reaction.tangent_impulse = (impulse - n * impulse.dot(n)).to_array();
        }
        Ok(reaction)
    }
}
