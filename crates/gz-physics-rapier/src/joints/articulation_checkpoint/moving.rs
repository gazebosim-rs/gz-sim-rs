// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join and release one freely moving sphere alongside a two-link revolute.
use super::*;

const MAGIC: &[u8; 8] = b"GZAMV001";

impl RapierBackend {
    /// Read the one-step torque from a moving-profile worker's native checkpoint.
    /// The caller owns construction validation. This checks the envelope and reads
    /// the command without rebuilding the coupled solver graph.
    pub fn moving_checkpoint_effort(bytes: &[u8]) -> Result<Option<f64>, PhysicsError> {
        let (header, payload) = bytes.split_at_checked(40).ok_or_else(invalid)?;
        if bytes.len() > GROUNDED_REVOLUTE_LIMIT || header[8..] != Sha256::digest(payload)[..] {
            return Err(invalid());
        }
        let effort = if &header[..8] == super::MAGIC
            && (bytes.len() == EMPTY_BYTES || bytes.len() == EMPTY_BYTES + 8)
        {
            codec()
                .deserialize::<Image>(payload)
                .map_err(|_| invalid())?
                .effort
        } else if &header[..8] == MAGIC || &header[..8] == super::pair::MAGIC {
            let (_, _, effort): ([u8; 32], f64, Option<f64>) = codec()
                .deserialize_from(&mut &payload[..])
                .map_err(|_| invalid())?;
            effort
        } else {
            return Err(invalid());
        };
        if effort.is_some_and(|value| !value.is_finite()) {
            return Err(invalid());
        }
        Ok(effort)
    }

    fn companion_sphere(&self) -> Result<BodyId, PhysicsError> {
        self.bodies
            .iter()
            .find_map(|(&id, entry)| {
                (entry.1.articulation.is_none() && entry.1.collider.shape().as_ball().is_some())
                    .then_some(id)
            })
            .ok_or_else(unsupported)
    }

    pub(super) fn require_drained_sphere(&self, sphere: BodyId) -> Result<(), PhysicsError> {
        let (handle, ..) = self.bodies.get(&sphere).ok_or(PhysicsError::MissingBody)?;
        let [collider] = self.world.bodies[*handle].colliders() else {
            return Err(unsupported());
        };
        if self
            .world
            .narrow_phase
            .contact_pairs_with(*collider)
            .any(|pair| {
                pair.has_any_active_contact()
                    || pair
                        .manifolds()
                        .iter()
                        .any(|m| !m.points.is_empty() || !m.data.solver_contacts.is_empty())
            })
        {
            return Err(unsupported());
        }
        Ok(())
    }

    pub fn checkpoint_moving_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_companion(Companion::Sphere(self.companion_sphere()?), MAGIC)
    }

    pub fn restore_moving_revolute(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_companion(bytes, Companion::Sphere(self.companion_sphere()?), MAGIC)
    }

    pub fn moving_reaction(
        &self,
        link: BodyId,
        sphere: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        if self.companion_sphere()? != sphere {
            return Err(unsupported());
        }
        let reaction = self.link_reaction(link, sphere)?;
        // Empty broad-phase pairs can retain the direction of a former contact.
        // There is no current normal to report once that contact and its load are gone.
        if !reaction.active
            && reaction.normal_impulse == 0.
            && reaction.warmstart_normal_impulse == 0.
            && reaction.tangent_impulse == [0.; 3]
        {
            return Ok(MovingReaction::default());
        }
        Ok(reaction)
    }

    /// Export the original two-link recipe's compact motion image after the
    /// pair leaves the narrow phase. The companion must be the last body authored.
    pub fn detached_hinge_image(&self, sphere: BodyId) -> Result<Vec<u8>, PhysicsError> {
        let layout = self.revolute_layout(Some(Companion::Detaching(sphere)))?;
        // A fresh broad phase may not have processed a just-placed collider yet.
        // Check the actual link frames as well as the cached contact graph.
        let ball = |id: BodyId| -> Result<(Vector, f64), PhysicsError> {
            let body = &self.world.bodies[self.bodies[&id].0];
            let [handle] = body.colliders() else {
                return Err(unsupported());
            };
            let collider = &self.world.colliders[*handle];
            let pose = body.position() * collider.position_wrt_parent().ok_or_else(unsupported)?;
            Ok((
                pose.translation,
                collider.shape().as_ball().ok_or_else(unsupported)?.radius,
            ))
        };
        let (center, radius) = ball(sphere)?;
        for link in layout.links {
            let (other, link_radius) = ball(link)?;
            let margin =
                radius + link_radius + self.world.integration_parameters.prediction_distance();
            if center.distance(other) <= margin {
                return Err(unsupported());
            }
        }
        self.encode_hinge_motion(&layout)
    }

    /// Install current physical motion on a separately constructed companion.
    /// Commit its collider's pose before any full-world checkpoint can be observed.
    pub fn place_detached_sphere(
        &mut self,
        sphere: BodyId,
        state: BodyState,
    ) -> Result<(), PhysicsError> {
        self.soft_sphere_mobility(sphere)?;
        if self.bodies[&sphere].1.can_sleep {
            return Err(unsupported());
        }
        self.require_drained_sphere(sphere)?;
        self.set_body_state(sphere, state)?;
        self.world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut self.world.colliders);
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../../tests/private/moving_revolute.rs"]
mod tests;
