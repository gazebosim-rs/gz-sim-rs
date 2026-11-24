// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// A solved point of the native shared halfspace in the most recent native interval.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroundPatchPoint {
    /// Point on the halfspace surface, in world coordinates (m).
    pub plane_witness: [f64; 3],
    /// Solver application midpoint in world coordinates at the native contact update (m).
    /// Rapier can retain this anchor across narrow-phase recycling.
    pub solver_point: [f64; 3],
    /// World-space solver lever about the dynamic body's center of mass, at that update (m).
    pub body_lever: [f64; 3],
    /// Normal load applied during the latest native interval (N·s).
    pub normal_impulse: f64,
}

/// One active native manifold between a finite body and the unparented shared plane.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeGroundPatch {
    pub body: BodyId,
    /// World-space direction of the normal reaction on the finite body.
    pub normal_on_body: [f64; 3],
    pub points: Vec<GroundPatchPoint>,
}

impl RapierBackend {
    /// Read only solved halfspace points. No per-point collection runs during normal stepping.
    pub fn partition_ground_patches(&self) -> Vec<NativeGroundPatch> {
        let Some((ground, collider)) = self
            .world
            .colliders
            .iter()
            .find(|(_, c)| c.parent().is_none() && c.shape().as_halfspace().is_some())
        else {
            return Vec::new();
        };
        self.solved_plane_patches(ground, collider, Vector::ZERO)
    }

    /// Solved contacts against one authored fixed halfspace with a public body ID.
    pub fn fixed_ground_patches(&self, id: BodyId) -> Result<Vec<NativeGroundPatch>, PhysicsError> {
        let handle = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?.0;
        let rb = &self.world.bodies[handle];
        let [ground] = rb.colliders() else {
            return Err(PhysicsError::InvalidValue);
        };
        let collider = &self.world.colliders[*ground];
        if !rb.is_fixed() || collider.shape().as_halfspace().is_none() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(self.solved_plane_patches(*ground, collider, rb.center_of_mass()))
    }

    fn solved_plane_patches(
        &self,
        ground: ColliderHandle,
        collider: &Collider,
        ground_com: Vector,
    ) -> Vec<NativeGroundPatch> {
        let get_id = self.contact_body_lookup();
        let mut patches = Vec::new();
        for pair in self
            .world
            .narrow_phase
            .contact_pairs_with(ground)
            .filter(|p| p.has_any_active_contact())
        {
            let ground_first = pair.collider1 == ground;
            let finite = if ground_first {
                pair.collider2
            } else {
                pair.collider1
            };
            let Some(body) = get_id(finite) else { continue };
            for manifold in pair
                .solver_manifolds()
                .iter()
                .filter(|m| !m.data.solver_contacts.is_empty())
            {
                let points = crate::partition::solved_point_indices(manifold)
                    .map(|i| {
                        let p = &manifold.points[i];
                        let (witness, solver, lever) = if ground_first {
                            (p.local_p1, p.data.solver_dp1, p.data.solver_dp2)
                        } else {
                            (p.local_p2, p.data.solver_dp2, p.data.solver_dp1)
                        };
                        GroundPatchPoint {
                            plane_witness: (collider.position() * witness).to_array(),
                            solver_point: (if (ground_first
                                && manifold.data.relative_dominance <= 0)
                                || (!ground_first && manifold.data.relative_dominance >= 0)
                            {
                                ground_com + solver
                            } else {
                                solver
                            })
                            .to_array(),
                            body_lever: lever.to_array(),
                            normal_impulse: p.data.impulse,
                        }
                    })
                    .collect();
                patches.push(NativeGroundPatch {
                    body,
                    normal_on_body: (if ground_first {
                        manifold.data.normal
                    } else {
                        -manifold.data.normal
                    })
                    .to_array(),
                    points,
                });
            }
        }
        patches.sort_by_key(|p| p.body);
        patches
    }
}

#[cfg(test)]
#[path = "../../tests/private/ground_patch_geometry.rs"]
mod tests;
