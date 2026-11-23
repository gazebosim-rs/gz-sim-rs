// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::utils::OrthonormalBasis;

/// One geometric point selected by the native rigid solver in its latest step.
#[cfg_attr(
    feature = "partition-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidPatchPoint {
    /// Frozen native solver anchor on a, in world axes (m). A responding
    /// body stores its COM-relative lever. A welded link retains its own public
    /// ID and the lever is measured from the responding assembly COM.
    /// A side attached to the world by
    /// native dominance stores the absolute point at the last full update.
    pub lever_a: [f64; 3],
    /// Anchor on b, with the same convention as `lever_a`.
    pub lever_b: [f64; 3],
    /// Normal impulse accumulated over this native step (N·s).
    pub normal_impulse: f64,
    /// Accumulated world-space tangential impulse on body a (N·s), for the
    /// native per-point model or a generalized multibody contact. The ordinary
    /// grouped rigid-body model solves a separate patch wrench.
    #[cfg_attr(
        feature = "partition-checkpoint",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub tangent_impulse_on_a: Option<[f64; 3]>,
}

/// One active native contact manifold between two finite public bodies.
/// One body pair can have more than one manifold.
#[derive(Debug, Clone)]
pub struct RigidContactPatch {
    pub a: BodyId,
    pub b: BodyId,
    /// World-space unit normal in the direction of the reaction on body a.
    pub normal_on_a: [f64; 3],
    pub points: Vec<RigidPatchPoint>,
}

/// Stored contact-point seeds for a pair of public bodies, in N·s.
/// These are initial guesses for a later constraint solve. They may persist on
/// dormant points and are distinct from the impulses delivered in a given step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactWarmstart {
    /// Number of tracked points in the queried warm-start history.
    pub points: usize,
    pub normal_ns: f64,
    /// Vector sum of stored world-space friction seeds acting on the first body.
    pub tangent_on_a_ns: [f64; 3],
}

impl RapierBackend {
    /// Native point history assigned to this pair for its next solve. Dormant
    /// prescribed pairs remain visible until the transition. A pair explicitly
    /// marked for a fresh solve has no effective warm start.
    pub fn contact_warmstart(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<ContactWarmstart, PhysicsError> {
        if self.contact_warmstart_discard_pending(a, b)? {
            return Ok(ContactWarmstart::default());
        }
        self.native_contact_warmstart(a, b)
    }

    /// Whether this pair's dormant history must be cleared at the next geometry
    /// update, before it can feed a solver row. The other points on the same
    /// foot collider keep their native manifold identity across this boundary.
    pub fn contact_warmstart_discard_pending(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<bool, PhysicsError> {
        let x = self.bodies.get(&a).ok_or(PhysicsError::MissingBody)?.0;
        let y = self.bodies.get(&b).ok_or(PhysicsError::MissingBody)?.0;
        if x == y {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(self
            .cold_support_pairs
            .iter()
            .any(|&p| p == (x, y) || p == (y, x)))
    }

    /// Inspect raw native point storage, including dormant old points queued
    /// for discard. This diagnostic never mutates solver state.
    pub fn native_contact_warmstart(
        &self,
        a: BodyId,
        b: BodyId,
    ) -> Result<ContactWarmstart, PhysicsError> {
        let handles = |id| -> Result<&[ColliderHandle], PhysicsError> {
            let handle = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?.0;
            Ok(self.world.bodies[handle].colliders())
        };
        let a_colliders = handles(a)?;
        let b_colliders = handles(b)?;
        if a == b {
            return Err(PhysicsError::InvalidValue);
        }
        let mut cache = ContactWarmstart::default();
        let mut tangent = Vector::ZERO;
        for &first in a_colliders {
            for &second in b_colliders {
                let Some(pair) = self.world.narrow_phase.contact_pair(first, second) else {
                    continue;
                };
                let sign = if pair.collider1 == first { 1. } else { -1. };
                for manifold in pair.solver_manifolds() {
                    cache.points += manifold.points.len();
                    for point in &manifold.points {
                        cache.normal_ns += point.data.warmstart_impulse;
                        tangent += sign * point.data.warmstart_tangent_world;
                    }
                }
            }
        }
        cache.tangent_on_a_ns = tangent.to_array();
        Ok(cache)
    }

    /// The generic multibody and per-point rigid solvers store applied tangent
    /// impulses in Rapier's basis for collider one, before public ID sorting.
    pub(crate) fn applied_tangent_basis(
        &self,
        manifold: &rapier3d_f64::geometry::ContactManifold,
    ) -> Option<(Vector, Vector)> {
        let per_point = self.per_point_friction_enabled()
            || self.welded_projection.is_none()
                && [manifold.data.rigid_body1, manifold.data.rigid_body2]
                    .into_iter()
                    .flatten()
                    .any(|h| self.world.multibody_joints.rigid_body_link(h).is_some());
        per_point.then(|| {
            let n = -manifold.data.normal;
            let tangent = n.orthonormal_vector();
            (tangent, n.cross(tangent))
        })
    }

    /// Read solved geometric points on demand. A caller using external
    /// substepping can read each native step before advancing the solver again.
    pub fn rigid_contact_patches(&self) -> Vec<RigidContactPatch> {
        let lookup = self.contact_body_lookup();
        let mut result = Vec::new();
        for pair in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|p| p.has_any_active_contact())
        {
            let (Some(first), Some(second)) = (lookup(pair.collider1), lookup(pair.collider2))
            else {
                continue;
            };
            let ordered = first <= second;
            for manifold in pair.solver_manifolds() {
                if manifold.data.solver_contacts.is_empty() {
                    continue;
                }
                let basis = self.applied_tangent_basis(manifold);
                let points = super::solved_point_indices(manifold)
                    .map(|i| {
                        let data = &manifold.points[i].data;
                        let (a, b) = if ordered {
                            (data.solver_dp1, data.solver_dp2)
                        } else {
                            (data.solver_dp2, data.solver_dp1)
                        };
                        RigidPatchPoint {
                            lever_a: a.to_array(),
                            lever_b: b.to_array(),
                            normal_impulse: data.impulse,
                            tangent_impulse_on_a: basis.map(|(t1, t2)| {
                                let impulse =
                                    t1 * data.tangent_impulse.x + t2 * data.tangent_impulse.y;
                                (if ordered { impulse } else { -impulse }).to_array()
                            }),
                        }
                    })
                    .collect();
                result.push(RigidContactPatch {
                    a: first.min(second),
                    b: first.max(second),
                    normal_on_a: (if ordered {
                        -manifold.data.normal
                    } else {
                        manifold.data.normal
                    })
                    .to_array(),
                    points,
                });
            }
        }
        result.sort_by_key(|p| (p.a, p.b));
        result
    }
}
