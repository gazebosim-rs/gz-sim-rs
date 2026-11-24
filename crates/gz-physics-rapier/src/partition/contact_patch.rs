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

/// Compressive impulse during one ordinary public step, in N·s. The vector
/// acts on the requested link. The magnitude sums individual point magnitudes,
/// so opposing faces can carry a load while their resultant cancels.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NormalReaction {
    pub impulse: [f64; 3],
    pub magnitude: f64,
    pub loaded_at_end: bool,
}

pub(crate) fn manifold_reaction(
    pair: &rapier3d_f64::geometry::ContactPair,
    first_is_target: bool,
) -> NormalReaction {
    let mut magnitude = 0.;
    let mut impulse = Vector::ZERO;
    for manifold in pair.solver_manifolds() {
        let j = super::solved_point_indices(manifold)
            .map(|i| manifold.points[i].data.impulse)
            .sum::<f64>();
        magnitude += j;
        impulse += manifold.data.normal * j;
    }
    NormalReaction {
        impulse: (if first_is_target { -impulse } else { impulse }).to_array(),
        magnitude,
        loaded_at_end: magnitude > 0.,
    }
}

impl NormalReaction {
    pub(crate) fn add(&mut self, other: Self, last: bool) {
        self.impulse = (Vector::from(self.impulse) + Vector::from(other.impulse)).to_array();
        self.magnitude += other.magnitude;
        self.loaded_at_end |= last && other.loaded_at_end;
    }
}

/// Native persistent point history, including dormant manifold points. Entries
/// retain collider order, the point feature used for matching, frozen lever arms
/// and all three directions of the next solver's impulse seeds.
#[derive(Debug, Clone, Copy)]
pub struct StoredContactPoint {
    pub colliders: [(u32, u32); 2],
    pub subshapes: [u32; 2],
    pub features: [u32; 2],
    pub local_anchors: [[f64; 3]; 2],
    pub solver_arms: [[f64; 3]; 2],
    pub normal: [f64; 3],
    pub distance_m: f64,
    pub normal_ns: f64,
    pub tangent_basis_ns: [f64; 2],
    pub tangent_world_ns: [f64; 3],
    pub twist_nms: f64,
}

impl RapierBackend {
    /// Read point storage in native pair, manifold and feature order without
    /// waking an island, refreshing geometry or clearing solver warm starts.
    pub fn stored_contact_points(&self) -> Vec<StoredContactPoint> {
        let mut points = Vec::new();
        for pair in self.world.narrow_phase.contact_pairs() {
            let colliders = [
                pair.collider1.into_raw_parts(),
                pair.collider2.into_raw_parts(),
            ];
            for manifold in pair.solver_manifolds() {
                for point in &manifold.points {
                    let data = &point.data;
                    points.push(StoredContactPoint {
                        colliders,
                        subshapes: [manifold.subshape1, manifold.subshape2],
                        features: [point.fid1.0, point.fid2.0],
                        local_anchors: [point.local_p1.to_array(), point.local_p2.to_array()],
                        solver_arms: [data.solver_dp1.to_array(), data.solver_dp2.to_array()],
                        normal: manifold.data.normal.to_array(),
                        distance_m: point.dist,
                        normal_ns: data.warmstart_impulse,
                        tangent_basis_ns: [
                            data.warmstart_tangent_impulse.x,
                            data.warmstart_tangent_impulse.y,
                        ],
                        tangent_world_ns: data.warmstart_tangent_world.to_array(),
                        twist_nms: data.warmstart_twist_impulse,
                    });
                }
            }
        }
        points
    }
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
