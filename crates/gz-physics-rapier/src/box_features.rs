// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fresh separating features for explicitly selected finite-box pairs.
use super::*;
use crate::contact_dispatcher::{DiscardPairs, Prepare};
use rapier3d_f64::{geometry::ContactManifold, parry::shape::Shape as NativeShape};
use std::collections::HashSet;
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

mod return_witness;
pub use return_witness::{ReturnFeatureCounts, ReturnFeatureMode, ReturnPlaneEvidence};

/// The named pair and the actual unloaded-manifold queries made inside this interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReturningBoxFeatureReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub counts: ReturnFeatureCounts,
}

#[derive(Clone)]
pub(super) struct BoxFeatures(Arc<Selected>);
type NativePair = (RigidBodyHandle, RigidBodyHandle);

struct Conditional {
    bodies: (BodyId, BodyId),
    key: NativePair,
    witness: return_witness::Witness,
}
#[derive(Default)]
struct Selected {
    pairs: HashSet<NativePair>,
    returning: Vec<Conditional>,
    rows: Vec<ColliderHandle>,
    rebuilt: AtomicU64,
}
impl BoxFeatures {
    fn locate(
        solver: &RapierBackend,
        pair: (BodyId, BodyId),
    ) -> Result<(NativePair, Option<ColliderHandle>), PhysicsError> {
        let f = solver
            .bodies
            .get(&pair.0)
            .ok_or(PhysicsError::MissingBody)?
            .0;
        let p = solver
            .bodies
            .get(&pair.1)
            .ok_or(PhysicsError::MissingBody)?
            .0;
        let (foot, pad) = (&solver.world.bodies[f], &solver.world.bodies[p]);
        let invalid = || PhysicsError::Unsupported {
            engine: "rapier",
            capability: "fresh box features require one cuboid per dynamic foot and fixed or kinematic support",
        };
        let ([a], [b]) = (foot.colliders(), pad.colliders()) else {
            return Err(invalid());
        };
        let colliders = &solver.world.colliders;
        if !foot.is_dynamic()
            || !(pad.is_kinematic() || pad.is_fixed())
            || [a, b]
                .into_iter()
                .any(|h| colliders[*h].shape().as_cuboid().is_none())
        {
            return Err(invalid());
        }
        let recycles = (colliders[*a].active_hooks() | colliders[*b].active_hooks()).is_empty();
        Ok(((f, p), recycles.then_some(*a)))
    }
    pub(super) fn new(
        solver: &RapierBackend,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<Option<Self>, PhysicsError> {
        Self::enroll(
            solver,
            pairs,
            &[],
            ReturnFeatureMode::FirstStale,
            Duration::ZERO,
            1,
        )
    }
    pub(super) fn enroll(
        solver: &RapierBackend,
        always: &[(BodyId, BodyId)],
        conditional: &[(BodyId, BodyId)],
        mode: ReturnFeatureMode,
        dt: Duration,
        calls: u32,
    ) -> Result<Option<Self>, PhysicsError> {
        if always.is_empty() && conditional.is_empty() {
            return Ok(None);
        }
        let mut s = Selected::default();
        for &pair in always {
            let (key, row) = Self::locate(solver, pair)?;
            s.pairs.insert(key);
            // The historical unconditional option explicitly requests fresh geometry even if the native solver recycles rows.
            if let Some(h) = row {
                s.rows.push(h);
            }
        }
        s.rows.sort_unstable_by_key(|h| h.into_raw_parts());
        s.rows.dedup();
        for &pair in conditional {
            let (key, _) = Self::locate(solver, pair)?;
            if s.pairs.contains(&key) || s.returning.iter().any(|e| e.key == key) {
                return Err(PhysicsError::InvalidValue);
            }
            // This option only observes naturally invoked narrow-phase queries.
            // It never invalidates a collider or disables recycling on an unrelated pair.
            let tolerance = Self::threshold(solver, pair, mode, dt, calls);
            s.returning.push(Conditional {
                bodies: pair,
                key,
                witness: return_witness::Witness::new(mode, tolerance),
            });
        }
        Ok(Some(Self(Arc::new(s))))
    }
    fn threshold(
        solver: &RapierBackend,
        pair: (BodyId, BodyId),
        mode: ReturnFeatureMode,
        dt: Duration,
        calls: u32,
    ) -> f64 {
        if mode == ReturnFeatureMode::FirstTravel {
            let body = |id| &solver.world.bodies[solver.bodies[&id].0];
            let foot = body(pair.0);
            let pad = body(pair.1);
            let arm = |b: &RigidBody| {
                let shape = &solver.world.colliders[b.colliders()[0]];
                let half = shape
                    .shape()
                    .as_cuboid()
                    .expect("verified cuboid")
                    .half_extents;
                half.length() + shape.position().translation.distance(b.center_of_mass())
            };
            let radii = [arm(foot), arm(pad)];
            let speed = (foot.linvel() - pad.linvel()).length()
                + foot.angvel().length() * radii[0]
                + pad.angvel().length() * radii[1]
                + solver.world.gravity.length() * dt.as_secs_f64();
            let longest_step_s = dt.as_nanos().div_ceil(u128::from(calls)) as f64 * 1e-9;
            (speed * longest_step_s).max(128. * f64::EPSILON * (1. + radii[0] + radii[1]))
        } else {
            solver.world.integration_parameters.allowed_linear_error() / 8.
        }
    }
    #[cfg(feature = "landing-audit")]
    pub(super) fn returning(
        solver: &RapierBackend,
        pair: (BodyId, BodyId),
        mode: ReturnFeatureMode,
        dt: Duration,
        calls: u32,
    ) -> Result<Option<Self>, PhysicsError> {
        Self::enroll(solver, &[], &[pair], mode, dt, calls)
    }
    #[cfg(feature = "landing-audit")]
    pub(super) fn return_counts(&self) -> ReturnFeatureCounts {
        self.0.returning[0].witness.counts()
    }
    pub(super) fn return_receipts(&self) -> Vec<ReturningBoxFeatureReceipt> {
        self.0
            .returning
            .iter()
            .map(|r| ReturningBoxFeatureReceipt {
                foot: r.bodies.0,
                support: r.bodies.1,
                counts: r.witness.counts(),
            })
            .collect()
    }
    pub(super) fn request_queries(&self, colliders: &mut ColliderSet) {
        // Rapier can skip the dispatcher for recycled pairs. Invalidate rows
        // only on enrolled colliders lacking a native hook. Other manifolds use ordinary matching.
        for &h in &self.0.rows {
            crate::articulation_support::refresh_contact_rows(&mut colliders[h]);
        }
    }
    pub(super) fn rebuilt(&self) -> u64 {
        self.0.rebuilt.load(Ordering::Relaxed)
    }
}
pub(super) struct PrepareContacts {
    pub(super) cold: Option<DiscardPairs>,
    pub(super) fresh: Option<BoxFeatures>,
}
impl Prepare for PrepareContacts {
    fn prepare(
        &self,
        _pose: &Pose,
        a: &dyn NativeShape,
        b: &dyn NativeShape,
        manifolds: &mut Vec<ContactManifold>,
    ) {
        if let Some(cold) = &self.cold {
            cold.prepare(_pose, a, b, manifolds);
        }
        let Some(fresh) = &self.fresh else {
            return;
        };
        for m in manifolds {
            if let Some((first, second)) = m.data.rigid_body1.zip(m.data.rigid_body2)
                && !m.points.is_empty()
            {
                let key = (first, second);
                let inverse = (second, first);
                if !(fresh.0.pairs.contains(&key) || fresh.0.pairs.contains(&inverse)) {
                    let Some(control) = fresh
                        .0
                        .returning
                        .iter()
                        .find(|e| e.key == key || e.key == inverse)
                    else {
                        continue;
                    };
                    if !control.witness.admit(_pose, a, b, m) {
                        continue;
                    }
                    control.witness.queried.fetch_add(1, Ordering::Relaxed);
                }
                // Keep matching points and warm starts. Recompute both hulls'
                // supporting features and separating axis in their current poses.
                m.mark_shapes_deformed();
                fresh.0.rebuilt.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

impl RapierBackend {
    /// Whether the last solve retained active geometry for this single-collider pair.
    pub fn has_box_contact(&self, a: BodyId, b: BodyId) -> bool {
        let collider = |id| {
            let handle = self.bodies.get(&id)?.0;
            match self.world.bodies.get(handle)?.colliders() {
                [c] => Some(*c),
                _ => None,
            }
        };
        collider(a)
            .zip(collider(b))
            .and_then(|(a, b)| self.world.narrow_phase.contact_pair(a, b))
            .is_some_and(|p| p.has_any_active_contact())
    }
}
