// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cache-free rigid-body construction and contact measurements at tick barriers.
use super::*;
use std::collections::HashMap;

/// Rebuilt at the next topology-changing boundary. Most construction sites
/// allocate public IDs and native slots together. Every candidate is verified
/// against the authoritative map, including the native slot generation.
#[derive(Default)]
pub(super) struct ContactBodyIndex {
    displaced: std::cell::RefCell<Vec<(RigidBodyHandle, Option<BodyId>)>>,
    full: std::cell::OnceCell<HashMap<RigidBodyHandle, BodyId>>,
}
impl ContactBodyIndex {
    pub(super) fn get(&self, solver: &RapierBackend, collider: ColliderHandle) -> Option<BodyId> {
        let handle = solver.world.colliders.get(collider)?.parent()?;
        let candidate = BodyId::from_raw(u64::from(handle.into_raw_parts().0));
        if solver
            .bodies
            .get(&candidate)
            .is_some_and(|entry| entry.0 == handle)
        {
            return Some(candidate);
        }
        if solver.bodies.len() <= 8 {
            return solver
                .bodies
                .iter()
                .find_map(|(&id, entry)| (entry.0 == handle).then_some(id));
        }
        if let Some(all) = self.full.get() {
            return all.get(&handle).copied();
        }
        let mut displaced = self.displaced.borrow_mut();
        if let Some((_, id)) = displaced.iter().find(|(h, _)| *h == handle) {
            return *id;
        }
        // Scan once for the handful of recycled slots in a sparse contact set.
        // For many displaced handles a full reverse index amortizes the search.
        if displaced.len() < 4 {
            let id = solver
                .bodies
                .iter()
                .find_map(|(&id, entry)| (entry.0 == handle).then_some(id));
            displaced.push((handle, id));
            return id;
        }
        self.full
            .get_or_init(|| {
                solver
                    .bodies
                    .iter()
                    .map(|(&id, entry)| (entry.0, id))
                    .collect()
            })
            .get(&handle)
            .copied()
    }
}

/// The native solver writes the impulse of this step only to the selected
/// geometric points. Other retained points may still hold a previous impulse.
/// Traverse in canonical geometric order, including the common all-active path.
pub(super) fn solved_point_indices(
    manifold: &rapier3d_f64::geometry::ContactManifold,
) -> impl Iterator<Item = usize> + '_ {
    let rows = &manifold.data.solver_contacts;
    let all = rows.len() == manifold.points.len();
    (0..manifold.points.len()).filter(move |&index| {
        all || rows.iter().any(|row| {
            (row.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize == index
        })
    })
}

pub(super) fn normal_impulse(pair: &rapier3d_f64::geometry::ContactPair) -> f64 {
    pair.solver_manifolds().iter().fold(0.0, |sum, manifold| {
        sum + solved_point_indices(manifold)
            .map(|i| manifold.points[i].data.impulse)
            .sum::<f64>()
    })
}

/// One narrow-phase pair of public bodies. A cached pair may be inactive.
#[derive(Clone, Copy, Debug)]
pub struct RigidContactObservation {
    pub a: BodyId,
    pub b: BodyId,
    pub active: bool,
    /// Sum of normal impulse magnitudes from the pair’s solved points this step (N s).
    pub normal_impulse: f64,
}

mod bounds;
mod contact_patch;
pub(crate) use contact_patch::manifold_reaction;
pub use contact_patch::{
    ContactWarmstart, NormalReaction, RigidContactPatch, RigidPatchPoint, StoredContactPoint,
};
mod box_forecast;
pub use bounds::{PartitionBounds, PartitionBoxBounds};
pub use box_forecast::predicts_box_contact;

pub fn partition_prediction_distance() -> f64 {
    IntegrationParameters::default().prediction_distance()
}

impl RapierBackend {
    /// Admit an isolated box to the distributed profile. This solver recomputes
    /// contact geometry every step while retaining native per-point warm-start impulses.
    pub fn spawn_awake_frictionless_box(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        prescribed: bool,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_partition_box(spec, rotation, prescribed, 0.)
    }

    /// Insert a centered finite box carrying an isotropic Coulomb coefficient.
    /// A pair uses the smaller of the two surface values.
    pub fn spawn_partition_box(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        prescribed: bool,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        if !matches!(spec.shape, Shape::Box { .. }) {
            return Err(PhysicsError::InvalidValue);
        }
        self.spawn_partition_primitive(spec, rotation, prescribed, friction)
    }

    /// Insert one admitted homogeneous primitive into the native checkpoint profile.
    pub fn spawn_partition_primitive(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        prescribed: bool,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        if !friction.is_finite() || friction < 0. || (prescribed && spec.mass.is_none()) {
            return Err(PhysicsError::InvalidValue);
        }
        let id = self.spawn_primitive(
            spec,
            rotation,
            None,
            PrimitiveMode::AwakeBox {
                prescribed,
                friction,
            },
        )?;
        // An empty manifold may otherwise be reused while boxes approach. Its
        // update then depends on broad-phase history lost in a pre-contact cold merge.
        self.world.integration_parameters.contact_recycling = false;
        // Use the same native kinematic integrator before and after a partition
        // gains a dynamic neighbor. The fleet formula rounds translations differently.
        self.materialize_fleet();
        Ok(id)
    }

    pub(super) fn require_separated_box(
        &self,
        shape: &Collider,
        position: [f64; 3],
        orientation: Rotation,
    ) -> Result<(), PhysicsError> {
        let pose = Pose::from_parts(Vector::from(position), orientation);
        let candidate = PartitionBounds::from_collider(shape, &(pose * *shape.position()))?;
        let bounds = candidate.padded_aabb(self.world.integration_parameters.prediction_distance());
        for (_, other) in self.world.colliders.iter() {
            if other.parent().is_none()
                && let Some(plane) = other.shape().as_halfspace()
            {
                if !candidate.above_plane(other.translation().to_array(), plane.normal.to_array()) {
                    return Err(PhysicsError::TransitionDeferred {
                        reason: "box starts below the shared ground",
                    });
                }
                continue;
            }
            let other_pose = if let Some(parent) = other.parent() {
                let body = &self.world.bodies[parent];
                let pose =
                    Pose::from_parts(self.current_translation(parent, body), *body.rotation());
                pose * other.position_wrt_parent().expect("parented collider")
            } else {
                *other.position()
            };
            if bounds.intersects(&other.shape().compute_aabb(&other_pose))
                && ((other.shape().as_cuboid().is_none()
                    && other.shape().as_ball().is_none()
                    && other.shape().as_capsule().is_none()
                    && other.shape().as_cylinder().is_none())
                    || candidate
                        .predicts_overlap(&PartitionBounds::from_collider(other, &other_pose)?))
            {
                return Err(PhysicsError::TransitionDeferred {
                    reason: "box intersects an existing body's contact prediction bounds",
                });
            }
        }
        Ok(())
    }

    /// Resolve a complete contact pass. The lookup is valid only while body
    /// topology is fixed. Hidden roots and removed handles have no public ID.
    pub(super) fn contact_body_lookup(&self) -> impl Fn(ColliderHandle) -> Option<BodyId> + '_ {
        let index = ContactBodyIndex::default();
        move |collider| index.get(self, collider)
    }

    /// Includes speculative pairs, allowing transfer to detect any contact
    /// history even when its solver impulse or active contact count is zero.
    pub fn rigid_contact_observations(&self) -> Vec<RigidContactObservation> {
        let mut pairs = self.world.narrow_phase.contact_pairs().peekable();
        if pairs.peek().is_none() {
            return Vec::new();
        }
        let get_id = self.contact_body_lookup();
        let mut observations: Vec<_> = pairs
            .filter_map(|pair| Self::observe_rigid_pair(pair, &get_id))
            .collect();
        observations.sort_unstable_by_key(|p| (p.a, p.b));
        observations
    }
    pub(super) fn observe_rigid_pair(
        pair: &rapier3d_f64::geometry::ContactPair,
        get_id: &impl Fn(ColliderHandle) -> Option<BodyId>,
    ) -> Option<RigidContactObservation> {
        let a = get_id(pair.collider1)?;
        let b = get_id(pair.collider2)?;
        Some(RigidContactObservation {
            a: a.min(b),
            b: a.max(b),
            active: pair.has_any_active_contact(),
            normal_impulse: normal_impulse(pair),
        })
    }
}

#[cfg(test)]
#[path = "../tests/private/partition.rs"]
mod tests;

#[cfg(all(test, feature = "partition-checkpoint"))]
#[path = "../tests/private/loaded_impulses.rs"]
mod loaded_impulses;
