// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A grounded hinge retains the island, broad phase, contact manifold and warm impulses.
use super::*;
use rapier3d_f64::utils::OrthonormalBasis;

const MAGIC: &[u8; 8] = b"GZAGR001";
/// Maximum encoded three-body native graph, including its version and checksum.
pub const GROUNDED_REVOLUTE_LIMIT: usize = 128 * 1024;

pub(super) fn graph_codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit((GROUNDED_REVOLUTE_LIMIT - 40) as u64)
        .reject_trailing_bytes()
}

/// Native impulse on the link during the most recent 1 ms contact solve. The
/// normal points along +Z and the tangential component uses world axes.
/// Rapier retains a separate normal impulse to warm-start the following solve.
/// A sleeping island may retain the last solution until it wakes.
#[derive(Clone, Copy, Default, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundReaction {
    pub active: bool,
    pub normal_impulse: f64,
    pub warmstart_normal_impulse: f64,
    pub tangent_impulse: [f64; 3],
}

/// Native impulse on the hinge link from its companion sphere during the last solve.
/// The signed normal direction and tangential impulse are expressed in world axes.
#[derive(Clone, Copy, Default, Debug, PartialEq, Serialize, Deserialize)]
pub struct MovingReaction {
    pub active: bool,
    pub normal: [f64; 3],
    pub normal_impulse: f64,
    pub warmstart_normal_impulse: f64,
    pub tangent_impulse: [f64; 3],
}

// Early two-sphere images used a zero-density offset collider on each hinge
// link. Rapier transformed its zero-mass frame into the body frame and added
// the authored inertia, which can re-diagonalize the same physical tensor.
// Match that exact construction while retaining the original native solver state.
fn legacy_pair_link_mass(
    body: &RigidBody,
    collider: &rapier3d_f64::geometry::Collider,
    expected_body: &RigidBody,
    expected_collider: &rapier3d_f64::geometry::Collider,
) -> bool {
    let Some(RigidBodyAdditionalMassProps::MassProps(authored)) = expected_body
        .mass_properties()
        .additional_local_mprops
        .as_deref()
    else {
        return false;
    };
    let Some(pose) = expected_collider.position_wrt_parent() else {
        return false;
    };
    let zero = expected_collider.shape().mass_properties(0.0);
    *pose != Pose::IDENTITY
        && collider.mass_properties() == zero
        && body.mass_properties().local_mprops == zero.transform_by(pose) + *authored
}

pub(super) fn validate_contacts(world: &PhysicsWorld) -> Result<(), PhysicsError> {
    validate_manifold_points(world, 1)
}

pub(super) fn validate_manifold_points(world: &PhysicsWorld, maximum: usize) -> Result<(), PhysicsError> {
    for pair in world.narrow_phase.contact_pairs() {
        if !world.colliders.contains(pair.collider1)
            || !world.colliders.contains(pair.collider2)
            || pair.collider1 == pair.collider2
        {
            return Err(invalid());
        }
        for manifold in pair.solver_manifolds() {
            if manifold.points.len() > maximum
                || manifold.data.solver_contacts.len() > manifold.points.len()
                || !manifold.data.normal.is_finite()
                || !manifold.data.friction.is_finite()
                || manifold.data.friction < 0.
                || !manifold.data.restitution.is_finite()
                || manifold.points.iter().any(|p| {
                    !p.local_p1.is_finite()
                        || !p.local_p2.is_finite()
                        || !p.dist.is_finite()
                        || !p.data.impulse.is_finite()
                        || !p.data.warmstart_impulse.is_finite()
                        || !p.data.warmstart_twist_impulse.is_finite()
                        || !p.data.tangent_impulse.iter().all(|v| v.is_finite())
                        || !p
                            .data
                            .warmstart_tangent_impulse
                            .iter()
                            .all(|v| v.is_finite())
                        || !p.data.warmstart_tangent_world.is_finite()
                        || !p.data.solver_dp1.is_finite()
                        || !p.data.solver_dp2.is_finite()
                })
                || manifold.data.solver_contacts.iter().any(|p| {
                    (p.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize
                        >= manifold.points.len()
                        || !p.anchor1.is_finite()
                        || !p.anchor2.is_finite()
                        || !p.dist.is_finite()
                        || !p.tangent_velocity.is_finite()
                })
            {
                return Err(invalid());
            }
        }
    }
    Ok(())
}

impl RapierBackend {
    fn hinge_ground(&self) -> Result<BodyId, PhysicsError> {
        self.bodies
            .iter()
            .find_map(|(&id, entry)| entry.1.collider.shape().as_halfspace().map(|_| id))
            .ok_or_else(unsupported)
    }

    pub fn ground_reaction(
        &self,
        link: BodyId,
        ground: BodyId,
    ) -> Result<GroundReaction, PhysicsError> {
        if self.hinge_ground()? != ground {
            return Err(unsupported());
        }
        let r = self.link_reaction(link, ground)?;
        Ok(GroundReaction {
            active: r.active,
            normal_impulse: r.normal_impulse,
            warmstart_normal_impulse: r.warmstart_normal_impulse,
            tangent_impulse: r.tangent_impulse,
        })
    }

    pub(super) fn link_reaction(
        &self,
        link: BodyId,
        other: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        if self
            .bodies
            .get(&link)
            .is_none_or(|b| b.1.articulation != Some(link))
        {
            return Err(unsupported());
        }
        self.contact_reaction(link, other)
    }

    pub(super) fn contact_reaction(
        &self,
        link: BodyId,
        other: BodyId,
    ) -> Result<MovingReaction, PhysicsError> {
        let collider = |id| {
            let (h, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            let rb = self.world.bodies.get(*h).ok_or_else(invalid)?;
            let [co] = rb.colliders() else {
                return Err(unsupported());
            };
            Ok(*co)
        };
        let link = collider(link)?;
        let mut result = MovingReaction::default();
        if let Some(pair) = self.world.narrow_phase.contact_pair(link, collider(other)?) {
            result.active = pair.has_any_active_contact();
            let mut tangent_impulse = Vector::ZERO;
            for manifold in pair.solver_manifolds() {
                // The generic multibody contact solver reports tangent impulse
                // components in this basis, defined on collider 1.
                let n = -manifold.data.normal;
                let t = n.orthonormal_vector();
                let b = n.cross(t);
                let sign = if pair.collider1 == link { 1. } else { -1. };
                result.normal = (n * sign).to_array();
                for i in crate::partition::solved_point_indices(manifold) {
                    let p = &manifold.points[i].data;
                    result.normal_impulse += p.impulse;
                    result.warmstart_normal_impulse += p.warmstart_impulse;
                    tangent_impulse += sign * (t * p.tangent_impulse.x + b * p.tangent_impulse.y);
                }
            }
            result.tangent_impulse = tangent_impulse.to_array();
        }
        Ok(result)
    }

    pub fn checkpoint_grounded_revolute(&self) -> Result<Vec<u8>, PhysicsError> {
        self.checkpoint_companion(Companion::Ground(self.hinge_ground()?), MAGIC)
    }

    pub(super) fn checkpoint_companion(
        &self,
        companion: Companion,
        magic: &[u8; 8],
    ) -> Result<Vec<u8>, PhysicsError> {
        let layout = self.revolute_layout(Some(companion))?;
        let pair = matches!(companion, Companion::Pair(_));
        if self.joints.open && !pair
            || !self.joints.open && self.world.integration_parameters.dt != STEP.as_secs_f64()
        {
            return Err(unsupported());
        }
        let joint = &self.joints.records[&layout.joint];
        validate_contacts(&self.world)?;
        if !joint.position.is_finite()
            || self.world.bodies.iter().any(|(_, rb)| {
                !rb.translation().is_finite()
                    || rotation(rb.rotation().to_array()).is_err()
                    || !rb.linvel().is_finite()
                    || !rb.angvel().is_finite()
            })
        {
            return Err(invalid());
        }
        let state = (
            layout.construction,
            joint.position,
            self.joints.efforts.get(&layout.joint),
            &self.world,
        );
        let data = if pair {
            let observer = self.pair_observer.ok_or_else(unsupported)?;
            graph_codec().serialize(&(
                state.0,
                state.1,
                state.2,
                self.joints.open,
                observer.initial_velocity,
                state.3,
            ))
        } else {
            graph_codec().serialize(&state)
        }
        .map_err(|_| invalid())?;
        let mut bytes = Vec::with_capacity(data.len() + 40);
        bytes.extend_from_slice(magic);
        bytes.extend_from_slice(&Sha256::digest(&data));
        bytes.extend_from_slice(&data);
        Ok(bytes)
    }

    pub(super) fn validate_companion_bodies(
        &self,
        world: &PhysicsWorld,
        companion: Companion,
        sphere_island: bool,
    ) -> Result<(), PhysicsError> {
        for (&id, (h, construction, ..)) in &self.bodies {
            let rb = world.bodies.get(*h).ok_or_else(invalid)?;
            let [collider] = rb.colliders() else {
                return Err(invalid());
            };
            let c = world.colliders.get(*collider).ok_or_else(invalid)?;
            let expected_rb = &self.world.bodies[*h];
            let expected_c = &self.world.colliders[expected_rb.colliders()[0]];
            let independent = matches!(companion, Companion::Pair(ids) if ids.contains(&id));
            if c.parent() != Some(*h)
                || (matches!(companion, Companion::Sphere(_) | Companion::Pair(_))
                    && (rb.additional_solver_iterations()
                        != expected_rb.additional_solver_iterations()
                        || rb.is_ccd_enabled() != expected_rb.is_ccd_enabled()))
                || rb.body_type() != expected_rb.body_type()
                || (!rb.is_enabled() && !independent && !sphere_island)
                || (sphere_island && rb.is_enabled() != independent)
                || rb.user_data != expected_rb.user_data
                || rb.locked_axes() != expected_rb.locked_axes()
                || rb.linear_damping() != expected_rb.linear_damping()
                || rb.angular_damping() != expected_rb.angular_damping()
                || rb.gravity_scale() != expected_rb.gravity_scale()
                // Rapier recomputes total mass when a collider is disabled. The
                // pair worker never solves a disabled link, and merge overwrites it
                // with the authoritative hinge body before reenabling the collider.
                || ((!sphere_island || independent)
                    && rb.mass_properties().local_mprops != expected_rb.mass_properties().local_mprops
                    && !(matches!(companion, Companion::Pair(_)) && !independent
                        && legacy_pair_link_mass(rb, c, expected_rb, expected_c)))
                || rb.mass_properties().additional_local_mprops
                    != expected_rb.mass_properties().additional_local_mprops
                || rb.activation().normalized_linear_threshold
                    != expected_rb.activation().normalized_linear_threshold
                || rb.activation().angular_threshold != expected_rb.activation().angular_threshold
                || rb.user_force() != Vector::ZERO
                || rb.user_torque() != Vector::ZERO
                || (matches!(companion, Companion::Sphere(_) | Companion::Pair(_))
                    && c.position_wrt_parent()
                        .is_none_or(|local| *c.position() != rb.position() * local))
                || c.position_wrt_parent() != expected_c.position_wrt_parent()
                || c.friction() != expected_c.friction()
                || c.restitution() != expected_c.restitution()
                || c.friction_combine_rule() != expected_c.friction_combine_rule()
                || c.restitution_combine_rule() != expected_c.restitution_combine_rule()
                || c.active_hooks() != expected_c.active_hooks()
                || c.user_data != expected_c.user_data
                || c.collision_groups() != expected_c.collision_groups()
                || c.solver_groups() != expected_c.solver_groups()
                || c.active_collision_types() != expected_c.active_collision_types()
                || c.is_sensor()
                || c.is_enabled() != rb.is_enabled()
                || c.contact_skin() != expected_c.contact_skin()
                || rotation(rb.rotation().to_array()).is_err()
                || !rb.position().translation.is_finite()
                || !rb.linvel().is_finite()
                || !rb.angvel().is_finite()
                || (if matches!(companion, Companion::Ground(ground) if id == ground) {
                    rb.position() != expected_rb.position()
                        || rb.linvel() != Vector::ZERO
                        || rb.angvel() != Vector::ZERO
                        || c.position() != expected_c.position()
                        || c.shape()
                            .as_halfspace()
                            .is_none_or(|s| s.normal != Vector::Z)
                } else {
                    !rb.is_dynamic()
                        || c.shape().as_ball().map(|s| s.radius)
                            != construction.collider.shape().as_ball().map(|s| s.radius)
                })
            {
                return Err(invalid());
            }
        }
        Ok(())
    }

    /// Restore the bounded native graph from a trusted, matching local worker.
    /// The outer construction and ground identity must agree before the graph
    /// can replace the freshly authored destination.
    pub fn restore_grounded_revolute(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.restore_companion(bytes, Companion::Ground(self.hinge_ground()?), MAGIC)
    }

    pub(super) fn restore_companion(
        &mut self,
        bytes: &[u8],
        companion: Companion,
        magic: &[u8; 8],
    ) -> Result<(), PhysicsError> {
        if !(40..=GROUNDED_REVOLUTE_LIMIT).contains(&bytes.len())
            || &bytes[..8] != magic
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
        {
            return Err(invalid());
        }
        let layout = self.revolute_layout(Some(companion))?;
        if !self.joints.open || !self.joints.efforts.is_empty() {
            return Err(unsupported());
        }
        let mut raw = &bytes[40..];
        let construction: [u8; 32] = graph_codec()
            .deserialize_from(&mut raw)
            .map_err(|_| invalid())?;
        if construction != layout.construction {
            return Err(invalid());
        }
        let (position, effort): (f64, Option<f64>) = graph_codec()
            .deserialize_from(&mut raw)
            .map_err(|_| invalid())?;
        let (cold, previous_velocity) = if matches!(companion, Companion::Pair(_)) {
            if self.pair_observer.is_none() {
                return Err(unsupported());
            }
            graph_codec()
                .deserialize_from(&mut raw)
                .map_err(|_| invalid())?
        } else {
            (false, [0.; 3])
        };
        let world: PhysicsWorld = graph_codec()
            .deserialize_from(&mut raw)
            .map_err(|_| invalid())?;
        if !finite(previous_velocity) {
            return Err(invalid());
        }
        if !raw.is_empty()
            || !position.is_finite()
            || effort.is_some_and(|e| !e.is_finite())
            || world.gravity != self.world.gravity
            || world.integration_parameters
                != (IntegrationParameters {
                    dt: if cold {
                        self.world.integration_parameters.dt
                    } else {
                        STEP.as_secs_f64()
                    },
                    ..self.world.integration_parameters
                })
        {
            return Err(invalid());
        }
        let (tree, idx) = world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?;
        let fresh = self
            .world
            .multibody_joints
            .get(layout.handle)
            .ok_or_else(invalid)?
            .0;
        if idx != 1
            || tree.num_links() != 2
            || tree.ndofs() != 7
            || tree.link(1).ok_or_else(invalid)?.joint.coords()[3].to_bits() != position.to_bits()
            || !tree.generalized_velocity().iter().all(|v| v.is_finite())
            || !tree.couplings().is_empty()
            || (cold
                && (tree.damping() != fresh.damping()
                    || tree.generalized_velocity() != fresh.generalized_velocity()))
            || (!cold && (tree.damping().len() != 7 || tree.damping().iter().any(|x| *x != 0.)))
            || [tree.armature(), tree.frictions()]
                .into_iter()
                .any(|v| v.len() != 7 || v.iter().any(|x| *x != 0.))
        {
            return Err(invalid());
        }
        self.validate_companion_bodies(&world, companion, false)?;
        validate_contacts(&world)?;
        let original = std::mem::replace(&mut self.world, world);
        if self.revolute_layout(Some(companion)).is_err() {
            self.world = original;
            return Err(invalid());
        }
        self.joints
            .records
            .get_mut(&layout.joint)
            .expect("validated joint")
            .position = position;
        self.joints.open = cold;
        if let Some(observer) = &mut self.pair_observer {
            observer.initial_velocity = previous_velocity;
        }
        if let Some(e) = effort {
            self.joints.efforts.insert(layout.joint, e);
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../../tests/private/grounded_revolute.rs"]
mod tests;
