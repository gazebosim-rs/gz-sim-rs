// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Center the normal resultant of a cold, axially symmetric cap impact before solving.
use super::*;
use crate::contact_prediction::dynamic::{free_sweep, swept_aabb};
use rapier3d_f64::parry::bounding_volume::{Aabb, BoundingVolume};
use std::cell::OnceCell;
mod index;
use index::Index;
pub(super) use index::Registry;

pub(super) struct Impact {
    cylinder: ColliderHandle,
    floor: ColliderHandle,
    center: Vector,
    outward: Vector,
    original: ActiveHooks,
    centered: std::sync::atomic::AtomicBool,
}
pub(super) fn primitive(c: &Collider) -> Option<(Pose, &dyn rapier3d_f64::parry::shape::Shape)> {
    if let Some(compound) = c.shape().as_compound() {
        let [(local, shape)] = compound.shapes() else {
            return None;
        };
        Some((*local, shape.as_ref()))
    } else {
        Some((Pose::identity(), c.shape()))
    }
}
const ROUND: f64 = 256. * f64::EPSILON;
impl Impact {
    pub(super) fn apply(&self, ctx: &mut ContactModificationContext) {
        let forward = ctx.collider1 == self.cylinder && ctx.collider2 == self.floor;
        if !forward && !(ctx.collider2 == self.cylinder && ctx.collider1 == self.floor) {
            return;
        }
        let Some(m) = ctx.rigid_mut() else {
            return;
        };
        let rows = &mut *m.solver_contacts;
        // A penetrating plane, an edge manifold, an old warm row or a conveyor
        // has additional history or physics outside the centered impact model.
        let normal = if forward { -*m.normal } else { *m.normal };
        if (normal - self.outward).length() > ROUND || *m.restitution <= 0. || rows.len() != 4 {
            return;
        }
        let first = rows[0];
        let tol = ROUND * (1. + self.center.length());
        if rows.iter().any(|r| {
            r.contact_id[0] & rapier3d_f64::geometry::NEW_CONTACT_BIT == 0
                || r.tangent_velocity != Vector::ZERO
                || (r.dist - first.dist).abs() > tol
        }) {
            return;
        }
        *m.normal = if forward { -self.outward } else { self.outward };
        rows.truncate(1);
        let project = |p: Vector| self.center + self.outward * (p - self.center).dot(self.outward);
        rows[0].anchor1 = project(first.anchor1);
        rows[0].anchor2 = project(first.anchor2);
        self.centered
            .store(true, std::sync::atomic::Ordering::Relaxed);
        // Rapier localizes these exact anchors into solver_dp1/2 after the
        // hook and writes the solved impulse to the selected point's ledger.
    }
}
impl RapierBackend {
    /// Enable cold axial cylinder-cap impacts against the interior of a fixed box face.
    /// Other manifolds, resting contacts and off-axis motion use their normal native rows.
    pub fn set_axial_cap_impact(&mut self, enabled: bool) {
        self.axial_caps = enabled.then(|| Registry::from_backend(self));
    }
    pub(super) fn begin_axial_cap_impacts(&mut self) -> Vec<Impact> {
        let Some(ids) = &self.axial_caps else {
            return Vec::new();
        };
        let swept = OnceCell::new();
        let found: Vec<_> = ids
            .ids
            .iter()
            .filter_map(|id| self.axial_cap(*id, ids, &swept))
            .collect();
        for hit in &found {
            self.world.colliders[hit.cylinder]
                .set_active_hooks(hit.original | ActiveHooks::MODIFY_SOLVER_CONTACTS);
        }
        found
    }
    pub(super) fn end_axial_cap_impacts(&mut self, found: &[Impact]) {
        for hit in found {
            self.world.colliders[hit.cylinder].set_active_hooks(hit.original);
            // A positive center impulse has no equivalent warm seed at a rim
            // vertex. On the next full native query, discard just this pair's
            // originally cold patch. Leave its committed public impulse readable now.
            if !hit.centered.load(std::sync::atomic::Ordering::Relaxed) {
                continue;
            }
            let applied = self
                .world
                .narrow_phase
                .contact_pair(hit.cylinder, hit.floor)
                .is_some_and(|p| {
                    p.solver_manifolds().iter().any(|m| {
                        let rows = &m.data.solver_contacts;
                        rows.len() == 1
                            && m.points.len() == 4
                            && m.points[(rows[0].contact_id[0]
                                & !rapier3d_f64::geometry::NEW_CONTACT_BIT)
                                as usize]
                                .data
                                .impulse
                                > 0.
                    })
                });
            if applied
                && let Some(pair) = self.world.colliders[hit.cylinder]
                    .parent()
                    .zip(self.world.colliders[hit.floor].parent())
            {
                self.cold_support_pairs.push(pair);
            }
        }
    }
    fn axial_cap(
        &self,
        id: BodyId,
        registry: &Registry,
        swept: &OnceCell<Index>,
    ) -> Option<Impact> {
        let handle = self.bodies.get(&id)?.0;
        let world = &self.world;
        let body = &world.bodies[handle];
        let &[cylinder] = body.colliders() else {
            return None;
        };
        let c = &world.colliders[cylinder];
        let (local, primitive_shape) = primitive(c)?;
        let geom = primitive_shape.as_cylinder()?;
        if !body.is_dynamic()
            || body.is_sleeping()
            || body.is_ccd_enabled()
            || !body.locked_axes().is_empty()
            || self.has_joint(id) != Some(false)
            || c.contact_skin() != 0.
            || c.is_sensor()
            || !c.is_enabled()
        {
            return None;
        }
        let pose = *body.position() * c.position_wrt_parent()? * local;
        let center = pose.translation;
        let axis = pose.rotation * Vector::Y;
        let speed = body.linvel().length();
        let outward = -body.linvel().try_normalize()?;
        let total_acceleration =
            world.gravity * body.gravity_scale() + body.user_force() / body.mass();
        let scale = 1. + center.length() + geom.radius + geom.half_height;
        let position_epsilon = ROUND * scale;
        // Decline small features at large absolute coordinates when the budget
        // would exceed sqrt(machine epsilon) of either characteristic length.
        if position_epsilon > f64::EPSILON.sqrt() * geom.radius.min(geom.half_height) {
            return None;
        }
        let speed_epsilon = position_epsilon / world.integration_parameters.dt;
        if axis.cross(outward).length() * speed > speed_epsilon
            || (center - body.center_of_mass()).length() > ROUND * (1. + center.length())
            || body.angvel().length() * geom.radius.max(geom.half_height) > speed_epsilon
            || body.user_torque() != Vector::ZERO
            || total_acceleration.cross(outward).length()
                > ROUND * (1. + total_acceleration.length())
        {
            return None;
        }
        let p = &world.integration_parameters;
        let dt = p.dt;
        let reach = p.prediction_distance() + (speed + total_acceleration.length() * dt) * dt;
        let bound = primitive_shape.compute_aabb(&pose).loosened(reach);
        // The fixed-face tree is shared across all flights and rebuilt only by structural
        // changes. Only a possible arrival requests a swept index of all moving obstacles.
        if registry
            .floors
            .get_or_init(|| Index::floors(world))
            .query(&bound)
            .next()
            .is_none()
        {
            return None;
        }
        let cap = center - outward * geom.half_height;
        let mut floor = None;
        for other in swept.get_or_init(|| Index::swept(world)).query(&bound) {
            let obstacle = &world.colliders[other];
            if other == cylinder {
                continue;
            }
            let rb = obstacle.parent().map(|h| &world.bodies[h]);
            let kind = rb.map_or(RigidBodyType::Fixed, RigidBody::body_type);
            if !crate::collision::can_exchange_impulses(c, obstacle, kind) {
                continue;
            }
            let loc = rb.map_or(*obstacle.position(), |b| {
                *b.position() * obstacle.position_wrt_parent().unwrap()
            });
            if floor.is_some() || kind != RigidBodyType::Fixed || obstacle.contact_skin() != 0. {
                return None;
            }
            let (part, shape) = primitive(obstacle)?;
            let ext = shape.as_cuboid()?.half_extents;
            let loc = loc * part;
            let local = loc.inverse() * cap;
            let n = loc.rotation.inverse() * outward;
            let k = n.abs().max_position();
            let mut face = Vector::ZERO;
            face[k] = n[k].signum();
            if (n - face).length() * speed > speed_epsilon
                || axis.cross(loc.rotation * face).length() * geom.radius > ROUND * scale
                || (local[k] * face[k] - ext[k]).abs() > reach
            {
                return None;
            }
            // Every point of the cap must be strictly inside this finite face.
            let tol = ROUND * (1. + center.length() + loc.translation.length() + ext.length());
            if tol > f64::EPSILON.sqrt() * geom.radius.min(ext.min_element()) {
                return None;
            }
            if (0..3).any(|i| i != k && local[i].abs() + geom.radius + tol >= ext[i]) {
                return None;
            }
            let gap = local[k] * face[k] - ext[k];
            // A rebound must escape the entire incoming step's possible penetration
            // against gravity with at least one outward step left. Small, reloading
            // contacts keep their original geometry and warm history, even on first touch.
            let acceleration = (-total_acceleration.dot(loc.rotation * face)).max(0.);
            let depth = (speed * dt + acceleration * dt * dt).max(-gap);
            let escape = (2. * acceleration * depth).sqrt() + acceleration * dt;
            let restitution = CoefficientCombineRule::combine(
                c.restitution(),
                obstacle.restitution(),
                c.restitution_combine_rule(),
                obstacle.restitution_combine_rule(),
            );
            if restitution * speed <= escape {
                return None;
            }
            floor = Some((other, loc.rotation * face));
        }
        let (floor, outward) = floor?;
        Some(Impact {
            cylinder,
            floor,
            center,
            outward,
            original: c.active_hooks(),
            centered: false.into(),
        })
    }
}
