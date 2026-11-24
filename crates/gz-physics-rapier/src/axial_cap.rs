// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Center the normal resultant of a cold, axially symmetric cap impact before solving.
use super::*;
use crate::contact_prediction::dynamic::{free_sweep, swept_aabb};
use rapier3d_f64::parry::bounding_volume::BoundingVolume;

#[derive(Clone, Copy)]
pub(super) struct Impact {
    cylinder: ColliderHandle,
    floor: ColliderHandle,
    center: Vector,
    outward: Vector,
    original: ActiveHooks,
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
        rows.truncate(1);
        let project = |p: Vector| self.center + self.outward * (p - self.center).dot(self.outward);
        rows[0].anchor1 = project(first.anchor1);
        rows[0].anchor2 = project(first.anchor2);
        // Rapier localizes these exact anchors into solver_dp1/2 after the
        // hook and writes the solved impulse to the selected point's ledger.
    }
}
impl RapierBackend {
    /// Enable cold axial cylinder-cap impacts against the interior of a fixed box face.
    /// Other manifolds, resting contacts and off-axis motion use their normal native rows.
    pub fn set_axial_cap_impact(&mut self, enabled: bool) {
        self.axial_caps = enabled.then(|| {
            self.bodies
                .iter()
                .filter_map(|(id, e)| primitive(&e.1.collider)?.1.as_cylinder().map(|_| *id))
                .collect()
        });
    }
    pub(super) fn begin_axial_cap_impacts(&mut self) -> Vec<Impact> {
        let Some(ids) = &self.axial_caps else {
            return Vec::new();
        };
        let found: Vec<_> = ids.iter().filter_map(|id| self.axial_cap(*id)).collect();
        for hit in &found {
            self.world.colliders[hit.cylinder]
                .set_active_hooks(hit.original | ActiveHooks::MODIFY_SOLVER_CONTACTS);
        }
        found
    }
    pub(super) fn end_axial_cap_impacts(&mut self, found: &[Impact]) {
        for hit in found {
            self.world.colliders[hit.cylinder].set_active_hooks(hit.original);
        }
    }
    fn axial_cap(&self, id: BodyId) -> Option<Impact> {
        let handle = self.bodies[&id].0;
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
        if speed == 0.
            || axis.cross(outward).length() > ROUND
            || (center - body.center_of_mass()).length() > ROUND * (1. + center.length())
            || body.angvel() != Vector::ZERO
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
        let cap = center - outward * geom.half_height;
        let mut floor = None;
        for (other, obstacle) in world.colliders.iter() {
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
            let (v, margin) = if let Some(b) = rb.filter(|b| !b.is_fixed()) {
                free_sweep(
                    b,
                    obstacle.shape(),
                    &loc,
                    world.gravity,
                    p.max_linear_velocity(),
                    dt,
                )
            } else {
                (Vector::ZERO, 0.)
            };
            let swept = swept_aabb(
                obstacle.shape(),
                &loc,
                v,
                margin + obstacle.contact_skin(),
                dt,
            );
            if !bound.intersects(&swept) {
                continue;
            }
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
            if (n - face).length() > ROUND || (local[k] * face[k] - ext[k]).abs() > reach {
                return None;
            }
            // Every point of the cap must be strictly inside this finite face.
            let tol = ROUND * (1. + center.length() + ext.length());
            if (0..3).any(|i| i != k && local[i].abs() + geom.radius + tol >= ext[i]) {
                return None;
            }
            floor = Some(other);
        }
        Some(Impact {
            cylinder,
            floor: floor?,
            center,
            outward,
            original: c.active_hooks(),
        })
    }
}
