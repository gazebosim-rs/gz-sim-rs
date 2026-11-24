// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resultant pressure impulse of a reflection-symmetric finite sole patch.
use super::*;
use gz_physics::{Bearing, PlanarImpactResponse, PlanarPatchImpact};

/// A distributed flat sole reaction represented by its resultant at the
/// center of pressure. Individual manifold point loads need not be unique.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarImpactReceipt {
    pub resultant: CornerImpactReceipt,
    pub bearing: Bearing,
    pub supported_x: [f64; 2],
    pub pressure_x: f64,
}
struct Patch {
    site: ArticulatedContactSite,
    point: Vector,
    normal: Vector,
    tangent: Vector,
    deck_velocity: Vector,
    boxes: RegisteredBoxes,
    input: PlanarPatchImpact,
    response: PlanarImpactResponse,
    gap: f64,
}
impl RapierBackend {
    pub(super) fn planar_mass_properties(
        &self,
        tree: &Multibody,
        shoe: &RigidBody,
        axis: Vector,
    ) -> Option<(f64, Vector, f64)> {
        let eps = 256. * f64::EPSILON;
        let (mass, com) = self.locked_mass_center(tree, shoe)?;
        let ia = self.locked_inertia_action(tree, com, axis);
        let inertia = ia.dot(axis);
        if !ia.is_finite() || inertia <= 0. || ia.cross(axis).length() > eps * inertia {
            return None;
        }
        Some((mass, com, inertia))
    }
    /// Check all principal directions about one measured aggregate COM.
    pub(super) fn spherical_mass_properties(
        &self,
        tree: &Multibody,
        shoe: &RigidBody,
    ) -> Option<(f64, Vector, f64)> {
        let (mass, center) = self.locked_mass_center(tree, shoe)?;
        let axes = [Vector::X, Vector::Y, Vector::Z];
        let mut action = [Vector::ZERO; 3];
        for link in tree.links() {
            let b = &self.world.bodies[link.rigid_body_handle()];
            let m = &b.mass_properties().local_mprops;
            let q = *b.rotation() * m.principal_inertia_local_frame;
            let inv = q.inverse();
            let inertia = m.inv_principal_inertia.recip();
            let r = b.center_of_mass() - center;
            for (i, axis) in axes.into_iter().enumerate() {
                action[i] += q * (inertia * (inv * axis)) + b.mass() * r.cross(axis.cross(r));
            }
        }
        let inertia = action[0].x;
        for (v, axis) in action.into_iter().zip(axes) {
            let moment = v.dot(axis);
            if !v.is_finite()
                || moment <= 0.
                || v.cross(axis).length() > 256. * f64::EPSILON * moment
                || (moment - inertia).abs() > 128. * f64::EPSILON * inertia
            {
                return None;
            }
        }
        Some((mass, center, inertia))
    }
    pub(crate) fn locked_mass_center(
        &self,
        tree: &Multibody,
        shoe: &RigidBody,
    ) -> Option<(f64, Vector)> {
        let w = &self.world;
        let mut mass = 0.;
        let mut weighted = Vector::ZERO;
        for l in tree.links() {
            let b = &w.bodies[l.rigid_body_handle()];
            if !b.is_dynamic()
                || (!l.is_root() && (l.joint.ndofs() != 0 || !passive_contact_coordinate(l)))
            {
                return None;
            }
            mass += b.mass();
            weighted += b.mass() * (b.center_of_mass() - shoe.center_of_mass());
        }
        if mass <= 0. || !mass.is_finite() {
            return None;
        }
        Some((mass, shoe.center_of_mass() + weighted / mass))
    }
    pub(crate) fn locked_inertia_action(
        &self,
        tree: &Multibody,
        com: Vector,
        axis: Vector,
    ) -> Vector {
        let w = &self.world;
        let mut ia = Vector::ZERO;
        for l in tree.links() {
            let b = &w.bodies[l.rigid_body_handle()];
            let m = &b.mass_properties().local_mprops;
            let q = *b.rotation() * m.principal_inertia_local_frame;
            let r = b.center_of_mass() - com;
            ia += q * (m.inv_principal_inertia.recip() * (q.inverse() * axis))
                + b.mass() * r.cross(axis.cross(r));
        }
        ia
    }
    fn patch_candidate(
        &self,
        foot: BodyId,
        pad: BodyId,
        displacement: Vector,
        velocity_increment: Vector,
        allowance: f64,
    ) -> Result<Option<Patch>, PhysicsError> {
        let Some(boxes) = self.registered_boxes(foot, pad)? else {
            return Ok(None);
        };
        let w = &self.world;
        let shoe = &w.bodies[boxes.foot];
        let deck = &w.bodies[boxes.pad];
        let index = w
            .multibody_joints
            .rigid_body_link(boxes.foot)
            .expect("registered tree")
            .multibody;
        let tree = w
            .multibody_joints
            .get_multibody(index)
            .ok_or(PhysicsError::InvalidJoint)?;
        // Reflection symmetry suppresses transverse motion and coupling to
        // roll or yaw. The observable is a resultant; transverse pressure
        // distributions satisfying the same momentum equations are equivalent.
        let frame = boxes.pad_pose;
        let tangent = frame.rotation * Vector::X;
        let axis = frame.rotation * Vector::Y;
        let normal = frame.rotation * Vector::Z;
        let rel = frame.rotation.inverse() * (shoe.linvel() - deck.linvel() + velocity_increment);
        let eps = 256. * f64::EPSILON;
        if deck.angvel() != Vector::ZERO
            || rel.y.abs() > eps * (1. + rel.length())
            || rel.z >= -eps * (1. + rel.length())
        {
            return Ok(None);
        }
        let basis = frame.rotation.inverse() * boxes.sole_pose.rotation;
        if [Vector::X, Vector::Y, Vector::Z]
            .into_iter()
            .any(|u| (basis * u - u).length() > eps)
        {
            return Ok(None);
        }
        if tree.links().any(|l| {
            let b = &w.bodies[l.rigid_body_handle()];
            b.angvel() != Vector::ZERO
                || (b.linvel() - shoe.linvel()).length() > eps * (1. + shoe.linvel().length())
        }) {
            return Ok(None);
        }
        let Some((mass, com, inertia)) = self.planar_mass_properties(tree, shoe, axis) else {
            return Ok(None);
        };
        let c = frame.inverse_transform_point(com + displacement);
        let loc = frame.inverse_transform_point(boxes.sole_pose.translation + displacement);
        let h = boxes.sole_half;
        let p = boxes.pad_half;
        let tol = eps
            * (1.
                + com.abs().max_element()
                + frame.translation.abs().max_element()
                + h.max_element()
                + p.max_element());
        let gap = loc.z - h.z - p.z;
        let y0 = (-p.y).max(loc.y - h.y);
        let y1 = p.y.min(loc.y + h.y);
        if gap.abs() > allowance + tol || y1 - y0 <= tol || (0.5 * (y1 + y0) - c.y).abs() > tol {
            return Ok(None);
        }
        let span = [(-p.x).max(loc.x - h.x) - c.x, p.x.min(loc.x + h.x) - c.x];
        if span[1] - span[0] <= tol {
            return Ok(None);
        }
        let input = PlanarPatchImpact {
            mass,
            pitch_inertia: inertia,
            com_height: c.z - loc.z + h.z,
            supported_x: span,
            tangent_speed: rel.x,
            closing_speed: -rel.z,
            friction: boxes.friction,
        };
        let Ok(response) = input.solve() else {
            return Ok(None);
        };
        let point =
            frame * Vector::new(c.x + response.center_of_pressure, c.y, loc.z - h.z) - displacement;
        let origin =
            Vector::from_array(self.state(foot).ok_or(PhysicsError::MissingBody)?.position);
        let site = ArticulatedContactSite {
            body: foot,
            local_point: (shoe.rotation().inverse() * (point - origin)).to_array(),
        };
        Ok(Some(Patch {
            site,
            point,
            normal,
            tangent,
            deck_velocity: deck.linvel(),
            boxes,
            input,
            response,
            gap,
        }))
    }
    pub(super) fn patch_colliders(
        &self,
        foot: BodyId,
        pad: BodyId,
        displacement: Vector,
        dv: Vector,
        allowance: f64,
    ) -> Result<Option<(ColliderHandle, ColliderHandle)>, PhysicsError> {
        Ok(self
            .patch_candidate(foot, pad, displacement, dv, allowance)?
            .map(|p| p.boxes.colliders))
    }
    pub fn planar_at_support_face(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        Ok(self
            .patch_candidate(foot, pad, Vector::ZERO, Vector::ZERO, 0.)?
            .is_some())
    }
    pub(crate) fn apply_planar_at(
        &mut self,
        foot: BodyId,
        pad: BodyId,
        elapsed: Duration,
        allowance: f64,
    ) -> Result<Option<PlanarImpactReceipt>, PhysicsError> {
        let Some(p) = self.patch_candidate(foot, pad, Vector::ZERO, Vector::ZERO, allowance)?
        else {
            return Ok(None);
        };
        let tx = match self.prepare_contact_sites(&[p.site], Some(p.boxes.colliders)) {
            Ok(t) => t,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        if tx.sample().dofs != 6 {
            return Ok(None);
        }
        let before = Vector::from_array(tx.sample().velocities[0]) - p.deck_velocity;
        let incoming = p.tangent * p.input.tangent_speed - p.normal * p.input.closing_speed;
        if before.distance(incoming) > 256. * f64::EPSILON * (1. + incoming.length()) {
            return Ok(None);
        }
        let j = p.tangent * p.response.impulse[0] + p.normal * p.response.impulse[1];
        // Compare the independent aggregate rigid inertia with the full
        // native Jacobian at the actual pressure center before committing.
        let mobility = tx
            .sample()
            .mobility(0, 0)
            .ok_or(PhysicsError::InvalidValue)?;
        let delta = Vector::from_array(mobility.map(|r| Vector::from_array(r).dot(j)));
        let [vx, vz] = p.response.com_velocity;
        let expected = p.tangent * (vx - p.input.com_height * p.response.pitch_rate)
            + p.normal * (vz - p.response.center_of_pressure * p.response.pitch_rate);
        if (before + delta).distance(expected) > 1e-10 * (1. + before.length() + delta.length()) {
            return Ok(None);
        }
        let applied = tx.commit(&[j.to_array()])?;
        Ok(Some(PlanarImpactReceipt {
            bearing: p.response.bearing,
            pressure_x: p.response.center_of_pressure,
            supported_x: p.input.supported_x,
            resultant: CornerImpactReceipt {
                elapsed,
                gap: p.gap,
                foot,
                support: pad,
                point: p.point.to_array(),
                normal: p.normal.to_array(),
                impulse_on_foot: j.to_array(),
                before_relative_velocity: before.to_array(),
                after_relative_velocity: (Vector::from_array(applied.after_velocities[0])
                    - p.deck_velocity)
                    .to_array(),
                friction: p.input.friction,
                sticking: p.response.sticking,
                kinetic_energy_change: applied.kinetic_energy_change,
                dissipated_energy: p.response.dissipated_energy,
            },
        }))
    }
}
