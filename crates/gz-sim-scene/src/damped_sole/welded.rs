// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Express a fixed two-link assembly in the coated lower-link frame.
use super::*;

#[derive(Clone, Copy)]
pub(super) struct GuardBox {
    pub(super) center: DVec3,
    pub(super) rotation: DQuat,
    pub(super) half: DVec3,
}
impl GuardBox {
    pub(super) fn radius(self, body: &SoleBody) -> f64 {
        (body.foot_offset + self.center).length() + self.half.length()
    }
    /// Separate the upper box from the finite support along any one support axis.
    /// Reach bounds the relative path of every point throughout the accepted interval.
    pub(super) fn separated(
        self,
        body: &SoleBody,
        s: &SoleState,
        deck: &SoleDeck,
        reach: f64,
    ) -> Result<(), &'static str> {
        let center = s.center + s.orientation * (body.foot_offset + self.center);
        let delta = deck.orientation.conjugate() * (center - deck.center);
        let rotation = deck.orientation.conjugate() * s.orientation * self.rotation;
        let extent = (rotation * DVec3::X).abs() * self.half.x
            + (rotation * DVec3::Y).abs() * self.half.y
            + (rotation * DVec3::Z).abs() * self.half.z;
        let separation = (delta.abs() - extent - deck.half_extents).max_element();
        let roundoff = 64.
            * f64::EPSILON
            * (1.
                + center.length()
                + deck.center.length()
                + extent.length()
                + deck.half_extents.length());
        if separation > reach + roundoff {
            Ok(())
        } else {
            Err("upper foot collider may contact a support outside the compliant lower face")
        }
    }
}
pub(super) fn configure(
    native: &MultiFidelityBackend,
    id: BodyId,
    body: &mut SoleBody,
) -> Result<(), &'static str> {
    let a = native
        .welded_assembly_state(id)
        .map_err(|_| "requires a floating fixed assembly")?;
    let lower = native.state(id).ok_or("missing lower link")?;
    let orientation = DQuat::from_array(lower.orientation);
    let basis = glam::DMat3::from_quat(orientation);
    let inertia = basis.transpose() * glam::DMat3::from_cols_array_2d(&a.inertia_columns) * basis;
    let principal = DVec3::new(inertia.x_axis.x, inertia.y_axis.y, inertia.z_axis.z);
    let off_axis = inertia - glam::DMat3::from_diagonal(principal);
    if principal.min_element() <= 0.
        || off_axis
            .to_cols_array()
            .iter()
            .any(|v| v.abs() > 1e-11 * principal.min_element())
    {
        return Err("aggregate sole inertia must be diagonal in the coated link frame");
    }
    body.inertia = principal;
    body.mass = a.mass;
    body.foot_offset =
        orientation.conjugate() * (DVec3::from(lower.position) - DVec3::from(a.center));
    Ok(())
}
pub(super) fn state(
    native: &MultiFidelityBackend,
    binding: SoleFoot,
    body: &SoleBody,
) -> Result<SoleState, PhysicsError> {
    let lower = native
        .state(binding.lower())
        .ok_or(PhysicsError::MissingBody)?;
    let mut s = sole_state(body, lower);
    if let SoleFoot::Weld { .. } = binding {
        let a = native.welded_assembly_state(binding.lower())?;
        s.center = a.center.into();
        s.velocity = a.velocity.into();
        s.spin = a.spin.into();
    }
    Ok(s)
}
