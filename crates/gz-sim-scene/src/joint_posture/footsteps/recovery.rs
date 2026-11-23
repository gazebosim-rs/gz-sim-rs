// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Lateral swing-side capture and early touchdown from current native measurements.
use super::{FootstepController, PostureController, trajectory::Target};
use gz_physics::{BodyId, PhysicsBackend, PhysicsError};

impl FootstepController {
    pub(super) fn recover(
        &self,
        t: &mut Target,
        ctl: &PostureController,
        be: &PhysicsBackend,
    ) -> Result<(), PhysicsError> {
        let scheduled = (2700..6600).contains(&t.local_tick);
        let acquired =
            self.plan.support_relative && self.plan.motion == super::FootstepMotion::Step;
        if !scheduled && !(acquired && t.local_tick >= 4700) {
            return Ok(());
        }
        let swing = t.swing_foot;
        let feet = self.support.foot_centers(be)?;
        let displacement = [
            feet[0] - self.initial_feet[0],
            feet[1] - self.initial_feet[1],
        ];
        let risk = if scheduled {
            let (com, v) = ctl.center_of_mass(be)?;
            let (stance, sign) = if swing == 1 { (0, 1.) } else { (1, -1.) };
            let capture = com.y + v.y * (com.z.max(0.) / self.gravity).sqrt();
            let margin = sign * (capture - feet[stance].y) + self.sole_half_width[stance];
            // Enter before the capture point crosses the inside edge of the supporting sole.
            let mut risk = smooth((0.029 - margin) / 0.020);
            if t.local_tick >= 4950 {
                // Retain double support while the COM is ahead of its scheduled return.
                let ahead = sign * (ctl.reference_center_of_mass().y + t.balance[1] - com.y);
                risk *= smooth((ahead - 0.022) / 0.025);
            }
            risk
        } else {
            0.
        };
        let missed = if acquired && t.local_tick >= 4700 {
            smooth((t.landing_error(displacement[swing]) - 0.015) / 0.025)
        } else {
            0.
        };
        if risk == 0. && missed == 0. {
            return Ok(());
        }
        let loaded = foot_loaded(be, self.foot_ids[swing], self.floor)?;
        if risk > 0. {
            t.lower_for_recovery(risk, loaded);
        }
        if acquired && loaded {
            t.acquire_support(displacement, risk.max(missed));
        }
        Ok(())
    }
}
fn smooth(fraction: f64) -> f64 {
    let x = fraction.clamp(0., 1.);
    x * x * (3. - 2. * x)
}
fn foot_loaded(be: &PhysicsBackend, foot: BodyId, floor: BodyId) -> Result<bool, PhysicsError> {
    if let Some(world) = be.downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>() {
        return Ok(world
            .rigid_contact_patches()
            .into_iter()
            .filter(|p| p.a == foot && p.b == floor || p.a == floor && p.b == foot)
            .any(|p| p.points.iter().any(|c| c.normal_impulse > 1.0e-3)));
    }
    #[cfg(feature = "distributed-physics")]
    if let Some(world) = be.downcast_ref::<crate::distributed_hinge::DistributedGroundedRobot>() {
        return Ok(world
            .fixed_ground_patches(floor)?
            .iter()
            .filter(|p| p.body == foot)
            .any(|p| p.points.iter().any(|c| c.normal_impulse > 1.0e-3)));
    }
    Err(PhysicsError::InvalidValue)
}
