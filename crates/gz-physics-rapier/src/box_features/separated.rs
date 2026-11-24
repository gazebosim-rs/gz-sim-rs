// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Refresh an unloaded, disproved separating plane before it delivers a false impulse.
use super::*;
use rapier3d_f64::geometry::ContactPair;

fn loaded(p: &ContactPair) -> bool {
    p.solver_manifolds().iter().any(return_witness::has_load)
}

impl RapierBackend {
    /// Request a fresh native query on the next step of a monitored convex box pair.
    /// The moving box must be dynamic and the second box fixed. The current plane
    /// must enter the incoming linear/angular travel allowance while a different
    /// separating axis clears it. Contact skins enter the allowance. Native
    /// collision detection still decides whether and how the contact responds.
    /// A collider with any warm or loaded neighbors is never invalidated.
    pub fn recheck_separated_boxes(
        &mut self,
        moving: BodyId,
        fixed: BodyId,
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        let [a, b] = [moving, fixed].map(|id| self.bodies.get(&id).map(|e| e.0));
        let (a, b) = (
            a.ok_or(PhysicsError::MissingBody)?,
            b.ok_or(PhysicsError::MissingBody)?,
        );
        let (m, f) = (&self.world.bodies[a], &self.world.bodies[b]);
        if dt.is_zero() || !m.is_dynamic() || !f.is_fixed() {
            return Err(PhysicsError::InvalidValue);
        }
        let ([mh], [fh]) = (m.colliders(), f.colliders()) else {
            return Err(PhysicsError::InvalidValue);
        };
        let [mh, fh] = [*mh, *fh];
        let (mc, fc) = (&self.world.colliders[mh], &self.world.colliders[fh]);
        let (Some(ms), Some(fs)) = (mc.shape().as_cuboid(), fc.shape().as_cuboid()) else {
            return Err(PhysicsError::InvalidValue);
        };
        let narrow = &self.world.narrow_phase;
        let Some(pair) = narrow.contact_pair(mh, fh) else {
            return Ok(false);
        };
        if !self.world.integration_parameters.contact_recycling
            || !(mc.active_hooks() | fc.active_hooks()).is_empty()
            || loaded(pair)
        {
            return Ok(false);
        }
        let seconds = dt.as_secs_f64();
        let arm = ms.half_extents.length() + mc.position().translation.distance(m.center_of_mass());
        let reach = mc.contact_skin()
            + fc.contact_skin()
            + seconds
                * (m.linvel().length()
                    + arm * m.angvel().length()
                    + seconds * self.world.gravity.length());
        let moving_first = pair.collider1 == mh;
        let (c1, c2, h1, h2, b1, b2) = if moving_first {
            (mc, fc, ms.half_extents, fs.half_extents, m, f)
        } else {
            (fc, mc, fs.half_extents, ms.half_extents, f, m)
        };
        let pose = c1.position().inv_mul(c2.position());
        let stale = pair.manifolds().iter().any(|row| {
            let n = row.data.normal;
            let crossing = row.points.iter().any(|pt| {
                let w1 = *c1.position() * pt.local_p1;
                let w2 = *c2.position() * pt.local_p2;
                let signed = (w2 - w1).dot(n);
                signed <= reach
                    && pt.dist - signed > reach
                    && (b1.velocity_at_point(w1) - b2.velocity_at_point(w2)).dot(n) > 0.
            });
            crossing
                && return_witness::separating_plane(&pose, h1, h2, row.local_n1, 0., reach)
                    .is_some_and(|(_, old, _)| old <= reach)
        });
        if !stale {
            return Ok(false);
        }
        let Some(handle) = [fh, mh]
            .into_iter()
            .find(|h| !narrow.contact_pairs_with(*h).any(loaded))
        else {
            return Ok(false);
        };
        crate::articulation_support::refresh_contact_rows(&mut self.world.colliders[handle]);
        Ok(true)
    }
}

#[cfg(test)]
mod tests;
