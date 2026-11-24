// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Geometry and surface motion of a finite cuboid spinning around its local Z axis.
use super::*;

#[derive(Clone, Copy)]
pub(super) struct Turntable {
    pub half: DVec3,
    pub initial: DQuat,
    pub spin: DVec3,
}
impl Turntable {
    pub fn orientation(self, t: f64) -> DQuat {
        DQuat::from_scaled_axis(t * self.spin) * self.initial
    }

    pub fn projection(
        self,
        start: DVec3,
        end: DVec3,
        dt: Interval,
    ) -> Result<Option<Projection>, XpbdError> {
        // The shortest-arc cast cannot represent more than half a revolution.
        if dt.h * self.spin.length() >= std::f64::consts::PI {
            return Err(XpbdError::UnresolvedSweep);
        }
        let geometry = OrientedShape::Cuboid(self.half);
        let sweep = Sweep {
            geometry,
            start,
            end,
            q0: self.orientation(dt.t0),
            q1: self.orientation(dt.t1),
            radius: dt.radius,
        };
        let Some(hit) = sweep.impact()? else {
            return Ok(None);
        };
        let (normal, surface) = sweep.end_witness(&hit);
        let depth = dt.radius - (end - surface).dot(normal);
        let tolerance =
            128. * f64::EPSILON * (start.length() + end.length() + self.half.length() + dt.radius);
        if hit.time > 0. && geometry.witness(sweep.q1.conjugate() * end).0 > dt.radius + tolerance {
            // At a rounded edge a material particle can both enter and leave in
            // one substep. Impulse acts at first impact and changes only the
            // remaining fraction of the displacement.
            let orientation = sweep.rotation_at(hit.time);
            let normal = orientation * hit.local_normal;
            let arm = orientation * hit.local_surface;
            let relative = (end - start) / dt.h - self.spin.cross(arm);
            let closing = -relative.dot(normal);
            return Ok((closing > 0.).then_some(Projection {
                normal,
                depth: 0.,
                initial_overlap: 0.,
                event: Some(SweptImpact {
                    time: hit.time,
                    transient: true,
                    orientation,
                    arm,
                    closing,
                }),
            }));
        }
        let closing = (start - end + (dt.h * self.spin).cross(surface)).dot(normal);
        Ok(
            (depth > tolerance || depth.abs() <= tolerance && closing > tolerance).then_some(
                Projection {
                    normal,
                    depth: depth.max(0.),
                    initial_overlap: hit.initial_overlap,
                    event: None,
                },
            ),
        )
    }
}
