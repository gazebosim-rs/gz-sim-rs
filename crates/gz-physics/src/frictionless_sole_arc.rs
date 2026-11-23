// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A rolling frictionless rectangle can briefly become flat, transfer its
//! normal impulse to the opposite edge and continue bearing in the same tick.
use crate::{
    BoundFrictionlessEdge, EdgeSupportEnvelope, EdgeSupportSample, EdgeSupportState,
    FrictionlessEdgeSupport, PhysicsError, RectangularNormalContact, RectangularNormalResponse,
};
use glam::{DMat3, DVec3};

#[derive(Clone, Copy, Debug)]
pub struct FrictionlessSoleArc {
    pub mass: f64,
    pub inertia_body: DMat3,
    /// Body-relative COM levers ordered (−x,−y),(−x,+y),(+x,−y),(+x,+y).
    pub corners: [DVec3; 4],
    pub normal: DVec3,
    pub plane_offset: f64,
    pub gravity: DVec3,
}
#[derive(Clone, Copy, Debug)]
pub struct SoleBearingSpan {
    pub vertices: [usize; 2],
    pub start: EdgeSupportState,
    pub end: EdgeSupportSample,
    pub duration_seconds: f64,
    pub envelope: EdgeSupportEnvelope,
    pub integration_steps: u32,
}
#[derive(Clone, Copy, Debug)]
pub struct SolePressureTransfer {
    /// Seconds from this branch's start. The internal geometry event can lie
    /// between integer-nanosecond clock stamps.
    pub seconds: f64,
    pub numerical_bracket: [f64; 2],
    pub before: EdgeSupportState,
    pub after: EdgeSupportState,
    pub pressure: RectangularNormalResponse,
    pub before_normal: [f64; 4],
    pub dissipated_energy: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct SoleBearingAdvance {
    pub first: SoleBearingSpan,
    /// Present only if both former bearing sites open after the flat-face event
    /// and compression can continue on the two previously arriving sites.
    pub transfer: Option<(SolePressureTransfer, SoleBearingSpan)>,
}
impl SoleBearingAdvance {
    pub fn end(self) -> EdgeSupportState {
        self.transfer
            .map_or(self.first.end.state, |(_, p)| p.end.state)
    }
}
#[derive(Clone, Copy)]
struct Edge {
    ids: [usize; 2],
    other: [usize; 2],
    arc: BoundFrictionlessEdge,
    start: EdgeSupportState,
}
struct Impact {
    pressure: RectangularNormalResponse,
    after: EdgeSupportState,
    before_normal: [f64; 4],
    loss: f64,
}
impl FrictionlessSoleArc {
    fn edge(self, start: EdgeSupportState, ids: [usize; 2]) -> Result<Option<Edge>, PhysicsError> {
        if ids.into_iter().any(|k| k >= 4)
            || (ids[0] ^ ids[1]).count_ones() != 1
            || self.corners.into_iter().any(|p| !p.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let dx = self.corners[2] - self.corners[0];
        let dy = self.corners[1] - self.corners[0];
        let x = dx.length();
        let y = dy.length();
        let tol = 512. * f64::EPSILON;
        if x == 0.
            || y == 0.
            || dx.dot(dy).abs() > tol * x * y
            || (self.corners[3] - self.corners[0] - dx - dy).length() > tol * (x + y)
        {
            return Err(PhysicsError::InvalidValue);
        }
        let remaining = if ids[0] ^ ids[1] == 1 { 2 } else { 1 };
        let other = ids.map(|k| k ^ remaining);
        let body = FrictionlessEdgeSupport {
            mass: self.mass,
            inertia_body: self.inertia_body,
            sites_body: ids.map(|k| self.corners[k]),
            normal: self.normal,
            plane_offset: self.plane_offset,
            gravity: self.gravity,
        };
        Ok(body.bind(start)?.map(|arc| Edge {
            ids,
            other,
            arc,
            start,
        }))
    }
    fn gap(self, e: Edge, s: EdgeSupportState) -> f64 {
        self.normal
            .dot(s.orientation * (self.corners[e.other[0]] - self.corners[e.ids[0]]))
    }
    fn span(self, e: Edge, h: f64) -> Result<Option<SoleBearingSpan>, PhysicsError> {
        let Some(b) = e.arc.enclose(h)? else {
            return Ok(None);
        };
        if b.force_bounds.iter().any(|f| f[0] <= 0.) {
            return Ok(None);
        }
        let Some((end, steps)) = e.arc.resolved_sample(h)? else {
            return Ok(None);
        };
        Ok(Some(SoleBearingSpan {
            vertices: e.ids,
            start: e.start,
            end,
            duration_seconds: h,
            envelope: b,
            integration_steps: steps,
        }))
    }
    fn valid_through(self, e: Edge, span: SoleBearingSpan) -> bool {
        let tol = 512.
            * f64::EPSILON
            * (1.
                + self
                    .corners
                    .into_iter()
                    .map(|x| x.length())
                    .fold(0., f64::max));
        let s = span.envelope;
        let r = self.corners[e.other[0]];
        if s.directed_height(r)[0] > tol {
            return true;
        }
        let speed = s.normal_speed(r);
        let a = self.gap(e, e.start);
        let b = self.gap(e, span.end.state);
        (a >= -tol && speed[0] > 0.) || (b > tol && speed[1] < 0.)
    }
    fn impact(self, s: EdgeSupportState) -> Result<Option<Impact>, PhysicsError> {
        let q = s.orientation;
        let n = self.normal;
        let face = (self.corners[2] - self.corners[0])
            .cross(self.corners[1] - self.corners[0])
            .normalize();
        if (q * face - n).length() > 1e-11 {
            return Ok(None);
        }
        let arms = self.corners.map(|r| q * r);
        let torque = arms.map(|r| r.cross(n));
        let inverse = self.inertia_body.inverse();
        let inv = |t: DVec3| q * (inverse * (q.conjugate() * t));
        let mobility = torque.map(|a| torque.map(|b| 1. / self.mass + a.dot(inv(b))));
        let u = arms.map(|r| n.dot(s.velocity + s.angular_velocity.cross(r)));
        let law = match (RectangularNormalContact {
            inverse_mass: mobility,
            free_normal: u,
        })
        .solve()
        {
            Ok(r) => r,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(error) => return Err(error),
        };
        let Some([x, y]) = law.normalized_center else {
            return Ok(None);
        };
        let cp =
            arms[0] + (0.5 + 0.5 * x) * (arms[2] - arms[0]) + (0.5 + 0.5 * y) * (arms[1] - arms[0]);
        let j = law.resultant * n;
        let after = EdgeSupportState {
            velocity: s.velocity + j / self.mass,
            angular_velocity: s.angular_velocity + inv(cp.cross(j)),
            ..s
        };
        let loss = -0.5
            * j.dot(
                s.velocity
                    + after.velocity
                    + (s.angular_velocity + after.angular_velocity).cross(cp),
            );
        Ok((loss >= 0.).then_some(Impact {
            pressure: law,
            after,
            before_normal: u,
            loss,
        }))
    }
    /// Advance a single two-bearing segment, optionally through one verified
    /// change of edge. Other shapes, finite support extents and force ownership
    /// must be checked by the caller. Declines tension, unexplained contacts,
    /// whole-face sticking or more than one transition inside the duration.
    pub fn advance(
        self,
        start: EdgeSupportState,
        sites: [usize; 2],
        seconds: f64,
    ) -> Result<Option<SoleBearingAdvance>, PhysicsError> {
        if !seconds.is_finite() || seconds <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(edge) = self.edge(start, sites)? else {
            return Ok(None);
        };
        let Some(trial) = self.span(edge, seconds)? else {
            return Ok(None);
        };
        if self.valid_through(edge, trial) {
            return Ok(Some(SoleBearingAdvance {
                first: trial,
                transfer: None,
            }));
        }
        let prior = self.gap(edge, start);
        let next = self.gap(edge, trial.end.state);
        let speed = trial.envelope.normal_speed(self.corners[edge.other[0]]);
        let tol = 512.
            * f64::EPSILON
            * (1.
                + start.velocity.length()
                + start.angular_velocity.length() * self.corners[0].length());
        let roundoff =
            64. * f64::EPSILON * (self.corners[edge.other[0]] - self.corners[edge.ids[0]]).length();
        if prior < -roundoff || speed[1] >= -tol {
            return Ok(None);
        }
        if next > roundoff {
            // Directed speed proves the entire interval is still separated.
            return Ok(Some(SoleBearingAdvance {
                first: trial,
                transfer: None,
            }));
        }
        let mut lo = 0.;
        let mut hi = seconds;
        let t = if prior.abs() <= roundoff {
            hi = (roundoff / -speed[1]).min(seconds);
            0.
        } else if next.abs() <= roundoff {
            lo = (seconds - roundoff / -speed[1]).max(0.);
            seconds
        } else {
            for _ in 0..52 {
                let t = 0.5 * lo + 0.5 * hi;
                if t == lo || t == hi {
                    break;
                }
                let Some((x, _)) = edge.arc.resolved_sample(t)? else {
                    return Ok(None);
                };
                let gap = self.gap(edge, x.state);
                if gap > 0. {
                    lo = t;
                } else {
                    hi = t;
                }
                // At every intermediate tilt the other edge's gap decreases
                // at a speed within the certified interval. The mean value
                // theorem bounds the remaining time directly from this gap.
                let error = 8.
                    * f64::EPSILON
                    * (self.corners[edge.other[0]] - self.corners[edge.ids[0]]).length();
                let candidates = [
                    (gap - error) / -speed[0],
                    (gap - error) / -speed[1],
                    (gap + error) / -speed[0],
                    (gap + error) / -speed[1],
                ];
                let low = (t + candidates.into_iter().fold(f64::INFINITY, f64::min))
                    .next_down()
                    .max(lo);
                let high = (t + candidates.into_iter().fold(f64::NEG_INFINITY, f64::max))
                    .next_up()
                    .min(hi);
                if low <= high {
                    lo = low;
                    hi = high;
                }
                if hi - lo <= 4. * error / -speed[1] + 2e-15 / (1. + speed[0].abs()) {
                    break;
                }
            }
            0.5 * lo + 0.5 * hi
        };
        let Some(first) = self.span(edge, t)? else {
            return Ok(None);
        };
        let before = first.end.state;
        let Some(Impact {
            pressure,
            after,
            before_normal: u,
            loss,
        }) = self.impact(before)?
        else {
            return Ok(None);
        };
        if edge.ids.iter().any(|&k| pressure.normal[k] <= tol)
            || edge.other.iter().any(|&k| pressure.normal[k] != 0.)
        {
            return Ok(None);
        }
        let Some(other) = self.edge(after, edge.other)? else {
            return Ok(None);
        };
        let Some(second) = self.span(other, seconds - t)? else {
            return Ok(None);
        };
        if !self.valid_through(other, second) {
            return Ok(None);
        }
        Ok(Some(SoleBearingAdvance {
            first,
            transfer: Some((
                SolePressureTransfer {
                    seconds: t,
                    numerical_bracket: [lo, hi],
                    before,
                    after,
                    pressure,
                    before_normal: u,
                    dissipated_energy: loss,
                },
                second,
            )),
        }))
    }
}
#[cfg(test)]
mod tests;
