// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A sole face tips about a line on the prescribed pad. The material points
//! on the sole slide past the fixed pad edge. Reaction direction follows the
//! sole normal, so angular and horizontal linear momentum can both change.
mod enclosure;
use crate::{EdgeSupportState, FrictionlessFaceSupport, PhysicsError};
pub use enclosure::DeckEdgeEnvelope;
use glam::{DQuat, DVec3};
#[derive(Clone, Copy, Debug)]
pub struct BoundDeckEdge {
    body: FrictionlessFaceSupport,
    start: EdgeSupportState,
    n: DVec3,
    out: DVec3,
    axis: DVec3,
    origin: DVec3,
    half: f64,
    sole_normal: DVec3,
    local_axis: DVec3,
    depth: f64,
    theta0: f64,
    face_exit: bool,
    y0: f64,
    vy: f64,
    initial: Coordinates,
}
#[derive(Clone, Copy, Debug)]
pub struct DeckEdgeSample {
    pub state: EdgeSupportState,
    pub sole_normal: DVec3,
    pub center_of_pressure: DVec3,
    /// Span and centroid along the pad-owned edge in meters.
    pub segment: [f64; 2],
    pub center_along_edge: f64,
    pub equivalent_endpoint_force: [f64; 2],
    pub normal_force: f64,
    pub normal_impulse: f64,
    pub integrated_force: DVec3,
    pub angular_impulse_about_com: DVec3,
    pub signed_tilt: f64,
    pub tilt_rate: f64,
    pub tilt_acceleration: f64,
}
#[derive(Clone, Copy, Debug, Default)]
struct Coordinates {
    u: f64,
    du: f64,
    tilt: f64,
    tilt_rate: f64,
    yaw: f64,
    yaw_rate: f64,
    impulse: f64,
}
impl Coordinates {
    fn add(self, d: Self, h: f64) -> Self {
        Self {
            u: self.u + h * d.u,
            du: self.du + h * d.du,
            tilt: self.tilt + h * d.tilt,
            tilt_rate: self.tilt_rate + h * d.tilt_rate,
            yaw: self.yaw + h * d.yaw,
            yaw_rate: self.yaw_rate + h * d.yaw_rate,
            impulse: self.impulse + h * d.impulse,
        }
    }
}
struct Stage {
    rates: Coordinates,
    normal: DVec3,
    tangent: DVec3,
    local_line: DVec3,
    local_tangent: DVec3,
    force: f64,
    center_offset: f64,
}
impl FrictionlessFaceSupport {
    /// Bind a load on one specified pad edge (−x,+x,−y,+y). Endpoints and force
    /// must be checked over an interval before applying this bilateral trajectory.
    pub fn bind_deck_edge(
        self,
        s: EdgeSupportState,
        edge: usize,
    ) -> Result<Option<BoundDeckEdge>, PhysicsError> {
        if edge >= 4 {
            return Err(PhysicsError::InvalidValue);
        }
        // Reuse the face input checks. Tilted and overhanging states are
        // valid inputs here even when flat face support is unavailable.
        let _ = self.bind(s)?;
        let n = self.pad_axes[0].cross(self.pad_axes[1]);
        let out = self.pad_axes[edge / 2] * if edge.is_multiple_of(2) { -1. } else { 1. };
        let axis = n.cross(out);
        let origin = out * self.pad_half[edge / 2];
        let half = self.pad_half[1 - edge / 2];
        let dx = self.corners[2] - self.corners[0];
        let dy = self.corners[1] - self.corners[0];
        let z = dx.cross(dy).normalize();
        let depth = -self.corners[0].dot(z);
        let normal = s.orientation * z;
        let eps = 512. * f64::EPSILON;
        if depth <= 0. || normal.dot(axis).abs() > eps || normal.dot(n) <= 0. {
            return Ok(None);
        }
        let mut theta = normal.dot(out).atan2(normal.dot(n));
        if theta < -eps {
            return Ok(None);
        }
        if theta.abs() < eps {
            theta = 0.;
        }
        let tangent = axis.cross(normal);
        let displacement = s.center - origin;
        let u = displacement.dot(tangent);
        let rate = s.angular_velocity.dot(axis);
        let yaw = s.angular_velocity.dot(normal);
        if (displacement.dot(normal) - depth).abs() > eps * (1. + s.center.length() + depth)
            || s.angular_velocity.dot(tangent).abs() > eps * (1. + s.angular_velocity.length())
            || (s.velocity.dot(normal) + u * rate).abs()
                > eps * (1. + s.velocity.length() + u.abs() * rate.abs())
        {
            return Ok(None);
        }
        let result = BoundDeckEdge {
            body: self,
            start: s,
            n,
            out,
            axis,
            origin,
            half,
            sole_normal: z,
            local_axis: s.orientation.conjugate() * axis,
            depth,
            theta0: theta,
            face_exit: false,
            y0: displacement.dot(axis),
            vy: s.velocity.dot(axis),
            initial: Coordinates {
                u,
                du: s.velocity.dot(tangent) - depth * rate,
                tilt: theta,
                tilt_rate: rate,
                yaw: 0.,
                yaw_rate: yaw,
                impulse: 0.,
            },
        };
        let first = result.sample(0., 1.)?;
        let margin = eps * (1. + self.pad_half.length() + s.center.length());
        Ok((first.normal_force > 0.
            && first.center_along_edge > first.segment[0] + margin
            && first.center_along_edge < first.segment[1] - margin)
            .then_some(result))
    }
}
impl BoundDeckEdge {
    pub(crate) fn at_face_exit(&mut self) {
        // The originating face solution has exactly zero tilt and tilt rate.
        // Reprojecting its yaw on a rotated pad axis can otherwise give the
        // orthogonal component either floating-point sign at transition.
        self.theta0=0.;
        self.initial.tilt=0.;
        self.initial.tilt_rate=0.;
        self.initial.du=self.start.velocity.dot(self.out);
        self.face_exit=true;
    }
    fn stage(self, x: Coordinates) -> Result<Stage, PhysicsError> {
        let m = self.body.mass;
        let ib = self.body.inertia_body;
        let z = self.sole_normal;
        let l = DQuat::from_axis_angle(z, -x.yaw) * self.local_axis;
        let t = l.cross(z);
        // In local axes, dl/dyaw = l × z. The kinetic metric depends on yaw
        // through these two principal projections of the assembled tensor.
        let j = ib * l;
        let iz = ib * z;
        let a = l.dot(j);
        let b = l.dot(iz);
        let c = z.dot(iz);
        let ap = 2. * t.dot(j);
        let bp = t.dot(iz);
        let (sin, cos) = x.tilt.sin_cos();
        let normal = self.n * cos + self.out * sin;
        let tangent = self.out * cos - self.n * sin;
        let gn = self.body.gravity.dot(normal);
        let gt = self.body.gravity.dot(tangent);
        let k = x.tilt_rate;
        let s = x.yaw_rate;
        let metric = a + m * x.u * x.u - b * b / c;
        let bend = (-m * x.u * (gn + self.depth * k * k + 2. * x.du * k)
            - ap * s * k
            - bp * s * s
            - 0.5 * b * ap * k * k / c)
            / metric;
        let axial = (0.5 * ap * k * k - b * bend) / c;
        let du = gt + x.u * k * k - self.depth * bend;
        let force = m * (-2. * x.du * k - self.depth * k * k - x.u * bend - gn);
        let omega = l * k + z * s;
        let alpha = l * bend + z * axial + t * (k * s);
        let torque = ib * alpha + omega.cross(ib * omega);
        let offset = torque.dot(t) / force;
        if metric <= 0.
            || !bend.is_finite()
            || !axial.is_finite()
            || !du.is_finite()
            || !force.is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Stage {
            rates: Coordinates {
                u: x.du,
                du,
                tilt: k,
                tilt_rate: bend,
                yaw: s,
                yaw_rate: axial,
                impulse: force,
            },
            normal,
            tangent,
            local_line: l,
            local_tangent: t,
            force,
            center_offset: offset,
        })
    }
    fn segment(self, x: Coordinates, stage: &Stage, y: f64) -> [f64; 2] {
        let low = self.body.corners[0];
        let dx = self.body.corners[2] - low;
        let dy = self.body.corners[1] - low;
        let r = -x.u * stage.local_tangent - self.depth * self.sole_normal;
        let mut range = [-self.half, self.half];
        for d in [dx, dy] {
            let len = d.length();
            let v = d / len;
            let p = (r - low).dot(v);
            let speed = stage.local_line.dot(v);
            if speed.abs() <= 128. * f64::EPSILON {
                if !(0.0..=len).contains(&p) {
                    return [1., -1.];
                }
            } else {
                let ends = [y - p / speed, y + (len - p) / speed];
                range[0] = range[0].max(ends[0].min(ends[1]));
                range[1] = range[1].min(ends[0].max(ends[1]));
            }
        }
        range
    }
    fn evaluate(self, t: f64, x: Coordinates) -> Result<DeckEdgeSample, PhysicsError> {
        let a = self.stage(x)?;
        let g = self.body.gravity;
        let y = self.y0 + self.vy * t + 0.5 * g.dot(self.axis) * t * t;
        let vy = self.vy + g.dot(self.axis) * t;
        let q = DQuat::from_axis_angle(self.axis, x.tilt - self.theta0)
            * self.start.orientation
            * DQuat::from_axis_angle(self.sole_normal, x.yaw);
        let center = self.origin + self.axis * y + a.tangent * x.u + a.normal * self.depth;
        let velocity = self.axis * vy + a.tangent * (x.du + self.depth * x.tilt_rate)
            - a.normal * (x.u * x.tilt_rate);
        let w = self.axis * x.tilt_rate + a.normal * x.yaw_rate;
        let momentum = |q: DQuat, w: DVec3| q * (self.body.inertia_body * (q.conjugate() * w));
        let s = y + a.center_offset;
        let segment = self.segment(x, &a, y);
        let f1 = (s - segment[0]) / (segment[1] - segment[0]) * a.force;
        Ok(DeckEdgeSample {
            state: EdgeSupportState {
                center,
                velocity,
                orientation: q,
                angular_velocity: w,
            },
            sole_normal: a.normal,
            center_of_pressure: self.origin + self.axis * s,
            segment,
            center_along_edge: s,
            equivalent_endpoint_force: [a.force - f1, f1],
            normal_force: a.force,
            normal_impulse: x.impulse,
            integrated_force: self.body.mass * (velocity - self.start.velocity - g * t),
            angular_impulse_about_com: momentum(q, w)
                - momentum(self.start.orientation, self.start.angular_velocity),
            signed_tilt: x.tilt,
            tilt_rate: x.tilt_rate,
            tilt_acceleration: a.rates.tilt_rate,
        })
    }
    pub fn trajectory(self) -> DeckEdgeTrajectory {
        DeckEdgeTrajectory {
            body: self,
            time: 0.,
            coords: self.initial,
        }
    }
    pub fn sample(self, seconds: f64, maximum_step: f64) -> Result<DeckEdgeSample, PhysicsError> {
        self.trajectory().advance(seconds, maximum_step)
    }
}
#[derive(Clone, Copy, Debug)]
pub struct DeckEdgeTrajectory {
    body: BoundDeckEdge,
    time: f64,
    coords: Coordinates,
}
impl DeckEdgeTrajectory {
    /// Advance the bilateral equations. Callers check positive pressure and
    /// the outward side of the pad before accepting a constrained result.
    pub fn advance(
        &mut self,
        seconds: f64,
        maximum_step: f64,
    ) -> Result<DeckEdgeSample, PhysicsError> {
        if !seconds.is_finite() || seconds < 0. || !maximum_step.is_finite() || maximum_step <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let count = (seconds / maximum_step).ceil().max(1.);
        if count > 1e6 {
            return Err(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "bounded deck-edge trajectory",
            });
        }
        let h = seconds / count;
        let mut x = self.coords;
        for _ in 0..count as usize {
            let a = self.body.stage(x)?.rates;
            let b = self.body.stage(x.add(a, 0.5 * h))?.rates;
            let c = self.body.stage(x.add(b, 0.5 * h))?.rates;
            let d = self.body.stage(x.add(c, h))?.rates;
            x = x
                .add(a, h / 6.)
                .add(b, h / 3.)
                .add(c, h / 3.)
                .add(d, h / 6.);
        }
        let end = self.time + seconds;
        let result = self.body.evaluate(end, x)?;
        self.time = end;
        self.coords = x;
        Ok(result)
    }
}
