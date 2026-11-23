// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Frictionless rigid motion while two distinct sites bear on a translating plane.
//! The allowed angular velocity is a spin about the sole edge plus a rotation
//! about the plane normal. Yaw momentum is conserved even for triaxial inertia.
mod enclosure;
use crate::PhysicsError;
pub use enclosure::EdgeSupportEnvelope;
use glam::{DMat3, DQuat, DVec2, DVec3};

#[derive(Debug, Clone, Copy)]
pub struct FrictionlessEdgeSupport {
    pub mass: f64,
    /// About the assembled COM, expressed in body coordinates.
    pub inertia_body: DMat3,
    pub sites_body: [DVec3; 2],
    /// Unit inward normal in the nonrotating support frame.
    pub normal: DVec3,
    /// Both absolute points satisfy normal · point = plane_offset.
    pub plane_offset: f64,
    pub gravity: DVec3,
}
#[derive(Debug, Clone, Copy)]
pub struct EdgeSupportState {
    /// Coordinates and twist in the translating, nonrotating support frame.
    pub center: DVec3,
    pub orientation: DQuat,
    pub velocity: DVec3,
    pub angular_velocity: DVec3,
}
#[derive(Clone, Copy, Debug)]
pub struct BoundFrictionlessEdge {
    body: FrictionlessEdgeSupport,
    start: EdgeSupportState,
    edge: DVec3,
    initial_normal_body: DVec3,
    edge_momentum: DVec3,
    midpoint: DVec3,
    momentum: f64,
    initial_roll_speed: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct EdgeSupportSample {
    pub state: EdgeSupportState,
    /// Signed forces required by the bilateral constraints, in newtons.
    /// A nonpositive value marks a boundary of the two-bearing mode.
    pub signed_force: [f64; 2],
    /// Integral of each signed force over this trajectory segment in N·s.
    pub signed_impulse: [f64; 2],
    pub yaw_momentum: f64,
    pub mechanical_energy: f64,
}
#[derive(Clone, Copy)]
struct Coordinates {
    yaw: f64,
    roll: f64,
    rate: f64,
    impulse: DVec2,
}
struct Stage {
    rates: Coordinates,
    spin: DVec3,
    angular_momentum: DVec3,
    normal: DVec3,
    height_rate: f64,
}
impl Coordinates {
    fn add(self, d: Self, h: f64) -> Self {
        Self {
            yaw: self.yaw + h * d.yaw,
            roll: self.roll + h * d.roll,
            rate: self.rate + h * d.rate,
            impulse: self.impulse + h * d.impulse,
        }
    }
}
impl FrictionlessEdgeSupport {
    /// Bind only an already co-level, normal-stationary pair. Missing bearing
    /// geometry or an initial tensile reaction declines the mode.
    pub fn bind(
        self,
        start: EdgeSupportState,
    ) -> Result<Option<BoundFrictionlessEdge>, PhysicsError> {
        let i = self.inertia_body;
        let n = self.normal;
        let q = start.orientation;
        if self.mass <= 0.
            || !self.mass.is_finite()
            || !self.plane_offset.is_finite()
            || !self.sites_body.into_iter().all(DVec3::is_finite)
            || !self.gravity.is_finite()
            || !n.is_finite()
            || (n.length_squared() - 1.).abs() > 256. * f64::EPSILON
            || !q.is_finite()
            || (q.length_squared() - 1.).abs() > 512. * f64::EPSILON
            || ![start.center, start.velocity, start.angular_velocity]
                .into_iter()
                .all(DVec3::is_finite)
            || !i.is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        let scale = i
            .to_cols_array()
            .into_iter()
            .map(f64::abs)
            .fold(0., f64::max);
        if scale == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let t = i * (1. / scale);
        if (t - t.transpose())
            .to_cols_array()
            .into_iter()
            .any(|x| x.abs() > 128. * f64::EPSILON)
            || t.x_axis.x <= 0.
            || t.x_axis.x.mul_add(t.y_axis.y, -t.x_axis.y * t.y_axis.x) <= 0.
            || t.determinant() <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let inv = t.inverse() * (1. / scale);
        if !inv.is_finite()
            || t.inverse()
                .to_cols_array()
                .into_iter()
                .any(|v| v.abs() > 1e12)
        {
            return Ok(None);
        }
        let delta = self.sites_body[1] - self.sites_body[0];
        let Some(edge) = delta.try_normalize() else {
            return Ok(None);
        };
        let d = q * edge;
        let tol = 512. * f64::EPSILON;
        if d.dot(n).abs() > tol
            || start.angular_velocity.dot(d.cross(n)).abs()
                > tol * (1. + start.angular_velocity.length())
        {
            return Ok(None);
        }
        for r in self.sites_body.map(|r| q * r) {
            if (n.dot(start.center + r) - self.plane_offset).abs()
                > tol * (1. + start.center.length() + r.length() + self.plane_offset.abs())
                || n.dot(start.velocity + start.angular_velocity.cross(r))
                    .abs()
                    > tol
                        * (1.
                            + start.velocity.length()
                            + r.length() * start.angular_velocity.length())
            {
                return Ok(None);
            }
        }
        let arc = BoundFrictionlessEdge {
            body: self,
            start,
            edge,
            initial_normal_body: q.conjugate() * n,
            edge_momentum: i * edge,
            midpoint: (self.sites_body[0] + self.sites_body[1]) * 0.5,
            momentum: n.dot(q * (i * (q.conjugate() * start.angular_velocity))),
            initial_roll_speed: start.angular_velocity.dot(d),
        };
        let data = arc.evaluate(0., arc.initial())?;
        Ok(data.signed_force.iter().all(|v| *v > 0.).then_some(arc))
    }
}
impl BoundFrictionlessEdge {
    fn initial(self) -> Coordinates {
        Coordinates {
            yaw: 0.,
            roll: 0.,
            rate: self.initial_roll_speed,
            impulse: DVec2::ZERO,
        }
    }
    /// The stage equations live in body coordinates. Yaw is cyclic, so the
    /// quadrature stages need only the current body-frame plane normal.
    fn stage(self, y: Coordinates) -> Result<Stage, PhysicsError> {
        let b = self.body;
        let e = self.edge;
        let n = DQuat::from_axis_angle(e, -y.roll) * self.initial_normal_body;
        let r = self.midpoint;
        let jn = b.inertia_body * n;
        let je = self.edge_momentum;
        let a = n.dot(jn);
        let cross = n.dot(je);
        let hr = n.dot(e.cross(r));
        let hrr = n.dot(e.cross(e.cross(r)));
        let c = e.dot(je) + b.mass * hr.powi(2);
        let ap = 2. * n.dot(e.cross(jn));
        let bp = n.dot(e.cross(je));
        let cp = 2. * b.mass * hr * hrr;
        let m = c - cross * cross / a;
        let mp = cp - 2. * cross * bp / a + cross * cross * ap / a.powi(2);
        let gravity = b.gravity.dot(b.normal);
        let potential = b.mass * gravity * hr - 0.5 * (self.momentum / a).powi(2) * ap;
        let acc = (-potential - 0.5 * mp * y.rate.powi(2)) / m;
        let yaw = (self.momentum - cross * y.rate) / a;
        let yaw_acc = -(ap * yaw * y.rate + bp * y.rate * y.rate + cross * acc) / a;
        let omega = n * yaw + e * y.rate;
        let angular_acc = n * yaw_acc + e * acc + n.cross(e) * (yaw * y.rate);
        let total = -b.mass * (n.dot(angular_acc.cross(r) + omega.cross(omega.cross(r))) + gravity);
        let arm = (b.sites_body[1] - b.sites_body[0]).cross(n);
        let h = b.inertia_body * omega;
        let couple = b.inertia_body * angular_acc + omega.cross(h) - r.cross(n) * total;
        let difference = 2. * couple.dot(arm) / arm.length_squared();
        let force = DVec2::new((total - difference) * 0.5, (total + difference) * 0.5);
        if m <= 0. || !force.is_finite() || !acc.is_finite() || !h.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Stage {
            rates: Coordinates {
                yaw,
                roll: y.rate,
                rate: acc,
                impulse: force,
            },
            spin: omega,
            angular_momentum: h,
            normal: n,
            height_rate: hr,
        })
    }
    fn evaluate(self, time: f64, y: Coordinates) -> Result<EdgeSupportSample, PhysicsError> {
        let b = self.body;
        let n = b.normal;
        let s = self.stage(y)?;
        let q = DQuat::from_axis_angle(n, y.yaw)
            * (self.start.orientation * DQuat::from_axis_angle(self.edge, y.roll));
        let tangent = |v: DVec3| v - n * n.dot(v);
        let center =
            tangent(self.start.center + self.start.velocity * time + 0.5 * b.gravity * time * time)
                + n * (b.plane_offset - self.midpoint.dot(s.normal));
        let velocity =
            tangent(self.start.velocity + b.gravity * time) - n * (y.rate * s.height_rate);
        let energy = 0.5 * (b.mass * velocity.length_squared() + s.spin.dot(s.angular_momentum))
            - b.mass * b.gravity.dot(center);
        let momentum = s.normal.dot(s.angular_momentum);
        if !energy.is_finite() || !momentum.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(EdgeSupportSample {
            state: EdgeSupportState {
                center,
                orientation: q,
                velocity,
                angular_velocity: q * s.spin,
            },
            signed_force: s.rates.impulse.to_array(),
            signed_impulse: y.impulse.to_array(),
            yaw_momentum: momentum,
            mechanical_energy: energy,
        })
    }
    /// Estimate local truncation error by step doubling before handing an
    /// endpoint to a native world. All coordinates and both integrated site
    /// loads must agree. A poorly resolved segment is declined.
    pub fn resolved_sample(
        self,
        seconds: f64,
    ) -> Result<Option<(EdgeSupportSample, u32)>, PhysicsError> {
        if !seconds.is_finite() || seconds < 0. {
            return Err(PhysicsError::InvalidValue);
        }
        if seconds == 0. {
            return Ok(Some((self.evaluate(0., self.initial())?, 0)));
        }
        let mut prior = self.sample(seconds, seconds)?;
        for steps in [2, 4, 8, 16] {
            let end = self.sample(seconds, seconds / f64::from(steps))?;
            let (a, b) = (prior.state, end.state);
            let tol = 2e-12;
            let dq = (a.orientation - b.orientation)
                .length()
                .min((a.orientation + b.orientation).length());
            if dq <= tol
                && a.center.distance(b.center) <= tol * (1. + a.center.distance(self.start.center))
                && a.velocity.distance(b.velocity) <= tol * (1. + self.start.velocity.length())
                && a.angular_velocity.distance(b.angular_velocity)
                    <= tol * (1. + self.start.angular_velocity.length())
                && (0..2).all(|k| {
                    (end.signed_impulse[k] - prior.signed_impulse[k]).abs()
                        <= tol * (1. + end.signed_impulse[k].abs())
                })
            {
                return Ok(Some((end, steps)));
            }
            prior = end;
        }
        Ok(None)
    }
    pub fn trajectory(self) -> EdgeSupportTrajectory {
        EdgeSupportTrajectory {
            arc: self,
            time: 0.,
            coordinates: self.initial(),
        }
    }
    /// Follow the bilateral branch on a fixed integration grid.
    pub fn sample(
        self,
        seconds: f64,
        maximum_step: f64,
    ) -> Result<EdgeSupportSample, PhysicsError> {
        self.trajectory().advance(seconds, maximum_step)
    }
}
/// A matched continuation of the two equality constraints. Before applying a
/// segment to a unilateral scene, its caller must check compression and the
/// clearance of the other geometric features over that segment.
#[derive(Clone, Copy)]
pub struct EdgeSupportTrajectory {
    arc: BoundFrictionlessEdge,
    coordinates: Coordinates,
    time: f64,
}
impl EdgeSupportTrajectory {
    /// Integrate from the current sample. A failed advance leaves the cursor
    /// unchanged. The returned impulses integrate from the initial binding.
    pub fn advance(
        &mut self,
        seconds: f64,
        maximum_step: f64,
    ) -> Result<EdgeSupportSample, PhysicsError> {
        if !seconds.is_finite() || !maximum_step.is_finite() || maximum_step <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let count = (seconds.abs() / maximum_step).ceil().max(1.);
        if count > 1_000_000. {
            return Err(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "bounded two-bearing arc integration",
            });
        }
        let h = seconds / count;
        let mut y = self.coordinates;
        let arc = self.arc;
        for _ in 0..count as usize {
            let a = arc.stage(y)?.rates;
            let b = arc.stage(y.add(a, 0.5 * h))?.rates;
            let c = arc.stage(y.add(b, 0.5 * h))?.rates;
            let d = arc.stage(y.add(c, h))?.rates;
            y = y
                .add(a, h / 6.)
                .add(b, h / 3.)
                .add(c, h / 3.)
                .add(d, h / 6.);
        }
        let end = self.time + seconds;
        let sample = arc.evaluate(end, y)?;
        self.coordinates = y;
        self.time = end;
        Ok(sample)
    }
}

#[cfg(test)]
mod tests;
