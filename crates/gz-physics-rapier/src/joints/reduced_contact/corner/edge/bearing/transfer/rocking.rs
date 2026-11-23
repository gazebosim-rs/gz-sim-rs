// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{PlanarRockingAdvance, PlanarSoleRocking};
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RockingLimitReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub normal: [f64; 3],
    /// Geometry and initial tilt/rate use this world axis and transverse direction.
    pub roll_axis: [f64; 3],
    pub transverse: [f64; 3],
    pub signed_initial_angle: f64,
    pub initial_angular_rate: f64,
    pub speed_ratio: f64,
    /// All times in this scalar solution are relative to `start`.
    pub response: PlanarRockingAdvance,
    /// First instantaneous pressure resultant. The continuous bearing and
    /// any subsequent reactions are also included in response.integrated_normal_impulse.
    pub first_pressure_world: Option<[f64; 3]>,
    pub angular_impulse_about_com: [f64; 3],
}
impl Frame {
    pub(super) fn rocking(
        self,
        b: &RapierBackend,
        t0: Duration,
        dt: Duration,
        vertices: [Vector; 8],
    ) -> Result<Option<PreparedBearing>, PhysicsError> {
        let n = self.normal;
        let s = self.start;
        let q = s.orientation;
        let dx = vertices[2] - vertices[0];
        let dy = vertices[1] - vertices[0];
        let (Some(x), Some(y)) = (dx.try_normalize(), dy.try_normalize()) else {
            return Ok(None);
        };
        let z = x.cross(y);
        let midpoint = 0.5 * (vertices[3] + vertices[0]);
        let height = -midpoint.dot(z);
        let eps = 512. * f64::EPSILON;
        if height <= 0.
            || (midpoint + height * z).length() > self.tolerance
            || self.gravity.dot(n) >= 0.
        {
            return Ok(None);
        }
        let seconds = dt.as_secs_f64();
        let up = n.dot(q * z);
        if up <= 0. {
            return Ok(None);
        }
        for (e, f, width) in [(x, y, 0.5 * dy.length()), (y, -x, 0.5 * dx.length())] {
            let axis = q * e;
            let rate = s.angular_velocity.dot(axis);
            let inertia_e = self.inertia * e;
            let i = e.dot(inertia_e);
            if axis.dot(n).abs() > eps
                || (s.angular_velocity - axis * rate).length()
                    > eps * (1. + s.angular_velocity.length())
                || inertia_e.cross(e).length() > eps * i
                || i <= 0.
            {
                continue;
            }
            let model = PlanarSoleRocking {
                mass: self.mass,
                roll_inertia: i,
                half_width: width,
                center_height: height,
                gravity: -self.gravity.dot(n),
            };
            let mut theta = n.dot(q * f).atan2(up);
            if theta.abs() <= 32. * f64::EPSILON {
                theta = 0.;
            }
            let abs = theta.abs();
            let (sin, cos) = abs.sin_cos();
            let h = height * cos + width * sin;
            let lever = width * cos - height * sin;
            if (s.center.dot(n) - h).abs() > self.tolerance {
                continue;
            }
            let vn = s.velocity.dot(n);
            let vt = eps
                * (1. + s.velocity.length() + s.angular_velocity.length() * vertices[0].length());
            let sign = if theta > 0. {
                1
            } else if theta < 0. {
                -1
            } else if (vn - rate * lever).abs() <= (vn + rate * lever).abs() {
                1
            } else {
                -1
            };
            if (vn - f64::from(sign) * rate * lever).abs() > vt {
                continue;
            }
            let Some(response) = model.advance(theta, rate, sign, seconds)? else {
                continue;
            };
            let tangent = |v: Vector| v - n * v.dot(n);
            let center_speed = tangent(s.velocity).length()
                + tangent(self.gravity).length() * seconds
                + width * response.angular_speed_bound;
            let enough = vertices.into_iter().all(|r| {
                let p = self
                    .boxes
                    .pad_pose
                    .inverse_transform_point(self.top + s.center + q * r);
                let reach = self.tolerance
                    + seconds * (center_speed + response.angular_speed_bound * r.length());
                p.x.abs() + reach < self.boxes.pad_half.x
                    && p.y.abs() + reach < self.boxes.pad_half.y
            });
            if !enough {
                return Ok(None);
            }
            let tree = b
                .world
                .multibody_joints
                .get_multibody(self.owner)
                .ok_or(PhysicsError::InvalidJoint)?;
            if !b.swept_locked_neighborhood(
                tree,
                self.owner,
                self.boxes,
                self.com,
                self.gravity,
                self.tolerance,
                center_speed,
                response.angular_speed_bound,
                dt,
            )? {
                return Ok(None);
            }
            let level = q * Rotation::from_axis_angle(e, -theta);
            let center = |t: f64| tangent(s.center + s.velocity * t + 0.5 * self.gravity * t * t);
            let state = EdgeSupportState {
                center: center(seconds) + n * response.center_height,
                orientation: level * Rotation::from_axis_angle(e, response.signed_angle),
                velocity: tangent(s.velocity + self.gravity * seconds)
                    + response.normal_velocity * n,
                angular_velocity: axis * response.angular_rate,
            };
            let first_pressure_world = response.first_impact.map(|j| {
                (self.top
                    + self.deck_velocity * j.seconds
                    + center(j.seconds)
                    + n * height
                    + level * (midpoint + j.pressure_center * width * f))
                    .to_array()
            });
            let receipt = RockingLimitReceipt {
                foot: self.foot,
                support: self.pad,
                start: t0,
                duration: dt,
                normal: n.to_array(),
                roll_axis: axis.to_array(),
                transverse: (level * f).to_array(),
                signed_initial_angle: theta,
                initial_angular_rate: rate,
                speed_ratio: model.speed_ratio()?,
                response,
                first_pressure_world,
                angular_impulse_about_com: (self.momentum(state) - self.momentum(s)).to_array(),
            };
            return Ok(Some(PreparedBearing {
                colliders: self.boxes.colliders,
                motion: self.motion(state, seconds),
                receipt: BearingResult::Rocking(Box::new(receipt)),
            }));
        }
        Ok(None)
    }
}
