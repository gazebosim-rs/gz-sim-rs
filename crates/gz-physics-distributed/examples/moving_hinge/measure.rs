// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::math::{Rotation, Vector};
use serde::Serialize;

#[derive(Default, Serialize)]
pub(super) struct Differences {
    pub position_m: f64,
    pub orientation_components: f64,
    pub linear_velocity_m_s: f64,
    pub angular_velocity_rad_s: f64,
    pub joint_rad: f64,
    pub joint_rad_s: f64,
    pub contact_components: f64,
}
fn delta<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(a, b)| {
            if a.is_finite() && b.is_finite() {
                (a - b).abs()
            } else {
                f64::INFINITY
            }
        })
        .fold(0., f64::max)
}
impl Differences {
    pub(super) fn check(&mut self, d: &MovingObservation, w: &MovingObservation) -> Result<()> {
        if (d.hinge.tick, d.hinge.joint_id) != (w.hinge.tick, w.hinge.joint_id) {
            return Err("moving identity or clock mismatch".into());
        }
        self.joint_rad = self.joint_rad.max(delta([d.hinge.q], [w.hinge.q]));
        self.joint_rad_s = self.joint_rad_s.max(delta([d.hinge.qdot], [w.hinge.qdot]));
        for (a, b) in [d.hinge.links[0], d.hinge.links[1], d.sphere]
            .into_iter()
            .zip([w.hinge.links[0], w.hinge.links[1], w.sphere])
        {
            if a.id != b.id {
                return Err("body identity mismatch".into());
            }
            self.position_m = self.position_m.max(delta(a.position, b.position));
            self.orientation_components = self.orientation_components.max(
                delta(a.orientation, b.orientation)
                    .min(delta(a.orientation, b.orientation.map(|x| -x))),
            );
            self.linear_velocity_m_s = self.linear_velocity_m_s.max(delta(a.velocity, b.velocity));
            self.angular_velocity_rad_s = self
                .angular_velocity_rad_s
                .max(delta(a.angular_velocity, b.angular_velocity));
        }
        for (a, b) in d.reactions.iter().zip(&w.reactions) {
            if a.active != b.active {
                return Err(format!("contact activity differs at {}", d.hinge.tick).into());
            }
            self.contact_components = self
                .contact_components
                .max(delta(a.normal, b.normal))
                .max(delta(a.tangent_impulse, b.tangent_impulse))
                .max(delta(
                    [a.normal_impulse, a.warmstart_normal_impulse],
                    [b.normal_impulse, b.warmstart_normal_impulse],
                ));
        }
        if self.position_m > 1e-8
            || self.orientation_components > 1e-8
            || self.linear_velocity_m_s > 1e-7
            || self.angular_velocity_rad_s > 1e-7
            || self.joint_rad > 1e-8
            || self.joint_rad_s > 1e-7
            || self.contact_components > 1e-8
        {
            return Err(format!(
                "standalone differential at tick {}: {}",
                d.hinge.tick,
                serde_json::to_string(self)?
            )
            .into());
        }
        Ok(())
    }
}
pub(super) fn momentum(s: &MovingSpec, o: &MovingObservation) -> (Vector, Vector) {
    let mut p = Vector::ZERO;
    let mut angular = Vector::ZERO;
    for (r, b) in [s.hinge.links[0], s.hinge.links[1], s.sphere]
        .into_iter()
        .zip([o.hinge.links[0], o.hinge.links[1], o.sphere])
    {
        let rot = Rotation::from_array(b.orientation);
        let com = Vector::from(b.position) + rot * Vector::from(r.inertia_center);
        let axes = rot * Rotation::from_array(r.inertia_orientation);
        let w = axes.inverse() * Vector::from(b.angular_velocity);
        let [xx, yy, zz, xy, xz, yz] = r.inertia_tensor;
        let spin = axes
            * Vector::new(
                xx * w.x + xy * w.y + xz * w.z,
                xy * w.x + yy * w.y + yz * w.z,
                xz * w.x + yz * w.y + zz * w.z,
            );
        let mv = Vector::from(b.velocity) * r.mass;
        p += mv;
        angular += com.cross(mv) + spin;
    }
    (p, angular)
}
pub(super) fn impulse_on_hinge(o: &MovingObservation) -> Vector {
    o.reactions
        .iter()
        .map(|r| Vector::from(r.normal) * r.normal_impulse + Vector::from(r.tangent_impulse))
        .sum()
}
