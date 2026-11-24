// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Implicit midpoint integration of a point that starts sliding from rest.
//! Solving the friction direction inside the stage avoids dividing by the
//! vanishing slip before its direction is dynamically determined.
use super::*;
impl FiniteFeatureArc {
    pub(super) fn slip_launch_step(self, y: State, h: f64) -> Option<State> {
        if h == 0. {
            return Some(y);
        }
        let half = h / 2.;
        let mut mid = y.add(self.derivative(y)?, half);
        let f = self.flight;
        let m = self.material;
        for _ in 0..12 {
            let (moving, s) = self.frame(mid)?;
            let n = s.site.normal;
            let r = s.site.point - mid.c;
            let x = n.any_orthonormal_vector();
            let axes = [x, n.cross(x), n];
            let omega = mid.w;
            let point = mid.v + omega.cross(r);
            let bn =
                n.dot(f.acceleration + omega.cross(omega.cross(r))) + 2. * s.normal_rate.dot(point);
            if bn >= 0. {
                return None;
            }
            // At the midpoint the material port equals free + h/2 W F.
            // Eliminate the midpoint velocities in the tangent directions.
            // The normal row enforces zero gap acceleration with the current
            // feature curvature. The same circular Coulomb law now chooses F.
            let free = y.v + half * f.acceleration + y.w.cross(r);
            let w = std::array::from_fn(|i| {
                std::array::from_fn(|j| {
                    axes[i].dot(axes[j] / m.mass + r.cross(axes[j]).cross(r) / m.inertia)
                })
            });
            let law = crate::PointImpact {
                inverse_mass: w,
                relative_velocity: [x.dot(free) / half, axes[1].dot(free) / half, bn],
                friction: m.friction,
            };
            let p = if self.scalar_slip {
                law.solve_implicit()
            } else {
                law.solve()
            }
            .ok()?;
            if p.sticking {
                return None;
            }
            let force = x * p.impulse[0] + axes[1] * p.impulse[1] + n * p.impulse[2];
            let torque = r.cross(force);
            let spin = DQuat::from_xyzw(omega.x, omega.y, omega.z, 0.);
            let d = State {
                c: mid.v,
                q: DVec4::from_array((spin * moving.orientation).to_array()) * 0.5,
                v: f.acceleration + force / m.mass,
                w: torque / m.inertia,
                j: force,
                normal: force.dot(n),
                normal_vector: n * force.dot(n),
                l: torque,
                work: force.dot(point),
            };
            let next = y.add(d, half);
            let eps = 96. * f64::EPSILON;
            if next.c.distance(mid.c) <= eps * (1. + mid.c.length())
                && next.v.distance(mid.v) <= eps * (1. + mid.v.length())
                && next.w.distance(mid.w) <= eps * (1. + mid.w.length())
                && next.q.distance(mid.q) <= eps
            {
                let mut out = y.add(d, h);
                out.q = out.q.normalize();
                return Some(out);
            }
            mid = next;
        }
        None
    }
}
