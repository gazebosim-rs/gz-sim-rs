// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A sufficient finite-time contact-mode bound for one sliding shoe vertex
//! on a finite plane face. The integrator is used only after the bound holds.
use super::*;

#[derive(Clone, Copy, Debug)]
pub struct SlidingVertexSpan {
    pub motion: FiniteArcResult,
    /// Support-frame speed bounds for swept tests on the rest of the assembly.
    pub center_speed_bound: f64,
    pub angular_speed_bound: f64,
    pub minimum_slip_speed: f64,
    pub slip_growth: Option<SlipGrowth>,
    /// Magnitude of the guaranteed inward free normal acceleration.
    pub minimum_inward_acceleration: f64,
    pub subdivisions: u32,
}
impl FiniteFeatureArc {
    /// Admit a fixed support-face normal and a material vertex for the entire
    /// duration. Guards cover loss of normal pressure, vanishing Coulomb slip,
    /// another shoe vertex reaching the plane and all four finite face rims.
    /// An admitted geometric mode is advanced by mesh-checked RK4.
    pub fn sliding_vertex_span(self, h: f64) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        self.sliding_bound(h, false)
    }
    /// Carry a vertex up to a proposed handoff, allowing a single strictly
    /// decreasing clearance only if its checked endpoint is still interior.
    pub fn sliding_vertex_approach(
        self,
        h: f64,
    ) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        Ok(self
            .sliding_bound(h, true)?
            .filter(|s| s.motion.end == FiniteArcEnd::Horizon))
    }
    pub(super) fn sliding_bound(
        self,
        h: f64,
        allow_exit: bool,
    ) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        if !h.is_finite() || h <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        if self.locked || self.initial_slip_force.is_some() {
            return Ok(None);
        }
        let state = self.start();
        let (moving, s) = self.frame(state).ok_or(PhysicsError::InvalidValue)?;
        if !matches!(s.site.feature, crate::SeparatingFeature::SupportFace(_)) {
            return Ok(None);
        }
        let Some(load) = self.force(state, moving, s) else {
            return Ok(None);
        };
        if load.mode != FiniteLoadMode::Sliding {
            return Ok(None);
        }
        let m = self.material;
        let g = self.flight.acceleration.length();
        let r = s.site.point.distance(state.c);
        let slip = load
            .relative_point_velocity
            .reject_from_normalized(s.site.normal)
            .length();
        // For dissipative contact, d(sqrt(2K/m))/dt <= |g|. The off-diagonal
        // part of the inverse point mobility in the normal row has magnitude
        // at most r²/(2I), uniformly in the orientation of the material point.
        let up = |v: f64| v * (1. + 2e-12) + 1e-14;
        let down = |v: f64| v - 2e-12 * (1. + v.abs());
        let speed = up(state
            .v
            .length()
            .hypot((m.inertia / m.mass).sqrt() * state.w.length())
            + h * g);
        let global_spin = up(speed * (m.mass / m.inertia).sqrt());
        let floor = down(1. / m.mass - m.friction * r * r / (2. * m.inertia));
        if !floor.is_finite() || floor <= 0. {
            return Ok(None);
        }
        let max_force = |w: f64| up((g + w * w * r) * m.friction.hypot(1.) / floor);
        let spin = global_spin.min(up(
            state.w.length() + h * r * max_force(global_spin) / m.inertia
        ));
        let force = max_force(spin);
        let alpha = up(r * force / m.inertia);
        let accel = up(g + force / m.mass + r * (alpha + spin * spin));
        let min_slip = down(slip - accel * h);
        // b = n·(g + ω×(ω×r)), with fixed n and rotating r.
        let b_rate = up(r * (2. * spin * alpha + spin.powi(3)));
        let inward = down(-load.free_normal_acceleration - b_rate * h);
        if ![speed, spin, min_slip, inward, accel]
            .into_iter()
            .all(f64::is_finite)
            || min_slip <= 0.
            || inward <= 0.
        {
            return Ok(None);
        }
        let tol = 1024.
            * f64::EPSILON
            * (1. + moving.center.length() + self.flight.support.center.length() + r);
        let mut crossing = None;
        for margin in s.margins() {
            let curvature = match margin.limit {
                FeatureLimit::MovingNeighbor(i) => {
                    up(2. * moving.half_extent[i] * (alpha + spin * spin))
                }
                FeatureLimit::SupportExtent(..) => accel,
                _ => return Ok(None),
            };
            // d(t) >= d0 + t(d'0 - Ct/2). The concave lower parabola
            // attains its minimum at an endpoint. At initial incidence require
            // a strictly positive right-hand slope for the full interval.
            let slope = down(margin.speed - 0.5 * curvature * h);
            let may_exit = margin.distance < -tol
                || (margin.distance <= tol && slope <= 0.)
                || (margin.distance > tol && down(margin.distance + h * slope) <= tol);
            if may_exit
                && (!allow_exit
                    || !transition::unique_closing(&mut crossing, margin, h, curvature, tol))
            {
                return Ok(None);
            }
        }
        self.checked_slide(
            h,
            crossing,
            tol,
            speed,
            spin,
            min_slip,
            inward,
            s.site.normal,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub(super) fn checked_slide(
        self,
        h: f64,
        crossing: Option<FeatureLimit>,
        tol: f64,
        speed: f64,
        spin: f64,
        min_slip: f64,
        inward: f64,
        normal: DVec3,
    ) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        let measure = |n| {
            if crossing.is_some() {
                self.measure_candidate(h, n, 1e-17, crossing)
            } else {
                self.measure(h, n)
            }
        };
        let mut a = measure(1)?;
        let meshes: &[u32] = if self.implicit_slip {
            &[2, 4, 8, 16, 32]
        } else {
            &[2, 4, 8]
        };
        let mut public_span = None;
        for &subdivisions in meshes {
            let b = measure(subdivisions)?;
            let close = |x: DVec3, y: DVec3, tol: f64| {
                x.distance(y) <= tol * (1. + x.length().max(y.length()))
            };
            let rotation = (a.moving.orientation * b.moving.orientation.inverse())
                .xyz()
                .length();
            let energy = self.material.mass * speed * speed;
            if transition::same_event(&a, &b, crossing, tol)
                && close(a.center, b.center, 2e-12)
                && rotation <= 2e-12
                && close(a.velocity, b.velocity, 2e-10)
                && close(a.angular_velocity, b.angular_velocity, 2e-10)
                && close(a.impulse, b.impulse, 2e-10)
                && b.energy_defect.abs() < 2e-10 * (1. + energy)
                && b.site.gap.abs() <= tol
                && b.normal_velocity_defect.abs() <= 2e-11 * (1. + speed)
                && b.friction_work <= 0.
                && b.impulse.dot(normal) > 0.
            {
                let qualified = SlidingVertexSpan {
                    motion: b,
                    center_speed_bound: speed,
                    angular_speed_bound: spin,
                    minimum_slip_speed: min_slip,
                    slip_growth: None,
                    minimum_inward_acceleration: inward,
                    subdivisions,
                };
                // The reconstructed material port supplies the next start on
                // both internal and public clocks. Prefer a mesh accurate to
                // that start's normal-velocity check. A public interval can
                // still commit its first ordinarily qualified endpoint when
                // no mesh meets the tighter restart condition.
                let restart_tolerance = 128.
                    * f64::EPSILON
                    * (1.
                        + b.velocity.length()
                        + b.angular_velocity.length() * b.site.site.point.distance(b.center));
                if !self.implicit_slip || b.normal_velocity_defect.abs() <= restart_tolerance {
                    return Ok(Some(qualified));
                }
                if !self.require_restart_precision && public_span.is_none() {
                    public_span = Some(qualified);
                }
            }
            a = b;
        }
        Ok(public_span)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{FinitePointSite, SeparatingFeature};
    use glam::DQuat;
    fn translated(x: f64, speed: f64, mu: f64, tilt: f64, mass: f64) -> FiniteFeatureArc {
        let frame = DQuat::from_rotation_y(tilt) * DQuat::from_rotation_z(0.4);
        let q = frame * DQuat::from_rotation_y(0.27) * DQuat::from_rotation_x(-0.13);
        let n = frame * DVec3::Z;
        let half = DVec3::new(0.13, 0.06, 0.04);
        let point = frame * (x * DVec3::X);
        let offset = q * (half * (q.inverse() * -n).signum());
        let f = FiniteBoxFlight {
            moving: FlightBox {
                center: point - offset,
                orientation: q,
                half_extent: half,
            },
            support: FlightBox {
                center: -0.2 * n,
                orientation: frame,
                half_extent: DVec3::new(0.3, 0.5, 0.2),
            },
            center_of_mass: point,
            relative_velocity: frame * (speed * DVec3::X),
            angular_velocity: DVec3::ZERO,
            acceleration: -9.81 * n,
        };
        let site = FinitePointSite {
            feature: SeparatingFeature::SupportFace(2),
            point,
            normal: n,
        };
        FiniteFeatureArc::new(
            f,
            site,
            SphericalPointMaterial {
                mass,
                inertia: 0.1 * mass,
                friction: mu,
            },
            false,
            1e-13,
        )
        .unwrap()
        .unwrap()
    }
    #[test]
    fn admits_sliding_load_inside_the_face_and_declines_rim_or_stopping_events() {
        for tilt in [-0.81, 0., 0.77] {
            for mass in [0.2, 1., 11.] {
                for mu in [0., 0.13, 0.7] {
                    let h = 0.001;
                    let a = translated(0., 1., mu, tilt, mass);
                    let span = a.sliding_vertex_span(h).unwrap().unwrap();
                    let end = span.motion;
                    let frame = DQuat::from_rotation_y(tilt) * DQuat::from_rotation_z(0.4);
                    let v = frame * (DVec3::X * (1. - 9.81 * mu * h));
                    assert!(end.velocity.distance(v) < 1e-13);
                    assert!(
                        end.impulse
                            .distance(frame * (DVec3::Z - mu * DVec3::X) * (mass * 9.81 * h))
                            < 2e-13
                    );
                    assert!(
                        span.minimum_inward_acceleration <= 9.81
                            && span.minimum_slip_speed <= v.length()
                    );
                    assert!(
                        translated(0.2997, 1., mu, tilt, mass)
                            .sliding_vertex_span(h)
                            .unwrap()
                            .is_none()
                    );
                    if mu > 0. {
                        let slow = translated(0., 0.2 * 9.81 * mu * h, mu, tilt, mass);
                        assert!(slow.sliding_vertex_span(h).unwrap().is_none());
                    }
                }
            }
        }
    }
    #[test]
    fn admitted_spinning_vertices_keep_the_same_feature_on_a_finer_mesh() {
        let mut seen = 0;
        for tilt in [-0.48, 0.13, 0.78] {
            for omega in [-12., -3., 1.6, 7.] {
                for h in [1e-5, 4e-4, 0.006, 0.03] {
                    let base = translated(0.06, 0.9, 0.37, tilt, 2.);
                    let (moving, s) = base.frame(base.start()).unwrap();
                    let arm = moving.orientation * DVec3::new(0.09, 0.03, 0.16);
                    let w = moving.orientation * DVec3::new(omega, 0.2 * omega, -0.3 * omega);
                    let f = FiniteBoxFlight {
                        center_of_mass: s.site.point + arm,
                        angular_velocity: w,
                        relative_velocity: base.flight.relative_velocity + w.cross(arm),
                        ..base.flight
                    };
                    let Some(arc) =
                        FiniteFeatureArc::new(f, s.site, base.material, false, 1e-13).unwrap()
                    else {
                        continue;
                    };
                    if let Some(span) = arc.sliding_vertex_span(h).unwrap() {
                        seen += 1;
                        let fine = arc.measure(h, 192).unwrap();
                        assert_eq!(fine.end, FiniteArcEnd::Horizon);
                        assert!(fine.center.distance(span.motion.center) < 2e-11);
                        assert!(fine.impulse.distance(span.motion.impulse) < 5e-9);
                    }
                }
            }
        }
        assert!(seen >= 20);
    }
}
