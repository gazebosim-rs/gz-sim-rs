// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A finite-time sticking certificate for nonparallel edges and vertex/face
//! contact. At rest in the support frame, the coincident material points are
//! fixed: for two intersecting nonparallel lines, u ds = v dt implies ds=dt=0.
use super::*;
#[derive(Clone, Copy, Debug)]
pub struct StickingPointSpan {
    pub motion: FiniteArcResult,
    pub center_speed_bound: f64,
    pub angular_speed_bound: f64,
    pub minimum_normal_force: f64,
    pub minimum_cone_margin: f64,
    pub subdivisions: u32,
}
impl FiniteFeatureArc {
    /// Return the sticking trajectory only if pressure, static friction and
    /// the finite feature all have sufficient margin for the whole duration.
    pub fn sticking_point_span(self, h: f64) -> Result<Option<StickingPointSpan>, PhysicsError> {
        self.sticking_bound(h, false)
    }
    pub(super) fn sticking_bound(
        self,
        h: f64,
        allow_exit: bool,
    ) -> Result<Option<StickingPointSpan>, PhysicsError> {
        if !h.is_finite() || h <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        if !self.locked {
            return Ok(None);
        }
        let y = self.start();
        let (moving, s) = self.frame(y).ok_or(PhysicsError::InvalidValue)?;
        let Some(load) = self.force(y, moving, s) else {
            return Ok(None);
        };
        if load.mode != FiniteLoadMode::Sticking {
            return Ok(None);
        }
        let m = self.material;
        let n = s.site.normal;
        let r = s.site.point.distance(y.c);
        let g = self.flight.acceleration.length();
        let up = |x: f64| x * (1. + 2e-12) + 1e-14;
        let down = |x: f64| x - 2e-12 * (1. + x.abs());
        let speed = up(y
            .v
            .length()
            .hypot((m.inertia / m.mass).sqrt() * y.w.length())
            + g * h);
        let global = up(speed * (m.mass / m.inertia).sqrt());
        // F = -W⁻¹(g+ω×(ω×r)), W = I/m + (r² I-rrᵀ)/I_com.
        // W is positive definite with ||W⁻¹|| <= m for all orientations.
        let force = |w: f64| up(m.mass * (g + w * w * r));
        let spin = global.min(up(y.w.length() + h * r * force(global) / m.inertia));
        let fmax = force(spin);
        let alpha = up(r * fmax / m.inertia);
        if down(1. / m.mass - m.friction * r * r / (2. * m.inertia)) <= 0. {
            return Ok(None);
        }
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        let (dn, ddn) = match s.site.feature {
            crate::SeparatingFeature::SupportFace(_) => (0., 0.),
            crate::SeparatingFeature::MovingFace(_) => (spin, up(alpha + spin * spin)),
            crate::SeparatingFeature::EdgePair(i, j) => {
                let u = moving.orientation * axes[i];
                let v = self.flight.support.orientation * axes[j];
                let sine = down(u.cross(v).length() - h * spin);
                if sine <= 1e-7 {
                    return Ok(None);
                }
                let rate = up(spin / sine);
                (rate, up((alpha + spin * spin) / sine + 3. * rate * rate))
            }
        };
        // At sticking r'=ω×r, orthogonal to r. Therefore
        // ||W'|| = |r| |r'|/I_com and F'=-W⁻¹(b'+W'F).
        let df =
            up(m.mass * (r * (2. * spin * alpha + spin.powi(3)) + r * r * spin * fmax / m.inertia));
        let rate = up(df + dn * fmax);
        let pn = load.force.dot(n);
        let pressure = down(pn - h * rate);
        let margin = down(
            m.friction * pn
                - load.force.reject_from_normalized(n).length()
                - (1. + m.friction) * h * rate,
        );
        if ![speed, spin, pressure, margin]
            .into_iter()
            .all(f64::is_finite)
            || pressure <= 0.
            || margin <= 0.
        {
            return Ok(None);
        }
        let tol = 1024.
            * f64::EPSILON
            * (1. + moving.center.length() + self.flight.support.center.length() + r);
        let mut crossing = None;
        for d in s.margins() {
            let curvature = match d.limit {
                FeatureLimit::MovingNeighbor(i) => {
                    up(2. * moving.half_extent[i] * (alpha + spin * spin + 2. * spin * dn + ddn))
                }
                FeatureLimit::SupportNeighbor(j) => {
                    up(2. * self.flight.support.half_extent[j] * ddn)
                }
                // Both material coordinates stay fixed. Require a strictly
                // interior contact for every finite tangent extent.
                FeatureLimit::MovingExtent(..) | FeatureLimit::SupportExtent(..) => {
                    if d.distance <= tol {
                        return Ok(None);
                    }
                    continue;
                }
            };
            let slope = down(d.speed - 0.5 * h * curvature);
            let may_exit = d.distance < -tol
                || (d.distance <= tol && slope <= 0.)
                || (d.distance > tol && down(d.distance + h * slope) <= 0.);
            if may_exit
                && (!allow_exit || !transition::unique_closing(&mut crossing, d, h, curvature, tol))
            {
                return Ok(None);
            }
        }
        let measure = |n| {
            if crossing.is_some() {
                self.measure_candidate(h, n, 1e-17, crossing)
            } else {
                self.measure(h, n)
            }
        };
        let mut a = measure(1)?;
        let near =
            |x: DVec3, y: DVec3, tol: f64| x.distance(y) <= tol * (1. + x.length().max(y.length()));
        for subdivisions in [2, 4, 8] {
            let b = measure(subdivisions)?;
            if transition::same_event(&a, &b, crossing, tol)
                && near(a.center, b.center, 2e-12)
                && near(a.angular_velocity, b.angular_velocity, 2e-10)
                && near(a.velocity, b.velocity, 2e-10)
                && near(a.impulse, b.impulse, 2e-10)
                && (a.moving.orientation * b.moving.orientation.inverse())
                    .xyz()
                    .length()
                    <= 2e-12
                && b.site.site.point.distance(s.site.point) <= tol
                && b.site.gap.abs() <= tol
                && (b.velocity + b.angular_velocity.cross(b.site.site.point - b.center)).length()
                    <= 2e-11 * (1. + speed)
                && b.normal_impulse > 0.
                && b.energy_defect.abs() <= 2e-10 * (1. + m.mass * speed * speed)
            {
                return Ok(Some(StickingPointSpan {
                    motion: b,
                    center_speed_bound: speed,
                    angular_speed_bound: spin,
                    minimum_normal_force: pressure,
                    minimum_cone_margin: margin,
                    subdivisions,
                }));
            }
            a = b;
        }
        Ok(None)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::SeparatingFeature;
    fn skew(mu: f64, omega: DVec3, arm: DVec3) -> FiniteFeatureArc {
        let q = DQuat::from_rotation_y(0.22) * DQuat::from_rotation_x(0.31);
        let n = (q * DVec3::X).cross(DVec3::Y).normalize();
        let moving = FlightBox {
            center: q * DVec3::new(0., 0.08, 0.06),
            half_extent: DVec3::new(0.23, 0.08, 0.06),
            orientation: q,
        };
        let support = FlightBox {
            center: DVec3::new(-0.14, 0., -0.09),
            half_extent: DVec3::new(0.14, 0.29, 0.09),
            orientation: DQuat::IDENTITY,
        };
        let f = FiniteBoxFlight {
            moving,
            support,
            center_of_mass: -arm,
            angular_velocity: omega,
            relative_velocity: -omega.cross(arm),
            acceleration: -9.81 * DVec3::Z,
        };
        let site = FinitePointSite {
            feature: SeparatingFeature::EdgePair(0, 1),
            point: DVec3::ZERO,
            normal: n,
        };
        FiniteFeatureArc::new(
            f,
            site,
            SphericalPointMaterial {
                mass: 2.,
                inertia: 0.08,
                friction: mu,
            },
            true,
            1e-13,
        )
        .unwrap()
        .unwrap()
    }
    #[test]
    fn stationary_skew_pivot_carries_gravity_and_a_rotating_normal() {
        let w = DVec3::new(0.7, 0.4, -0.9);
        let arc = skew(0.6, w, DVec3::ZERO);
        let h = 0.003;
        let s = arc.sticking_point_span(h).unwrap().unwrap();
        assert!(s.motion.center.length() < 2e-13 && s.motion.velocity.length() < 2e-13);
        assert!(s.motion.impulse.distance(DVec3::Z * (19.62 * h)) < 1e-13);
        assert!(s.motion.angular_velocity.distance(w) < 2e-13);
        assert!(s.motion.site.site.point.length() < 2e-13 && s.motion.friction_work.abs() < 1e-13);
        assert!(s.minimum_cone_margin > 0. && s.minimum_normal_force > 0.);
        assert!(
            skew(0.1, w, DVec3::ZERO)
                .sticking_point_span(h)
                .unwrap()
                .is_none()
        );
        assert!(arc.sticking_point_span(2.).unwrap().is_none());
    }
    #[test]
    fn qualified_off_center_pivots_remain_inside_the_cone_and_on_both_material_edges() {
        let mut accepted = 0;
        for wx in [-12., -4., 0.8, 7.] {
            let w = DVec3::new(wx, 0.21 * wx, -0.41 * wx);
            for r in [
                DVec3::new(0.02, -0.07, 0.02),
                DVec3::new(0.1, -0.01, -0.08),
                DVec3::new(0.04, 0.03, 0.06),
            ] {
                let arc = skew(0.75, w, r);
                for h in [0.00002, 0.0005, 0.003, 0.03] {
                    let Some(s) = arc.sticking_point_span(h).unwrap() else {
                        continue;
                    };
                    accepted += 1;
                    let fine = arc.measure(h, 128).unwrap();
                    assert_eq!(fine.end, FiniteArcEnd::Horizon);
                    assert!(fine.site.site.point.length() < 1e-11);
                    assert!(fine.impulse.distance(s.motion.impulse) < 2e-9);
                    for k in 0..=8 {
                        let sample = arc.measure(h * (k as f64 + 0.001) / 8.001, 4).unwrap();
                        let f = FiniteBoxFlight {
                            moving: sample.moving,
                            center_of_mass: sample.center,
                            relative_velocity: -sample
                                .angular_velocity
                                .cross(sample.site.site.point - sample.center),
                            angular_velocity: sample.angular_velocity,
                            ..arc.flight
                        };
                        let load = f
                            .first_point_load(sample.site.site, arc.material, true)
                            .unwrap()
                            .unwrap();
                        assert_eq!(load.mode, FiniteLoadMode::Sticking);
                        let n = sample.site.site.normal;
                        let normal = load.force.dot(n);
                        assert!(
                            normal >= s.minimum_normal_force
                                && arc.material.friction * normal
                                    - load.force.reject_from_normalized(n).length()
                                    >= s.minimum_cone_margin
                        );
                    }
                }
            }
        }
        assert!(accepted >= 8);
    }
}
