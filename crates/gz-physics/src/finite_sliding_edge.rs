// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Whole-interval clearance and force bounds when the loaded point migrates
//! along two spatial lines. Support axes translate without rotation.
use super::*;
impl FiniteFeatureArc {
    /// Certify sliding pressure, finite line extents and changing normal for a
    /// single native interval. Near-parallel and zero-slip events fall back.
    pub fn sliding_skew_span(self, h: f64) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        self.sliding_skew_bound(h, false, false)
    }
    /// Also allow a strictly increasing material velocity component to prove
    /// sliding through a launch from rest or a near-zero incoming slip.
    pub fn sliding_skew_launch_span(
        self,
        h: f64,
    ) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        self.sliding_skew_bound(h, false, true)
    }
    pub(super) fn sliding_skew_bound(
        self,
        h: f64,
        allow_exit: bool,
        allow_launch: bool,
    ) -> Result<Option<SlidingVertexSpan>, PhysicsError> {
        if !h.is_finite() || h <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        if self.locked || (!allow_launch && self.initial_slip_force.is_some()) {
            return Ok(None);
        }
        let y = self.start();
        let (moving, s) = self.frame(y).ok_or(PhysicsError::InvalidValue)?;
        let crate::SeparatingFeature::EdgePair(i, j) = s.site.feature else {
            return Ok(None);
        };
        let Some(load) = self.force(y, moving, s) else {
            return Ok(None);
        };
        if load.mode != FiniteLoadMode::Sliding
            && !(allow_launch && load.mode == FiniteLoadMode::IncipientSlip)
        {
            return Ok(None);
        }
        let m = self.material;
        let g = self.flight.acceleration.length();
        let up = |x: f64| x * (1. + 2e-12) + 1e-14;
        let down = |x: f64| x - 2e-12 * (1. + x.abs());
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        let u = moving.orientation * axes[i];
        let v = self.flight.support.orientation * axes[j];
        let sine0 = u.cross(v).length();
        let speed = up(y
            .v
            .length()
            .hypot((m.inertia / m.mass).sqrt() * y.w.length())
            + h * g);
        let global = up(speed * (m.mass / m.inertia).sqrt());
        let sine = down(sine0 - h * global);
        if sine <= 1e-7 {
            return Ok(None);
        }
        let arm = s.site.point - y.c;
        let along = (s.site.point - moving.center).dot(u);
        let far = up([-1., 1.]
            .into_iter()
            .map(|sign| (arm + (sign * moving.half_extent[i] - along) * u).length())
            .fold(0., f64::max));
        // V_material + u s' = v t'. Crossing with either edge bounds the
        // migration speed on the other by |V_material|/sin(angle).
        let radius = far.min(up(arm.length() + h * (speed + global * far) / sine));
        let floor = down(1. / m.mass - m.friction * radius * radius / (2. * m.inertia));
        if floor <= 0. {
            return Ok(None);
        }
        let force = |w: f64, v: f64| {
            let dn = up(w / down(sine0 - h * w));
            up((g + w * w * radius + 2. * dn * (v + w * radius)) * m.friction.hypot(1.) / floor)
        };
        let absolute_force = force(global, speed);
        let spin = global.min(up(y.w.length() + h * radius * absolute_force / m.inertia));
        let center_speed = speed.min(up(y.v.length() + h * (g + absolute_force / m.mass)));
        let sine = down(sine0 - h * spin);
        let f = force(spin, center_speed);
        let alpha = up(radius * f / m.inertia);
        let dn = up(spin / sine);
        let ddn = up((alpha + spin * spin) / sine + 3. * dn * dn);
        let port = up(center_speed + spin * radius);
        let migration = up(port / sine);
        let dr = up(spin * radius + migration);
        let dp = up(g + f / m.mass + alpha * radius + spin * dr);
        let slip = load
            .relative_point_velocity
            .reject_from_normalized(s.site.normal)
            .length();
        let mut min_slip = down(slip - h * dp);
        let mut growth = None;
        if min_slip <= 0. && allow_launch {
            growth = self.edge_slip_growth(
                s,
                slip_growth::EdgeMotionBounds {
                    h,
                    radius,
                    spin,
                    alpha,
                    normal_rate: dn,
                    force: f,
                    arm_rate: dr,
                    port_acceleration: dp,
                    sine,
                },
            );
            if let Some(witness) = growth {
                min_slip = witness.initial_component;
            }
        }
        // b=n·(g+ω×(ω×r))+2n'·V_material. The moment arm r includes
        // migration along the moving edge. Differentiating b bounds loss of load.
        let b_rate = up(dn * (g + spin * spin * radius)
            + 2. * spin * alpha * radius
            + spin * spin * dr
            + 2. * ddn * port
            + 2. * dn * dp);
        let inward = down(-load.free_normal_acceleration - h * b_rate);
        if ![min_slip, inward, dp, ddn].into_iter().all(f64::is_finite)
            || (min_slip <= 0. && growth.is_none())
            || inward <= 0.
        {
            return Ok(None);
        }
        let tol = 1024.
            * f64::EPSILON
            * (1. + moving.center.length() + self.flight.support.center.length() + radius);
        let mut crossing = None;
        for margin in s.margins() {
            let curvature = match margin.limit {
                FeatureLimit::MovingNeighbor(k) => {
                    up(2. * moving.half_extent[k] * (alpha + spin * spin + 2. * spin * dn + ddn))
                }
                FeatureLimit::SupportNeighbor(k) => {
                    up(2. * self.flight.support.half_extent[k] * ddn)
                }
                // Differentiate V_material + u s' = v t' once more.
                FeatureLimit::MovingExtent(..) | FeatureLimit::SupportExtent(..) => {
                    up((dp + spin * migration) / sine)
                }
            };
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
        // A native clock can divide a launch only picoseconds after it began.
        // A positive but tiny incoming slip still requires the implicit law.
        let arc = Self {
            implicit_slip: self.implicit_slip || growth.is_some(),
            scalar_slip: growth.is_some(),
            ..self
        };
        let mut span = arc.checked_slide(
            h,
            crossing,
            tol,
            center_speed,
            spin,
            min_slip,
            inward,
            s.site.normal,
        )?;
        if let Some(s) = &mut span {
            s.slip_growth = growth;
        }
        Ok(span)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn moving_edge(speed: f64, arm: DVec3, spin: DVec3, mu: f64) -> FiniteFeatureArc {
        let q = DQuat::from_rotation_y(0.22) * DQuat::from_rotation_x(0.31);
        let n = (q * DVec3::X).cross(DVec3::Y).normalize();
        let f = FiniteBoxFlight {
            moving: FlightBox {
                center: q * DVec3::new(0., 0.08, 0.06),
                orientation: q,
                half_extent: DVec3::new(0.23, 0.08, 0.06),
            },
            support: FlightBox {
                center: DVec3::new(-0.14, 0., -0.09),
                orientation: DQuat::IDENTITY,
                half_extent: DVec3::new(0.14, 0.29, 0.09),
            },
            center_of_mass: -arm,
            relative_velocity: speed * DVec3::Y - spin.cross(arm),
            angular_velocity: spin,
            acceleration: -9. * n,
        };
        FiniteFeatureArc::new(
            f,
            FinitePointSite {
                feature: crate::SeparatingFeature::EdgePair(0, 1),
                normal: n,
                point: DVec3::ZERO,
            },
            SphericalPointMaterial {
                mass: 2.,
                inertia: 0.12,
                friction: mu,
            },
            false,
            1e-13,
        )
        .unwrap()
        .unwrap()
    }
    #[test]
    fn translating_skew_intersection_has_the_exact_coulomb_trajectory() {
        for slip in [-2., -0.22, 0.4, 3.] {
            for mu in [0., 0.18, 0.6] {
                let arc = moving_edge(slip, DVec3::ZERO, DVec3::ZERO, mu);
                let h = 0.0005;
                let s = arc.sliding_skew_span(h).unwrap().unwrap();
                let n = arc.frame(arc.start()).unwrap().1.site.normal;
                let friction = -slip.signum() * mu * 9.;
                assert!(
                    s.motion
                        .center
                        .distance((slip * h + 0.5 * friction * h * h) * DVec3::Y)
                        < 1e-13
                );
                assert!(
                    s.motion
                        .impulse
                        .distance(2. * h * (9. * n + friction * DVec3::Y))
                        < 1e-13
                );
                assert!(s.motion.angular_velocity.length() < 1e-13);
                assert!(
                    s.minimum_slip_speed < (slip + h * friction).abs()
                        && s.minimum_inward_acceleration <= 9.
                );
            }
        }
        assert!(
            moving_edge(0.02, DVec3::ZERO, DVec3::ZERO, 0.5)
                .sliding_skew_span(0.01)
                .unwrap()
                .is_none()
        );
        assert!(
            moving_edge(40., DVec3::ZERO, DVec3::ZERO, 0.)
                .sliding_skew_span(0.01)
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn strict_cone_separation_proves_a_launch_but_declines_static_balance() {
        for heading in [-1., 1.] {
            for drive in [2., 3.8, 8.] {
                let seed = moving_edge(0.1, DVec3::ZERO, DVec3::ZERO, 0.42);
                let site = seed.frame(seed.start()).unwrap().1.site;
                let g = -9. * site.normal + drive * heading * DVec3::Y;
                let f = FiniteBoxFlight {
                    acceleration: g,
                    relative_velocity: DVec3::ZERO,
                    ..seed.flight
                };
                let a = FiniteFeatureArc::new(f, site, seed.material, true, 1e-13)
                    .unwrap()
                    .unwrap();
                let h = 0.0001;
                assert!(a.sliding_skew_span(h).unwrap().is_none());
                let result = a.sliding_skew_launch_span(h).unwrap();
                if drive <= 3.78 {
                    assert!(result.is_none());
                    continue;
                }
                let span = result.expect("strictly excessive tangent load");
                let growth = span.slip_growth.unwrap();
                let acc = (drive - 3.78) * heading * DVec3::Y;
                assert!(
                    growth.acceleration > 0.
                        && growth.acceleration <= growth.direction.dot(acc) + 1e-12
                );
                assert_eq!(growth.initial_component, 0.);
                assert!(span.motion.velocity.distance(h * acc) < 2e-13);
                assert!(span.motion.center.distance(0.5 * h * h * acc) < 2e-13);
                assert!(
                    (span.motion.normal_impulse - 18. * h).abs() < 1e-13
                        && span.motion.friction_work < 0.
                );
            }
        }
    }
    #[test]
    fn a_public_sliding_step_keeps_its_qualified_mesh_when_only_the_restart_check_fails() {
        let seed = moving_edge(0.1, DVec3::ZERO, DVec3::ZERO, 0.25);
        let site = seed.frame(seed.start()).unwrap().1.site;
        // A fixed normal and a spherical mass centered at the contact retain
        // the incoming normal residual across every implicit integration mesh.
        let drift = 190. * f64::EPSILON;
        let arc = FiniteFeatureArc::new(
            FiniteBoxFlight {
                relative_velocity: 1e-8 * DVec3::Y + drift * site.normal,
                acceleration: -9. * site.normal + 12. * DVec3::Y,
                ..seed.flight
            },
            site,
            seed.material,
            false,
            1e-13,
        )
        .unwrap()
        .unwrap();
        let h = 1e-4;
        let public = arc.sliding_skew_launch_span(h).unwrap().unwrap();
        assert!(public.slip_growth.is_some());
        assert!(public.motion.normal_velocity_defect > 128. * f64::EPSILON);
        assert_eq!(public.subdivisions, 2);
        let internal = FiniteFeatureArc {
            require_restart_precision: true,
            ..arc
        };
        assert!(internal.sliding_skew_launch_span(h).unwrap().is_none());
    }

    #[test]
    fn oblique_slip_has_a_forward_dual_witness_without_modifying_incoming_velocity() {
        let seed = moving_edge(1., DVec3::ZERO, DVec3::ZERO, 0.25);
        let site = seed.frame(seed.start()).unwrap().1.site;
        let transverse = DVec3::Y.cross(site.normal);
        let v = 1e-8 * (2. * transverse - DVec3::Y);
        let g = -9. * site.normal + 12. * DVec3::Y;
        let make = |velocity| {
            FiniteFeatureArc::new(
                FiniteBoxFlight {
                    relative_velocity: velocity,
                    acceleration: g,
                    ..seed.flight
                },
                site,
                seed.material,
                false,
                1e-13,
            )
            .unwrap()
            .unwrap()
        };
        let duration = 3e-9;
        let arc = make(v);
        let w = arc
            .sliding_skew_launch_span(duration)
            .unwrap()
            .unwrap()
            .slip_growth
            .unwrap();
        assert!(w.direction.dot(v) >= 0. && w.initial_component == w.direction.dot(v));
        assert!(w.acceleration > 0.);
        // Any circular friction force of this magnitude obeys the bound.
        for k in 0..128 {
            let (sn, cs) = (std::f64::consts::TAU * f64::from(k) / 128.).sin_cos();
            let acc = 12. * DVec3::Y + 2.25 * (sn * DVec3::Y + cs * transverse);
            assert!(w.direction.dot(acc) >= w.acceleration);
        }
        // Exactly opposed slip can arrest inside the step. No forward witness
        // can prove positive growth here.
        assert!(
            make(-1e-8 * DVec3::Y)
                .sliding_skew_launch_span(duration)
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn launch_witness_bounds_material_slip_on_rotating_finite_edges() {
        let mut found = 0;
        for spin in [-3.2, -0.74, 0.41, 1.83] {
            for mu in [0.14, 0.32, 0.55] {
                let w = DVec3::new(spin, -0.2 * spin, 0.6 * spin);
                let seed = moving_edge(0.1, DVec3::new(0.018, -0.027, 0.042), w, mu);
                let site = seed.frame(seed.start()).unwrap().1.site;
                let r = site.point - seed.flight.center_of_mass;
                for load in [-12., -6., 3., 7., 16.] {
                    let f = FiniteBoxFlight {
                        relative_velocity: -w.cross(r),
                        acceleration: -9. * site.normal + load * DVec3::Y,
                        ..seed.flight
                    };
                    let Some(arc) =
                        FiniteFeatureArc::new(f, site, seed.material, true, 1e-13).unwrap()
                    else {
                        continue;
                    };
                    let h = 25e-6;
                    let Some(span) = arc.sliding_skew_launch_span(h).unwrap() else {
                        continue;
                    };
                    let Some(g) = span.slip_growth else { continue };
                    found += 1;
                    for i in 1..=8 {
                        let t = h * f64::from(i) / 8.;
                        let refn = arc.measure(t, 48).unwrap();
                        assert_eq!(refn.end, FiniteArcEnd::Horizon);
                        let velocity = refn.velocity
                            + refn
                                .angular_velocity
                                .cross(refn.site.site.point - refn.center);
                        assert!(
                            velocity.dot(g.direction) + 2e-11
                                >= g.initial_component + t * g.acceleration
                        );
                        assert!(refn.friction_work <= 0. && refn.normal_impulse > 0.);
                        if i == 8 {
                            assert!(span.motion.impulse.distance(refn.impulse) < 1e-9);
                        }
                    }
                }
            }
        }
        assert!(found >= 12, "only {found} admitted");
    }
    #[test]
    fn turning_edges_stay_inside_the_force_and_slip_bounds() {
        let mut seen = 0;
        for wx in [-7., -1., 0.4, 4.] {
            for h in [1e-6, 0.0001, 0.001, 0.01] {
                let arc = moving_edge(
                    0.33,
                    DVec3::new(0.07, -0.09, 0.12),
                    DVec3::new(wx, 0.3 * wx, -0.5 * wx),
                    0.41,
                );
                if let Some(s) = arc.sliding_skew_span(h).unwrap() {
                    seen += 1;
                    for t in 1..=8 {
                        let q = arc.measure(h * t as f64 / 8., 48).unwrap();
                        assert_eq!(q.end, FiniteArcEnd::Horizon);
                        let site = q.site.site;
                        let n = site.normal;
                        let v = q.velocity + q.angular_velocity.cross(site.point - q.center);
                        let f = FiniteBoxFlight {
                            moving: q.moving,
                            center_of_mass: q.center,
                            relative_velocity: q.velocity - n * n.dot(v),
                            angular_velocity: q.angular_velocity,
                            ..arc.flight
                        };
                        let load = f
                            .first_point_load(site, arc.material, false)
                            .unwrap()
                            .unwrap();
                        assert!(
                            v.reject_from_normalized(n).length() >= s.minimum_slip_speed
                                && -load.free_normal_acceleration >= s.minimum_inward_acceleration
                        );
                    }
                    assert!(
                        s.motion
                            .impulse
                            .distance(arc.measure(h, 96).unwrap().impulse)
                            < 2e-9
                    )
                }
            }
        }
        assert!(seen >= 8);
    }
}
