// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Acceleration complementarity immediately after a single finite box impact.
//! The support translates at constant velocity and the moving body's aggregate
//! inertia is spherical. This is an instantaneous law while the feature persists.
use crate::{FiniteBoxFlight, PhysicsError, PointImpact, SeparatingFeature};
use glam::DVec3;

#[derive(Clone, Copy, Debug)]
pub struct FinitePointSite {
    pub feature: SeparatingFeature,
    /// World normal from the support into the moving body.
    pub normal: DVec3,
    /// Moving material point at the contact.
    pub point: DVec3,
}
#[derive(Clone, Copy, Debug)]
pub struct SphericalPointMaterial {
    pub mass: f64,
    /// The three identical principal moments about the aggregate COM.
    pub inertia: f64,
    pub friction: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FiniteLoadMode {
    Opening,
    Sticking,
    Sliding,
    IncipientSlip,
}
#[derive(Clone, Copy, Debug)]
pub struct FinitePointLoad {
    pub mode: FiniteLoadMode,
    /// Force on the moving body in world axes, in newtons.
    pub force: DVec3,
    pub free_normal_acceleration: f64,
    pub constrained_normal_acceleration: f64,
    pub relative_point_velocity: DVec3,
    /// Unit normal derivative in world coordinates (s⁻¹).
    pub normal_rate: DVec3,
}

impl FiniteBoxFlight {
    /// Resolve a closed single point with zero relative normal speed. The caller
    /// supplies the outgoing state and the material sticking flag from the impact.
    /// Indeterminate grazing, near-parallel edges and an unresolved Coulomb
    /// denominator return `None` so a native contact solve can take over.
    pub fn first_point_load(
        self,
        site: FinitePointSite,
        material: SphericalPointMaterial,
        impact_stuck: bool,
    ) -> Result<Option<FinitePointLoad>, PhysicsError> {
        let SphericalPointMaterial {
            mass,
            inertia,
            friction,
        } = material;
        let n = site.normal;
        if ![mass, inertia, friction].into_iter().all(f64::is_finite)
            || mass <= 0.
            || inertia <= 0.
            || friction < 0.
            || !site.point.is_finite()
            || !n.is_finite()
            || (n.length_squared() - 1.).abs() > 1e-10
            || ![
                self.angular_velocity,
                self.relative_velocity,
                self.acceleration,
                self.center_of_mass,
            ]
            .into_iter()
            .all(|x| x.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        let w = self.angular_velocity;
        let axis = |i: usize| {
            [DVec3::X, DVec3::Y, DVec3::Z]
                .get(i)
                .copied()
                .ok_or(PhysicsError::InvalidValue)
        };
        let (direction, rate) = match site.feature {
            SeparatingFeature::SupportFace(j) => (self.support.orientation * axis(j)?, DVec3::ZERO),
            SeparatingFeature::MovingFace(i) => (self.moving.orientation * axis(i)?, w.cross(n)),
            SeparatingFeature::EdgePair(i, j) => {
                let u = self.moving.orientation * axis(i)?;
                let v = self.support.orientation * axis(j)?;
                let line = u.cross(v);
                let length = line.length();
                if length < 1e-7 {
                    return Ok(None);
                }
                let dq = w.cross(u).cross(v);
                (
                    line / length,
                    (dq - n * n.dot(dq)) * (line.dot(n).signum() / length),
                )
            }
        };
        if (direction.dot(n).abs() - 1.).abs() > 1e-10 || !rate.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let r = site.point - self.center_of_mass;
        let v = self.relative_velocity + w.cross(r);
        let slip = v - n * v.dot(n);
        let vtol =
            256. * f64::EPSILON * (1. + self.relative_velocity.length() + w.length() * r.length());
        if v.dot(n).abs() > vtol
            || (impact_stuck && slip.length() > vtol)
            || (!impact_stuck && slip.length() <= vtol)
        {
            return Ok(None);
        }
        let free = self.acceleration + w.cross(w.cross(r));
        // Anchor both material features at their initially coincident contact
        // points. Their signed infinite-feature gap is (p(t)-q)·n(t), also
        // for skew infinite lines. The initial offset and normal speed vanish.
        let bn = n.dot(free) + 2. * rate.dot(v);
        let response = |f: DVec3| f / mass + r.cross(f).cross(r) / inertia;
        let an = response(n);
        // Positive normal acceleration for every force direction in the cone
        // rules out simultaneous admissible release and compressive binding.
        let minimum = n.dot(an) - friction * (an - n * n.dot(an)).length();
        let atol = 256. * f64::EPSILON * (1. + free.length() + 2. * rate.length() * v.length());
        if !bn.is_finite() || bn.abs() <= atol || minimum <= 1e-12 * an.length() {
            return Ok(None);
        }
        let (mode, force) = if bn > 0. {
            (FiniteLoadMode::Opening, DVec3::ZERO)
        } else if !impact_stuck {
            let cone = n - friction * slip.normalize();
            (
                FiniteLoadMode::Sliding,
                (-bn / n.dot(response(cone))) * cone,
            )
        } else {
            let x = n.any_orthonormal_vector();
            let basis = [x, n.cross(x), n];
            let a = PointImpact {
                inverse_mass: std::array::from_fn(|i| {
                    std::array::from_fn(|j| basis[i].dot(response(basis[j])))
                }),
                relative_velocity: [basis[0].dot(free), basis[1].dot(free), bn],
                friction,
            };
            let Ok(s) = a.solve() else { return Ok(None) };
            (
                if s.sticking {
                    FiniteLoadMode::Sticking
                } else {
                    FiniteLoadMode::IncipientSlip
                },
                basis[0] * s.impulse[0] + basis[1] * s.impulse[1] + n * s.impulse[2],
            )
        };
        let constrained = bn + n.dot(response(force));
        if !force.is_finite() || !constrained.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Some(FinitePointLoad {
            mode,
            force,
            free_normal_acceleration: bn,
            constrained_normal_acceleration: constrained,
            relative_point_velocity: v,
            normal_rate: rate,
        }))
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::FlightBox;
    use glam::DQuat;
    fn run(
        q: DQuat,
        feature: SeparatingFeature,
        slip: DVec3,
        g: DVec3,
    ) -> (FiniteBoxFlight, FinitePointSite, SphericalPointMaterial) {
        let w = q * DVec3::new(0.8, -3., 5.);
        let r = q * DVec3::new(0.11, -0.23, -0.14);
        let ob = FlightBox {
            center: DVec3::ZERO,
            orientation: q,
            half_extent: DVec3::ONE,
        };
        let support = FlightBox {
            orientation: q * DQuat::from_rotation_x(0.41),
            ..ob
        };
        let n = match feature {
            SeparatingFeature::SupportFace(_) => support.orientation * DVec3::Z,
            SeparatingFeature::MovingFace(_) => q * DVec3::Z,
            SeparatingFeature::EdgePair(..) => (q * DVec3::X)
                .cross(support.orientation * DVec3::Y)
                .normalize(),
        };
        let slip = q * slip;
        (
            FiniteBoxFlight {
                moving: ob,
                support,
                center_of_mass: -r,
                relative_velocity: slip - n * n.dot(slip) - w.cross(r),
                angular_velocity: w,
                acceleration: q * g,
            },
            FinitePointSite {
                normal: n,
                point: DVec3::ZERO,
                feature,
            },
            SphericalPointMaterial {
                mass: 2.,
                inertia: 0.05,
                friction: 0.42,
            },
        )
    }
    #[test]
    fn measured_free_gap_curvature_and_constraint_residual_are_frame_invariant() {
        use SeparatingFeature::*;
        for q in [
            DQuat::IDENTITY,
            DQuat::from_rotation_y(-0.8) * DQuat::from_rotation_z(1.2),
        ] {
            for feature in [SupportFace(2), MovingFace(2), EdgePair(0, 1)] {
                for slip in [DVec3::ZERO, DVec3::new(0.43, 0.27, -0.6)] {
                    for g in [DVec3::Z * (-9.81), DVec3::Z * 20.] {
                        let (f, site, m) = run(q, feature, slip, g);
                        let load = f
                            .first_point_load(site, m, slip == DVec3::ZERO)
                            .unwrap()
                            .unwrap();
                        let gap = |t: f64| {
                            let delta = DQuat::from_scaled_axis(t * f.angular_velocity);
                            let p = f.relative_velocity * t
                                + 0.5 * t * t * f.acceleration
                                + (delta * (site.point - f.center_of_mass)
                                    - (site.point - f.center_of_mass));
                            let n = match feature {
                                SupportFace(_) => site.normal,
                                MovingFace(_) => delta * site.normal,
                                EdgePair(..) => (delta * (f.moving.orientation * DVec3::X))
                                    .cross(f.support.orientation * DVec3::Y)
                                    .normalize(),
                            };
                            p.dot(n)
                        };
                        let h = 3e-5;
                        assert!(
                            ((gap(h) + gap(-h)) / (h * h) - load.free_normal_acceleration).abs()
                                < 3e-6
                        );
                        if load.mode == FiniteLoadMode::Opening {
                            assert_eq!(load.force, DVec3::ZERO);
                            assert!(load.free_normal_acceleration > 0.)
                        } else {
                            assert!(load.constrained_normal_acceleration.abs() < 1e-11);
                            let j = load.force.dot(site.normal);
                            assert!(j > 0.);
                            assert!(
                                (load.force - j * site.normal).length() <= m.friction * j + 1e-10
                            );
                            assert!(load.force.dot(load.relative_point_velocity) <= 1e-11);
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn vertical_static_vertex_and_sliding_force_have_analytic_values() {
        let (mut f, site, mut m) = run(
            DQuat::IDENTITY,
            SeparatingFeature::MovingFace(2),
            DVec3::ZERO,
            -9.81 * DVec3::Z,
        );
        f.angular_velocity = DVec3::ZERO;
        f.relative_velocity = DVec3::ZERO;
        f.center_of_mass = site.point + 0.15 * DVec3::Z;
        let x = f.first_point_load(site, m, true).unwrap().unwrap();
        assert_eq!(x.mode, FiniteLoadMode::Sticking);
        assert!(x.force.distance(DVec3::Z * 19.62) < 1e-12);
        f.relative_velocity = DVec3::X;
        let x = f.first_point_load(site, m, false).unwrap().unwrap();
        assert_eq!(x.mode, FiniteLoadMode::Sliding);
        assert!(x.force.distance((DVec3::Z - m.friction * DVec3::X) * 19.62) < 1e-12);
        f.relative_velocity = DVec3::ZERO;
        f.acceleration += 80. * DVec3::X;
        m.friction = 0.1;
        assert_eq!(
            f.first_point_load(site, m, true).unwrap().unwrap().mode,
            FiniteLoadMode::IncipientSlip
        );
    }
}
