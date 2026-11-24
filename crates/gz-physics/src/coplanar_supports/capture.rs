// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Equilibrium of an entire sole instantaneously at the second belt's speed
//! while sliding over the first belt. Static pressure/traction is distributed
//! across the finite polygon, including yaw moments from transverse traction.
mod simplex;
use super::*;
type Wrench = [f64; 6];
#[derive(Clone, Copy, Debug)]
pub struct BeltVertexReaction {
    pub pad: usize,
    pub position: DVec2,
    pub normal_force: f64,
    pub tangent_force: DVec2,
}
#[derive(Clone, Debug)]
pub struct BeltEquilibriumWitness {
    pub fraction_b: f64,
    pub reactions: Vec<BeltVertexReaction>,
    pub dissipated_power: f64,
    pub prescribed_support_power: f64,
}
/// Virtual linear and angular velocity at the assembly COM. Its power against
/// every unit admissible contact wrench is at least zero on the sliding pad
/// and at least `objective` on the stationary pad.
#[derive(Clone, Copy, Debug)]
pub struct BeltDualBound {
    pub linear: glam::DVec3,
    pub angular: glam::DVec3,
    pub objective: f64,
    /// Virtual work of the unit-normal target wrench.
    pub target_work: f64,
}
#[derive(Clone, Debug)]
pub struct BeltCaptureRange {
    /// Every intermediate fraction is attainable by convex mixing.
    pub inner: [f64; 2],
    /// Dual bounds cover the continuous circular static cone at every vertex.
    pub outer: [f64; 2],
    pub endpoints: [BeltEquilibriumWitness; 2],
    pub certificates: [BeltDualBound; 2],
}
#[derive(Clone, Debug)]
pub enum BeltCapture {
    Feasible(Box<BeltCaptureRange>),
    /// A separating wrench has negative target work even after including the
    /// maximizing tangential direction at every static vertex.
    Infeasible {
        separation: f64,
        certificate: BeltDualBound,
    },
    Unresolved,
}
#[derive(Clone, Copy)]
struct Column {
    site: usize,
    p: DVec2,
    k: DVec2,
    w: Wrench,
}
#[derive(Clone, Copy)]
struct Problem<'a> {
    body: CoplanarSlidingContact,
    patches: [&'a [DVec2]; 2],
    kinetic: DVec2,
    mu: f64,
    length: f64,
    rhs: Wrench,
}
fn dot(a: Wrench, b: Wrench) -> f64 {
    a.into_iter().zip(b).map(|(u, v)| u * v).sum()
}
impl Problem<'_> {
    fn col(self, site: usize, p: DVec2, k: DVec2) -> Column {
        let r = (p - self.body.center) / self.length;
        let h = self.body.height / self.length;
        Column {
            site,
            p,
            k,
            w: [1., k.x, k.y, r.y + h * k.y, -r.x - h * k.x, r.perp_dot(k)],
        }
    }
    fn price(self, p: DVec2, y: Wrench) -> Column {
        let r = (p - self.body.center) / self.length;
        let h = self.body.height / self.length;
        let gradient = DVec2::new(y[1] - h * y[4] - r.y * y[5], y[2] + h * y[3] + r.x * y[5]);
        self.col(1, p, -self.mu * gradient.normalize_or_zero())
    }
    fn witness(
        self,
        columns: &[Column],
        weights: &[(usize, f64)],
    ) -> Option<BeltEquilibriumWitness> {
        let n = -self.body.mass * self.body.gravity_normal;
        let mut wrench = [0.; 6];
        let mut b = 0.;
        let mut reactions = vec![];
        let mut diss = 0.;
        let mut work = 0.;
        for &(i, w) in weights {
            let c = columns[i];
            for (x, v) in wrench.iter_mut().zip(c.w) {
                *x += w * v;
            }
            if c.site == 1 {
                b += w;
            }
            let normal = n * w;
            let tangent = c.k * normal;
            diss -= tangent.dot(self.body.velocity - self.body.pad_velocities[c.site]);
            work += tangent.dot(self.body.pad_velocities[c.site]);
            reactions.push(BeltVertexReaction {
                pad: c.site,
                position: c.p,
                normal_force: normal,
                tangent_force: tangent,
            });
        }
        let scale =
            1. + self.mu.max(self.kinetic.length()) + self.rhs[1].abs().max(self.rhs[2].abs());
        if wrench
            .into_iter()
            .zip(self.rhs)
            .any(|(u, v)| (u - v).abs() > 2e-10 * scale)
            || !(0. ..=1. + 1e-10).contains(&b)
        {
            return None;
        }
        Some(BeltEquilibriumWitness {
            fraction_b: b.clamp(0., 1.),
            reactions,
            dissipated_power: diss,
            prescribed_support_power: work,
        })
    }
    fn extreme(self, sign: f64, rotation: f64) -> Option<Extreme> {
        let mut columns = self.patches[0]
            .iter()
            .map(|&p| self.col(0, p, self.kinetic))
            .collect::<Vec<_>>();
        for &p in self.patches[1] {
            for j in 0..16 {
                columns.push(self.col(
                    1,
                    p,
                    self.mu * DVec2::from_angle(rotation + std::f64::consts::TAU * j as f64 / 16.),
                ));
            }
        }
        let mut best = None;
        for _ in 0..40 {
            let Some(solved) = simplex::solve(&columns, self.rhs, sign) else {
                break;
            };
            let (dual, goal, witness) = match solved {
                simplex::LinearSolution::Optimal { weights, dual } => {
                    (dual, sign, Some(self.witness(&columns, &weights)?))
                }
                simplex::LinearSolution::Infeasible { dual } => (dual, 0., None),
            };
            let mut violation = 0f64;
            let mut additions = vec![];
            for &p in self.patches[0] {
                violation = violation.max(-dot(dual, self.col(0, p, self.kinetic).w));
            }
            for &p in self.patches[1] {
                let c = self.price(p, dual);
                let v = goal - dot(dual, c.w);
                violation = violation.max(v);
                if v > 5e-12
                    && !columns.iter().any(|old| {
                        old.site == 1 && old.p == p && old.k.distance(c.k) <= 2e-13 * (1. + self.mu)
                    })
                {
                    additions.push(c);
                }
            }
            // Increasing the dual coefficient of unit total pressure by the
            // maximum violation makes it feasible over both continuous cones.
            let roundoff = 4096.
                * f64::EPSILON
                * (1. + self.mu.max(self.kinetic.length()) + self.rhs[1].abs() + self.rhs[2].abs())
                * (1. + dual.iter().map(|v| v.abs()).sum::<f64>());
            let mut feasible = dual;
            feasible[0] += violation + roundoff;
            let upper = dot(feasible, self.rhs);
            let certificate = BeltDualBound {
                linear: glam::DVec3::new(feasible[1], feasible[2], feasible[0]),
                angular: glam::DVec3::new(feasible[3], feasible[4], feasible[5]) / self.length,
                objective: goal,
                target_work: upper,
            };
            if !upper.is_finite() {
                return None;
            }
            if let Some(w) = witness {
                let value = sign * w.fraction_b;
                if upper < value - 2e-10 {
                    return None;
                }
                let gap = (upper - value).max(0.);
                if best.as_ref().is_none_or(|(_, _, g)| gap < *g) {
                    best = Some((w, certificate, gap));
                }
                if gap < 2e-9 {
                    break;
                }
            } else if upper < -5e-10 {
                return Some(Extreme::Infeasible(certificate));
            }
            if additions.is_empty() {
                break;
            }
            columns.extend(additions);
        }
        best.map(|(w, b, _)| Extreme::Bound(w, b))
    }
}
enum Extreme {
    Bound(BeltEquilibriumWitness, BeltDualBound),
    Infeasible(BeltDualBound),
}
impl CoplanarSlidingContact {
    /// Right-hand response initiating uniform sole slip at the previously
    /// stationary second belt. Maximum dissipation sets its shear opposite
    /// the emerging relative acceleration. The specified normal fraction
    /// must also sustain zero angular acceleration on the actual polygons.
    pub fn begin_second_belt_slip(
        self,
        patches: [&[DVec2]; 2],
        fraction_b: f64,
    ) -> Result<Option<SlidingLoadChoice>, PhysicsError> {
        if !fraction_b.is_finite()
            || !(0.0..=1.).contains(&fraction_b)
            || !self.kinetic_friction[1].is_finite()
            || self.kinetic_friction[1] < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let mut incoming = self;
        incoming.kinetic_friction[1] = 0.;
        let Some(traction) = incoming.tractions(patches)? else {
            return Ok(None);
        };
        if self.velocity != self.pad_velocities[1] {
            return Ok(None);
        }
        let ng = -self.gravity_normal;
        let drive = self.gravity_tangent + ng * (1. - fraction_b) * traction[0];
        let speed = drive.length();
        let opposing = ng * fraction_b * self.kinetic_friction[1];
        let tolerance =
            512. * f64::EPSILON * (ng + self.gravity_tangent.length() + opposing + speed);
        if !speed.is_finite() || !opposing.is_finite() || speed <= opposing + tolerance {
            return Ok(None);
        }
        let axis = drive / speed;
        let mut right = self;
        right.velocity = DVec2::ZERO;
        right.pad_velocities = [
            (self.pad_velocities[0] - self.velocity).normalize_or_zero(),
            -axis,
        ];
        let Some(choice) = right.solve(patches)? else {
            return Ok(None);
        };
        let Some(mut applied) = choice.choose(patches, fraction_b)? else {
            return Ok(None);
        };
        // The virtual unit velocities above select directions only. Work is
        // evaluated at the body's real velocity and both real pad velocities.
        applied.prescribed_support_power = 0.;
        applied.dissipated_power = 0.;
        for i in 0..2 {
            applied.prescribed_support_power +=
                applied.tangential_force[i].dot(self.pad_velocities[i]);
            applied.dissipated_power -=
                applied.tangential_force[i].dot(self.velocity - self.pad_velocities[i]);
        }
        Ok(Some(applied))
    }
    /// Compute feasible rest at the second pad's prescribed translation when
    /// the first pad slides with kinetic friction. The sole is initially flat
    /// and nonspinning. Feasibility alone does not select this future motion.
    pub fn stationary_second_belt(
        self,
        patches: [&[DVec2]; 2],
        static_friction: f64,
    ) -> Result<BeltCapture, PhysicsError> {
        if !static_friction.is_finite()
            || static_friction < 0.
            || self
                .kinetic_friction
                .iter()
                .any(|v| !v.is_finite() || *v < 0.)
        {
            return Err(PhysicsError::InvalidValue);
        }
        let mut sliding = self;
        sliding.kinetic_friction[1] = 0.;
        let Some(k) = sliding.tractions(patches)? else {
            return Ok(BeltCapture::Unresolved);
        };
        if self.velocity != self.pad_velocities[1] {
            return Ok(BeltCapture::Unresolved);
        }
        let length = patches
            .iter()
            .flat_map(|p| p.iter())
            .map(|p| p.distance(self.center))
            .fold(self.height, f64::max);
        if !length.is_finite() || length <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        for p in patches {
            Patch::from(p, self.center, length)?;
        }
        let t = self.gravity_tangent / self.gravity_normal;
        if !t.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let p = Problem {
            body: self,
            patches,
            kinetic: k[0],
            mu: static_friction,
            length,
            rhs: [1., t.x, t.y, 0., 0., 0.],
        };
        let resolve = |sign| match p
            .extreme(sign, 0.)
            .or_else(|| p.extreme(sign, 0.157693))
            .or_else(|| p.extreme(sign, -0.08391))
        {
            Some(Extreme::Bound(w, b)) => Ok((w, b)),
            Some(Extreme::Infeasible(certificate)) => Err(BeltCapture::Infeasible {
                separation: -certificate.target_work,
                certificate,
            }),
            None => Err(BeltCapture::Unresolved),
        };
        let (low, lo) = match resolve(-1.) {
            Ok(v) => v,
            Err(v) => return Ok(v),
        };
        let (high, hi) = match resolve(1.) {
            Ok(v) => v,
            Err(v) => return Ok(v),
        };
        Ok(BeltCapture::Feasible(Box::new(BeltCaptureRange {
            inner: [low.fraction_b, high.fraction_b],
            outer: [
                (-lo.target_work).clamp(0., low.fraction_b),
                hi.target_work.clamp(high.fraction_b, 1.),
            ],
            endpoints: [low, high],
            certificates: [lo, hi],
        })))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn model() -> CoplanarSlidingContact {
        CoplanarSlidingContact {
            mass: 2.,
            height: 0.2,
            gravity_tangent: DVec2::ZERO,
            gravity_normal: -9.81,
            center: DVec2::ZERO,
            velocity: DVec2::X,
            pad_velocities: [DVec2::ZERO, DVec2::X],
            kinetic_friction: [0.45, 0.2],
        }
    }
    #[test]
    fn stationary_whole_sole_requires_static_capacity_and_an_available_normal_lever() {
        let shape = |x0, x1| {
            [
                DVec2::new(x0, -0.3),
                DVec2::new(x1, -0.3),
                DVec2::new(x1, 0.3),
                DVec2::new(x0, 0.3),
            ]
        };
        let pa = shape(-1., -0.3);
        let pb = shape(0.3, 1.);
        let c = model();
        for angle in [0., 0.231, 1.73] {
            let q = DVec2::from_angle(angle);
            let a = pa.map(|p| q.rotate(p));
            let b = pb.map(|p| q.rotate(p));
            let c = CoplanarSlidingContact {
                velocity: q,
                pad_velocities: [DVec2::ZERO, q],
                ..c
            };
            let BeltCapture::Feasible(r) = c.stationary_second_belt([&a, &b], 0.3).unwrap() else {
                panic!("feasible but unreported")
            };
            for (i, want) in [0.6, 1. / 1.3].into_iter().enumerate() {
                assert!((r.inner[i] - want).abs() < 3e-9, "{:?}", r.inner);
                assert!((r.outer[i] - want).abs() < 3e-9);
            }
            assert!(matches!(
                c.stationary_second_belt([&a, &b], 0.05).unwrap(),
                BeltCapture::Infeasible { .. }
            ));
        }
    }
    #[test]
    fn static_pressure_and_shear_may_have_different_centers_on_a_finite_pad() {
        let c = CoplanarSlidingContact {
            kinetic_friction: [0.35, 0.2],
            ..model()
        };
        let a = [DVec2::new(-1., 0.1)];
        let b = [DVec2::new(1., -0.6), DVec2::new(1., 0.4)];
        let BeltCapture::Feasible(r) = c.stationary_second_belt([&a, &b], 0.7).unwrap() else {
            panic!()
        };
        for w in r.endpoints {
            assert!((w.fraction_b - 0.5).abs() < 1e-10);
            let mut v = DVec2::ZERO;
            let mut m = glam::DVec3::ZERO;
            for p in w.reactions {
                assert!(p.tangent_force.length() <= p.normal_force * [0.35, 0.7][p.pad] + 1e-12);
                v += p.tangent_force;
                m += (p.position - c.center)
                    .extend(-c.height)
                    .cross(p.tangent_force.extend(p.normal_force));
            }
            assert!(v.length() < 1e-9 && m.length() < 1e-9);
            assert!(
                w.dissipated_power > 0.
                    && (w.dissipated_power - w.prescribed_support_power).abs() < 1e-10
            );
        }
    }
}

#[cfg(test)]
mod emerging_tests {
    use super::*;
    #[test]
    fn different_normal_fractions_permit_stick_and_immediate_slip_from_the_same_state() {
        let c = CoplanarSlidingContact {
            mass: 2.,
            center: DVec2::ZERO,
            height: 0.13,
            gravity_normal: -9.81,
            gravity_tangent: DVec2::ZERO,
            velocity: DVec2::X,
            pad_velocities: [DVec2::ZERO, DVec2::X],
            kinetic_friction: [0.34, 0.36],
        };
        let a = [
            DVec2::new(-0.18, -0.10),
            DVec2::new(-0.01, -0.1),
            DVec2::new(-0.01, 0.1),
            DVec2::new(-0.18, 0.1),
        ];
        let b = [
            DVec2::new(0.01, -0.1),
            DVec2::new(0.18, -0.1),
            DVec2::new(0.18, 0.1),
            DVec2::new(0.01, 0.1),
        ];
        let BeltCapture::Feasible(stuck) = c.stationary_second_belt([&a, &b], 0.36).unwrap() else {
            panic!()
        };
        assert!(stuck.inner[0] < 0.5 && stuck.inner[1] > 0.5);
        let start = c.begin_second_belt_slip([&a, &b], 0.32).unwrap().unwrap();
        assert!(
            start
                .acceleration
                .distance(DVec2::new(9.81 * (0.36 * 0.32 - 0.34 * 0.68), 0.))
                < 1e-13
        );
        assert!(
            (start.prescribed_support_power
                - start.dissipated_power
                - (start.tangential_force[0] + start.tangential_force[1]).dot(c.velocity))
            .abs()
                < 1e-12
        );
        assert!(c.begin_second_belt_slip([&a, &b], 0.6).unwrap().is_none());
    }
}
