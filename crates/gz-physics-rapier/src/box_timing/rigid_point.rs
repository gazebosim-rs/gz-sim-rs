// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A measured isolated vertex impact on a free rigid body, followed by strictly outward flight.
use super::*;
#[path = "upper_point.rs"]
pub(crate) mod upper;
use crate::free_rotation::{RotationUpdate, attitude_interval};
use gz_physics::PointImpact;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RigidPointDecline {
    Environment,
    Material,
    Neighbor,
    State,
    Bracket,
    Law,
    Departure,
    Accuracy,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidPointImpactReceipt {
    pub impact: CornerImpactReceipt,
    pub corner: u8,
    /// Physical time after the start of this short pulse, before rounding to the actuator nanosecond.
    pub after_cut_s: f64,
    pub pulse_start: Duration,
    pub pulse_duration: Duration,
    /// Lower bound on outward normal acceleration at this vertex over the remaining pulse.
    pub outward_acceleration: f64,
}
pub(crate) struct RigidPointStep {
    pub pair: (ColliderHandle, ColliderHandle),
    pub(crate) opposing: Option<opponent::OpponentGuard>,
    body: RigidBodyHandle,
    end: Cut,
    pub hit: RigidPointImpactReceipt,
}
impl RigidPointStep {
    pub(crate) fn commit(self, e: &mut RapierBackend) -> RigidPointImpactReceipt {
        RotationUpdate::new(self.body, self.end.q, self.end.spin)
            .commit_with_translation(&mut e.world, Some((self.end.com, self.end.v)));
        self.hit
    }
}
#[derive(Clone, Copy)]
struct Cut {
    q: Rotation,
    spin: Vector,
    com: Vector,
    v: Vector,
}
#[derive(Clone, Copy)]
struct Rotor {
    axes: Rotation,
    inv: Vector,
    acc: Vector,
}
impl Rotor {
    fn inverse_at(&self, q: Rotation, j: Vector) -> Vector {
        let p = q * self.axes;
        p * (self.inv * (p.inverse() * j))
    }
    fn angular_momentum(&self, s: Cut) -> Vector {
        let p = s.q * self.axes;
        p * (self.inv.recip() * (p.inverse() * s.spin))
    }
    fn at(&self, start: Cut, t: f64) -> Result<Cut, RigidPointDecline> {
        if t == 0. {
            return Ok(start);
        }
        let p = start.q * self.axes;
        let l = self.angular_momentum(start);
        // Every proposed pulse is at most 4 microseconds. Independently compare
        // two resolutions of the conserved-momentum map before using the result.
        let eval = |n| {
            attitude_interval(p, l, Vector::ZERO, self.inv, t, n)
                .map_err(|_| RigidPointDecline::Accuracy)
        };
        let rough = eval(1)?;
        let exact = eval(2)?;
        let diff = rough * exact.inverse();
        if 2. * diff.xyz().length().atan2(diff.w.abs()) > 1e-12 {
            return Err(RigidPointDecline::Accuracy);
        }
        let q = (exact * self.axes.inverse()).normalize();
        Ok(Cut {
            q,
            spin: self.inverse_at(q, l),
            com: start.com + (start.v + 0.5 * t * self.acc) * t,
            v: start.v + self.acc * t,
        })
    }
}
struct Surface {
    arm: Vector,
    top: Vector,
    velocity: Vector,
    n: Vector,
}
impl Surface {
    fn gap(&self, s: Cut, t: f64) -> f64 {
        (s.com + s.q * self.arm - self.top - t * self.velocity).dot(self.n)
    }
    fn incoming(&self, s: Cut) -> Vector {
        s.v - self.velocity + s.spin.cross(s.q * self.arm)
    }
    fn solve(
        &self,
        rotor: &Rotor,
        cut: Cut,
        mass: f64,
        basis: [Vector; 3],
        mu: f64,
    ) -> Result<(Cut, Vector, gz_physics::PointImpactResponse, Vector), RigidPointDecline> {
        let arm = cut.q * self.arm;
        let relative = self.incoming(cut);
        let n = self.n;
        let response = |j: Vector| j / mass + rotor.inverse_at(cut.q, arm.cross(j)).cross(arm);
        let w = basis.map(|ax| basis.map(|ay| ax.dot(response(ay))));
        let law = PointImpact {
            inverse_mass: w,
            relative_velocity: basis.map(|u| u.dot(relative)),
            friction: mu,
        }
        .solve()
        .map_err(|_| RigidPointDecline::Law)?;
        let [jx, jy, jn] = law.impulse;
        let impulse = jx * basis[0] + jy * basis[1] + jn * n;
        if jn <= 0. || !law.dissipated_energy.is_finite() || law.dissipated_energy < 0. {
            return Err(RigidPointDecline::Law);
        }
        let after = Cut {
            v: cut.v + impulse / mass,
            spin: cut.spin + rotor.inverse_at(cut.q, arm.cross(impulse)),
            ..cut
        };
        let outgoing = self.incoming(after);
        let check = Vector::from(law.relative_velocity);
        let expected = check.x * basis[0] + check.y * basis[1] + check.z * n;
        if outgoing.distance(expected)
            > 1024. * f64::EPSILON * (1. + relative.length() + after.spin.length() * arm.length())
        {
            return Err(RigidPointDecline::Accuracy);
        }
        Ok((after, impulse, law, outgoing))
    }
    fn event(
        &self,
        rotor: &Rotor,
        start: Cut,
        upper: f64,
    ) -> Result<(f64, Cut), RigidPointDecline> {
        let at = |t| {
            let state = rotor.at(start, t)?;
            Ok((state, self.gap(state, t)))
        };
        let mut lo = 0.;
        let mut hi = upper;
        let mut a = self.gap(start, 0.);
        let (_, mut b) = at(hi)?;
        if !(a >= 0. && b <= 0.) {
            return Err(RigidPointDecline::Bracket);
        }
        let tol =
            64. * f64::EPSILON * (1. + start.com.length() + self.top.length() + self.arm.length());
        for _ in 0..48 {
            let t = if a == b {
                0.5 * (lo + hi)
            } else {
                lo + (hi - lo) * a / (a - b)
            };
            let (s, g) = at(t)?;
            if g.abs() <= tol {
                // A small distance residual can still hide a long interval at
                // grazing speed or at a very large world coordinate.
                let closing = -self.incoming(s).dot(self.n);
                if closing > 0. && (g.abs() + tol) / closing <= 0.25e-9 {
                    return Ok((t, s));
                }
                return Err(RigidPointDecline::Accuracy);
            }
            if !(lo < t && t < hi) {
                break;
            }
            if g > 0. {
                lo = t;
                a = g;
            } else {
                hi = t;
                b = g;
            }
        }
        Err(RigidPointDecline::Bracket)
    }
}
impl Candidate {
    fn point_pulse(
        &self,
        e: &RapierBackend,
        ids: (BodyId, BodyId),
        p: PulseReceipt,
        h: Duration,
        ballistic: bool,
        check_neighbors: bool,
    ) -> Result<RigidPointStep, RigidPointDecline> {
        use RigidPointDecline::*;
        let world = &e.world;
        let f = &world.bodies[self.foot];
        let d = &world.bodies[self.pad];
        if (!check_neighbors && world.colliders.len() != 2)
            || world.multibody_joints.rigid_body_link(self.foot).is_some()
            || self.links.len() != 1
            || f.is_sleeping()
            || world
                .narrow_phase
                .contact_pair(self.sole, self.deck)
                .is_some_and(|x| x.has_any_active_contact() || x.total_impulse_magnitude() > 0.)
            || self.kinematics(e).is_err()
        {
            return Err(Environment);
        }
        let a = &world.colliders[self.sole];
        let b = &world.colliders[self.deck];
        if !crate::collision::can_exchange_impulses(a, b, d.body_type())
            || a.contact_skin() != 0.
            || b.contact_skin() != 0.
            || CoefficientCombineRule::combine(
                a.restitution(),
                b.restitution(),
                a.restitution_combine_rule(),
                b.restitution_combine_rule(),
            ) != 0.
        {
            return Err(Material);
        }
        let mu = CoefficientCombineRule::combine(
            a.friction(),
            b.friction(),
            a.friction_combine_rule(),
            b.friction_combine_rule(),
        );
        let m = &f.mass_properties().local_mprops;
        let rotor = Rotor {
            axes: m.principal_inertia_local_frame,
            inv: m.inv_principal_inertia,
            acc: world.gravity * self.gravity_scale,
        };
        if (!ballistic && rotor.acc != Vector::ZERO)
            || (!f.gyroscopic_forces_enabled()
                && self.inertia.is_some()
                && f.angvel() != Vector::ZERO)
            || !rotor.inv.is_finite()
            || rotor.inv.min_element() <= 0.
        {
            return Err(State);
        }
        let half = a.shape().as_cuboid().ok_or(State)?.half_extents;
        let deck = b.shape().as_cuboid().ok_or(State)?.half_extents;
        let local = a.position_wrt_parent().ok_or(State)?;
        let plane = *d.position() * b.position_wrt_parent().ok_or(State)?;
        let k = p.corner;
        let shoe = Vector::new(
            if k & 2 == 0 { -half.x } else { half.x },
            if k & 1 == 0 { -half.y } else { half.y },
            -half.z,
        );
        let n = plane.rotation * Vector::Z;
        let surface = Surface {
            arm: *local * shoe - m.local_com,
            top: plane.translation + n * deck.z,
            velocity: d.linvel(),
            n,
        };
        let start = Cut {
            q: *f.rotation(),
            com: center(f),
            v: f.linvel(),
            spin: f.angvel(),
        };
        let opposing = if world.colliders.len() > 2 {
            let motion = self.pulse_motion(e, start.v - d.linvel(), start.spin, h.as_secs_f64());
            self.pulse_neighbors_clear(e, h, start.com, motion)
                .ok_or(Neighbor)?
        } else {
            None
        };
        let upper = p.arrival_by_ns.ok_or(Bracket)? as f64 * 1e-9;
        if upper > h.as_secs_f64() {
            return Err(Bracket);
        }
        let (time, cut) = surface.event(&rotor, start, upper)?;
        let arm = cut.q * surface.arm;
        let relative = surface.incoming(cut);
        let basis = [plane.rotation * Vector::X, plane.rotation * Vector::Y, n];
        let (after, impulse, law, outgoing) = surface.solve(&rotor, cut, self.mass, basis, mu)?;
        // PulseWindow has already bounded every other vertex and the finite
        // footprint from an upper bound on post-impact kinetic energy. The
        // loaded vertex starts at physical tangency with zero normal speed.
        let tail = h.as_secs_f64() - time;
        let jet = self.inertia.map_or(
            taylor::AngularJet {
                alpha: Vector::ZERO,
                beta: Vector::ZERO,
                snap: after.spin.length().powi(4),
            },
            |i| i.jet(cut.q, after.spin, tail),
        );
        let height = jet.height(
            arm,
            n,
            after.spin,
            (after.v - d.linvel(), rotor.acc),
            0.,
            cut.com.length() + arm.length(),
        );
        let outward = height.outward_acceleration(tail).ok_or(Departure)?;
        let end = rotor.at(after, tail)?;
        let final_gap = surface.gap(end, h.as_secs_f64());
        let tol =
            512. * f64::EPSILON * (1. + surface.top.length() + end.com.length() + arm.length());
        if !final_gap.is_finite() || final_gap < -tol {
            return Err(Departure);
        }
        let start = Duration::from_nanos(p.cut_ns);
        let hit = RigidPointImpactReceipt {
            corner: k,
            after_cut_s: time,
            pulse_start: start,
            pulse_duration: h,
            outward_acceleration: outward,
            impact: CornerImpactReceipt {
                elapsed: start + Duration::from_secs_f64(time),
                gap: surface.gap(cut, time),
                foot: ids.0,
                support: ids.1,
                point: (cut.com + arm).to_array(),
                normal: n.to_array(),
                impulse_on_foot: impulse.to_array(),
                before_relative_velocity: relative.to_array(),
                after_relative_velocity: outgoing.to_array(),
                friction: mu,
                sticking: law.sticking,
                kinetic_energy_change: impulse.dot(surface.velocity) - law.dissipated_energy,
                dissipated_energy: law.dissipated_energy,
            },
        };
        Ok(RigidPointStep {
            pair: (self.sole, self.deck),
            opposing,
            body: self.foot,
            end,
            hit,
        })
    }
}
impl BoxTiming {
    pub(crate) fn rigid_point(
        &mut self,
        e: &RapierBackend,
        elapsed: Duration,
        h: Duration,
    ) -> Option<RigidPointStep> {
        let pair = self.point_pair?;
        let receipt = self.receipt.pulses.last().copied()?;
        if receipt.cut_ns != elapsed.as_nanos() as u64
            || receipt.applied_ns != Some(h.as_nanos() as u64)
        {
            return None;
        }
        let result = self.candidate.as_ref()?.point_pulse(
            e,
            pair,
            receipt,
            h,
            self.ballistic,
            self.point_neighbors,
        );
        self.receipt
            .point_attempts
            .as_mut()?
            .push((receipt.cut_ns, result.as_ref().err().copied()));
        result.ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn root_is_local_to_the_finite_bracket_and_refuses_an_unresolved_graze() {
        let rotor = Rotor {
            axes: Rotation::IDENTITY,
            inv: Vector::ONE,
            acc: Vector::new(0., 0., -9.81),
        };
        let plane = Surface {
            arm: Vector::new(0.1, -0.05, -0.12),
            top: Vector::new(0., 0., 0.3),
            velocity: Vector::new(0.23, -0.27, 0.3),
            n: Vector::Z,
        };
        let at = 218.739e-9;
        let mut start = Cut {
            q: Rotation::IDENTITY,
            spin: Vector::ZERO,
            com: plane.top - plane.arm,
            v: Vector::new(0.23, 0.23, -1.2),
        };
        start.com.z += (plane.velocity.z - start.v.z) * at - 0.5 * rotor.acc.z * at * at;
        let (t, end) = plane.event(&rotor, start, 310e-9).unwrap();
        assert!((t - at).abs() < 1e-13 && plane.gap(end, t).abs() < 1e-13);
        assert!(matches!(
            plane.event(&rotor, start, 120e-9),
            Err(RigidPointDecline::Bracket)
        ));
        let no_gravity = Rotor {
            acc: Vector::ZERO,
            ..rotor
        };
        start.com = plane.top - plane.arm + Vector::Z * 2e-15;
        start.v = plane.velocity - Vector::Z * 1e-8;
        assert!(matches!(
            plane.event(&no_gravity, start, 1e-6),
            Err(RigidPointDecline::Accuracy)
        ));
    }
    #[test]
    fn changed_material_filter_skin_or_external_force_declines_the_measured_pulse() {
        use gz_physics::{BodySpec, DynamicBackend, Inertia, RotationSpec};
        let mut e = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let half = Vector::new(0.18, 0.10, 0.06);
        let q = Rotation::from_rotation_y(0.17) * Rotation::from_rotation_x(0.21);
        let (corner, lowest) = (0..4u8)
            .map(|k| {
                let p = Vector::new(
                    if k & 2 == 0 { -half.x } else { half.x },
                    if k & 1 == 0 { -half.y } else { half.y },
                    -half.z,
                );
                (k, (q * p).z)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        let f = e
            .spawn_link(
                BodySpec {
                    shape: gz_physics::Shape::Box {
                        half_extents: half.to_array(),
                    },
                    position: [0., 0., -lowest + 4e-9],
                    velocity: [0., 0., -1.],
                    mass: Some(2.),
                },
                RotationSpec {
                    orientation: q.to_array(),
                    ..Default::default()
                },
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.08, 0.08, 0.08, 0., 0., 0.],
                }),
            )
            .unwrap();
        let d = e
            .spawn(BodySpec {
                shape: gz_physics::Shape::Box {
                    half_extents: [0.5, 0.4, 0.04],
                },
                position: [0., 0., -0.04],
                velocity: [0.; 3],
                mass: None,
            })
            .unwrap();
        let ids = (f, d);
        let c = Candidate::new(&e, e.bodies[&f].0, e.bodies[&d].0).unwrap();
        let p = PulseReceipt {
            corner,
            cut_ns: 0,
            arrival_by_ns: Some(5),
            applied_ns: Some(24),
            decline: None,
        };
        let trial =
            |e: &RapierBackend| c.point_pulse(e, ids, p, Duration::from_nanos(24), false, false);
        let h = trial(&e).unwrap();
        assert_eq!(h.hit.impact.elapsed, Duration::from_nanos(4));
        assert!(h.hit.outward_acceleration > 0.);
        e.world.colliders[c.deck].set_restitution(0.5);
        assert!(matches!(trial(&e), Err(RigidPointDecline::Material)));
        e.world.colliders[c.deck].set_restitution(0.0);
        e.world.colliders[c.sole].set_contact_skin(1e-5);
        assert!(matches!(trial(&e), Err(RigidPointDecline::Material)));
        e.world.colliders[c.sole].set_contact_skin(0.);
        e.set_self_collision_group(f, Some(23)).unwrap();
        e.set_self_collision_group(d, Some(23)).unwrap();
        assert!(matches!(trial(&e), Err(RigidPointDecline::Material)));
        e.set_self_collision_group(d, None).unwrap();
        e.add_world_wrench(f, [1., 0., 0.], [0.; 3]).unwrap();
        assert!(matches!(trial(&e), Err(RigidPointDecline::Environment)));
    }
}
