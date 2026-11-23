// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One-tick swept bounds for a floating two-sphere revolute and a separate ball.
use super::{Cut, MovingSpec, Routing};
use glam::{DQuat, DVec3};

const DT: f64 = 0.001;
const JOIN: f64 = 0.025;
const RELEASE: f64 = 0.075;
// Include native pose construction, COM-to-collider conversion, and differences
// of finite nonzero coordinates in the clearance estimate (metres).
const ROUNDING: f64 = 32768. * f64::EPSILON;
const MAX_ROUNDING_M: f64 = 1e-6;

pub(crate) struct RoutingBounds {
    sweep: Option<LinkSweep>,
}

impl RoutingBounds {
    pub fn new(spec: &MovingSpec) -> Self {
        let hinge = &spec.hinge;
        let arms = hinge.links.map(|l| {
            let offset = DQuat::from_array(l.orientation) * DVec3::from(l.inertia_center);
            (DVec3::from(l.position) + offset).distance(DVec3::from(hinge.anchor))
        });
        Self {
            sweep: LinkSweep::new(spec, arms),
        }
    }

    pub(crate) fn needs_worker(
        &self,
        spec: &MovingSpec,
        observation: &super::MovingObservation,
        pending_effort: Option<f64>,
        joined: bool,
    ) -> bool {
        let c = SweepState {
            spec,
            links: &observation.hinge.links,
            q: observation.hinge.q,
            sphere: &observation.sphere,
            pending_effort,
        };
        match self.decide(&c, observation.hinge.tick, &observation.reactions, joined) {
            Routing::Join => true,
            Routing::CheckRelease => false,
            Routing::Keep => joined,
        }
    }
    pub(super) fn next(&self, c: &Cut) -> Routing {
        self.decide(&c.into(), c.tick, &c.reactions, c.joined())
    }
    fn decide(
        &self,
        c: &SweepState<'_>,
        tick: u64,
        reactions: &[super::MovingReaction; 2],
        joined: bool,
    ) -> Routing {
        if joined
            && (tick == 0
                || reactions.iter().any(|r| {
                    r.active
                        || r.normal_impulse != 0.
                        || r.warmstart_normal_impulse != 0.
                        || r.tangent_impulse != [0.; 3]
                }))
        {
            return Routing::Keep;
        }
        if let Some((gap, outward)) = self.sweep.as_ref().and_then(|s| s.clearance(c)) {
            if joined && outward && gap > RELEASE {
                return Routing::CheckRelease;
            }
            if !joined && gap > JOIN {
                return Routing::Keep;
            }
        }
        if joined { Routing::Keep } else { Routing::Join }
    }
}

/// Only physical inputs from a committed state enter a one-sphere sweep.
struct SweepState<'a> {
    spec: &'a MovingSpec,
    links: &'a [crate::articulation::ArticulationLinkState; 2],
    sphere: &'a crate::articulation::ArticulationLinkState,
    q: f64,
    pending_effort: Option<f64>,
}
impl<'a> From<&'a Cut> for SweepState<'a> {
    fn from(c: &'a Cut) -> Self {
        Self {
            spec: &c.spec,
            links: &c.hinge.links,
            sphere: &c.sphere,
            q: c.hinge.q,
            pending_effort: c.pending_effort,
        }
    }
}
/// Bounds on Rapier 0.36.1's seven generalized velocities (root linear/angular
/// velocities and relative hinge rate). Derived from the two-link case of
/// Multibody::{update_acceleration,update_mass_matrix}, for an internally actuated hinge.
struct LinkSweep {
    substeps: usize,
    mass_floor: f64,
    mass_derivative: f64,
    force: f64,
    collider_speed: [f64; 2],
    coordinate_scale: f64,
    child_to_pivot: f64,
}

impl LinkSweep {
    fn new(spec: &MovingSpec, arms: [f64; 2]) -> Option<Self> {
        let [root, child] = spec.hinge.links;
        // Gershgorin eigenvalue bounds survive rotation into the world frame.
        let inertia = spec.hinge.links.map(|l| {
            let [xx, yy, zz, xy, xz, yz] = l.inertia_tensor;
            let radii = [
                xy.abs() + xz.abs(),
                xy.abs() + yz.abs(),
                xz.abs() + yz.abs(),
            ];
            [xx, yy, zz]
                .into_iter()
                .zip(radii)
                .fold((f64::INFINITY, 0.0_f64), |(lo, hi), (d, r)| {
                    (lo.min(d - r), hi.max(d + r))
                })
        });
        let [(rmin, rmax), (cmin, cmax)] = inertia;
        // qdot² <= 2(|omega_root|² + |omega_child|²). Root translation
        // alone supplies m_root*|v_root|² to the mass quadratic form.
        let mass_floor = root.mass.min(rmin / 3.).min(cmin / 2.);
        let mut coordinate_scale = 1.0_f64.max(DVec3::from(spec.hinge.anchor).length());
        for link in [root, child, spec.sphere] {
            coordinate_scale = coordinate_scale
                .max(DVec3::from(link.position).length())
                .max(DVec3::from(link.inertia_center).length())
                .max(link.radius);
        }
        // Pad arms for both authored coordinates and substep world coordinates.
        // clearance() declines the tighter bound if the current scene exceeds this budget.
        let length_error = ROUNDING * coordinate_scale;
        let [a, b] = arms.map(|v| v + MAX_ROUNDING_M);
        let jv = 1. + a + 2. * b;
        let mass_ceiling = root.mass + rmax + child.mass * jv * jv + 4. * cmax;
        // Discard poor scales before relying on the floating-point inertia
        // decomposition or native 7x7 solve. Uncertified geometry stays coupled.
        // Rapier reconstructs world angular inertia by inverting a 3x3
        // inverse inertia with explicit cofactors and a determinant. Absolute
        // scale matters too: bound its cubic intermediates away from f64 limits.
        if !(mass_floor >= 1e-48
            && mass_ceiling <= 1e48
            && mass_ceiling / mass_floor < 1e6
            && length_error < MAX_ROUNDING_M)
        {
            return None;
        }
        // With |u| <= S, the child's angular and linear Jacobian norms are
        // <= 2 and jv. Its zero-input bias accelerations are <= S² and
        // (a+5b) S² respectively. The gyro moment is <= I*|omega|².
        let force = rmax + jv * child.mass * (a + 5. * b) + 10. * cmax;
        // The angular/linear Coriolis Jacobian norms are <= 2S and (3a+10b)S.
        // Include the derivative of the gyro moment on BOTH links as well.
        let mass_derivative = 2. * rmax + jv * child.mass * (3. * a + 10. * b) + 20. * cmax;
        let root_offset = DVec3::from(root.inertia_center).length() + MAX_ROUNDING_M;
        let child_to_pivot =
            DVec3::from(child.position).distance(DVec3::from(spec.hinge.anchor)) + MAX_ROUNDING_M;
        Some(Self {
            substeps: gz_physics_multifidelity::MultiFidelityBackend::moving_solver_substeps(),
            mass_floor: mass_floor * 0.99,
            force: force * 1.01,
            mass_derivative: mass_derivative * 1.01,
            collider_speed: [1. + root_offset, 1. + a + 2. * child_to_pivot],
            coordinate_scale,
            child_to_pivot,
        })
    }

    /// The free-root native translation is its COM. spawn_articulation_link
    /// shifts the collider by -inertia_center and zeros the native local COM.
    fn speed_limit(&self, c: &SweepState<'_>) -> Option<f64> {
        let [root, child] = *c.links;
        let omega = DVec3::from(root.angular_velocity);
        let child_omega = DVec3::from(child.angular_velocity);
        // Native construction projects the relative angular velocity onto its
        // unit axis. At warm cuts finish_velocities publishes J*u on each link.
        let relative =
            (child_omega - omega).length() + 1e-9 * (omega.length() + child_omega.length());
        let mut speed = DVec3::new(
            DVec3::from(root.velocity).length(),
            omega.length(),
            relative,
        )
        .length()
            * (1. + 1e-9)
            + 1e-12;
        // Equal and opposite torques cancel in the common root columns of
        // the angular Jacobians. The hinge column projects onto its unit axis.
        let torque = 1.01 * c.pending_effort.unwrap_or(0.).abs();
        let h = DT / self.substeps as f64;
        for _ in 0..self.substeps {
            // The staged solver forms M+hC at the START of this substep. Its
            // unbiased retry has the same lower bound. The next speed bounds
            // both the velocity increment and the subsequent pose integration.
            let floor = self.mass_floor - h * self.mass_derivative * speed;
            if floor < 0.1 * self.mass_floor {
                return None;
            }
            speed += h * (self.force * speed * speed + torque) / floor;
            if !speed.is_finite() {
                return None;
            }
        }
        Some(speed)
    }

    fn clearance(&self, c: &SweepState<'_>) -> Option<(f64, bool)> {
        let speed = self.speed_limit(c)?;
        let pos = DVec3::from(c.sphere.position);
        let vel = DVec3::from(c.sphere.velocity);
        let advance = vel * DT;
        let span = advance.length_squared();
        let mut min_gap = f64::INFINITY;
        let mut outward = false;
        for (i, l) in c.links.iter().enumerate() {
            let at = DVec3::from(l.position);
            let dr = pos - at;
            let t = if span == 0. {
                0.
            } else {
                (-dr.dot(advance) / span).clamp(0., 1.)
            };
            // Native q is unwrapped; a large q can lose its small substep
            // increment even when the resulting world-space coordinates are small.
            let coordinate_error = ROUNDING
                * self
                    .coordinate_scale
                    .max(pos.length())
                    .max(at.length())
                    .max(advance.length());
            let error = coordinate_error + ROUNDING * c.q.abs() * self.child_to_pivot;
            let gap = (dr + t * advance).length()
                - c.spec.sphere.radius
                - c.spec.hinge.links[i].radius
                - DT * self.collider_speed[i] * speed
                - error;
            if !gap.is_finite() || error >= MAX_ROUNDING_M {
                return None;
            }
            if gap <= min_gap {
                let offset = DQuat::from_array(l.orientation)
                    * DVec3::from(c.spec.hinge.links[i].inertia_center);
                let origin_velocity =
                    DVec3::from(l.velocity) - DVec3::from(l.angular_velocity).cross(offset);
                let receding = dr.dot(vel - origin_velocity) > 0.;
                outward = receding && (gap < min_gap || outward);
                min_gap = gap;
            }
        }
        Some((min_gap, outward))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::moving::{Motion, Part, Resident, Result, Seed, crash_tests::spec};

    #[test]
    fn sweep_distinguishes_fast_transit_from_fast_clearance_and_refuses_uncertain_cuts()
    -> Result<()> {
        let spec = spec();
        // The native pair update first gathers rigid points inside prediction_distance.
        // These colliders have no soft CCD, soft-body margin or contact skin.
        assert!(JOIN > gz_physics_multifidelity::partition_prediction_distance());
        let bounds = RoutingBounds::new(&spec);
        let mut c = Cut::new(spec, 4)?;
        c.sphere.position = [3., 0.4, 0.5];
        c.sphere.velocity = [-6000., 0., 0.];
        assert_eq!(bounds.next(&c), Routing::Keep);
        c.pending_effort = Some(0.5);
        assert_eq!(bounds.next(&c), Routing::Keep);
        c.pending_effort = Some(-1e8);
        assert_eq!(bounds.next(&c), Routing::Join);
        c.pending_effort = None;
        c.hinge.q = 1e12;
        assert_eq!(bounds.next(&c), Routing::Join);
        c.hinge.q = 0.;
        c.hinge.links[0].angular_velocity = [0., 0., 1e3];
        assert_eq!(bounds.next(&c), Routing::Join);
        c.hinge.links[0].angular_velocity = [0.; 3];
        c.sphere.position[1] = 0.;
        assert_eq!(bounds.next(&c), Routing::Join);
        let mut bad = c.spec;
        bad.hinge.links[0].inertia_tensor = [1e-12, 1e-12, 1e-12, 0., 0., 0.];
        assert!(RoutingBounds::new(&bad).sweep.is_none());
        for scale in [1e-120, 1e120] {
            let mut bad = c.spec;
            for link in &mut bad.hinge.links {
                link.mass *= scale;
                link.inertia_tensor = link.inertia_tensor.map(|x| x * scale);
            }
            assert!(RoutingBounds::new(&bad).sweep.is_none());
        }
        Ok(())
    }

    #[test]
    fn net_drive_expands_a_near_margin_sweep_from_rest() -> Result<()> {
        let spec = spec();
        let mut c = Cut::new(spec, 2)?;
        let bounds = RoutingBounds::new(&spec);
        let sweep = bounds.sweep.as_ref().unwrap();
        // At rest the child clears the join margin by 0.01 mm. A bounded couple
        // fits the dynamics certificate but consumes this clearance allowance.
        c.sphere.position = c.hinge.links[1].position;
        c.sphere.position[1] += spec.hinge.links[1].radius + spec.sphere.radius + JOIN + 0.00001;
        c.sphere.velocity = [0.; 3];
        let idle = sweep.speed_limit(&(&c).into()).unwrap();
        for effort in [0.5, -0.5] {
            c.pending_effort = Some(effort);
            let speed = sweep.speed_limit(&(&c).into()).unwrap();
            assert_eq!(bounds.next(&c), Routing::Join);
            let mut native = Resident::load(&spec, 0, &Seed::Hinge(Motion::Cold(Some(effort))))?;
            native.advance()?;
            let Part::Hinge(_, after) = native.capture(&spec)? else {
                unreachable!()
            };
            assert!(after.qdot.abs() > idle);
            assert!(after.qdot.signum() == effort.signum());
            let root = after.links[0];
            let u = DVec3::new(
                DVec3::from(root.velocity).length(),
                DVec3::from(root.angular_velocity).length(),
                after.qdot,
            )
            .length();
            assert!(u <= speed);
            for (i, link) in after.links.iter().enumerate() {
                let travel =
                    DVec3::from(link.position).distance(DVec3::from(c.hinge.links[i].position));
                assert!(travel <= DT * speed * sweep.collider_speed[i]);
            }
        }
        for zero in [None, Some(0.), Some(-0.)] {
            c.pending_effort = zero;
            assert_eq!(sweep.speed_limit(&(&c).into()), Some(idle));
            assert_eq!(bounds.next(&c), Routing::Keep);
        }
        for effort in [1e4, f64::MAX, -f64::MAX] {
            c.pending_effort = Some(effort);
            assert!(sweep.speed_limit(&(&c).into()).is_none());
            assert_eq!(bounds.next(&c), Routing::Join);
        }
        Ok(())
    }

    #[test]
    fn uncertified_drive_and_residual_spin_keep_even_distant_bodies_together() -> Result<()> {
        let mut spec = spec();
        let torque = 1e8;
        let mut native = Resident::load(&spec, 0, &Seed::Hinge(Motion::Cold(Some(torque))))?;
        native.advance()?;
        let Part::Hinge(_, end) = native.capture(&spec)? else {
            unreachable!()
        };
        // Place the free sphere at the genuinely measured endpoint of a link.
        // The native integrator can move its free root far outside the ideal
        // COM enclosure in a single extreme torque tick.
        spec.sphere.position = end.links[0].position;
        spec.sphere.velocity = [0.; 3];
        let mut c = Cut::new(spec, 2)?;
        assert!(
            DVec3::from(c.hinge.links[0].position).distance(DVec3::from(spec.sphere.position))
                > 100.
        );
        let bounds = RoutingBounds::new(&spec);
        let sweep = bounds.sweep.as_ref().unwrap();
        assert_eq!(sweep.substeps, 4);
        c.pending_effort = Some(torque);
        assert!(sweep.speed_limit(&(&c).into()).is_none());
        assert_eq!(bounds.next(&c), Routing::Join);
        // Even after the command clears, the high outgoing speed defeats the
        // certificate. Moving the sphere another 1000 m away must not release.
        c.tick = 1;
        c.hinge = end;
        c.sphere.position[0] += 1000.;
        c.owners = [0, 0];
        c.pending_effort = None;
        assert!(sweep.speed_limit(&(&c).into()).is_none());
        assert_eq!(bounds.next(&c), Routing::Keep);
        Ok(())
    }

    #[test]
    fn rotating_offset_links_stay_inside_the_bound_under_native_integration() -> Result<()> {
        // Move the parent's authored collider 2.1 m from its COM, then tilt
        // the complete rig and initialize a compatible free spinning hinge.
        // This distinguishes the native free-root origin from the SDF origin.
        for spin in [0.05, -0.2, 0.3] {
            let mut spec = spec();
            let anchor = DVec3::from(spec.hinge.anchor);
            let rot = DQuat::from_axis_angle(DVec3::new(1., 2., 3.).normalize(), 0.7);
            let axis = rot * DVec3::Z;
            let omega = rot * DVec3::new(spin, -0.1, 0.08);
            let pivot_velocity = DVec3::new(0.07, 0.02, -0.05);
            spec.hinge.links[0].position[0] -= 2.1;
            spec.hinge.links[0].inertia_center[0] += 2.1;
            for (i, l) in spec.hinge.links.iter_mut().enumerate() {
                l.position = (anchor + rot * (DVec3::from(l.position) - anchor)).to_array();
                l.orientation = rot.to_array();
                let center = DVec3::from(l.position) + rot * DVec3::from(l.inertia_center);
                let w = if i == 0 { omega } else { omega + 0.07 * axis };
                l.velocity = (pivot_velocity + w.cross(center - anchor)).to_array();
                l.angular_velocity = w.to_array();
            }
            spec.hinge.axis = axis.to_array();
            let mut c = Cut::new(spec, 2)?;
            let sweep = RoutingBounds::new(&spec).sweep.unwrap();
            let mut native = Resident::load(&spec, 0, &Seed::Hinge(Motion::Cold(None)))?;
            for step in 0..96 {
                c.pending_effort = match step % 3 {
                    0 => None,
                    1 => Some(0.5),
                    _ => Some(-0.3),
                };
                if let Some(effort) = c.pending_effort {
                    native.effort(effort)?;
                }
                let prior = c.hinge.links;
                let speed = sweep
                    .speed_limit(&(&c).into())
                    .expect("well-conditioned native motion");
                native.advance()?;
                let Part::Hinge(_, state) = native.capture(&spec)? else {
                    unreachable!()
                };
                c.hinge = state;
                for (i, l) in prior.iter().enumerate() {
                    let travel =
                        DVec3::from(c.hinge.links[i].position).distance(DVec3::from(l.position));
                    assert!(
                        travel <= DT * speed * sweep.collider_speed[i],
                        "link {i} exceeded the route sweep: {travel}"
                    );
                }
            }
        }
        Ok(())
    }
}
