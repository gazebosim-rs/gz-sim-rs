// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact gravity-plus-unilateral-linear-spring motion for a centered sole.
use super::*;
use std::f64::consts::TAU;

#[derive(Clone, Copy)]
struct Drop {
    x: f64, // signed indentation
    u: f64, // closing speed relative to the deck
    peak: f64,
    impulse: f64,
}
fn checked(drop: Drop) -> Result<Drop, SoleError> {
    [drop.x, drop.u, drop.peak, drop.impulse]
        .into_iter()
        .all(f64::is_finite)
        .then_some(drop)
        .ok_or(SoleError::Nonfinite)
}

fn propagate(mut d: Drop, dt: f64, mass: f64, k: f64, g: f64) -> Result<Drop, SoleError> {
    // Preserve a representable frequency when the squared ratio lies
    // outside binary64 (very light/heavy bodies and very soft/stiff layers).
    let ratio = k / mass;
    let w = if ratio.is_normal() {
        ratio.sqrt()
    } else {
        k.sqrt() / mass.sqrt()
    };
    if !w.is_finite() || w <= 0. {
        return Err(SoleError::Nonfinite);
    }
    let equilibrium = g / w / w;
    let mut left = dt;
    for _ in 0..6 {
        if left <= 0. {
            return checked(d);
        }
        if d.x < 0. || (d.x == 0. && d.u < 0.) {
            let hit_speed = d.u.hypot((-2. * g * d.x).sqrt());
            let hit = if g == 0. {
                if d.u > 0. { -d.x / d.u } else { f64::INFINITY }
            } else if d.u < 0. {
                (hit_speed - d.u) / g
            } else {
                -2. * d.x / (d.u + hit_speed)
            };
            if left < hit || !hit.is_finite() {
                d.x += left * (d.u + 0.5 * g * left);
                d.u += g * left;
                return checked(d);
            }
            d.x = 0.;
            d.u = hit_speed;
            left -= hit;
            continue;
        }

        // Whole bounces return to exactly the same surface state. Skipping
        // complete cycles bounds the work even for long public intervals.
        if d.x == 0. && d.u > 0. && g > 0. {
            let contact = (TAU - 2. * (d.u * w).atan2(g)) / w;
            let period = contact + 2. * d.u / g;
            if left >= period && period > 0. {
                let tail = left % period;
                d.impulse += mass * g * (left - tail);
                d.peak = d.peak.max(equilibrium + equilibrium.hypot(d.u / w));
                left = tail;
                if left == 0. {
                    return checked(d);
                }
            }
        }
        let a = d.x - equilibrium;
        let b = d.u / w;
        let radius = a.hypot(b);
        let phase = b.atan2(a);
        // At a grazing minimum the spring immediately reloads. At release the
        // outward speed can also be computed from the oscillator invariant.
        let release = if radius > equilibrium {
            (phase + (-equilibrium / radius).acos()).rem_euclid(TAU) / w
        } else {
            f64::INFINITY
        };
        let h = left.min(release);
        let theta = w * h;
        let (sin, cos) = theta.sin_cos();
        let one_minus_cos = 2. * (0.5 * theta).sin().powi(2);
        let theta_minus_sin = if theta.abs() < 0.001 {
            theta.powi(3) * (1. / 6. - theta * theta / 120. + theta.powi(4) / 5040.)
        } else {
            theta - sin
        };
        d.impulse +=
            ((k / w) * (d.x * sin + b * one_minus_cos + equilibrium * theta_minus_sin)).max(0.);
        let x = d.x - a * one_minus_cos + b * sin;
        let u = d.u * cos - w * a * sin;
        d.peak = d.peak.max(d.x).max(x);
        if h >= phase.rem_euclid(TAU) / w {
            d.peak = d.peak.max(equilibrium + radius)
        }
        if h < release {
            d.x = x.max(0.);
            d.u = u;
            return checked(d);
        }
        d.x = 0.;
        d.u = -u.abs();
        left -= h;
    }
    Err(SoleError::Nonfinite)
}

impl SoleBody {
    /// A centered normal trajectory has no tangential velocity, acceleration,
    /// spin or moment and its entire sole is supported. The returned height
    /// is the deck-frame COM coordinate where the lower face first touches.
    pub(super) fn centered_normal_state(
        &self,
        state: &SoleState,
        deck: &SoleDeck,
        gravity: DVec3,
        force: DVec3,
        torque: DVec3,
        dt: f64,
    ) -> Result<Option<(SoleState, DVec3, f64)>, SoleError> {
        if !self.valid(*state, *deck, 0)
            || !gravity.is_finite()
            || !force.is_finite()
            || !torque.is_finite()
            || !dt.is_finite()
            || dt <= 0.
        {
            return Err(SoleError::InvalidInput);
        }
        if state.spin != DVec3::ZERO
            || torque != DVec3::ZERO
            || self.foot_offset.x != 0.
            || self.foot_offset.y != 0.
        {
            return Ok(None);
        }
        let s = state.in_deck(*deck);
        let acc = deck.orientation.conjugate() * (gravity + force / self.mass);
        if !s.finite() || !acc.is_finite() {
            return Err(SoleError::Nonfinite);
        }
        if acc.x != 0.
            || acc.y != 0.
            || s.orientation.x != 0.
            || s.orientation.y != 0.
            || s.velocity.x != 0.
            || s.velocity.y != 0.
        {
            return Ok(None);
        }
        let corners = self.corners(s);
        if corners.iter().any(|p| !p.is_finite()) {
            return Err(SoleError::Nonfinite);
        }
        if corners
            .iter()
            .any(|p| p.x.abs() > deck.half_extents.x || p.y.abs() > deck.half_extents.y)
        {
            return Ok(None);
        }
        let contact_center = deck.half_extents.z + self.half_extents.z - self.foot_offset.z;
        if !contact_center.is_finite() {
            return Err(SoleError::Nonfinite);
        }
        Ok(Some((s, acc, contact_center)))
    }

    /// Closed-form alternative for an undamped, centered, fully covered,
    /// unspun sole under constant normal acceleration into a translating deck.
    /// None leaves the state unchanged so the caller can use the 3D solver.
    #[allow(clippy::too_many_arguments)]
    pub fn try_flat_elastic_step(
        &self,
        state: &mut SoleState,
        deck: &SoleDeck,
        material: &SoleMaterial,
        gravity: DVec3,
        dt: f64,
        force: DVec3,
        torque: DVec3,
    ) -> Result<Option<SoleStep>, SoleError> {
        let Some((mut s, acc, contact_center)) =
            self.centered_normal_state(state, deck, gravity, force, torque, dt)?
        else {
            return Ok(None);
        };
        if material.alpha != 0. || acc.z > 0. {
            return Ok(None);
        }
        // Keep the actual COM trajectory in free flight. Recovering COM from
        // a large negative sole indentation can erase a small translation when
        // an authored link places the sole far from its center of mass.
        if let Some(end) =
            self.free_world(*state, *deck, gravity + force / self.mass, dt, torque)?
        {
            *state = end;
            return Ok(Some(SoleStep::default()));
        }
        let stiffness = material
            .elastic
            .sample(1.)
            .ok_or(SoleError::Nonfinite)?
            .pressure_pa
            * (4. * self.half_extents.x * self.half_extents.y);
        let x = contact_center - s.center.z;
        let d = propagate(
            Drop {
                x,
                u: -s.velocity.z,
                peak: x.max(0.),
                impulse: 0.,
            },
            dt,
            self.mass,
            stiffness,
            -acc.z,
        )?;
        if d.peak > self.half_extents.z {
            return Err(SoleError::OutsideCoating);
        }
        s.center.z = contact_center - d.x;
        s.velocity.z = -d.u;
        let end = deck.center + deck.velocity * dt;
        let world = s.world(*deck, end);
        let mut out = SoleStep {
            contact: self
                .patch(s, deck.half_extents, *material, 0)?
                .world(*deck, end),
            impulse: deck.orientation * (DVec3::Z * d.impulse),
            normal_impulse: d.impulse,
            peak_normal_force: stiffness * d.peak,
            ..Default::default()
        };
        out.peak_normal_force = out.peak_normal_force.max(out.contact.normal_force);
        if !world.finite()
            || !out.contact.finite()
            || !out.impulse.is_finite()
            || !out.peak_normal_force.is_finite()
        {
            return Err(SoleError::Nonfinite);
        }
        *state = world;
        Ok(Some(out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    fn start(x: f64, u: f64) -> Drop {
        Drop {
            x,
            u,
            peak: x.max(0.),
            impulse: 0.,
        }
    }
    #[test]
    fn free_flight_across_touch_and_release() {
        let d = propagate(start(-0.1, 1.), 0.1 + PI / 10. + 0.02, 2., 200., 0.).unwrap();
        assert!((d.x + 0.02).abs() < 1e-14);
        assert!((d.u + 1.).abs() < 1e-14);
        assert!((d.impulse - 4.).abs() < 1e-13);
        assert!((d.peak - 0.1).abs() < 1e-14);
    }
    #[test]
    fn extreme_frequency_ratios_remain_representable() {
        // k/m underflows in one direction and overflows in the other. A
        // quarter-period release has a closed-form endpoint in both cases.
        for (mass, k, x) in [(1e160_f64, 1e-200_f64, 1e-40_f64), (1e-160, 1e200, 1e-40)] {
            let w = k.sqrt() / mass.sqrt();
            let d = propagate(start(x, 0.), std::f64::consts::FRAC_PI_2 / w, mass, k, 0.).unwrap();
            assert!(d.x.abs() < 1e-53);
            assert!((d.u / (-w * x) - 1.).abs() < 1e-13);
            assert!((d.impulse / (mass * w * x) - 1.).abs() < 1e-13);
        }
    }

    #[test]
    fn airborne_large_foot_offset_preserves_small_center_position() {
        let body = SoleBody {
            mass: 1.,
            inertia: DVec3::ONE,
            half_extents: DVec3::splat(0.05),
            foot_offset: DVec3::Z * 1e16,
        };
        let deck = SoleDeck {
            center: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            half_extents: DVec3::ONE,
            velocity: DVec3::ZERO,
        };
        let mat = SoleMaterial::new(1e3, 1e3, 0., 0., 0.01).unwrap();
        let mut s = SoleState {
            center: DVec3::Z,
            orientation: DQuat::IDENTITY,
            velocity: DVec3::Z * 0.2,
            spin: DVec3::ZERO,
        };
        let result = body
            .try_flat_elastic_step(
                &mut s,
                &deck,
                &mat,
                DVec3::ZERO,
                0.5,
                DVec3::ZERO,
                DVec3::ZERO,
            )
            .unwrap()
            .unwrap();
        assert_eq!(s.center.z, 1.1);
        assert_eq!(result.normal_impulse, 0.);
    }

    #[test]
    fn equilibrium_and_grazing_remain_supported() {
        for n in [1., 10., 1e4] {
            let h = n * TAU / 10.;
            let d = propagate(start(0.02, 0.), h, 1., 100., 2.).unwrap();
            assert_eq!(d.x, 0.02);
            assert_eq!(d.u, 0.);
            assert!((d.impulse - 2. * h).abs() < 1e-11);
            let d = propagate(start(0., 0.), h, 1., 100., 2.).unwrap();
            assert!(d.x.abs() < 1e-14);
            assert!(d.u.abs() < 1e-10);
            assert!((d.peak - 0.04).abs() < 1e-14);
        }
    }
    #[test]
    fn long_bounces_match_subdivided_ticks_and_linear_momentum() {
        let m = 1.3;
        let k = 1500.;
        let g = 9.81;
        for (x, u) in [
            (-0.1, -0.6),
            (-0.03, 0.4),
            (0., 0.7),
            (0.03, -0.8),
            (0.05, 0.1),
        ] {
            let start = start(x, u);
            let h = 21.753;
            let one = propagate(start, h, m, k, g).unwrap();
            let mut many = start;
            for _ in 0..2000 {
                many = propagate(many, h / 2000., m, k, g).unwrap()
            }
            assert!((one.x - many.x).abs() < 1e-10, "{} {}", one.x, many.x);
            assert!((one.u - many.u).abs() < 1e-9);
            assert!((one.peak - many.peak).abs() < 1e-10);
            assert!((one.impulse - many.impulse).abs() < 1e-7);
            assert!((one.impulse - m * (u + g * h - one.u)).abs() < 1e-10);
        }
    }
}
