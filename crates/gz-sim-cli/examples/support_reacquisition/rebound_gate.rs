// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Secondary returns of a nearly level, rectangular sole above a translating horizontal deck.
use super::*;
#[path = "rebound_event_audit.rs"]
mod event_audit;
pub(super) use event_audit::write_event_audit;

const RELEASE_M: f64 = 10e-6;
const PRIMARY_RELEASE_M: f64 = 1e-3;
const MIN_CLOSING_M_S: f64 = 0.04;
const GRAVITY_M_S2: f64 = 9.81;

// A rectangle clipped by four half-planes has at most eight distinct vertices.
struct Polygon {
    vertices: [DVec3; 8],
    len: usize,
}

impl Polygon {
    fn sole(center: DVec3, q: DQuat) -> Self {
        let [x, y, z] = FOOT_HALF;
        let mut vertices = [DVec3::ZERO; 8];
        for (point, (sx, sy)) in vertices
            .iter_mut()
            .zip([(-x, -y), (x, -y), (x, y), (-x, y)])
        {
            *point = center + q * DVec3::new(sx, sy, -z);
        }
        Self { vertices, len: 4 }
    }

    fn clip(&mut self, axis: usize, sign: f64, limit: f64) {
        if self.len == 0 {
            return;
        }
        let vertices = &self.vertices[..self.len];
        if vertices.iter().all(|p| sign * p[axis] <= limit) {
            return;
        }
        let mut out = [DVec3::ZERO; 8];
        let mut n = 0;
        let mut a = vertices[self.len - 1];
        let mut d0 = sign * a[axis] - limit;
        for &b in vertices {
            let d1 = sign * b[axis] - limit;
            // Strict crossings exclude vertices already exactly on the edge.
            if (d0 < 0. && d1 > 0.) || (d0 > 0. && d1 < 0.) {
                out[n] = a.lerp(b, d0 / (d0 - d1));
                n += 1;
            }
            if d1 <= 0. {
                out[n] = b;
                n += 1;
            }
            a = b;
            d0 = d1;
        }
        self.vertices = out;
        self.len = n;
    }

    fn over_deck(mut self) -> Self {
        for (axis, limit) in PAD_HALF[..2].iter().copied().enumerate() {
            self.clip(axis, 1., limit);
            self.clip(axis, -1., limit);
        }
        self
    }

    fn points(&self) -> &[DVec3] {
        &self.vertices[..self.len]
    }
}

fn relative_center(foot: BodyState, pad: BodyState) -> DVec3 {
    DVec3::from(foot.position) - DVec3::from(pad.position)
}

fn supported_polygon(foot: BodyState, pad: BodyState) -> Polygon {
    Polygon::sole(relative_center(foot, pad), DQuat::from_array(foot.orientation)).over_deck()
}

pub(super) fn supported_vertices(foot: BodyState, pad: BodyState) -> impl Iterator<Item = DVec3> {
    let polygon = supported_polygon(foot, pad);
    polygon.vertices.into_iter().take(polygon.len).map(move |p| p + DVec3::from(pad.position))
}

pub(super) fn supported_gap(foot: BodyState, pad: BodyState) -> Option<f64> {
    supported_polygon(foot, pad)
        .points()
        .iter()
        .map(|p| p.z - PAD_HALF[2])
        .reduce(f64::min)
}

fn projected_patch(
    foot: BodyState,
    pad: BodyState,
    v: DVec3,
    spin: DVec3,
    dt: f64,
) -> (DVec3, Polygon) {
    let center = relative_center(foot, pad) + v * dt - DVec3::Z * (0.5 * GRAVITY_M_S2 * dt * dt);
    let q = DQuat::from_scaled_axis(spin * dt) * DQuat::from_array(foot.orientation);
    (center, Polygon::sole(center, q).over_deck())
}

fn end_closing_speed(p: DVec3, center: DVec3, v: DVec3, spin: DVec3, dt: f64) -> f64 {
    -(v.z + spin.cross(p - center).z - GRAVITY_M_S2 * dt)
}

/// Predicted minimum supported gap and fastest approaching vertex at or below
/// the deck. None for a disjoint footprint or, for the speed, no penetration.
pub(super) fn projected_clearance_and_speed(
    foot: BodyState,
    pad: BodyState,
    dt: f64,
) -> Option<(f64, Option<f64>)> {
    let v = DVec3::from(foot.velocity) - DVec3::from(pad.velocity);
    let spin = DVec3::from(foot.angular_velocity);
    let (center, end) = projected_patch(foot, pad, v, spin, dt);
    let gap = end
        .points()
        .iter()
        .map(|p| p.z - PAD_HALF[2])
        .reduce(f64::min)?;
    let closing = end
        .points()
        .iter()
        .filter(|p| p.z <= PAD_HALF[2])
        .map(|p| end_closing_speed(*p, center, v, spin, dt))
        .reduce(f64::max);
    Some((gap, closing))
}

#[derive(Default)]
pub(super) struct ReboundGate {
    primary: Gate,
    weak_recontacts: bool,
    // A positive, committed reaction makes the next unloading eligible.
    was_loaded: bool,
    separated: bool,
    remaining: u64,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Admission {
    Ordinary,
    Primary,
    SecondaryStart,
    SecondaryFollow,
    WeakStart,
}
impl ReboundGate {
    pub(super) fn primary_started(&self) -> bool {
        self.primary.triggered_on_current_boundary()
    }
    pub(super) fn with_weak_recontacts(enabled: bool) -> Self {
        Self {
            weak_recontacts: enabled,
            ..Self::default()
        }
    }
    /// Return (select this outer tick, first selected tick of a secondary return).
    #[cfg(test)]
    pub(super) fn advance(
        &mut self,
        foot: BodyState,
        pad: BodyState,
        previous_normal_ns: f64,
        ticks: u64,
        dt: f64,
    ) -> (bool, bool) {
        let decision = self.admit(foot, pad, previous_normal_ns, ticks, dt);
        (
            decision != Admission::Ordinary,
            decision == Admission::SecondaryStart,
        )
    }

    pub(super) fn admit(
        &mut self,
        foot: BodyState,
        pad: BodyState,
        previous_normal_ns: f64,
        ticks: u64,
        dt: f64,
    ) -> Admission {
        let primary = self.primary.advance(foot, pad, ticks, dt);
        if previous_normal_ns > 0. {
            self.was_loaded = true;
            self.separated = false;
        }
        if self.remaining > 0 {
            self.remaining -= 1;
            return if primary {
                Admission::Primary
            } else {
                Admission::SecondaryFollow
            };
        }
        if primary || previous_normal_ns > 0. || !self.was_loaded {
            return if primary {
                Admission::Primary
            } else {
                Admission::Ordinary
            };
        }
        let Some(gap) = supported_gap(foot, pad) else {
            return Admission::Ordinary;
        };
        // Defer full flights to the original primary gate. An outboard low
        // corner can lie below the lowest supported point on a partial deck.
        if sole_gap(foot.into(), pad.into()) > PRIMARY_RELEASE_M {
            self.was_loaded = false;
            self.separated = false;
            return Admission::Ordinary;
        }
        self.separated |= gap > RELEASE_M;
        if !self.separated {
            return Admission::Ordinary;
        }
        let v = DVec3::from(foot.velocity) - DVec3::from(pad.velocity);
        let spin = DVec3::from(foot.angular_velocity);
        let closing_bound =
            -v.z + spin.length() * DVec3::from(FOOT_HALF).length() + GRAVITY_M_S2 * dt;
        if closing_bound < MIN_CLOSING_M_S && (!self.weak_recontacts || closing_bound <= 0.) {
            return Admission::Ordinary;
        }
        // Extrapolate the sole and clip at the *predicted* footprint. A low
        // overhanging corner alone cannot establish a deck-top impact.
        let (center, end) = projected_patch(foot, pad, v, spin, dt);
        let mut admission = Admission::Ordinary;
        for &point in end.points().iter().filter(|p| p.z <= PAD_HALF[2]) {
            let speed = end_closing_speed(point, center, v, spin, dt);
            if speed >= MIN_CLOSING_M_S {
                self.remaining = ticks - 1;
                admission = Admission::SecondaryStart;
                break;
            }
            if self.weak_recontacts && speed > 0. {
                admission = Admission::WeakStart;
            }
        }
        if admission != Admission::Ordinary {
            self.was_loaded = false;
            self.separated = false;
        }
        admission
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.0005;

    fn body(x: f64, z: f64, vz: f64) -> BodyState {
        BodyState {
            position: [x, 0., z],
            velocity: [0., 0., vz],
            orientation: [0., 0., 0., 1.],
            angular_velocity: [0.; 3],
        }
    }

    fn deck() -> BodyState {
        body(0., -PAD_HALF[2], 0.)
    }

    #[test]
    fn a_return_needs_load_release_and_closing_speed_and_spends_only_two_ticks() {
        let mut g = ReboundGate::default();
        let step = |g: &mut ReboundGate, gap, vz, j| {
            g.advance(body(0., FOOT_HALF[2] + gap, vz), deck(), j, 2, DT)
        };
        let idle = (false, false);
        assert_eq!(step(&mut g, 30e-6, -0.2, 0.), idle); // no preceding load
        assert_eq!(step(&mut g, 0., 0., 0.04), idle);
        assert_eq!(step(&mut g, 4e-6, -0.2, 0.), idle); // chatter
        assert_eq!(step(&mut g, 120e-6, 0.1, 0.), idle); // real release, rising
        assert_eq!(step(&mut g, 2e-6, -0.015, 0.), idle); // slow grazing
        assert_eq!(step(&mut g, 50e-6, -0.2, 0.), (true, true));
        assert_eq!(step(&mut g, 0., 0., 0.2), (true, false));
        assert_eq!(step(&mut g, 0., 0., 0.01), idle);
        assert_eq!(step(&mut g, 0., -0.2, 0.), idle);
        assert_eq!(step(&mut g, 30e-6, -0.2, 0.), (true, true));
    }

    #[test]
    fn weak_crossing_spends_one_release_and_parked_chatter_cannot_rearm_it() {
        use Admission::{Ordinary, SecondaryFollow, SecondaryStart, WeakStart};
        let mut g = ReboundGate::with_weak_recontacts(true);
        let mut step = |gap, vz, j| g.admit(body(0., FOOT_HALF[2] + gap, vz), deck(), j, 2, DT);
        assert_eq!(step(30e-6, -0.01, 0.), Ordinary);
        assert_eq!(step(0., 0., 0.02), Ordinary);
        assert_eq!(step(4e-6, -0.01, 0.), Ordinary);
        assert_eq!(step(30e-6, 0.1, 0.), Ordinary);
        assert_eq!(step(2e-6, -0.01, 0.), WeakStart);
        assert_eq!(step(1e-6, -0.01, 0.), Ordinary);
        assert_eq!(step(30e-6, 0.1, 0.), Ordinary);
        assert_eq!(step(2e-6, -0.01, 0.), Ordinary);
        assert_eq!(step(0., 0., 0.02), Ordinary);
        assert_eq!(step(1e-6, 0., 0.), Ordinary);
        assert_eq!(step(30e-6, 0.1, 0.), Ordinary);
        assert_eq!(step(2e-6, -0.01, 0.), WeakStart);
        assert_eq!(step(0., 0., 0.02), Ordinary);
        assert_eq!(step(30e-6, -0.2, 0.), SecondaryStart);
        assert_eq!(step(1e-6, -0.01, 0.), SecondaryFollow);
    }

    #[test]
    fn full_flight_keeps_the_original_primary_window() {
        let mut old = Gate::default();
        let mut new = ReboundGate::default();
        for (gap, vz, previous) in [
            (0., 0., 0.1),
            (2e-3, 0.5, 0.),
            (3e-3, -2., 0.),
            (0.8e-3, -2., 0.),
            (0., 0., 2.),
            (0., 0., 0.01),
        ] {
            let f = body(0., FOOT_HALF[2] + gap, vz);
            assert_eq!(
                new.advance(f, deck(), previous, 2, DT),
                (old.advance(f, deck(), 2, DT), false)
            );
        }
    }

    #[test]
    fn an_overhanging_low_corner_cannot_trigger_a_secondary_until_the_deck_edge_is_reached() {
        let theta = 0.1_f64;
        let x = PAD_HALF[0] + 0.15;
        // At the right-hand deck edge the tilted underside plane is at 'gap'.
        let pose = |gap, vz| {
            let z = FOOT_HALF[2] / theta.cos() + (PAD_HALF[0] - x) * theta.tan() + gap;
            let mut f = body(x, z, vz);
            f.orientation = DQuat::from_rotation_y(theta).to_array();
            f
        };
        let mut g = ReboundGate::default();
        let f = pose(300e-6, -0.1);
        assert!(sole_gap(f.into(), deck().into()) < -0.01);
        assert!((supported_gap(f, deck()).unwrap() - 300e-6).abs() < 1e-14);
        assert_eq!(g.advance(pose(0., 0.), deck(), 0.04, 2, DT), (false, false));
        assert_eq!(g.advance(f, deck(), 0., 2, DT), (false, false));
        assert_eq!(
            g.advance(pose(25e-6, -0.1), deck(), 0., 2, DT),
            (true, true)
        );
        assert!(supported_gap(body(2., 0.06, 0.), deck()).is_none());
    }

    #[test]
    fn a_rocking_corner_can_return_while_the_sole_center_is_moving_away() {
        let mut g = ReboundGate::default();
        let _ = g.advance(body(0., 0.06, 0.), deck(), 0.01, 2, DT);
        let mut f = body(0., 0.06005, 0.1);
        f.angular_velocity = [0., 2., 0.];
        assert_eq!(g.advance(f, deck(), 0., 2, DT), (true, true));
    }
}
