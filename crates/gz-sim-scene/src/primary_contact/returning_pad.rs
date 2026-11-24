// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Returning corners on a finite, already load-bearing pad face.
use super::{BodyId, BodyState, BoxShape, Duration};
use glam::{DQuat, DVec3};
use gz_physics_multifidelity::{ContactSubstep, RigidContactPatch};

#[derive(Clone, Copy, Default)]
pub(super) struct CornerHistory {
    released: u8,
    follow: bool,
}

impl CornerHistory {
    pub(super) fn select(
        &mut self,
        loaded: bool,
        face: DVec3,
        (foot, shape): (BodyState, BoxShape),
        (support, pad): (BodyState, BoxShape),
        dt: f64,
    ) -> bool {
        let follow = std::mem::take(&mut self.follow);
        if !loaded {
            self.released = 0;
            return follow;
        }
        let q = DQuat::from_array(foot.orientation);
        let pq = DQuat::from_array(support.orientation);
        let normal = pq * face;
        let relative = DVec3::from(foot.position) - DVec3::from(support.position);
        let relative_velocity = DVec3::from(foot.velocity) - DVec3::from(support.velocity);
        let foot_spin = DVec3::from(foot.angular_velocity);
        let pad_spin = DVec3::from(support.angular_velocity);
        // Roundoff on an exactly coplanar resting face is not a measured release.
        let tolerance = 32. * f64::EPSILON * (relative.abs() + shape.half + pad.half).max_element();
        let mut released = 0;
        let mut imminent = false;
        for i in 0_u8..8 {
            let sign = |bit| if i & (1_u8 << bit) == 0_u8 { -1.0 } else { 1.0 };
            let corner = shape.half * DVec3::new(sign(0), sign(1), sign(2));
            let vertex = relative + q * corner;
            let local = pq.conjugate() * vertex;
            let gap = local.dot(face) - pad.half.dot(face.abs());
            let arm = q * (corner - shape.local_com);
            let slip = relative_velocity + foot_spin.cross(arm)
                - pad_spin.cross(vertex - pq * pad.local_com);
            let closing = -slip.dot(normal);
            if gap > tolerance {
                released |= 1 << i;
            }
            if self.released & (1 << i) != 0 && closing > 0.0 && gap <= closing * dt {
                // Test the finite footprint at the prospective plane crossing.
                let at_face = local + pq.conjugate() * slip * (gap / closing).clamp(0., dt);
                let tangent = at_face - face * at_face.dot(face);
                imminent |= tangent.abs().cmple(pad.half).all();
            }
        }
        // A strike right at the end of a public tick loads the new edge in
        // the next tick. Keep one follow-through interval, then release.
        self.released = released;
        self.follow = imminent;
        imminent || follow
    }
}

/// Measured load on an enrolled foot during a selected public interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PadImpact {
    pub foot: BodyId,
    pub support: BodyId,
    /// Largest normal impulse per native-step duration, summed over the pair's contact points.
    pub peak_normal_force_n: f64,
    /// Native-step end relative to the start of the public interval. Zero for an unloaded pair.
    pub peak_end: Duration,
    /// Integrated applied contact moment on the foot about its responding body's COM, in world N·m·s.
    /// Absent if any native point lacks a resolved tangential impulse.
    pub angular_impulse_on_foot: Option<[f64; 3]>,
}

impl PadImpact {
    pub(super) fn new(foot: BodyId, support: BodyId) -> Self {
        Self {
            foot,
            support,
            peak_normal_force_n: 0.,
            peak_end: Duration::ZERO,
            angular_impulse_on_foot: Some([0.; 3]),
        }
    }

    fn measure(&mut self, patches: &[RigidContactPatch], dt: Duration, end: Duration) {
        let mut normal = 0.;
        let mut angular = self.angular_impulse_on_foot.map(DVec3::from);
        for patch in patches {
            let foot_a = patch.a == self.foot && patch.b == self.support;
            if !foot_a && !(patch.b == self.foot && patch.a == self.support) {
                continue;
            }
            let n = DVec3::from(patch.normal_on_a);
            for point in &patch.points {
                normal += point.normal_impulse;
                angular = angular
                    .zip(point.tangent_impulse_on_a)
                    .map(|(total, tangent)| {
                        let impulse = n * point.normal_impulse + DVec3::from(tangent);
                        total
                            + if foot_a {
                                DVec3::from(point.lever_a).cross(impulse)
                            } else {
                                DVec3::from(point.lever_b).cross(-impulse)
                            }
                    });
            }
        }
        let force = normal / dt.as_secs_f64();
        if force > self.peak_normal_force_n {
            self.peak_normal_force_n = force;
            self.peak_end = end;
        }
        self.angular_impulse_on_foot = angular.map(|value| value.to_array());
    }

    pub(super) fn sample_all(rows: &mut [Self], step: &ContactSubstep<'_>) {
        if !rows.is_empty() {
            let patches = step.contact_patches();
            for row in rows {
                row.measure(&patches, step.duration, step.elapsed);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gz_physics_multifidelity::RigidPatchPoint;

    #[test]
    fn finite_moving_face_requires_a_release_and_one_follow_through_tick() {
        let pad = BodyState {
            position: [0.; 3],
            velocity: [4., 0., 0.],
            orientation: DQuat::IDENTITY.to_array(),
            angular_velocity: [0.; 3],
        };
        let shape = BoxShape {
            half: DVec3::splat(0.5),
            local_com: DVec3::ZERO,
        };
        let mut history = CornerHistory::default();
        let mut step = |loaded, x, y| {
            let foot = BodyState {
                position: [x, y, 0.],
                velocity: [3., 0., 0.],
                ..pad
            };
            history.select(loaded, DVec3::X, (foot, shape), (pad, shape), 0.1)
        };
        assert!(!step(true, 1.2, 2.));
        assert!(!step(true, 1.05, 2.)); // the infinite plane is hit outside the pad
        assert!(step(true, 1.05, 0.));
        assert!(step(false, 1., 0.)); // the final native sample had no load
        assert!(!step(false, 1., 0.));
        assert!(!step(true, 1. + f64::EPSILON, 0.));
        assert!(!step(true, 1., 0.)); // roundoff did not rearm the corner
    }

    #[test]
    fn applied_point_moment_uses_the_foot_side_and_native_duration() {
        let [a, b, c] = [0, 1, 2].map(BodyId::from_raw);
        let mut row = PadImpact::new(a, b);
        let patch = RigidContactPatch {
            a,
            b,
            normal_on_a: [1., 0., 0.],
            points: vec![RigidPatchPoint {
                lever_a: [0., 1., 0.],
                lever_b: [0., 0., 1.],
                normal_impulse: 2.,
                tangent_impulse_on_a: Some([0., 0., 3.]),
            }],
        };
        let dt = Duration::from_millis(2);
        let unrelated = RigidContactPatch {
            b: c,
            ..patch.clone()
        };
        row.measure(&[unrelated, patch.clone(), patch.clone()], dt, dt);
        row.measure(std::slice::from_ref(&patch), dt, dt * 2);
        assert_eq!(row.angular_impulse_on_foot, Some([9., 0., -6.]));
        assert_eq!((row.peak_normal_force_n, row.peak_end), (2000., dt));
        let mut reversed = PadImpact::new(b, a);
        reversed.measure(std::slice::from_ref(&patch), dt, dt);
        assert_eq!(reversed.angular_impulse_on_foot, Some([0., -2., 0.]));
        let mut unresolved = patch.clone();
        unresolved.points[0].tangent_impulse_on_a = None;
        reversed.measure(&[unresolved, patch], dt, dt * 2);
        assert_eq!(reversed.angular_impulse_on_foot, None);
        assert_eq!(
            (reversed.peak_normal_force_n, reversed.peak_end),
            (2000., dt * 2)
        );
    }
}
