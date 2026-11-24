// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Returning corners on a finite, already load-bearing pad face.
use super::{BodyId, BodyState, BoxShape, Duration};
use glam::{DQuat, DVec3};
use gz_physics_multifidelity::{ContactSubstep, RigidContactPatch};

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum PadMotion {
    Ordinary,
    Rocking,
    Impact,
}

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
        track_rocking: bool,
    ) -> PadMotion {
        let follow = std::mem::take(&mut self.follow);
        if !loaded {
            self.released = 0;
            return if follow {
                PadMotion::Impact
            } else {
                PadMotion::Ordinary
            };
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
        // A spinning face can have a loaded edge and a lifted edge long before
        // a return is imminent. Track only corners inside the finite footprint.
        let turning = track_rocking
            && (foot_spin - pad_spin).cross(normal).length() * dt > 32. * f64::EPSILON;
        let mut footprint = 0_u8;
        let mut lifting = 0_u8;
        let mut imminent = false;
        for i in 0_u8..8 {
            let sign = |bit| if i & (1_u8 << bit) == 0_u8 { -1.0 } else { 1.0 };
            let corner = shape.half * DVec3::new(sign(0), sign(1), sign(2));
            let vertex = relative + q * corner;
            let local = pq.conjugate() * vertex;
            if turning && (local - face * local.dot(face)).abs().cmple(pad.half).all() {
                footprint |= 1 << i;
            }
            let gap = local.dot(face) - pad.half.dot(face.abs());
            let arm = q * (corner - shape.local_com);
            let slip = relative_velocity + foot_spin.cross(arm)
                - pad_spin.cross(vertex - pq * pad.local_com);
            let closing = -slip.dot(normal);
            // A corner can still be compressed at this boundary yet lift
            // during the next tick. Predict its first release with the same
            // point velocity as the returning-corner test.
            if turning && gap - closing * dt > tolerance {
                lifting |= 1 << i;
            }
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
        if imminent || follow {
            return PadMotion::Impact;
        }
        if footprint != 0 {
            // Compare the four sites of the material face most directly opposing
            // the pad. Corners on the back of a seated box are always separated.
            let facing = q.conjugate() * -normal;
            let axis = facing.abs().max_position();
            let positive = [0xaa_u8, 0xcc, 0xf0][axis];
            let sites = footprint
                & if facing[axis] > 0. {
                    positive
                } else {
                    !positive
                };
            let split = |separated: u8| sites & separated != 0 && sites & !separated != 0;
            if split(released) || split(lifting) {
                return PadMotion::Rocking;
            }
        }
        PadMotion::Ordinary
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
            history.select(loaded, DVec3::X, (foot, shape), (pad, shape), 0.1, false)
                == PadMotion::Impact
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
    fn rocking_needs_a_turning_material_face_with_both_edges_over_the_pad() {
        let shape = BoxShape {
            half: DVec3::splat(0.5),
            local_com: DVec3::ZERO,
        };
        let pad = BodyState {
            position: [0.; 3],
            velocity: [0.; 3],
            orientation: DQuat::IDENTITY.to_array(),
            angular_velocity: [0.; 3],
        };
        for face in [
            DVec3::X,
            DVec3::NEG_X,
            DVec3::Y,
            DVec3::NEG_Y,
            DVec3::Z,
            DVec3::NEG_Z,
        ] {
            let hinge = face.cross(DVec3::new(1., 1., 1.)).normalize();
            let tilt = DQuat::from_axis_angle(hinge, 0.15);
            let foot = BodyState {
                position: (0.99 * face).to_array(),
                orientation: tilt.to_array(),
                angular_velocity: hinge.to_array(),
                ..pad
            };
            let classify = |foot, support, half| {
                CornerHistory::default().select(
                    true,
                    face,
                    (foot, shape),
                    (support, BoxShape { half, ..shape }),
                    0.0002,
                    true,
                )
            };
            // The tilt lifts corners of the facing parcel face on one side only.
            let wide = DVec3::splat(2.) - 1.5 * face.abs();
            assert!(classify(foot, pad, wide) == PadMotion::Rocking);
            // Rotating both together, or spinning about the pad normal, cannot rock the face.
            assert!(
                classify(
                    foot,
                    BodyState {
                        angular_velocity: foot.angular_velocity,
                        ..pad
                    },
                    wide
                ) == PadMotion::Ordinary
            );
            assert!(
                classify(
                    BodyState {
                        angular_velocity: face.to_array(),
                        ..foot
                    },
                    pad,
                    wide
                ) == PadMotion::Ordinary
            );
            // A narrow support between the parcel's edges has neither site in its footprint.
            let narrow = DVec3::splat(0.01) + 0.49 * face.abs();
            assert!(classify(foot, pad, narrow) == PadMotion::Ordinary);
        }
    }

    #[test]
    fn a_compressed_edge_can_lift_within_the_next_tick() {
        let pad = BodyState {
            position: [0.; 3],
            velocity: [0.; 3],
            orientation: DQuat::IDENTITY.to_array(),
            angular_velocity: [0.; 3],
        };
        let shape = BoxShape {
            half: DVec3::splat(0.5),
            local_com: DVec3::ZERO,
        };
        let classify = |omega, vx| {
            let foot = BodyState {
                position: [0.999, 0., 0.],
                angular_velocity: [0., omega, 0.],
                velocity: [vx, 0., 0.],
                ..pad
            };
            CornerHistory::default().select(
                true,
                DVec3::X,
                (foot, shape),
                (pad, shape),
                0.001,
                true,
            )
        };
        assert!(classify(1., 0.) == PadMotion::Ordinary);
        assert!(classify(3., 0.) == PadMotion::Rocking);
        assert!(classify(-3., 0.) == PadMotion::Rocking);
        assert!(classify(3., 4.) == PadMotion::Ordinary); // both edges exit together
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
