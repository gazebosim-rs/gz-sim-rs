// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Vertical clearance of an oriented box over a finite horizontal support.
use glam::{DQuat, DVec3};
use gz_physics::BodyState;

const RELEASE_M: f64 = 0.001;
#[derive(Clone, Copy)]
pub(super) struct BoxShape {
    pub half: DVec3,
    /// Translation from the link/box origin to the COM in link coordinates.
    pub local_com: DVec3,
}

struct Polygon {
    points: [DVec3; 8],
    len: usize,
}
impl Polygon {
    fn face(vertices: &[DVec3; 8], ids: [usize; 4]) -> Self {
        let mut points = [DVec3::ZERO; 8];
        for (p, id) in points.iter_mut().zip(ids) {
            *p = vertices[id];
        }
        Self { points, len: 4 }
    }
    fn clip(&mut self, axis: usize, sign: f64, limit: f64) {
        if self.len == 0 {
            return;
        }
        let mut clipped = [DVec3::ZERO; 8];
        let mut n = 0;
        let mut a = self.points[self.len - 1];
        let mut da = sign * a[axis] - limit;
        for b in self.points[..self.len].iter().copied() {
            let db = sign * b[axis] - limit;
            if (da < 0. && db > 0.) || (da > 0. && db < 0.) {
                clipped[n] = a.lerp(b, da / (da - db));
                n += 1;
            }
            if db <= 0. {
                clipped[n] = b;
                n += 1;
            }
            a = b;
            da = db;
        }
        self.points = clipped;
        self.len = n;
    }
}

#[derive(Clone, Copy)]
struct BoxAt {
    /// COM and twist relative to the support center, in support coordinates.
    com: DVec3,
    rotation: DQuat,
    velocity: DVec3,
    spin: DVec3,
    shape: BoxShape,
}
impl BoxAt {
    fn at(self, gravity: DVec3, dt: f64) -> Self {
        Self {
            com: self.com + self.velocity * dt + gravity * (0.5 * dt * dt),
            rotation: DQuat::from_scaled_axis(self.spin * dt) * self.rotation,
            velocity: self.velocity + gravity * dt,
            ..self
        }
    }
    fn over_support(self, pad: DVec3) -> Option<(f64, bool)> {
        // A rotation cannot put a corner farther from the link origin than
        // the sum of its half extents. Far lateral pairs can skip clipping.
        let origin = self.com - self.rotation * self.shape.local_com;
        let radius = self.shape.half.element_sum();
        if origin.x.abs() > pad.x + radius || origin.y.abs() > pad.y + radius {
            return None;
        }
        // Enumerate the projected lower hull. This also handles a sole that
        // overhangs the deck: a low but outboard corner cannot trigger it.
        let mut vertices = [DVec3::ZERO; 8];
        for (i, p) in vertices.iter_mut().enumerate() {
            let coordinate = |axis: usize| {
                if i & (1 << axis) == 0 {
                    -self.shape.half[axis]
                } else {
                    self.shape.half[axis]
                }
            };
            let local =
                DVec3::new(coordinate(0), coordinate(1), coordinate(2)) - self.shape.local_com;
            *p = self.com + self.rotation * local;
        }
        let upward = self.rotation.conjugate() * DVec3::Z;
        let mut gap = f64::INFINITY;
        let mut closing = false;
        for axis in 0..3 {
            if upward[axis] == 0. {
                continue;
            }
            let side = usize::from(upward[axis] < 0.) << axis;
            let a = 1 << ((axis + 1) % 3);
            let b = 1 << ((axis + 2) % 3);
            let mut face = Polygon::face(&vertices, [side, side | a, side | a | b, side | b]);
            for axis in 0..2 {
                face.clip(axis, 1., pad[axis]);
                face.clip(axis, -1., pad[axis]);
            }
            for p in &face.points[..face.len] {
                let z = p.z - pad.z;
                gap = gap.min(z);
                closing |= z <= 0. && (self.velocity + self.spin.cross(*p - self.com)).z < 0.;
            }
        }
        gap.is_finite().then_some((gap, closing))
    }
}
pub(super) fn horizontal(pad: BodyState) -> bool {
    let normal = DQuat::from_array(pad.orientation) * DVec3::Z;
    // Yaw is allowed. Tiny quaternion roundoff is immaterial to the face normal.
    normal.z > 0. && normal.truncate().length_squared() <= 1e-20 && pad.angular_velocity == [0.; 3]
}

pub(super) fn predict(
    armed: &mut bool,
    foot: (BodyState, BoxShape),
    support: (BodyState, BoxShape),
    gravity: DVec3,
    dt: f64,
) -> bool {
    predict_oriented(armed, foot, support, gravity, dt, false)
}

/// The same lower-hull and footprint computation applies in the frame of a
/// translating inclined support. Rotation during the prediction is excluded.
pub(super) fn predict_oriented(
    armed: &mut bool,
    foot: (BodyState, BoxShape),
    support: (BodyState, BoxShape),
    gravity: DVec3,
    dt: f64,
    allow_inclined: bool,
) -> bool {
    let (s, pad) = support;
    let (f, shape) = foot;
    if s.angular_velocity != [0.; 3] || !allow_inclined && !horizontal(s) {
        *armed = false;
        return false;
    }
    let q = DQuat::from_array(s.orientation);
    let inv = q.conjugate();
    // BodyState reports the link origin and the COM velocity. Integrate the COM,
    // then rotate each link-local vertex about it. The pad has no angular motion.
    let com = inv
        * (DVec3::from(f.position) - DVec3::from(s.position)
            + DQuat::from_array(f.orientation) * shape.local_com);
    let body = BoxAt {
        com,
        shape,
        rotation: inv * DQuat::from_array(f.orientation),
        velocity: inv * (DVec3::from(f.velocity) - DVec3::from(s.velocity)),
        spin: inv * DVec3::from(f.angular_velocity),
    };
    let Some((z, _)) = body.over_support(pad.half) else {
        *armed = false;
        return false;
    };
    *armed |= z > RELEASE_M;
    if !*armed {
        return false;
    }
    // Reject arrivals from the side or below the slab after a previously
    // separated footprint has missed it. A measured positive load also disarms.
    if z < -2. * pad.half.z {
        *armed = false;
        return false;
    }
    if body
        .at(inv * gravity, dt)
        .over_support(pad.half)
        .is_some_and(|(_, closing)| closing)
    {
        *armed = false;
        true
    } else {
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum FaceArrival {
    TouchingParallelFaces,
    SeparatedParallelFaces(std::time::Duration),
    Predicted,
}

/// Detect closure across the finite authored face from the current rigid twists.
/// For fixed orientations also test where the incident box first reaches
/// the face plane. A fast transverse motion can miss both endpoint footprints.
/// Intermediate rotational contacts and unmodeled accelerations remain outside this test.
pub(super) fn incoming_face(
    latched: &mut bool,
    face: DVec3,
    foot: (BodyState, BoxShape),
    support: (BodyState, BoxShape),
    dt: f64,
) -> Option<FaceArrival> {
    let face_rotation = DQuat::from_rotation_arc(DVec3::Z, face);
    let half = (face_rotation.conjugate() * support.1.half).abs();
    let tolerance = 64.
        * f64::EPSILON
        * (1. + DVec3::from(foot.0.position).length() + DVec3::from(support.0.position).length());
    let at = |(state, shape): (BodyState, BoxShape), t| {
        let q0 = DQuat::from_array(state.orientation);
        let w = DVec3::from(state.angular_velocity);
        let v = DVec3::from(state.velocity);
        let q = DQuat::from_scaled_axis(w * t) * q0;
        let com = DVec3::from(state.position) + q0 * shape.local_com + v * t;
        (com, q, v, w)
    };
    let body_at = |t| {
        let (a, qa, va, wa) = at(foot, t);
        let (b, qb, vb, wb) = at(support, t);
        let inv = (qb * face_rotation).conjugate();
        BoxAt {
            com: inv * (a - b + qb * support.1.local_com),
            rotation: inv * qa,
            velocity: inv * (va - vb - wb.cross(a - b)),
            spin: inv * (wa - wb),
            shape: foot.1,
        }
    };
    let start = body_at(0.);
    let gap = start.over_support(half).map(|(gap, _)| gap);
    if gap.is_none_or(|z| z > tolerance) {
        *latched = false;
    }
    if *latched || gap.is_some_and(|z| z < -tolerance) {
        return None;
    }
    let hit = gap.is_some_and(|z| {
        body_at(dt)
            .over_support(half)
            .is_some_and(|(end, closing)| closing && end < z)
    }) || (foot.0.angular_velocity == [0.; 3]
        && support.0.angular_velocity == [0.; 3]
        && translating_face_entry(start, half, dt, tolerance));
    *latched = hit;
    hit.then(|| {
        let normal_in_box = start.rotation.conjugate() * DVec3::Z;
        let origin = start.com - start.rotation * start.shape.local_com;
        if let Some(gap) = gap
            && foot.0.angular_velocity == [0.; 3]
            && support.0.angular_velocity == [0.; 3]
            && start.velocity.z < 0.
            && start.velocity.truncate().length() * dt <= tolerance
            && origin.x.abs() + tolerance < half.x
            && origin.y.abs() + tolerance < half.y
            && DVec3::AXES
                .iter()
                .any(|axis| normal_in_box.cross(*axis).length() <= 64. * f64::EPSILON)
        {
            if gap <= tolerance {
                FaceArrival::TouchingParallelFaces
            } else {
                // The finite face and overlap tests above exclude a crossing of the infinite plane alone.
                FaceArrival::SeparatedParallelFaces(std::time::Duration::from_secs_f64(
                    (gap / -start.velocity.z).min(dt),
                ))
            }
        } else {
            FaceArrival::Predicted
        }
    })
}

/// Project the box half extents onto the receiver normal. Until its lowest
/// point reaches the plane, the two solid boxes are separated along this axis.
fn translating_face_entry(body: BoxAt, pad: DVec3, dt: f64, tolerance: f64) -> bool {
    if body.velocity.z >= 0. {
        return false;
    }
    let normal_in_box = body.rotation.conjugate() * DVec3::Z;
    let bottom = body.com.z
        - normal_in_box.dot(body.shape.local_com)
        - normal_in_box.abs().dot(body.shape.half);
    let arrival = (bottom - pad.z) / -body.velocity.z;
    (0. ..=dt).contains(&arrival)
        && body
            .at(DVec3::ZERO, arrival)
            .over_support(pad)
            .is_some_and(|(gap, _)| gap.abs() <= tolerance)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn state(position: DVec3, rotation: DQuat, velocity: DVec3, spin: DVec3) -> BodyState {
        BodyState {
            position: position.to_array(),
            orientation: rotation.to_array(),
            velocity: velocity.to_array(),
            angular_velocity: spin.to_array(),
        }
    }
    const DT: f64 = 0.0005;

    #[test]
    fn cold_strike_uses_relative_motion_inside_each_moving_finite_face() {
        let q = DQuat::from_rotation_z(0.4);
        let w = DVec3::new(0., 0., 0.3);
        let center = DVec3::new(0.4, 0.3, -0.15);
        let v = DVec3::new(0.2, 0.1, -0.3);
        let shape = BoxShape {
            half: DVec3::splat(0.025),
            local_com: DVec3::new(0., 0., 0.006),
        };
        let pad = (state(center, q, v, w), shape);
        for face in [
            DVec3::X,
            DVec3::NEG_X,
            DVec3::Y,
            DVec3::NEG_Y,
            DVec3::Z,
            DVec3::NEG_Z,
        ] {
            let outward = q * face;
            let foot = |gap, speed, lateral| {
                let offset = outward * (0.05 + gap) + lateral;
                (
                    state(center + offset, q, v + w.cross(offset) - outward * speed, w),
                    shape,
                )
            };
            let mut latch = false;
            assert!(incoming_face(&mut latch, face, foot(0., 1., DVec3::ZERO), pad, DT).is_some());
            assert!(incoming_face(&mut latch, face, foot(0., 1., DVec3::ZERO), pad, DT).is_none());
            assert!(
                incoming_face(&mut latch, face, foot(0.002, 1., DVec3::ZERO), pad, DT).is_none()
            );
            assert!(
                incoming_face(&mut latch, face, foot(0.0001, 1., DVec3::ZERO), pad, DT).is_some()
            );
            assert!(
                incoming_face(
                    &mut latch,
                    face,
                    foot(0., 1., q * face.any_orthonormal_vector() * 0.3),
                    pad,
                    DT
                )
                .is_none()
            );
            assert!(incoming_face(&mut latch, face, foot(0., -1., DVec3::ZERO), pad, DT).is_none());
            assert!(
                incoming_face(&mut latch, face, foot(-0.08, 1., DVec3::ZERO), pad, DT).is_none()
            );
        }
    }

    #[test]
    fn swept_face_checks_the_lowest_feature_at_its_plane_crossing() {
        let half = DVec3::new(0.006, 0.010, 0.005);
        let turn = DQuat::from_rotation_y(0.31);
        let shape = BoxShape {
            half: DVec3::splat(0.004),
            local_com: DVec3::new(-0.001, 0.002, 0.003),
        };
        let relative_point = turn * shape.local_com;
        let lowest = (turn.conjugate() * DVec3::Z).abs().dot(shape.half);
        let body = BoxAt {
            com: DVec3::new(-0.020, 0., half.z + lowest + 0.04) + relative_point,
            rotation: turn,
            velocity: DVec3::new(2., 0., -4.),
            spin: DVec3::ZERO,
            shape,
        };
        assert!(body.over_support(half).is_none());
        assert!(body.at(DVec3::ZERO, 0.020).over_support(half).is_none());
        assert!(translating_face_entry(body, half, 0.020, 1e-13));
        // The plane is reached before the projected footprint enters the pad.
        let beyond = BoxAt {
            com: body.com - DVec3::X * 0.020,
            ..body
        };
        assert!(!translating_face_entry(beyond, half, 0.020, 1e-13));
        // Only the upper corner overlaps the footprint at the instant of crossing.
        let outboard = BoxAt {
            com: body.com + DVec3::X * 0.008,
            ..body
        };
        assert!(!translating_face_entry(outboard, half, 0.020, 1e-13));
        for dt in [0., 0.005] {
            assert!(!translating_face_entry(body, half, dt, 1e-13));
        }
        for speed in [0., 1.] {
            assert!(!translating_face_entry(
                BoxAt {
                    velocity: DVec3::Z * speed,
                    ..body
                },
                half,
                0.020,
                1e-13
            ));
        }
    }

    #[test]
    fn low_outboard_corner_does_not_land_over_empty_space() {
        let foot = BoxShape {
            half: DVec3::new(0.18, 0.1, 0.06),
            local_com: DVec3::ZERO,
        };
        let pad = BoxShape {
            half: DVec3::new(0.04, 0.08, 0.05),
            local_com: DVec3::ZERO,
        };
        let pose = state(
            DVec3::new(0.12, 0., 0.19),
            DQuat::from_rotation_y(0.35),
            DVec3::Z * (-100.),
            DVec3::ZERO,
        );
        let support = state(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO);
        let mut armed = false;
        assert!(!predict(
            &mut armed,
            (pose, foot),
            (support, pad),
            DVec3::ZERO,
            DT
        ));
        assert!(armed);
        let pose = BodyState {
            position: [0.12, 0., 0.14],
            velocity: [0., 0., -180.],
            ..pose
        };
        assert!(predict(
            &mut armed,
            (pose, foot),
            (support, pad),
            DVec3::ZERO,
            DT
        ));
        assert!(!armed);
    }
    #[test]
    fn yawed_frame_and_offset_com_preserve_centered_translation() {
        let pad = BoxShape {
            half: DVec3::new(0.20, 0.05, 0.04),
            local_com: DVec3::new(0.015, -0.008, 0.009),
        };
        let foot = BoxShape {
            half: DVec3::new(0.1, 0.03, 0.03),
            local_com: DVec3::new(-0.05, 0.02, 0.02),
        };
        let rot = DQuat::from_rotation_z(1.3);
        let origin = DVec3::new(13., -4., 7.);
        let support = state(origin, rot, DVec3::Z * 0.3, DVec3::ZERO);
        let pose = state(
            origin + rot * DVec3::new(0.02, 0.01, 0.072),
            rot,
            DVec3::Z * (-4.),
            DVec3::ZERO,
        );
        let mut armed = false;
        assert!(predict(
            &mut armed,
            (pose, foot),
            (support, pad),
            DVec3::ZERO,
            DT
        ));
        let rising = BodyState {
            velocity: [0., 0., 1.],
            ..pose
        };
        assert!(!predict(
            &mut armed,
            (rising, foot),
            (support, pad),
            DVec3::ZERO,
            DT
        ));
        assert!(armed);
        let side = BodyState {
            position: (origin + rot * DVec3::new(0., 0.5, 0.072)).into(),
            ..pose
        };
        assert!(!predict(
            &mut armed,
            (side, foot),
            (support, pad),
            DVec3::ZERO,
            DT
        ));
        assert!(!armed);
    }
}
