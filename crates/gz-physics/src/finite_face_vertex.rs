// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A single moving vertex arriving at the strict interior of a static box face.
use crate::{BoxCrossing, FiniteBoxFlight, FlightBox, SeparatingFeature};
use glam::DVec3;

#[derive(Clone, Copy, Debug)]
pub struct FaceVertexCrossing {
    vertex: usize,
    axis: usize,
    positive: bool,
}
impl FaceVertexCrossing {
    fn point(self, body: FlightBox, i: usize) -> DVec3 {
        let signs = DVec3::from_array(std::array::from_fn(|k| {
            if i & (1 << k) == 0 { -1. } else { 1. }
        }));
        body.center + body.orientation * (signs * body.half_extent)
    }
    fn normal(self, pad: FlightBox) -> DVec3 {
        let axis = [DVec3::X, DVec3::Y, DVec3::Z][self.axis];
        pad.orientation * if self.positive { axis } else { -axis }
    }
    fn heights(self, body: FlightBox, pad: FlightBox) -> [f64; 8] {
        let n = self.normal(pad);
        std::array::from_fn(|i| {
            (self.point(body, i) - pad.center).dot(n) - pad.half_extent[self.axis]
        })
    }
    fn footprint(self, point: DVec3, pad: FlightBox) -> f64 {
        let local = (pad.orientation.conjugate() * (point - pad.center)).abs();
        (0..3)
            .filter(|k| *k != self.axis)
            .map(|k| pad.half_extent[k] - local[k])
            .fold(f64::INFINITY, f64::min)
    }
    fn precision(body: FlightBox, pad: FlightBox) -> f64 {
        256. * f64::EPSILON
            * (1.
                + body.center.abs().max_element()
                + pad.center.abs().max_element()
                + body.half_extent.length()
                + pad.half_extent.length())
    }
    /// Revalidate the same material vertex and every competing vertex on the committed native shapes.
    pub fn measure(
        self,
        body: FlightBox,
        pad: FlightBox,
        allowance: f64,
    ) -> Option<(SeparatingFeature, DVec3, [DVec3; 2], f64)> {
        if !(body.valid() && pad.valid() && allowance.is_finite() && allowance >= 0.) {
            return None;
        }
        let eps = Self::precision(body, pad);
        let heights = self.heights(body, pad);
        let gap = heights[self.vertex];
        if gap > eps
            || -gap > allowance + eps
            || heights
                .iter()
                .enumerate()
                .any(|(i, z)| i != self.vertex && *z <= eps)
        {
            return None;
        }
        let p = self.point(body, self.vertex);
        if self.footprint(p, pad) <= eps {
            return None;
        }
        let n = self.normal(pad);
        Some((
            SeparatingFeature::SupportFace(self.axis),
            n,
            [p, p - gap * n],
            gap,
        ))
    }
}
impl FiniteBoxFlight {
    /// Bound every other material vertex above this face throughout the final nanosecond.
    /// This certificate requires a strictly closing incident vertex on the finite face.
    pub fn isolated_face_vertex(self, crossing: BoxCrossing) -> Option<FaceVertexCrossing> {
        self.face_vertex_window(
            crossing.earliest.as_secs_f64(),
            crossing.latest.as_secs_f64(),
            crossing.feature,
            crossing.normal,
        )
    }
    pub(crate) fn face_vertex_window(
        self,
        t: f64,
        end: f64,
        feature: SeparatingFeature,
        normal: DVec3,
    ) -> Option<FaceVertexCrossing> {
        let SeparatingFeature::SupportFace(axis) = feature else {
            return None;
        };
        if axis >= 3 || !self.moving.valid() || !self.support.valid() {
            return None;
        }
        let width = end - t;
        if !width.is_finite() || width <= 0. || width > 1.000001e-9 {
            return None;
        }
        let low = self.at(t);
        let high = self.at(end);
        let face = self.support.orientation * [DVec3::X, DVec3::Y, DVec3::Z][axis];
        let mut witness = FaceVertexCrossing {
            axis,
            positive: face.dot(normal) > 0.,
            vertex: 0,
        };
        let h0 = witness.heights(low, self.support);
        let h1 = witness.heights(high, self.support);
        let v = (0..8).min_by(|i, j| h1[*i].total_cmp(&h1[*j]))?;
        witness.vertex = v;
        let eps = FaceVertexCrossing::precision(high, self.support);
        let radius =
            self.moving.center.distance(self.center_of_mass) + self.moving.half_extent.length();
        let accel = self.acceleration.length() + self.angular_velocity.length_squared() * radius;
        // A twice-differentiable projected height differs from its chord by at most A*h²/8.
        let clearance = eps + 0.125 * accel * width * width;
        if h0[v] < -eps || h1[v] > eps || (0..8).any(|i| i != v && h0[i].min(h1[i]) <= clearance) {
            return None;
        }
        let point = witness.point(low, v);
        let n = witness.normal(self.support);
        let velocity = self.relative_velocity
            + t * self.acceleration
            + self.angular_velocity.cross(point - self.com_at(t));
        if velocity.dot(n) + accel * width >= -256. * f64::EPSILON * (1. + velocity.length()) {
            return None;
        }
        if witness
            .footprint(point, self.support)
            .min(witness.footprint(witness.point(high, v), self.support))
            <= clearance
        {
            return None;
        }
        Some(witness)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FiniteBoxArrival;
    use glam::DQuat;
    use std::time::Duration;

    fn tilted(roll: f64, pitch: f64, late_fraction: f64) -> (FiniteBoxFlight, BoxCrossing) {
        let rotation = DQuat::from_rotation_x(roll) * DQuat::from_rotation_y(pitch);
        let half = DVec3::new(0.18, 0.1, 0.06);
        let height = half.dot((rotation.conjugate() * DVec3::Z).abs());
        let moving = FlightBox {
            center: DVec3::Z * (height + (133134. - late_fraction) * 1e-9),
            orientation: rotation,
            half_extent: half,
        };
        let flight = FiniteBoxFlight {
            moving,
            support: FlightBox {
                center: -0.06 * DVec3::Z,
                orientation: DQuat::IDENTITY,
                half_extent: DVec3::new(0.35, 0.3, 0.06),
            },
            center_of_mass: moving.center,
            relative_velocity: -DVec3::Z,
            angular_velocity: DVec3::ZERO,
            acceleration: DVec3::ZERO,
        };
        let FiniteBoxArrival::Crossing(hit) =
            flight.first_crossing(Duration::from_micros(500)).unwrap()
        else {
            panic!("finite footprint contact")
        };
        (flight, hit)
    }
    #[test]
    fn accepts_one_material_vertex_between_nanosecond_boundaries_and_revalidates_the_shape() {
        let (f, hit) = tilted(2e-8, 3e-8, 0.7);
        let proof = f.isolated_face_vertex(hit).unwrap();
        let end = f.at(hit.latest.as_secs_f64());
        assert!(f64::abs((hit.latest - hit.earliest).as_secs_f64() - 1e-9) < 1e-20);
        let (_, n, [a, b], g) = proof.measure(end, f.support, 1.0001e-9).unwrap();
        assert_eq!(n, DVec3::Z);
        assert!(g < 0. && g > -1e-9);
        assert!(a.distance(b) <= 1e-9);
        let changed = FlightBox {
            orientation: DQuat::IDENTITY,
            ..end
        };
        assert!(proof.measure(changed, f.support, 1.0001e-9).is_none());
        let shifted = FlightBox {
            center: f.support.center + DVec3::X,
            ..f.support
        };
        assert!(proof.measure(end, shifted, 1.0001e-9).is_none());
    }
    #[test]
    fn declines_coincident_or_already_incident_neighboring_vertices() {
        for roll in [0., 1e-10] {
            let (f, hit) = tilted(roll, 3e-8, 0.8);
            assert!(f.isolated_face_vertex(hit).is_none());
        }
        let (f, hit) = tilted(2e-8, 3e-8, 0.7);
        let invalid = BoxCrossing {
            earliest: Duration::ZERO,
            ..hit
        };
        assert!(f.isolated_face_vertex(invalid).is_none());
    }
}
