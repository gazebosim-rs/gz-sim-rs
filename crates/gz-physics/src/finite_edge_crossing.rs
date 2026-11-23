// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Geometric isolation of a pair of skew finite box edges over a short arrival bracket.
use crate::{
    BoxCrossing, FaceVertexCrossing, FiniteBoxFlight, FiniteFeatureArc, FinitePointSite, FlightBox,
    PhysicsError, SeparatingFeature, SphericalPointMaterial,
};
use glam::DVec3;

const BASIS: [DVec3; 3] = [DVec3::X, DVec3::Y, DVec3::Z];

/// Identifies both physical edges and the orientation of their separating plane.
#[derive(Clone, Copy, Debug)]
pub struct SkewEdgeCrossing {
    axes: [usize; 2],
    signs: [DVec3; 2],
    direction: f64,
}
struct Measurement {
    normal: DVec3,
    points: [DVec3; 2],
    gap: f64,
    off_edge_clearance: f64,
    interior: f64,
    sine: f64,
}
impl SkewEdgeCrossing {
    fn signs(body: FlightBox, axis: usize, normal: DVec3) -> DVec3 {
        let local = body.orientation.conjugate() * normal;
        DVec3::from_array(std::array::from_fn(|k| {
            if k == axis { 0. } else { local[k].signum() }
        }))
    }
    fn measure_at(self, body: FlightBox, pad: FlightBox) -> Option<Measurement> {
        let [i, j] = self.axes;
        let u = body.orientation * BASIS[i];
        let v = pad.orientation * BASIS[j];
        let cross = u.cross(v);
        let sine = cross.length();
        if sine < 1e-7 {
            return None;
        }
        let n = self.direction * cross / sine;
        let anchor = |b: FlightBox, s: DVec3| b.center + b.orientation * (s * b.half_extent);
        let a = anchor(body, self.signs[0]);
        let b = anchor(pad, self.signs[1]);
        let delta = b - a;
        let s = delta.cross(v).dot(cross) / cross.length_squared();
        let t = u.dot(v) * s - v.dot(delta);
        let points = [a + s * u, b + t * v];
        let gap = (a - b).dot(n);
        // Each adjacent edge flips one non-free coordinate. More distant vertices
        // add both clearances, so these four tests cover all twelve off-edge vertices.
        let mut clearance = f64::INFINITY;
        for (shape, axis, signs, normal) in
            [(body, i, self.signs[0], n), (pad, j, self.signs[1], -n)]
        {
            for k in 0..3 {
                if k == axis {
                    continue;
                }
                let separation = -2.
                    * signs[k]
                    * shape.half_extent[k]
                    * (shape.orientation * BASIS[k]).dot(normal);
                clearance = clearance.min(separation).min(gap + separation);
            }
        }
        Some(Measurement {
            normal: n,
            points,
            gap,
            off_edge_clearance: clearance,
            interior: (body.half_extent[i] - s.abs()).min(pad.half_extent[j] - t.abs()),
            sine,
        })
    }
    fn precision(a: FlightBox, b: FlightBox) -> f64 {
        256. * f64::EPSILON
            * (1.
                + a.center.length()
                + b.center.length()
                + a.half_extent.length()
                + b.half_extent.length())
    }
    /// Recheck the same pair of finite edges on the shapes actually being committed.
    pub fn measure(
        self,
        body: FlightBox,
        pad: FlightBox,
        allowance: f64,
    ) -> Option<(SeparatingFeature, DVec3, [DVec3; 2], f64)> {
        if !body.valid() || !pad.valid() || !allowance.is_finite() || allowance < 0. {
            return None;
        }
        let m = self.measure_at(body, pad)?;
        let eps = Self::precision(body, pad);
        (m.gap <= eps
            && -m.gap <= allowance + eps
            && m.off_edge_clearance > eps
            && m.interior > eps / m.sine
            && (m.points[0] - m.points[1] - m.normal * m.gap).length() <= eps / m.sine)
            .then_some((
                SeparatingFeature::EdgePair(self.axes[0], self.axes[1]),
                m.normal,
                m.points,
                m.gap,
            ))
    }
}
impl FiniteBoxFlight {
    /// Certifies that the unimpeded ballistic flight has exactly one incident
    /// edge pair through the whole bracket. Subsequent material changes require
    /// separate contact-dynamics checks after the first impulse.
    pub fn isolated_skew_edges(self, crossing: BoxCrossing) -> Option<SkewEdgeCrossing> {
        self.skew_edge_window(
            crossing.earliest.as_secs_f64(),
            crossing.latest.as_secs_f64(),
            crossing.feature,
            crossing.normal,
        )
    }
    pub(crate) fn skew_edge_window(
        self,
        lo: f64,
        hi: f64,
        feature: SeparatingFeature,
        normal: DVec3,
    ) -> Option<SkewEdgeCrossing> {
        let SeparatingFeature::EdgePair(i, j) = feature else {
            return None;
        };
        if i >= 3 || j >= 3 || !self.moving.valid() || !self.support.valid() {
            return None;
        }
        let h = hi - lo;
        if !h.is_finite() || h <= 0. || h > 1.000001e-9 {
            return None;
        }
        let a = self.at(lo);
        let b = self.at(hi);
        let pad = self.support;
        let cross = (b.orientation * BASIS[i]).cross(pad.orientation * BASIS[j]);
        let direction = cross.dot(normal).signum();
        let n = direction * cross.try_normalize()?;
        let witness = SkewEdgeCrossing {
            axes: [i, j],
            signs: [
                SkewEdgeCrossing::signs(b, i, -n),
                SkewEdgeCrossing::signs(pad, j, n),
            ],
            direction,
        };
        let m0 = witness.measure_at(a, pad)?;
        let m1 = witness.measure_at(b, pad)?;
        let omega = self.angular_velocity.length();
        let sin = m0.sine.min(m1.sine) - omega * h;
        if sin < 1e-7 {
            return None;
        }
        let eps = SkewEdgeCrossing::precision(b, pad);
        let r = self.moving.center.distance(self.center_of_mass) + self.moving.half_extent.length();
        let accel = self.acceleration.length() + omega * omega * r;
        let speed = (self.relative_velocity + lo * self.acceleration).length()
            + self.acceleration.length() * h
            + omega * r;
        let d = a.center.distance(pad.center)
            + a.half_extent.length()
            + pad.half_extent.length()
            + speed * h;
        // For n = (u × v) / |u × v| on a rotating edge u, bound n' and n''.
        let dn = omega / sin;
        let ddn = omega * omega / sin + 3. * dn * dn;
        let curvature = d * ddn + 2. * dn * speed + accel;
        // The two points of an on-body separation can move, unlike a
        // moving-to-static gap. Bound both using two independent extremal vertices.
        let relative_curvature =
            (2. * r * ddn + 4. * dn * omega * r + 2. * omega * omega * r).max(curvature);
        let separation_error = eps + relative_curvature * h * h / 8.;
        // Differentiate s=(q-p)·w/sin and t=(u·v)s-v·(q-p). The normalized
        // in-plane w varies no faster than omega/sin.
        let ds = speed / sin + 2. * d * omega / (sin * sin);
        let dt = ds + d * omega / sin + speed;
        let endpoint_error = eps / sin + 0.5 * h * ds.max(dt);
        if m0.gap < -eps
            || m1.gap > eps
            || (m1.gap - m0.gap + 2. * eps) / h + curvature * h >= 0.
            || m0.off_edge_clearance.min(m1.off_edge_clearance) <= separation_error
            || m0.interior.min(m1.interior) <= endpoint_error
        {
            return None;
        }
        Some(witness)
    }
}

/// A material point whose correspondence can be rechecked on the live native shapes.
#[derive(Clone, Copy, Debug)]
pub enum FiniteContactWitness {
    FaceVertex(FaceVertexCrossing),
    SkewEdges(SkewEdgeCrossing),
}
impl FiniteContactWitness {
    pub fn measure(
        self,
        body: FlightBox,
        pad: FlightBox,
        allowance: f64,
    ) -> Option<(SeparatingFeature, DVec3, [DVec3; 2], f64)> {
        match self {
            Self::FaceVertex(w) => w.measure(body, pad, allowance),
            Self::SkewEdges(w) => w.measure(body, pad, allowance),
        }
    }
}
impl FiniteBoxFlight {
    /// For a spherical passive body, verify the physical single-site outgoing
    /// motion for a whole nanosecond from tangency. This guards the gap between
    /// a subnanosecond physical impact and its integer native application.
    pub fn single_skew_impact_in_bracket(
        self,
        crossing: BoxCrossing,
        material: SphericalPointMaterial,
    ) -> Result<Option<SkewEdgeCrossing>, PhysicsError> {
        let Some(w) = self.isolated_skew_edges(crossing) else {
            return Ok(None);
        };
        if ![material.mass, material.inertia, material.friction]
            .into_iter()
            .all(f64::is_finite)
            || material.mass <= 0.
            || material.inertia <= 0.
            || material.friction < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let mut lo = crossing.earliest.as_secs_f64();
        let mut hi = crossing.latest.as_secs_f64();
        let width = hi - lo;
        // Keep the opposite signs of the measured edge-plane gap. Only intervals
        // with strict closure and unchanged finite edge identity reach this step.
        for _ in 0..32 {
            let mid = lo + 0.5 * (hi - lo);
            let Some(m) = w.measure_at(self.at(mid), self.support) else {
                return Ok(None);
            };
            if m.gap > 0. {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let t = lo + 0.5 * (hi - lo);
        let moving = self.at(t);
        let Some(m) = w.measure_at(moving, self.support) else {
            return Ok(None);
        };
        let site = FinitePointSite {
            point: m.points[0],
            normal: m.normal,
            feature: crossing.feature,
        };
        let Some(hit) = self.spherical_impact_at(t, site, material)? else {
            return Ok(None);
        };
        let after = hit.after;
        let tol = SkewEdgeCrossing::precision(moving, self.support) * 4.;
        let Some(arc) = FiniteFeatureArc::new(after, site, material, hit.sticking, tol)? else {
            return Ok(None);
        };
        let full = if hit.sticking {
            arc.sticking_point_span(width)?.is_some()
        } else {
            arc.sliding_skew_launch_span(width)?.is_some()
        };
        Ok(full.then_some(w))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FiniteBoxArrival;
    use glam::DQuat;
    use std::time::Duration;
    const MATERIAL: SphericalPointMaterial = SphericalPointMaterial {
        mass: 2.,
        inertia: 0.05,
        friction: 0.5,
    };
    fn measured(case: u64) -> FiniteBoxFlight {
        let (pos, q, center, velocity, omega, pad) = match case {
            122 => (
                [
                    0.0006870459668999287,
                    0.0005813561670885263,
                    0.030024618977305788,
                ],
                [
                    0.003030676401423667,
                    9.214359340299829e-06,
                    0.0030411743650535757,
                    0.9999907830445873,
                ],
                [
                    0.0006896263194709127,
                    0.00015706930474078786,
                    0.10002333306549618,
                ],
                [
                    -0.00022404525304203391,
                    0.036836572272653335,
                    -1.251719683646183,
                ],
                [
                    -0.2774943420300811,
                    -0.0016877607150277765,
                    -9.970292806489682e-10,
                ],
                [0.175, 0.0, -0.09105000001513969],
            ),
            231 => (
                [
                    0.0007806461175550924,
                    -0.0018845523002655896,
                    0.03158095271480072,
                ],
                [
                    0.023083619897307724,
                    7.664386128072773e-05,
                    0.0033193789103308706,
                    0.9997280241854807,
                ],
                [
                    0.0008021005991442408,
                    -0.00511534452240102,
                    0.10150635240134245,
                ],
                [
                    0.00013611212178376412,
                    -0.020495188883282316,
                    -1.2937218409166977,
                ],
                [
                    0.15377272235879044,
                    0.0010212318345392344,
                    6.052584592173695e-10,
                ],
                [0.175, 0.0, -0.09326399998214882],
            ),

            _ => panic!("unknown test donor"),
        };
        FiniteBoxFlight {
            moving: FlightBox {
                center: pos.into(),
                orientation: DQuat::from_array(q),
                half_extent: DVec3::new(0.18, 0.1, 0.06),
            },
            support: FlightBox {
                center: pad.into(),
                orientation: DQuat::IDENTITY,
                half_extent: DVec3::new(0.33, 0.18, 0.06),
            },
            center_of_mass: center.into(),
            relative_velocity: velocity.into(),
            angular_velocity: omega.into(),
            acceleration: -9.81 * DVec3::Z,
        }
    }
    fn crossing(f: FiniteBoxFlight) -> BoxCrossing {
        let FiniteBoxArrival::Crossing(c) = f.first_crossing(Duration::from_micros(500)).unwrap()
        else {
            panic!("measured crossing");
        };
        c
    }
    #[test]
    fn strict_edges_and_one_site_survive_the_final_bracket() {
        let f = measured(122);
        let c = crossing(f);
        let result = f
            .single_skew_impact_in_bracket(c, MATERIAL)
            .unwrap()
            .unwrap();
        assert_eq!(c.latest.as_nanos(), 383002);
        let (feature, n, [a, b], gap) = result
            .measure(f.at(c.latest.as_secs_f64()), f.support, 2e-9)
            .unwrap();
        assert_eq!(feature, SeparatingFeature::EdgePair(0, 1));
        assert!((a - b - n * gap).length() < 2e-14);
        let different = FlightBox {
            center: f.support.center + 0.1 * DVec3::Z,
            ..f.support
        };
        assert!(
            result
                .measure(f.at(c.latest.as_secs_f64()), different, 2e-9)
                .is_none()
        );
    }
    #[test]
    fn unimpeded_edges_can_be_unique_while_the_postimpact_vertex_changes() {
        let seed = measured(231);
        // Start from the same physical flight with the positive impact at phase 49.5/64 ns.
        let dt = (0.2685889168 - 49.5 / 64.) * 1e-9;
        let f = FiniteBoxFlight {
            moving: seed.at(dt),
            center_of_mass: seed.com_at(dt),
            relative_velocity: seed.relative_velocity + dt * seed.acceleration,
            ..seed
        };
        let c = crossing(f);
        assert!(f.isolated_skew_edges(c).is_some());
        assert!(
            f.single_skew_impact_in_bracket(c, MATERIAL)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn fractional_material_path_resolves_both_impulses_and_declines_uncertain_inputs() {
        let f = measured(231);
        let c = crossing(f);
        let path = f.fractional_point_path(c, MATERIAL).unwrap().unwrap();
        let later: Vec<_> = path
            .follow
            .iter()
            .filter_map(|piece| piece.impact)
            .collect();
        assert_eq!(later.len(), 1);
        assert!((path.first_time * 1e9 - 224078.268588928).abs() < 0.0001);
        assert!(path.first.lost_energy > 0. && later[0].law.lost_energy > 0.);
        let net = DVec3::new(0.227743088, 0.610170629, 2.322893387);
        assert!(path.impulse().distance(net) < 2e-8);
        assert!(
            f.fractional_point_path(
                BoxCrossing {
                    earliest: c.latest,
                    ..c
                },
                MATERIAL
            )
            .unwrap()
            .is_none()
        );
        assert!(
            f.fractional_point_path(
                c,
                SphericalPointMaterial {
                    friction: -0.01,
                    ..MATERIAL
                }
            )
            .is_err()
        );
        assert!(
            FiniteBoxFlight {
                relative_velocity: DVec3::NAN,
                ..f
            }
            .fractional_point_path(c, MATERIAL)
            .is_err()
        );
    }
}
