// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A bounded material trajectory from a fractional onset to its integer clock endpoint.
use crate::{
    BoundPointPiece, BoxCrossing, FiniteBoxFlight, FiniteContactWitness, FiniteFeatureArc,
    FinitePointSite, PhysicsError, PointImpact, SeparatingFeature, SphericalPointMaterial,
};
use glam::DVec3;

#[derive(Clone, Copy, Debug)]
pub struct SphericalImpactState {
    pub after: FiniteBoxFlight,
    pub impulse: DVec3,
    pub before_relative_velocity: DVec3,
    pub after_relative_velocity: DVec3,
    pub sticking: bool,
    pub lost_energy: f64,
}
#[derive(Clone, Debug)]
pub struct FractionalPointPath {
    /// Absolute seconds from the native flight's initial state.
    pub crossing_bracket: [f64; 2],
    pub first_time: f64,
    pub first_site: FinitePointSite,
    pub first: SphericalImpactState,
    /// Measured offsets from fractional first contact.
    pub follow: Vec<BoundPointPiece>,
}
impl FractionalPointPath {
    pub fn impulse(&self) -> DVec3 {
        self.follow.iter().fold(self.first.impulse, |sum, p| {
            sum + p.motion.impulse + p.impact.map_or(DVec3::ZERO, |e| e.law.impulse)
        })
    }
}
impl FiniteBoxFlight {
    pub(crate) fn spherical_impact_at(
        self,
        t: f64,
        site: FinitePointSite,
        m: SphericalPointMaterial,
    ) -> Result<Option<SphericalImpactState>, PhysicsError> {
        if ![t, m.mass, m.inertia, m.friction]
            .into_iter()
            .all(f64::is_finite)
            || m.mass <= 0.
            || m.inertia <= 0.
            || m.friction < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let center = self.com_at(t);
        let rel = self.relative_velocity + t * self.acceleration;
        let arm = site.point - center;
        let velocity = rel + self.angular_velocity.cross(arm);
        let n = site.normal;
        let x = n.any_orthonormal_vector();
        let basis = [x, n.cross(x), n];
        let law = PointImpact {
            inverse_mass: std::array::from_fn(|i| {
                std::array::from_fn(|j| {
                    basis[i].dot(basis[j] / m.mass + arm.cross(basis[j]).cross(arm) / m.inertia)
                })
            }),
            relative_velocity: basis.map(|n| n.dot(velocity)),
            friction: m.friction,
        };
        let Ok(hit) = law.solve() else {
            return Ok(None);
        };
        let impulse = basis
            .into_iter()
            .zip(hit.impulse)
            .map(|(n, j)| n * j)
            .sum::<DVec3>();
        let after = Self {
            moving: self.at(t),
            center_of_mass: center,
            relative_velocity: rel + impulse / m.mass,
            angular_velocity: self.angular_velocity + arm.cross(impulse) / m.inertia,
            ..self
        };
        Ok(Some(SphericalImpactState {
            after,
            impulse,
            before_relative_velocity: velocity,
            after_relative_velocity: after.relative_velocity + after.angular_velocity.cross(arm),
            sticking: hit.sticking,
            lost_energy: hit.dissipated_energy,
        }))
    }
    /// Track the first strictly closing separating plane and every subsequent
    /// supported piece through the upper integer timestamp. Unresolved mode,
    /// geometry, work or event isolation leaves the whole candidate uncommitted.
    pub fn fractional_point_path(
        self,
        c: BoxCrossing,
        m: SphericalPointMaterial,
    ) -> Result<Option<FractionalPointPath>, PhysicsError> {
        if !self.moving.valid()
            || !self.support.valid()
            || ![
                self.center_of_mass,
                self.relative_velocity,
                self.angular_velocity,
                self.acceleration,
            ]
            .into_iter()
            .all(|v| v.is_finite())
            || ![m.mass, m.inertia, m.friction]
                .into_iter()
                .all(f64::is_finite)
            || m.mass <= 0.
            || m.inertia <= 0.
            || m.friction < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let start = c.earliest.as_secs_f64();
        let end = c.latest.as_secs_f64();
        let h = end - start;
        if h <= 0. || h > 1.000001e-9 {
            return Ok(None);
        }
        let eps = 256.
            * f64::EPSILON
            * (1.
                + self.moving.center.length()
                + self.support.center.length()
                + self.moving.half_extent.length()
                + self.support.half_extent.length());
        if self.separation_at(start).0 <= eps || self.separation_at(end).0 >= -eps {
            return Ok(None);
        }
        let (mut l, mut r) = (start, end);
        // The bisection locates a proposed crossing; the following fixed plane
        // independently excludes every earlier intersection, including reentry.
        for _ in 0..32 {
            let t = l + 0.5 * (r - l);
            if self.separation_at(t).0 > 0. {
                l = t;
            } else {
                r = t;
            }
        }
        let (_, n, feature) = self.separation_at(r);
        let radius =
            self.moving.center.distance(self.center_of_mass) + self.moving.half_extent.length();
        let closing = -(self.relative_velocity + start * self.acceleration).dot(n)
            - self.angular_velocity.length() * radius
            - h * self.acceleration.length();
        if closing <= 0. || !closing.is_finite() {
            return Ok(None);
        }
        let pad = 8. * eps / closing;
        let lo = l - pad;
        let hi = r + pad;
        if lo < start || hi > end {
            return Ok(None);
        }
        let low = self.at(lo);
        let b = self.support;
        let plane = (low.center - b.center).dot(n)
            - low.half_extent.dot((low.orientation.conjugate() * n).abs())
            - b.half_extent.dot((b.orientation.conjugate() * n).abs());
        if plane <= eps || self.separation_at(hi).0 >= -eps {
            return Ok(None);
        }
        let witness = match feature {
            SeparatingFeature::EdgePair(..) => self
                .skew_edge_window(lo, hi, feature, n)
                .map(FiniteContactWitness::SkewEdges),
            SeparatingFeature::SupportFace(_) => self
                .face_vertex_window(lo, hi, feature, n)
                .map(FiniteContactWitness::FaceVertex),
            _ => None,
        };
        let Some(w) = witness else {
            return Ok(None);
        };
        let Some((kind, normal, [point, _], _)) = w.measure(self.at(r), b, eps * 2.) else {
            return Ok(None);
        };
        let site = FinitePointSite {
            feature: kind,
            normal,
            point,
        };
        let Some(first) = self.spherical_impact_at(r, site, m)? else {
            return Ok(None);
        };
        let Some(arc) = FiniteFeatureArc::new(first.after, site, m, first.sticking, 4. * eps)?
        else {
            return Ok(None);
        };
        let duration = end - r;
        let simple = if first.sticking {
            arc.sticking_point_span(duration)?.map(|s| (s.motion, None))
        } else if matches!(kind, SeparatingFeature::EdgePair(..)) {
            arc.sliding_skew_launch_span(duration)?
                .map(|s| (s.motion, s.slip_growth))
        } else {
            arc.sliding_vertex_span(duration)?.map(|s| (s.motion, None))
        };
        let follow = if let Some((motion, slip_growth)) = simple {
            vec![BoundPointPiece {
                start: 0.,
                sticking: first.sticking,
                slip_growth,
                motion,
                impact: None,
            }]
        } else if let Some(p) = arc.directed_point_transfers(duration, true, true)? {
            p.pieces
        } else {
            return Ok(None);
        };
        // Exclude a transfer whose timing overlaps either end of the bracket;
        // avoid changing the number of represented impulses for a rounded root.
        for piece in &follow {
            if piece.impact.is_some() {
                let at = piece.start + piece.motion.bracket[0];
                if at <= hi - lo || duration - at <= hi - lo {
                    return Ok(None);
                }
            }
        }
        Ok(Some(FractionalPointPath {
            crossing_bracket: [lo, hi],
            first_time: r,
            first_site: site,
            first,
            follow,
        }))
    }
}
