// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A point-impact certificate on a newly incident finite line segment.
use crate::{FiniteContactSegment, PhysicsError, PointImpact, SphericalPointMaterial};
use glam::DVec3;
#[derive(Debug, Clone, Copy)]
pub struct SegmentPointImpact {
    pub loaded_end: usize,
    pub impulse: DVec3,
    pub velocity_before: [DVec3; 2],
    pub velocity_after: [DVec3; 2],
    pub opening_lower_bound: f64,
    pub sticking: bool,
    pub lost_energy: f64,
}
impl FiniteContactSegment {
    /// Certify a single carrying endpoint for any admissible Coulomb pressure
    /// along this line. Normal mobility is affine in the source coordinate.
    /// Its minimum over a friction disk is concave, and its maximum convex,
    /// so both bounds are attained at the segment endpoints. If every
    /// nonpenetrating impulse field must open one end, affine normal velocity
    /// can vanish only at the other. Static two-sided loads remain unresolved.
    pub fn single_loaded_endpoint(
        self,
        center: DVec3,
        relative_com_velocity: DVec3,
        angular: DVec3,
        material: SphericalPointMaterial,
    ) -> Result<Option<SegmentPointImpact>, PhysicsError> {
        let SphericalPointMaterial {
            mass,
            inertia,
            friction,
        } = material;
        let n = self.normal;
        if ![center, relative_com_velocity, angular, n]
            .into_iter()
            .chain(self.moving_points)
            .all(|v| v.is_finite())
            || ![mass, inertia, friction].into_iter().all(f64::is_finite)
            || mass <= 0.
            || inertia <= 0.
            || friction < 0.
            || (n.length_squared() - 1.).abs() > 1e-10
        {
            return Err(PhysicsError::InvalidValue);
        }
        let r = self.moving_points.map(|p| p - center);
        let before = r.map(|x| relative_com_velocity + angular.cross(x));
        segment_point_law(before, n, friction, |i, j, f| {
            f / mass + r[j].cross(f).cross(r[i]) / inertia
        })
    }
}
/// Use measured generalized point mobility in world axes for a straight contact
/// segment on one rigid link. The two ports must share a support with uniform
/// material friction, and the interior port Jacobians interpolate affinely.
pub fn generalized_segment_impact(
    mobility: crate::EdgeMobility,
    before: [DVec3; 2],
    normal: DVec3,
    friction: f64,
) -> Result<Option<SegmentPointImpact>, PhysicsError> {
    use crate::edge_impact::{ScaledEdge, positive_semidefinite};
    let scaled = ScaledEdge::new(mobility, before.map(|v| v.to_array()), [friction; 2])?;
    if !positive_semidefinite(scaled.w)
        || !normal.is_finite()
        || (normal.length_squared() - 1.).abs() > 1e-10
    {
        return Err(PhysicsError::InvalidValue);
    }
    segment_point_law(before, normal, friction, |i, j, f| {
        DVec3::from_array(mobility[i][j].map(|row| DVec3::from_array(row).dot(f)))
    })
}
fn segment_point_law(
    before: [DVec3; 2],
    n: DVec3,
    friction: f64,
    response: impl Fn(usize, usize, DVec3) -> DVec3,
) -> Result<Option<SegmentPointImpact>, PhysicsError> {
    let tol = 512. * f64::EPSILON * (1. + before[0].length() + before[1].length());
    let row = |i: usize, j: usize| response(j, i, n);
    for i in 0..2 {
        let other = 1 - i;
        let needed = -before[i].dot(n);
        if needed <= tol {
            continue;
        }
        let mut upper = 0.0_f64;
        let mut lower = f64::INFINITY;
        for j in 0..2 {
            let a = row(i, j);
            let b = row(other, j);
            upper = upper.max(n.dot(a) + friction * (a - n * n.dot(a)).length());
            lower = lower.min(n.dot(b) - friction * (b - n * n.dot(b)).length());
        }
        if upper <= 0. || lower < 0. {
            continue;
        }
        let bound = before[other].dot(n) + lower * (needed / upper);
        if bound <= tol {
            continue;
        }
        let x = n.any_orthonormal_vector();
        let basis = [x, n.cross(x), n];
        let law = PointImpact {
            inverse_mass: std::array::from_fn(|a| {
                std::array::from_fn(|b| basis[a].dot(response(i, i, basis[b])))
            }),
            relative_velocity: basis.map(|b| b.dot(before[i])),
            friction,
        };
        let Ok(s) = law.solve() else { continue };
        let j = s.impulse[0] * x + s.impulse[1] * basis[1] + s.impulse[2] * n;
        let after = std::array::from_fn(|k| before[k] + response(k, i, j));
        if after[other].dot(n) + tol < bound || after[i].dot(n).abs() > tol * (1. + j.length()) {
            return Err(PhysicsError::InvalidValue);
        }
        return Ok(Some(SegmentPointImpact {
            loaded_end: i,
            impulse: j,
            velocity_before: before,
            velocity_after: after,
            opening_lower_bound: bound,
            sticking: s.sticking,
            lost_energy: s.dissipated_energy,
        }));
    }
    Ok(None)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn segment(half: f64) -> FiniteContactSegment {
        let p = [DVec3::new(-half, 0., 0.), DVec3::new(half, 0., 0.)];
        FiniteContactSegment {
            moving_points: p,
            support_points: p,
            normal: DVec3::Z,
        }
    }
    #[test]
    fn new_endpoint_opens_the_old_material_point_under_every_admissible_pressure() {
        let m = SphericalPointMaterial {
            mass: 2.,
            inertia: 0.05,
            friction: 0.5,
        };
        for sign in [-1., 1.] {
            let seg = segment(0.06);
            let w = sign * 2. * DVec3::Y;
            let c = DVec3::Z * 0.1;
            let old = usize::from(sign < 0.);
            let v = 0.35 * DVec3::Y - w.cross(seg.moving_points[old] - c);
            let h = seg.single_loaded_endpoint(c, v, w, m).unwrap().unwrap();
            assert_eq!(h.loaded_end, 1 - old);
            assert!(h.opening_lower_bound > 0.);
            assert!(h.velocity_after[old].z >= h.opening_lower_bound - 1e-13);
            assert!(h.velocity_after[1 - old].z.abs() < 1e-13);
            for j in 0..=50 {
                let alpha = f64::from(j) / 50.;
                assert!(h.velocity_after[0].lerp(h.velocity_after[1], alpha).z >= -1e-13);
            }
            let v2 = v + h.impulse / m.mass;
            let w2 = w + (seg.moving_points[h.loaded_end] - c).cross(h.impulse) / m.inertia;
            let loss = 0.5
                * (m.mass * (v.length_squared() - v2.length_squared())
                    + m.inertia * (w.length_squared() - w2.length_squared()));
            assert!((loss - h.lost_energy).abs() < 1e-12);
        }
    }
    #[test]
    fn widely_separated_ends_can_both_be_needed() {
        let m = SphericalPointMaterial {
            mass: 2.,
            inertia: 0.05,
            friction: 0.2,
        };
        let seg = segment(0.28);
        let c = 0.1 * DVec3::Z;
        let w = 2. * DVec3::Y;
        assert!(
            seg.single_loaded_endpoint(c, -w.cross(seg.moving_points[0] - c), w, m)
                .unwrap()
                .is_none()
        );
    }
}

/// Resolve a single carrying endpoint using either the inexpensive opening
/// bound or an exclusion of simultaneous line closure by distributed friction
/// work. A positive certificate refers to every admissible pressure on the
/// continuous segment. The opposite-end-only solution must independently fail.
pub fn generalized_segment_work_impact(
    mobility: crate::EdgeMobility,
    before: [DVec3; 2],
    normal: DVec3,
    direction: DVec3,
    friction: f64,
    max_boxes: u32,
) -> Result<Option<(SegmentPointImpact, u32)>, PhysicsError> {
    if let Some(h) = generalized_segment_impact(mobility, before, normal, friction)? {
        return Ok(Some((h, 0)));
    }
    if !direction.is_finite() || direction.length_squared() == 0. {
        return Err(PhysicsError::InvalidValue);
    }
    let dir = direction.normalize();
    if dir.dot(normal).abs() > 1e-10 {
        return Ok(None);
    }
    let x = dir.reject_from_normalized(normal).normalize();
    let basis = [x, normal.cross(x), normal];
    let response = |i: usize, j: usize, f: DVec3| {
        DVec3::from_array(mobility[i][j].map(|r| DVec3::from(r).dot(f)))
    };
    let w: crate::EdgeMobility = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            std::array::from_fn(|a| std::array::from_fn(|b| basis[a].dot(response(i, j, basis[b]))))
        })
    });
    let v = before.map(|u| basis.map(|a| a.dot(u)));
    let tol = 1024. * f64::EPSILON * (1. + before[0].length() + before[1].length());
    for i in 0..2 {
        let o = 1 - i;
        if v[i][2] >= -tol {
            continue;
        }
        let own = DVec3::from(w[o][o][2]);
        let cross = DVec3::from(w[i][o][2]);
        let minimum = own.z - friction * own.truncate().length();
        let maximum = (cross.z + friction * cross.truncate().length()).max(0.);
        if minimum <= 0. || v[i][2] + (-v[o][2]).max(0.) * maximum / minimum >= -tol {
            continue;
        }
        let Ok(h) = (PointImpact {
            inverse_mass: w[i][i],
            relative_velocity: v[i],
            friction,
        })
        .solve() else {
            continue;
        };
        let j = basis[0] * h.impulse[0] + basis[1] * h.impulse[1] + basis[2] * h.impulse[2];
        let after = [0, 1].map(|k| before[k] + response(k, i, j));
        let opening = after[o].dot(normal);
        if opening <= tol || after[i].dot(normal).abs() > tol * (1. + j.length()) {
            continue;
        }
        let proof = crate::exclude_closed_line(w, v, friction, max_boxes)?;
        if proof.status != crate::LineClosure::Excluded {
            return Ok(None);
        }
        return Ok(Some((
            SegmentPointImpact {
                loaded_end: i,
                impulse: j,
                velocity_before: before,
                velocity_after: after,
                opening_lower_bound: opening - tol,
                sticking: h.sticking,
                lost_energy: h.dissipated_energy,
            },
            proof.visited,
        )));
    }
    Ok(None)
}
