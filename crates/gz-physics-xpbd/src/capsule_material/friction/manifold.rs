// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Simultaneous unilateral and circular Coulomb solve on small, strongly
//! coupled material manifolds. Only the frozen endpoint velocities change.
use super::*;
use crate::coulomb_manifold::{dot, recover};

struct Direction {
    nodal: Vec<(usize, DVec3)>,
    translation: DVec3,
    torque: DVec3,
    boundary: usize,
}

impl Direction {
    fn mobility(&self, other: &Self, p: &[Particle], bs: &[PreparedBoundary]) -> f64 {
        let mut sum = 0.;
        for &(id, axis) in &self.nodal {
            if let Some((_, d)) = other.nodal.iter().find(|(i, _)| *i == id) {
                sum += p[id].inverse_mass * axis.dot(*d);
            }
        }
        if self.boundary == other.boundary
            && let Some(r) = &bs[self.boundary].receiver
        {
            sum += r.inverse_mass * self.translation.dot(other.translation)
                + self.torque.dot(r.inverse_inertia(other.torque));
        }
        sum
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn refine(
    rows: &[(Key, Contact)],
    witnesses: &[Row],
    points: &[ContactPoint<'_>],
    targets: &[f64],
    normals: &mut [f64],
    tangents: &mut [DVec3],
    p: &mut [Particle],
    bs: &mut [PreparedBoundary],
    mu: f64,
    retry: bool,
) -> bool {
    if rows.len() > 8 {
        return false;
    }
    let mut directions = Vec::with_capacity(3 * rows.len());
    let mut seed = Vec::with_capacity(3 * rows.len());
    let mut velocity = Vec::with_capacity(3 * rows.len());
    for (i, ((key, _), w)) in rows.iter().zip(witnesses).enumerate() {
        let b = &bs[key.boundary];
        let point = &points[i];
        let n = w.c.normal;
        let u = n.any_orthonormal_vector();
        let v = n.cross(u);
        let normal_arm = w.rigid.map_or(DVec3::ZERO, |(_, arm)| arm);
        let rel = point.relative_velocity(p, b);
        directions.push(Direction {
            nodal: w
                .c
                .feature
                .vertices()
                .iter()
                .zip(w.c.weights)
                .map(|(&id, weight)| (id, weight * n))
                .collect(),
            translation: n,
            torque: normal_arm.cross(n),
            boundary: key.boundary,
        });
        directions.extend([u, v].map(|d| Direction {
            nodal: point.skin.nodal(d).collect(),
            translation: d,
            torque: point.arm.cross(d),
            boundary: key.boundary,
        }));
        seed.extend([normals[i], tangents[i].dot(u), tangents[i].dot(v)]);
        velocity.extend([w.normal_velocity(p, b) - targets[i], rel.dot(u), rel.dot(v)]);
    }
    let d = directions.len();
    let mut k = vec![vec![0.; d]; d];
    for (i, a) in directions.iter().enumerate() {
        for (j, b) in directions[..=i].iter().enumerate() {
            k[i][j] = a.mobility(b, p, bs);
            k[j][i] = k[i][j];
        }
    }
    let free: Vec<_> = k
        .iter()
        .zip(velocity)
        .map(|(row, vel)| vel - dot(row, &seed))
        .collect();
    let Some(result) = recover(&k, &free, &seed, mu, retry) else {
        return false;
    };
    for (i, j) in result.chunks_exact(3).enumerate() {
        let w = &witnesses[i];
        let b = &mut bs[rows[i].0.boundary];
        let point = &points[i];
        let total =
            j[1] * directions[3 * i + 1].translation + j[2] * directions[3 * i + 2].translation;
        let delta = total - tangents[i];
        w.apply_normal(p, b, (j[0] - normals[i]) * w.c.normal);
        point.skin.apply(delta, p);
        if let Some(r) = &mut b.receiver {
            r.apply_velocity_impulse(point.arm, -delta);
        }
        normals[i] = j[0];
        tangents[i] = total;
    }
    true
}
