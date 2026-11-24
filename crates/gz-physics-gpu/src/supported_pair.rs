// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The invariant frictionless-plane family for two co-supported spheres.
use crate::{
    GpuError, ResidentBatch, STEP, SpherePairLane,
    kernel::build,
    spatial::{V3, dot},
    sphere_pair::{PARAMETERS, tick},
};
use bytemuck::{Pod, Zeroable};
use naga::Module;

/// Both equal-radius spheres remain tangent to the same stationary plane.
/// Initial velocities lie in its tangent plane, spin is around its normal,
/// and gravity continuously presses both spheres against the frictionless plane.
/// Pair friction and restitution retain their ordinary three-dimensional meanings.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct SupportedSpherePairLane {
    pub pair: SpherePairLane,
    pub plane_normal: [f64; 3],
    pub plane_offset: f64,
    /// Scalar impulses on A then B along the outward plane normal this tick.
    pub support_normal_impulses: [f64; 2],
}
const WIDTH: usize = 49;
const _: () = assert!(std::mem::size_of::<SupportedSpherePairLane>() == WIDTH * 8);
fn vector_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}
impl SupportedSpherePairLane {
    pub(super) fn valid(self) -> bool {
        let tolerance = 64. * f64::EPSILON;
        let n = self.plane_normal;
        let g_n = vector_dot(n, self.pair.gravity);
        if !self.pair.initial_valid()
            || self.pair.a.radius != self.pair.b.radius
            || !self.plane_offset.is_finite()
            || !n.into_iter().all(f64::is_finite)
            || (vector_dot(n, n) - 1.).abs() > tolerance
            || !g_n.is_finite()
            || g_n >= 0.
            || self.support_normal_impulses != [0.; 2]
        {
            return false;
        }
        for body in [self.pair.a, self.pair.b] {
            let center = vector_dot(n, body.position);
            let vn = vector_dot(n, body.velocity);
            // Bound errors by the dot/cross products actually evaluated.
            // A large coordinate or speed parallel to an axis-aligned plane
            // cannot explain any normal lift or normal velocity.
            let sum_abs = |v: [f64; 3]| {
                v.into_iter()
                    .zip(n)
                    .map(|(x, y)| (x * y).abs())
                    .sum::<f64>()
            };
            let height_scale = sum_abs(body.position) + self.plane_offset.abs() + body.radius;
            let w = body.angular_velocity;
            let spin_aligned = [(0, 1), (1, 2), (2, 0)].into_iter().all(|(a, b)| {
                let (u, v) = (w[a] * n[b], w[b] * n[a]);
                (u - v).abs() <= tolerance * (u.abs() + v.abs())
            });
            let support = -(vn + STEP.as_secs_f64() * g_n) / body.inverse_mass;
            if !height_scale.is_finite()
                || !center.is_finite()
                || (center - self.plane_offset - body.radius).abs() > tolerance * height_scale
                || vn.abs() > tolerance * sum_abs(body.velocity)
                || !spin_aligned
                || !support.is_finite()
                || support <= 0.
            {
                return false;
            }
        }
        true
    }
}
pub type SupportedSpherePairBatch = ResidentBatch<SupportedSpherePairLane>;
impl SupportedSpherePairBatch {
    pub fn new(
        adapter_name: &str,
        initial: Vec<SupportedSpherePairLane>,
    ) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter_name,
            initial,
            SupportedSpherePairLane::valid,
            module,
            None,
        )
    }
}
fn tangent<T: crate::arithmetic::Scalar>(v: V3<T>, n: V3<T>) -> V3<T> {
    let normal = dot(v, n);
    std::array::from_fn(|i| v[i] - normal * n[i])
}
pub(super) fn module() -> Module {
    build::<WIDTH, 36>(PARAMETERS, |lane, c| tick_supported(&lane, c))
}
pub(super) fn tick_supported<T: crate::arithmetic::Scalar>(
    lane: &[T],
    c: &[T],
) -> [(usize, T); 36] {
    let h = c[0];
    let zero = c[1];
    let n = [lane[43], lane[44], lane[45]];
    let offset = lane[46];
    let radius = lane[13];
    let vector = |i: usize| [lane[i], lane[i + 1], lane[i + 2]];
    let project = |v| tangent(v, n);
    let mut local: [_; 43] = std::array::from_fn(|i| lane[i]);
    let g = vector(30);
    let gn = dot(g, n);
    let tangent_gravity = project(g);
    local[30..33].copy_from_slice(&tangent_gravity);
    // Work in the common tangent subspace. The fixed external normal
    // constraint and the pair's internal impulse act in orthogonal spaces.
    for base in [0, 15] {
        let p = project(vector(base));
        let v = project(vector(base + 3));
        let wn = dot(vector(base + 6), n);
        let w = n.map(|x| x * wn);
        local[base..base + 3].copy_from_slice(&p);
        local[base + 3..base + 6].copy_from_slice(&v);
        local[base + 6..base + 9].copy_from_slice(&w);
    }
    let mut result = tick(&local, c);
    for (start, base) in [(0, 0), (13, 15)] {
        let p = project([result[start].1, result[start + 1].1, result[start + 2].1]);
        let v = project([
            result[start + 3].1,
            result[start + 4].1,
            result[start + 5].1,
        ]);
        let wn = dot(
            [
                result[start + 6].1,
                result[start + 7].1,
                result[start + 8].1,
            ],
            n,
        );
        for i in 0..3 {
            result[start + i] = (base + i, p[i] + n[i] * (offset + radius));
            result[start + 3 + i] = (base + 3 + i, v[i]);
            result[start + 6 + i] = (base + 6 + i, n[i] * wn);
        }
    }
    let ja = (zero - (dot(vector(3), n) + h * gn)) / lane[14];
    let jb = (zero - (dot(vector(18), n) + h * gn)) / lane[29];
    std::array::from_fn(|i| match i {
        34 => (47, ja),
        35 => (48, jb),
        _ => result[i],
    })
}
