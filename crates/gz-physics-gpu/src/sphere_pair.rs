// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Continuous contact between two finite, spinning solid spheres in free space.
use crate::arithmetic::{Condition, Scalar};
use crate::{
    GpuError, ResidentBatch, STEP,
    kernel::build,
    spatial::{V3, cross, dot, rotate},
};
use bytemuck::{Pod, Zeroable};
use naga::Module;

/// World-frame SI state of a homogeneous sphere; orientation is xyzw.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct SolidSphere {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub orientation: [f64; 4],
    pub radius: f64,
    pub inverse_mass: f64,
}
impl SolidSphere {
    fn valid(&self, g: [f64; 3]) -> bool {
        let q2 = self.orientation.into_iter().map(|x| x * x).sum::<f64>();
        let w2 = self
            .angular_velocity
            .into_iter()
            .map(|x| x * x)
            .sum::<f64>();
        let free2 = self
            .velocity
            .into_iter()
            .zip(g)
            .map(|(v, g)| (v + STEP.as_secs_f64() * g).powi(2))
            .sum::<f64>();
        let inv_i = 2.5 * self.inverse_mass / (self.radius * self.radius);
        (q2 - 1.).abs() <= 2e-12
            && self.radius > 0.
            && self.inverse_mass > 0.
            && self.inverse_mass.recip().is_finite()
            && inv_i.is_finite()
            && inv_i > 0.
            && (free2.sqrt() + self.radius * w2.sqrt()).powi(2).is_finite()
    }
}

/// One independent pair under common uniform gravity. The last impulse acts on
/// sphere B, along the normal from A to B plus its world-frame tangent vector.
/// impact_time is the GPU's continuous time of impact in seconds from tick start,
/// or -1 if unavailable. A guarded native solve provides a separate substep interval.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct SpherePairLane {
    pub a: SolidSphere,
    pub b: SolidSphere,
    pub gravity: [f64; 3],
    pub friction: f64,
    pub restitution: f64,
    pub normal_impulse: f64,
    pub tangential_impulse: [f64; 3],
    pub contact_normal: [f64; 3],
    pub impact_time: f64,
}
const WIDTH: usize = 43;
const _: () = assert!(std::mem::size_of::<SpherePairLane>() == WIDTH * 8);
impl SpherePairLane {
    pub(super) fn initial_valid(self) -> bool {
        let d2 = self
            .a
            .position
            .into_iter()
            .zip(self.b.position)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>();
        let u2 = self
            .a
            .velocity
            .into_iter()
            .zip(self.b.velocity)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>();
        let r = self.a.radius + self.b.radius;
        let dist = d2.sqrt();
        bytemuck::cast_ref::<Self, [f64; WIDTH]>(&self)
            .iter()
            .all(|v| v.is_finite())
            && self.a.valid(self.gravity)
            && self.b.valid(self.gravity)
            && self.friction >= 0.
            && (0.0..=1.).contains(&self.restitution)
            && (3.5 * (self.a.inverse_mass + self.b.inverse_mass)).is_finite()
            && d2.is_finite()
            && d2 > 0.
            && u2.is_finite()
            && r.powi(2).is_finite()
            && r.powi(2) > 0.
            && dist + 16. * f64::EPSILON * dist.max(r) >= r
            && self.normal_impulse == 0.
            && self.tangential_impulse == [0.; 3]
            && self.contact_normal == [0.; 3]
            && self.impact_time == -1.
    }
}
pub type SpherePairBatch = ResidentBatch<SpherePairLane>;
impl SpherePairBatch {
    pub fn new(adapter_name: &str, initial: Vec<SpherePairLane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter_name,
            initial,
            SpherePairLane::initial_valid,
            module,
            None,
        )
    }
}
pub(super) const PARAMETERS: &[f64] = &[
    STEP.as_secs_f64(),
    0.,
    1.,
    0.5,
    2.5,
    3.5,
    -1.,
    64. * f64::EPSILON,
];
pub(super) fn module() -> Module {
    build::<WIDTH, 34>(PARAMETERS, |s, c| tick(&s, c))
}
pub(super) fn tick<T: Scalar>(s: &[T], c: &[T]) -> [(usize, T); 34] {
    let [h, zero, one, half, inertia_factor, kfactor, miss, roundoff] = *c else {
        unreachable!()
    };
    let v = |i: usize| [s[i], s[i + 1], s[i + 2]];
    let a = v(0);
    let b = v(15);
    let va = v(3);
    let vb = v(18);
    let wa = v(6);
    let wb = v(21);
    let ra = s[13];
    let rb = s[28];
    let ia = s[14];
    let ib = s[29];
    let g = v(30);
    let friction = s[33];
    let restitution = s[34];
    let r = ra + rb;
    let delta: V3<T> = std::array::from_fn(|i| b[i] - a[i]);
    let u: V3<T> = std::array::from_fn(|i| vb[i] - va[i]);
    let dist = dot(delta, delta).root();
    let speed = dot(u, u).root();
    let speed_nonzero = speed.greater(zero);
    let safe_speed = speed_nonzero.select(speed, one);
    let dir = u.map(|x| x / safe_speed);
    let approach = zero - dot(delta, dir);
    let lateral = cross(delta, dir);
    let lateral2 = dot(lateral, lateral);
    // Solve for first touch via gap*(distance+R)/(approach+half-chord).
    // The cross product retains glancing offsets, and this root avoids
    // subtracting nearly equal distances when the pair starts in contact.
    let disc = r * r - lateral2;
    let half_chord = disc.maximum(zero).root();
    let denom = approach + half_chord;
    let surface_distance = ((dist - r) * (dist + r)) / denom.greater(zero).select(denom, one);
    let toi = (surface_distance / safe_speed).maximum(zero);
    // Close the endpoint under relative-coordinate rounding. This only
    // admits a root within a spatial roundoff band; an actual swept
    // intersection and an approaching relative velocity are still required.
    let endpoint: V3<T> = std::array::from_fn(|i| delta[i] + h * u[i]);
    let endpoint_radius = dot(endpoint, endpoint).root();
    let slack = roundoff * (r + dist + h * speed);
    let in_interval = toi.at_most(h).or(endpoint_radius.at_most(r + slack));
    let toi = toi.minimum(h);
    let reaches = speed_nonzero
        .and(approach.greater(zero))
        .and(disc.at_least(zero))
        .and(in_interval);
    let at = reaches.select(toi, zero);
    let separation: V3<T> = std::array::from_fn(|i| delta[i] + u[i] * at);
    let length = dot(separation, separation).root();
    let n = separation.map(|x| x / length.greater(zero).select(length, one));
    let un = dot(u, n);
    let hit = reaches.and(un.less(zero));
    let t = hit.select(toi, h);
    let remaining = h - t;
    let normal_impulse = hit.select((zero - un) * (one + restitution) / (ia + ib), zero);
    let arm_a = n.map(|x| x * ra);
    let arm_b = n.map(|x| (zero - x) * rb);
    let point_a = cross(wa, arm_a);
    let point_b = cross(wb, arm_b);
    let relative: V3<T> = std::array::from_fn(|i| u[i] + point_b[i] - point_a[i]);
    let vn = dot(relative, n);
    let slip: V3<T> = std::array::from_fn(|i| relative[i] - vn * n[i]);
    let tangential_speed = dot(slip, slip).root();
    let safe = tangential_speed.greater(zero).select(tangential_speed, one);
    let scale = (one / (kfactor * (ia + ib))).minimum(friction * normal_impulse / safe);
    let jt = slip.map(|x| (zero - x) * scale);
    let impulse: V3<T> = std::array::from_fn(|i| n[i] * normal_impulse + jt[i]);
    let mut output = [(0, zero); 34];
    for (k, base, sign, m, radius, arm, spin) in [
        (0, 0, miss, ia, ra, arm_a, wa),
        (13, 15, one, ib, rb, arm_b, wb),
    ] {
        let dv = impulse.map(|x| sign * m * x);
        let torque = cross(arm, jt);
        let spin1: V3<T> = std::array::from_fn(|i| {
            spin[i] + sign * inertia_factor * m / (radius * radius) * torque[i]
        });
        let q = [s[base + 9], s[base + 10], s[base + 11], s[base + 12]];
        let q1 = rotate(rotate(q, spin, t, half), spin1, remaining, half);
        for i in 0..3 {
            output[k + i] = (
                base + i,
                s[base + i] + h * (s[base + 3 + i] + half * h * g[i]) + dv[i] * remaining,
            );
            output[k + 3 + i] = (base + 3 + i, s[base + 3 + i] + h * g[i] + dv[i]);
            output[k + 6 + i] = (base + 6 + i, spin1[i]);
        }
        for i in 0..4 {
            output[k + 9 + i] = (base + 9 + i, q1[i]);
        }
    }
    output[26] = (35, normal_impulse);
    for i in 0..3 {
        output[27 + i] = (36 + i, jt[i]);
        output[30 + i] = (39 + i, hit.select(n[i], zero));
    }
    output[33] = (42, hit.select(toi, miss));
    output
}
