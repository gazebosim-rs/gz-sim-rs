// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fixed-base planar two-revolute arms carrying rigid point payloads at their tips.
use crate::arithmetic::Scalar;
use crate::{GpuError, ResidentBatch, STEP, kernel};
use bytemuck::{Pod, Zeroable};

/// Both axes point along world −Y. The first angle is measured from +X toward +Z,
/// and the second from link one. Positions and servo targets are [cos(q), sin(q)].
/// Link centers and lengths are in m, masses in kg, and COM inertias in kg m².
/// The grip is rigid. There are no contacts or limits. Results are SI f64 values.
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
#[repr(C)]
pub struct PlanarArmLane {
    pub angle: [[f64; 2]; 2],
    pub speed: [f64; 2],
    pub length: [f64; 2],
    pub center: [f64; 2],
    pub mass: [f64; 2],
    pub inertia: [f64; 2],
    pub tip_mass: f64,
    /// Downward acceleration magnitude, m/s².
    pub gravity: f64,
    pub target: [[f64; 2]; 2],
    /// Sinusoidal error gain in N m and viscous gain in N m s/rad.
    pub stiffness: [f64; 2],
    pub damping: [f64; 2],
    pub effort_limit: [f64; 2],
    /// Midpoint generalized torque during the last tick (initially zero).
    pub effort: [f64; 2],
}
const WIDTH: usize = 28;
const _: () = assert!(size_of::<PlanarArmLane>() == WIDTH * 8);
pub type PlanarArmBatch = ResidentBatch<PlanarArmLane>;

impl PlanarArmBatch {
    /// Change one servo pair at the committed boundary. Angles are relative joint
    /// coordinates in radians. The next tick uploads the current frame, while
    /// a world reset restores the construction targets and initial pose.
    pub fn set_target_angles(&mut self, lane: usize, radians: [f64; 2]) -> Result<(), GpuError> {
        set_target(lane, self.committed.get_mut(lane), radians)?;
        self.upload_needed = true;
        Ok(())
    }
    pub fn new(adapter_name: &str, initial: Vec<PlanarArmLane>) -> Result<Self, GpuError> {
        Self::new_profile(adapter_name, initial, PlanarArmLane::valid, module, None)
    }
}

pub(super) fn set_target(
    lane: usize,
    arm: Option<&mut PlanarArmLane>,
    radians: [f64; 2],
) -> Result<(), GpuError> {
    if !radians.iter().all(|v| v.is_finite()) {
        return Err(GpuError::InvalidArmTarget(lane));
    }
    arm.ok_or(GpuError::InvalidArmTarget(lane))?.target = radians.map(|q| {
        let (s, c) = q.sin_cos();
        [c, s]
    });
    Ok(())
}
impl PlanarArmLane {
    pub(super) fn valid(self) -> bool {
        let words = bytemuck::cast_ref::<Self, [f64; WIDTH]>(&self);
        if !words.iter().all(|v| v.is_finite()) {
            return false;
        }
        let unit = |v: &[f64; 2]| (v[0] * v[0] + v[1] * v[1] - 1.).abs() <= 2e-12;
        if !self.angle.iter().chain(&self.target).all(unit)
            || self.tip_mass < 0.
            || self.gravity < 0.
            || self.effort != [0.; 2]
        {
            return false;
        }
        if !(0..2).all(|i| {
            self.length[i] > 0.
                && (0.0..=self.length[i]).contains(&self.center[i])
                && self.mass[i] > 0.
                && self.inertia[i] > 0.
                && self.stiffness[i] >= 0.
                && self.damping[i] >= 0.
                && self.effort_limit[i] >= 0.
        }) {
            return false;
        }
        let matrix = ArmInertia::from_lane(words);
        if matrix.det0 <= 0. || !(matrix.det0 + matrix.det_s).is_finite() {
            return false;
        }
        // Reject incoming scales for which even the first physical solve overflows.
        tick(words, &[STEP.as_secs_f64(), 0., 1., 0.5])
            .iter()
            .all(|(_, value)| value.is_finite())
    }
    /// Inspect the committed elbow and tip positions as [world x, world z].
    pub fn points(&self) -> [[f64; 2]; 2] {
        let [a, b] = self.angle;
        let elbow = a.map(|v| self.length[0] * v);
        [
            elbow,
            [
                elbow[0] + self.length[1] * (a[0] * b[0] - a[1] * b[1]),
                elbow[1] + self.length[1] * (a[1] * b[0] + a[0] * b[1]),
            ],
        ]
    }
}

fn module() -> naga::Module {
    kernel::build::<WIDTH, 8>(&[STEP.as_secs_f64(), 0., 1., 0.5], |v, c| tick(&v, c))
}

// Configuration-independent coefficients of the symmetric 2 × 2 mass matrix.
#[derive(Clone, Copy)]
pub(super) struct ArmInertia<T> {
    a: T,
    b: T,
    k: T,
    det0: T,
    det_s: T,
    shoulder_weight: T,
    distal_weight: T,
}
impl<T: Scalar> ArmInertia<T> {
    /// Generalized velocity produced by a unit spatial impulse with this joint Jacobian.
    pub(super) fn inverse(&self, c: T, s: T, j: [T; 2]) -> [T; 2] {
        let m12 = self.b + self.k * c;
        let m11 = self.a + self.b + self.k * c + self.k * c;
        let det = self.det0 + self.det_s * s * s;
        [
            (self.b * j[0] - m12 * j[1]) / det,
            (m11 * j[1] - m12 * j[0]) / det,
        ]
    }
    pub(super) fn from_lane(v: &[T]) -> Self {
        let (l1, l2, r1, r2, m1, m2, i1, i2, p, g) = (
            v[6], v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15],
        );
        let first = i1 + m1 * r1 * r1;
        let ends = m2 * r2 + p * l2;
        let k = l1 * ends;
        let b = i2 + m2 * r2 * r2 + p * l2 * l2;
        let total = m2 + p;
        let a = first + total * l1 * l1;
        // Positive-term expansion of det(M) avoids subtracting nearly equal products.
        let diff = l2 - r2;
        let det0 = first * b + l1 * l1 * (total * i2 + m2 * p * diff * diff);
        let det_s = l1 * l1 * ends * ends;
        Self {
            a,
            b,
            k,
            det0,
            det_s,
            shoulder_weight: g * (m1 * r1 + total * l1),
            distal_weight: g * ends,
        }
    }
}
pub(super) fn tick<T: Scalar>(v: &[T], constants: &[T]) -> [(usize, T); 8] {
    let [dt, zero, one, half] = *constants else {
        unreachable!()
    };
    let (c1, s1, c2, s2) = (v[0], v[1], v[2], v[3]);
    let (w1, w2) = (v[4], v[5]);
    let torque = |i: usize, c: T, s: T, w: T| {
        let err = v[17 + 2 * i] * c - v[16 + 2 * i] * s;
        (v[20 + i] * err - v[22 + i] * w)
            .maximum(zero - v[24 + i])
            .minimum(v[24 + i])
    };
    let mass = ArmInertia::from_lane(v);
    let k = mass.k;
    let solve = |[c1, s1]: [T; 2], [c2, s2]: [T; 2], w1: T, w2: T| {
        let u1 = torque(0, c1, s1, w1);
        let u2 = torque(1, c2, s2, w2);
        let g2 = mass.distal_weight * (c1 * c2 - s1 * s2);
        let f1 = u1 + k * s2 * w2 * (w1 + w1 + w2) - mass.shoulder_weight * c1 - g2;
        let f2 = u2 - k * s2 * w1 * w1 - g2;
        let [a, b] = mass.inverse(c2, s2, [f1, f2]);
        [a, b, u1, u2]
    };
    let rotate = |c: T, s: T, w: T| {
        let t = half * dt * w;
        let x = c * (one - t * t) - s * (t + t);
        let y = s * (one - t * t) + c * (t + t);
        let norm = (x * x + y * y).root();
        [x / norm, y / norm]
    };
    // Explicit midpoint in relative joint coordinates. The Cayley rotation
    // advances a unit phasor without vendor-specific f64 trigonometry.
    let [a0, b0, _, _] = solve([c1, s1], [c2, s2], w1, w2);
    let w1_mid = w1 + half * dt * a0;
    let w2_mid = w2 + half * dt * b0;
    let [a1, b1, u1, u2] = solve(
        rotate(c1, s1, w1 * half),
        rotate(c2, s2, w2 * half),
        w1_mid,
        w2_mid,
    );
    let [a, b] = rotate(c1, s1, w1_mid);
    let [c, d] = rotate(c2, s2, w2_mid);
    let w1 = w1 + dt * a1;
    let w2 = w2 + dt * b1;
    [
        (0, a),
        (1, b),
        (2, c),
        (3, d),
        (4, w1),
        (5, w2),
        (26, u1),
        (27, u2),
    ]
}

#[cfg(test)]
#[path = "../tests/private/planar_arm.rs"]
mod tests;
