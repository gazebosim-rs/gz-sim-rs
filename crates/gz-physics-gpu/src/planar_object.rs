// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A round two-joint tool, one free sphere and a frictionless horizontal floor.
use crate::arithmetic::{Condition, Scalar};
use crate::planar_arm::{ArmInertia, rotate_by, tick_with_velocity};
use crate::{GpuError, PlanarToolLane, ResidentBatch, STEP, kernel};
use bytemuck::{Pod, Zeroable};

/// Position and linear velocity in the shoulder's world-aligned XZ plane, in SI units.
/// Frictionless sphere contacts produce no torque about the object's center.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct FreeSphere {
    pub position: [f64; 2],
    pub velocity: [f64; 2],
    pub radius: f64,
    pub mass: f64,
    /// Compressive tool impulse and its vector on the object during the last tick, N s.
    pub normal_impulse: f64,
    pub tool_impulse: [f64; 2],
    pub floor_impulse: f64,
}
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct PlanarObjectLane {
    pub tool: PlanarToolLane,
    pub object: FreeSphere,
}
pub type PlanarObjectBatch = ResidentBatch<PlanarObjectLane>;
const WIDTH: usize = 72;
const _: () = assert!(size_of::<PlanarObjectLane>() == 8 * WIDTH);
impl PlanarObjectLane {
    fn valid(self) -> bool {
        let b = self.object;
        let [x, z] = self.tool.arm.points()[1];
        let dx = b.position[0] - x;
        let dz = b.position[1] - z;
        let distance_squared = dx * dx + dz * dz;
        // Resolve only subtraction roundoff at an authored tangent contact.
        let roundoff = 8. * f64::EPSILON;
        let floor_gap = b.position[1] - self.tool.ground_z - b.radius;
        let pair_radius = b.radius + self.tool.radius;
        self.tool.valid()
            && self.tool.friction == 0.
            && !self.tool.stops.iter().any(|s| s.enabled())
            && bytemuck::cast_ref::<_, [f64; 10]>(&b)
                .iter()
                .all(|x| x.is_finite())
            && b.radius > 0.
            && b.mass > 0.
            && b.mass.recip().is_finite()
            && distance_squared.is_finite()
            && distance_squared.sqrt() >= pair_radius * (1. - roundoff)
            && floor_gap
                >= -roundoff
                    * b.position[1]
                        .abs()
                        .max(self.tool.ground_z.abs())
                        .max(b.radius)
            && b.normal_impulse == 0.
            && b.floor_impulse == 0.
            && b.tool_impulse == [0.; 2]
    }
}
impl PlanarObjectBatch {
    pub fn new(adapter: &str, lanes: Vec<PlanarObjectLane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter,
            lanes,
            PlanarObjectLane::valid,
            || {
                kernel::build::<WIDTH, 23>(&[STEP.as_secs_f64() / 4., 0., 1., 0.5], |v, c| {
                    tick(v, c)
                })
            },
            None,
        )
    }
    pub fn set_target_angles(&mut self, lane: usize, angles: [f64; 2]) -> Result<(), GpuError> {
        crate::planar_arm::set_target(
            lane,
            self.committed.get_mut(lane).map(|l| &mut l.tool.arm),
            angles,
        )?;
        self.upload_needed = true;
        Ok(())
    }
    pub fn set_motor_speed(
        &mut self,
        lane: usize,
        axis: usize,
        speed: f64,
    ) -> Result<(), GpuError> {
        let tool = &mut self
            .committed
            .get_mut(lane)
            .ok_or(GpuError::InvalidArmTarget(lane))?
            .tool;
        if !speed.is_finite() || tool.velocity_mode.get(axis) != Some(&1.) {
            return Err(GpuError::InvalidArmTarget(lane));
        }
        if tool.velocity_target[axis] != speed {
            tool.velocity_target[axis] = speed;
            self.upload_needed = true;
        }
        Ok(())
    }
}
fn dot<T: Scalar>([a, b]: [T; 2], [x, y]: [T; 2]) -> T {
    a * x + b * y
}
struct ContactMass<T> {
    floor: [T; 2],
    coupling: [T; 2],
    pair: T,
}
// The two floor rows have no common moving body. Eliminating their impulses gives
// a scalar monotone piecewise-linear equation for the tool/sphere normal impulse.
// Try its four possible floor active sets. Reuse the exact same operator for position.
impl<T: Scalar> ContactMass<T> {
    fn impulses(&self, gap: [T; 3], speed: [T; 3], zero: T, one: T) -> [T; 3] {
        let f: [T; 2] = std::array::from_fn(|i| {
            let k = self.floor[i];
            gap[i]
                .at_most(zero)
                .and(k.greater(zero))
                .select(one / k.greater(zero).select(k, one), zero)
        });
        let floor = |p: T| {
            std::array::from_fn::<_, 2, _>(|i| {
                ((zero - speed[i] - self.coupling[i] * p) * f[i]).maximum(zero)
            })
        };
        let closing = gap[2]
            .at_most(zero)
            .and((speed[2] + dot(self.coupling, floor(zero))).less(zero));
        let mut pair = zero;
        for active in [[false, false], [true, false], [false, true], [true, true]] {
            let d = std::array::from_fn::<_, 2, _>(|i| {
                if active[i] {
                    self.coupling[i] * f[i]
                } else {
                    zero
                }
            });
            let k = self.pair - dot(d, self.coupling);
            let p = (dot(d, [speed[0], speed[1]]) - speed[2]) / k.greater(zero).select(k, one);
            let mut valid = closing.and(k.greater(zero)).and(p.at_least(zero));
            for i in 0..2 {
                let slack = speed[i] + self.coupling[i] * p;
                valid = valid.and(if active[i] {
                    f[i].greater(zero).and(slack.at_most(zero))
                } else {
                    f[i].at_most(zero).or(slack.at_least(zero))
                });
            }
            pair = valid.select(p, pair);
        }
        let [a, b] = floor(pair);
        [a, b, pair]
    }
}
fn tick<T: Scalar>(mut v: [T; WIDTH], constants: &[T]) -> [(usize, T); 23] {
    let [dt, zero, one, half] = *constants else {
        unreachable!()
    };
    let inertia = ArmInertia::from_lane(&v);
    let inv = one / v[67];
    let mut sums = [zero; 5];
    let mut onset = zero - one;
    let mut first = zero;
    let mut elapsed = zero;
    for _ in 0..4 {
        let old = [[v[0], v[1]], [v[2], v[3]]];
        let old_height = v[6] * v[1] + v[7] * (v[1] * v[2] + v[0] * v[3]);
        for (i, value) in tick_with_velocity(&v, constants, Some(([v[38], v[39]], [v[40], v[41]])))
        {
            v[i] = value;
        }
        v[62] = v[62] + dt * v[64];
        v[63] = v[63] + dt * (v[65] - half * dt * v[15]);
        v[65] = v[65] - dt * v[15];
        let x2 = v[7] * (v[0] * v[2] - v[1] * v[3]);
        let z2 = v[7] * (v[1] * v[2] + v[0] * v[3]);
        let x = v[6] * v[0] + x2;
        let z = v[6] * v[1] + z2;
        let dx = v[62] - x;
        let dz = v[63] - z;
        let separation = (dx * dx + dz * dz).root();
        let safe = separation.greater(zero).select(separation, one);
        let n = [separation.greater(zero).select(dx / safe, one), dz / safe];
        let j_floor = [x, x2];
        // Both sphere normals pass through their centers; only translation matters.
        let j_pair = [z * n[0] - x * n[1], z2 * n[0] - x2 * n[1]];
        let af = inertia.inverse(v[2], v[3], j_floor);
        let ap = inertia.inverse(v[2], v[3], j_pair);
        let mass = ContactMass {
            floor: [dot(j_floor, af), inv],
            coupling: [dot(j_floor, ap), inv * n[1]],
            pair: dot(j_pair, ap) + inv,
        };
        let gap = [
            z - v[29] - v[28],
            v[63] - v[66] - v[28],
            separation - v[29] - v[66],
        ];
        let w = [v[4], v[5]];
        let velocity = [
            dot(j_floor, w),
            v[65],
            dot(j_pair, w) + dot(n, [v[64], v[65]]),
        ];
        let [a, b, p] = mass.impulses(gap, velocity, zero, one);
        let [ca, cb, cp] = mass.impulses(gap, gap, zero, one);
        for i in 0..2 {
            v[4 + i] = v[4 + i] + af[i] * a + ap[i] * p;
            let angle = [v[2 * i], v[2 * i + 1]];
            let corrected = rotate_by(angle[0], angle[1], half * (af[i] * ca + ap[i] * cp), one);
            v[36 + i] = v[36 + i]
                + crate::planar_tool::crossing(old[i], angle, zero, one)
                + crate::planar_tool::crossing(angle, corrected, zero, one);
            [v[2 * i], v[2 * i + 1]] = corrected;
            v[62 + i] = v[62 + i] + inv * n[i] * cp;
            v[64 + i] = v[64 + i] + inv * n[i] * p;
            sums[2 + i] = sums[2 + i] + n[i] * p;
        }
        v[63] = v[63] + inv * cb;
        v[65] = v[65] + inv * b;
        let old_gap = old_height - v[29] - v[28];
        let travel = old_gap - gap[0];
        let fraction = (old_gap / travel.greater(zero).select(travel, one))
            .maximum(zero)
            .minimum(one);
        let loaded = onset.less(zero).and(a.greater(zero));
        onset = loaded.select(elapsed + dt * fraction, onset);
        first = loaded.select(a, first);
        sums[0] = sums[0] + a;
        sums[1] = sums[1] + p;
        sums[4] = sums[4] + b;
        elapsed = elapsed + dt;
    }
    [
        (0, v[0]),
        (1, v[1]),
        (2, v[2]),
        (3, v[3]),
        (4, v[4]),
        (5, v[5]),
        (26, v[26]),
        (27, v[27]),
        (31, sums[0]),
        (33, onset),
        (34, first),
        (36, v[36]),
        (37, v[37]),
        (62, v[62]),
        (63, v[63]),
        (64, v[64]),
        (65, v[65]),
        (68, sums[1]),
        (69, sums[2]),
        (70, sums[3]),
        (71, sums[4]),
        (32, zero),
        (35, zero),
    ]
}

#[cfg(test)]
#[path = "../tests/private/planar_object.rs"]
mod tests;
