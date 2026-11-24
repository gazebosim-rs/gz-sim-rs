// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A three-slider gripper with two finite parallel box pads and a free solid sphere.
use crate::arithmetic::{Condition, Scalar};
use crate::{GpuError, ResidentBatch, STEP, kernel};
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct ParallelGripperLane {
    /// Common fingertip center height, left center X and right center X; SI units.
    pub position: [f64; 3],
    pub speed: [f64; 3],
    /// Mass of carriage alone and each independently sliding finger.
    pub mass: [f64; 3],
    pub half_size: [[f64; 2]; 2],
    pub ball: [f64; 2],
    pub velocity: [f64; 2],
    pub angle: f64,
    pub spin: f64,
    pub radius: f64,
    pub ball_mass: f64,
    pub gravity: f64,
    pub ground: f64,
    pub target: [f64; 3],
    pub stiffness: [f64; 3],
    pub damping: [f64; 3],
    pub force_limit: [f64; 3],
    /// Combined Coulomb coefficients of left pad, right pad and table against sphere.
    pub friction: [f64; 3],
    pub normal_impulse: [f64; 3],
    pub tangent_impulse: [f64; 3],
    /// Vertical components of left and right pad impulses on the sphere, N s.
    pub support_impulse: [f64; 2],
    pub lower: [f64; 3],
    pub upper: [f64; 3],
}
const WIDTH: usize = 52;
const _: () = assert!(size_of::<ParallelGripperLane>() == 8 * WIDTH);
pub type ParallelGripperBatch = ResidentBatch<ParallelGripperLane>;
impl ParallelGripperLane {
    fn accepts_target(&self, target: [f64; 3]) -> bool {
        (0..3).all(|i| (self.lower[i]..=self.upper[i]).contains(&target[i]))
    }
    pub fn valid(self) -> bool {
        let finite = bytemuck::cast_ref::<_, [f64; WIDTH]>(&self)
            .iter()
            .all(|x| x.is_finite());
        finite
            && self
                .mass
                .iter()
                .chain(self.half_size.iter().flatten())
                .chain([&self.radius, &self.ball_mass])
                .all(|&x| x > 0. && x.recip().is_finite())
            && (self.mass.iter().sum::<f64>() * self.gravity).is_finite()
            && (2.5 / (self.ball_mass * self.radius * self.radius)).is_normal()
            && self.gravity >= 0.
            && self.friction.iter().all(|x| (0. ..=1.).contains(x))
            && (0..3).all(|i| {
                self.lower[i] < self.upper[i]
                    && (self.lower[i]..=self.upper[i]).contains(&self.position[i])
                    && (self.lower[i]..=self.upper[i]).contains(&self.target[i])
                    && self.stiffness[i] >= 0.
                    && self.damping[i] >= 0.
                    && self.force_limit[i] > 0.
            })
            && self.lower[0] - self.half_size[0][1].max(self.half_size[1][1]) > self.ground
            && self.upper[1] + self.half_size[0][0] < self.lower[2] - self.half_size[1][0]
            && self.ball[1] - self.ground >= self.radius - 1e-14
            && (0..2).all(|i| {
                let gap = box_gap(
                    [
                        self.ball[0] - self.position[i + 1],
                        self.ball[1] - self.position[0],
                    ],
                    self.half_size[i],
                    self.radius,
                    0.,
                    1.,
                )
                .0;
                gap.is_finite() && gap >= -1e-14
            })
            && self.normal_impulse == [0.; 3]
            && self.tangent_impulse == [0.; 3]
            && self.support_impulse == [0.; 2]
    }
}
impl ParallelGripperBatch {
    pub fn new(adapter: &str, lanes: Vec<ParallelGripperLane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter,
            lanes,
            ParallelGripperLane::valid,
            || {
                kernel::build::<WIDTH, 20>(&[STEP.as_secs_f64() / 4., 0., 1., 0.5], |v, c| {
                    tick(v, c)
                })
            },
            None,
        )
    }
    pub fn set_targets(&mut self, index: usize, target: [f64; 3]) -> Result<(), GpuError> {
        let l = self
            .committed
            .get_mut(index)
            .ok_or(GpuError::InvalidArmTarget(index))?;
        if !l.accepts_target(target) {
            return Err(GpuError::InvalidArmTarget(index));
        }
        l.target = target;
        self.upload_needed = true;
        Ok(())
    }
    /// Install displacements from each initial slider pose in physical row order.
    /// A malformed action leaves all previously accepted targets intact.
    pub fn set_displacements(&mut self, actions: &[[f64; 3]]) -> Result<(), GpuError> {
        if actions.len() != self.committed.len() {
            return Err(GpuError::GripperActionCount {
                expected: self.committed.len(),
                actual: actions.len(),
            });
        }
        let absolute = |lane: &ParallelGripperLane, delta: &[f64; 3]| {
            std::array::from_fn(|j| lane.position[j] + delta[j])
        };
        for (index, (lane, delta)) in self.initial.iter().zip(actions).enumerate() {
            if !lane.accepts_target(absolute(lane, delta)) {
                return Err(GpuError::InvalidArmTarget(index));
            }
        }
        for ((current, initial), delta) in self.committed.iter_mut().zip(&self.initial).zip(actions)
        {
            current.target = absolute(initial, delta);
        }
        self.upload_needed = true;
        Ok(())
    }
}
/// Signed distance and outward normal from a finite axis-aligned rectangle to the ball.
fn box_gap<T: Scalar>(p: [T; 2], half: [T; 2], radius: T, z: T, one: T) -> (T, [T; 2]) {
    let s = p.map(|x| x.at_least(z).select(one, z - one));
    let a = [s[0] * p[0] - half[0], s[1] * p[1] - half[1]];
    let d = a.map(|x| x.maximum(z));
    let len = (d[0] * d[0] + d[1] * d[1]).root();
    let outside = len.greater(z);
    let div = outside.select(len, one);
    let xface = a[0].at_least(a[1]);
    let n = [
        outside.select(d[0] * s[0] / div, xface.select(s[0], z)),
        outside.select(d[1] * s[1] / div, xface.select(z, s[1])),
    ];
    (len + a[0].maximum(a[1]).minimum(z) - radius, n)
}
type Row<T> = [T; 6];
fn dot<T: Scalar>(x: Row<T>, y: Row<T>) -> T {
    x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3] + x[4] * y[4] + x[5] * y[5]
}
struct Contacts<T> {
    rows: [[Row<T>; 2]; 3],
    mob: [[Row<T>; 2]; 3],
    gaps: [T; 3],
    k: [[T; 3]; 3],
}
impl<T: Scalar> Contacts<T> {
    /// Accumulated block impulses. Actuator travel limits are unilateral constraints in
    /// the same velocity iteration, so a loaded pad can push against its rail stop.
    fn project<const POSITION: bool>(
        &self,
        mut v: Row<T>,
        mu: [T; 3],
        limits: [[T; 2]; 3],
        z: T,
        one: T,
    ) -> (Row<T>, [[T; 2]; 3]) {
        let mut impulses = [[z; 2]; 3];
        let mut stops = [z; 3];
        for _ in 0..8 {
            for i in 0..3 {
                let [lo, hi] = limits[i];
                let free = v[i] - stops[i];
                let bounded = lo.at_most(z).select(
                    free.maximum(if POSITION { z - lo } else { z }),
                    hi.at_most(z)
                        .select(free.minimum(if POSITION { hi } else { z }), free),
                );
                v[i] = bounded;
                stops[i] = bounded - free;
            }
            for (i, imp) in impulses.iter_mut().enumerate() {
                let [pn, pt] = *imp;
                let [nn, nt, tt] = self.k[i];
                let vn = dot(self.rows[i][0], v) - nn * pn - nt * pt
                    + if POSITION { self.gaps[i] } else { z };
                let new = if POSITION {
                    [(z - vn).maximum(z) / nn, z]
                } else {
                    let vt = dot(self.rows[i][1], v) - nt * pn - tt * pt;
                    crate::planar_tool::impact([vn, vt], self.k[i], mu[i], z, one)
                };
                let [qn, qt] = new.map(|p| self.gaps[i].at_most(z).select(p, z));
                for (j, w) in v.iter_mut().enumerate() {
                    *w = *w + self.mob[i][0][j] * (qn - pn) + self.mob[i][1][j] * (qt - pt);
                }
                *imp = [qn, qt];
            }
        }
        (v, impulses)
    }
}
fn tick<T: Scalar>(mut v: [T; WIDTH], constants: &[T]) -> [(usize, T); 20] {
    let [dt, z, one, half] = *constants else {
        unreachable!()
    };
    let total = v[6] + v[7] + v[8];
    let inv = [
        one / total,
        one / v[7],
        one / v[8],
        one / v[20],
        one / v[20],
        (one + one + half) / (v[20] * v[19] * v[19]),
    ];
    let mut normal = [z; 3];
    let mut tangent = [z; 3];
    let mut support = [z; 2];
    for _ in 0..4 {
        for i in 0..3 {
            let weight = if i == 0 { total * v[21] } else { z };
            let force = (v[26 + i] * (v[23 + i] - v[i]) - v[29 + i] * v[3 + i] + weight)
                .maximum(z - v[32 + i])
                .minimum(v[32 + i])
                - weight;
            let dv = dt * inv[i] * force;
            v[i] = v[i] + dt * (v[3 + i] + half * dv);
            v[3 + i] = v[3 + i] + dv;
        }
        v[13] = v[13] + dt * v[15];
        v[14] = v[14] + dt * (v[16] - half * dt * v[21]);
        v[16] = v[16] - dt * v[21];
        v[17] = v[17] + dt * v[18];
        let pad = std::array::from_fn::<_, 2, _>(|i| {
            box_gap(
                [v[13] - v[i + 1], v[14] - v[0]],
                [v[9 + 2 * i], v[10 + 2 * i]],
                v[19],
                z,
                one,
            )
        });
        let normals = [pad[0].1, pad[1].1, [z, one]];
        let gaps = [pad[0].0, pad[1].0, v[14] - v[19] - v[22]];
        let rows = std::array::from_fn(|i| {
            let [x, h] = normals[i];
            let row = |x: T, h: T, rotation: T| {
                [
                    if i < 2 { z - h } else { z },
                    if i == 0 { z - x } else { z },
                    if i == 1 { z - x } else { z },
                    x,
                    h,
                    rotation,
                ]
            };
            [row(x, h, z), row(h, z - x, v[19])]
        });
        let mob = rows.map(|p| p.map(|r| std::array::from_fn(|j| r[j] * inv[j])));
        let k = std::array::from_fn(|i| {
            [
                dot(rows[i][0], mob[i][0]),
                dot(rows[i][0], mob[i][1]),
                dot(rows[i][1], mob[i][1]),
            ]
        });
        let contacts = Contacts { rows, mob, gaps, k };
        let limits = std::array::from_fn(|i| [v[i] - v[46 + i], v[49 + i] - v[i]]);
        let mu = [v[35], v[36], v[37]];
        let (speeds, imp) =
            contacts.project::<false>([v[3], v[4], v[5], v[15], v[16], v[18]], mu, limits, z, one);
        [v[3], v[4], v[5], v[15], v[16], v[18]] = speeds;
        let (dx, _) = contacts.project::<true>([z; 6], mu, limits, z, one);
        for (i, d) in [0, 1, 2, 13, 14].into_iter().zip(dx) {
            v[i] = v[i] + d;
        }
        for i in 0..3 {
            normal[i] = normal[i] + imp[i][0];
            tangent[i] = tangent[i] + imp[i][1];
            if i < 2 {
                support[i] = support[i] + normals[i][1] * imp[i][0] - normals[i][0] * imp[i][1];
            }
        }
    }
    [
        (0, v[0]),
        (1, v[1]),
        (2, v[2]),
        (3, v[3]),
        (4, v[4]),
        (5, v[5]),
        (13, v[13]),
        (14, v[14]),
        (15, v[15]),
        (16, v[16]),
        (17, v[17]),
        (18, v[18]),
        (38, normal[0]),
        (39, normal[1]),
        (40, normal[2]),
        (41, tangent[0]),
        (42, tangent[1]),
        (43, tangent[2]),
        (44, support[0]),
        (45, support[1]),
    ]
}

#[cfg(test)]
#[path = "../tests/private/parallel_gripper.rs"]
mod tests;
