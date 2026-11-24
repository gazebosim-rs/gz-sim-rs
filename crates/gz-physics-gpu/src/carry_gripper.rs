// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A planar four-prismatic-axis gripper and a receiving shelf.
use crate::{
    GpuError, ParallelGripperLane, ResidentBatch, STEP,
    arithmetic::Scalar,
    kernel,
    parallel_gripper::{Contacts, Row, bounded, box_gap},
};
use bytemuck::{Pod, Zeroable};
mod box_manifold;
mod contact_step;
mod flight;
use bevy_ecs::prelude::Resource;
use box_manifold::Rectangle;
use contact_step::{Loads, Point, Step};
use flight::BoxFlight;
pub use flight::{BoxFlightMaterial, BoxLinkState, FreeBoxState, ShelfImpact, SpatialShelf};
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct CarryGripperLane {
    /// The finger centers are world coordinates. Their targets and stops use the track's initial frame.
    pub hand: ParallelGripperLane,
    /// Horizontal track coordinate and speed.
    pub track: f64,
    pub track_speed: f64,
    /// Mass of the rail slider alone. The vertical carriage follows it horizontally.
    pub track_mass: f64,
    pub track_origin: f64,
    pub track_target: f64,
    pub track_stiffness: f64,
    pub track_damping: f64,
    pub track_effort: f64,
    pub track_lower: f64,
    pub track_upper: f64,
    pub shelf_center: [f64; 2],
    pub shelf_half: [f64; 2],
    pub shelf_friction: f64,
    pub shelf_normal: f64,
    pub shelf_tangent: f64,
    pub shelf_up: f64,
    /// Geometry origin to mass center in the X-Z plane at zero spin angle.
    pub com_offset: [f64; 2],
    /// Moment about the free body's transverse principal axis through its mass center.
    pub angular_inertia: f64,
    /// Finite box half-extents (X,Z), or both zero for the existing sphere.
    pub box_half: [f64; 2],
    /// Box's authored angle from +X toward +Z at zero reported spin angle.
    pub box_heading: f64,
    /// Last public tick's normal impulses at both manifold points, grouped by
    /// left pad, right pad, floor and receiving shelf. Spheres have one witness.
    pub point_impulse: [[f64; 2]; 4],
}
const WIDTH: usize = 84;
const _: () = assert!(size_of::<CarryGripperLane>() == WIDTH * 8);
#[derive(Resource)]
pub struct CarryGripperBatch {
    planar: ResidentBatch<CarryGripperLane>,
    flight: Option<BoxFlight>,
}
impl std::ops::Deref for CarryGripperBatch {
    type Target = ResidentBatch<CarryGripperLane>;
    fn deref(&self) -> &Self::Target {
        &self.planar
    }
}
impl CarryGripperLane {
    /// Current world-space horizontal and vertical reaches from the geometric center.
    pub fn payload_reach(&self) -> [f64; 2] {
        if self.box_half == [0.; 2] {
            return [self.hand.radius; 2];
        }
        let (s, c) = (self.box_heading + self.hand.angle).sin_cos();
        [
            self.box_half[0] * c.abs() + self.box_half[1] * s.abs(),
            self.box_half[0] * s.abs() + self.box_half[1] * c.abs(),
        ]
    }
    /// Bound vertical material speed over the support footprint while a box tips.
    pub fn settling_speed(&self) -> f64 {
        let extent = if self.box_half == [0.; 2] {
            0.
        } else {
            self.payload_reach()[0]
        };
        self.hand.velocity[1].abs() + self.hand.spin.abs() * extent
    }
    pub fn valid(self) -> bool {
        let h = self.hand;
        let top = self.shelf_center[1] + self.shelf_half[1];
        let shape_ok = if self.box_half == [0.; 2] {
            let gap = box_gap(
                [
                    h.ball[0] - self.shelf_center[0],
                    h.ball[1] - self.shelf_center[1],
                ],
                self.shelf_half,
                h.radius,
                0.,
                1.,
            )
            .0;
            h.valid()
                && self.box_heading == 0.
                && gap.is_finite()
                && gap >= -1e-12
                && self.angular_inertia / h.ball_mass
                    + self.com_offset[0].powi(2)
                    + self.com_offset[1].powi(2)
                    <= h.radius * h.radius
        } else {
            let (si, co) = (self.box_heading + h.angle).sin_cos();
            let b = Rectangle {
                center: h.ball,
                x_axis: [co, si],
                half: self.box_half,
            };
            let gap = |center, half| {
                box_manifold::rectangles(
                    Rectangle {
                        center,
                        x_axis: [1., 0.],
                        half,
                    },
                    b,
                    0.,
                    1.,
                )
            };
            let (sin, cos) = self.box_heading.sin_cos();
            let [dx, dz] = self.com_offset;
            h.mechanism_valid()
                && h.radius == 0.
                && self
                    .box_half
                    .iter()
                    .all(|&v| v > 0. && v.recip().is_normal())
                && (dx * cos + dz * sin).abs() <= self.box_half[0]
                && (dz * cos - dx * sin).abs() <= self.box_half[1]
                && self.angular_inertia / h.ball_mass + dx * dx + dz * dz
                    <= self.box_half[0].powi(2) + self.box_half[1].powi(2)
                && (0..2)
                    .flat_map(|i| gap([h.position[i + 1], h.position[0]], h.half_size[i]))
                    .chain(gap(self.shelf_center, self.shelf_half))
                    .chain(box_manifold::floor(b, h.ground, 0., 1.))
                    .all(|w| w.gap.is_finite() && w.gap >= -1e-12)
        };
        shape_ok
            && self.point_impulse == [[0.; 2]; 4]
            && bytemuck::cast_ref::<_, [f64; WIDTH]>(&self)
                .iter()
                .all(|v| v.is_finite())
            && self.angular_inertia > 0.
            && self.angular_inertia.recip().is_normal()
            && self.track_mass > 0.
            && (self.track_mass + h.mass[0]).recip().is_normal()
            && self.track == self.track_origin
            && self.track_lower < self.track_upper
            && [self.track, self.track_target]
                .iter()
                .all(|x| (self.track_lower..=self.track_upper).contains(x))
            && self.track_stiffness >= 0.
            && self.track_damping >= 0.
            && self.track_effort > 0.
            && self.shelf_half.iter().all(|&v| v > 0.)
            && (0. ..=1.).contains(&self.shelf_friction)
            && self.shelf_center[1] - self.shelf_half[1] >= h.ground - 1e-12
            && top < h.lower[0] - h.half_size[0][1].max(h.half_size[1][1])
            && self.shelf_normal == 0.
            && self.shelf_tangent == 0.
            && self.shelf_up == 0.
    }
}
impl CarryGripperBatch {
    pub fn new(adapter: &str, lanes: Vec<CarryGripperLane>) -> Result<Self, GpuError> {
        let boxes = lanes.first().is_some_and(|l| l.box_half != [0.; 2]);
        if let Some(i) = lanes.iter().position(|l| (l.box_half != [0.; 2]) != boxes) {
            return Err(GpuError::InvalidLane(i));
        }
        let eccentric = lanes.iter().any(|l| l.com_offset != [0.; 2]);
        let planar = ResidentBatch::new_profile(
            adapter,
            lanes,
            CarryGripperLane::valid,
            match (boxes, eccentric) {
                (true, true) => module::<true, true>,
                (true, false) => module::<true, false>,
                (false, true) => module::<false, true>,
                (false, false) => module::<false, false>,
            },
            None,
        )?;
        Ok(Self {
            planar,
            flight: None,
        })
    }
    /// Absolute track and lift coordinates, followed by finger targets in the initial rail frame.
    pub fn set_targets(&mut self, index: usize, target: [f64; 4]) -> Result<(), GpuError> {
        let l = self
            .planar
            .committed
            .get_mut(index)
            .ok_or(GpuError::InvalidArmTarget(index))?;
        if !(l.track_lower..=l.track_upper).contains(&target[0])
            || !(0..3).all(|j| (l.hand.lower[j]..=l.hand.upper[j]).contains(&target[j + 1]))
        {
            return Err(GpuError::InvalidArmTarget(index));
        }
        [
            l.track_target,
            l.hand.target[0],
            l.hand.target[1],
            l.hand.target[2],
        ] = target;
        self.planar.upload_needed = true;
        Ok(())
    }
}
/// Each finger slides relative to a dynamic track. Equal and opposite stop impulses
/// act on finger and track. The vertical lift and world-rooted track have absolute stops.
fn stops<const POSITION: bool, T: Scalar>(
    lim: [[T; 2]; 4],
    inv: Row<T, 7>,
    z: T,
) -> impl FnMut(&mut Row<T, 7>) {
    let mut applied = [z; 4];
    move |speed| {
        for (i, dof) in [6, 0, 1, 2].into_iter().enumerate() {
            let relative = i >= 2;
            let k = inv[dof] + if relative { inv[6] } else { z };
            let free = speed[dof] - if relative { speed[6] } else { z } - k * applied[i];
            let q = (bounded::<POSITION, T>(free, lim[i], z) - free) / k;
            let change = q - applied[i];
            speed[dof] = speed[dof] + inv[dof] * change;
            if relative {
                speed[6] = speed[6] - inv[6] * change;
            }
            applied[i] = q;
        }
    }
}
fn constants() -> [f64; 11] {
    [
        STEP.as_secs_f64() / 4.,
        0.,
        1.,
        0.5,
        std::f64::consts::TAU,
        -1. / 6.,
        1. / 120.,
        -1. / 5040.,
        1. / 362880.,
        -1. / 39916800.,
        1. / 6227020800.,
    ]
}
fn module<const BOX: bool, const ECCENTRIC: bool>() -> naga::Module {
    kernel::build(&constants(), |v, c| tick::<BOX, ECCENTRIC, _>(v, c))
}
/// Vulkan's extended sine and cosine operations are single precision. Reduce to
/// ±π/8, evaluate sine to degree 13, then double the angle three times in f64.
fn rotated_offset<T: Scalar>(angle: T, offset: [T; 2], c: &[T]) -> [T; 2] {
    let half = c[3];
    let a = (angle - c[4] * (angle / c[4] + half).floor()) * half * half * half;
    let square = a * a;
    let mut sinc = c[10];
    for &v in c[5..10].iter().rev() {
        sinc = v + square * sinc;
    }
    let mut s = a * (c[2] + square * sinc);
    let mut cos = (c[2] - s * s).maximum(c[1]).root();
    for _ in 0..3 {
        (s, cos) = ((s + s) * cos, cos * cos - s * s);
    }
    [
        cos * offset[0] - s * offset[1],
        s * offset[0] + cos * offset[1],
    ]
}
fn tick<const BOX: bool, const ECCENTRIC: bool, T: Scalar>(
    v: [T; WIDTH],
    constants: &[T],
) -> [(usize, T); 33] {
    integrate::<BOX, ECCENTRIC, false, T>(v, constants).0
}
fn integrate<const BOX: bool, const ECCENTRIC: bool, const EMPTY: bool, T: Scalar>(
    mut v: [T; WIDTH],
    constants: &[T],
) -> ([(usize, T); 33], [[T; 2]; 3]) {
    let mut swept = std::array::from_fn(|i| [v[i]; 2]);
    let [dt, z, one, half, ..] = *constants else {
        unreachable!()
    };
    let total = v[6] + v[7] + v[8];
    let inv = [
        one / total,
        one / v[7],
        one / v[8],
        one / v[20],
        one / v[20],
        one / v[72],
        one / (v[6] + v[54]),
    ];
    let initial_offset = [v[70], v[71]];
    let offset = |angle| {
        if ECCENTRIC {
            rotated_offset(angle, initial_offset, constants)
        } else {
            [z; 2]
        }
    };
    let mut center_offset = offset(v[17]);
    // Integrate impulses and free motion at the COM. Published pose and twist
    // remain the geometric link origin, the point used by the collision shape.
    if ECCENTRIC {
        v[13] = v[13] + center_offset[0];
        v[14] = v[14] + center_offset[1];
        v[15] = v[15] - v[18] * center_offset[1];
        v[16] = v[16] + v[18] * center_offset[0];
    }
    let mut loads = Loads {
        normal: [z; 4],
        tangent: [z; 4],
        up: [z; 4],
        points: [[z; 2]; 4],
    };
    for _ in 0..4 {
        let dx = v[52] - v[55];
        let force: [T; 3] = std::array::from_fn(|i| {
            let x = if i == 0 { z } else { dx };
            let speed = if i == 0 { z } else { v[53] };
            let weight = if i == 0 { total * v[21] } else { z };
            (v[26 + i] * (v[23 + i] + x - v[i]) - v[29 + i] * (v[3 + i] - speed) + weight)
                .maximum(z - v[32 + i])
                .minimum(v[32 + i])
                - weight
        });
        let drive = (v[57] * (v[56] - v[52]) - v[58] * v[53])
            .maximum(z - v[59])
            .minimum(v[59]);
        let rail_dv = dt * inv[6] * (drive - force[1] - force[2]);
        v[52] = v[52] + dt * (v[53] + half * rail_dv);
        v[53] = v[53] + rail_dv;
        for i in 0..3 {
            let dv = dt * inv[i] * force[i];
            if EMPTY {
                let speed = v[3 + i];
                let reach = dt * (speed.maximum(z - speed) + half * dv.maximum(z - dv));
                swept[i][0] = swept[i][0].minimum(v[i] - reach);
                swept[i][1] = swept[i][1].maximum(v[i] + reach);
            }
            v[i] = v[i] + dt * (v[3 + i] + half * dv);
            v[3 + i] = v[3 + i] + dv;
        }
        v[13] = v[13] + dt * v[15];
        v[14] = v[14] + dt * (v[16] - half * dt * v[21]);
        v[16] = v[16] - dt * v[21];
        v[17] = v[17] + dt * v[18];
        center_offset = offset(v[17]);
        let x = v[13] - center_offset[0];
        let height = v[14] - center_offset[1];
        let disp = v[52] - v[55];
        let limits = [
            [v[52] - v[60], v[61] - v[52]],
            [v[0] - v[46], v[49] - v[0]],
            [v[1] - disp - v[47], v[50] - v[1] + disp],
            [v[2] - disp - v[48], v[51] - v[2] + disp],
        ];
        let mu = [v[35], v[36], v[37], v[66]];
        let speeds = [v[3], v[4], v[5], v[15], v[16], v[18], v[53]];
        let step = Step {
            limits,
            inv,
            speed: speeds,
            mu,
            zero: z,
            one,
        };
        let (next, delta) = if EMPTY {
            step.solve([], &mut loads)
        } else if BOX {
            let payload = Rectangle {
                center: [x, height],
                half: [v[73], v[74]],
                x_axis: rotated_offset(v[17] + v[75], [one, z], constants),
            };
            let pad = |center, half| {
                box_manifold::rectangles(
                    Rectangle {
                        center,
                        half,
                        x_axis: [one, z],
                    },
                    payload,
                    z,
                    one,
                )
            };
            let groups = [
                pad([v[1], v[0]], [v[9], v[10]]),
                pad([v[2], v[0]], [v[11], v[12]]),
                box_manifold::floor(payload, v[22], z, one),
                pad([v[62], v[63]], [v[64], v[65]]),
            ];
            step.solve(
                std::array::from_fn::<_, 8, _>(|j| {
                    let w = groups[j / 2][j % 2];
                    let [nx, nz] = w.normal;
                    let rx = w.point[0] - v[13];
                    let rz = w.point[1] - v[14];
                    Point {
                        support: j / 2,
                        normal: w.normal,
                        gap: w.gap,
                        spin_normal: rx * nz - rz * nx,
                        spin_tangent: z - rx * nx - rz * nz,
                    }
                }),
                &mut loads,
            )
        } else {
            let pads = std::array::from_fn::<_, 2, _>(|i| {
                box_gap(
                    [x - v[1 + i], height - v[0]],
                    [v[9 + 2 * i], v[10 + 2 * i]],
                    v[19],
                    z,
                    one,
                )
            });
            let shelf = box_gap([x - v[62], height - v[63]], [v[64], v[65]], v[19], z, one);
            let normals = [pads[0].1, pads[1].1, [z, one], shelf.1];
            let gaps = [pads[0].0, pads[1].0, height - v[19] - v[22], shelf.0];
            let [dx, dz] = center_offset;
            step.solve(
                std::array::from_fn::<_, 4, _>(|i| {
                    let [nx, nz] = normals[i];
                    Point {
                        support: i,
                        normal: normals[i],
                        gap: gaps[i],
                        spin_normal: dz * nx - dx * nz,
                        spin_tangent: v[19] + dx * nx + dz * nz,
                    }
                }),
                &mut loads,
            )
        };
        [v[3], v[4], v[5], v[15], v[16], v[18], v[53]] = next;
        for (i, j) in [(0, 0), (1, 1), (2, 2), (3, 13), (4, 14), (6, 52)] {
            v[j] = v[j] + delta[i];
        }
        if EMPTY {
            for i in 0..3 {
                swept[i][0] = swept[i][0].minimum(v[i]);
                swept[i][1] = swept[i][1].maximum(v[i]);
            }
        }
        if ECCENTRIC || BOX {
            v[17] = v[17] + delta[5];
        }
    }
    let Loads {
        normal,
        tangent,
        up,
        points,
    } = loads;
    if ECCENTRIC {
        let [dx, dz] = offset(v[17]);
        v[13] = v[13] - dx;
        v[14] = v[14] - dz;
        v[15] = v[15] + v[18] * dz;
        v[16] = v[16] - v[18] * dx;
    }
    (
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
            (44, up[0]),
            (45, up[1]),
            (52, v[52]),
            (53, v[53]),
            (67, normal[3]),
            (68, tangent[3]),
            (69, up[3]),
            (76, points[0][0]),
            (77, points[0][1]),
            (78, points[1][0]),
            (79, points[1][1]),
            (80, points[2][0]),
            (81, points[2][1]),
            (82, points[3][0]),
            (83, points[3][1]),
        ],
        swept,
    )
}

#[cfg(test)]
#[path = "../tests/private/eccentric_gripper.rs"]
mod tests;
