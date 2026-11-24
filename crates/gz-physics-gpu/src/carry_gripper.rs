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
}
const WIDTH: usize = 70;
const _: () = assert!(size_of::<CarryGripperLane>() == WIDTH * 8);
pub type CarryGripperBatch = ResidentBatch<CarryGripperLane>;
impl CarryGripperLane {
    pub fn valid(self) -> bool {
        let h = self.hand;
        let top = self.shelf_center[1] + self.shelf_half[1];
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
            && bytemuck::cast_ref::<_, [f64; WIDTH]>(&self)
                .iter()
                .all(|v| v.is_finite())
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
            && gap.is_finite()
            && gap >= -1e-12
            && self.shelf_normal == 0.
            && self.shelf_tangent == 0.
            && self.shelf_up == 0.
    }
}
impl CarryGripperBatch {
    pub fn new(adapter: &str, lanes: Vec<CarryGripperLane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter,
            lanes,
            CarryGripperLane::valid,
            || {
                kernel::build::<WIDTH, 25>(&[STEP.as_secs_f64() / 4., 0., 1., 0.5], |v, c| {
                    tick(v, c)
                })
            },
            None,
        )
    }
    /// Absolute track and lift coordinates, followed by finger targets in the initial rail frame.
    pub fn set_targets(&mut self, index: usize, target: [f64; 4]) -> Result<(), GpuError> {
        let l = self
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
        self.upload_needed = true;
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
fn tick<T: Scalar>(mut v: [T; WIDTH], constants: &[T]) -> [(usize, T); 25] {
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
        one / (v[6] + v[54]),
    ];
    let mut normal = [z; 4];
    let mut tangent = [z; 4];
    let mut up = [z; 4];
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
            v[i] = v[i] + dt * (v[3 + i] + half * dv);
            v[3 + i] = v[3 + i] + dv;
        }
        v[13] = v[13] + dt * v[15];
        v[14] = v[14] + dt * (v[16] - half * dt * v[21]);
        v[16] = v[16] - dt * v[21];
        v[17] = v[17] + dt * v[18];
        let pads = std::array::from_fn::<_, 2, _>(|i| {
            box_gap(
                [v[13] - v[1 + i], v[14] - v[0]],
                [v[9 + 2 * i], v[10 + 2 * i]],
                v[19],
                z,
                one,
            )
        });
        let shelf = box_gap(
            [v[13] - v[62], v[14] - v[63]],
            [v[64], v[65]],
            v[19],
            z,
            one,
        );
        let normals = [pads[0].1, pads[1].1, [z, one], shelf.1];
        let gaps = [pads[0].0, pads[1].0, v[14] - v[19] - v[22], shelf.0];
        let rows = std::array::from_fn(|i| {
            let [x, h] = normals[i];
            let row = |x: T, h: T, spin: T| {
                [
                    if i < 2 { z - h } else { z },
                    if i == 0 { z - x } else { z },
                    if i == 1 { z - x } else { z },
                    x,
                    h,
                    spin,
                    z,
                ]
            };
            [row(x, h, z), row(h, z - x, v[19])]
        });
        let contacts = Contacts::new(rows, inv, gaps);
        let disp = v[52] - v[55];
        let limits = [
            [v[52] - v[60], v[61] - v[52]],
            [v[0] - v[46], v[49] - v[0]],
            [v[1] - disp - v[47], v[50] - v[1] + disp],
            [v[2] - disp - v[48], v[51] - v[2] + disp],
        ];
        let mu = [v[35], v[36], v[37], v[66]];
        let speeds = [v[3], v[4], v[5], v[15], v[16], v[18], v[53]];
        let (next, imp) =
            contacts.project::<false>(speeds, mu, z, one, stops::<false, T>(limits, inv, z));
        [v[3], v[4], v[5], v[15], v[16], v[18], v[53]] = next;
        let (delta, _) =
            contacts.project::<true>([z; 7], mu, z, one, stops::<true, T>(limits, inv, z));
        for (i, j) in [(0, 0), (1, 1), (2, 2), (3, 13), (4, 14), (6, 52)] {
            v[j] = v[j] + delta[i];
        }
        for i in 0..4 {
            normal[i] = normal[i] + imp[i][0];
            tangent[i] = tangent[i] + imp[i][1];
            up[i] = up[i] + normals[i][1] * imp[i][0] - normals[i][0] * imp[i][1];
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
        (44, up[0]),
        (45, up[1]),
        (52, v[52]),
        (53, v[53]),
        (67, normal[3]),
        (68, tangent[3]),
        (69, up[3]),
    ]
}
