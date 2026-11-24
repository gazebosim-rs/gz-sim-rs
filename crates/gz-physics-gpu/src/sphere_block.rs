// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Multi-tick sphere observations with one device submission and one readback.
use crate::{
    GpuError, ResidentBatch, SpherePlaneLane, StepMetrics,
    kernel::build_loop,
    sphere_plane::{PARAMETERS, tick},
};
use bytemuck::{Pod, Zeroable};
use naga::{BinaryOperator as Bin, MathFunction as Math, Module};

/// Bound per-invocation work while allowing up to 4.096 s between host observations.
pub const MAX_BLOCK_TICKS: u32 = 4096;

/// End-of-block state and reductions over exactly the most recent block.
/// Impulses act on the sphere and use world axes. Clearance is measured at
/// the initial and each committed sphere center relative to the plane surface.
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
#[repr(C)]
pub struct SpherePlaneBlockObservation {
    pub state: SpherePlaneLane,           // slots 0..27
    pub normal_impulse_sum: f64,          // 27
    pub tangential_impulse_sum: [f64; 3], // 28..31
    pub minimum_clearance: f64,           // 31
    pub maximum_normal_impulse: f64,      // 32
    count: f64,
    first: f64,
    last: f64,       // 33..36, exact integers
    invalid_at: f64, // 36, first invalid intermediate state
}
impl SpherePlaneBlockObservation {
    fn initial(state: SpherePlaneLane) -> Self {
        let minimum_clearance = state
            .plane_normal
            .into_iter()
            .zip(state.position)
            .map(|(n, p)| n * p)
            .sum::<f64>()
            - state.plane_offset
            - state.radius;
        Self {
            state,
            minimum_clearance,
            ..Self::zeroed()
        }
    }
    /// Number of ticks with strictly positive normal impulse during this block.
    pub fn contact_ticks(&self) -> u32 {
        self.count as u32
    }
    /// One-based offsets from the beginning of the block, inclusive.
    pub fn contact_range(&self) -> Option<(u32, u32)> {
        (self.count > 0.).then_some((self.first as u32, self.last as u32))
    }
}
pub type SpherePlaneBlockBatch = ResidentBatch<SpherePlaneBlockObservation>;
impl SpherePlaneBlockBatch {
    pub fn new(adapter_name: &str, initial: Vec<SpherePlaneLane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter_name,
            initial
                .into_iter()
                .map(SpherePlaneBlockObservation::initial)
                .collect(),
            |v| v.state.initial_valid(),
            module,
            Some(crate::BlockSetup {
                kernel: crate::ObservationKernel::Plane,
                invalid_at: check,
            }),
        )
    }
    /// Advance exactly `ticks` one-millisecond ticks on the device. A refusal
    /// or failed block preserves the previous public clock and observations.
    pub fn step_block(&mut self, ticks: u32) -> Result<StepMetrics, GpuError> {
        if !(1..=MAX_BLOCK_TICKS).contains(&ticks) {
            return Err(GpuError::BlockLength(ticks));
        }
        self.advance(ticks)
    }
}
const WIDTH: usize = 37;
const _: () = assert!(std::mem::size_of::<SpherePlaneBlockObservation>() == 8 * WIDTH);
pub(super) fn module() -> Module {
    build_loop::<WIDTH, 27>(PARAMETERS, |l, c, offset| {
        let motion = tick(&l, c);
        let [_, zero, one, ..] = *c else {
            unreachable!()
        };
        let first = offset.binary(Bin::Equal, one);
        let prev = |old| first.choose(zero, old);
        let axis =
            |i: usize| l[16] * motion[i].1 + l[17] * motion[i + 1].1 + l[18] * motion[i + 2].1;
        let old_clearance = l[16] * l[0] + l[17] * l[1] + l[18] * l[2] - l[19] - l[20];
        let new_clearance = axis(0) - l[19] - l[20];
        let jn = motion[13].1;
        let contact = jn.binary(Bin::Greater, zero);
        let count = prev(l[33]);
        let first_hit = contact.binary(Bin::LogicalAnd, count.binary(Bin::Equal, zero));
        // Preserve a nonfinite intermediate even if a later tick re-enters
        // finite territory. This is a typed, pre-commit block failure.
        let mut invalid = motion[0].1.nonfinite();
        for (_, x) in &motion[1..] {
            invalid = invalid.binary(Bin::LogicalOr, x.nonfinite());
        }
        let bad = prev(l[36]);
        let record_bad = bad.binary(Bin::Equal, zero).choose(offset, bad);
        let extra = [
            (27, prev(l[27]) + jn),
            (28, prev(l[28]) + motion[14].1),
            (29, prev(l[29]) + motion[15].1),
            (30, prev(l[30]) + motion[16].1),
            (
                31,
                first
                    .choose(old_clearance, l[31])
                    .math(Math::Min, Some(new_clearance)),
            ),
            (32, prev(l[32]).math(Math::Max, Some(jn))),
            (33, count + contact.choose(one, zero)),
            (34, first_hit.choose(offset, prev(l[34]))),
            (35, contact.choose(offset, prev(l[35]))),
            (36, invalid.choose(record_bad, bad)),
        ];
        std::array::from_fn(|index| {
            if index < motion.len() {
                motion[index]
            } else {
                extra[index - motion.len()]
            }
        })
    })
}
pub(super) fn check(lane: &SpherePlaneBlockObservation) -> Option<u32> {
    (lane.invalid_at != 0.).then_some(lane.invalid_at as u32)
}
