// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Continuous finite-sphere impacts across a resident observation interval.
use crate::{
    GpuError, MAX_BLOCK_TICKS, ResidentBatch, SpherePairLane, SphereRotation, StepMetrics,
    kernel::build_loop, pair_rotation::tick_with_rotation, sphere_pair::PARAMETERS,
};
use bytemuck::{Pod, Zeroable};
use naga::{BinaryOperator as Bin, MathFunction as Math, Module};

/// State at the last physical tick, plus reductions over this observation alone.
/// The world-frame impulse acts on B. A receives its opposite.
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
#[repr(C)]
pub struct SpherePairBlockObservation {
    pub state: SpherePairLane,
    pub impulse_on_b: [f64; 3],
    pub normal_impulse_sum: f64,
    pub maximum_normal_impulse: f64,
    count: f64,
    first_time: f64,
    last_time: f64,
    invalid_at: f64,
}
impl SpherePairBlockObservation {
    /// Number of internal ticks that applied a positive normal impulse.
    pub fn contact_ticks(&self) -> u32 {
        self.count as u32
    }
    /// First and last continuous times of impact, in seconds from block start.
    pub fn contact_times(&self) -> Option<(f64, f64)> {
        (self.count > 0.).then_some((self.first_time, self.last_time))
    }
}
pub type SpherePairBlockBatch = ResidentBatch<SpherePairBlockObservation>;
impl SpherePairBlockBatch {
    pub fn new(adapter_name: &str, initial: Vec<SpherePairLane>) -> Result<Self, GpuError> {
        Self::with_rotation(adapter_name, initial, SphereRotation::Midpoint)
    }
    /// Choose the attitude integrator for every internal tick in this block.
    pub fn with_rotation(
        adapter_name: &str,
        initial: Vec<SpherePairLane>,
        rotation: SphereRotation,
    ) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter_name,
            initial
                .into_iter()
                .map(|state| SpherePairBlockObservation {
                    state,
                    ..SpherePairBlockObservation::zeroed()
                })
                .collect(),
            |v| v.state.initial_valid(),
            match rotation {
                SphereRotation::Midpoint => module,
                SphereRotation::FourthOrder => fourth_module,
            },
            Some(crate::BlockSetup {
                kernel: crate::ObservationKernel::Pair(rotation),
                invalid_at: |v| (v.invalid_at != 0.).then_some(v.invalid_at as u32),
            }),
        )
    }

    /// Advance a bounded number of 1 ms ticks with one device submission.
    pub fn step_block(&mut self, ticks: u32) -> Result<StepMetrics, GpuError> {
        if !(1..=MAX_BLOCK_TICKS).contains(&ticks) {
            return Err(GpuError::BlockLength(ticks));
        }
        self.advance(ticks)
    }
}
const WIDTH: usize = 52;
const _: () = assert!(std::mem::size_of::<SpherePairBlockObservation>() == WIDTH * 8);
pub(super) fn module() -> Module {
    build_module(SphereRotation::Midpoint)
}
fn fourth_module() -> Module {
    build_module(SphereRotation::FourthOrder)
}
fn build_module(rotation: SphereRotation) -> Module {
    build_loop::<WIDTH, 43>(PARAMETERS, |state, constants, offset| {
        let motion = tick_with_rotation(&state, constants, rotation);
        let [h, zero, one, ..] = *constants else {
            unreachable!()
        };
        let first = offset.binary(Bin::Equal, one);
        let prev = |slot: usize| first.choose(zero, state[slot]);
        let normal = motion[26].1;
        let contact = normal.binary(Bin::Greater, zero);
        let count = prev(48);
        let first_contact = contact.binary(Bin::LogicalAnd, count.binary(Bin::Equal, zero));
        let time = (offset - one) * h + motion[33].1;
        let impulse = |i: usize| prev(43 + i) + (motion[30 + i].1 * normal + motion[27 + i].1);
        let extra = [
            (43, impulse(0)),
            (44, impulse(1)),
            (45, impulse(2)),
            (46, prev(46) + normal),
            (47, prev(47).math(Math::Max, Some(normal))),
            (48, count + contact.choose(one, zero)),
            (49, first_contact.choose(time, prev(49))),
            (50, contact.choose(time, prev(50))),
        ];
        let invalid = motion
            .iter()
            .chain(&extra)
            .map(|(_, x)| x.nonfinite())
            .reduce(|a, b| a.binary(Bin::LogicalOr, b))
            .unwrap();
        let bad = prev(51);
        let record = invalid.binary(Bin::LogicalAnd, bad.binary(Bin::Equal, zero));
        std::array::from_fn(|index| match index {
            0..34 => motion[index],
            34..42 => extra[index - 34],
            _ => (51, record.choose(offset, bad)),
        })
    })
}

mod checkpoint;
pub use checkpoint::SpherePairBlockCheckpoint;
