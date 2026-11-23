// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stateless stochastic inputs for explicitly numbered simulation streams.
fn mix(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}
/// Key a draw to its logical environment, committed step, stream and draw
/// ordinal. The order of registration or evaluation consumes no shared state.
/// This is a deterministic SplitMix64-based counter mapping for simulation.
pub fn counter(seed: u64, id: u64, step: u64, stream: u64, draw: u64) -> u64 {
    [id, step, stream, draw]
        .into_iter()
        .fold(mix(seed), |state, field| {
            mix(state.wrapping_add(0x9e37_79b9_7f4a_7c15) ^ field)
        })
}
/// Exactly one of 2^53 equally spaced representable values in [0,1).
pub fn uniform(seed: u64, id: u64, step: u64, stream: u64, draw: u64) -> f64 {
    (counter(seed, id, step, stream, draw) >> 11) as f64 / (1_u64 << 53) as f64
}
