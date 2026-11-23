// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Deterministic SplitMix64 sampling shared by the qualification sweeps.
pub struct Rng(pub u64);
impl Rng {
    pub fn number(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut v = self.0;
        v = (v ^ (v >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        v = (v ^ (v >> 27)).wrapping_mul(0x94d049bb133111eb);
        lo + (hi - lo) * (((v ^ (v >> 31)) >> 11) as f64 * (1. / (1u64 << 53) as f64))
    }
}
