// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::time::Duration;
#[derive(Default)]
pub struct Times(Vec<Duration>);
impl Times {
    pub fn add(&mut self, x: Duration) {
        self.0.push(x);
    }
    pub fn total(&self) -> f64 {
        self.0.iter().sum::<Duration>().as_secs_f64()
    }
    pub fn report(&mut self, name: &str) {
        if self.0.is_empty() {
            println!("{name}: unavailable");
            return;
        }
        self.0.sort_unstable();
        let quantile = |n: usize| self.0[(self.0.len() * n).div_ceil(100) - 1].as_secs_f64() * 1e6;
        println!(
            "{name}: mean {:.1} us, median {:.1} us, p99 {:.1} us",
            self.total() / self.0.len() as f64 * 1e6,
            quantile(50),
            quantile(99)
        );
    }
}
