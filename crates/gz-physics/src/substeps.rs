// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::PhysicsError;
use std::time::Duration;

/// Positive native intervals that exactly partition a single public physics tick.
///
/// Boundary i is floor(i * tick / count) nanoseconds after the start. Doubling
/// the count retains every old boundary, including for odd tick lengths.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NativeStepGrid {
    base: Duration,
    remainder: u32,
    count: u32,
}
impl NativeStepGrid {
    pub fn new(tick: Duration, count: u32) -> Result<Self, PhysicsError> {
        if count == 0 || tick.as_nanos() < u128::from(count) {
            return Err(PhysicsError::InvalidValue);
        }
        let n = u128::from(count);
        let ns = tick.as_nanos() / n;
        Ok(Self {
            base: Duration::new((ns / 1_000_000_000) as u64, (ns % 1_000_000_000) as u32),
            remainder: (tick.as_nanos() % n) as u32,
            count,
        })
    }

    fn at(self, i: u32) -> Duration {
        if self.remainder == 0 {
            return self.base;
        }
        let r = u64::from(self.remainder);
        let n = u64::from(self.count);
        let before = u64::from(i) * r;
        self.base + Duration::from_nanos((before + r) / n - before / n)
    }

    /// Duration of the zero-based native interval.
    pub fn interval(self, index: u32) -> Option<Duration> {
        (index < self.count).then(|| self.at(index))
    }
    pub fn intervals(self) -> impl ExactSizeIterator<Item = Duration> {
        (0..self.count).map(move |i| self.at(i))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn refinement_preserves_every_boundary_and_the_public_clock() {
        for tick in [
            Duration::from_nanos(513),
            Duration::from_micros(500),
            Duration::new(3, 27),
            Duration::MAX,
        ] {
            for count in [1, 2, 3, 5, 8, 16, 32, 64, 128, 256] {
                let g = NativeStepGrid::new(tick, count).unwrap();
                let fine = NativeStepGrid::new(tick, count * 2)
                    .unwrap()
                    .intervals()
                    .collect::<Vec<_>>();
                let mut total = Duration::ZERO;
                for (i, h) in g.intervals().enumerate() {
                    assert!(!h.is_zero());
                    assert_eq!(h, fine[2 * i] + fine[2 * i + 1]);
                    assert_eq!(Some(h), g.interval(i as u32));
                    total += h;
                }
                assert_eq!(total, tick);
                assert!(g.interval(count).is_none());
            }
        }
    }
    #[test]
    fn refuses_zero_intervals_before_stepping() {
        assert!(NativeStepGrid::new(Duration::ZERO, 1).is_err());
        assert!(NativeStepGrid::new(Duration::from_nanos(1), 0).is_err());
        assert!(NativeStepGrid::new(Duration::from_nanos(3), 4).is_err());
    }
}
