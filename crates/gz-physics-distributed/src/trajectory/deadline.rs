// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Wall-clock admission for shortening a warm, repeatable worker request. Bootstrap,
//! material transfers and the replacement attempt retain their normal transport limits.
use super::*;
#[derive(Default)]
pub(super) struct WarmBudget {
    floor: Option<Duration>,
    peak: Duration,
    samples: u8,
}
impl WarmBudget {
    pub fn configure(&mut self, floor: Option<Duration>) -> Result<()> {
        if floor.is_some_and(|d| !(Duration::from_millis(10)..=Duration::from_secs(6)).contains(&d))
        {
            return Err(invalid(
                "warm worker budget must be between 10 and 6000 milliseconds",
            ));
        }
        self.floor = floor;
        self.clear();
        Ok(())
    }
    pub fn clear(&mut self) {
        self.peak = Duration::ZERO;
        self.samples = 0;
    }
    pub fn limit(&self) -> Option<Duration> {
        if self.samples < 4 {
            return None;
        }
        let d = self.floor?.max(
            self.peak
                .saturating_mul(4)
                .saturating_add(Duration::from_millis(2)),
        );
        (d < Duration::from_secs(6)).then_some(d)
    }
    pub fn observe(&mut self, d: Duration) {
        if self.floor.is_none() {
            return;
        }
        self.samples = self.samples.saturating_add(1);
        // Keep isolated expensive work in the upper estimate, then decay during cheaper steps.
        self.peak = d.max(self.peak - self.peak / 16);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cold_samples_tail_spike_and_reset_preserve_the_ordinary_fallback() {
        let mut b = WarmBudget::default();
        for _ in 0..8 {
            b.observe(Duration::from_millis(3));
        }
        assert!(b.limit().is_none());
        assert!(b.configure(Some(Duration::from_millis(9))).is_err());
        b.configure(Some(Duration::from_millis(80))).unwrap();
        for _ in 0..3 {
            b.observe(Duration::from_millis(5));
        }
        assert!(b.limit().is_none());
        b.observe(Duration::from_millis(5));
        assert_eq!(b.limit(), Some(Duration::from_millis(80)));
        b.observe(Duration::from_millis(95));
        assert_eq!(b.limit(), Some(Duration::from_millis(382)));
        b.observe(Duration::from_secs(3));
        assert!(b.limit().is_none());
        for _ in 0..120 {
            b.observe(Duration::from_millis(3));
        }
        assert_eq!(b.limit(), Some(Duration::from_millis(80)));
        b.clear();
        assert!(b.limit().is_none());
    }
}
