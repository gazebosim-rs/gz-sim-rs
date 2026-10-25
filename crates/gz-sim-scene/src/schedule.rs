// Copyright (C) 2017, 2019, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::time::Duration;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub(super) enum ScheduleError {
    #[error("sensor rate has no finite representable sampling period")]
    InvalidRate,
    #[error("sensor sampling time overflow")]
    TimeOverflow,
}

#[derive(Debug, Clone)]
pub(super) struct SampleSchedule {
    period: Option<Duration>,
    next: Duration,
}

impl SampleSchedule {
    pub(super) fn reset(&mut self) {
        self.next = Duration::ZERO;
    }

    pub(super) fn new(rate_hz: f64) -> Result<Self, ScheduleError> {
        if !rate_hz.is_finite() {
            return Err(ScheduleError::InvalidRate);
        }
        let period = if rate_hz <= 0.0 {
            None
        } else {
            let nanoseconds = (1.0 / rate_hz) * 1_000_000_000.0;
            // Native Linux steady_clock casts double seconds to signed ns.
            if !nanoseconds.is_finite() || nanoseconds >= i64::MAX as f64 {
                return Err(ScheduleError::InvalidRate);
            }
            Some(Duration::from_nanos((nanoseconds as u64).max(1)))
        };
        Ok(Self {
            period,
            next: Duration::ZERO,
        })
    }

    pub(super) fn is_due(&self, now: Duration) -> bool {
        self.period.is_none() || now >= self.next
    }

    pub(super) fn take_sample(
        &mut self,
        now: Duration,
        forced: bool,
    ) -> Result<bool, ScheduleError> {
        let Some(period) = self.period else {
            return Ok(true);
        };
        if forced {
            return Ok(true);
        }
        if now < self.next {
            return Ok(false);
        }
        let next = self
            .next
            .checked_add(period)
            .ok_or(ScheduleError::TimeOverflow)?;
        self.next = if next <= now {
            // Preserve native phase without multiplying a possibly huge tick count.
            let remainder = (now - next).as_nanos() % period.as_nanos();
            now.checked_add(period - Duration::from_nanos(remainder as u64))
                .ok_or(ScheduleError::TimeOverflow)?
        } else {
            next
        };
        Ok(true)
    }
}

#[cfg(test)]
#[path = "../tests/private/schedule.rs"]
mod tests;
