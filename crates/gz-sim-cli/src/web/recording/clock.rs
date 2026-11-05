// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Playback-only clock. No simulation or physics work is performed here.
use std::{
    io,
    time::{Duration, Instant},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Stamp {
    pub epoch: u64,
    pub sequence: u64,
    pub seconds: u64,
    pub nanoseconds: u32,
}

impl Stamp {
    fn time(self) -> Duration {
        Duration::new(self.seconds, self.nanoseconds)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Credit {
    pub generation: u64,
    pub frame: usize,
}

#[derive(Clone)]
pub(super) struct Clock {
    stamps: Vec<Stamp>,
    frame: usize,
    generation: u64,
    epoch: u64,
    playing: bool,
    acknowledged: bool,
    since: Instant,
    elapsed: Duration,
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

impl Clock {
    pub fn new(stamps: Vec<Stamp>, now: Instant) -> io::Result<Self> {
        if stamps.is_empty() || stamps.len() > 4096 {
            return Err(invalid("playback needs 1–4096 frames"));
        }
        if stamps[0].epoch != 0 || stamps[0].seconds != 0 || stamps[0].nanoseconds != 0 {
            return Err(invalid(
                "playback must begin at startup epoch and time zero",
            ));
        }
        if stamps.iter().any(|s| s.nanoseconds >= 1_000_000_000) {
            return Err(invalid("invalid recorded nanoseconds"));
        }
        for pair in stamps.windows(2) {
            let [a, b] = [pair[0], pair[1]];
            if b.sequence <= a.sequence
                || b.epoch < a.epoch
                || (b.epoch == a.epoch && b.time() < a.time())
                || (b.epoch != a.epoch && b.time() != Duration::ZERO)
            {
                return Err(invalid("recorded clock is inconsistent"));
            }
        }
        Ok(Self {
            stamps,
            frame: 0,
            generation: 0,
            epoch: 0,
            playing: false,
            acknowledged: false,
            since: now,
            elapsed: Duration::ZERO,
        })
    }

    pub fn frame(&self) -> usize {
        self.frame
    }
    pub fn frames(&self) -> usize {
        self.stamps.len()
    }
    pub fn epoch(&self) -> u64 {
        self.epoch
    }
    pub fn playing(&self) -> bool {
        self.playing
    }
    pub fn finished(&self) -> bool {
        self.frame + 1 == self.stamps.len()
    }
    pub fn credit(&self) -> Credit {
        Credit {
            generation: self.generation,
            frame: self.frame,
        }
    }

    /// A heartbeat for the same immutable frame may carry a new transport
    /// sequence/token; it still acknowledges this generation and ordinal.
    pub fn forget_credit(&mut self) {
        self.acknowledged = false;
    }

    pub fn accept(&mut self, credit: Credit) {
        if credit == self.credit() {
            self.acknowledged = true;
        }
    }

    pub fn play(&mut self, now: Instant) -> io::Result<()> {
        if self.finished() {
            return Err(invalid("restart finished playback before playing"));
        }
        if !self.playing {
            self.since = now;
            self.playing = true;
        }
        Ok(())
    }

    pub fn pause(&mut self, now: Instant) {
        if self.playing {
            self.elapsed = self
                .elapsed
                .saturating_add(now.saturating_duration_since(self.since));
            self.playing = false;
        }
    }

    /// Restart preflights counters before changing any cursor state. A Ready
    /// from the previous generation can retire its old flight, never this one.
    pub fn restart(&mut self, now: Instant) -> io::Result<()> {
        let generation = self
            .generation
            .checked_add(1)
            .ok_or_else(|| invalid("playback generation exhausted"))?;
        let epoch = self
            .epoch
            .checked_add(1)
            .ok_or_else(|| invalid("playback epoch exhausted"))?;
        self.generation = generation;
        self.epoch = epoch;
        self.frame = 0;
        self.playing = false;
        self.acknowledged = false;
        self.since = now;
        self.elapsed = Duration::ZERO;
        Ok(())
    }

    /// Move at most one ordinal per accepted presentation, even for same-time
    /// mutations. Slow consumers stretch wall time; no frame is skipped.
    pub fn advance(&mut self, now: Instant) -> io::Result<bool> {
        if !self.playing || !self.acknowledged || self.finished() {
            return Ok(false);
        }
        let current = self.stamps[self.frame];
        let next = self.stamps[self.frame + 1];
        let delay = if current.epoch == next.epoch {
            next.time() - current.time()
        } else {
            Duration::ZERO
        };
        if self
            .elapsed
            .saturating_add(now.saturating_duration_since(self.since))
            < delay
        {
            return Ok(false);
        }
        let epoch = if current.epoch == next.epoch {
            self.epoch
        } else {
            self.epoch
                .checked_add(1)
                .ok_or_else(|| invalid("playback epoch exhausted"))?
        };
        self.epoch = epoch;
        self.frame += 1;
        self.playing = !self.finished();
        self.acknowledged = false;
        self.since = now;
        self.elapsed = Duration::ZERO;
        Ok(true)
    }
}

#[cfg(test)]
#[path = "clock_tests.rs"]
mod tests;
