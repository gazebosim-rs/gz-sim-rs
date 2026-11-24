// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional worker-local timing of native island solves at committed barriers.
use super::*;
use std::{
    sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed},
    time::Instant,
};

/// Accumulated wall-clock time in a native component's Bevy solve and readback.
/// Timings are diagnostics and never enter physical snapshots or recovery proofs.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentWork {
    pub members: Vec<u64>,
    pub elapsed_ns: u64,
}

pub(super) struct Probe {
    start: u64,
    end: u64,
    valid: AtomicBool,
    pub(super) scheduled: bool,
    rows: Vec<TimingRow>,
}

struct TimingRow {
    members: Vec<u64>,
    elapsed_ns: AtomicU64,
}

impl Probe {
    pub(super) fn measuring(&self, tick: u64) -> bool {
        self.start < tick && tick <= self.end && self.valid.load(Relaxed)
    }

    pub(super) fn record(&self, index: usize, island: &Island, start: Instant) {
        let elapsed = start.elapsed();
        if let Some(row) = self.rows.get(index)
            && island.index.keys().eq(row.members.iter())
            && let Ok(ns) = u64::try_from(elapsed.as_nanos())
            && let Some(total) = row.elapsed_ns.load(Relaxed).checked_add(ns)
        {
            // Each solver index is assigned to exactly one lane per barrier.
            // Both lanes join before the next step or any read of this window.
            row.elapsed_ns.store(total, Relaxed);
        } else {
            self.valid.store(false, Relaxed);
        }
    }
}

impl Partition {
    pub(crate) fn start_probe(&mut self, tick: u64, span: u16) -> Result<(), WorldError> {
        self.start_probe_in_mode(tick, span, false)
    }
    pub(crate) fn start_probe_in_mode(
        &mut self,
        tick: u64,
        span: u16,
        scheduled: bool,
    ) -> Result<(), WorldError> {
        let end = tick
            .checked_add(u64::from(span))
            .ok_or(WorldError::TimeOverflow)?;
        if self.committed.tick != tick || span == 0 {
            return Err(WorldError::Checkpoint);
        }
        if let Some(probe) = &self.probe
            && probe.start == tick
            && probe.end == end
        {
            return if probe.scheduled == scheduled {
                Ok(())
            } else {
                Err(WorldError::Checkpoint)
            };
        }
        self.probe = Some(Probe {
            start: tick,
            end,
            valid: AtomicBool::new(true),
            scheduled,
            rows: self
                .components
                .iter()
                .map(|c| TimingRow {
                    members: c.ids.clone(),
                    elapsed_ns: AtomicU64::new(0),
                })
                .collect(),
        });
        Ok(())
    }
    pub(crate) fn read_probe(&self, start: u64, span: u16) -> Option<Vec<ComponentWork>> {
        let probe = self.probe.as_ref()?;
        (probe.valid.load(Relaxed)
            && probe.start == start
            && probe.end == self.committed.tick
            && probe.end.checked_sub(start) == Some(u64::from(span))
            && probe
                .rows
                .iter()
                .map(|c| &c.members)
                .eq(self.components.iter().map(|c| &c.ids)))
        .then(|| {
            probe
                .rows
                .iter()
                .map(|row| ComponentWork {
                    members: row.members.clone(),
                    elapsed_ns: row.elapsed_ns.load(Relaxed),
                })
                .collect()
        })
    }
}
