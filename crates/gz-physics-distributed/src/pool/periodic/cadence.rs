// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reuse timing only while the observed contact work and execution resources are unchanged.
use super::{PeriodicPlacement, WorkerPool};
use crate::{physics::ComponentInfo, protocol::Observation};
use std::num::NonZeroU16;

fn finite(w: &Observation) -> impl Iterator<Item = (u64, u64, bool)> + '_ {
    w.snapshot
        .impulses
        .iter()
        .map(|p| (p.a, p.b, p.normal_impulse > 0.))
}
fn floors(w: &Observation) -> impl Iterator<Item = (u64, bool)> + '_ {
    w.snapshot
        .supports
        .iter()
        .map(|p| (p.body, p.normal_impulse > 0.))
}
struct WorkerPattern {
    session: String,
    islands: Vec<ComponentInfo>,
    finite_contacts: Vec<(u64, u64, bool)>,
    floor_contacts: Vec<(u64, bool)>,
}
struct LoadPattern {
    workers: Vec<WorkerPattern>,
    refined: Option<Vec<u64>>,
    helper_limit: Option<usize>,
    granted_helpers: u8,
}
impl LoadPattern {
    fn read(pool: &WorkerPool) -> Self {
        let workers = pool
            .frame
            .workers
            .iter()
            .zip(&pool.peers)
            .map(|(w, p)| WorkerPattern {
                session: p.session_id().to_owned(),
                islands: w.components.clone(),
                finite_contacts: finite(w).collect(),
                floor_contacts: floors(w).collect(),
            })
            .collect();
        Self {
            workers,
            refined: pool.last_island_ids.clone(),
            helper_limit: pool.local_helper_limit,
            granted_helpers: pool.last_helper_mask.unwrap_or(0),
        }
    }
    fn matches(&self, pool: &WorkerPool) -> bool {
        self.helper_limit == pool.local_helper_limit
            && self.granted_helpers == pool.last_helper_mask.unwrap_or(0)
            && self.refined.as_deref() == pool.last_island_ids.as_deref()
            && self
                .workers
                .iter()
                .zip(&pool.frame.workers)
                .zip(&pool.peers)
                .all(|((saved, w), p)| {
                    saved.session == p.session_id()
                        && saved.islands == w.components
                        && saved.finite_contacts.iter().copied().eq(finite(w))
                        && saved.floor_contacts.iter().copied().eq(floors(w))
                })
    }
}

pub(super) struct IdleCadence {
    observed: LoadPattern,
    periods: u16,
    next: u64,
}
impl IdleCadence {
    pub(super) fn at_start(
        previous: Option<Self>,
        config: PeriodicPlacement,
        pool: &WorkerPool,
    ) -> Option<Self> {
        if config.max_idle_periods == NonZeroU16::MIN {
            return None;
        }
        Some(
            previous
                .filter(|s| s.unchanged(pool))
                .unwrap_or_else(|| Self {
                    observed: LoadPattern::read(pool),
                    periods: 1,
                    next: pool.frame.tick,
                }),
        )
    }
    pub(super) fn after_stable_sample(mut self, config: PeriodicPlacement, since: u64) -> Self {
        self.periods = self
            .periods
            .saturating_mul(2)
            .min(config.max_idle_periods.get());
        self.next =
            since.saturating_add(u64::from(self.periods) * u64::from(config.period_ticks.get()));
        self
    }
    pub(super) fn next_sample_tick(&self) -> u64 {
        self.next
    }
    pub(super) fn unchanged(&self, pool: &WorkerPool) -> bool {
        self.observed.matches(pool)
    }
}
