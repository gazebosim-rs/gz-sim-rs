// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional timing of resident islands and measured suggestions for a whole-island handoff.
use super::*;
use crate::physics::ComponentWork;
use std::{num::NonZeroU16, time::Duration};

/// Choose whether measuring native work permits the usual worker-local scheduling.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PoolLoadMode {
    Serial,
    Scheduled,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct ProbeId {
    start: u64,
    span: NonZeroU16,
    mode: PoolLoadMode,
    participants: u8,
}
impl ProbeId {
    pub(super) fn covers(self, tick: u64) -> bool {
        tick < self.start.saturating_add(u64::from(self.span.get()))
    }
}
/// Identifies the exact uninterrupted barrier range and worker incarnations being sampled.
#[derive(Clone, Debug)]
pub struct PoolLoadWindow {
    run: u64,
    id: ProbeId,
    workers: Vec<(String, u64)>,
    scheduled_worker_mask: u8,
}
impl PoolLoadWindow {
    /// Workers that acknowledged scheduled sampling. A set bit permits their
    /// ordinary helper grants; the scheduler decides whether a helper is useful.
    pub fn scheduled_worker_mask(&self) -> u8 {
        self.scheduled_worker_mask
    }
}
#[derive(Debug)]
pub struct PoolLoadSample {
    window: PoolLoadWindow,
    rows: Vec<Vec<ComponentWork>>,
}
impl PoolLoadSample {
    pub fn scheduled_worker_mask(&self) -> u8 {
        self.window.scheduled_worker_mask()
    }
    pub fn workers(&self) -> &[Vec<ComponentWork>] {
        &self.rows
    }
    pub fn measured_steps(&self) -> NonZeroU16 {
        self.window.id.span
    }
}
/// The anticipated lifetime of this placement and the cost of a single transfer.
/// Supply a handoff time measured for the applicable transport and scene.
#[derive(Copy, Clone, Debug)]
pub struct PoolBalancePolicy {
    pub horizon: NonZeroU16,
    pub handoff_cost: Duration,
}
/// Estimated native work per public tick, measured per island in the selected mode. Wall-clock transport, queueing and OS delays are outside this model.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PoolBalanceDecision {
    pub members: Vec<u64>,
    pub source: u8,
    pub destination: u8,
    pub before_lower_ns: u64,
    pub after_serial_peak_ns: u64,
    pub estimated_payback_ticks: u64,
}

impl WorkerPool {
    fn load_workers(&self) -> Result<Vec<(String, u64)>, Error> {
        if self.dirty.iter().any(|&d| d) {
            return Err(Error::Pending);
        }
        self.peers
            .iter()
            .enumerate()
            .map(|(w, p)| {
                let sequence = self.baseline[w]
                    .checked_add(self.history[w].entries.len() as u64)
                    .ok_or(Error::Capacity)?;
                Ok((p.session_id().to_owned(), sequence))
            })
            .collect()
    }
    /// Start timing the next contiguous public ticks across all workers. Active
    /// measurement makes native batches serial until the window ends. No physics
    /// command, migration or recovery record is published by starting a probe.
    pub fn start_load_sample(&mut self, span: NonZeroU16) -> Result<PoolLoadWindow, Error> {
        self.start_load_sample_with_mode(span, PoolLoadMode::Serial)
    }
    /// Scheduled timing retains ordinary lane selection on capable peers. Older
    /// peers measure serially and leave their bit clear in the returned window.
    pub fn start_load_sample_with_mode(
        &mut self,
        span: NonZeroU16,
        mode: PoolLoadMode,
    ) -> Result<PoolLoadWindow, Error> {
        self.start_load_sample_before(span, mode, crate::transport::REQUEST_TIMEOUT, u8::MAX)
    }
    fn start_load_sample_before(
        &mut self,
        span: NonZeroU16,
        mode: PoolLoadMode,
        timeout: Duration,
        participants: u8,
    ) -> Result<PoolLoadWindow, Error> {
        let start = self.frame.tick;
        start
            .checked_add(u64::from(span.get()))
            .ok_or(Error::Capacity)?;
        let workers = self.load_workers()?;
        if self
            .active_probe
            .is_some_and(|id| (id.start, id.span) == (start, span) && id.mode != mode)
        {
            return Err(Error::Invalid(
                "pool load window already uses another scheduling mode",
            ));
        }
        // Only a fully acknowledged window can authorize placement. Invalidate
        // an earlier selection if a superseding request partially reaches the pool.
        self.active_probe = None;
        let scheduled_worker_mask = WorkerProcess::start_pool_probes(
            &mut self.peers,
            start,
            span.get(),
            mode == PoolLoadMode::Scheduled,
            timeout,
            participants,
        )?;
        let id = ProbeId {
            start,
            span,
            mode,
            participants,
        };
        self.active_probe = Some(id);
        Ok(PoolLoadWindow {
            run: self.run,
            id,
            workers,
            scheduled_worker_mask,
        })
    }
    /// An empty worker contributes zero native work. The session and sequence
    /// of every worker, including each empty destination, still fence this sample.
    pub(super) fn start_resident_load_sample(
        &mut self,
        span: NonZeroU16,
        timeout: Duration,
    ) -> Result<PoolLoadWindow, Error> {
        let participants = self
            .frame
            .workers
            .iter()
            .enumerate()
            .fold(0u8, |mask, (i, w)| {
                mask | (u8::from(!w.components.is_empty()) << i)
            });
        self.start_load_sample_before(span, PoolLoadMode::Scheduled, timeout, participants)
    }
    fn load_window_is_current(&self, window: &PoolLoadWindow) -> Result<bool, Error> {
        let span = u64::from(window.id.span.get());
        if self.active_probe != Some(window.id)
            || window.run != self.run
            || window.id.start.checked_add(span) != Some(self.frame.tick)
            || window.workers.len() != self.peers.len()
            || !self.load_workers()?.into_iter().zip(&window.workers).all(
                |((session, seq), (old, start))| {
                    &session == old && start.checked_add(span) == Some(seq)
                },
            )
        {
            return Ok(false);
        }
        Ok(true)
    }
    /// A published input, edit, handoff, recovery or changed native component
    /// membership invalidates a previously opened window.
    pub fn finish_load_sample(&mut self, window: PoolLoadWindow) -> Result<PoolLoadSample, Error> {
        self.try_finish_load_sample(window, crate::transport::REQUEST_TIMEOUT)?
            .ok_or(Error::Invalid("pool load window is stale or interrupted"))
    }
    pub(super) fn try_finish_load_sample(
        &mut self,
        window: PoolLoadWindow,
        timeout: Duration,
    ) -> Result<Option<PoolLoadSample>, Error> {
        if !self.load_window_is_current(&window)? {
            return Ok(None);
        }
        let mut rows = Vec::with_capacity(self.peers.len());
        let reports = WorkerProcess::read_pool_probes(
            &mut self.peers,
            window.id.start,
            window.id.span.get(),
            timeout,
            window.id.participants,
        )?;
        for (report, state) in reports.into_iter().zip(&self.frame.workers) {
            let Some(work) = report else { return Ok(None) };
            if work
                .iter()
                .map(|c| &c.members)
                .ne(state.components.iter().map(|c| &c.ids))
            {
                return Ok(None);
            }
            rows.push(work);
        }
        Ok(Some(PoolLoadSample { rows, window }))
    }
    /// Pick at most one isolated native component at this committed barrier.
    /// The projected serial peak must beat an optimistic bound for the current
    /// owners over the caller's horizon after the supplied cost of migration.
    /// The caller can submit the returned member and destination to transfer_component.
    pub fn plan_rebalance(
        &self,
        sample: &PoolLoadSample,
        policy: PoolBalancePolicy,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<Option<PoolBalanceDecision>, Error> {
        if !self.load_window_is_current(&sample.window)? {
            return Err(Error::Invalid("pool load window is stale or interrupted"));
        }
        let plan = routing::ProspectiveGroups::new(
            &self.frame,
            gravity,
            &self.queued,
            inputs,
            self.profile,
        )?;
        self.plan_rebalance_for(sample, policy, &plan)
    }

    /// The caller has validated the sample at this exact committed barrier.
    pub(super) fn plan_rebalance_for(
        &self,
        sample: &PoolLoadSample,
        policy: PoolBalancePolicy,
        plan: &routing::ProspectiveGroups,
    ) -> Result<Option<PoolBalanceDecision>, Error> {
        // The next mandatory colocation can change the measured load and owners.
        // Let it settle before spending another transfer on optional placement.
        if self.limit < 3
            || plan
                .groups
                .iter()
                .any(|g| g.iter().any(|&i| plan.owners[i] != plan.owners[g[0]]))
        {
            return Ok(None);
        }
        let mut sums = vec![0u64; self.peers.len()];
        let mut costs = Vec::with_capacity(plan.components.len());
        for (owner, rows) in sample.rows.iter().enumerate() {
            for row in rows {
                let ns = row
                    .elapsed_ns
                    .div_ceil(u64::from(sample.measured_steps().get()));
                sums[owner] = sums[owner].checked_add(ns).ok_or(Error::Capacity)?;
                costs.push(ns);
            }
        }
        let populated = self
            .frame
            .workers
            .iter()
            .filter(|w| !w.components.is_empty())
            .count();
        let spare = self.local_lanes.saturating_sub(populated);
        let helpers = spare.min(self.local_helper_limit.unwrap_or(usize::MAX));
        let mut before = costs.iter().copied().max().unwrap_or(0);
        for (&sum, w) in sums.iter().zip(&self.frame.workers) {
            let can_use_helper = helpers > 0
                && self.last_island_ids.as_ref().is_some_and(|ids| {
                    w.components
                        .iter()
                        .filter(|c| ids.binary_search(&c.ids[0]).is_ok())
                        .nth(1)
                        .is_some()
                });
            before = before.max(if can_use_helper { sum.div_ceil(2) } else { sum });
        }
        let budget = u64::try_from(policy.handoff_cost.as_nanos()).map_err(|_| Error::Capacity)?;
        let mut best = None;
        for group in &plan.groups {
            let [i] = group[..] else { continue };
            let ids = &plan.components[i].ids;
            let src = plan.owners[i];
            for (dst, target) in self.frame.workers.iter().enumerate() {
                if dst == src {
                    continue;
                }
                let after_populated = populated + usize::from(target.components.is_empty())
                    - usize::from(self.frame.workers[src].components.len() == 1);
                if after_populated > self.local_lanes {
                    continue;
                }
                if !self.placement_destination_fits(ids, dst) {
                    continue;
                }
                let new_sum = sums[dst].checked_add(costs[i]).ok_or(Error::Capacity)?;
                let after = sums
                    .iter()
                    .enumerate()
                    .map(|(w, &n)| {
                        if w == src {
                            n - costs[i]
                        } else if w == dst {
                            new_sum
                        } else {
                            n
                        }
                    })
                    .max()
                    .unwrap_or(0);
                let Some(gain) = before.checked_sub(after).filter(|&n| n > 0) else {
                    continue;
                };
                if u128::from(gain) * u128::from(policy.horizon.get()) <= u128::from(budget) {
                    continue;
                }
                // Prefer the smaller transfer on equal projected work, then stable IDs.
                let key = (after, ids.len(), ids[0], dst, src, i);
                if best.is_none_or(|old| key < old) {
                    best = Some(key);
                }
            }
        }
        Ok(best.map(|(after, _, _, dst, src, i)| PoolBalanceDecision {
            members: plan.components[i].ids.clone(),
            source: src as u8,
            destination: dst as u8,
            before_lower_ns: before,
            after_serial_peak_ns: after,
            estimated_payback_ticks: budget.div_ceil(before - after),
        }))
    }
}
