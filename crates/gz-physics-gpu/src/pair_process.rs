// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional local process ownership for independent device-resident finite sphere pairs.
mod child;
mod schedule;
pub use schedule::{install, set_width};
mod wire;
use crate::{
    GpuError, PairBlockBarrier, PairBlockReceipt, SpherePairBlockBatch,
    SpherePairBlockCheckpoint as Cut,
};
use bevy_ecs::prelude::Resource;
use child::Child;
use std::{
    io,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
use wire::Response;
pub use wire::serve;

pub const MAX_PAIR_WORKERS: usize = 16;
/// Select the response representation independently of the solver, schedule and checkpoint format.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairReplyFormat {
    /// Send a self-contained worker cut, including its original reset population.
    Portable,
    /// Transfer measured rows and bind them to the source population in the simulator.
    #[default]
    Observations,
}
const TIMEOUT: Duration = Duration::from_secs(30);
fn bad(s: impl ToString) -> GpuError {
    GpuError::Device(s.to_string())
}
/// One successfully published barrier's local process and device work. Byte counts cover
/// framed application payloads, excluding the four-byte length prefix and startup handshake.
#[derive(Default, Clone, Copy, Debug)]
pub struct PairProcessMetrics {
    pub workers: usize,
    pub restarted: usize,
    pub reused: usize,
    pub request_bytes: u64,
    pub response_bytes: u64,
    pub upload_bytes: u64,
    pub readback_bytes: u64,
    /// Sum of durations on individual workers that supplied timestamp queries.
    pub dispatch_sum: Duration,
    /// Clone and fingerprint the public source, before any worker request.
    pub prepare: Duration,
    /// Elapsed time actually blocked for replies, including transport, worker solve and scheduling.
    pub wait: Duration,
    /// Coordinator CPU time decoding the returned envelopes and checking physical rows.
    pub decode: Duration,
    /// Validate all ranges and publish the complete host state.
    pub commit: Duration,
    pub latency: Duration,
}
/// Persistent workers for one independently partitionable physical population. The host remains
/// the sole committer. Worker failure replays only that interval on a fresh process.
#[derive(Resource)]
pub struct PairProcessPool {
    path: PathBuf,
    adapter: String,
    children: Vec<Child>,
    width: usize,
    format: PairReplyFormat,
    last: Option<PairProcessMetrics>,
}
impl PairProcessPool {
    pub fn new(path: &Path, adapter: &str, width: usize) -> Result<Self, GpuError> {
        Self::with_format(path, adapter, width, PairReplyFormat::default())
    }
    pub fn with_format(
        path: &Path,
        adapter: &str,
        width: usize,
        format: PairReplyFormat,
    ) -> Result<Self, GpuError> {
        let mut p = Self {
            path: path.into(),
            adapter: adapter.into(),
            children: Vec::new(),
            width,
            format,
            last: None,
        };
        p.resize(width)?;
        Ok(p)
    }
    /// PIDs in assigned-range order, for operator monitoring.
    pub fn processes(&self) -> impl ExactSizeIterator<Item = u32> + '_ {
        self.children.iter().map(Child::pid)
    }
    pub fn last(&self) -> Option<PairProcessMetrics> {
        self.last
    }
    pub(crate) fn clear_metrics(&mut self) {
        self.last = None;
    }
    pub(crate) fn resize(&mut self, width: usize) -> Result<(), GpuError> {
        if !(1..=MAX_PAIR_WORKERS).contains(&width) {
            return Err(bad("pair process count must be 1 through 16"));
        }
        let until = Instant::now() + TIMEOUT;
        let mut added = Vec::new();
        for _ in self.children.len()..width {
            added.push(
                Child::start(
                    &self.path,
                    &self.adapter,
                    self.format == PairReplyFormat::Observations,
                    until,
                )
                .map_err(bad)?,
            );
        }
        self.children.truncate(width);
        self.children.extend(added);
        self.width = width;
        Ok(())
    }
    /// Advance disjoint ranges. No public state is changed until all receipts pass the barrier.
    pub fn advance(
        &mut self,
        batch: &mut SpherePairBlockBatch,
        ticks: u32,
    ) -> Result<PairProcessMetrics, GpuError> {
        let start = Instant::now();
        let cut = Cut::capture(batch)?;
        let barrier = PairBlockBarrier::new(&cut, ticks, self.width)?;
        if self.children.len() != self.width {
            self.resize(self.width)?;
        }
        let prepare = start.elapsed();
        let (next, mut m, committing) =
            match self.solve(&barrier, start).and_then(|(receipts, m)| {
                let committing = Instant::now();
                barrier.commit(receipts).map(|next| (next, m, committing))
            }) {
                Ok(result) => result,
                Err(e) => {
                    // A lost final reply might still be buffered. Reopen from the last public cut
                    // even if the caller resets or restores before its retry.
                    self.children.clear();
                    return Err(e);
                }
            };
        batch.committed.copy_from_slice(next.observations());
        batch.tick = next.tick();
        batch
            .block_control
            .as_mut()
            .expect("pair block control")
            .last_ticks = ticks;
        batch.upload_needed = true;
        m.prepare = prepare;
        m.commit = committing.elapsed();
        m.latency = start.elapsed();
        self.last = Some(m);
        Ok(m)
    }
    fn solve(
        &mut self,
        barrier: &PairBlockBarrier<'_>,
        start: Instant,
    ) -> Result<(Vec<PairBlockReceipt>, PairProcessMetrics), GpuError> {
        let mut m = PairProcessMetrics {
            workers: self.children.len(),
            ..Default::default()
        };
        let until = start + TIMEOUT;
        let mut pending = Vec::with_capacity(m.workers);
        for (i, c) in self.children.iter().enumerate() {
            let cue = barrier.continuation(i)?.to_bytes().to_vec();
            m.request_bytes += cue.len() as u64;
            pending.push(c.send(cue));
        }
        let mut receipts = Vec::with_capacity(m.workers);
        // Drain every outstanding reply even if an earlier range failed. This prevents stale
        // responses from being attributed to a later interval on an otherwise healthy worker.
        let mut failure = None;
        for (i, (child, sent)) in self.children.iter_mut().zip(pending).enumerate() {
            let full = |c: &mut Child,
                        m: &mut PairProcessMetrics,
                        until: Instant|
             -> io::Result<Response> {
                let input = barrier
                    .assignment(i)
                    .and_then(|a| a.to_bytes())
                    .map_err(io::Error::other)?;
                m.request_bytes += input.len() as u64;
                c.send(input)?;
                receive(c, until, m, barrier, i)
            };
            let ans = sent
                .and_then(|_| receive(child, until, &mut m, barrier, i))
                .and_then(|r| match r {
                    Response::Miss => full(child, &mut m, until),
                    other => Ok(other),
                });
            let response = match ans {
                Ok(v) => Ok(v),
                Err(_) => {
                    let deadline = Instant::now() + TIMEOUT;
                    // Spawn successfully before replacing a child. The old request cannot
                    // be published, even if the old worker later finishes.
                    Child::start(
                        &self.path,
                        &self.adapter,
                        self.format == PairReplyFormat::Observations,
                        deadline,
                    )
                    .and_then(|new| {
                        *child = new;
                        m.restarted += 1;
                        full(child, &mut m, deadline)
                    })
                }
            };
            match response {
                Ok(Response::Done {
                    receipt,
                    reused,
                    upload,
                    readback,
                    dispatch_ns,
                }) => {
                    receipts.push(receipt);
                    m.reused += usize::from(reused);
                    m.upload_bytes += upload;
                    m.readback_bytes += readback;
                    m.dispatch_sum += Duration::from_nanos(dispatch_ns.unwrap_or(0));
                }
                Ok(Response::Error(e)) => {
                    failure.get_or_insert_with(|| bad(e));
                }
                Ok(Response::Miss) => {
                    failure.get_or_insert_with(|| bad("worker refused a full assignment"));
                }
                Err(e) => {
                    failure.get_or_insert_with(|| bad(e));
                }
            }
        }
        if let Some(e) = failure {
            return Err(e);
        }
        Ok((receipts, m))
    }
}
fn receive(
    c: &mut Child,
    until: Instant,
    m: &mut PairProcessMetrics,
    barrier: &PairBlockBarrier<'_>,
    ordinal: usize,
) -> io::Result<Response> {
    let waiting = Instant::now();
    let bytes = c.receive(until)?;
    m.wait += waiting.elapsed();
    let decode = Instant::now();
    m.response_bytes += bytes.len() as u64;
    let result = wire::response(&bytes, barrier, ordinal);
    m.decode += decode.elapsed();
    result
}
