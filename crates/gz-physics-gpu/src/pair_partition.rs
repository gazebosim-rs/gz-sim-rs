// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One observation interval over disjoint contiguous sets of independent physical pairs.
//! Workers can finish in any order. The owner publishes only a complete successor.
mod rows;
use crate::{
    GpuError, MAX_BLOCK_TICKS, SpherePairBlockBatch, SpherePairBlockCheckpoint as Cut, StepMetrics,
};
use sha2::{Digest, Sha256};
use std::ops::Range;
fn bad(s: &str) -> GpuError {
    GpuError::Checkpoint(s.into())
}
const HEADER: usize = 48;
const HASH: usize = 32;
const TASK: &[u8; 8] = b"GZPRWK01";
const DONE: &[u8; 8] = b"GZPRDN01";
struct Piece {
    source: [u8; 32],
    start: u32,
    steps: u32,
    cut: Cut,
}
impl Piece {
    fn encode(&self, tag: &[u8; 8]) -> Result<Vec<u8>, GpuError> {
        let body = self.cut.to_bytes()?;
        let mut out = Vec::with_capacity(HEADER + body.len() + HASH);
        out.extend_from_slice(tag);
        out.extend_from_slice(&self.source);
        out.extend_from_slice(&self.start.to_le_bytes());
        out.extend_from_slice(&self.steps.to_le_bytes());
        out.extend_from_slice(&body);
        let sum = Sha256::digest(&out);
        out.extend_from_slice(&sum);
        Ok(out)
    }
    fn decode(b: &[u8], tag: &[u8; 8]) -> Result<Self, GpuError> {
        if !(HEADER + HASH..=PairBlockAssignment::MAX_BYTES).contains(&b.len()) || &b[..8] != tag {
            return Err(bad("invalid pair work record type or length"));
        }
        let end = b.len() - HASH;
        if Sha256::digest(&b[..end])[..] != b[end..] {
            return Err(bad("pair work record checksum mismatch"));
        }
        let source = b[8..40].try_into().unwrap();
        let start = u32::from_le_bytes(b[40..44].try_into().unwrap());
        let steps = u32::from_le_bytes(b[44..48].try_into().unwrap());
        if !(1..=MAX_BLOCK_TICKS).contains(&steps) {
            return Err(GpuError::BlockLength(steps));
        }
        let cut = Cut::from_bytes(&b[HEADER..end])?;
        if cut.observation_cadence().is_some() {
            return Err(bad("worker subpopulation cannot own the public clock"));
        }
        start
            .checked_add(cut.lanes() as u32)
            .ok_or_else(|| bad("overflowing pair range"))?;
        Ok(Self {
            source,
            start,
            steps,
            cut,
        })
    }
}
/// A warm, portable job for one worker, including the subpopulation's reset seeds.
pub struct PairBlockAssignment(Piece);
impl PairBlockAssignment {
    pub const MAX_BYTES: usize = Cut::MAX_BYTES + HEADER + HASH;
    pub fn to_bytes(&self) -> Result<Vec<u8>, GpuError> {
        self.0.encode(TASK)
    }
    pub fn from_bytes(b: &[u8]) -> Result<Self, GpuError> {
        let part = Piece::decode(b, TASK)?;
        part.cut.check_next_block(part.steps)?;
        Ok(Self(part))
    }
    pub fn range(&self) -> Range<usize> {
        let start = self.0.start as usize;
        start..start + self.0.cut.lanes()
    }
    /// Restore on a compatible Vulkan device, solve the prescribed interval and return its complete measured cut.
    pub fn execute(self, adapter: &str) -> Result<PairBlockReceipt, GpuError> {
        PairBlockExecutor::new(adapter)
            .execute(self)
            .map(|(cut, _, _)| cut)
    }
}
/// Fixed-size request for a worker that may still have exactly this subpopulation resident.
#[derive(Clone, Copy)]
pub struct PairBlockContinuation {
    source: [u8; 32],
    input: [u8; 32],
    start: u32,
    steps: u32,
}
impl PairBlockContinuation {
    pub const BYTES: usize = 112;
    pub fn to_bytes(self) -> [u8; Self::BYTES] {
        let mut out = [0; Self::BYTES];
        out[..8].copy_from_slice(b"GZPRCT01");
        out[8..40].copy_from_slice(&self.source);
        out[40..72].copy_from_slice(&self.input);
        out[72..76].copy_from_slice(&self.start.to_le_bytes());
        out[76..80].copy_from_slice(&self.steps.to_le_bytes());
        let sum = Sha256::digest(&out[..80]);
        out[80..].copy_from_slice(&sum);
        out
    }
    pub fn from_bytes(b: &[u8]) -> Result<Self, GpuError> {
        if b.len() != Self::BYTES
            || &b[..8] != b"GZPRCT01"
            || Sha256::digest(&b[..80])[..] != b[80..]
        {
            return Err(bad("invalid pair continuation"));
        }
        let steps = u32::from_le_bytes(b[76..80].try_into().unwrap());
        if !(1..=MAX_BLOCK_TICKS).contains(&steps) {
            return Err(GpuError::BlockLength(steps));
        }
        Ok(Self {
            source: b[8..40].try_into().unwrap(),
            input: b[40..72].try_into().unwrap(),
            start: u32::from_le_bytes(b[72..76].try_into().unwrap()),
            steps,
        })
    }
}
/// Device state retained by one worker between assignments. Exact equality of the incoming
/// subpopulation cut permits continued execution with only the four-byte tick count uploaded. A move or rollback
/// to another state restores the complete input before solving.
pub struct PairBlockExecutor {
    adapter: String,
    resident: Option<(u32, [u8; 32], SpherePairBlockBatch)>,
}
impl PairBlockExecutor {
    pub fn new(adapter: &str) -> Self {
        Self {
            adapter: adapter.into(),
            resident: None,
        }
    }
    /// Return the measured result, device transfer/timing metrics, and whether the worker reused its resident state.
    pub fn execute(
        &mut self,
        assignment: PairBlockAssignment,
    ) -> Result<(PairBlockReceipt, StepMetrics, bool), GpuError> {
        let mut p = assignment.0;
        let input = p.cut.digest()?;
        let (mut batch, reused) = match self.resident.take() {
            Some((start, id, batch)) if start == p.start && id == input => (batch, true),
            _ => (p.cut.restore(&self.adapter)?, false),
        };
        let metrics = batch.step_block(p.steps)?;
        p.cut = Cut::capture(&batch)?;
        self.resident = Some((p.start, p.cut.digest()?, batch));
        Ok((PairBlockReceipt(p), metrics, reused))
    }
    /// A cache miss leaves this worker unchanged, so the coordinator can send a complete assignment.
    pub fn try_continue(
        &mut self,
        request: PairBlockContinuation,
    ) -> Result<Option<(PairBlockReceipt, StepMetrics)>, GpuError> {
        if !self
            .resident
            .as_ref()
            .is_some_and(|(start, digest, _)| (*start, *digest) == (request.start, request.input))
        {
            return Ok(None);
        }
        let (_, _, mut batch) = self.resident.take().expect("matched resident cut");
        let metrics = batch.step_block(request.steps)?;
        let cut = Cut::capture(&batch)?;
        self.resident = Some((request.start, cut.digest()?, batch));
        Ok(Some((
            PairBlockReceipt(Piece {
                source: request.source,
                start: request.start,
                steps: request.steps,
                cut,
            }),
            metrics,
        )))
    }
}
/// Independent result for a specific source cut, interval and range.
pub struct PairBlockReceipt(Piece);
impl PairBlockReceipt {
    pub fn to_bytes(&self) -> Result<Vec<u8>, GpuError> {
        self.0.encode(DONE)
    }
    pub fn from_bytes(b: &[u8]) -> Result<Self, GpuError> {
        let p = Piece::decode(b, DONE)?;
        if p.cut.last_block_ticks() != p.steps {
            return Err(bad("receipt interval differs from solved block"));
        }
        Ok(Self(p))
    }
}
/// A single synchronous observation of a fixed population. This changes only its worker layout.
/// A Bevy cut advances by one public interval and retains its pause setting, as for step_exact(1).
pub struct PairBlockBarrier<'a> {
    cut: &'a Cut,
    source: [u8; 32],
    steps: u32,
    next: u64,
    workers: usize,
}
impl<'a> PairBlockBarrier<'a> {
    pub fn new(cut: &'a Cut, steps: u32, workers: usize) -> Result<Self, GpuError> {
        if workers == 0 || workers > cut.lanes() {
            return Err(bad("each worker must own a nonempty range"));
        }
        let next = cut.check_next_block(steps)?;
        Ok(Self {
            cut,
            source: cut.digest()?,
            steps,
            next,
            workers,
        })
    }
    pub fn workers(&self) -> usize {
        self.workers
    }
    fn range(&self, worker: usize) -> Result<Range<usize>, GpuError> {
        if worker >= self.workers {
            return Err(bad("worker is outside this partition"));
        }
        let base = self.cut.lanes() / self.workers;
        let extra = self.cut.lanes() % self.workers;
        let start = worker * base + worker.min(extra);
        Ok(start..start + base + usize::from(worker < extra))
    }
    pub fn continuation(&self, worker: usize) -> Result<PairBlockContinuation, GpuError> {
        let r = self.range(worker)?;
        Ok(PairBlockContinuation {
            source: self.source,
            start: r.start as u32,
            steps: self.steps,
            input: self.cut.subset_digest(r)?,
        })
    }
    pub fn assignment(&self, worker: usize) -> Result<PairBlockAssignment, GpuError> {
        let r = self.range(worker)?;
        Ok(PairBlockAssignment(Piece {
            source: self.source,
            start: r.start as u32,
            steps: self.steps,
            cut: self.cut.subset(r),
        }))
    }
    /// Accept an unordered set of receipts. Identical duplicates are idempotent.
    /// A stale, conflicting, missing or overlapping response leaves the source cut available for retry.
    pub fn commit(
        &self,
        receipts: impl IntoIterator<Item = PairBlockReceipt>,
    ) -> Result<Cut, GpuError> {
        let mut parts: Vec<Option<Cut>> =
            std::iter::repeat_with(|| None).take(self.workers).collect();
        let base = self.cut.lanes() / self.workers;
        let extra = self.cut.lanes() % self.workers;
        let split = (base + 1) * extra;
        for PairBlockReceipt(p) in receipts {
            if p.source != self.source
                || p.steps != self.steps
                || p.cut.tick() != self.next
                || p.cut.last_block_ticks() != self.steps
            {
                return Err(bad("receipt belongs to another source or interval"));
            }
            let start = p.start as usize;
            let end = start
                .checked_add(p.cut.lanes())
                .filter(|&e| e <= self.cut.lanes())
                .ok_or_else(|| bad("receipt range outside source"))?;
            let worker = if start < split {
                start / (base + 1)
            } else {
                extra + (start - split) / base
            };
            if self.range(worker)? != (start..end) {
                return Err(bad("receipt differs from assigned worker range"));
            }
            if !self.cut.matches_subset(start..end, &p.cut) {
                return Err(bad("incompatible reset seeds or execution profile"));
            }
            if let Some(previous) = &parts[worker] {
                if previous.digest()? != p.cut.digest()? {
                    return Err(bad("conflicting receipt for same range"));
                }
            } else {
                parts[worker] = Some(p.cut);
            }
        }
        if parts.iter().any(Option::is_none) {
            return Err(bad("incomplete pair barrier"));
        }
        let mut rows = Vec::with_capacity(self.cut.lanes());
        for part in parts.into_iter().flatten() {
            rows.extend_from_slice(part.observations());
        }
        self.cut.advance_with_rows(self.steps, rows)
    }
}
