// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Send only measurements whose initial conditions already belong to the coordinator.
use super::{PairBlockBarrier, PairBlockReceipt, Piece, bad};
use crate::{GpuError, SpherePairBlockObservation as Obs};
use std::mem::size_of;
const MAGIC: &[u8; 8] = b"GZPRRO01";
const HEADER: usize = 96;

impl PairBlockReceipt {
    /// Encode the measured rows with a digest of the complete portable successor, including
    /// its omitted seeds and execution profile. A receiver needs the source barrier to decode it.
    pub fn observation_bytes(&self) -> Result<Vec<u8>, GpuError> {
        let p = &self.0;
        let mut out = Vec::with_capacity(HEADER + size_of_val(p.cut.observations()));
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&p.source);
        out.extend_from_slice(&p.start.to_le_bytes());
        out.extend_from_slice(&p.steps.to_le_bytes());
        out.extend_from_slice(&p.cut.tick().to_le_bytes());
        out.extend_from_slice(&(p.cut.lanes() as u32).to_le_bytes());
        out.extend_from_slice(&[0; 4]);
        out.extend_from_slice(&p.cut.digest()?);
        debug_assert_eq!(out.len(), HEADER);
        out.extend_from_slice(bytemuck::cast_slice(p.cut.observations()));
        Ok(out)
    }
}
impl PairBlockBarrier<'_> {
    /// Reconstruct and verify a successor from the assigned range's original seeds, checking
    /// the same physical invariants and complete state digest as a self-contained receipt.
    pub fn observation_receipt(
        &self,
        worker: usize,
        bytes: &[u8],
    ) -> Result<PairBlockReceipt, GpuError> {
        let range = self.range(worker)?;
        if bytes.len() != HEADER + range.len() * size_of::<Obs>()
            || &bytes[..8] != MAGIC
            || bytes[8..40] != self.source
            || bytes[40..44] != (range.start as u32).to_le_bytes()
            || bytes[44..48] != self.steps.to_le_bytes()
            || bytes[48..56] != self.next.to_le_bytes()
            || bytes[56..60] != (range.len() as u32).to_le_bytes()
            || bytes[60..64] != [0; 4]
        {
            return Err(bad("incompatible GPU observation receipt or range"));
        }
        let start = range.start as u32;
        let cut = self
            .cut
            .decode_subset_successor(range, self.steps, &bytes[HEADER..])?;
        if cut.digest()?[..] != bytes[64..96] {
            return Err(bad("reconstructed GPU observation digest differs"));
        }
        Ok(PairBlockReceipt(Piece {
            source: self.source,
            start,
            steps: self.steps,
            cut,
        }))
    }
}
