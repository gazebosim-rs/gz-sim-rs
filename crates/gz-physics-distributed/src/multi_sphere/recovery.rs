// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::articulation::ledger::WriterIdentity;
use serde::Serialize;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RecoveryPhase {
    /// Restoring the committed state with the ordinary worker repair procedure.
    Restoring,
    /// Waiting to start or installing a fresh worker pool.
    Reopening,
    /// Running the interrupted operation again against the restored state.
    Replaying,
    /// Recovery stopped. Explicit reopen and inspection are required.
    Fenced,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct RecoveryStatus {
    /// Last acknowledged epoch and tick. A publication failure can be ambiguous on disk.
    pub epoch: u64,
    pub tick: u64,
    pub phase: RecoveryPhase,
    /// One-based index of the current fresh-pool attempt, or zero for ordinary repairs.
    pub attempt: u8,
    pub budget: u8,
}

/// Coordinator-issued authority to reinstall one verified transport-fenced
/// checkpoint. The entire archive, including the envelope, is identified by
/// the lowercase SHA-256. The live receipt also pins the originating session
/// and controller marker. Inspection and deserialization cannot issue receipts.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct FenceReceipt {
    epoch: u64,
    tick: u64,
    sha256: String,
    #[serde(skip)]
    writer: WriterIdentity,
}

impl FenceReceipt {
    pub(super) fn verified(epoch: u64, tick: u64, sha256: String, writer: WriterIdentity) -> Self {
        Self {
            epoch,
            tick,
            sha256,
            writer,
        }
    }

    pub(super) fn owns_writer(&self, writer: &WriterIdentity) -> bool {
        self.writer == *writer
    }

    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn sha256(&self) -> &str {
        &self.sha256
    }
}

#[derive(Debug)]
struct Progress {
    status: Option<RecoveryStatus>,
    fence: Option<FenceReceipt>,
}

/// Read the latest recovery phase from another thread, including while a worker
/// RPC is blocked. A monitor belongs to one coordinator instance. Healthy
/// operation returns `None` and intermediate transitions may be coalesced.
#[derive(Clone, Debug)]
pub struct RecoveryMonitor(Arc<Mutex<Progress>>);

impl RecoveryMonitor {
    pub(super) fn new(status: Option<RecoveryStatus>, fence: Option<FenceReceipt>) -> Self {
        Self(Arc::new(Mutex::new(Progress { status, fence })))
    }

    pub fn snapshot(&self) -> Option<RecoveryStatus> {
        self.0.lock().unwrap_or_else(|e| e.into_inner()).status
    }

    pub fn fence_receipt(&self) -> Option<FenceReceipt> {
        self.0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .fence
            .clone()
    }

    pub(super) fn update(&self, status: Option<RecoveryStatus>, fence: Option<FenceReceipt>) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = Progress { status, fence };
    }
}
