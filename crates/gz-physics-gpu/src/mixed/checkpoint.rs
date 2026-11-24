// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bincode::Options;
use gz_sim_core::UpdateInfo;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
const MAGIC: &[u8; 8] = b"GZMIXV02";
const V1: &[u8; 8] = b"GZMIXV01";
/// Maximum serialized checkpoint size, including header and checksum.
pub const MAX_BYTES: usize = 128 * 1024 * 1024;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
struct Target {
    vendor: u32,
    device: u32,
    driver: String,
    driver_info: String,
    kernels: [u8; 32],
}
impl Target {
    fn from_device(i: &GpuInfo) -> Self {
        let mut hash = Sha256::new();
        for src in [
            include_str!("../arithmetic.rs"),
            include_str!("../kernel.rs"),
            include_str!("../spatial.rs"),
            include_str!("../sphere_plane.rs"),
            include_str!("../sphere_pair.rs"),
            include_str!("../supported_pair.rs"),
        ] {
            hash.update((src.len() as u64).to_le_bytes());
            hash.update(src.as_bytes());
        }
        Self {
            vendor: i.vendor_id,
            device: i.device_id,
            driver: i.driver.clone(),
            driver_info: i.driver_info.clone(),
            kernels: hash.finalize().into(),
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
struct Clock {
    tick: u64,
    epoch: u64,
    start: Duration,
    end: Duration,
    dt: Duration,
    paused_update: bool,
    paused_mode: bool,
}
/// A complete mixed physics frame, original full-reset seeds and admitted inputs.
/// A fresh device restores at the same committed public clock. Other Bevy
/// resources, including a learner's own policy state, are owned by their callers.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MixedCheckpoint {
    initial: Vec<Environment>,
    committed: Vec<Environment>,
    pending: Vec<CentralImpulse>,
    clock: Clock,
    target: Target,
    initial_cpu: Vec<u64>,
    cpu: Vec<u64>,
}
#[derive(Deserialize)]
struct LegacyCheckpoint {
    initial: Vec<Environment>,
    committed: Vec<Environment>,
    pending: Vec<CentralImpulse>,
    clock: Clock,
    target: Target,
}
fn error(e: impl ToString) -> GpuError {
    GpuError::Checkpoint(e.to_string())
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_little_endian()
        .with_limit(MAX_BYTES as u64)
        .reject_trailing_bytes()
}
impl MixedCheckpoint {
    pub const MAX_BYTES: usize = MAX_BYTES;
    pub fn tick(&self) -> u64 {
        self.clock.tick
    }
    pub fn epoch(&self) -> u64 {
        self.clock.epoch
    }
    pub fn to_bytes(&self) -> Result<Vec<u8>, GpuError> {
        let n = codec().serialized_size(self).map_err(error)?;
        if n > (MAX_BYTES - 40) as u64 {
            return Err(error("encoded checkpoint exceeds byte limit"));
        }
        let mut out = Vec::with_capacity(40 + n as usize);
        out.extend_from_slice(MAGIC);
        codec().serialize_into(&mut out, self).map_err(error)?;
        let digest = Sha256::digest(&out);
        out.extend_from_slice(&digest);
        Ok(out)
    }
    pub fn from_bytes(data: &[u8]) -> Result<Self, GpuError> {
        if !(40..=MAX_BYTES).contains(&data.len()) || (&data[..8] != MAGIC && &data[..8] != V1) {
            return Err(error("invalid checkpoint version or length"));
        }
        let (payload, hash) = data.split_at(data.len() - 32);
        if &Sha256::digest(payload)[..] != hash {
            return Err(error("checkpoint SHA-256 mismatch"));
        }
        let cp: Self = if &payload[..8] == V1 {
            let v: LegacyCheckpoint = codec().deserialize(&payload[8..]).map_err(error)?;
            Self {
                initial: v.initial,
                committed: v.committed,
                pending: v.pending,
                clock: v.clock,
                target: v.target,
                initial_cpu: vec![],
                cpu: vec![],
            }
        } else {
            codec().deserialize(&payload[8..]).map_err(error)?
        };
        cp.validate()?;
        Ok(cp)
    }
    fn validate(&self) -> Result<(), GpuError> {
        if self.initial.is_empty() || self.initial.len() != self.committed.len() {
            return Err(error("invalid population length"));
        }
        if !self.initial.windows(2).all(|p| p[0].id < p[1].id) {
            return Err(error("environment IDs must be unique and ordered"));
        }
        for (a, b) in self.initial.iter().zip(&self.committed) {
            if a.id != b.id
                || a.state.profile() != b.state.profile()
                || !a.state.valid()
                || !b.state.current_valid()
            {
                return Err(error(format!(
                    "invalid snapshot state or topology at environment {}",
                    a.id
                )));
            }
        }
        for ids in [&self.initial_cpu, &self.cpu] {
            if ids.len() > self.initial.len()
                || !ids.windows(2).all(|w| w[0] < w[1])
                || ids
                    .iter()
                    .any(|id| self.initial.binary_search_by_key(id, |e| e.id).is_err())
            {
                return Err(error("invalid checkpoint CPU execution placement"));
            }
        }
        let mut prev = None;
        for &a in &self.pending {
            if prev.is_some_and(|id| id >= a.id) {
                return Err(error("pending actions must be unique and ID-ordered"));
            }
            let index = self
                .committed
                .binary_search_by_key(&a.id, |s| s.id)
                .map_err(|_| error("unknown pending action identity"))?;
            self.committed[index].state.with_impulse(a)?;
            prev = Some(a.id);
        }
        Ok(())
    }
    pub(crate) fn capture(
        batch: &MixedBatch,
        info: UpdateInfo,
        paused: bool,
    ) -> Result<Self, GpuError> {
        if batch.fixed_scene {
            return Err(GpuError::FixedScene);
        }
        if info.iteration != batch.tick || info.sim_time != batch.sim_time() {
            return Err(GpuError::StaleBoundary);
        }
        Ok(Self {
            initial: batch.initial.clone(),
            committed: batch.committed.clone(),
            pending: batch.pending_impulses().copied().collect(),
            target: Target::from_device(&batch.gpu.info),
            initial_cpu: batch
                .initial_placement
                .as_ref()
                .map_or_else(Vec::new, |layout| {
                    layout.cpu.iter().map(|&i| batch.initial[i].id).collect()
                }),
            cpu: batch.placement.as_ref().map_or_else(Vec::new, |layout| {
                layout.cpu.iter().map(|&i| batch.initial[i].id).collect()
            }),
            clock: Clock {
                tick: batch.tick,
                epoch: info.epoch,
                start: info.start_time,
                end: info.sim_time,
                dt: info.dt,
                paused_update: info.paused,
                paused_mode: paused,
            },
        })
    }
    pub(crate) fn info(&self) -> (UpdateInfo, bool) {
        let c = &self.clock;
        (
            UpdateInfo {
                iteration: c.tick,
                epoch: c.epoch,
                start_time: c.start,
                sim_time: c.end,
                dt: c.dt,
                paused: c.paused_update,
            },
            c.paused_mode,
        )
    }
    pub(crate) fn restore(&self, selector: &str) -> Result<MixedBatch, GpuError> {
        self.validate()?;
        let mut batch = MixedBatch::new(selector, self.initial.clone())?;
        if self.target != Target::from_device(&batch.gpu.info) {
            return Err(error(
                "kernel or Vulkan driver/device profile differs from checkpoint",
            ));
        }
        batch = batch.with_initial_cpu(&self.initial_cpu)?;
        let now_gpu: Vec<_> = self
            .initial_cpu
            .iter()
            .copied()
            .filter(|id| self.cpu.binary_search(id).is_err())
            .collect();
        batch.set_placement(0, &now_gpu, ExecutionPlacement::Gpu)?;
        batch.set_placement(0, &self.cpu, ExecutionPlacement::Cpu)?;
        batch.committed.copy_from_slice(&self.committed);
        batch.tick = self.clock.tick;
        batch.queue_impulses(batch.tick, &self.pending)?;
        Ok(batch)
    }
}

#[cfg(test)]
#[path = "../../tests/private/checkpoint.rs"]
mod tests;
