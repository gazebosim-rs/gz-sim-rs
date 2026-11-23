// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Manual execution placement for the three explicitly independent profiles.
use super::*;
pub(super) const CPU: usize = usize::MAX;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionPlacement {
    Cpu,
    Gpu,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PlacementChange {
    /// Number of environments whose execution owner changed.
    pub moved: usize,
    /// GPU-owned rows that need upload because their owner or physical offset changed.
    pub reseeded_gpu_rows: usize,
}
#[derive(Clone)]
pub(super) struct ActiveLayout {
    pub(super) offsets: Vec<usize>,
    pub(super) groups: Vec<GroupInfo>,
    pub(super) dispatches: Vec<Dispatch>,
    pub(super) cpu: Vec<usize>,
    pub(super) bytes: u64,
}
impl MixedBatch {
    pub(super) fn active_offsets(&self) -> &[usize] {
        self.placement
            .as_ref()
            .map_or(&self.offsets, |l| &l.offsets)
    }
    /// Occupied compact state prefix, including alignment between active families.
    pub fn active_gpu_bytes(&self) -> u64 {
        self.placement
            .as_ref()
            .map_or(self.gpu.info.state_bytes, |l| l.bytes)
    }
    /// Number of CPU execution owners restored by a full scene reset.
    pub fn initial_cpu_environments(&self) -> usize {
        self.initial_placement.as_ref().map_or(0, |l| l.cpu.len())
    }
    pub fn cpu_environments(&self) -> usize {
        self.placement.as_ref().map_or(0, |l| l.cpu.len())
    }
    pub fn execution_placement(&self, id: u64) -> Result<ExecutionPlacement, GpuError> {
        let i = self.index(id)?;
        Ok(if self.active_offsets()[i] == CPU {
            ExecutionPlacement::Cpu
        } else {
            ExecutionPlacement::Gpu
        })
    }
    /// Seal the full-reset execution owners at construction. Physics uses the same
    /// stable seed records even when initially placed entirely on the host.
    pub fn with_initial_cpu(mut self, ids: &[u64]) -> Result<Self, GpuError> {
        #[cfg(feature = "native-handoff")]
        if self.native.is_some() {
            return Err(GpuError::FixedScene);
        }
        if self.tick != 0 || !self.pending.is_empty() || self.committed != self.initial {
            return Err(GpuError::StaleBoundary);
        }
        // This constructor describes the complete initial set, even if a caller
        // configured a different initial placement earlier at this same boundary.
        self.placement = None;
        self.dirty.clear();
        self.upload_needed = true;
        self.set_placement(0, ids, ExecutionPlacement::Cpu)?;
        self.initial_placement = self.placement.clone();
        Ok(self)
    }
    /// Transfer ownership only at the current committed frame. Existing positions,
    /// spins, last-step impulses and pending next-step inputs remain byte-identical.
    pub fn set_placement(
        &mut self,
        tick: u64,
        ids: &[u64],
        target: ExecutionPlacement,
    ) -> Result<PlacementChange, GpuError> {
        #[cfg(feature = "native-handoff")]
        if let Some(set) = &self.native {
            for id in ids {
                if set.eligible.contains(&self.index(*id)?) {
                    return Err(GpuError::NativeHandoff {
                        id: *id,
                        reason: native::NativeHandoffRefusal::GuardRequired,
                    });
                }
            }
        }
        self.place(tick, ids, target)
    }
    pub(super) fn place(
        &mut self,
        tick: u64,
        ids: &[u64],
        target: ExecutionPlacement,
    ) -> Result<PlacementChange, GpuError> {
        if self.tick != tick {
            return Err(GpuError::StaleBoundary);
        }
        let mut selection = BTreeSet::new();
        for &id in ids {
            let index = self.index(id)?;
            if !selection.insert(index) {
                return Err(GpuError::DuplicateEnvironment(id));
            }
        }
        let previous = self.active_offsets();
        let cpu = target == ExecutionPlacement::Cpu;
        let moved = selection
            .iter()
            .filter(|&&i| (previous[i] == CPU) != cpu)
            .count();
        if moved == 0 {
            return Ok(PlacementChange::default());
        }
        let cpu: Vec<usize> = (0..previous.len())
            .filter(|i| {
                if selection.contains(i) {
                    cpu
                } else {
                    previous[*i] == CPU
                }
            })
            .collect();
        let layout = if cpu.is_empty() {
            None
        } else {
            Some(self.compact(cpu)?)
        };
        let offsets = layout.as_ref().map_or(&self.offsets, |l| &l.offsets);
        let reseed: Vec<_> = previous
            .iter()
            .zip(offsets)
            .enumerate()
            .filter(|(_, (old, new))| old != new && **new != CPU)
            .map(|(i, _)| i)
            .collect();
        let report = PlacementChange {
            moved,
            reseeded_gpu_rows: reseed.len(),
        };
        self.placement = layout;
        self.dirty.extend(reseed);
        Ok(report)
    }
    fn compact(&self, cpu: Vec<usize>) -> Result<ActiveLayout, GpuError> {
        let mut offsets = vec![0; self.committed.len()];
        for &i in &cpu {
            offsets[i] = CPU;
        }
        let mut counts = [0u32; 3];
        for (i, e) in self.committed.iter().enumerate() {
            if offsets[i] != CPU {
                counts[e.state.profile() as usize] += 1;
            }
        }
        let mut groups = Vec::new();
        let mut dispatches = Vec::new();
        let mut cursor = [0; 3];
        let mut bytes = 0u64;
        let alignment = u64::from(self.gpu.device.limits().min_storage_buffer_offset_alignment);
        for (original, template) in self.groups.iter().zip(&self.dispatches) {
            let family = original.profile as usize;
            let count = counts[family];
            if count == 0 {
                continue;
            }
            let offset = bytes.next_multiple_of(alignment);
            let length = (original.bytes / original.environments as u64) * u64::from(count);
            let workgroups = count.div_ceil(GROUP_WIDTH);
            bytes = offset + length;
            if bytes > self.gpu.info.state_bytes {
                return Err(GpuError::Capacity);
            }
            groups.push(GroupInfo {
                profile: original.profile,
                environments: count as usize,
                offset,
                bytes: length,
                workgroups,
            });
            dispatches.push(
                self.gpu
                    .bind_mixed_range(template, offset, length, workgroups),
            );
            cursor[family] = offset as usize;
        }
        for (at, env) in offsets.iter_mut().zip(&self.committed) {
            if *at != CPU {
                let c = &mut cursor[env.state.profile() as usize];
                *at = *c;
                *c += size_of_val(env.state.values());
            }
        }
        Ok(ActiveLayout {
            offsets,
            groups,
            dispatches,
            cpu,
            bytes,
        })
    }
}
pub(super) fn cpu_step(mut state: EnvironmentState) -> EnvironmentState {
    fn apply<const N: usize>(s: &mut [f64], results: [(usize, f64); N]) {
        for (i, v) in results {
            s[i] = v;
        }
    }
    match &mut state {
        EnvironmentState::SpherePlane(s) => {
            let f = bytemuck::cast_slice_mut(std::slice::from_mut(s));
            let out = sphere_plane::tick(f, sphere_plane::PARAMETERS);
            apply(f, out);
        }
        EnvironmentState::SpherePair(s) => {
            let f = bytemuck::cast_slice_mut(std::slice::from_mut(s));
            let out = sphere_pair::tick(f, sphere_pair::PARAMETERS);
            apply(f, out);
        }
        EnvironmentState::SupportedSpherePair(s) => {
            let f = bytemuck::cast_slice_mut(std::slice::from_mut(s));
            let out = supported_pair::tick_supported(f, sphere_pair::PARAMETERS);
            apply(f, out);
        }
    }
    state
}

#[cfg(test)]
#[path = "../../tests/private/placement.rs"]
mod tests;
