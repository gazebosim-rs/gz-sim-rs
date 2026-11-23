// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BodySlot {
    A,
    B,
}
/// A single world-space linear impulse at one sphere's center of mass.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CentralImpulse {
    pub id: u64,
    pub body: BodySlot,
    pub impulse: [f64; 3],
}
impl EnvironmentState {
    pub(super) fn with_impulse(self, action: CentralImpulse) -> Result<Self, GpuError> {
        let applied = self.apply_impulse(action)?;
        if !applied.current_valid() {
            return Err(GpuError::InvalidAction(action.id));
        }
        Ok(applied)
    }
    fn apply_impulse(mut self, action: CentralImpulse) -> Result<Self, GpuError> {
        if !action.impulse.into_iter().all(f64::is_finite) {
            return Err(GpuError::InvalidAction(action.id));
        }
        let update = |velocity: &mut [f64; 3], mass: f64| {
            for (v, j) in velocity.iter_mut().zip(action.impulse) {
                *v += j * mass;
            }
        };
        let pair_update = |s: &mut SpherePairLane| {
            let b = match action.body {
                BodySlot::A => &mut s.a,
                BodySlot::B => &mut s.b,
            };
            update(&mut b.velocity, b.inverse_mass);
        };
        match &mut self {
            Self::SpherePlane(s) if action.body == BodySlot::A => {
                update(&mut s.velocity, s.inverse_mass)
            }
            Self::SpherePlane(_) => return Err(GpuError::InvalidAction(action.id)),
            Self::SpherePair(s) => pair_update(s),
            Self::SupportedSpherePair(s) => {
                // Enforce the constrained impulse itself. A large preexisting
                // tangential velocity cannot make an outward impulse admissible.
                let terms: [f64; 3] =
                    std::array::from_fn(|i| s.plane_normal[i] * action.impulse[i]);
                if terms.iter().sum::<f64>().abs()
                    > 64. * f64::EPSILON * terms.iter().map(|v| v.abs()).sum::<f64>()
                {
                    return Err(GpuError::InvalidAction(action.id));
                }
                pair_update(&mut s.pair);
            }
        }

        Ok(self)
    }
    /// Validate a continuation with its current contact observations. Reuse the
    /// initial geometry and motion bounds with observation counters cleared.
    pub(super) fn current_valid(&self) -> bool {
        let mut candidate = *self;
        let clear = |s: &mut SpherePairLane| {
            s.normal_impulse = 0.;
            s.tangential_impulse = [0.; 3];
            s.contact_normal = [0.; 3];
            s.impact_time = -1.;
        };
        match &mut candidate {
            Self::SpherePlane(s) => {
                s.normal_impulse = 0.;
                s.tangential_impulse = [0.; 3];
            }
            Self::SpherePair(s) => clear(s),
            Self::SupportedSpherePair(s) => {
                clear(&mut s.pair);
                s.support_normal_impulses = [0.; 2];
            }
        }
        self.values().iter().all(|x| x.is_finite()) && candidate.valid()
    }
}
impl MixedBatch {
    pub(super) fn index(&self, id: u64) -> Result<usize, GpuError> {
        self.committed
            .binary_search_by_key(&id, |s| s.id)
            .map_err(|_| GpuError::UnknownEnvironment(id))
    }
    fn check_tick(&self, tick: u64) -> Result<(), GpuError> {
        if self.tick == tick {
            Ok(())
        } else {
            Err(GpuError::StaleBoundary)
        }
    }
    /// Replace selected environments atomically without advancing the clock.
    /// A new episode clears the previous episode's pending action on those IDs.
    /// The next step uploads only changed records on an already resident device.
    pub fn reset_selected(
        &mut self,
        tick: u64,
        replacements: &[Environment],
    ) -> Result<(), GpuError> {
        self.check_tick(tick)?;
        if self.fixed_scene {
            return Err(GpuError::FixedScene);
        }
        let mut edits = BTreeMap::new();
        for env in replacements {
            let i = self.index(env.id)?;
            if env.state.profile() != self.committed[i].state.profile() {
                return Err(GpuError::ProfileChange(env.id));
            }
            if !env.state.valid() {
                return Err(GpuError::InvalidEnvironment {
                    id: env.id,
                    profile: env.state.profile(),
                });
            }
            if edits.insert(i, env.state).is_some() {
                return Err(GpuError::DuplicateEnvironment(env.id));
            }
        }
        for (i, state) in edits {
            self.committed[i].state = state;
            self.dirty.insert(i);
            self.pending.remove(&i);
        }
        Ok(())
    }
    /// Queue at most one impulse per environment for the next advancing tick.
    /// All inputs validate before any command is accepted; paused ticks retain
    /// them, and a failed advance leaves them available for an exact retry.
    pub fn queue_impulses(
        &mut self,
        tick: u64,
        actions: &[CentralImpulse],
    ) -> Result<(), GpuError> {
        self.check_tick(tick)?;
        let mut staged = BTreeMap::new();
        for &action in actions {
            let i = self.index(action.id)?;
            if self.pending.contains_key(&i) || staged.contains_key(&i) {
                return Err(GpuError::DuplicateEnvironment(action.id));
            }
            let state = self.committed[i].state;
            #[cfg(feature = "native-handoff")]
            let applied = if self
                .native
                .as_ref()
                .is_some_and(|set| set.active.contains_key(&i))
            {
                let state = state.apply_impulse(action)?;
                native::motion_envelope(action.id, state)?;
                state
            } else {
                state.with_impulse(action)?
            };
            #[cfg(not(feature = "native-handoff"))]
            let applied = state.with_impulse(action)?;
            staged.insert(i, (action, applied));
        }
        self.dirty.extend(staged.keys().copied());
        self.pending.append(&mut staged);
        Ok(())
    }
    pub fn pending_impulses(&self) -> impl ExactSizeIterator<Item = &CentralImpulse> {
        self.pending.values().map(|(a, _)| a)
    }
    fn pack_record(&mut self, i: usize) -> std::ops::Range<usize> {
        let value = self
            .pending
            .get(&i)
            .map_or(&self.committed[i].state, |(_, s)| s);
        let bytes = bytemuck::cast_slice(value.values());
        let offset = self.active_offsets()[i];
        let range = offset..offset + bytes.len();
        self.pack[range.clone()].copy_from_slice(bytes);
        range
    }
    pub(super) fn upload_changes(&mut self, metrics: &mut StepMetrics) {
        if !self.upload_needed && self.dirty.is_empty() {
            return;
        }
        let start = Instant::now();
        if self.upload_needed {
            for i in 0..self.committed.len() {
                if self.active_offsets()[i] != placement::CPU {
                    self.pack_record(i);
                }
            }
            metrics.pack = start.elapsed();
            let upload = Instant::now();
            let size = self.active_gpu_bytes();
            self.gpu
                .queue
                .write_buffer(&self.gpu.state, 0, &self.pack[..size as usize]);
            metrics.uploaded_bytes = size;
            metrics.upload = upload.elapsed();
            return;
        }
        // Consecutive physical records can share one write. Never bridge a
        // clean lane: its authoritative resident state has advanced since pack.
        let offsets = self.active_offsets();
        let mut indices: Vec<_> = self
            .dirty
            .iter()
            .copied()
            .filter(|&i| offsets[i] != placement::CPU)
            .collect();
        indices.sort_unstable_by_key(|i| offsets[*i]);
        let mut ranges: Vec<std::ops::Range<usize>> = vec![];
        for i in indices {
            let range = self.pack_record(i);
            if let Some(last) = ranges.last_mut()
                && last.end == range.start
            {
                last.end = range.end;
            } else {
                ranges.push(range);
            }
        }
        metrics.pack = start.elapsed();
        let upload = Instant::now();
        for range in ranges {
            metrics.uploaded_bytes += range.len() as u64;
            self.gpu
                .queue
                .write_buffer(&self.gpu.state, range.start as u64, &self.pack[range]);
        }
        metrics.upload = upload.elapsed();
    }
}
