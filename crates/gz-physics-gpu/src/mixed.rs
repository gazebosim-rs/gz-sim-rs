// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicitly registered independent environments sharing one device and observation barrier.
use super::*;
use std::mem::size_of_val;
mod control;
#[cfg(feature = "native-handoff")]
pub(crate) mod native;
mod placement;
pub use control::{BodySlot, CentralImpulse};
use placement::ActiveLayout;
pub use placement::{ExecutionPlacement, PlacementChange};
mod checkpoint;
pub use checkpoint::MixedCheckpoint;
use std::collections::{BTreeMap, BTreeSet};

/// The selected physical model also fixes the ABI of this environment's device record.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum EnvironmentState {
    SpherePlane(SpherePlaneLane),
    SpherePair(SpherePairLane),
    SupportedSpherePair(SupportedSpherePairLane),
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Environment {
    /// Caller-selected identity, preserved across resets and independent of registration order.
    pub id: u64,
    pub state: EnvironmentState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Profile {
    SpherePlane,
    SpherePair,
    SupportedSpherePair,
}

impl EnvironmentState {
    pub fn profile(&self) -> Profile {
        match self {
            Self::SpherePlane(_) => Profile::SpherePlane,
            Self::SpherePair(_) => Profile::SpherePair,
            Self::SupportedSpherePair(_) => Profile::SupportedSpherePair,
        }
    }
    fn valid(&self) -> bool {
        match self {
            Self::SpherePlane(s) => s.initial_valid(),
            Self::SpherePair(s) => s.initial_valid(),
            Self::SupportedSpherePair(s) => s.valid(),
        }
    }
    /// Exact packed observation, with only this model's native f64 fields.
    pub fn values(&self) -> &[f64] {
        fn fields<T: Pod>(s: &T) -> &[f64] {
            bytemuck::cast_slice(std::slice::from_ref(s))
        }
        match self {
            Self::SpherePlane(s) => fields(s),
            Self::SpherePair(s) => fields(s),
            Self::SupportedSpherePair(s) => fields(s),
        }
    }
    fn bytes_mut(&mut self) -> &mut [u8] {
        match self {
            Self::SpherePlane(s) => bytemuck::bytes_of_mut(s),
            Self::SpherePair(s) => bytemuck::bytes_of_mut(s),
            Self::SupportedSpherePair(s) => bytemuck::bytes_of_mut(s),
        }
    }
}
const PROFILES: [Profile; 3] = [
    Profile::SpherePlane,
    Profile::SpherePair,
    Profile::SupportedSpherePair,
];

#[derive(Debug, Clone, Copy)]
pub struct GroupInfo {
    pub profile: Profile,
    pub environments: usize,
    /// Byte offset within the shared state and readback buffers.
    pub offset: u64,
    pub bytes: u64,
    pub workgroups: u32,
}

struct Layout {
    groups: Vec<GroupInfo>,
    offsets: Vec<usize>,
    plan: DevicePlan,
}
impl Layout {
    fn new(
        initial: &[Environment],
        features: wgpu::Features,
        caps: wgpu::DownlevelFlags,
        limits: &wgpu::Limits,
    ) -> Result<Self, GpuError> {
        let mut counts = [0; 3];
        for env in initial {
            counts[env.state.profile() as usize] += 1;
        }
        let mut groups = Vec::new();
        let mut next = [0; 3];
        let mut end = 0u64;
        let mut workgroups = 0u32;
        let mut last_plan = None;
        let alignment = u64::from(limits.min_storage_buffer_offset_alignment);
        for (profile, count) in PROFILES.into_iter().zip(counts) {
            if count == 0 {
                continue;
            }
            let plan = match profile {
                Profile::SpherePlane => {
                    DevicePlan::new::<SpherePlaneLane>(count, features, caps, limits)
                }
                Profile::SpherePair => {
                    DevicePlan::new::<SpherePairLane>(count, features, caps, limits)
                }
                Profile::SupportedSpherePair => {
                    DevicePlan::new::<SupportedSpherePairLane>(count, features, caps, limits)
                }
            }?;
            let offset = end
                .checked_next_multiple_of(alignment)
                .ok_or(GpuError::Capacity)?;
            next[profile as usize] = offset;
            end = offset
                .checked_add(plan.state_bytes)
                .ok_or(GpuError::Capacity)?;
            workgroups = workgroups
                .checked_add(plan.workgroups)
                .ok_or(GpuError::Capacity)?;
            groups.push(GroupInfo {
                profile,
                environments: count,
                offset,
                bytes: plan.state_bytes,
                workgroups: plan.workgroups,
            });
            last_plan = Some(plan);
        }
        let mut plan = last_plan.ok_or(GpuError::EmptyBatch)?;
        plan.state_bytes = end;
        plan.workgroups = workgroups;
        plan.readback_bytes = end
            .checked_add(u64::from(plan.query_count) * 8)
            .ok_or(GpuError::Capacity)?;
        if plan.readback_bytes > limits.max_buffer_size
            || usize::try_from(plan.readback_bytes).is_err()
        {
            return Err(GpuError::Capacity);
        }
        let offsets = initial
            .iter()
            .map(|env| {
                let cursor = &mut next[env.state.profile() as usize];
                let offset = *cursor as usize;
                *cursor += size_of_val(env.state.values()) as u64;
                offset
            })
            .collect();
        Ok(Self {
            groups,
            offsets,
            plan,
        })
    }
}

#[cfg(feature = "native-handoff")]
type StagedNative = BTreeMap<usize, native::NativePair>;
#[cfg(not(feature = "native-handoff"))]
type StagedNative = ();

struct StartedTick {
    start: Instant,
    next: u64,
    metrics: StepMetrics,
    submitted: Option<Result<device::Submission, GpuError>>,
}
/// An owning identity for validating runtime bindings across world lifecycle operations.
#[derive(Debug, Clone)]
pub struct MixedBatchOwner(Arc<()>);
impl MixedBatchOwner {
    pub(crate) fn fresh() -> Self {
        Self(Arc::new(()))
    }
}
impl PartialEq for MixedBatchOwner {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}
impl Eq for MixedBatchOwner {}
/// All registered environments advance at one 1 ms barrier. Each occupied family
/// gets one dispatch into its own aligned range, followed by one shared copy and fence.
#[derive(Resource)]
pub struct MixedBatch {
    owner: MixedBatchOwner,
    #[cfg(feature = "native-handoff")]
    native: Option<native::Registry>,
    fixed_scene: bool,
    placement: Option<ActiveLayout>,
    initial_placement: Option<ActiveLayout>,
    gpu: DeviceIo,
    dispatches: Vec<Dispatch>,
    groups: Vec<GroupInfo>,
    offsets: Vec<usize>,
    pack: Vec<u8>,
    initial: Vec<Environment>,
    committed: Vec<Environment>,
    scratch: Vec<Environment>,
    tick: u64,
    upload_needed: bool,
    dirty: BTreeSet<usize>,
    pending: BTreeMap<usize, (CentralImpulse, EnvironmentState)>,
}
impl MixedBatch {
    pub fn new(adapter_name: &str, mut initial: Vec<Environment>) -> Result<Self, GpuError> {
        if !cfg!(target_endian = "little") {
            return Err(GpuError::ByteOrder);
        }
        let selector = adapter_name.trim().to_lowercase();
        if selector.is_empty() {
            return Err(GpuError::EmptySelector);
        }
        if initial.is_empty() {
            return Err(GpuError::EmptyBatch);
        }
        initial.sort_unstable_by_key(|env| env.id);
        for pair in initial.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(GpuError::DuplicateEnvironment(pair[0].id));
            }
        }
        for env in &initial {
            if !env.state.valid() {
                return Err(GpuError::InvalidEnvironment {
                    id: env.id,
                    profile: env.state.profile(),
                });
            }
        }
        let adapter = device::adapter(&selector)?;
        let Layout {
            groups,
            offsets,
            plan,
        } = Layout::new(
            &initial,
            adapter.features(),
            adapter.get_downlevel_capabilities().flags,
            &adapter.limits(),
        )?;
        let gpu = DeviceIo::new(adapter, plan, 0)?;
        let mut dispatches = Vec::with_capacity(groups.len());
        for g in &groups {
            let shader = match g.profile {
                Profile::SpherePlane => sphere_plane::module,
                Profile::SpherePair => sphere_pair::module,
                Profile::SupportedSpherePair => supported_pair::module,
            };
            dispatches.push(gpu.dispatch(
                shader,
                g.offset,
                g.bytes,
                g.bytes / g.environments as u64,
                g.workgroups,
                None,
            )?);
        }
        let pack = vec![0; gpu.info.state_bytes as usize];
        Ok(Self {
            #[cfg(feature = "native-handoff")]
            native: None,
            owner: MixedBatchOwner::fresh(),
            fixed_scene: false,
            placement: None,
            initial_placement: None,
            gpu,
            dispatches,
            groups,
            offsets,
            pack,
            committed: initial.clone(),
            scratch: initial.clone(),
            initial,
            tick: 0,
            upload_needed: true,
            dirty: BTreeSet::new(),
            pending: BTreeMap::new(),
        })
    }
    /// The SDF loader owns this population's fixed geometry, inertias and starting poses.
    /// Construction-frame reset and queued center impulses remain available.
    pub fn from_fixed_scene(adapter: &str, initial: Vec<Environment>) -> Result<Self, GpuError> {
        let mut batch = Self::new(adapter, initial)?;
        batch.fixed_scene = true;
        Ok(batch)
    }
    /// Identity for detecting replacement of the physical owner at a Gazebo lifecycle boundary.
    pub fn owner(&self) -> MixedBatchOwner {
        self.owner.clone()
    }
    /// Committed observations in ascending stable environment ID order.
    pub fn environments(&self) -> &[Environment] {
        &self.committed
    }
    pub fn get(&self, id: u64) -> Option<&EnvironmentState> {
        self.committed
            .binary_search_by_key(&id, |env| env.id)
            .ok()
            .map(|i| &self.committed[i].state)
    }
    /// Look up an observation batch in caller order from the complete committed host frame.
    pub fn get_many(&self, ids: &[u64]) -> Result<Vec<&Environment>, GpuError> {
        ids.iter()
            .map(|id| self.index(*id).map(|index| &self.committed[index]))
            .collect()
    }
    pub fn tick(&self) -> u64 {
        self.tick
    }
    pub fn sim_time(&self) -> Duration {
        Duration::from_millis(self.tick)
    }
    pub fn device_info(&self) -> &GpuInfo {
        &self.gpu.info
    }
    /// Allocated host bytes for the initial, committed and scratch environment
    /// states, upload slab, offset tables and host-side dispatch/placement vectors.
    /// Driver allocations and sparse command bookkeeping are accounted separately.
    pub fn host_state_bytes(&self) -> usize {
        (self.initial.capacity() + self.committed.capacity() + self.scratch.capacity())
            * size_of::<Environment>()
            + self.pack.capacity()
            + self.offsets.capacity() * size_of::<usize>()
            + self.groups.capacity() * size_of::<GroupInfo>()
            + self.dispatches.capacity() * size_of::<Dispatch>()
            + [&self.initial_placement, &self.placement]
                .into_iter()
                .flatten()
                .map(|layout| {
                    (layout.offsets.capacity() + layout.cpu.capacity()) * size_of::<usize>()
                        + layout.groups.capacity() * size_of::<GroupInfo>()
                        + layout.dispatches.capacity() * size_of::<Dispatch>()
                })
                .sum::<usize>()
    }
    pub fn groups(&self) -> &[GroupInfo] {
        self.placement
            .as_ref()
            .map_or(&self.groups, |layout| &layout.groups)
    }
    pub fn allocator_usage(&self) -> Option<(u64, u64)> {
        self.gpu
            .device
            .generate_allocator_report()
            .map(|r| (r.total_allocated_bytes, r.total_reserved_bytes))
    }
    pub fn reset(&mut self) {
        #[cfg(feature = "native-handoff")]
        if let Some(native) = &mut self.native {
            native.reset();
        }
        self.placement = self.initial_placement.clone();
        self.committed.copy_from_slice(&self.initial);
        self.tick = 0;
        self.upload_needed = true;
        self.dirty.clear();
        self.pending.clear();
    }
    fn stage_cpu(&mut self, metrics: &mut StepMetrics) -> Result<StagedNative, GpuError> {
        #[cfg(feature = "native-handoff")]
        let mut prepared = BTreeMap::new();
        #[cfg(not(feature = "native-handoff"))]
        let prepared = ();
        if let Some(layout) = &self.placement {
            let cpu = Instant::now();
            #[cfg(feature = "native-handoff")]
            let mut staged = self
                .native
                .as_ref()
                .and_then(|set| set.stage_parallel(&self.committed, &self.pending));
            for &i in &layout.cpu {
                let state = self
                    .pending
                    .get(&i)
                    .map_or(self.committed[i].state, |(_, candidate)| *candidate);
                #[cfg(feature = "native-handoff")]
                let result = if let Some(native) =
                    self.native.as_ref().and_then(|set| set.active.get(&i))
                {
                    let (body, outcome) = match staged.as_mut().and_then(|rows| rows.remove(&i)) {
                        Some(result) => result?,
                        None => native.advance(self.committed[i].id, state)?,
                    };
                    prepared.insert(i, body);
                    outcome
                } else {
                    placement::cpu_step(state)
                };
                #[cfg(not(feature = "native-handoff"))]
                let result = placement::cpu_step(state);
                self.scratch[i].state = result;
                if !self.scratch[i].state.values().iter().all(|v| v.is_finite()) {
                    return Err(GpuError::NonfiniteEnvironment {
                        id: self.committed[i].id,
                        profile: state.profile(),
                    });
                }
            }
            metrics.cpu_solve = cpu.elapsed();
        }
        Ok(prepared)
    }
    fn submit_gpu(&mut self, metrics: &mut StepMetrics) -> Result<device::Submission, GpuError> {
        self.gpu.check()?;
        self.upload_changes(metrics);
        // Once a submission can have modified resident storage, every failure
        // must retry by uploading the last committed host states plus pending inputs.
        self.upload_needed = true;
        let groups = self
            .placement
            .as_ref()
            .map_or(&self.dispatches, |layout| &layout.dispatches);
        Ok(self
            .gpu
            .submit_prefix(groups, self.active_gpu_bytes(), metrics))
    }
    fn finish_gpu(
        &mut self,
        job: device::Submission,
        metrics: &mut StepMetrics,
    ) -> Result<(), GpuError> {
        let offsets = self
            .placement
            .as_ref()
            .map_or(&self.offsets, |layout| &layout.offsets);
        self.gpu.finish_prefix(job, metrics, |bytes| {
            for (env, &offset) in self.scratch.iter_mut().zip(offsets) {
                if offset == placement::CPU {
                    continue;
                }
                let dst = env.state.bytes_mut();
                dst.copy_from_slice(&bytes[offset..offset + dst.len()]);
                if !env.state.values().iter().all(|x| x.is_finite()) {
                    return Err(GpuError::NonfiniteEnvironment {
                        id: env.id,
                        profile: env.state.profile(),
                    });
                }
            }
            Ok(())
        })
    }
    fn begin_tick(&mut self) -> Result<StartedTick, GpuError> {
        let start = Instant::now();
        let next = self.tick.checked_add(1).ok_or(GpuError::Clock)?;
        #[cfg(feature = "native-handoff")]
        let overlap = self
            .native
            .as_ref()
            .is_some_and(|s| s.overlap_gpu && !s.active.is_empty())
            && self.active_gpu_bytes() > 0;
        #[cfg(not(feature = "native-handoff"))]
        let overlap = false;
        let mut metrics = StepMetrics::default();
        let submitted = overlap.then(|| self.submit_gpu(&mut metrics));
        Ok(StartedTick {
            start,
            next,
            metrics,
            submitted,
        })
    }
    pub fn step(&mut self) -> Result<StepMetrics, GpuError> {
        let tick = self.begin_tick()?;
        self.finish_tick(tick)
    }
    fn finish_tick(&mut self, tick: StartedTick) -> Result<StepMetrics, GpuError> {
        let StartedTick {
            start,
            next,
            mut metrics,
            submitted,
        } = tick;
        let host = self.stage_cpu(&mut metrics);
        if matches!(submitted, Some(Ok(_))) {
            metrics.cpu_between_submit_and_fence = metrics.cpu_solve;
        }
        let job = submitted.or_else(|| {
            (host.is_ok() && self.active_gpu_bytes() > 0).then(|| self.submit_gpu(&mut metrics))
        });
        // Drain an early submission even if the host rejected a row. Both paths
        // give the same precedence to the first deterministic host-side error.
        let device = job.map_or(Ok(()), |result| {
            result.and_then(|job| self.finish_gpu(job, &mut metrics))
        });
        #[cfg(feature = "native-handoff")]
        let prepared = host?;
        #[cfg(not(feature = "native-handoff"))]
        host?;
        device?;
        #[cfg(feature = "native-handoff")]
        if let Some(set) = &mut self.native {
            set.active = prepared;
        }
        std::mem::swap(&mut self.scratch, &mut self.committed);
        self.tick = next;
        self.dirty.clear();
        self.pending.clear();
        self.upload_needed = false;
        metrics.latency = start.elapsed();
        Ok(metrics)
    }
}

#[cfg(test)]
#[path = "../tests/private/mixed.rs"]
mod tests;
