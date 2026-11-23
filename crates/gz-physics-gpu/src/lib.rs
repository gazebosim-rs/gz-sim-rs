// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Headless f64 Vulkan compute for independent cable/receiver or oblique sphere/plane environments.
mod device;
use device::{DeviceIo, Dispatch};
mod kernel;
mod mixed;
#[cfg(feature = "native-handoff")]
pub use mixed::native::{
    MAX_NATIVE_PAIRS, MAX_NATIVE_WORKERS, NativeContact, NativeHandoffRefusal, PairSolver,
};
pub use mixed::{
    BodySlot, CentralImpulse, Environment, EnvironmentState, ExecutionPlacement, GroupInfo,
    MixedBatch, MixedBatchOwner, MixedCheckpoint, PlacementChange, Profile,
};
mod arithmetic;
mod pair_partition;
pub mod pair_process;
mod pair_rotation;
pub use pair_partition::{
    PairBlockAssignment, PairBlockBarrier, PairBlockContinuation, PairBlockExecutor,
    PairBlockReceipt,
};
mod spatial;
mod sphere_block;
mod sphere_pair;
mod sphere_pair_block;
pub use pair_rotation::SphereRotation;
pub use sphere_pair_block::{
    SpherePairBlockBatch, SpherePairBlockCheckpoint, SpherePairBlockObservation,
};
mod supported_pair;
pub use sphere_pair::{SolidSphere, SpherePairBatch, SpherePairLane};
pub use supported_pair::{SupportedSpherePairBatch, SupportedSpherePairLane};
mod sphere_plane;
pub use sphere_block::{MAX_BLOCK_TICKS, SpherePlaneBlockBatch, SpherePlaneBlockObservation};
pub use sphere_plane::SpherePlaneLane;
pub mod sampling;
pub mod simulation;

use bevy_ecs::prelude::Resource;
use bytemuck::{Pod, Zeroable};
use std::{
    borrow::Cow,
    mem::size_of,
    sync::{Arc, OnceLock, mpsc},
    time::{Duration, Instant},
};

/// Public step duration shared by the device kernel and Bevy resource.
pub const STEP: Duration = Duration::from_millis(1);
const GROUP_WIDTH: u32 = 64;

/// One independent, vertical SI fixture. Velocities and impulses are signed world-Z components.
/// Positive inverse masses are in kg⁻¹. Compliance is in m/N.
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
#[repr(C)]
pub struct Lane {
    pub anchor_z: f64,
    pub anchor_v: f64,
    pub bob_z: f64,
    pub bob_v: f64,
    pub rigid_z: f64,
    pub rigid_v: f64,
    pub rest_length: f64,
    pub compliance: f64,
    pub bob_inverse_mass: f64,
    pub rigid_inverse_mass: f64,
    pub combined_radius: f64,
    pub last_receiver_impulse_z: f64,
}
const _: () = assert!(size_of::<Lane>() == 12 * size_of::<f64>());

impl Lane {
    fn finite(self) -> bool {
        bytemuck::cast_ref::<Lane, [f64; 12]>(&self)
            .iter()
            .all(|v| v.is_finite())
    }

    fn initial_valid(self) -> bool {
        let h = STEP.as_secs_f64();
        // Check the actual native denominators and unforced initial separation.
        let span = self.bob_z - self.anchor_z;
        let clearance = self.rigid_z - self.bob_z;
        self.finite()
            && span.is_finite()
            && span > 0.0
            && clearance.is_finite()
            && clearance >= self.combined_radius
            && self.rest_length > 0.0
            && self.compliance >= 0.0
            && self.combined_radius > 0.0
            && self.bob_inverse_mass > 0.0
            && self.bob_inverse_mass.recip().is_finite()
            && self.rigid_inverse_mass > 0.0
            && self.rigid_inverse_mass.recip().is_finite()
            && (self.bob_inverse_mass + self.rigid_inverse_mass).is_finite()
            && (self.bob_inverse_mass + self.compliance / (h * h)).is_finite()
    }
}

/// Host phase times are elapsed wall durations. GPU counters are measured on
/// the device timeline and overlap the host's submission/wait intervals.
#[derive(Clone, Copy, Debug, Default)]
pub struct StepMetrics {
    /// Borrow the committed contiguous ABI records for upload; zero on a resident tick.
    pub pack: Duration,
    /// Wall time advancing CPU-owned cells in a manually placed mixed batch.
    pub cpu_solve: Duration,
    /// CPU staging elapsed after a GPU submission and before its host fence.
    /// Commands can already have finished on the device during this host interval.
    pub cpu_between_submit_and_fence: Duration,
    /// Host staging via queue.write_buffer, including the host-side copy.
    pub upload: Duration,
    pub encode_submit: Duration,
    /// Wait for this submission, its copy and the dense readback map callback.
    pub wait_map: Duration,
    /// Copy observations into host storage, verify finiteness, and unmap.
    pub decode: Duration,
    pub latency: Duration,
    pub gpu_dispatch: Option<Duration>,
    /// GPU copy of dense physics records, when encoder timestamps are supported.
    pub gpu_copy: Option<Duration>,
    pub uploaded_bytes: u64,
    /// Dense state and optional device timestamps transferred together.
    pub readback_bytes: u64,
}

#[derive(Clone, Debug)]
pub struct GpuInfo {
    pub name: String,
    pub backend: String,
    pub driver: String,
    pub driver_info: String,
    pub vendor_id: u32,
    pub device_id: u32,
    pub timestamp_queries: bool,
    pub timestamp_period_ns: Option<f32>,
    /// Explicit state, readback and timestamp resolve buffers; excludes driver allocations.
    pub allocated_buffer_bytes: u64,
    pub state_bytes: u64,
    pub workgroups: u32,
}

#[derive(Debug, thiserror::Error)]
pub enum GpuError {
    #[cfg(feature = "native-handoff")]
    #[error(
        "native staging requires 1 to 4 workers in a fresh world with registered native contacts"
    )]
    NativeStaging,
    #[cfg(feature = "native-handoff")]
    #[error("could not create native staging worker threads")]
    NativeStagingUnavailable,
    #[error("GPU batch needs a nonempty hardware adapter name selector")]
    EmptySelector,
    #[error("GPU batch needs at least one lane")]
    EmptyBatch,
    #[error("initial lane {0} violates the fixture's numeric or geometry bounds")]
    InvalidLane(usize),
    #[error("Vulkan hardware adapter matching '{0}' was not found")]
    NoAdapter(String),
    #[error("more than one Vulkan hardware adapter matches '{0}'")]
    AmbiguousAdapter(String),
    #[error("the selected Vulkan adapter lacks native f64 compute support")]
    MissingF64,
    #[error("batch exceeds the selected adapter or kernel addressing limits")]
    Capacity,
    #[error("unsupported host byte order for native f64 transfer")]
    ByteOrder,
    #[error("GPU device or dispatch failed: {0}")]
    Device(String),
    #[error("GPU readback failed: {0}")]
    Readback(String),
    #[error("GPU lane {0} produced a nonfinite state")]
    Nonfinite(usize),
    #[error("duplicate GPU environment identity {0}")]
    DuplicateEnvironment(u64),
    #[error("initial environment {id} violates the {profile:?} profile")]
    InvalidEnvironment { id: u64, profile: Profile },
    #[error("GPU environment {id} produced a nonfinite {profile:?} observation")]
    NonfiniteEnvironment { id: u64, profile: Profile },
    #[error("environment {0} is absent from this mixed population")]
    UnknownEnvironment(u64),
    #[error(
        "the loaded SDF world owns this batch's geometry and configuration; use whole-world reset"
    )]
    FixedScene,
    #[error("Gazebo link {0} has no dynamic GPU body in the loaded world")]
    UnknownSceneLink(u64),
    #[error("selected reset cannot change the registered profile for environment {0}")]
    ProfileChange(u64),
    #[error("invalid center impulse or unsupported body slot for environment {0}")]
    InvalidAction(u64),
    #[error("the requested mixed-batch epoch or iteration differs from the committed clock")]
    StaleBoundary,
    #[error("a committed mixed-batch checkpoint was refused: {0}")]
    Checkpoint(String),
    #[cfg(feature = "native-handoff")]
    #[error("native/GPU handoff for environment {id} refused: {reason}")]
    NativeHandoff {
        id: u64,
        reason: mixed::native::NativeHandoffRefusal,
    },
    #[error("GPU batch simulation clock exhausted")]
    Clock,
    #[error("resident block length {0} must be between 1 and {MAX_BLOCK_TICKS} ticks")]
    BlockLength(u32),
    #[error("lane {lane} produced a nonfinite intermediate at block offset {offset}")]
    InvalidIntermediate { lane: usize, offset: u32 },
}

#[derive(Clone, Copy)]
enum ObservationKernel {
    Plane,
    Pair(SphereRotation),
}
struct BlockSetup<T> {
    kernel: ObservationKernel,
    invalid_at: fn(&T) -> Option<u32>,
}
struct BlockControl<T> {
    kernel: ObservationKernel,
    last_ticks: u32,
    count: wgpu::Buffer,
    invalid_at: fn(&T) -> Option<u32>,
}
struct Queries {
    set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    count: u32,
    period_ns: f32,
}

/// A resident batch with a dense, transactional host snapshot after each successful
/// step. A failed readback or invalid observation leaves the public state and time
/// unchanged, and any subsequent retry starts from the committed host snapshot.
#[derive(Resource)]
pub struct ResidentBatch<T: Pod + Send + Sync> {
    owner: MixedBatchOwner,
    gpu: DeviceIo,
    dispatch: Dispatch,
    block_control: Option<BlockControl<T>>,
    initial: Vec<T>,
    committed: Vec<T>,
    scratch: Vec<T>,
    tick: u64,
    upload_needed: bool,
}

pub type GpuBatch = ResidentBatch<Lane>;
pub type SpherePlaneBatch = ResidentBatch<SpherePlaneLane>;

fn buffer(
    device: &wgpu::Device,
    label: &str,
    size: u64,
    usage: wgpu::BufferUsages,
) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size,
        usage,
        mapped_at_creation: false,
    })
}

fn select_hardware(
    selector: &str,
    adapters: impl Iterator<Item = (wgpu::DeviceType, String)>,
) -> Result<usize, GpuError> {
    let mut matches = adapters.enumerate().filter(|(_, (kind, name))| {
        matches!(
            kind,
            wgpu::DeviceType::DiscreteGpu | wgpu::DeviceType::IntegratedGpu
        ) && name.to_lowercase().contains(selector)
    });
    let (index, _) = matches
        .next()
        .ok_or_else(|| GpuError::NoAdapter(selector.to_owned()))?;
    if matches.next().is_some() {
        return Err(GpuError::AmbiguousAdapter(selector.to_owned()));
    }
    Ok(index)
}

// Size both the storage binding and the larger observation mapping before
// creating the device. Timestamps can overflow a buffer which exactly fits
// the physics records; disabling all timing must still allow f64 compute.
struct DevicePlan {
    required_features: wgpu::Features,
    query_count: u32,
    state_bytes: u64,
    readback_bytes: u64,
    workgroups: u32,
}

impl DevicePlan {
    fn new<T>(
        lane_count: usize,
        features: wgpu::Features,
        downlevel: wgpu::DownlevelFlags,
        limits: &wgpu::Limits,
    ) -> Result<Self, GpuError> {
        if !features.contains(wgpu::Features::SHADER_F64)
            || !downlevel.contains(wgpu::DownlevelFlags::COMPUTE_SHADERS)
        {
            return Err(GpuError::MissingF64);
        }
        let lanes = u32::try_from(lane_count).map_err(|_| GpuError::Capacity)?;
        lanes
            .checked_mul((size_of::<T>() / size_of::<f64>()) as u32)
            .ok_or(GpuError::Capacity)?;
        let state_bytes = u64::from(lanes) * size_of::<T>() as u64;
        let workgroups = lanes.div_ceil(GROUP_WIDTH);

        let mut required_features = wgpu::Features::SHADER_F64;
        let query_count = if features.contains(wgpu::Features::TIMESTAMP_QUERY) {
            required_features |= wgpu::Features::TIMESTAMP_QUERY;
            if features.contains(wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS) {
                required_features |= wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS;
                4
            } else {
                2
            }
        } else {
            0
        };
        let query_bytes = u64::from(query_count) * 8;
        let readback_bytes = state_bytes + query_bytes;
        if state_bytes > limits.max_storage_buffer_binding_size
            || readback_bytes > limits.max_buffer_size
            || workgroups > limits.max_compute_workgroups_per_dimension
            || GROUP_WIDTH > limits.max_compute_workgroup_size_x
            || GROUP_WIDTH > limits.max_compute_invocations_per_workgroup
            || limits.max_storage_buffers_per_shader_stage == 0
        {
            return Err(GpuError::Capacity);
        }
        Ok(Self {
            required_features,
            query_count,
            state_bytes,
            readback_bytes,
            workgroups,
        })
    }
}

impl GpuBatch {
    pub fn new(adapter_name: &str, initial: Vec<Lane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter_name,
            initial,
            Lane::initial_valid,
            kernel::module,
            None,
        )
    }
}
impl SpherePlaneBatch {
    pub fn new(adapter_name: &str, initial: Vec<SpherePlaneLane>) -> Result<Self, GpuError> {
        Self::new_profile(
            adapter_name,
            initial,
            SpherePlaneLane::initial_valid,
            sphere_plane::module,
            None,
        )
    }
}
impl<T: Pod + Send + Sync> ResidentBatch<T> {
    fn new_profile(
        adapter_name: &str,
        initial: Vec<T>,
        valid: fn(T) -> bool,
        shader_module: fn() -> naga::Module,
        block: Option<BlockSetup<T>>,
    ) -> Result<Self, GpuError> {
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
        if let Some(id) = initial.iter().position(|lane| !valid(*lane)) {
            return Err(GpuError::InvalidLane(id));
        }
        let adapter = device::adapter(&selector)?;
        let plan = DevicePlan::new::<T>(
            initial.len(),
            adapter.features(),
            adapter.get_downlevel_capabilities().flags,
            &adapter.limits(),
        )?;
        let gpu = DeviceIo::new(adapter, plan, if block.is_some() { 4 } else { 0 })?;
        let block_control = block.map(|setup| BlockControl {
            last_ticks: 0,
            kernel: setup.kernel,
            count: buffer(
                &gpu.device,
                "exact resident tick count",
                4,
                wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            ),
            invalid_at: setup.invalid_at,
        });
        let dispatch = gpu.dispatch(
            shader_module,
            0,
            gpu.info.state_bytes,
            size_of::<T>() as u64,
            gpu.info.workgroups,
            block_control.as_ref().map(|v| &v.count),
        )?;
        Ok(Self {
            owner: MixedBatchOwner::fresh(),
            gpu,
            dispatch,
            block_control,
            committed: initial.clone(),
            scratch: initial.clone(),
            initial,
            tick: 0,
            upload_needed: true,
        })
    }

    /// Stable identity for binding public scene links to this runtime batch.
    pub fn owner(&self) -> MixedBatchOwner {
        self.owner.clone()
    }

    pub fn lanes(&self) -> &[T] {
        &self.committed
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

    /// Current Vulkan allocator totals (allocated, reserved), in bytes, when
    /// supplied by the backend. Sample outside timing loops. These per-device
    /// values include allocator overhead and may exceed the explicit payloads.
    pub fn allocator_usage(&self) -> Option<(u64, u64)> {
        self.gpu
            .device
            .generate_allocator_report()
            .map(|report| (report.total_allocated_bytes, report.total_reserved_bytes))
    }

    /// Restore the construction snapshot immediately; the next tick uploads it
    /// before dispatch, superseding any old device state.
    pub fn reset(&mut self) {
        self.committed.copy_from_slice(&self.initial);
        self.tick = 0;
        if let Some(control) = &mut self.block_control {
            control.last_ticks = 0;
        }
        self.upload_needed = true;
    }

    pub fn step(&mut self) -> Result<StepMetrics, GpuError> {
        self.advance(1)
    }
    fn advance(&mut self, ticks: u32) -> Result<StepMetrics, GpuError> {
        let start = Instant::now();
        let next = self
            .tick
            .checked_add(u64::from(ticks))
            .ok_or(GpuError::Clock)?;
        self.gpu.check()?;
        let mut metrics = StepMetrics::default();
        if self.upload_needed {
            let pack = Instant::now();
            let bytes = bytemuck::cast_slice(&self.committed);
            metrics.pack = pack.elapsed();
            let upload = Instant::now();
            self.gpu.queue.write_buffer(&self.gpu.state, 0, bytes);
            metrics.upload = upload.elapsed();
            metrics.uploaded_bytes = self.gpu.info.state_bytes;
        }
        if let Some(control) = &self.block_control {
            let upload = Instant::now();
            self.gpu
                .queue
                .write_buffer(&control.count, 0, &ticks.to_le_bytes());
            metrics.upload += upload.elapsed();
            metrics.uploaded_bytes += 4;
        }
        // Any exit before publication makes the next attempt upload the last
        // committed state, even if the preceding dispatch actually completed.
        self.upload_needed = true;
        let scratch = &mut self.scratch;
        let block_control = &self.block_control;
        self.gpu.run(
            std::slice::from_ref(&self.dispatch),
            &mut metrics,
            |lanes| {
                bytemuck::cast_slice_mut(scratch).copy_from_slice(lanes);
                if let Some(control) = block_control
                    && let Some((lane, offset)) = scratch
                        .iter()
                        .enumerate()
                        .find_map(|(i, v)| (control.invalid_at)(v).map(|offset| (i, offset)))
                {
                    return Err(GpuError::InvalidIntermediate { lane, offset });
                }
                if let Some(i) = scratch.iter().position(|lane| {
                    !bytemuck::cast_slice::<T, f64>(std::slice::from_ref(lane))
                        .iter()
                        .all(|v| v.is_finite())
                }) {
                    return Err(GpuError::Nonfinite(i));
                }
                Ok(())
            },
        )?;
        std::mem::swap(&mut self.scratch, &mut self.committed);
        self.tick = next;
        if let Some(control) = &mut self.block_control {
            control.last_ticks = ticks;
        }
        self.upload_needed = false;
        metrics.latency = start.elapsed();
        Ok(metrics)
    }
}

fn interval(pair: &[u8], period_ns: f32) -> Result<Duration, GpuError> {
    let a = u64::from_le_bytes(pair[..8].try_into().expect("timestamp width"));
    let b = u64::from_le_bytes(pair[8..].try_into().expect("timestamp width"));
    let ticks = b
        .checked_sub(a)
        .ok_or_else(|| GpuError::Readback("device timestamp moved backwards".into()))?;
    Duration::try_from_secs_f64(ticks as f64 * f64::from(period_ns) * 1e-9)
        .map_err(|e| GpuError::Readback(e.to_string()))
}

#[cfg(test)]
#[path = "../tests/private/host.rs"]
mod tests;
