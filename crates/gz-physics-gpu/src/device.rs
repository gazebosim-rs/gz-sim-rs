// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared device, command submission and transactional dense readback.
use super::*;
#[derive(Clone)]
pub(super) struct Dispatch {
    pipeline: wgpu::ComputePipeline,
    bindings: wgpu::BindGroup,
    workgroups: u32,
}
/// The queue owns the compute and copy. Mapping starts only at the public fence.
/// A submission must be drained even when an independent host stage rejects the tick.
#[must_use]
pub(super) struct Submission {
    index: wgpu::SubmissionIndex,
    state_bytes: u64,
    read_bytes: u64,
}
pub(super) struct DeviceIo {
    pub(super) device: wgpu::Device,
    pub(super) queue: wgpu::Queue,
    pub(super) state: wgpu::Buffer,
    pub(super) readback: wgpu::Buffer,
    queries: Option<Queries>,
    pub(super) failure: Arc<OnceLock<String>>,
    pub(super) info: GpuInfo,
}
pub(super) fn adapter(selector: &str) -> Result<wgpu::Adapter, GpuError> {
    let mut descriptor = wgpu::InstanceDescriptor::new_without_display_handle();
    descriptor.backends = wgpu::Backends::VULKAN;
    let instance = wgpu::Instance::new(descriptor);
    let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::VULKAN));
    let infos = adapters.iter().map(|adapter| {
        let info = adapter.get_info();
        (info.device_type, info.name)
    });
    let index = select_hardware(selector, infos)?;
    Ok(adapters
        .into_iter()
        .nth(index)
        .expect("selected an enumerated adapter"))
}
impl DeviceIo {
    pub(super) fn new(
        adapter: wgpu::Adapter,
        plan: DevicePlan,
        extra_bytes: u64,
    ) -> Result<Self, GpuError> {
        let limits = adapter.limits();
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("Gazebo f64 compute"),
            required_features: plan.required_features,
            required_limits: limits,
            ..Default::default()
        }))
        .map_err(|error| GpuError::Device(error.to_string()))?;
        let failure = Arc::new(OnceLock::new());
        let lost = Arc::clone(&failure);
        device.set_device_lost_callback(move |reason, message| {
            let _ = lost.set(format!("{reason:?}: {message}"));
        });
        let errors = Arc::clone(&failure);
        device.on_uncaptured_error(Arc::new(move |error: wgpu::Error| {
            let _ = errors.set(error.to_string());
        }));
        Self::buffers(device, queue, failure, plan, extra_bytes)
    }
    /// Allocate a second physics layout on this already qualified compute device and queue.
    pub(super) fn companion<T>(&self, lanes: usize) -> Result<Self, GpuError> {
        let plan = DevicePlan::new::<T>(
            lanes,
            self.device.features(),
            wgpu::DownlevelFlags::COMPUTE_SHADERS,
            &self.device.limits(),
        )?;
        Self::buffers(
            self.device.clone(),
            self.queue.clone(),
            self.failure.clone(),
            plan,
            0,
        )
    }
    fn buffers(
        device: wgpu::Device,
        queue: wgpu::Queue,
        failure: Arc<OnceLock<String>>,
        plan: DevicePlan,
        extra_bytes: u64,
    ) -> Result<Self, GpuError> {
        let adapter_info = device.adapter_info();
        let DevicePlan {
            query_count,
            state_bytes,
            readback_bytes,
            workgroups,
            ..
        } = plan;
        let query_bytes = u64::from(query_count) * 8;
        let state = buffer(
            &device,
            "resident f64 lanes",
            state_bytes,
            wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        );
        let readback = buffer(
            &device,
            "dense committed readback",
            readback_bytes,
            wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        );
        let queries = (query_count != 0).then(|| Queries {
            set: device.create_query_set(&wgpu::QuerySetDescriptor {
                label: Some("tick timestamps"),
                ty: wgpu::QueryType::Timestamp,
                count: query_count,
            }),
            resolve: buffer(
                &device,
                "timestamp resolve",
                query_bytes,
                wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
            ),
            count: query_count,
            period_ns: queue.get_timestamp_period(),
        });
        if let Some(error) = failure.get() {
            return Err(GpuError::Device(error.clone()));
        }
        let info = GpuInfo {
            name: adapter_info.name,
            backend: format!("{:?}", adapter_info.backend),
            driver: adapter_info.driver,
            driver_info: adapter_info.driver_info,
            vendor_id: adapter_info.vendor,
            device_id: adapter_info.device,
            timestamp_queries: queries.is_some(),
            timestamp_period_ns: queries.as_ref().map(|q| q.period_ns),
            allocated_buffer_bytes: state_bytes + readback_bytes + query_bytes + extra_bytes,
            state_bytes,
            workgroups,
        };

        Ok(Self {
            device,
            queue,
            state,
            readback,
            queries,
            failure,
            info,
        })
    }
    pub(super) fn dispatch(
        &self,
        shader_module: fn() -> naga::Module,
        offset: u64,
        bytes: u64,
        stride: u64,
        workgroups: u32,
        count: Option<&wgpu::Buffer>,
    ) -> Result<Dispatch, GpuError> {
        let device = &self.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Gazebo f64 physics lanes"),
            source: wgpu::ShaderSource::Naga(Cow::Owned(shader_module())),
        });
        let mut entries = vec![wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(stride),
            },
            count: None,
        }];
        if count.is_some() {
            entries.push(wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(4),
                },
                count: None,
            });
        }
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("physics lanes and optional tick count"),
            entries: &entries,
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("f64 lane layout"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("f64 lane tick"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: Some("step"),
            compilation_options: Default::default(),
            cache: None,
        });
        let mut entries = vec![wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer: &self.state,
                offset,
                size: wgpu::BufferSize::new(bytes),
            }),
        }];
        if let Some(count) = count {
            entries.push(wgpu::BindGroupEntry {
                binding: 1,
                resource: count.as_entire_binding(),
            });
        }
        let bindings = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("resident lane binding"),
            layout: &layout,
            entries: &entries,
        });

        self.check()?;
        Ok(Dispatch {
            pipeline,
            bindings,
            workgroups,
        })
    }
    /// Reuse a physical profile's compiled pipeline and bind its compact active prefix.
    pub(super) fn bind_mixed_range(
        &self,
        template: &Dispatch,
        offset: u64,
        bytes: u64,
        workgroups: u32,
    ) -> Dispatch {
        let layout = template.pipeline.get_bind_group_layout(0);
        let bindings = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("active Gazebo environment range"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &self.state,
                    offset,
                    size: wgpu::BufferSize::new(bytes),
                }),
            }],
        });
        Dispatch {
            pipeline: template.pipeline.clone(),
            bindings,
            workgroups,
        }
    }
    pub(super) fn check(&self) -> Result<(), GpuError> {
        if let Some(error) = self.failure.get() {
            return Err(GpuError::Device(error.clone()));
        }
        Ok(())
    }
    pub(super) fn run(
        &self,
        groups: &[Dispatch],
        metrics: &mut StepMetrics,
        decode_lanes: impl FnOnce(&[u8]) -> Result<(), GpuError>,
    ) -> Result<(), GpuError> {
        self.run_prefix(groups, self.info.state_bytes, metrics, decode_lanes)
    }
    pub(super) fn run_prefix(
        &self,
        groups: &[Dispatch],
        state_bytes: u64,
        metrics: &mut StepMetrics,
        decode_lanes: impl FnOnce(&[u8]) -> Result<(), GpuError>,
    ) -> Result<(), GpuError> {
        let submission = self.submit_prefix(groups, state_bytes, metrics);
        self.finish_prefix(submission, metrics, decode_lanes)
    }
    pub(super) fn submit_prefix(
        &self,
        groups: &[Dispatch],
        state_bytes: u64,
        metrics: &mut StepMetrics,
    ) -> Submission {
        let read_bytes = state_bytes + self.queries.as_ref().map_or(0, |q| u64::from(q.count) * 8);
        let submit = Instant::now();
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Gazebo physics advance"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("f64 dynamics"),
                timestamp_writes: self
                    .queries
                    .as_ref()
                    .map(|q| wgpu::ComputePassTimestampWrites {
                        query_set: &q.set,
                        beginning_of_pass_write_index: Some(0),
                        end_of_pass_write_index: Some(1),
                    }),
            });
            for group in groups {
                pass.set_pipeline(&group.pipeline);
                pass.set_bind_group(0, &group.bindings, &[]);
                pass.dispatch_workgroups(group.workgroups, 1, 1);
            }
        }
        if let Some(q) = &self.queries
            && q.count == 4
        {
            encoder.write_timestamp(&q.set, 2);
        }
        encoder.copy_buffer_to_buffer(&self.state, 0, &self.readback, 0, state_bytes);
        if let Some(q) = &self.queries {
            if q.count == 4 {
                encoder.write_timestamp(&q.set, 3);
            }
            encoder.resolve_query_set(&q.set, 0..q.count, &q.resolve, 0);
            encoder.copy_buffer_to_buffer(
                &q.resolve,
                0,
                &self.readback,
                state_bytes,
                u64::from(q.count) * 8,
            );
        }
        let submission = self.queue.submit([encoder.finish()]);
        metrics.encode_submit = submit.elapsed();

        Submission {
            index: submission,
            state_bytes,
            read_bytes,
        }
    }
    pub(super) fn finish_prefix(
        &self,
        submitted: Submission,
        metrics: &mut StepMetrics,
        decode_lanes: impl FnOnce(&[u8]) -> Result<(), GpuError>,
    ) -> Result<(), GpuError> {
        let Submission {
            index: submission,
            state_bytes,
            read_bytes,
        } = submitted;
        let wait = Instant::now();
        let (tx, rx) = mpsc::sync_channel(1);
        self.readback
            .slice(..read_bytes)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        if let Err(error) = self.device.poll(wgpu::PollType::Wait {
            submission_index: Some(submission),
            timeout: Some(Duration::from_secs(30)),
        }) {
            self.readback.unmap();
            return Err(GpuError::Device(error.to_string()));
        }
        let mapped = rx
            .try_recv()
            .map_err(|e| GpuError::Readback(e.to_string()))
            .and_then(|result| result.map_err(|e| GpuError::Readback(e.to_string())));
        metrics.wait_map = wait.elapsed();
        let decode = Instant::now();
        let result = mapped.and_then(|()| {
            let view = self
                .readback
                .slice(..read_bytes)
                .get_mapped_range()
                .map_err(|e| GpuError::Readback(e.to_string()))?;
            let (lanes, stamps) = view.split_at(state_bytes as usize);
            decode_lanes(lanes)?;
            if let Some(q) = &self.queries {
                let mut pairs = stamps.chunks_exact(16);
                metrics.gpu_dispatch = Some(interval(
                    pairs.next().expect("dispatch query pair"),
                    q.period_ns,
                )?);
                if let Some(copy) = pairs.next() {
                    metrics.gpu_copy = Some(interval(copy, q.period_ns)?);
                }
            }
            Ok(())
        });
        self.readback.unmap();
        result?;
        if let Some(error) = self.failure.get() {
            return Err(GpuError::Device(error.clone()));
        }
        metrics.decode = decode.elapsed();
        metrics.readback_bytes = read_bytes;
        Ok(())
    }
}
