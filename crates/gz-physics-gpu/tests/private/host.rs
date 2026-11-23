// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn sample() -> Lane {
    Lane {
        anchor_z: 0.1,
        anchor_v: 0.05,
        bob_z: 0.35,
        bob_v: 0.05,
        rigid_z: 0.5,
        rigid_v: -0.1,
        rest_length: 0.25,
        compliance: 0.0001,
        bob_inverse_mass: 1.5,
        rigid_inverse_mass: 0.8,
        combined_radius: 0.1,
        last_receiver_impulse_z: 0.0,
    }
}

#[test]
fn refuse_invalid_lanes_and_empty_selection_before_device_request() {
    let seed = sample();
    assert!(seed.initial_valid());
    assert!(matches!(
        GpuBatch::new(" ", vec![seed]),
        Err(GpuError::EmptySelector)
    ));
    assert!(matches!(
        GpuBatch::new("5090", vec![]),
        Err(GpuError::EmptyBatch)
    ));
    for bad in [
        Lane { bob_z: 0.0, ..seed },
        Lane {
            rigid_z: 0.37,
            ..seed
        },
        Lane {
            compliance: -0.1,
            ..seed
        },
        Lane {
            rest_length: 0.0,
            ..seed
        },
        Lane {
            rigid_inverse_mass: 0.0,
            ..seed
        },
        Lane {
            combined_radius: 0.0,
            ..seed
        },
        Lane {
            anchor_v: f64::INFINITY,
            ..seed
        },
        Lane {
            compliance: f64::MAX,
            ..seed
        },
        Lane {
            rigid_inverse_mass: f64::MAX,
            bob_inverse_mass: f64::MAX,
            ..seed
        },
    ] {
        assert!(matches!(
            GpuBatch::new("5090", vec![seed, bad]),
            Err(GpuError::InvalidLane(1))
        ));
    }
}

#[test]
fn timestamp_decode_accepts_optional_subnanosecond_units_and_refuses_backwards_clock() {
    let mut pair = [0_u8; 16];
    pair[..8].copy_from_slice(&17_u64.to_le_bytes());
    pair[8..].copy_from_slice(&29_u64.to_le_bytes());
    assert_eq!(interval(&pair, 0.5).unwrap(), Duration::from_nanos(6));
    assert!(interval(&pair, f32::NAN).is_err());
    pair.swap(0, 8);
    assert!(interval(&pair, 1.).is_err());
}

#[test]
#[ignore = "requires the requested f64 Vulkan hardware"]
fn resident_transfers_reset_and_failed_observation_are_atomic() {
    let selector = std::env::var("GZ_GPU_ADAPTER").expect("set the required hardware name");
    let seed = sample();
    let mut batch = GpuBatch::new(&selector, vec![seed; 65]).unwrap();
    assert_eq!(batch.gpu.info.workgroups, 2);
    let first = batch.step().unwrap();
    assert_eq!(first.uploaded_bytes, 65 * 96);
    assert!(first.readback_bytes >= first.uploaded_bytes);
    let committed = batch.lanes().to_vec();
    assert_eq!(committed[0], committed[64]);
    assert_ne!(committed[64], seed);
    assert_eq!(batch.step().unwrap().uploaded_bytes, 0);
    batch.reset();
    assert_eq!(batch.sim_time(), Duration::ZERO);
    assert_eq!(batch.lanes(), &[seed; 65]);
    assert_eq!(batch.step().unwrap().uploaded_bytes, first.uploaded_bytes);
    assert_eq!(batch.lanes(), committed);
    // Simulate a corrupted resident input after a successful public tick:
    // the shader propagates NaN, and the dense readback must reject it.
    batch
        .gpu
        .queue
        .write_buffer(&batch.gpu.state, 8, &f64::NAN.to_le_bytes());
    assert!(matches!(batch.step(), Err(GpuError::Nonfinite(0))));
    assert_eq!(batch.tick(), 1);
    assert_eq!(batch.lanes(), committed);
    assert_eq!(batch.step().unwrap().uploaded_bytes, first.uploaded_bytes);
    let recovered = batch.lanes().to_vec();
    batch.reset();
    batch.step().unwrap();
    batch.step().unwrap();
    assert_eq!(batch.lanes(), recovered);
}

#[test]
fn hardware_selection_excludes_software_and_rejects_ambiguous_device_names() {
    use wgpu::DeviceType::{Cpu, DiscreteGpu, IntegratedGpu};
    let all = [
        (Cpu, "simulated 5090"),
        (DiscreteGpu, "NVIDIA RTX 5090"),
        (IntegratedGpu, "Integrated GPU"),
    ];
    let devices = || all.iter().map(|(kind, name)| (*kind, name.to_string()));
    assert_eq!(select_hardware("5090", devices()).unwrap(), 1);
    assert_eq!(select_hardware("gpu", devices()).unwrap(), 2);
    assert!(matches!(
        select_hardware("simulated", devices()),
        Err(GpuError::NoAdapter(_))
    ));
    assert!(matches!(
        select_hardware("missing", devices()),
        Err(GpuError::NoAdapter(_))
    ));
    let identical = [
        (DiscreteGpu, "GPU".to_owned()),
        (DiscreteGpu, "GPU".to_owned()),
    ];
    assert!(matches!(
        select_hardware("gpu", identical.into_iter()),
        Err(GpuError::AmbiguousAdapter(_))
    ));
}

#[test]
fn require_f64_compute_and_request_only_the_supported_timing_features() {
    let float = wgpu::Features::SHADER_F64;
    let timestamps = wgpu::Features::TIMESTAMP_QUERY;
    let encoder = wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS;
    let compute = wgpu::DownlevelFlags::COMPUTE_SHADERS;
    let limits = wgpu::Limits::default();
    for (features, capabilities) in [
        (timestamps, compute),
        (float, wgpu::DownlevelFlags::empty()),
    ] {
        assert!(matches!(
            DevicePlan::new::<Lane>(65, features, capabilities, &limits),
            Err(GpuError::MissingF64)
        ));
    }
    for (features, queries) in [
        (float, 0),
        (float | timestamps, 2),
        (float | timestamps | encoder, 4),
    ] {
        let plan = DevicePlan::new::<Lane>(65, features, compute, &limits).unwrap();
        assert_eq!(plan.required_features, features);
        assert_eq!(plan.query_count, queries);
        assert_eq!(plan.workgroups, 2);
        assert_eq!(plan.state_bytes, 65 * 96);
        assert_eq!(
            plan.readback_bytes - plan.state_bytes,
            8 * u64::from(queries)
        );
    }
}

#[test]
fn check_address_space_dispatch_binding_and_timestamp_tail_before_allocation() {
    let features = wgpu::Features::SHADER_F64;
    let compute = wgpu::DownlevelFlags::COMPUTE_SHADERS;
    let base = wgpu::Limits::default();
    let truncated_timestamp = wgpu::Limits {
        max_buffer_size: 65 * 96,
        ..base.clone()
    };
    assert!(DevicePlan::new::<Lane>(65, features, compute, &truncated_timestamp).is_ok());
    assert!(matches!(
        DevicePlan::new::<Lane>(
            65,
            features | wgpu::Features::TIMESTAMP_QUERY,
            compute,
            &truncated_timestamp
        ),
        Err(GpuError::Capacity)
    ));
    for limit in [
        wgpu::Limits {
            max_storage_buffer_binding_size: 65 * 96 - 1,
            ..base.clone()
        },
        wgpu::Limits {
            max_compute_workgroups_per_dimension: 1,
            ..base.clone()
        },
        wgpu::Limits {
            max_compute_workgroup_size_x: 63,
            ..base.clone()
        },
        wgpu::Limits {
            max_compute_invocations_per_workgroup: 32,
            ..base.clone()
        },
        wgpu::Limits {
            max_storage_buffers_per_shader_stage: 0,
            ..base.clone()
        },
    ] {
        assert!(matches!(
            DevicePlan::new::<Lane>(65, features, compute, &limit),
            Err(GpuError::Capacity)
        ));
    }
    assert!(matches!(
        DevicePlan::new::<Lane>(u32::MAX as usize / 12 + 1, features, compute, &base),
        Err(GpuError::Capacity)
    ));
    assert!(matches!(
        DevicePlan::new::<Lane>(usize::MAX, features, compute, &base),
        Err(GpuError::Capacity)
    ));
}

#[test]
#[ignore = "requires the requested f64 Vulkan hardware"]
fn physical_device_loss_preserves_the_last_observation_and_clock() {
    let selector = std::env::var("GZ_GPU_ADAPTER").expect("explicit required hardware");
    let seed = sample();
    let mut batch = GpuBatch::new(&selector, vec![seed]).unwrap();
    assert!(
        batch
            .device_info()
            .name
            .to_lowercase()
            .contains(&selector.to_lowercase())
    );
    batch.step().unwrap();
    if let Some((allocated, reserved)) = batch.allocator_usage() {
        assert!(reserved >= allocated && allocated >= batch.device_info().allocated_buffer_bytes);
    }
    let committed = batch.lanes()[0];
    batch.gpu.device.destroy();
    // Drive the native callback at a public boundary.
    let _ = batch.gpu.device.poll(wgpu::PollType::wait_indefinitely());
    assert!(matches!(batch.step(), Err(GpuError::Device(_))));
    assert_eq!(batch.tick(), 1);
    assert_eq!(batch.lanes(), &[committed]);
    // Reset is a host operation. Loss keeps a retry from publishing a stale tick.
    batch.reset();
    assert!(matches!(batch.step(), Err(GpuError::Device(_))));
    assert_eq!(batch.tick(), 0);
    assert_eq!(batch.lanes(), &[seed]);
}

#[test]
#[ignore = "requires the requested f64 Vulkan hardware"]
fn invalidated_native_copy_target_does_not_publish_an_unread_dispatch() {
    let selector = std::env::var("GZ_GPU_ADAPTER").expect("explicit required hardware");
    let mut batch = GpuBatch::new(&selector, vec![sample()]).unwrap();
    batch.step().unwrap();
    let committed = batch.lanes()[0];
    batch.gpu.readback.destroy();
    assert!(matches!(
        batch.step(),
        Err(GpuError::Device(_) | GpuError::Readback(_))
    ));
    assert_eq!(batch.lanes(), &[committed]);
    assert_eq!(batch.tick(), 1);
    assert!(batch.step().is_err());
    assert_eq!(batch.sim_time(), STEP);
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn split_submission_preserves_optional_timestamp_layouts_and_dense_physics_bytes() {
    let selector = std::env::var("GZ_GPU_ADAPTER").unwrap().to_lowercase();
    let mut baseline = None;
    for timestamps in [false, true] {
        let adapter = device::adapter(&selector).unwrap();
        let features = wgpu::Features::SHADER_F64
            | if timestamps {
                adapter.features() & wgpu::Features::TIMESTAMP_QUERY
            } else {
                wgpu::Features::empty()
            };
        let plan = DevicePlan::new::<Lane>(
            1,
            features,
            adapter.get_downlevel_capabilities().flags,
            &adapter.limits(),
        )
        .unwrap();
        let with_stamps = plan.query_count == 2;
        let bytes = plan.state_bytes;
        let expected = plan.readback_bytes;
        let io = DeviceIo::new(adapter, plan, 0).unwrap();
        let dispatch = io
            .dispatch(kernel::module, 0, bytes, bytes, 1, None)
            .unwrap();
        io.queue
            .write_buffer(&io.state, 0, bytemuck::bytes_of(&sample()));
        let mut metrics = StepMetrics::default();
        let submitted = io.submit_prefix(&[dispatch], bytes, &mut metrics);
        let mut output = vec![];
        io.finish_prefix(submitted, &mut metrics, |data| {
            output.extend_from_slice(data);
            Ok(())
        })
        .unwrap();
        assert_eq!(metrics.readback_bytes, expected);
        assert_eq!(metrics.gpu_dispatch.is_some(), with_stamps);
        assert_eq!(metrics.gpu_copy, None);
        assert_eq!(output.len(), bytes as usize);
        assert_ne!(output.as_slice(), bytemuck::bytes_of(&sample()));
        if let Some(previous) = baseline.as_ref() {
            assert_eq!(&output, previous);
        } else {
            baseline = Some(output);
        }
    }
}
