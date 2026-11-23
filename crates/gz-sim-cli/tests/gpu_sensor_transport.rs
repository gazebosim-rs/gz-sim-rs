// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
#[path = "../examples/support/gpu_sensor_probe.rs"]
mod probe;
#[test]
#[ignore = "requires named f64 Vulkan hardware and local TCP"]
fn external_sensor_telemetry_stamped_link_inputs_and_epoch_replay() {
    let scratch = tempfile::tempdir().unwrap();
    probe::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_gz-sim-rs")),
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        scratch.path(),
        probe::Mode::Gpu,
    )
    .unwrap();
}

#[test]
#[ignore = "requires named f64 Vulkan hardware and local TCP"]
fn remote_placement_preserves_queued_impulses_actual_sensor_packets_and_reset() {
    let scratch = tempfile::tempdir().unwrap();
    probe::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_gz-sim-rs")),
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        scratch.path(),
        probe::Mode::ScalarPlacement,
    )
    .unwrap();
}

#[test]
#[ignore = "requires named f64 Vulkan hardware and local TCP"]
fn native_pair_acknowledgements_preserve_external_link_impulse_and_sensor_clocks() {
    let scratch = tempfile::tempdir().unwrap();
    probe::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_gz-sim-rs")),
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        scratch.path(),
        probe::Mode::NativeHandoff,
    )
    .unwrap();
}

#[test]
#[ignore = "requires named f64 Vulkan hardware and local TCP"]
fn native_supported_load_and_external_liftoff_are_visible_in_real_imu_and_rpc() {
    let scratch = tempfile::tempdir().unwrap();
    probe::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_gz-sim-rs")),
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        scratch.path(),
        probe::Mode::NativeSupport,
    )
    .unwrap();
}
