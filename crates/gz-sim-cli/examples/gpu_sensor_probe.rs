// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Start a separate Gazebo CLI process and verify its native Zenoh sensor and control endpoints.
#[path = "support/gpu_sensor_probe.rs"]
mod probe;
fn main() -> probe::Result<()> {
    let mut a = std::env::args_os().skip(1);
    let server = a
        .next()
        .ok_or("usage: gpu_sensor_probe GZ_SIM_RS ADAPTER OUTPUT_DIRECTORY")?;
    let adapter = a
        .next()
        .ok_or("supply a physical Vulkan adapter")?
        .into_string()
        .map_err(|_| "adapter must be UTF-8")?;
    let out = a.next().ok_or("supply an empty output directory")?;
    let place = match a.next().as_deref() {
        None => probe::Mode::Gpu,
        Some(v) if v == "--placement" => probe::Mode::ScalarPlacement,
        Some(v) if v == "--native" => probe::Mode::NativeHandoff,
        Some(v) if v == "--native-support" => probe::Mode::NativeSupport,
        _ => return Err("expected optional --placement, --native or --native-support".into()),
    };
    if a.next().is_some() {
        return Err("unexpected argument".into());
    }
    probe::run(
        std::path::Path::new(&server),
        &adapter,
        std::path::Path::new(&out),
        place,
    )
}
