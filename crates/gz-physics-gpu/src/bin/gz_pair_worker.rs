// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("expected Vulkan adapter name")?;
    if args.next().is_some() {
        return Err("expected only the Vulkan adapter name".into());
    }
    gz_physics_gpu::pair_process::serve(
        &adapter,
        std::io::stdin().lock(),
        std::io::BufWriter::new(std::io::stdout().lock()),
    )?;
    Ok(())
}
