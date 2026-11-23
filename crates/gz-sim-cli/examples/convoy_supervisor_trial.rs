// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a supervisor process crash on Linux, including an interrupted worker RPC.
#[cfg(target_os = "linux")]
#[path = "convoy_supervisor_trial/linux.rs"]
mod linux;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(target_os = "linux")]
    {
        linux::execute()
    }
    #[cfg(not(target_os = "linux"))]
    {
        Err("This process qualification uses Linux pidfds and procfs.".into())
    }
}
