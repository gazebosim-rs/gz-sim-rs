// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Canonical all-worker physics trajectory, independent of native ownership.
use super::scene::Layout;
use gz_physics_distributed::pool::PoolFrame;
use std::{path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
pub fn physical_bytes(f: &PoolFrame) -> Result<Vec<u8>> {
    let mut bodies: Vec<_> = f.workers.iter().flat_map(|w| &w.snapshot.bodies).collect();
    let mut pairs: Vec<_> = f
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    let mut floors: Vec<_> = f
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .collect();
    bodies.sort_unstable_by_key(|b| b.id);
    pairs.sort_unstable_by_key(|p| (p.a, p.b));
    floors.sort_unstable_by_key(|s| s.body);
    Ok(bincode::serialize(&(f.tick, bodies, pairs, floors))?)
}
pub fn local_digest(profile: &Path, layout: Layout) -> Result<String> {
    let name = match layout {
        Layout::Adjacent => "adjacent",
        Layout::Separated => "separated",
    };
    let output = Command::new(profile)
        .args(["--layout", name, "--local"])
        .output()?;
    if !output.status.success() {
        return Err(format!(
            "local reference failed: {}",
            String::from_utf8_lossy(&output.stderr)
        )
        .into());
    }
    let summary: serde_json::Value = serde_json::from_slice(&output.stdout)?;
    summary["physics_sha256"]
        .as_str()
        .map(ToOwned::to_owned)
        .ok_or_else(|| "missing local reference hash".into())
}
