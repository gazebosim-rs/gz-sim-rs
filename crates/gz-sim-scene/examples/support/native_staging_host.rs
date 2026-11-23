// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional Linux scheduler measurements at treatment boundaries.
use std::{collections::BTreeMap, fs};
pub type Snapshot = BTreeMap<u32, (u64, u64)>;
/// Linux schedstat reports nanoseconds executing and runnable but waiting per thread.
pub fn snapshot() -> Option<Snapshot> {
    let mut threads = BTreeMap::new();
    for entry in fs::read_dir("/proc/self/task").ok()? {
        let entry = entry.ok()?;
        let id = entry.file_name().to_str()?.parse().ok()?;
        let text = fs::read_to_string(entry.path().join("schedstat")).ok()?;
        let mut counters = text.split_whitespace();
        threads.insert(
            id,
            (
                counters.next()?.parse().ok()?,
                counters.next()?.parse().ok()?,
            ),
        );
    }
    Some(threads)
}
/// Report only treatments with identical live threads and monotone counters.
pub fn elapsed(a: Option<&Snapshot>, b: Option<&Snapshot>) -> Option<(u64, u64, usize)> {
    let (a, b) = (a?, b?);
    if a.len() != b.len() {
        return None;
    }
    let (mut exec, mut wait) = (0u64, 0u64);
    for (id, (run0, wait0)) in a {
        let (run1, wait1) = b.get(id)?;
        exec = exec.checked_add(run1.checked_sub(*run0)?)?;
        wait = wait.checked_add(wait1.checked_sub(*wait0)?)?;
    }
    Some((exec, wait, a.len()))
}
