// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Alternate contact formation and release on two separated floor-supported cargo slabs.
#[allow(dead_code)]
#[path = "heterogeneous_worlds.rs"]
mod loaded;
use clap::ValueEnum;
use gz_physics_distributed::physics::{Body, BodyWrench};
use serde::{Deserialize, Serialize};

pub use loaded::GRAVITY;
pub const SETTLED: u64 = 70;
pub const END: u64 = 350;
pub const LAUNCHES: [u64; 6] = [80, 120, 160, 200, 240, 280];

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Layout {
    Adjacent,
    Separated,
}

/// Cold placement for the fixed four-process fixture. Each physical row contains
/// one loaded pad and three light stacks, independent of canonical ID order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Placement {
    Single,
    Split,
}

// This shared fixture is also included by trace tests that only need its geometry.
#[allow(dead_code)]
impl Placement {
    pub fn owners(self, groups: &[Vec<Body>]) -> Vec<usize> {
        groups
            .iter()
            .map(|g| usize::from(self == Self::Split && g[0].position[1] > 0.))
            .collect()
    }

    pub fn worker_seeds(self, groups: &[Vec<Body>]) -> Vec<Vec<Body>> {
        let mut workers = vec![vec![]; 4];
        for (group, owner) in groups.iter().zip(self.owners(groups)) {
            workers[owner].extend_from_slice(group);
        }
        workers
    }
}

pub fn seeds(layout: Layout) -> Vec<Vec<Body>> {
    let mut groups = loaded::seeds(loaded::Layout::Twins);
    if matches!(layout, Layout::Separated) {
        groups.swap(1, 7);
        for (i, body) in groups.iter_mut().flatten().enumerate() {
            body.id = i as u64 + 1;
        }
    }
    groups
}

pub fn wrenches(tick: u64, groups: &[Vec<Body>]) -> Vec<BodyWrench> {
    wrenches_at(tick, groups, &LAUNCHES)
}

pub fn wrenches_at(tick: u64, groups: &[Vec<Body>], launch_ticks: &[u64; 6]) -> Vec<BodyWrench> {
    let Some(index) = launch_ticks.iter().position(|&event| event == tick) else {
        return vec![];
    };
    let pad = groups
        .iter()
        .filter(|g| g.len() > 2)
        .nth(index % 2)
        .unwrap();
    // 300 N for one 10 ms public tick on each 1 kg sphere. Gravity removes
    // 0.0981 m/s during the interval. The subsequent flight is unforced.
    pad[1..]
        .iter()
        .map(|b| BodyWrench {
            id: b.id,
            force: [0., 0., 300.],
            torque: [0.; 3],
        })
        .collect()
}
