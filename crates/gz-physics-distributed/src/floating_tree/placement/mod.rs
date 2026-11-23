// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Atomic placement of a motorized tree and its physically separated free payload.
use super::{native::TreeWorld, profile::sealed::Recipe, *};
use crate::articulation::ledger::SPLIT_GRASP_FILE;
use gz_physics_multifidelity::MovingReaction;
mod worker;
pub use worker::serve;
#[cfg(test)]
mod crash_tests;
pub mod fleet;
mod pool;
pub use pool::SplitGraspPool;
const FILE: &str = SPLIT_GRASP_FILE;
const ARCHIVE: [u8; 8] = *b"GZSPCU01";
const WIRE: [u8; 8] = *b"GZSPWK01";
const FLAG: &str = "--managed-split-grasp-v1";

/// Keep a warm native world resident, or split it after consecutive eligible intervals.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GraspPlacement {
    KeepTogether,
    Adaptive { release_dwell: u16 },
}
impl GraspPlacement {
    fn threshold(self) -> Option<u16> {
        match self {
            Self::KeepTogether => None,
            Self::Adaptive { release_dwell } => Some(release_dwell),
        }
    }
}
impl GraspTreeSpec {
    pub(super) fn bare_motor_spec(&self) -> MotorTreeSpec {
        MotorTreeSpec {
            tree: self.payload.tree.clone(),
            limits: self.payload.limits.clone(),
        }
    }
    fn partitionable(&self) -> bool {
        let part = &self.payload.payload;
        !self.payload.exact_revolute_locks
            && !self.payload.payload_can_sleep
            && self
                .payload
                .tree
                .links
                .last()
                .is_some_and(|last| part.id > last.id)
            && part.inertia_center == [0.; 3]
            && part.inertia_orientation == [0., 0., 0., 1.]
            && part.inertia_tensor[0] == part.inertia_tensor[1]
            && part.inertia_tensor[0] == part.inertia_tensor[2]
            && part.inertia_tensor[3..] == [0.; 3]
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct Box3 {
    lo: [f64; 3],
    hi: [f64; 3],
}
impl From<([f64; 3], [f64; 3])> for Box3 {
    fn from((lo, hi): ([f64; 3], [f64; 3])) -> Self {
        Self { lo, hi }
    }
}
impl Box3 {
    fn union(self, b: Self) -> Self {
        Self {
            lo: std::array::from_fn(|i| self.lo[i].min(b.lo[i])),
            hi: std::array::from_fn(|i| self.hi[i].max(b.hi[i])),
        }
    }
    fn valid(self) -> bool {
        (0..3).all(|i| self.lo[i].is_finite() && self.hi[i].is_finite() && self.lo[i] <= self.hi[i])
    }
    fn disjoint(self, b: Self) -> bool {
        self.valid() && b.valid() && (0..3).any(|i| self.hi[i] < b.lo[i] || b.hi[i] < self.lo[i])
    }
}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
enum Piece {
    Coupled(Box<Snapshot>),
    Tree(Box<Snapshot>),
    Part(ArticulationLinkState),
}
impl Piece {
    fn digest(&self) -> [u8; 32] {
        let mut h = Sha256::new();
        bincode::serialize_into(&mut h, self).expect("piece writes to digest");
        h.finalize().into()
    }
}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
enum Mode {
    Coupled(Box<Snapshot>),
    Split {
        tree: Box<Snapshot>,
        part: ArticulationLinkState,
    },
}
impl Mode {
    fn tree(&self) -> &Snapshot {
        match self {
            Self::Coupled(s) => s,
            Self::Split { tree, .. } => tree,
        }
    }
    fn part(&self) -> &ArticulationLinkState {
        match self {
            Self::Coupled(s) => &s.image.payload().expect("checked coupled receipt").body,
            Self::Split { part, .. } => part,
        }
    }
    fn joined(&self) -> bool {
        matches!(self, Self::Coupled(_))
    }
    fn pieces(&self, owners: [u8; 2]) -> Vec<(u8, Piece)> {
        match self {
            Self::Coupled(s) => vec![(owners[0], Piece::Coupled(s.clone()))],
            Self::Split { tree, part } => vec![
                (owners[0], Piece::Tree(tree.clone())),
                (owners[1], Piece::Part(*part)),
            ],
        }
    }
}
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlacementStats {
    pub splits: u64,
    pub joins: u64,
    pub split_steps: u64,
    pub rejected_previews: u64,
}
#[derive(Clone, Serialize, Deserialize)]
struct Cut {
    width: u8,
    owners: [u8; 2],
    epoch: u64,
    generation: u64,
    policy: GraspPlacement,
    streak: u16,
    stats: PlacementStats,
    mode: Mode,
}
impl Cut {
    fn tick(&self) -> u64 {
        self.mode.tree().state.tick
    }
    fn validate(&self, s: &GraspTreeSpec) -> Result<()> {
        let counts = self.stats;
        if counts.splits.checked_sub(counts.joins) != Some(u64::from(!self.mode.joined()))
            || counts.joins != counts.rejected_previews
            || counts
                .split_steps
                .checked_add(counts.rejected_previews)
                .is_none_or(|n| n > self.tick())
            || counts
                .splits
                .checked_add(counts.joins)
                .is_none_or(|n| n > self.generation)
            || !(2..=8).contains(&self.width)
            || self.owners.iter().any(|i| *i >= self.width)
            || self.mode.joined() != (self.owners[0] == self.owners[1])
            || self.epoch > self.generation
            || self.policy.threshold() == Some(0)
            || self.streak > self.policy.threshold().unwrap_or(0)
            || (!self.mode.joined()
                && (self.policy.threshold().is_none()
                    || !s.partitionable()
                    || self.tick() == 0
                    || self.streak != 0))
        {
            return Err(invalid(
                "invalid selective grasp ownership, age or placement",
            ));
        }
        match &self.mode {
            Mode::Coupled(snap) => snap.check(s, s.digest(), self.tick()),
            Mode::Split { tree, part } => {
                let bare = s.bare_motor_spec();
                if !valid_link(part, s.payload.payload.id) {
                    return Err(invalid("invalid detached payload state"));
                }
                tree.check(&bare, bare.digest(), self.tick())
            }
        }
    }
}
fn boundary(_edge: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_edge);
}
