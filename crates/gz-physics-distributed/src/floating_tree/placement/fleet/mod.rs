// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One atomic world clock for independently simulated, spatially separated grippers.
use super::{
    worker::{Bounds, Op, Reply, Request, RoleWorker, Seed},
    *,
};
use crate::articulation::ledger::GRASP_FLEET_FILE;
mod worker;
pub use worker::{serve, serve_prior, serve_warm};
#[cfg(test)]
mod crash_tests;
mod pool;
pub use pool::GraspFleetPool;
pub const MAX_GRASP_ASSEMBLIES: usize = 8;
// Each member owns the same boxed native receipt received from the large Piece::Coupled wire variant.
// Moving the handle between a trial, the accepted barrier and an install request preserves this allocation.
type MemberSnapshot = Box<Snapshot>;

const FILE: &str = GRASP_FLEET_FILE;
const ARCHIVE: [u8; 8] = *b"GZFFCU01";
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum MemberLane {
    Standard,
    Warm,
    RetainedPrior,
}
impl MemberLane {
    fn wire(self) -> [u8; 8] {
        match self {
            Self::Standard => *b"GZFFWK01",
            Self::Warm => *b"GZFFWK02",
            Self::RetainedPrior => *b"GZFFWK03",
        }
    }
    fn flag(self) -> &'static str {
        match self {
            Self::Standard => "--managed-grasp-fleet-v1",
            Self::Warm => "--managed-grasp-fleet-v2",
            Self::RetainedPrior => "--managed-grasp-fleet-v3",
        }
    }
}

const FLEET_LIMIT: usize = MAX_GRASP_ASSEMBLIES * crate::moving::worker::LIMIT;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraspFleetSpec {
    pub assemblies: Vec<GraspTreeSpec>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GraspFleetPlacement {
    Packed,
    RoundRobin,
}
impl GraspFleetSpec {
    fn check_ids(&self) -> Result<()> {
        if !(2..=MAX_GRASP_ASSEMBLIES).contains(&self.assemblies.len())
            || !self
                .assemblies
                .windows(2)
                .all(|p| p[0].payload.tree.model_id < p[1].payload.tree.model_id)
        {
            return Err(invalid(
                "fleet expects 2–8 grippers in ascending public model order",
            ));
        }
        let mut ids = BTreeSet::new();
        for group in &self.assemblies {
            let tree = &group.payload.tree;
            for id in [tree.model_id, group.payload.payload.id]
                .into_iter()
                .chain(tree.links.iter().map(|l| l.id))
                .chain(tree.joints.iter().map(|j| j.joint_id))
            {
                if !ids.insert(id) {
                    return Err(invalid("duplicate public ID in grasp fleet"));
                }
            }
            group.validate_configuration()?;
        }
        Ok(())
    }
    fn cold(&self) -> Result<(Vec<MemberSnapshot>, Vec<Bounds>)> {
        self.check_ids()?;
        let mut snapshots = Vec::new();
        let mut bounds = Vec::new();
        for s in &self.assemblies {
            let w = TreeWorld::new(s)?;
            let (tree, part) = w.collision_bounds()?;
            snapshots.push(Box::new(w.capture()?));
            bounds.push(Bounds { tree, part });
        }
        Ok((snapshots, bounds))
    }
    pub fn validate(&self) -> Result<()> {
        let (_, bounds) = self.cold()?;
        self.check_separation(&bounds, 0, 0)
    }
    fn group(&self, model: u64) -> Option<usize> {
        self.assemblies
            .binary_search_by_key(&model, |g| g.payload.tree.model_id)
            .ok()
    }
    fn joint(&self, id: u64) -> Option<usize> {
        self.assemblies
            .iter()
            .position(|g| g.payload.tree.joints.iter().any(|j| j.joint_id == id))
    }
    fn part(&self, id: u64) -> Option<usize> {
        self.assemblies
            .iter()
            .position(|g| g.payload.payload.id == id)
    }
    fn check_separation(&self, bounds: &[Bounds], epoch: u64, tick: u64) -> Result<()> {
        if bounds.len() != self.assemblies.len() {
            return Err(invalid("incomplete grasp fleet collision bounds"));
        }
        for (i, (s, b)) in self.assemblies.iter().zip(bounds).enumerate() {
            if b.tree.len() != s.payload.tree.links.len()
                || b.tree.iter().any(|v| !v.valid())
                || !b.part.is_some_and(Box3::valid)
            {
                return Err(invalid(
                    "native solver cannot certify a grasp assembly collision bound",
                ));
            }
            let first = s
                .payload
                .tree
                .links
                .iter()
                .map(|l| l.id)
                .chain([s.payload.payload.id])
                .zip(b.tree.iter().copied().chain(b.part));
            for (j, other) in self.assemblies[..i].iter().enumerate() {
                let bo = &bounds[j];
                let second = other
                    .payload
                    .tree
                    .links
                    .iter()
                    .map(|l| l.id)
                    .chain([other.payload.payload.id])
                    .zip(bo.tree.iter().copied().chain(bo.part));
                for (id1, v1) in first.clone() {
                    for (id2, v2) in second.clone() {
                        if !v1.disjoint(v2) {
                            return Err(ArticulationError::PossibleCrossAssemblyContact {
                                epoch,
                                tick,
                                first_model: other.payload.tree.model_id,
                                first_body: id2,
                                second_model: s.payload.tree.model_id,
                                second_body: id1,
                            });
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
impl GraspFleetPlacement {
    fn owners(self, n: usize, width: u8) -> Vec<u8> {
        (0..n)
            .map(|i| match self {
                Self::Packed => 0,
                Self::RoundRobin => i as u8 % width,
            })
            .collect()
    }
}
#[derive(Clone, Serialize, Deserialize)]
pub(in crate::floating_tree) struct FleetCut {
    pub(in crate::floating_tree) epoch: u64,
    pub(in crate::floating_tree) tick: u64,
    pub(in crate::floating_tree) generation: u64,
    pub(in crate::floating_tree) width: u8,
    pub(in crate::floating_tree) placement: GraspFleetPlacement,
    pub(in crate::floating_tree) owners: Vec<u8>,
    pub(in crate::floating_tree) leases: Vec<u64>,
    pub(in crate::floating_tree) groups: Vec<MemberSnapshot>,
}
impl FleetCut {
    fn validate(&self, s: &GraspFleetSpec) -> Result<()> {
        let n = s.assemblies.len();
        if !(2..=8).contains(&self.width)
            || self.epoch > self.generation
            || self.groups.len() != n
            || self.owners.len() != n
            || self.leases.len() != n
            || self.owners.iter().any(|x| *x >= self.width)
            || self.leases.iter().any(|&g| g > self.generation)
        {
            return Err(invalid("invalid grasp fleet ownership cut"));
        }
        for (snapshot, spec) in self.groups.iter().zip(&s.assemblies) {
            snapshot.check(spec, spec.digest(), self.tick)?;
        }
        Ok(())
    }
}
