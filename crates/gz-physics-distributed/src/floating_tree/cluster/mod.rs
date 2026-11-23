// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two grippers, both independent parts and every cross-assembly contact in one native solver.
use super::{placement::fleet::GraspFleetSpec, profile::sealed::Recipe, *};
use gz_physics_multifidelity::MovingReaction;
#[cfg(test)]
mod crash_tests;
mod native;
pub(super) use native::ClusterWorld;
mod pool;
mod worker;
pub(super) use pool::Cut;
pub use pool::{CoupledGraspFleetPool, GraspGraphTraffic};
pub use worker::{serve, serve_v2};
const FILE: &str = crate::articulation::ledger::COUPLED_GRASP_FILE;
#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) enum Version {
    Legacy,
    ObservedShear,
}
impl Version {
    fn archive(self) -> [u8; 8] {
        match self {
            Self::Legacy => *b"GZCCCU01",
            Self::ObservedShear => *b"GZCCCU02",
        }
    }
    fn wire(self) -> [u8; 8] {
        match self {
            Self::Legacy => *b"GZCCWK01",
            Self::ObservedShear => *b"GZCCWK02",
        }
    }
    fn native(self) -> [u8; 8] {
        match self {
            Self::Legacy => *b"GZFCPL01",
            Self::ObservedShear => *b"GZFCPL02",
        }
    }
    fn flag(self) -> &'static str {
        match self {
            Self::Legacy => "--managed-coupled-grasp-fleet-v1",
            Self::ObservedShear => "--managed-coupled-grasp-fleet-v2",
        }
    }
}
const SNAP_LIMIT: usize = gz_physics_multifidelity::GRASP_FLEET_IMAGE_LIMIT + 64 * 1024;
const FILE_LIMIT: usize = SNAP_LIMIT + 64 * 1024;
use bincode::Options;
fn codec(limit: usize) -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(limit as u64)
        .reject_trailing_bytes()
}

/// World-space contact impulse on the first body from the second. The first body
/// belongs to the gripper with the smaller public model ID.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct GraspFleetPairReaction {
    pub first_model: u64,
    pub first_body: u64,
    pub second_model: u64,
    pub second_body: u64,
    pub reaction: MovingReaction,
}
fn contact_bits(r: &MovingReaction) -> (bool, [u64; 3], [u64; 3], u64, u64) {
    (
        r.active,
        r.normal.map(f64::to_bits),
        r.tangent_impulse.map(f64::to_bits),
        r.normal_impulse.to_bits(),
        r.warmstart_normal_impulse.to_bits(),
    )
}
impl PartialEq for GraspFleetPairReaction {
    fn eq(&self, b: &Self) -> bool {
        self.first_model == b.first_model
            && self.second_model == b.second_model
            && self.first_body == b.first_body
            && self.second_body == b.second_body
            && contact_bits(&self.reaction) == contact_bits(&b.reaction)
    }
}
impl Eq for GraspFleetPairReaction {}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(super) struct Member {
    pub(super) tree: FloatingTreeObservation,
    pub(super) part: PayloadReceipt,
    pub(super) motors: Vec<Option<MotorSetting>>,
}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(super) struct ClusterSnapshot {
    pub(super) tick: u64,
    pub(super) members: Vec<Member>,
    pub(super) pairs: Vec<GraspFleetPairReaction>,
    pub(super) image: Option<Vec<u8>>,
}
fn parts(s: &GraspTreeSpec) -> Vec<u64> {
    let mut ids = s
        .payload
        .tree
        .links
        .iter()
        .map(|l| l.id)
        .chain([s.payload.payload.id])
        .collect::<Vec<_>>();
    ids.sort_unstable();
    ids
}
fn public_pairs(s: &GraspFleetSpec) -> Vec<[u64; 4]> {
    let [a, b] = s.assemblies.as_slice() else {
        return Vec::new();
    };
    let first = parts(a);
    let second = parts(b);
    first
        .into_iter()
        .flat_map(|id| {
            second
                .iter()
                .map(move |&other| [a.payload.tree.model_id, id, b.payload.tree.model_id, other])
        })
        .collect()
}
fn valid_contact(r: &MovingReaction) -> bool {
    r.normal_impulse.is_finite()
        && r.normal_impulse >= 0.
        && r.warmstart_normal_impulse.is_finite()
        && r.warmstart_normal_impulse >= 0.
        && r.normal
            .iter()
            .chain(&r.tangent_impulse)
            .all(|x| x.is_finite())
}
pub(super) fn admissible(s: &GraspFleetSpec) -> Result<()> {
    if s.assemblies.len() != 2 {
        return Err(invalid(
            "coupled grasp fleet requires exactly two assemblies",
        ));
    }
    let mut ids = BTreeSet::new();
    for a in &s.assemblies {
        a.validate_configuration()?;
        let t = &a.payload.tree;
        for id in [t.model_id, a.payload.payload.id]
            .into_iter()
            .chain(t.links.iter().map(|l| l.id))
            .chain(t.joints.iter().map(|j| j.joint_id))
        {
            if !ids.insert(id) {
                return Err(invalid(
                    "duplicate public body, joint or model in coupled grasp fleet",
                ));
            }
        }
    }
    if s.assemblies[0].payload.tree.model_id >= s.assemblies[1].payload.tree.model_id {
        return Err(invalid(
            "coupled grasp fleet model order differs from public IDs",
        ));
    }
    Ok(())
}
impl ClusterSnapshot {
    fn check(&self, s: &GraspFleetSpec, version: Version) -> Result<()> {
        if self.members.len() != 2
            || ((self.tick == 0) != self.image.is_none())
            || self.image.as_ref().is_some_and(|im| {
                !im.starts_with(&version.native())
                    || im.len() > gz_physics_multifidelity::GRASP_FLEET_IMAGE_LIMIT
            })
        {
            return Err(invalid("invalid coupled grasp graph or clock"));
        }
        for (m, a) in self.members.iter().zip(&s.assemblies) {
            let t = &a.payload.tree;
            if self.tick != m.tree.tick
                || t.links.len() != m.tree.links.len()
                || t.joints.len() != m.tree.joints.len()
                || t.joints.len() != m.motors.len()
                || t.links.len() != m.part.reactions.len()
                || !t
                    .links
                    .iter()
                    .zip(&m.tree.links)
                    .all(|(l, p)| valid_link(p, l.id))
                || !valid_link(&m.part.body, a.payload.payload.id)
                || !m.part.reactions.iter().all(valid_contact)
                || !m
                    .tree
                    .joints
                    .windows(2)
                    .all(|v| v[0].joint_id < v[1].joint_id)
                || m.tree.joints.iter().any(|j| {
                    !j.q.is_finite()
                        || !j.qdot.is_finite()
                        || !j.pending_effort.is_none_or(f64::is_finite)
                        || !t.joints.iter().any(|v| v.joint_id == j.joint_id)
                })
                || !m.motors.iter().flatten().all(|m| m.valid())
            {
                return Err(invalid(
                    "invalid public state, actuator or input in coupled grasp fleet",
                ));
            }
        }
        let expected = public_pairs(s);
        if self.pairs.len() != expected.len()
            || !self.pairs.iter().zip(expected).all(|(r, k)| {
                [r.first_model, r.first_body, r.second_model, r.second_body] == k
                    && valid_contact(&r.reaction)
            })
        {
            return Err(invalid("incorrect cross-assembly contact receipt"));
        }
        Ok(())
    }
}
