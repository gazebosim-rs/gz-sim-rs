// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A floating box-link robot with joint actuation and an external horizontal floor.
use crate::{
    articulation::ArticulationGroundSpec,
    floating_tree::{FloatingTreeJointLimit, FloatingTreeJointSpec},
};
use serde::{Deserialize, Serialize};

/// A centered, aligned box collider on a separately massed robot link.
/// Poses are in world axes. Inertia center and its orientation are in link axes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundedRobotLinkSpec {
    pub id: u64,
    pub size: [f64; 3],
    pub mass: f64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub inertia_center: [f64; 3],
    pub inertia_orientation: [f64; 4],
    /// xx, yy, zz, xy, xz and yz in the inertial frame.
    pub inertia_tensor: [f64; 6],
}
/// A 3–32-link tree under gravity with a single external fixed floor.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundedRobotSpec {
    pub model_id: u64,
    /// In ascending global public ID and authored ordinary body insertion order.
    pub links: Vec<GroundedRobotLinkSpec>,
    /// In the ordinary native joint construction order.
    pub joints: Vec<FloatingTreeJointSpec>,
    /// Sorted by the affected public joint ID.
    pub limits: Vec<FloatingTreeJointLimit>,
    pub ground: ArticulationGroundSpec,
}

use crate::articulation::{ArticulationError, GroundReaction, NativeError};
use crate::floating_tree::{FloatingTreeObservation, directed_root};
use bincode::Options;
use gz_physics_multifidelity::{GROUNDED_ROBOT_IMAGE_LIMIT, GroundPatchPoint};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::path::Path;
#[cfg(test)]
mod crash_tests;
mod native;
mod pool;
mod posture;
pub use pool::{GroundedRobotPool, ReturnedGroundedRobot, RobotGraphEncoding, RobotGraphTraffic, RobotLocalInputs};
pub use posture::{
    MAX_POSTURE_INTERVALS, RobotPostureFeedback, RobotPostureInterval, RobotPostureProgram,
    RobotPostureReceipt, RobotPostureRecord, RobotPostureTorque,
};

type Result<T> = std::result::Result<T, ArticulationError>;
const FILE: &str = crate::articulation::ledger::GROUNDED_ROBOT_FILE;
const WIRE: [u8; 8] = *b"GZRBWK01";
const ARCHIVE: [u8; 8] = *b"GZRBCU01";
const FLAG: &str = "--managed-grounded-robot-v1";
const SNAP_LIMIT: usize = GROUNDED_ROBOT_IMAGE_LIMIT + 96 * 1024;
const FILE_LIMIT: usize = SNAP_LIMIT + 2 * posture::PROGRAM_LIMIT + 32 * 1024;
const CONTROL_ARCHIVE: [u8; 8] = *b"GZRBCU02";
fn invalid(message: &'static str) -> ArticulationError {
    ArticulationError::Invalid(message)
}
fn codec(limit: usize) -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(limit as u64)
        .reject_trailing_bytes()
}
impl GroundedRobotSpec {
    fn root(&self) -> std::result::Result<usize, NativeError> {
        let links = self.links.iter().map(|l| l.id).collect::<Vec<_>>();
        let root = directed_root(self.model_id, &links, &self.joints, 32)?;
        let g = self.ground;
        if g.link_id == self.model_id
            || (links[0]..=*links.last().unwrap()).contains(&g.link_id)
            || self.joints.iter().any(|j| j.joint_id == g.link_id)
            || !g.height.is_finite()
            || !g.gravity_z.is_finite()
            || g.gravity_z >= 0.
            || !g.size.iter().all(|v| v.is_finite() && *v > 0.)
            || !self
                .limits
                .windows(2)
                .all(|p| p[0].joint_id < p[1].joint_id)
            || self.limits.iter().any(|l| {
                !l.lower.is_finite()
                    || !l.upper.is_finite()
                    || l.lower > 0.
                    || l.upper < 0.
                    || !self.joints.iter().any(|j| j.joint_id == l.joint_id)
            })
            || self.links.iter().any(|l| {
                !l.size.iter().all(|x| x.is_finite() && *x > 0.)
                    || !l.mass.is_finite()
                    || l.mass <= 0.
            })
        {
            return Err(NativeError::Invalid("invalid grounded robot construction"));
        }
        Ok(root)
    }
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::RobotWorld::new(self, 0, None).map(|_| ())
    }
}
/// One solved finite-link/ground manifold, indexed by the public SDF link.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct RobotGroundPatch {
    pub link_id: u64,
    pub normal_on_body: [f64; 3],
    pub points: Vec<GroundPatchPoint>,
}
#[derive(Clone, Serialize, Deserialize)]
struct Snapshot {
    state: FloatingTreeObservation,
    /// In public link order. Includes previous impulses for native warm starting.
    reactions: Vec<GroundReaction>,
    patches: Vec<RobotGroundPatch>,
    image: Option<Vec<u8>>,
}
impl Snapshot {
    fn check(&self, s: &GroundedRobotSpec) -> Result<()> {
        let state = &self.state;
        if !state
            .links
            .iter()
            .map(|x| x.id)
            .eq(s.links.iter().map(|x| x.id))
            || !state.joints.iter().map(|x| x.joint_id).eq(s
                .joints
                .iter()
                .map(|j| j.joint_id)
                .collect::<BTreeSet<_>>())
            || self.reactions.len() != s.links.len()
            || !self.reactions.iter().all(|r| {
                r.normal_impulse.is_finite()
                    && r.normal_impulse >= 0.
                    && r.warmstart_normal_impulse.is_finite()
                    && r.warmstart_normal_impulse >= 0.
                    && r.tangent_impulse.iter().all(|v| v.is_finite())
            })
            || !state.links.iter().all(|l| {
                l.position
                    .iter()
                    .chain(&l.velocity)
                    .chain(&l.orientation)
                    .chain(&l.angular_velocity)
                    .all(|x| x.is_finite())
            })
            || !state.joints.iter().all(|j| {
                j.q.is_finite() && j.qdot.is_finite() && j.pending_effort.is_none_or(f64::is_finite)
            })
            || !self.patches.windows(2).all(|w| w[0].link_id < w[1].link_id)
            || !self.patches.iter().all(|p| {
                s.links.binary_search_by_key(&p.link_id, |l| l.id).is_ok()
                    && p.normal_on_body.iter().all(|x| x.is_finite())
                    && (1..=4).contains(&p.points.len())
                    && p.points.iter().all(|q| {
                        q.normal_impulse.is_finite()
                            && q.normal_impulse >= 0.
                            && q.plane_witness
                                .iter()
                                .chain(&q.solver_point)
                                .chain(&q.body_lever)
                                .all(|x| x.is_finite())
                    })
            })
            || !match &self.image {
                None => state.tick == 0,
                Some(b) => {
                    state.tick > 0
                        && b.len() <= GROUNDED_ROBOT_IMAGE_LIMIT
                        && b.starts_with(b"GZHRBT01")
                }
            }
        {
            return Err(invalid(
                "invalid grounded robot observation, native image or support patch",
            ));
        }
        Ok(())
    }
}
pub fn serve(slot: u8, lease: &Path) -> std::io::Result<()> {
    serve_version(slot, lease, false, false)
}
pub fn serve_packed(slot: u8, lease: &Path) -> std::io::Result<()> {
    serve_version(slot, lease, true, false)
}
pub fn serve_controlled(slot: u8, lease: &Path) -> std::io::Result<()> {
    serve_version(slot, lease, true, true)
}
const CONTROL_WIRE: [u8; 8] = *b"GZRBWK03";
const CONTROL_FLAG: &str = "--managed-grounded-robot-v3";
const PACKED_WIRE: [u8; 8] = *b"GZRBWK02";
const PACKED_FLAG: &str = "--managed-grounded-robot-v2";
fn serve_version(slot: u8, lease: &Path, packed: bool, controlled: bool) -> std::io::Result<()> {
    if slot >= 8 || !lease.is_absolute() {
        return Err(std::io::Error::other("invalid robot slot or lease path"));
    }
    crate::moving::worker::serve_profile(
        slot,
        lease,
        if controlled {
            CONTROL_WIRE
        } else if packed {
            PACKED_WIRE
        } else {
            WIRE
        },
        move |s: GroundedRobotSpec| {
            s.validate()?;
            let mut state = crate::paged_native::State::<native::RobotWorld>::with_capabilities(
                packed, controlled,
            );
            Ok(move |tick, req, steps: &mut u64| state.handle(&s, (), tick, req, steps))
        },
    )
}
