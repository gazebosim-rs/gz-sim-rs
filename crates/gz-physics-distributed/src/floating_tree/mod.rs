// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One free-floating, reduced-coordinate revolute tree on an exclusively owned worker.
use crate::articulation::{
    ArticulationError, ArticulationLinkSpec, ArticulationLinkState, NativeError,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub mod automatic;
pub use automatic::{
    AutomaticConstruction, AutomaticImpactReceipt, AutomaticMaterialPool, AutomaticMaterialSpec,
    AutomaticPool, AutomaticRotorMaterialSpec, RotorMaterialPool, RotorPadSpec,
};
mod contact;
pub use contact::{
    ArticulatedPointSite, ContactImpulseReceipt, ContactPointBasis, ContactPointCertificate,
};
mod adaptive;
pub use adaptive::{
    AdaptiveGraspFleetPool, GraspFleetMerge, GraspMemberTiming, GraspReplayBudget,
    GraspReplayTiming,
};
pub mod cluster;
#[cfg(test)]
pub(crate) mod crash_tests;
mod fleet_session;
pub mod material;
pub use cluster::{CoupledGraspFleetPool, GraspFleetPairReaction, GraspGraphTraffic};
pub use material::{
    ArticulatedMaterialPool, ArticulatedMaterialSpec, MaterialContactLoad, MaterialContactReceipt,
    MaterialImpactBarrier,
};
mod native;
pub mod placement;
pub use placement::fleet::{
    GraspFleetPlacement, GraspFleetPool, GraspFleetSpec, MAX_GRASP_ASSEMBLIES,
};
pub use placement::{GraspPlacement, SplitGraspPool};
mod pool;
pub(crate) use pool::worker_supports;
mod profile;
pub use profile::{
    ContactTreeSpec, FloatingTreeJointLimit, GraspTreeSpec, GroundedTreeSpec, MotorTreeSpec,
    MotorizedTreeConstruction, PayloadTreeConstruction, PayloadTreeSpec, StoppedTreeSpec,
    TreeConstruction,
};
mod worker;
pub use pool::{
    ContactTreePool, FloatingTreePool, GraspTreePool, GroundedTreePool, MotorTreePool,
    PayloadTreePool, StoppedTreePool, TreePool,
};
pub use worker::{
    serve, serve_contact, serve_exact_payload, serve_grasp, serve_grounded, serve_motor,
    serve_payload, serve_stopped, serve_vector,
};

type Result<T> = std::result::Result<T, ArticulationError>;
/// Capability negotiated with the worker executable on its private startup channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FloatingTreeWorkerProfile {
    ScalarV1,
    VectorV2,
    StoppedV1,
    MotorV1,
    GroundedV1,
    PayloadV2,
    ExactPayloadV3,
    GraspV1,
    ContactV1,
}
impl FloatingTreeWorkerProfile {
    fn flag(self) -> &'static str {
        match self {
            Self::ScalarV1 => "--managed-floating-tree-v1",
            Self::VectorV2 => "--managed-floating-tree-v2",
            Self::StoppedV1 => "--managed-stopped-tree-v1",
            Self::MotorV1 => "--managed-motor-tree-v1",
            Self::GroundedV1 => "--managed-grounded-tree-v1",
            Self::PayloadV2 => "--managed-payload-tree-v2",
            Self::ExactPayloadV3 => "--managed-payload-tree-v3",
            Self::GraspV1 => "--managed-grasp-tree-v1",
            Self::ContactV1 => "--managed-contact-tree-v1",
        }
    }
    fn magic(self) -> [u8; 8] {
        match self {
            Self::ScalarV1 => *b"GZFTWK01",
            Self::VectorV2 => *b"GZFTWK02",
            Self::StoppedV1 => *b"GZFSWK01",
            Self::MotorV1 => *b"GZFMWK01",
            Self::GroundedV1 => *b"GZFGWK01",
            Self::PayloadV2 => *b"GZFPWK02",
            Self::ExactPayloadV3 => *b"GZFPWK03",
            Self::GraspV1 => *b"GZFRWK01",
            Self::ContactV1 => *b"GZFIWK01",
        }
    }
}
/// Check the complete set before a worker or durable input is touched.
fn validate_efforts(
    inputs: &[(u64, f64)],
    joint_count: usize,
    mut locate: impl FnMut(u64) -> Option<(usize, Option<f64>)>,
) -> Result<()> {
    if inputs.len() > joint_count {
        return Err(invalid("too many floating tree effort targets"));
    }
    let mut seen = 0u64;
    for &(joint, torque) in inputs {
        let (index, held) = locate(joint).ok_or(invalid("unknown floating tree joint"))?;
        let mask = 1u64 << index;
        if seen & mask != 0 {
            return Err(invalid("duplicate floating tree effort target"));
        }
        if !(held.unwrap_or(0.) + torque).is_finite() {
            return Err(invalid("invalid accumulated floating tree effort"));
        }
        seen |= mask;
    }
    Ok(())
}
fn invalid(message: &'static str) -> ArticulationError {
    ArticulationError::Invalid(message)
}

/// One unbounded revolute with a world-space anchor and axis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FloatingTreeJointSpec {
    pub joint_id: u64,
    pub parent_id: u64,
    pub child_id: u64,
    pub anchor: [f64; 3],
    pub axis: [f64; 3],
}

/// A connected floating tree in zero gravity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FloatingTreeSpec {
    pub model_id: u64,
    /// Links in ascending public ID order, matching native body insertion.
    pub links: Vec<ArticulationLinkSpec>,
    /// Joints in the ordinary Bevy solver's topological insertion order.
    pub joints: Vec<FloatingTreeJointSpec>,
}

impl FloatingTreeSpec {
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::TreeWorld::new(self).map(|_| ())
    }

    fn fingerprint(&self) -> [u8; 32] {
        let mut hash = Sha256::new();
        hash.update(b"Gazebo floating tree 1 ms native recipe v1");
        bincode::serialize_into(&mut hash, self).expect("tree recipe writes to a SHA-256 digest");
        hash.finalize().into()
    }

    fn link_index(&self, id: u64) -> std::result::Result<usize, NativeError> {
        self.links
            .binary_search_by_key(&id, |link| link.id)
            .map_err(|_| NativeError::Invalid("unknown floating tree link"))
    }

    /// Validate the directed spanning tree and find its floating root.
    fn root(&self) -> std::result::Result<usize, NativeError> {
        use gz_physics_multifidelity::MAX_FLOATING_TREE_LINKS;
        directed_root(
            self.model_id,
            &self.links.iter().map(|l| l.id).collect::<Vec<_>>(),
            &self.joints,
            MAX_FLOATING_TREE_LINKS,
        )
    }
}

pub(crate) fn directed_root(
    model_id: u64,
    links: &[u64],
    joints: &[FloatingTreeJointSpec],
    limit: usize,
) -> std::result::Result<usize, NativeError> {
    let index = |id| {
        links
            .binary_search(&id)
            .map_err(|_| NativeError::Invalid("unknown floating tree link"))
    };
    let n = links.len();
    if !(3..=limit).contains(&n) || joints.len() != n - 1 || !links.windows(2).all(|w| w[0] < w[1])
    {
        return Err(NativeError::Invalid(
            "invalid floating tree size or construction order",
        ));
    }
    let mut ids: BTreeSet<u64> = links.iter().copied().collect();
    if !ids.insert(model_id) {
        return Err(NativeError::Invalid("duplicate floating tree model ID"));
    }
    let mut parent = vec![None; n];
    for joint in joints {
        let p = index(joint.parent_id)?;
        let c = index(joint.child_id)?;
        if !ids.insert(joint.joint_id) || p == c || parent[c].replace(p).is_some() {
            return Err(NativeError::Invalid(
                "duplicate floating tree joint or parent",
            ));
        }
    }
    let root = parent
        .iter()
        .position(Option::is_none)
        .ok_or(NativeError::Invalid("floating tree has no root"))?;
    let mut connected = BTreeSet::from([links[root]]);
    for joint in joints {
        if !connected.contains(&joint.parent_id) || !connected.insert(joint.child_id) {
            return Err(NativeError::Invalid(
                "disconnected, cyclic or unordered floating tree",
            ));
        }
    }
    Ok(root)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FloatingTreeJointState {
    pub joint_id: u64,
    pub q: f64,
    pub qdot: f64,
    pub pending_effort: Option<f64>,
}
impl PartialEq for FloatingTreeJointState {
    fn eq(&self, b: &Self) -> bool {
        self.joint_id == b.joint_id
            && self.q.to_bits() == b.q.to_bits()
            && self.qdot.to_bits() == b.qdot.to_bits()
            && self.pending_effort.map(f64::to_bits) == b.pending_effort.map(f64::to_bits)
    }
}
impl Eq for FloatingTreeJointState {}
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FloatingTreeObservation {
    pub tick: u64,
    pub links: Vec<ArticulationLinkState>,
    pub joints: Vec<FloatingTreeJointState>,
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct MotorSetting {
    target_bits: u64,
    force_bits: u64,
}
impl From<gz_physics::JointVelocityMotor> for MotorSetting {
    fn from(m: gz_physics::JointVelocityMotor) -> Self {
        Self {
            target_bits: m.target_velocity.to_bits(),
            force_bits: m.max_force.to_bits(),
        }
    }
}
impl MotorSetting {
    fn native(self) -> gz_physics::JointVelocityMotor {
        gz_physics::JointVelocityMotor {
            target_velocity: f64::from_bits(self.target_bits),
            max_force: f64::from_bits(self.force_bits),
        }
    }
    fn valid(self) -> bool {
        let motor = self.native();
        motor.target_velocity.is_finite() && motor.max_force.is_finite() && motor.max_force >= 0.
    }
}
#[derive(Clone, Serialize, Deserialize)]
struct GroundReceipts(Vec<gz_physics_multifidelity::GroundReaction>);
impl PartialEq for GroundReceipts {
    fn eq(&self, other: &Self) -> bool {
        self.0.len() == other.0.len()
            && self.0.iter().zip(&other.0).all(|(a, b)| {
                a.active == b.active
                    && a.normal_impulse.to_bits() == b.normal_impulse.to_bits()
                    && a.warmstart_normal_impulse.to_bits() == b.warmstart_normal_impulse.to_bits()
                    && a.tangent_impulse.map(f64::to_bits) == b.tangent_impulse.map(f64::to_bits)
            })
    }
}
impl Eq for GroundReceipts {}
#[derive(Clone, Serialize, Deserialize)]
struct PayloadReceipt {
    body: ArticulationLinkState,
    reactions: Vec<gz_physics_multifidelity::MovingReaction>,
}
impl PartialEq for PayloadReceipt {
    fn eq(&self, other: &Self) -> bool {
        self.body == other.body
            && self.reactions.len() == other.reactions.len()
            && self.reactions.iter().zip(&other.reactions).all(|(a, b)| {
                a.active == b.active
                    && a.normal.map(f64::to_bits) == b.normal.map(f64::to_bits)
                    && a.normal_impulse.to_bits() == b.normal_impulse.to_bits()
                    && a.warmstart_normal_impulse.to_bits() == b.warmstart_normal_impulse.to_bits()
                    && a.tangent_impulse.map(f64::to_bits) == b.tangent_impulse.map(f64::to_bits)
            })
    }
}
impl Eq for PayloadReceipt {}
fn valid_link(link: &ArticulationLinkState, id: u64) -> bool {
    link.id == id
        && link
            .position
            .iter()
            .chain(&link.orientation)
            .chain(&link.velocity)
            .chain(&link.angular_velocity)
            .all(|v| v.is_finite())
}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
enum Image {
    Cold(Vec<Option<u64>>),
    Warm(Vec<u8>),
    MotorCold {
        efforts: Vec<Option<u64>>,
        motors: Vec<Option<MotorSetting>>,
    },
    MotorWarm {
        native: Vec<u8>,
        motors: Vec<Option<MotorSetting>>,
    },
    GroundCold {
        efforts: Vec<Option<u64>>,
        reactions: GroundReceipts,
    },
    GroundWarm {
        native: Vec<u8>,
        reactions: GroundReceipts,
    },
    PayloadCold {
        efforts: Vec<Option<u64>>,
        payload: PayloadReceipt,
    },
    PayloadWarm {
        native: Vec<u8>,
        payload: PayloadReceipt,
    },
    GraspCold {
        efforts: Vec<Option<u64>>,
        motors: Vec<Option<MotorSetting>>,
        payload: PayloadReceipt,
    },
    GraspWarm {
        native: Vec<u8>,
        motors: Vec<Option<MotorSetting>>,
        payload: PayloadReceipt,
    },
}
impl Image {
    fn native(&self) -> Option<&[u8]> {
        match self {
            Self::Warm(b)
            | Self::MotorWarm { native: b, .. }
            | Self::GroundWarm { native: b, .. }
            | Self::PayloadWarm { native: b, .. }
            | Self::GraspWarm { native: b, .. } => Some(b),
            _ => None,
        }
    }
    fn payload(&self) -> Option<&PayloadReceipt> {
        match self {
            Self::PayloadCold { payload, .. }
            | Self::PayloadWarm { payload, .. }
            | Self::GraspCold { payload, .. }
            | Self::GraspWarm { payload, .. } => Some(payload),
            _ => None,
        }
    }
    fn reactions(&self) -> Option<&[gz_physics_multifidelity::GroundReaction]> {
        match self {
            Self::GroundCold { reactions, .. } | Self::GroundWarm { reactions, .. } => {
                Some(&reactions.0)
            }
            _ => None,
        }
    }
    fn motors(&self) -> Option<&[Option<MotorSetting>]> {
        match self {
            Self::MotorCold { motors, .. }
            | Self::MotorWarm { motors, .. }
            | Self::GraspCold { motors, .. }
            | Self::GraspWarm { motors, .. } => Some(motors),
            _ => None,
        }
    }
}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
struct Snapshot {
    state: FloatingTreeObservation,
    image: Image,
    digest: [u8; 32],
}
impl Snapshot {
    fn seal(recipe: [u8; 32], tick: u64, image: &Image) -> [u8; 32] {
        let mut h = Sha256::new();
        h.update(recipe);
        h.update(tick.to_le_bytes());
        bincode::serialize_into(&mut h, image).expect("tree image writes to a SHA-256 digest");
        h.finalize().into()
    }
    fn check<S: TreeConstruction>(&self, setup: &S, recipe: [u8; 32], tick: u64) -> Result<()> {
        let spec = setup.base();
        let state = &self.state;
        if state.tick != tick
            || self.digest != Self::seal(recipe, tick, &self.image)
            || state.links.len() != spec.links.len()
            || state.joints.len() != spec.joints.len()
            || !state
                .links
                .iter()
                .zip(&spec.links)
                .all(|(s, l)| valid_link(s, l.id))
            || !state
                .joints
                .windows(2)
                .all(|w| w[0].joint_id < w[1].joint_id)
            || !spec.joints.iter().all(|j| {
                state
                    .joints
                    .binary_search_by_key(&j.joint_id, |s| s.joint_id)
                    .is_ok()
            })
            || !state.joints.iter().all(|s| {
                s.q.is_finite() && s.qdot.is_finite() && s.pending_effort.is_none_or(f64::is_finite)
            })
        {
            return Err(invalid(
                "invalid floating tree state, identity or image digest",
            ));
        }
        let payload = self.image.payload();
        if payload.is_some() != setup.payload().is_some()
            || payload.zip(setup.payload()).is_some_and(|(p, expected)| {
                !valid_link(&p.body, expected.id)
                    || p.reactions.len() != state.links.len()
                    || !p.reactions.iter().all(|r| {
                        r.normal_impulse.is_finite()
                            && r.normal_impulse >= 0.
                            && r.warmstart_normal_impulse.is_finite()
                            && r.warmstart_normal_impulse >= 0.
                            && r.normal
                                .iter()
                                .chain(&r.tangent_impulse)
                                .all(|v| v.is_finite())
                    })
            })
        {
            return Err(invalid(
                "invalid external payload state or reaction receipt",
            ));
        }
        let ground = setup.ground().is_some();
        let reactions = self.image.reactions();
        if ground != reactions.is_some()
            || reactions.is_some_and(|r| {
                r.len() != state.links.len()
                    || !r.iter().all(|s| {
                        s.normal_impulse.is_finite()
                            && s.warmstart_normal_impulse.is_finite()
                            && s.normal_impulse >= 0.
                            && s.warmstart_normal_impulse >= 0.
                            && s.tangent_impulse.iter().all(|v| v.is_finite())
                    })
            })
        {
            return Err(invalid("invalid grounded tree reaction receipt"));
        }
        let motorized = S::MOTORIZED;
        let motors = self.image.motors();
        if motorized != motors.is_some()
            || motors.is_some_and(|m| {
                m.len() != state.joints.len() || !m.iter().flatten().all(|m| m.valid())
            })
        {
            return Err(invalid(
                "invalid floating tree native motor profile or settings",
            ));
        }
        match &self.image {
            Image::Cold(efforts)
            | Image::MotorCold { efforts, .. }
            | Image::GroundCold { efforts, .. }
            | Image::PayloadCold { efforts, .. }
            | Image::GraspCold { efforts, .. }
                if tick == 0
                    && efforts.iter().copied().eq(state
                        .joints
                        .iter()
                        .map(|s| s.pending_effort.map(f64::to_bits))) =>
            {
                Ok(())
            }
            Image::Warm(bytes)
            | Image::MotorWarm { native: bytes, .. }
            | Image::GroundWarm { native: bytes, .. }
            | Image::PayloadWarm { native: bytes, .. }
            | Image::GraspWarm { native: bytes, .. }
                if tick > 0
                    && bytes.starts_with(&setup.native_magic())
                    && (1..=S::IMAGE_LIMIT).contains(&bytes.len()) =>
            {
                Ok(())
            }
            _ => Err(invalid("incorrect floating tree image kind or size")),
        }
    }
}
