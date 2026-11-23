// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Finite lower and upper revolute coordinates in radians. The construction pose is zero.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FloatingTreeJointLimit {
    pub joint_id: u64,
    pub lower: f64,
    pub upper: f64,
}

/// A floating tree with one or more finite stops, ordered by public joint ID.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StoppedTreeSpec {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
}
impl StoppedTreeSpec {
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::TreeWorld::new(self).map(|_| ())
    }
    fn validate_stops(&self) -> std::result::Result<(), NativeError> {
        validate_stops(&self.tree, &self.limits, true)
    }
}
/// Passive floating tree with a native, guarded point-impulse interface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactTreeSpec {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
}
/// A floating tree admitting persistent bounded velocity motors, optionally with finite stops.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorTreeSpec {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
}
impl MotorTreeSpec {
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::TreeWorld::new(self).map(|_| ())
    }
}
/// A floating tree on one fixed +Z plane under downward gravity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundedTreeSpec {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
    pub ground: crate::articulation::ArticulationGroundSpec,
}
impl GroundedTreeSpec {
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::TreeWorld::new(self).map(|_| ())
    }
    fn validate_ground(&self) -> std::result::Result<(), NativeError> {
        validate_stops(&self.tree, &self.limits, false)?;
        self.tree.root()?;
        let g = self.ground;
        if self.tree.links.len() > gz_physics_multifidelity::MAX_GROUNDED_TREE_LINKS
            || g.link_id == self.tree.model_id
            || self.tree.joints.iter().any(|j| j.joint_id == g.link_id)
            || (self.tree.links[0].id..=self.tree.links[self.tree.links.len() - 1].id)
                .contains(&g.link_id)
            || !g.height.is_finite()
            || !g.size.iter().all(|s| s.is_finite() && *s > 0.)
            || !g.gravity_z.is_finite()
            || g.gravity_z >= 0.
        {
            return Err(NativeError::Invalid(
                "invalid grounded tree support identity, geometry or gravity",
            ));
        }
        Ok(())
    }
}
/// A capped-motor gripper acting on one independently massed spherical part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraspTreeSpec {
    pub payload: PayloadTreeSpec,
}
impl GraspTreeSpec {
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::TreeWorld::new(self).map(|_| ())
    }
}
/// A floating revolute tree interacting with a separate, independently massed sphere.
#[derive(Clone, Debug, PartialEq)]
pub struct PayloadTreeSpec {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
    pub payload: ArticulationLinkSpec,
    /// SDF allow_auto_disable on the independently simulated body.
    pub payload_can_sleep: bool,
    /// Resolve stationary zero-width revolutes as exact native locks.
    pub exact_revolute_locks: bool,
}
fn allow_payload_sleep() -> bool {
    true
}
// The fourth v2 binary field was a bool. Its 0/1 representations remain byte-identical.
// Only v3 admits bit 1. The old JSON worker receives exactly the original four keys.
impl Serialize for PayloadTreeSpec {
    fn serialize<S: serde::Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        if s.is_human_readable() {
            use serde::ser::SerializeStruct;
            let mut fields = s.serialize_struct(
                "PayloadTreeSpec",
                4 + usize::from(self.exact_revolute_locks),
            )?;
            fields.serialize_field("tree", &self.tree)?;
            fields.serialize_field("limits", &self.limits)?;
            fields.serialize_field("payload", &self.payload)?;
            fields.serialize_field("payload_can_sleep", &self.payload_can_sleep)?;
            if self.exact_revolute_locks {
                fields.serialize_field("exact_revolute_locks", &true)?;
            }
            fields.end()
        } else {
            let flags =
                u8::from(self.payload_can_sleep) | (u8::from(self.exact_revolute_locks) << 1);
            (&self.tree, &self.limits, &self.payload, flags).serialize(s)
        }
    }
}
impl<'de> Deserialize<'de> for PayloadTreeSpec {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Self, D::Error> {
        if d.is_human_readable() {
            #[derive(Deserialize)]
            #[serde(deny_unknown_fields)]
            struct Fields {
                tree: FloatingTreeSpec,
                limits: Vec<FloatingTreeJointLimit>,
                payload: ArticulationLinkSpec,
                #[serde(default = "allow_payload_sleep")]
                payload_can_sleep: bool,
                #[serde(default)]
                exact_revolute_locks: bool,
            }
            let Fields {
                tree,
                limits,
                payload,
                payload_can_sleep,
                exact_revolute_locks,
            } = Fields::deserialize(d)?;
            Ok(Self {
                tree,
                limits,
                payload,
                payload_can_sleep,
                exact_revolute_locks,
            })
        } else {
            let (tree, limits, payload, flags): (_, _, _, u8) = Deserialize::deserialize(d)?;
            if flags > 3 {
                return Err(serde::de::Error::custom(
                    "unknown payload construction flags",
                ));
            }
            Ok(Self {
                tree,
                limits,
                payload,
                payload_can_sleep: flags & 1 != 0,
                exact_revolute_locks: flags & 2 != 0,
            })
        }
    }
}
impl PayloadTreeSpec {
    pub fn validate(&self) -> std::result::Result<(), NativeError> {
        native::TreeWorld::new(self).map(|_| ())
    }
    fn validate_payload(&self) -> std::result::Result<(), NativeError> {
        validate_stops(&self.tree, &self.limits, false)?;
        self.tree.root()?;
        let id = self.payload.id;
        if self.tree.links.len() > gz_physics_multifidelity::MAX_PAYLOAD_TREE_LINKS
            || id == self.tree.model_id
            || self.tree.joints.iter().any(|j| j.joint_id == id)
            || (self.tree.links[0].id..=self.tree.links[self.tree.links.len() - 1].id).contains(&id)
        {
            return Err(NativeError::Invalid(
                "invalid payload identity, size or construction order",
            ));
        }
        Ok(())
    }
}
fn validate_stops(
    tree: &FloatingTreeSpec,
    limits: &[FloatingTreeJointLimit],
    required: bool,
) -> std::result::Result<(), NativeError> {
    if (required && limits.is_empty())
        || limits.len() > tree.joints.len()
        || !limits.windows(2).all(|w| w[0].joint_id < w[1].joint_id)
        || !limits.iter().all(|s| {
            s.lower.is_finite()
                && s.upper.is_finite()
                && s.lower <= 0.
                && s.upper >= 0.
                && tree.joints.iter().any(|j| j.joint_id == s.joint_id)
        })
    {
        return Err(NativeError::Invalid(
            "invalid floating tree stop range or target order",
        ));
    }
    Ok(())
}
/// Construction recipes admitted by the durable tree controller.
pub trait TreeConstruction:
    sealed::Recipe + Clone + Serialize + serde::de::DeserializeOwned
{
}
impl TreeConstruction for FloatingTreeSpec {}
impl TreeConstruction for StoppedTreeSpec {}
impl TreeConstruction for MotorTreeSpec {}
impl TreeConstruction for ContactTreeSpec {}
impl TreeConstruction for GroundedTreeSpec {}
impl TreeConstruction for PayloadTreeSpec {}
impl TreeConstruction for GraspTreeSpec {}
/// Recipes whose worker accepts native persistent velocity motor commands.
pub trait MotorizedTreeConstruction: TreeConstruction {}
impl MotorizedTreeConstruction for MotorTreeSpec {}
impl MotorizedTreeConstruction for GraspTreeSpec {}
/// Recipes that include a separately massed dynamic payload with per-link contact readback.
pub trait PayloadTreeConstruction: TreeConstruction {}
impl PayloadTreeConstruction for PayloadTreeSpec {}
impl PayloadTreeConstruction for GraspTreeSpec {}

pub(super) mod sealed {
    use super::*;
    pub trait Recipe {
        const MOTORIZED: bool = false;
        const FILE: &'static str;
        const ARCHIVE_MAGIC: [u8; 8];
        const NATIVE_MAGIC: [u8; 8];
        const IMAGE_LIMIT: usize;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile>;
        fn base(&self) -> &FloatingTreeSpec;
        fn required_worker(&self) -> Option<FloatingTreeWorkerProfile> {
            Self::REQUIRED_WORKER
        }
        fn native_magic(&self) -> [u8; 8] {
            Self::NATIVE_MAGIC
        }
        fn archive_magic(&self) -> [u8; 8] {
            Self::ARCHIVE_MAGIC
        }
        fn exact_locks(&self) -> bool {
            false
        }
        fn payload(&self) -> Option<&ArticulationLinkSpec> {
            None
        }
        fn payload_can_sleep(&self) -> bool {
            true
        }
        fn ground(&self) -> Option<crate::articulation::ArticulationGroundSpec> {
            None
        }
        fn limits(&self) -> &[FloatingTreeJointLimit];
        fn validate_configuration(&self) -> std::result::Result<(), NativeError>;
        fn digest(&self) -> [u8; 32];
    }
    impl Recipe for FloatingTreeSpec {
        const FILE: &'static str = crate::articulation::ledger::FLOATING_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFTCU01";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFTRE01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::FLOATING_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> = None;
        fn base(&self) -> &FloatingTreeSpec {
            self
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &[]
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            Ok(())
        }
        fn digest(&self) -> [u8; 32] {
            self.fingerprint()
        }
    }
    impl Recipe for StoppedTreeSpec {
        const FILE: &'static str = crate::articulation::ledger::STOPPED_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFSCU01";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFSTP01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::STOPPED_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> =
            Some(FloatingTreeWorkerProfile::StoppedV1);
        fn base(&self) -> &FloatingTreeSpec {
            &self.tree
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &self.limits
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            self.validate_stops()
        }
        fn digest(&self) -> [u8; 32] {
            let mut hash = Sha256::new();
            hash.update(b"Gazebo finite-stop floating tree 1 ms native recipe v1");
            bincode::serialize_into(&mut hash, self).expect("stopped recipe writes to SHA-256");
            hash.finalize().into()
        }
    }
    impl Recipe for ContactTreeSpec {
        const FILE: &'static str = crate::articulation::ledger::CONTACT_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFICU01";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFTRE01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::STOPPED_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> =
            Some(FloatingTreeWorkerProfile::ContactV1);
        fn base(&self) -> &FloatingTreeSpec {
            &self.tree
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &self.limits
        }
        fn native_magic(&self) -> [u8; 8] {
            if self.limits.is_empty() {
                Self::NATIVE_MAGIC
            } else {
                *b"GZFSTP01"
            }
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            if self.tree.links.len() > 32 {
                return Err(NativeError::Invalid(
                    "contact tree supports at most 32 passive links",
                ));
            }
            validate_stops(&self.tree, &self.limits, false)
        }
        fn digest(&self) -> [u8; 32] {
            let mut h = Sha256::new();
            h.update(b"Gazebo guarded passive point-impulse tree 1ms v1");
            bincode::serialize_into(&mut h, self).expect("recipe writes to SHA-256");
            h.finalize().into()
        }
    }
    impl Recipe for MotorTreeSpec {
        const MOTORIZED: bool = true;
        const FILE: &'static str = crate::articulation::ledger::MOTOR_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFMCU01";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFMOT01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::MOTOR_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> =
            Some(FloatingTreeWorkerProfile::MotorV1);
        fn base(&self) -> &FloatingTreeSpec {
            &self.tree
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &self.limits
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            validate_stops(&self.tree, &self.limits, false)
        }
        fn digest(&self) -> [u8; 32] {
            let mut hash = Sha256::new();
            hash.update(b"Gazebo persistent velocity-motor floating tree 1 ms native recipe v1");
            bincode::serialize_into(&mut hash, self).expect("stopped recipe writes to SHA-256");
            hash.finalize().into()
        }
    }
    impl Recipe for GroundedTreeSpec {
        const FILE: &'static str = crate::articulation::ledger::GROUNDED_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFGCU01";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFGND01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::GROUNDED_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> =
            Some(FloatingTreeWorkerProfile::GroundedV1);
        fn base(&self) -> &FloatingTreeSpec {
            &self.tree
        }
        fn ground(&self) -> Option<crate::articulation::ArticulationGroundSpec> {
            Some(self.ground)
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &self.limits
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            self.validate_ground()
        }
        fn digest(&self) -> [u8; 32] {
            let mut hash = Sha256::new();
            hash.update(
                b"Gazebo floating tree with one fixed floor and gravity 1 ms native recipe v1",
            );
            bincode::serialize_into(&mut hash, self).expect("stopped recipe writes to SHA-256");
            hash.finalize().into()
        }
    }
    impl Recipe for PayloadTreeSpec {
        const FILE: &'static str = crate::articulation::ledger::PAYLOAD_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFPCU02";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFPAY01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::PAYLOAD_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> =
            Some(FloatingTreeWorkerProfile::PayloadV2);
        fn required_worker(&self) -> Option<FloatingTreeWorkerProfile> {
            if self.exact_revolute_locks {
                Some(FloatingTreeWorkerProfile::ExactPayloadV3)
            } else {
                Self::REQUIRED_WORKER
            }
        }
        fn archive_magic(&self) -> [u8; 8] {
            if self.exact_revolute_locks {
                *b"GZFPCU03"
            } else {
                Self::ARCHIVE_MAGIC
            }
        }
        fn exact_locks(&self) -> bool {
            self.exact_revolute_locks
        }
        fn base(&self) -> &FloatingTreeSpec {
            &self.tree
        }
        fn payload(&self) -> Option<&ArticulationLinkSpec> {
            Some(&self.payload)
        }
        fn payload_can_sleep(&self) -> bool {
            self.payload_can_sleep
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &self.limits
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            self.validate_payload()
        }
        fn digest(&self) -> [u8; 32] {
            let mut hash = Sha256::new();
            hash.update(
                b"Gazebo floating tree with one dynamic spherical payload 1 ms native recipe v2",
            );
            bincode::serialize_into(&mut hash, self).expect("stopped recipe writes to SHA-256");
            hash.finalize().into()
        }
    }
    impl Recipe for GraspTreeSpec {
        const MOTORIZED: bool = true;
        const FILE: &'static str = crate::articulation::ledger::GRASP_TREE_FILE;
        const ARCHIVE_MAGIC: [u8; 8] = *b"GZFRCU01";
        const NATIVE_MAGIC: [u8; 8] = *b"GZFGRP01";
        const IMAGE_LIMIT: usize = gz_physics_multifidelity::GRASP_TREE_IMAGE_LIMIT;
        const REQUIRED_WORKER: Option<FloatingTreeWorkerProfile> =
            Some(FloatingTreeWorkerProfile::GraspV1);
        fn base(&self) -> &FloatingTreeSpec {
            &self.payload.tree
        }
        fn payload(&self) -> Option<&ArticulationLinkSpec> {
            Some(&self.payload.payload)
        }
        fn payload_can_sleep(&self) -> bool {
            self.payload.payload_can_sleep
        }
        fn exact_locks(&self) -> bool {
            self.payload.exact_revolute_locks
        }
        fn limits(&self) -> &[FloatingTreeJointLimit] {
            &self.payload.limits
        }
        fn validate_configuration(&self) -> std::result::Result<(), NativeError> {
            self.payload.validate_payload()
        }
        fn digest(&self) -> [u8; 32] {
            let mut hash = Sha256::new();
            hash.update(b"Gazebo persistent velocity motors, one dynamic payload and signed world force, 1ms native recipe v1");
            bincode::serialize_into(&mut hash, self).expect("grasp recipe writes to SHA-256");
            hash.finalize().into()
        }
    }
}
