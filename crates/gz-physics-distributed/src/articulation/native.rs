// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Construction-bound state of a two-link floating revolute and its optional fixed ground in a private Bevy world.
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, Inertia, JointId, PhysicsBackend, PhysicsError, PlaneSpec,
    RotationSpec, Shape,
};
pub use gz_physics_multifidelity::GroundReaction;
use gz_physics_multifidelity::{GROUNDED_REVOLUTE_LIMIT, MultiFidelityBackend};
use gz_sim_core::{Simulation, StepError, UpdateInfo};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const COLD: &[u8; 8] = b"GZHINC01";
const WARM: &[u8; 8] = b"GZHINW01";
const GROUND_COLD: &[u8; 8] = b"GZGRDC01";
const GROUND_WARM: &[u8; 8] = b"GZGRDW01";
const HEADER: usize = 40;
const ISOLATED_IMAGE_LIMIT: usize = 512;
/// Inclusive limit for a construction-bound worker image, including its envelope.
pub const MAX_ARTICULATION_IMAGE: usize = HEADER + GROUNDED_REVOLUTE_LIMIT;

/// A centered spherical collider on an independently massed link. Poses are in
/// world coordinates. The inertial frame and its tensor are relative to the link.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArticulationLinkSpec {
    pub id: u64,
    pub radius: f64,
    pub mass: f64,
    pub position: [f64; 3],
    /// XYZW quaternion.
    pub orientation: [f64; 4],
    /// Center-of-mass velocity in world axes, in meters per second.
    pub velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub inertia_center: [f64; 3],
    pub inertia_orientation: [f64; 4],
    /// Components in the inertial frame, ordered xx, yy, zz, xy, xz, yz.
    pub inertia_tensor: [f64; 6],
}

impl ArticulationLinkSpec {
    fn body(&self) -> (BodySpec, RotationSpec, Inertia) {
        (
            BodySpec {
                position: self.position,
                velocity: self.velocity,
                mass: Some(self.mass),
                shape: Shape::Sphere {
                    radius: self.radius,
                },
            },
            RotationSpec {
                orientation: self.orientation,
                angular_velocity: self.angular_velocity,
            },
            Inertia {
                center: self.inertia_center,
                orientation: self.inertia_orientation,
                tensor: self.inertia_tensor,
            },
        )
    }
    pub(crate) fn spawn(&self, backend: &mut MultiFidelityBackend) -> Result<BodyId, PhysicsError> {
        let (body, rotation, inertia) = self.body();
        backend.spawn_articulation_link(body, rotation, Some(inertia))
    }
    pub(crate) fn spawn_dynamic(
        &self,
        backend: &mut MultiFidelityBackend,
    ) -> Result<BodyId, PhysicsError> {
        let (body, rotation, inertia) = self.body();
        backend.spawn_link(body, rotation, Some(inertia))
    }
}

/// One fixed horizontal halfspace, using the native solver's default material.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArticulationGroundSpec {
    pub link_id: u64,
    pub height: f64,
    pub size: [f64; 2],
    pub gravity_z: f64,
}

impl ArticulationGroundSpec {
    pub(crate) fn plane(&self) -> PlaneSpec {
        PlaneSpec {
            position: [0., 0., self.height],
            orientation: [0., 0., 0., 1.],
            size: self.size,
        }
    }
}

/// Floating hinge using one 1 ms native step per barrier.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArticulationSpec {
    /// Public model ID and the shared self-collision exclusion group.
    pub model_id: u64,
    /// Parent first, child second. Link and joint IDs share the public ID space.
    pub links: [ArticulationLinkSpec; 2],
    pub joint_id: u64,
    /// World-space hinge point, in meters.
    pub anchor: [f64; 3],
    /// World-space revolute axis. Native construction normalizes this direction.
    pub axis: [f64; 3],
    // Omitting the final field for the isolated profile preserves its v1 JSON
    // schema and its bincode recipe fingerprint and startup digest.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ground: Option<ArticulationGroundSpec>,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArticulationLinkState {
    pub id: u64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArticulationObservation {
    pub tick: u64,
    pub joint_id: u64,
    pub links: [ArticulationLinkState; 2],
    pub q: f64,
    pub qdot: f64,
    /// Per-link solver reactions in parent, child order for the last 1 ms step.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ground_reactions: Option<[GroundReaction; 2]>,
}

// A published cut is an exact native receipt. Signed zero carries direction in
// some f64 operations and survives the worker and archive codecs.
impl PartialEq for ArticulationLinkState {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
            && self.position.map(f64::to_bits) == other.position.map(f64::to_bits)
            && self.orientation.map(f64::to_bits) == other.orientation.map(f64::to_bits)
            && self.velocity.map(f64::to_bits) == other.velocity.map(f64::to_bits)
            && self.angular_velocity.map(f64::to_bits) == other.angular_velocity.map(f64::to_bits)
    }
}
impl Eq for ArticulationLinkState {}

impl PartialEq for ArticulationObservation {
    fn eq(&self, other: &Self) -> bool {
        self.tick == other.tick
            && self.joint_id == other.joint_id
            && self.links == other.links
            && self.q.to_bits() == other.q.to_bits()
            && self.qdot.to_bits() == other.qdot.to_bits()
            && match (&self.ground_reactions, &other.ground_reactions) {
                (Some(a), Some(b)) => a.iter().zip(b).all(|(a, b)| {
                    a.active == b.active
                        && a.normal_impulse.to_bits() == b.normal_impulse.to_bits()
                        && a.warmstart_normal_impulse.to_bits()
                            == b.warmstart_normal_impulse.to_bits()
                        && a.tangent_impulse.map(f64::to_bits)
                            == b.tangent_impulse.map(f64::to_bits)
                }),
                (None, None) => true,
                _ => false,
            }
    }
}
impl Eq for ArticulationObservation {}

#[derive(Debug, thiserror::Error)]
pub enum NativeError {
    #[error("{0}")]
    Invalid(&'static str),
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Step(#[from] StepError),
}

pub(crate) struct Construction {
    pub(crate) backend: MultiFidelityBackend,
    pub(crate) bodies: [BodyId; 2],
    pub(crate) joint: JointId,
    ground: Option<BodyId>,
}

impl ArticulationSpec {
    pub(crate) fn image_limit(&self) -> usize {
        if self.ground.is_some() {
            MAX_ARTICULATION_IMAGE
        } else {
            ISOLATED_IMAGE_LIMIT
        }
    }

    pub(crate) fn build(&self) -> Result<Construction, NativeError> {
        let [parent, child] = self.links;
        if parent.id == child.id
            || [parent.id, child.id].contains(&self.joint_id)
            || [parent.id, child.id, self.joint_id].contains(&self.model_id)
        {
            return Err(NativeError::Invalid("duplicate articulation entity ID"));
        }
        if let Some(ground) = self.ground
            && ([self.model_id, self.joint_id, parent.id, child.id].contains(&ground.link_id)
                || (parent.id.min(child.id)..=parent.id.max(child.id)).contains(&ground.link_id)
                || !ground.height.is_finite()
                || !ground.size.iter().all(|v| v.is_finite() && *v > 0.)
                || !ground.gravity_z.is_finite()
                || ground.gravity_z >= 0.)
        {
            return Err(NativeError::Invalid(
                "invalid ground identity, construction order or geometry",
            ));
        }
        let mut backend =
            MultiFidelityBackend::new([0., 0., self.ground.map_or(0., |g| g.gravity_z)])?;
        let before = self.ground.filter(|g| g.link_id < parent.id.min(child.id));
        let mut ground = before
            .map(|g| backend.spawn_static_plane(g.plane()))
            .transpose()?;
        let mut spawn = |link: &ArticulationLinkSpec| link.spawn(&mut backend);
        // Link entity IDs record the authored insertion order. Rapier binds native
        // handles into the construction signature even when the child was authored first.
        let (parent, child) = if parent.id < child.id {
            (spawn(&parent)?, spawn(&child)?)
        } else {
            let child = spawn(&child)?;
            (spawn(&parent)?, child)
        };
        for id in [parent, child] {
            backend.set_self_collision_group(id, Some(self.model_id))?;
        }
        let joint = backend.add_reduced_revolute(Some(parent), child, self.anchor, self.axis)?;
        if let Some(g) = self.ground.filter(|_| ground.is_none()) {
            ground = Some(backend.spawn_static_plane(g.plane())?);
        }
        Ok(Construction {
            backend,
            bodies: [parent, child],
            joint,
            ground,
        })
    }

    /// Apply the same construction and motion checks as the worker's native solver.
    pub fn validate(&self) -> Result<(), NativeError> {
        self.build().map(|_| ())
    }

    fn fingerprint(&self) -> [u8; 32] {
        let mut hash = Sha256::new();
        if self.ground.is_some() {
            hash.update(b"Gazebo grounded floating hinge construction v1, 1 ms");
        } else {
            hash.update(b"Gazebo isolated floating hinge construction v1, zero-g, 1 ms");
        }
        // All fields have a fixed binary representation, including authored ID order and signed zeros.
        bincode::serialize_into(&mut hash, self).expect("fixed hinge recipe writes to SHA-256");
        hash.finalize().into()
    }

    /// Validate a worker receipt before persisting it. This checks the bounded
    /// envelope and nested image format without constructing a native graph.
    /// Full numerical and construction validation runs when restoring the graph.
    pub fn verify_image(&self, tick: u64, image: &[u8]) -> Result<(), NativeError> {
        payload(&self.fingerprint(), self.ground.is_some(), tick, image).map(|_| ())
    }

    /// Return the native Rapier image after checking its profile, recipe and simulation tick.
    pub fn warm_payload<'a>(&self, tick: u64, image: &'a [u8]) -> Result<&'a [u8], NativeError> {
        if tick == 0 {
            return Err(NativeError::Invalid(
                "warm hinge payload requires a committed step",
            ));
        }
        payload(&self.fingerprint(), self.ground.is_some(), tick, image)
    }
}

fn checksum(recipe: &[u8; 32], tick: u64, magic: &[u8; 8], data: &[u8]) -> [u8; 32] {
    let mut hash = Sha256::new();
    hash.update(magic);
    hash.update(recipe);
    hash.update(tick.to_le_bytes());
    hash.update(data);
    hash.finalize().into()
}

fn payload<'a>(
    recipe: &[u8; 32],
    grounded: bool,
    tick: u64,
    image: &'a [u8],
) -> Result<&'a [u8], NativeError> {
    let magic = match (grounded, tick == 0) {
        (false, true) => COLD,
        (false, false) => WARM,
        (true, true) => GROUND_COLD,
        (true, false) => GROUND_WARM,
    };
    let limit = if grounded {
        MAX_ARTICULATION_IMAGE
    } else {
        ISOLATED_IMAGE_LIMIT
    };
    if !(HEADER..=limit).contains(&image.len())
        || &image[..8] != magic
        || image[8..HEADER] != checksum(recipe, tick, magic, &image[HEADER..])
    {
        return Err(NativeError::Invalid(
            "invalid construction-bound articulation image",
        ));
    }
    let data = &image[HEADER..];
    if tick == 0 {
        cold_effort(data)?;
    } else if grounded {
        if data.len() <= HEADER
            || &data[..8] != b"GZAGR001"
            || data[8..HEADER] != Sha256::digest(&data[HEADER..])[..]
        {
            return Err(NativeError::Invalid(
                "invalid grounded native hinge envelope",
            ));
        }
    } else {
        // Pinned GZART001 fixed-width payload: its Option<f64> at offset 360
        // accounts for the eight-byte difference between the two images.
        let expected_tag = match data.len() {
            369 => 0,
            377 => 1,
            _ => return Err(NativeError::Invalid("invalid native hinge image length")),
        };
        if &data[..8] != b"GZART001"
            || data[360] != expected_tag
            || data[8..40] != Sha256::digest(&data[40..])[..]
        {
            return Err(NativeError::Invalid("invalid native hinge image envelope"));
        }
    }
    Ok(data)
}

fn cold_effort(data: &[u8]) -> Result<Option<f64>, NativeError> {
    if data.is_empty() {
        return Ok(None);
    }
    let bytes = data
        .try_into()
        .map_err(|_| NativeError::Invalid("invalid cold effort length"))?;
    let value = f64::from_le_bytes(bytes);
    // Native effort accumulation starts at positive zero.
    if !value.is_finite() || value.to_bits() == (-0.0f64).to_bits() {
        return Err(NativeError::Invalid("invalid cold effort"));
    }
    Ok(Some(value))
}

/// A private ECS simulation with one indivisible native graph. The definition
/// and public/native mapping are rebuilt before admitting any transferred motion.
pub struct NativeArticulation {
    sim: Simulation,
    recipe: [u8; 32],
    public_links: [u64; 2],
    public_joint: u64,
    ground: Option<BodyId>,
    bodies: [BodyId; 2],
    joint: JointId,
    // Native moving images carry the accumulator. Before the first integration
    // the cold recipe and the same accumulator are the complete restart state.
}

impl NativeArticulation {
    pub fn new(spec: &ArticulationSpec) -> Result<Self, NativeError> {
        Self::start(spec, 0, None)
    }

    pub fn restore(spec: &ArticulationSpec, tick: u64, image: &[u8]) -> Result<Self, NativeError> {
        Self::start(spec, tick, Some(image))
    }

    fn start(
        spec: &ArticulationSpec,
        tick: u64,
        image: Option<&[u8]>,
    ) -> Result<Self, NativeError> {
        let recipe = spec.fingerprint();
        let data = image
            .map(|bytes| payload(&recipe, spec.ground.is_some(), tick, bytes))
            .transpose()?;
        let Construction {
            mut backend,
            bodies,
            joint,
            ground,
        } = spec.build()?;
        if let Some(data) = data {
            if tick > 0 {
                if ground.is_some() {
                    backend.restore_grounded_revolute(data)?;
                } else {
                    backend.restore_floating_revolute(data)?;
                }
            } else if let Some(value) = cold_effort(data)? {
                backend.add_joint_effort(joint, value)?;
            }
        }
        let mut sim = if tick == 0 {
            Simulation::new(DT)?
        } else {
            Simulation::from_committed_clock(
                DT,
                UpdateInfo {
                    start_time: Duration::from_millis(tick - 1),
                    sim_time: Duration::from_millis(tick),
                    dt: DT,
                    iteration: tick,
                    epoch: 0,
                    paused: false,
                },
                true,
            )?
        };
        gz_physics::install(&mut sim, backend)?;
        Ok(Self {
            sim,
            recipe,
            public_links: spec.links.map(|link| link.id),
            public_joint: spec.joint_id,
            ground,
            bodies,
            joint,
        })
    }

    pub(crate) fn tick(&self) -> u64 {
        self.sim.world().resource::<UpdateInfo>().iteration
    }

    pub fn observe(&self) -> ArticulationObservation {
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let joint = backend.joint_state(self.joint).expect("owned hinge");
        let links = std::array::from_fn(|i| {
            let state = backend.state(self.bodies[i]).expect("owned hinge link");
            ArticulationLinkState {
                id: self.public_links[i],
                position: state.position,
                orientation: state.orientation,
                velocity: state.velocity,
                angular_velocity: state.angular_velocity,
            }
        });
        ArticulationObservation {
            tick: self.tick(),
            joint_id: self.public_joint,
            links,
            q: joint.position,
            qdot: joint.velocity,
            ground_reactions: self.ground.map(|ground| {
                let native = backend
                    .downcast_ref::<MultiFidelityBackend>()
                    .expect("owned hinge solver");
                self.bodies.map(|body| {
                    native
                        .ground_reaction(body, ground)
                        .expect("owned hinge and ground")
                })
            }),
        }
    }

    /// Read the native joint's held one-step command after construction or restore.
    pub fn pending_joint_effort(&self) -> Option<f64> {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned hinge solver")
            .pending_joint_effort(self.joint)
    }

    pub fn capture(&self) -> Result<Vec<u8>, NativeError> {
        let tick = self.sim.committed_update_info()?.iteration;
        let (magic, data) = if tick == 0 {
            (
                if self.ground.is_some() {
                    GROUND_COLD
                } else {
                    COLD
                },
                self.pending_joint_effort()
                    .map_or_else(Vec::new, |e| e.to_le_bytes().to_vec()),
            )
        } else {
            let backend = self.sim.world().resource::<PhysicsBackend>();
            let native = backend
                .downcast_ref::<MultiFidelityBackend>()
                .expect("owned hinge solver");
            if self.ground.is_some() {
                (GROUND_WARM, native.checkpoint_grounded_revolute()?)
            } else {
                (WARM, native.checkpoint_floating_revolute()?)
            }
        };
        if data.len()
            > (if self.ground.is_some() {
                MAX_ARTICULATION_IMAGE
            } else {
                ISOLATED_IMAGE_LIMIT
            }) - HEADER
        {
            return Err(NativeError::Invalid(
                "articulation image exceeds protocol limit",
            ));
        }
        let mut bytes = Vec::with_capacity(HEADER + data.len());
        bytes.extend_from_slice(magic);
        bytes.extend_from_slice(&checksum(&self.recipe, tick, magic, &data));
        bytes.extend(data);
        Ok(bytes)
    }

    pub fn effort(&mut self, joint_id: u64, value: f64) -> Result<(), NativeError> {
        if joint_id != self.public_joint {
            return Err(NativeError::Invalid("unknown articulation joint ID"));
        }
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_joint_effort(self.joint, value)?;
        Ok(())
    }

    pub fn advance(&mut self) -> Result<(), NativeError> {
        self.sim.step_exact(1)?;
        Ok(())
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    pub(in crate::articulation) fn spec() -> ArticulationSpec {
        let parent = ArticulationLinkSpec {
            id: 37,
            radius: 0.16,
            mass: 2.,
            position: [0., 0., 1.],
            orientation: [0., 0., 0., 1.],
            velocity: [0.21, -0.04, 0.08],
            angular_velocity: [0.; 3],
            inertia_center: [0.07, -0.03, 0.],
            inertia_orientation: [0., 0., 0., 1.],
            inertia_tensor: [0.12, 0.12, 0.12, 0., 0., 0.],
        };
        let child = ArticulationLinkSpec {
            id: 12,
            position: [0.65, 0.25, 1.],
            mass: 0.8,
            radius: 0.12,
            inertia_center: [-0.04, 0.02, 0.],
            inertia_tensor: [0.036, 0.034, 0.038, 0.001, 0., 0.],
            ..parent
        };
        let unit = 14f64.sqrt().recip();
        ArticulationSpec {
            ground: None,
            model_id: 3,
            links: [parent, child],
            joint_id: 99,
            anchor: [0.33, 0.06, 1.],
            axis: [unit, 2. * unit, 3. * unit],
        }
    }

    #[test]
    fn ground_recipe_requires_a_fixed_halfspace_outside_the_authored_tree() {
        let isolated = spec();
        let good = ArticulationGroundSpec {
            link_id: 1,
            height: -0.25,
            size: [6., 8.],
            gravity_z: -9.81,
        };
        let with = |ground| ArticulationSpec {
            ground: Some(ground),
            ..isolated
        };
        let early = with(good);
        let late = with(ArticulationGroundSpec {
            link_id: 109,
            ..good
        });
        assert!(early.validate().is_ok() && late.validate().is_ok());
        assert_ne!(early.fingerprint(), late.fingerprint());
        for bad in [
            ArticulationGroundSpec {
                link_id: isolated.model_id,
                ..good
            },
            ArticulationGroundSpec {
                link_id: isolated.joint_id,
                ..good
            },
            ArticulationGroundSpec {
                link_id: isolated.links[0].id,
                ..good
            },
            ArticulationGroundSpec {
                link_id: 25,
                ..good
            },
            ArticulationGroundSpec {
                height: f64::NAN,
                ..good
            },
            ArticulationGroundSpec {
                size: [6., 0.],
                ..good
            },
            ArticulationGroundSpec {
                gravity_z: f64::NEG_INFINITY,
                ..good
            },
            ArticulationGroundSpec {
                gravity_z: 0.,
                ..good
            },
        ] {
            assert!(with(bad).validate().is_err());
        }
    }

    #[test]
    fn cold_and_warm_cuts_recover_the_exact_pending_effort_and_clock() -> Result<(), NativeError> {
        let spec = spec();
        let joint = spec.joint_id;
        let mut control = NativeArticulation::new(&spec)?;
        let mut journey = NativeArticulation::new(&spec)?;
        let empty = control.capture()?;
        // Start both with a held zero, distinct from no accepted input.
        control.effort(joint, -0.)?;
        journey.effort(joint, -0.)?;
        assert_ne!(empty, control.capture()?);
        for tick in 0..192 {
            let torque = if tick < 72 { 1.75 } else { -0.8125 };
            if tick % 3 == 0 {
                for world in [&mut control, &mut journey] {
                    world.effort(joint, torque)?;
                    world.effort(joint, 0.0625)?;
                }
            }
            let at = control.capture()?;
            assert_eq!(journey.capture()?, at);
            if tick % 11 == 0 {
                let next = NativeArticulation::restore(&spec, tick, &at)?;
                assert_eq!(next.capture()?, at);
                assert_eq!(next.observe(), control.observe());
                journey = next;
            }
            control.advance()?;
            journey.advance()?;
            let frame = journey.observe();
            assert_eq!(frame.tick, tick + 1);
            assert_eq!(frame.links.map(|b| b.id), [37, 12]);
            assert_eq!(frame.joint_id, joint);
            assert_eq!(
                bincode::serialize(&frame).unwrap(),
                bincode::serialize(&control.observe()).unwrap()
            );
        }
        assert!(journey.observe().q.abs() > 0.001);
        Ok(())
    }

    #[test]
    fn cheap_receipt_check_rejects_rechecksummed_noncanonical_nested_records()
    -> Result<(), NativeError> {
        let spec = spec();
        let seal = |tick, magic: &[u8; 8], data: &[u8]| {
            let digest = checksum(&spec.fingerprint(), tick, magic, data);
            [magic.as_slice(), digest.as_slice(), data].concat()
        };
        for data in [
            vec![42],
            f64::NAN.to_le_bytes().to_vec(),
            (-0.0f64).to_le_bytes().to_vec(),
        ] {
            assert!(spec.verify_image(0, &seal(0, COLD, &data)).is_err());
        }
        let mut world = NativeArticulation::new(&spec)?;
        world.advance()?;
        let valid = world.capture()?;
        spec.verify_image(1, &valid)?;
        let mut data = spec.warm_payload(1, &valid)?.to_vec();
        data[0] ^= 1;
        assert!(spec.verify_image(1, &seal(1, WARM, &data)).is_err());
        data[0] ^= 1;
        data[8] ^= 1;
        assert!(spec.verify_image(1, &seal(1, WARM, &data)).is_err());
        data[360] = 1;
        let inner = Sha256::digest(&data[40..]);
        data[8..40].copy_from_slice(&inner);
        assert!(spec.verify_image(1, &seal(1, WARM, &data)).is_err());
        assert!(spec.verify_image(1, &seal(1, WARM, &data[..368])).is_err());
        Ok(())
    }

    #[test]
    fn image_admission_binds_clock_ids_native_recipe_and_original_payload()
    -> Result<(), NativeError> {
        let s = spec();
        let mut control = NativeArticulation::new(&s)?;
        assert!(control.effort(s.links[0].id, 1.).is_err());
        assert!(control.effort(s.joint_id, f64::NAN).is_err());
        let receipt = control.observe();
        let mut signed = receipt;
        signed.q = -receipt.q;
        assert_ne!(signed, receipt);
        let cold = control.capture()?;
        for tick in [0, 1] {
            if tick == 1 {
                control.advance()?;
            }
            let saved = control.capture()?;
            assert!(NativeArticulation::restore(&s, tick + 1, &saved).is_err());
            for i in [0, 8, saved.len() - 1] {
                let mut damaged = saved.clone();
                damaged[i] ^= 1;
                assert!(NativeArticulation::restore(&s, tick, &damaged).is_err());
            }
            let mut different = s;
            different.links[0].id += 100;
            assert!(NativeArticulation::restore(&different, tick, &saved).is_err());
            different = s;
            different.axis = [0., 1., 0.];
            assert!(NativeArticulation::restore(&different, tick, &saved).is_err());
            let mut trailing = saved;
            trailing.push(0);
            assert!(NativeArticulation::restore(&s, tick, &trailing).is_err());
        }
        assert!(s.warm_payload(0, &cold).is_err());
        let saved = control.capture()?;
        assert!(s.warm_payload(1, &saved)?.starts_with(b"GZART001"));
        let bad = ArticulationSpec {
            joint_id: s.links[0].id,
            ..s
        };
        assert!(bad.validate().is_err());
        let mut invalid = s;
        invalid.links[0].inertia_tensor[1] = -1.;
        assert!(invalid.validate().is_err());
        let mut invalid = s;
        invalid.axis = [0.; 3];
        assert!(invalid.validate().is_err());
        assert!(NativeArticulation::restore(&s, 1, &vec![0; MAX_ARTICULATION_IMAGE + 1]).is_err());
        Ok(())
    }
}
