// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
mod contact;
mod partition;
mod timed;
use super::*;
use gz_physics::{BodyId, DynamicBackend, JointId, PhysicsBackend};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::{Simulation, UpdateInfo};
pub(super) use partition::as_body;
use std::time::Duration;
pub(super) use timed::TimedWorld;

const DT: Duration = Duration::from_millis(1);

pub(super) struct TreeWorld {
    sim: Simulation,
    links: Vec<(u64, BodyId)>,
    joints: Vec<(u64, JointId)>,
    recipe: [u8; 32],
    motorized: bool,
    ground: Option<BodyId>,
    payload: Option<(u64, BodyId)>,
}
impl TreeWorld {
    pub fn new(spec: &impl TreeConstruction) -> std::result::Result<Self, NativeError> {
        Self::start(spec, 0, None)
    }
    pub fn restore(spec: &impl TreeConstruction, snapshot: &Snapshot) -> Result<Self> {
        let tick = snapshot.state.tick;
        snapshot.check(spec, spec.digest(), tick)?;
        let world = Self::start(spec, tick, Some(&snapshot.image))?;
        if world.capture()? != *snapshot {
            return Err(invalid(
                "floating tree restore disagrees with accepted native state",
            ));
        }
        Ok(world)
    }
    fn start<S: TreeConstruction>(
        setup: &S,
        tick: u64,
        image: Option<&Image>,
    ) -> std::result::Result<Self, NativeError> {
        setup.validate_configuration()?;
        let spec = setup.base();
        let root = spec.root()?;
        let floor = setup.ground();
        let mut backend = MultiFidelityBackend::new([0., 0., floor.map_or(0., |g| g.gravity_z)])?;
        let mut ground = floor
            .filter(|g| g.link_id < spec.links[0].id)
            .map(|g| backend.spawn_static_plane(g.plane()))
            .transpose()?;
        let external = setup.payload();
        let spawn_payload = |backend: &mut MultiFidelityBackend,
                             link: &ArticulationLinkSpec|
         -> std::result::Result<_, gz_physics::PhysicsError> {
            let id = link.spawn_dynamic(backend)?;
            if !setup.payload_can_sleep() {
                backend.keep_body_awake(id)?;
            }
            Ok((link.id, id))
        };
        let mut payload = external
            .filter(|l| l.id < spec.links[0].id)
            .map(|l| spawn_payload(&mut backend, l))
            .transpose()?;
        let mut links = Vec::with_capacity(spec.links.len());
        for spec_link in &spec.links {
            let body = spec_link.spawn(&mut backend)?;
            backend.set_self_collision_group(body, Some(spec.model_id))?;
            links.push((spec_link.id, body));
        }
        let mut joints = Vec::with_capacity(spec.joints.len());
        for joint in &spec.joints {
            let p = links[spec.link_index(joint.parent_id)?].1;
            let c = links[spec.link_index(joint.child_id)?].1;
            let limits = setup.limits();
            let id = match limits.binary_search_by_key(&joint.joint_id, |s| s.joint_id) {
                Ok(i) => backend.add_limited_revolute(
                    Some(p),
                    c,
                    joint.anchor,
                    joint.axis,
                    [limits[i].lower, limits[i].upper],
                )?,
                Err(_) => backend.add_reduced_revolute(Some(p), c, joint.anchor, joint.axis)?,
            };
            joints.push((joint.joint_id, id));
        }
        if let Some(g) = floor.filter(|_| ground.is_none()) {
            ground = Some(backend.spawn_static_plane(g.plane())?);
        }
        if let Some(link) = external.filter(|_| payload.is_none()) {
            payload = Some(spawn_payload(&mut backend, link)?);
        }
        joints.sort_by_key(|(public, _)| *public);
        backend.switch_articulation(links[root].1, ArticulationTier::Dynamic, DT)?;
        if setup.exact_locks() {
            backend.enable_exact_revolute_locks(links[root].1)?;
        }
        match image {
            None => {}
            Some(
                Image::Cold(pending)
                | Image::MotorCold {
                    efforts: pending, ..
                }
                | Image::GroundCold {
                    efforts: pending, ..
                }
                | Image::GraspCold {
                    efforts: pending, ..
                }
                | Image::PayloadCold {
                    efforts: pending, ..
                },
            ) => {
                for ((_, joint), effort) in joints.iter().zip(pending) {
                    if let Some(bits) = effort {
                        backend.add_joint_effort(*joint, f64::from_bits(*bits))?;
                    }
                }
            }
            Some(Image::GraspWarm { native, .. }) => backend.restore_grasp_revolute_tree(native)?,
            Some(Image::Warm(bytes)) => backend.restore_floating_revolute_tree(bytes)?,
            Some(Image::PayloadWarm { native, .. }) => {
                backend.restore_payload_revolute_tree(native)?
            }
            Some(Image::GroundWarm { native, .. }) => {
                backend.restore_grounded_revolute_tree(native)?
            }
            Some(Image::MotorWarm { native, .. }) => backend.restore_motor_revolute_tree(native)?,
        }
        if let Some(Image::MotorCold { motors, .. } | Image::GraspCold { motors, .. }) = image {
            for ((_, id), input) in joints.iter().zip(motors) {
                if let Some(m) = input {
                    backend.set_joint_velocity_motor(*id, Some(m.native()))?;
                }
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
            links,
            joints,
            recipe: setup.digest(),
            ground,
            payload,
            motorized: S::MOTORIZED,
        })
    }
    pub fn tick(&self) -> u64 {
        self.sim.world().resource::<UpdateInfo>().iteration
    }
    pub fn effort(&mut self, public: u64, torque: f64) -> std::result::Result<(), NativeError> {
        let index = self
            .joints
            .binary_search_by_key(&public, |(id, _)| *id)
            .map_err(|_| NativeError::Invalid("unknown floating tree joint"))?;
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_joint_effort(self.joints[index].1, torque)?;
        Ok(())
    }
    pub fn set_motor(&mut self, joint: u64, setting: Option<MotorSetting>) -> Result<()> {
        if !self.motorized {
            return Err(invalid(
                "floating tree profile has no velocity motor control",
            ));
        }
        let i = self
            .joints
            .binary_search_by_key(&joint, |(id, _)| *id)
            .map_err(|_| invalid("unknown floating tree motor joint"))?;
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_motor(self.joints[i].1, setting.map(MotorSetting::native))
            .map_err(NativeError::from)?;
        Ok(())
    }
    pub fn apply_efforts(&mut self, efforts: &[(u64, f64)]) -> Result<()> {
        let backend = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned floating tree");
        validate_efforts(efforts, self.joints.len(), |public| {
            let i = self
                .joints
                .binary_search_by_key(&public, |(id, _)| *id)
                .ok()?;
            Some((i, backend.pending_joint_effort(self.joints[i].1)))
        })?;
        for &(id, e) in efforts {
            self.effort(id, e)?;
        }
        Ok(())
    }
    pub fn load_payload(&mut self, force: [f64; 3]) -> Result<()> {
        let (_, id) = self.payload.ok_or(invalid("missing external payload"))?;
        if !self.motorized || !force.iter().all(|x| x.is_finite()) {
            return Err(invalid("invalid grasp payload force"));
        }
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(id, force, [0.; 3])
            .map_err(NativeError::from)?;
        Ok(())
    }
    pub fn advance(&mut self) -> std::result::Result<(), NativeError> {
        self.sim.step_exact(1)?;
        Ok(())
    }
    fn observe_body(&self, id: u64, body: BodyId) -> ArticulationLinkState {
        let state = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .state(body)
            .expect("owned native tree link");
        ArticulationLinkState {
            id,
            position: state.position,
            orientation: state.orientation,
            velocity: state.velocity,
            angular_velocity: state.angular_velocity,
        }
    }
    pub(super) fn observation_at(&self, tick: u64) -> FloatingTreeObservation {
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let native = backend
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned native tree");
        let links = self
            .links
            .iter()
            .map(|&(id, body)| self.observe_body(id, body))
            .collect();
        let joints: Vec<_> = self
            .joints
            .iter()
            .map(|&(id, joint)| {
                let state = backend
                    .joint_state(joint)
                    .expect("owned floating tree joint");
                FloatingTreeJointState {
                    joint_id: id,
                    q: state.position,
                    qdot: state.velocity,
                    pending_effort: native.pending_joint_effort(joint),
                }
            })
            .collect();
        FloatingTreeObservation {
            tick,
            links,
            joints,
        }
    }
    pub fn capture(&self) -> std::result::Result<Snapshot, NativeError> {
        let tick = self.sim.committed_update_info()?.iteration;
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let native = backend
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned floating tree");
        let state = self.observation_at(tick);
        let joints = &state.joints;
        let held = || {
            joints
                .iter()
                .map(|s| s.pending_effort.map(f64::to_bits))
                .collect()
        };
        let read_motors = || {
            self.joints
                .iter()
                .map(|(_, id)| {
                    native
                        .motor_revolute_velocity(*id)
                        .map(|m| m.map(MotorSetting::from))
                })
                .collect::<std::result::Result<Vec<_>, _>>()
        };
        let image = if let Some((public, id)) = self.payload {
            let reactions = self
                .links
                .iter()
                .map(|(_, link)| native.payload_tree_reaction(*link, id))
                .collect::<std::result::Result<Vec<_>, _>>()?;
            let payload = PayloadReceipt {
                body: self.observe_body(public, id),
                reactions,
            };
            if self.motorized {
                let motors = read_motors()?;
                if tick == 0 {
                    Image::GraspCold {
                        efforts: held(),
                        payload,
                        motors,
                    }
                } else {
                    Image::GraspWarm {
                        native: native.checkpoint_grasp_revolute_tree()?,
                        payload,
                        motors,
                    }
                }
            } else if tick == 0 {
                Image::PayloadCold {
                    efforts: held(),
                    payload,
                }
            } else {
                Image::PayloadWarm {
                    native: native.checkpoint_payload_revolute_tree()?,
                    payload,
                }
            }
        } else if let Some(ground) = self.ground {
            let contacts = self
                .links
                .iter()
                .map(|(_, id)| native.ground_reaction(*id, ground))
                .collect::<std::result::Result<Vec<_>, _>>()?;
            let reactions = GroundReceipts(contacts);
            if tick == 0 {
                Image::GroundCold {
                    efforts: held(),
                    reactions,
                }
            } else {
                Image::GroundWarm {
                    native: native.checkpoint_grounded_revolute_tree()?,
                    reactions,
                }
            }
        } else if self.motorized {
            let motors = read_motors()?;
            if tick == 0 {
                Image::MotorCold {
                    efforts: held(),
                    motors,
                }
            } else {
                Image::MotorWarm {
                    native: native.checkpoint_motor_revolute_tree()?,
                    motors,
                }
            }
        } else if tick == 0 {
            Image::Cold(held())
        } else {
            Image::Warm(native.checkpoint_floating_revolute_tree()?)
        };
        let digest = Snapshot::seal(self.recipe, tick, &image);
        Ok(Snapshot {
            state,
            image,
            digest,
        })
    }
}
