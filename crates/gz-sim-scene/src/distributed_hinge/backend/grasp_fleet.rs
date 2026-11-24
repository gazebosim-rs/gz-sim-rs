// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Expose one globally fenced fleet of grippers through ordinary Bevy body/joint handles.
use super::*;
mod session;
use crate::distributed_hinge::grasp_fleet::fleet_and_sim;
use gz_physics::JointVelocityMotor;
use gz_physics_distributed::floating_tree::{
    FloatingTreeJointState, GraspFleetPlacement, GraspFleetPool, GraspFleetSpec,
};
pub use session::{GraspFleetLayout, ManagedGraspFleet};
use std::collections::{BTreeMap, BTreeSet};

pub fn load_distributed_grasp_fleet_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
    placement: impl Into<GraspFleetLayout>,
) -> Result<Simulation, LoadError> {
    let (assemblies, initial) = fleet_and_sim(xml, source, world)?;
    let pool = ManagedGraspFleet::open_scene(
        worker,
        directory,
        width,
        GraspFleetSpec { assemblies },
        placement.into(),
    )
    .map_err(physics_error)?;
    let clock = (pool.tick(), pool.epoch());
    let mut sim = at_session_clock(xml, source, world, initial, clock)?;
    let mut bodies = BTreeMap::new();
    let mut joints = BTreeMap::new();
    let body = |id| {
        entity_by_id(sim.world(), id)
            .and_then(|e| sim.world().get::<BodyHandle>(e))
            .map(BodyHandle::id)
            .ok_or(PhysicsError::MissingBody)
    };
    for assembly in &pool.spec().assemblies {
        let p = &assembly.payload;
        let model = p.tree.model_id;
        let state = pool.observation(model).ok_or(PhysicsError::MissingBody)?;
        for (idx, link) in state.links.iter().enumerate() {
            bodies.insert(body(link.id)?, FleetBody::Link { model, index: idx });
        }
        bodies.insert(
            body(p.payload.id)?,
            FleetBody::Payload {
                model,
                id: p.payload.id,
            },
        );
        for j in &p.tree.joints {
            let native = entity_by_id(sim.world(), j.joint_id)
                .and_then(|e| sim.world().get::<JointHandle>(e))
                .map(JointHandle::id)
                .ok_or(PhysicsError::MissingJoint)?;
            let index = state
                .joints
                .binary_search_by_key(&j.joint_id, |j| j.joint_id)
                .map_err(|_| PhysicsError::MissingJoint)?;
            let locked = p.exact_revolute_locks
                && p.limits
                    .iter()
                    .any(|l| l.joint_id == j.joint_id && l.lower == 0. && l.upper == 0.);
            joints.insert(
                native,
                FleetJoint {
                    model,
                    index,
                    child: body(j.child_id)?,
                    locked,
                },
            );
        }
    }
    sim.world_mut()
        .insert_resource(PhysicsBackend::from(DistributedGraspFleetBackend {
            pool,
            bodies,
            joints,
            identity: Arc::new(()),
            forces: BTreeMap::new(),
            efforts: Vec::new(),
            clock: None,
        }));
    finish_install(sim, clock)
}

enum FleetBody {
    Link { model: u64, index: usize },
    Payload { model: u64, id: u64 },
}
struct FleetJoint {
    model: u64,
    index: usize,
    child: BodyId,
    locked: bool,
}
/// Worker-managed fleet sharing one authoritative clock and one durable archive.
pub struct DistributedGraspFleetBackend {
    pool: ManagedGraspFleet,
    bodies: BTreeMap<BodyId, FleetBody>,
    joints: BTreeMap<JointId, FleetJoint>,
    identity: Arc<()>,
    forces: BTreeMap<u64, [f64; 3]>,
    efforts: Vec<(u64, f64)>,
    clock: Option<(u64, u64)>,
}
impl DistributedGraspFleetBackend {
    pub fn pool(&self) -> &ManagedGraspFleet {
        &self.pool
    }
    pub fn transfer(&mut self, model: u64, slot: u8) -> Result<(), ArticulationError> {
        self.pool.transfer(Some(model), slot)
    }
    pub fn transfer_coupled(&mut self, slot: u8) -> Result<(), ArticulationError> {
        self.pool.transfer(None, slot)
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<(), ArticulationError> {
        self.pool.replace_worker(slot)
    }
    fn joint(&self, id: JointId) -> Option<&FloatingTreeJointState> {
        let b = self.joints.get(&id)?;
        self.pool.observation(b.model)?.joints.get(b.index)
    }
    /// Apply distinct signed effort deltas together with the next guarded fleet interval.
    pub fn stage_step_efforts_at(
        &mut self,
        epoch: u64,
        tick: u64,
        efforts: &[(JointId, f64)],
    ) -> Result<(), PhysicsError> {
        self.pool
            .check_interval(epoch, tick)
            .map_err(physics_error)?;
        if self.clock.is_some() || efforts.len() > self.joints.len() {
            return Err(PhysicsError::InvalidValue);
        }
        let mut ids = BTreeSet::new();
        for &(id, effort) in efforts {
            let j = self.joint(id).ok_or(PhysicsError::MissingJoint)?;
            if !ids.insert(id)
                || !effort.is_finite()
                || !(j.pending_effort.unwrap_or(0.) + effort).is_finite()
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        self.efforts = efforts
            .iter()
            .map(|(id, e)| (self.joint(*id).unwrap().joint_id, *e))
            .collect();
        self.clock = Some((epoch, tick));
        Ok(())
    }
}
impl DynamicBackend for DistributedGraspFleetBackend {
    fn engine(&self) -> &'static str {
        ENGINE
    }
    fn state(&self, id: BodyId) -> Option<BodyState> {
        let row = match *self.bodies.get(&id)? {
            FleetBody::Link { model, index } => self.pool.observation(model)?.links.get(index)?,
            FleetBody::Payload { model, .. } => self.pool.payload_state(model)?,
        };
        Some(BodyState {
            position: row.position,
            orientation: row.orientation,
            velocity: row.velocity,
            angular_velocity: row.angular_velocity,
        })
    }
    fn joint_state(&self, id: JointId) -> Option<JointState> {
        let j = self.joint(id)?;
        Some(JointState {
            position: j.q,
            velocity: j.qdot,
        })
    }
    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        self.bodies.contains_key(&id).then_some(true)
    }
    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.bodies
            .get(&id)
            .map(|b| matches!(b, FleetBody::Link { .. }))
    }
    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.joints.get(&id).map(|j| j.child)
    }
    fn validate_joint_effort(&self, id: JointId, effort: f64) -> Result<(), PhysicsError> {
        let j = self.joint(id).ok_or(PhysicsError::MissingJoint)?;
        let staged = self
            .efforts
            .iter()
            .find(|(id, _)| *id == j.joint_id)
            .map_or(0., |(_, e)| *e);
        if !effort.is_finite() || !((j.pending_effort.unwrap_or(0.) + effort) + staged).is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }
    fn add_joint_effort(&mut self, id: JointId, effort: f64) -> Result<(), PhysicsError> {
        self.validate_joint_effort(id, effort)?;
        self.pool
            .queue_effort(self.joint(id).unwrap().joint_id, effort)
            .map_err(physics_error)
    }
    fn validate_joint_velocity_motor(
        &self,
        id: JointId,
        m: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        let j = self.joints.get(&id).ok_or(PhysicsError::MissingJoint)?;
        if j.locked && m.is_some() {
            return Err(unsupported("motor on an exact revolute lock"));
        }
        if m.is_some_and(|m| {
            !m.target_velocity.is_finite() || !m.max_force.is_finite() || m.max_force < 0.
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }
    fn set_joint_velocity_motor(
        &mut self,
        id: JointId,
        m: Option<JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.validate_joint_velocity_motor(id, m)?;
        self.pool
            .set_velocity_motor(self.joint(id).unwrap().joint_id, m)
            .map_err(physics_error)
    }
    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let FleetBody::Payload { id, .. } =
            self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?
        else {
            return Err(unsupported("force on a grasp-fleet articulated link"));
        };
        if torque != [0.; 3] {
            return Err(unsupported("grasp-fleet payload torque"));
        }
        let prev = self.forces.get(id).copied().unwrap_or([0.; 3]);
        let next: [f64; 3] = std::array::from_fn(|k| prev[k] + force[k]);
        if !next.iter().all(|n| n.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        self.forces.insert(*id, next);
        Ok(())
    }
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        if dt != STEP {
            return Err(unsupported("grasp-fleet timestep other than 1 ms"));
        }
        let (epoch, tick) = self.clock.unwrap_or((self.pool.epoch(), self.pool.tick()));
        let forces = self
            .forces
            .iter()
            .map(|(&id, &f)| (id, f))
            .collect::<Vec<_>>();
        self.pool
            .advance_with_inputs_at(epoch, tick, &self.efforts, &forces)
            .map_err(physics_error)?;
        self.clock = None;
        self.efforts.clear();
        self.forces.clear();
        Ok(())
    }
    fn reset(&mut self) -> Result<(), PhysicsError> {
        self.pool.reset().map_err(physics_error)?;
        self.clock = None;
        self.efforts.clear();
        self.forces.clear();
        Ok(())
    }
    fixed_backend_methods!();
}
