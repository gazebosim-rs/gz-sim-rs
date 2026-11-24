// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bind the ordinary Bevy robot, ground and joint identities to one process-owned physical graph.
use super::*;
use crate::distributed_hinge::grounded_robot::grounded_robot_and_sim;
use crate::joint_posture::PostureCommand;
use gz_physics_distributed::grounded_robot::{GroundedRobotPool, GroundedRobotSpec};
use gz_physics_multifidelity::{GroundReaction, NativeGroundPatch};
use std::collections::BTreeMap;

pub fn load_distributed_grounded_robot_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (spec, cold) = grounded_robot_and_sim(xml, source, world)?;
    let pool =
        GroundedRobotPool::open_scene(worker, directory, width, spec).map_err(physics_error)?;
    let clock = (pool.tick(), pool.epoch());
    let mut sim = at_session_clock(xml, source, world, cold, clock)?;
    let binding = RobotBindings::collect(&sim, pool.spec())?;
    sim.world_mut().insert_resource(binding.backend(pool));
    finish_install(sim, clock)
}

struct RobotBindings {
    links: BTreeMap<BodyId, usize>,
    handles: BTreeMap<u64, BodyId>,
    joints: BTreeMap<JointId, (usize, BodyId)>,
    floor: BodyId,
    floor_state: BodyState,
    root: BodyId,
}

impl RobotBindings {
    fn collect(sim: &Simulation, spec: &GroundedRobotSpec) -> Result<Self, PhysicsError> {
        let body = |id| {
            entity_by_id(sim.world(), id)
                .and_then(|e| sim.world().get::<BodyHandle>(e))
                .map(BodyHandle::id)
                .ok_or(PhysicsError::MissingBody)
        };
        let floor = body(spec.ground.link_id)?;
        let floor_state = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(floor)
            .ok_or(PhysicsError::MissingBody)?;
        let mut links = BTreeMap::new();
        let mut handles = BTreeMap::new();
        for (i, link) in spec.links.iter().enumerate() {
            let h = body(link.id)?;
            links.insert(h, i);
            handles.insert(link.id, h);
        }
        let mut ordered = spec.joints.iter().collect::<Vec<_>>();
        ordered.sort_by_key(|j| j.joint_id);
        let mut joints = BTreeMap::new();
        for (row, joint) in ordered.into_iter().enumerate() {
            let h = entity_by_id(sim.world(), joint.joint_id)
                .and_then(|e| sim.world().get::<JointHandle>(e))
                .map(JointHandle::id)
                .ok_or(PhysicsError::MissingJoint)?;
            joints.insert(h, (row, body(joint.child_id)?));
        }
        let root = spec
            .links
            .iter()
            .find(|l| !spec.joints.iter().any(|j| j.child_id == l.id))
            .ok_or(PhysicsError::MissingBody)?;
        Ok(Self {
            links,
            handles,
            joints,
            floor,
            floor_state,
            root: body(root.id)?,
        })
    }

    fn backend(self, pool: GroundedRobotPool) -> PhysicsBackend {
        PhysicsBackend::from(DistributedGroundedRobot {
            pool,
            links: self.links,
            handles: self.handles,
            joints: self.joints,
            floor: self.floor,
            floor_state: self.floor_state,
            identity: Arc::new(()),
            pending_forces: BTreeMap::new(),
            posture: Vec::new(),
        })
    }
}

mod ordinary_return;
pub(crate) use ordinary_return::remember_robot_program;
pub use ordinary_return::{
    LocalGroundedRobotSession, checkpoint_returned_grounded_robot,
    load_returned_grounded_robot_world, rejoin_grounded_robot_workers,
    return_grounded_robot_to_local,
};

fn validate_robot_identities(sim: &Simulation, cold: &Simulation) -> Result<(), LoadError> {
    for original in cold.world().iter_entities() {
        let Some(public) = original.get::<gz_sim_core::EntityId>() else {
            continue;
        };
        let Some(name) = original.get::<crate::Name>() else {
            continue;
        };
        let entity =
            entity_by_id(sim.world(), public.get()).ok_or(PhysicsError::ForeignInitialState)?;
        let live = sim.world().entity(entity);
        if live.get::<crate::Name>().map(|n| &n.0) != Some(&name.0)
            || live.get::<BodyHandle>().map(BodyHandle::id)
                != original.get::<BodyHandle>().map(BodyHandle::id)
            || live.get::<JointHandle>().map(JointHandle::id)
                != original.get::<JointHandle>().map(JointHandle::id)
        {
            return Err(PhysicsError::ForeignInitialState.into());
        }
    }
    Ok(())
}

/// Transfer a committed local robot to a fresh worker session without replacing its Bevy
/// entities, control systems, simulation clock or authored reset poses. The SDF binds the
/// stable model/link/joint IDs and the native checkpoint validates the complete construction.
pub fn handoff_grounded_robot_to_workers(
    sim: &mut Simulation,
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: u8,
) -> Result<(), LoadError> {
    sim.validate_steps(0)?;
    let clock = sim.committed_update_info()?;
    let (spec, cold) = grounded_robot_and_sim(xml, source, world)?;
    if sim.step_duration() != STEP
        || clock.iteration == 0
        || clock.sim_time != Duration::from_millis(clock.iteration)
        || !ordinary_return::matches_robot_program(sim, xml, world)
        || sim.world().contains_resource::<LocalGroundedRobotSession>()
        || !initial::matches_physics_owner(sim.world())
    {
        return Err(PhysicsError::InvalidValue.into());
    }
    validate_robot_identities(sim, &cold)?;
    let binding = RobotBindings::collect(sim, &spec)?;
    let be = sim.world().resource::<PhysicsBackend>();
    let native = be
        .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
        .ok_or(PhysicsError::ForeignInitialState)?;
    let image = native.checkpoint_grounded_robot_tree(binding.root, binding.floor)?;
    if sim
        .world_mut()
        .resource_mut::<initial::InitialScene>()
        .rebindable_physics()
        .is_none()
    {
        return Err(PhysicsError::ForeignInitialState.into());
    }
    let result = GroundedRobotPool::create_from_native(
        worker,
        directory,
        width,
        spec,
        clock.epoch,
        clock.iteration,
        &image,
    );
    let managed = binding.backend(ownership_result(sim, result)?);
    *sim.world_mut()
        .resource_mut::<initial::InitialScene>()
        .rebindable_physics()
        .expect("validated unique reset blueprint") = managed.initial_state();
    sim.world_mut().insert_resource(managed);
    Ok(())
}

// Once an atomic publication starts, a disk failure can leave either authority
// named on disk. The current simulation must reopen the accepted owner before advancing.
fn ownership_result<T>(
    sim: &Simulation,
    result: Result<T, ArticulationError>,
) -> Result<T, LoadError> {
    result.map_err(|error| {
        if matches!(error, ArticulationError::Publication(_)) {
            sim.world()
                .resource::<gz_sim_core::StepFault>()
                .report(std::io::Error::other(format!(
                    "uncertain robot publication, reopen the session: {error}"
                )));
        }
        physics_error(error).into()
    })
}

pub struct DistributedGroundedRobot {
    pool: GroundedRobotPool,
    links: BTreeMap<BodyId, usize>,
    handles: BTreeMap<u64, BodyId>,
    joints: BTreeMap<JointId, (usize, BodyId)>,
    floor: BodyId,
    floor_state: BodyState,
    identity: Arc<()>,
    pending_forces: BTreeMap<u64, [f64; 3]>,
    posture: Vec<(u64, f64)>,
}
impl DistributedGroundedRobot {
    pub fn pool(&self) -> &GroundedRobotPool {
        &self.pool
    }
    pub fn transfer(&mut self, target: u8) -> Result<(), ArticulationError> {
        self.pool.transfer(target)
    }
    pub fn ground_reaction(
        &self,
        link: BodyId,
        ground: BodyId,
    ) -> Result<GroundReaction, PhysicsError> {
        if ground != self.floor {
            return Err(PhysicsError::MissingBody);
        }
        self.links
            .get(&link)
            .and_then(|&i| self.pool.ground_reactions().get(i))
            .copied()
            .ok_or(PhysicsError::MissingBody)
    }
    pub fn fixed_ground_patches(
        &self,
        ground: BodyId,
    ) -> Result<Vec<NativeGroundPatch>, PhysicsError> {
        if ground != self.floor {
            return Err(PhysicsError::MissingBody);
        }
        self.pool
            .ground_patches()
            .iter()
            .map(|p| {
                Ok(NativeGroundPatch {
                    body: *self
                        .handles
                        .get(&p.link_id)
                        .ok_or(PhysicsError::MissingBody)?,
                    normal_on_body: p.normal_on_body,
                    points: p.points.clone(),
                })
            })
            .collect()
    }
    /// Stage one feedback output per actuator for the next Bevy step. Queued external torques
    /// already acknowledged by a worker remain in its durable cut and sum with these commands.
    pub(crate) fn stage_posture(
        &mut self,
        commands: &[PostureCommand],
    ) -> Result<(), PhysicsError> {
        if !self.posture.is_empty() {
            return Err(PhysicsError::InvalidValue);
        }
        let mut seen = std::collections::BTreeSet::new();
        for c in commands {
            self.validate_joint_effort(c.joint, c.effort_nm)?;
            if !seen.insert(c.joint) {
                return Err(PhysicsError::InvalidValue);
            }
        }
        self.posture.extend(commands.iter().map(|c| {
            let i = self.joints[&c.joint].0;
            (self.pool.observation().joints[i].joint_id, c.effort_nm)
        }));
        Ok(())
    }
}
impl DynamicBackend for DistributedGroundedRobot {
    fn engine(&self) -> &'static str {
        "distributed-grounded-robot"
    }
    fn state(&self, id: BodyId) -> Option<BodyState> {
        if id == self.floor {
            return Some(self.floor_state);
        }
        let r = self.pool.observation().links.get(*self.links.get(&id)?)?;
        Some(BodyState {
            position: r.position,
            orientation: r.orientation,
            velocity: r.velocity,
            angular_velocity: r.angular_velocity,
        })
    }
    fn joint_state(&self, id: JointId) -> Option<JointState> {
        let r = self
            .pool
            .observation()
            .joints
            .get(self.joints.get(&id)?.0)?;
        Some(JointState {
            position: r.q,
            velocity: r.qdot,
        })
    }
    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        if id == self.floor {
            Some(false)
        } else {
            self.links.contains_key(&id).then_some(true)
        }
    }
    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.is_dynamic(id)
    }
    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.joints.get(&id).map(|(_, child)| *child)
    }
    fn validate_joint_effort(&self, id: JointId, effort: f64) -> Result<(), PhysicsError> {
        let i = self.joints.get(&id).ok_or(PhysicsError::MissingJoint)?.0;
        let j = &self.pool.observation().joints[i];
        let staged = self
            .posture
            .iter()
            .find_map(|(id, e)| (*id == j.joint_id).then_some(*e))
            .unwrap_or_default();
        if !(j.pending_effort.unwrap_or_default() + effort + staged).is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }
    fn add_joint_effort(&mut self, id: JointId, effort: f64) -> Result<(), PhysicsError> {
        self.validate_joint_effort(id, effort)?;
        self.pool
            .queue_effort(
                self.pool.observation().joints[self.joints[&id].0].joint_id,
                effort,
            )
            .map_err(physics_error)
    }
    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        let i = *self.links.get(&id).ok_or(PhysicsError::MissingBody)?;
        if torque != [0.; 3] {
            return Err(unsupported("grounded robot external free torque"));
        }
        let public = self.pool.observation().links[i].id;
        let before = self
            .pending_forces
            .get(&public)
            .copied()
            .unwrap_or_default();
        let next = std::array::from_fn(|i| before[i] + force[i]);
        if !next.iter().all(|n| n.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        self.pending_forces.insert(public, next);
        Ok(())
    }
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        if dt != STEP {
            return Err(unsupported("grounded robot timestep other than 1 ms"));
        }
        let forces = self
            .pending_forces
            .iter()
            .map(|(&id, &force)| (id, force))
            .collect::<Vec<_>>();
        self.pool
            .advance_with_inputs_at(self.pool.epoch(), self.pool.tick(), &self.posture, &forces)
            .map_err(physics_error)?;
        self.posture.clear();
        self.pending_forces.clear();
        Ok(())
    }
    fn reset(&mut self) -> Result<(), PhysicsError> {
        self.pool.reset().map_err(physics_error)?;
        self.posture.clear();
        self.pending_forces.clear();
        Ok(())
    }
    fixed_backend_methods!();
}
