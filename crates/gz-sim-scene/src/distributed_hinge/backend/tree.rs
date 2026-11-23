// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::distributed_hinge::floating_tree::tree_and_sim;
use crate::distributed_hinge::{
    FloatingTreeRecipe, FloatingTreeSpec, MotorTreePool, MotorTreeSpec, StoppedTreeSpec,
    TreeConstruction, TreePool,
};
use crate::distributed_hinge::{
    GraspPlacement, GraspTreePool, GraspTreeSpec, GroundedTreePool, GroundedTreeSpec,
    PayloadTreePool, PayloadTreeSpec, SplitGraspPool,
};
use std::collections::BTreeMap;
mod session;
pub use session::TreeSceneSession;

/// Open a stock distributed session using the selected world’s ordinary SDF topology.
pub fn load_distributed_articulation_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    load_distributed_articulation_world_with_step(
        xml, source, world, worker, directory, width, None,
    )
}

/// Open a managed articulation and verify any requested step before creating its barrier.
pub fn load_distributed_articulation_world_with_step(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
    step: Option<Duration>,
) -> Result<Simulation, LoadError> {
    crate::require(
        xml.len() <= 1024 * 1024,
        world,
        "articulation SDF exceeds 1 MiB",
    )?;
    let doc = roxmltree::Document::parse(xml).map_err(|_| LoadError::Unsupported {
        entity: world.into(),
        feature: "invalid articulation XML",
    })?;
    let selected = doc
        .root_element()
        .children()
        .find(|n| n.has_tag_name("world") && n.attribute("name") == Some(world))
        .ok_or_else(|| LoadError::MissingWorld(world.into()))?;
    let profile = profile_for_world(selected);
    if profile != ManagedProfile::Material {
        crate::require(
            step.is_none_or(|v| v == STEP),
            world,
            "this managed articulation requires a 1 ms step",
        )?;
    }
    match profile {
        ManagedProfile::Material => crate::deformable::managed_impact::open_persistent(
            xml, source, world, worker, directory, width, step,
        ),
        ManagedProfile::GroundedRobot => {
            super::grounded_robot::load_distributed_grounded_robot_world(
                xml, source, world, worker, directory, width,
            )
        }
        ManagedProfile::Tree => {
            load_distributed_tree_world(xml, source, world, worker, directory, width)
        }
        ManagedProfile::Hinge => {
            load_distributed_hinge_world(xml, source, world, worker, directory, width)
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ManagedProfile {
    Material,
    GroundedRobot,
    Tree,
    Hinge,
}

/// Classify only the selected world. The chosen loader validates full native eligibility.
fn profile_for_world(world: roxmltree::Node<'_, '_>) -> ManagedProfile {
    if world
        .descendants()
        .any(|n| n.has_tag_name("articulated_impact") && n.attribute("detection").is_some())
    {
        return ManagedProfile::Material;
    }
    let models = || world.children().filter(|n| n.has_tag_name("model"));
    fn links<'a, 'input>(
        m: roxmltree::Node<'a, 'input>,
    ) -> impl Iterator<Item = roxmltree::Node<'a, 'input>> {
        m.children().filter(|n| n.has_tag_name("link"))
    }
    fn has_collision_shape(link: roxmltree::Node<'_, '_>, shape: &str) -> bool {
        link.children()
            .filter(|n| n.has_tag_name("collision"))
            .any(|collision| {
                collision
                    .children()
                    .filter(|n| n.has_tag_name("geometry"))
                    .any(|geometry| geometry.children().any(|n| n.has_tag_name(shape)))
            })
    }
    let has_plane = models().any(|m| links(m).any(|link| has_collision_shape(link, "plane")));
    if has_plane
        && models()
            .any(|m| links(m).count() >= 3 && links(m).all(|link| has_collision_shape(link, "box")))
    {
        return ManagedProfile::GroundedRobot;
    }
    if models().any(|m| {
        m.children()
            .filter(|n| n.has_tag_name("joint"))
            .nth(1)
            .is_some()
    }) {
        ManagedProfile::Tree
    } else {
        ManagedProfile::Hinge
    }
}

/// The ordinary Bevy schedule publishes the restored clock, link poses and joint samples.
pub fn load_distributed_floating_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let FloatingTreeRecipe::Unlimited(spec) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "expected unlimited revolutes",
        });
    };
    let pool = TreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?;
    install(xml, source, world, initial, pool)
}

/// Open a tree with at least one authored finite stop and adopt its committed interval.
pub fn load_distributed_stopped_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let FloatingTreeRecipe::Stopped(spec) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "expected finite revolute stops",
        });
    };
    let pool = TreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?;
    install(xml, source, world, initial, pool)
}

/// Open the complete native articulation and an independent dynamic spherical payload.
pub fn load_distributed_payload_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let FloatingTreeRecipe::Payload(spec) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "expected a separate dynamic payload",
        });
    };
    install(
        xml,
        source,
        world,
        initial,
        PayloadTreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?,
    )
}
/// Open the authored articulated tree and its fixed external floor under downward gravity.
pub fn load_distributed_grounded_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let FloatingTreeRecipe::Grounded(spec) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "expected an external fixed plane",
        });
    };
    install(
        xml,
        source,
        world,
        initial,
        GroundedTreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?,
    )
}
/// Select native placement for a motorized gripper and its separately moving workpiece.
pub fn load_distributed_split_grasp_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
    placement: GraspPlacement,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let FloatingTreeRecipe::Payload(payload) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "selective placement requires a separately moving payload",
        });
    };
    let pool = SplitGraspPool::open_scene(
        worker,
        directory,
        width,
        GraspTreeSpec { payload },
        placement,
    )
    .map_err(physics_error)?;
    install(xml, source, world, initial, pool)
}
/// Open a motor-driven articulation acting on a separate dynamic payload.
pub fn load_distributed_grasp_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let FloatingTreeRecipe::Payload(payload) = recipe else {
        return Err(LoadError::Unsupported {
            entity: world.into(),
            feature: "grasp requires a separate movable payload",
        });
    };
    let pool = GraspTreePool::open_scene(worker, directory, width, GraspTreeSpec { payload })
        .map_err(physics_error)?;
    install(xml, source, world, initial, pool)
}
/// Start an explicit torque-bounded velocity-motor session on authored unlimited or stopped joints.
pub fn load_distributed_motor_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    let pool = MotorTreePool::open_scene(worker, directory, width, recipe.into_motor(world)?)
        .map_err(physics_error)?;
    install(xml, source, world, initial, pool)
}

/// Select the solver from the SDF, adopting an existing motor-enabled session when present.
pub fn load_distributed_tree_world(
    xml: &str,
    source: &str,
    world: &str,
    worker: &Path,
    directory: &Path,
    width: Option<u8>,
) -> Result<Simulation, LoadError> {
    let (recipe, initial) = tree_and_sim(xml, source, world)?;
    if directory
        .join("barrier.gzsplitgrasp")
        .try_exists()
        .map_err(|e| LoadError::Unsupported {
            entity: e.to_string(),
            feature: "cannot inspect grasp placement archive",
        })?
    {
        let FloatingTreeRecipe::Payload(payload) = recipe else {
            return Err(LoadError::Unsupported {
                entity: world.into(),
                feature: "saved selective placement needs matching payload",
            });
        };
        let pool = SplitGraspPool::resume(worker, directory).map_err(physics_error)?;
        if pool.spec().payload != payload || width.is_some_and(|w| w != pool.width()) {
            return Err(LoadError::Unsupported {
                entity: world.into(),
                feature: "saved selective grasp construction or width differs",
            });
        }
        return install(xml, source, world, initial, pool);
    }
    if let FloatingTreeRecipe::Payload(payload) = &recipe
        && directory
            .join("barrier.gzgrasptree")
            .try_exists()
            .map_err(|err| LoadError::Unsupported {
                entity: err.to_string(),
                feature: "cannot inspect gripper session",
            })?
    {
        let pool = GraspTreePool::open_scene(
            worker,
            directory,
            width,
            GraspTreeSpec {
                payload: payload.clone(),
            },
        )
        .map_err(physics_error)?;
        return install(xml, source, world, initial, pool);
    }
    if directory
        .join("barrier.gzmotorizedtree")
        .try_exists()
        .map_err(|err| LoadError::Unsupported {
            entity: err.to_string(),
            feature: "cannot inspect articulation session",
        })?
    {
        let pool = MotorTreePool::open_scene(worker, directory, width, recipe.into_motor(world)?)
            .map_err(physics_error)?;
        return install(xml, source, world, initial, pool);
    }
    match recipe {
        FloatingTreeRecipe::Unlimited(spec) => install(
            xml,
            source,
            world,
            initial,
            TreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?,
        ),
        FloatingTreeRecipe::Payload(spec) => install(
            xml,
            source,
            world,
            initial,
            PayloadTreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?,
        ),
        FloatingTreeRecipe::Grounded(spec) => install(
            xml,
            source,
            world,
            initial,
            GroundedTreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?,
        ),
        FloatingTreeRecipe::Stopped(spec) => install(
            xml,
            source,
            world,
            initial,
            TreePool::open_scene(worker, directory, width, spec).map_err(physics_error)?,
        ),
    }
}

fn install<P: TreeSceneSession>(
    xml: &str,
    source: &str,
    world: &str,
    initial: Simulation,
    pool: P,
) -> Result<Simulation, LoadError> {
    let clock = (pool.observation().tick, pool.epoch());
    let mut sim = at_session_clock(xml, source, world, initial, clock)?;
    let body = |public| {
        entity_by_id(sim.world(), public)
            .and_then(|e| sim.world().get::<BodyHandle>(e))
            .map(BodyHandle::id)
            .ok_or(PhysicsError::MissingBody)
    };
    let ground = pool
        .ground_id()
        .map(|public| -> Result<_, PhysicsError> {
            let id = body(public)?;
            Ok((
                id,
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(id)
                    .ok_or(PhysicsError::MissingBody)?,
            ))
        })
        .transpose()?;
    let payload = pool.payload().map(|p| body(p.id)).transpose()?;
    let mut bodies = BTreeMap::new();
    for (index, link) in pool.observation().links.iter().enumerate() {
        bodies.insert(body(link.id)?, index);
    }
    let mut joints = BTreeMap::new();
    for joint in &pool.topology().joints {
        let native = entity_by_id(sim.world(), joint.joint_id)
            .and_then(|e| sim.world().get::<JointHandle>(e))
            .ok_or(PhysicsError::MissingJoint)?
            .id();
        let index = pool
            .observation()
            .joints
            .binary_search_by_key(&joint.joint_id, |j| j.joint_id)
            .map_err(|_| PhysicsError::MissingJoint)?;
        joints.insert(native, (index, body(joint.child_id)?));
    }
    sim.world_mut()
        .insert_resource(PhysicsBackend::from(WorkerTreeBackend {
            pool,
            ground,
            payload,
            bodies,
            joints,
            identity: Arc::new(()),
            staged_clock: None,
            staged_efforts: Vec::new(),
            payload_force: [0.; 3],
        }));
    finish_install(sim, clock)
}

/// Bevy bindings to the committed state of a worker-owned floating revolute tree.
pub type DistributedFloatingTreeBackend = DistributedTreeBackend<FloatingTreeSpec>;
/// Bevy backend for a floating tree with finite revolute stops.
pub type DistributedStoppedTreeBackend = DistributedTreeBackend<StoppedTreeSpec>;
/// Bevy backend with durable native velocity motors.
pub type DistributedMotorTreeBackend = DistributedTreeBackend<MotorTreeSpec>;
/// Worker-owned articulated tree in contact with a fixed external floor.
pub type DistributedGroundedTreeBackend = DistributedTreeBackend<GroundedTreeSpec>;
/// Grasp with independently placed tree and payload across native workers.
pub type DistributedSplitGraspBackend = WorkerTreeBackend<SplitGraspPool>;
/// Worker-owned motorized tree contacting a freely moving sphere.
pub type DistributedGraspTreeBackend = DistributedTreeBackend<GraspTreeSpec>;
/// Passive worker-owned tree contacting a freely moving sphere.
pub type DistributedPayloadTreeBackend = DistributedTreeBackend<PayloadTreeSpec>;

/// A Bevy backend with one coupled tree archive.
pub type DistributedTreeBackend<S> = WorkerTreeBackend<TreePool<S>>;

pub struct WorkerTreeBackend<P> {
    pool: P,
    ground: Option<(BodyId, BodyState)>,
    payload: Option<BodyId>,
    bodies: BTreeMap<BodyId, usize>,
    joints: BTreeMap<JointId, (usize, BodyId)>,
    identity: Arc<()>,
    staged_clock: Option<(u64, u64)>,
    staged_efforts: Vec<(u64, f64)>,
    payload_force: [f64; 3],
}
impl WorkerTreeBackend<SplitGraspPool> {
    /// Move the payload owner, retaining the common worker for a joined physics world.
    pub fn transfer_payload(&mut self, destination: u8) -> Result<(), ArticulationError> {
        self.pool.transfer_payload(destination)
    }
}
impl<P: TreeSceneSession> WorkerTreeBackend<P> {
    pub fn pool(&self) -> &P {
        &self.pool
    }
    pub fn transfer(&mut self, destination: u8) -> Result<(), ArticulationError> {
        self.pool.transfer(destination)
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<(), ArticulationError> {
        self.pool.replace_worker(slot)
    }
    fn joint(
        &self,
        id: JointId,
    ) -> Option<&gz_physics_distributed::floating_tree::FloatingTreeJointState> {
        self.pool.observation().joints.get(self.joints.get(&id)?.0)
    }
    /// Add one transient command to the next advancing barrier. An exclusive session
    /// lease protects the expected clock from other operators.
    pub fn stage_step_effort_at(
        &mut self,
        epoch: u64,
        tick: u64,
        joint: JointId,
        torque: f64,
    ) -> Result<(), PhysicsError> {
        self.stage_step_efforts_at(epoch, tick, &[(joint, torque)])
    }
    /// Bind a vector of distinct joint torques to one durable physics interval. Invalid
    /// inputs leave the staging slot and every previously queued torque unchanged.
    pub fn stage_step_efforts_at(
        &mut self,
        epoch: u64,
        tick: u64,
        inputs: &[(JointId, f64)],
    ) -> Result<(), PhysicsError> {
        self.pool
            .check_interval(epoch, tick)
            .map_err(physics_error)?;
        if self.staged_clock.is_some() {
            return Err(unsupported(
                "an effort vector is already staged for this interval",
            ));
        }
        if inputs.len() > self.joints.len() {
            return Err(PhysicsError::InvalidValue);
        }
        let mut seen = 0u64;
        for &(id, torque) in inputs {
            let &(index, _) = self.joints.get(&id).ok_or(PhysicsError::MissingJoint)?;
            let bit = 1u64 << index;
            let sample = &self.pool.observation().joints[index];
            if seen & bit != 0 || !(sample.pending_effort.unwrap_or(0.) + torque).is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
            seen |= bit;
        }
        self.staged_efforts.clear();
        self.staged_efforts
            .extend(inputs.iter().map(|&(id, torque)| {
                (
                    self.pool.observation().joints[self.joints[&id].0].joint_id,
                    torque,
                )
            }));
        self.staged_clock = Some((epoch, tick));
        Ok(())
    }
}
impl<P: TreeSceneSession> DynamicBackend for WorkerTreeBackend<P> {
    fn engine(&self) -> &'static str {
        ENGINE
    }
    fn state(&self, id: BodyId) -> Option<BodyState> {
        if let Some((ground, state)) = self.ground
            && ground == id
        {
            return Some(state);
        }
        let state = if self.payload == Some(id) {
            self.pool.payload()?
        } else {
            self.pool.observation().links.get(*self.bodies.get(&id)?)?
        };
        Some(BodyState {
            position: state.position,
            velocity: state.velocity,
            orientation: state.orientation,
            angular_velocity: state.angular_velocity,
        })
    }
    fn joint_state(&self, id: JointId) -> Option<JointState> {
        let sample = self.joint(id)?;
        Some(JointState {
            position: sample.q,
            velocity: sample.qdot,
        })
    }
    fn is_dynamic(&self, id: BodyId) -> Option<bool> {
        if self.ground.is_some_and(|(g, _)| g == id) {
            Some(false)
        } else {
            (self.bodies.contains_key(&id) || self.payload == Some(id)).then_some(true)
        }
    }
    fn is_jointed(&self, id: BodyId) -> Option<bool> {
        self.is_dynamic(id).map(|_| self.bodies.contains_key(&id))
    }
    fn joint_child(&self, id: JointId) -> Option<BodyId> {
        self.joints.get(&id).map(|v| v.1)
    }
    fn validate_joint_effort(&self, joint: JointId, torque: f64) -> Result<(), PhysicsError> {
        if !self.joints.contains_key(&joint) {
            return Err(PhysicsError::MissingJoint);
        }
        if !torque.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }
    fn add_joint_effort(&mut self, joint: JointId, torque: f64) -> Result<(), PhysicsError> {
        self.validate_joint_effort(joint, torque)?;
        let public = self
            .joint(joint)
            .ok_or(PhysicsError::MissingJoint)?
            .joint_id;
        if let Some((_, staged)) = self.staged_efforts.iter().find(|(id, _)| *id == public) {
            let pending = self
                .joint(joint)
                .and_then(|j| j.pending_effort)
                .unwrap_or(0.);
            if !((pending + torque) + staged).is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
        }
        self.pool
            .queue_effort(public, torque)
            .map_err(physics_error)
    }
    fn validate_joint_velocity_motor(
        &self,
        id: JointId,
        motor: Option<gz_physics::JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        if !self.pool.motor_supported() {
            return Err(unsupported("joint velocity motors"));
        }
        let joint = self.joint(id).ok_or(PhysicsError::MissingJoint)?.joint_id;
        if motor.is_some() && self.pool.motor_locked(joint) {
            return Err(unsupported("motor on an exact revolute lock"));
        }
        if motor.is_some_and(|m| {
            !m.target_velocity.is_finite() || !m.max_force.is_finite() || m.max_force < 0.
        }) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }
    fn set_joint_velocity_motor(
        &mut self,
        id: JointId,
        motor: Option<gz_physics::JointVelocityMotor>,
    ) -> Result<(), PhysicsError> {
        self.validate_joint_velocity_motor(id, motor)?;
        let public = self.joint(id).ok_or(PhysicsError::MissingJoint)?.joint_id;
        self.pool.set_motor(public, motor)
    }
    fn add_world_wrench(
        &mut self,
        id: BodyId,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !self.pool.supports_payload_force() || Some(id) != self.payload || torque != [0.; 3] {
            return Err(unsupported(
                "grasp force must act at the independent payload center of mass",
            ));
        }
        let sum = std::array::from_fn(|i| self.payload_force[i] + force[i]);
        if !sum.iter().all(|n| n.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        self.payload_force = sum;
        Ok(())
    }
    fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Ok(());
        }
        if dt != STEP {
            return Err(unsupported("floating-tree timestep other than 1 ms"));
        }
        self.pool
            .advance_commands(self.staged_clock, &self.staged_efforts, self.payload_force)?;
        self.staged_clock = None;
        self.staged_efforts.clear();
        self.payload_force = [0.; 3];
        Ok(())
    }
    fn reset(&mut self) -> Result<(), PhysicsError> {
        self.pool.reset().map_err(physics_error)?;
        self.payload_force = [0.; 3];
        self.staged_clock = None;
        self.staged_efforts.clear();
        Ok(())
    }
    fixed_backend_methods!();
}

#[cfg(test)]
mod dispatch_tests {
    use super::*;
    #[test]
    fn neighboring_worlds_cannot_select_a_different_physics_profile() {
        let xml = r#"<sdf><world name="material"><plugin><articulated_impact detection="first_impact"/></plugin></world>
            <world name="hinge"><model><joint/></model></world><world name="tree"><model><joint/><joint/></model></world></sdf>"#;
        let doc = roxmltree::Document::parse(xml).unwrap();
        let profiles: Vec<_> = doc
            .root_element()
            .children()
            .filter(|n| n.has_tag_name("world"))
            .map(profile_for_world)
            .collect();
        assert_eq!(
            profiles,
            [
                ManagedProfile::Material,
                ManagedProfile::Hinge,
                ManagedProfile::Tree
            ]
        );
    }
}
