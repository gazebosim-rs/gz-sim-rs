// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[cfg(feature = "partition-checkpoint")]
mod articulation_checkpoint;
#[cfg(feature = "partition-checkpoint")]
pub use articulation_checkpoint::{
    CLEAR_GRASP_MEMBER_IMAGE_LIMIT, FLOATING_TREE_IMAGE_LIMIT, GRASP_FLEET_IMAGE_LIMIT,
    GRASP_TREE_IMAGE_LIMIT, GROUNDED_REVOLUTE_LIMIT, GROUNDED_ROBOT_IMAGE_LIMIT,
    GROUNDED_TREE_IMAGE_LIMIT, GroundReaction, MAX_FLOATING_TREE_LINKS, MAX_GROUNDED_ROBOT_LINKS,
    MAX_GROUNDED_TREE_LINKS, MAX_PAYLOAD_TREE_LINKS, MOTOR_TREE_IMAGE_LIMIT, MovingReaction,
    PAYLOAD_TREE_IMAGE_LIMIT, STOPPED_TREE_IMAGE_LIMIT, WARM_GRASP_MEMBER_IMAGE_LIMIT,
};
#[cfg(feature = "landing-audit")]
mod landing_audit;
#[cfg(feature = "landing-audit")]
pub use landing_audit::{
    LandingCachedPoint, LandingFork, LandingRestartState, LandingSolverPoint, LandingTrial,
    RestartPairHistory,
};
mod exchange_inputs;
pub use exchange_inputs::ExchangeInputs;
mod coated_ankle;
pub use coated_ankle::{CoatedAnkle, CoatedAnkleStep, ShankContact};
mod exact_lock;
mod gravity;
pub use gravity::{JointGravityLoad, LockedSubtreeInertia};
mod parked_robot;
pub(crate) mod parked_speed;
mod robot;
mod screw;
mod universal;
mod universal_motor;
mod universal_projection;
mod welded;
pub use robot::{BallJointState, RobotJoint};
pub use universal::UniversalJointState;
mod spring;
pub use spring::RobotSpring;
mod welded_projection;
pub use welded::WeldedAssemblyState;
pub(super) use welded_projection::WeldedProjection;
mod motor;
mod position_servo;
mod reduced;
mod reduced_contact;
pub use reduced_contact::PointSpanLoad;
pub use reduced_contact::{
    ArticulatedContactSample, ArticulatedContactSite, ArticulatedContactTransaction,
    ArticulatedImpulseReport, ArticulatedNativeInterval, ArticulatedSweep, BoundPointInterval,
    BoundTransferReceipt, CornerImpactReceipt, DeckEdgeBearingReceipt, EdgeImpactReceipt,
    FaceBearingReceipt, FacePressureReceipt, FiniteEndpointCertificate, FiniteImpactReceipt,
    FractionalBearingSpan, FractionalPointEvent, FractionalPointReceipt,
    FrictionlessRecontactReceipt, InternalPointImpact, MAX_ARTICULATED_CONTACT_SITES,
    MixedEdgeImpactReceipt, OpeningEdgeImpactReceipt, PlanarImpactReceipt, RockingLimitReceipt,
    SharedFaceReceipt, SharedSupportLoad, SharedSupportRequest, SlidingEdgeImpactReceipt,
    SlidingPointReceipt, SlidingVertexReceipt, SoleTransferReceipt, StickingPointReceipt,
    SustainedEdgeReceipt, TwoBearingReceipt,
};
pub(crate) use reduced_contact::{BearingModes, BearingResult};
pub(crate) use reduced_contact::{CornerFlight, PreparedFractional, ProjectionGeometry};
pub(crate) use reduced_contact::{MaterialEndpoint, PointFollowModes};
mod servo;
mod speed;
mod speed_reference;

#[derive(Clone, Copy)]
pub(crate) enum Solver {
    Impulse,
    Reduced,
}
#[derive(Clone, Copy)]
enum AngularConstraint {
    Revolute(Option<[f64; 2]>),
    Weld,
    Prismatic(Option<[f64; 2]>),
    Screw {
        lead: f64,
        effort_limit: f64,
        limits: Option<[f64; 2]>,
    },
    Ball(Rotation),
    Universal(Rotation, universal::UniversalSettings),
}

#[derive(Clone, Copy)]
enum JointMotion {
    Angular,
    Prismatic,
    Screw { lead: f64, effort_limit: f64 },
    Ball,
    Universal(universal::UniversalSettings),
}

#[derive(Clone, Copy)]
enum Handle {
    Impulse(ImpulseJointHandle),
    Reduced(MultibodyJointHandle),
}
// Native private joint metadata survives prescribed suspension of the constraint row.
pub(super) const BOUNDED_REVOLUTE: u128 = 1;
pub(super) const EXACT_REVOLUTE_LOCK: u128 = 2;
#[derive(Clone, Copy)]
struct Construction {
    parent: Option<BodyId>,
    child: BodyId,
    frame1: Pose,
    frame2: Pose,
    solver: Solver,
    limits: Option<[f64; 2]>,
    exact_lock: bool,
    motion: JointMotion,
    robot: bool,
    spring: Option<RobotSpring>,
    additional_pgs_iterations: u8,
}
impl Construction {
    fn admits_effort(self, effort: f64) -> bool {
        let limit = match self.motion {
            JointMotion::Screw { effort_limit, .. } => effort_limit,
            _ => f64::INFINITY,
        };
        effort.is_finite() && effort.abs() <= limit
    }

    fn authored_weld(self) -> bool {
        self.exact_lock && self.limits.is_none()
    }

    fn data(self) -> GenericJoint {
        if self.exact_lock {
            return GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES)
                .local_frame1(self.frame1)
                .local_frame2(self.frame2)
                .contacts_enabled(false)
                .user_data(EXACT_REVOLUTE_LOCK)
                .build();
        }
        let mask = match self.motion {
            JointMotion::Universal(_) => return self.universal_anchor(),
            JointMotion::Angular => JointAxesMask::LOCKED_REVOLUTE_AXES,
            JointMotion::Prismatic => JointAxesMask::LOCKED_PRISMATIC_AXES,
            JointMotion::Screw { .. } => JointAxesMask::LOCKED_REVOLUTE_AXES - JointAxesMask::LIN_X,
            JointMotion::Ball => JointAxesMask::LOCKED_SPHERICAL_AXES,
        };
        let joint = GenericJointBuilder::new(mask)
            .local_frame1(self.frame1)
            .local_frame2(self.frame2)
            .contacts_enabled(false);
        if let Some(limits) = self.limits {
            if let JointMotion::Screw { lead, .. } = self.motion {
                // The axial travel remains continuous across any number of turns.
                if lead == 0. {
                    return joint.build();
                }
                let [a, b] = limits.map(|q| (q * lead).clamp(-f64::MAX, f64::MAX));
                return joint.limits(JointAxis::LinX, [a.min(b), a.max(b)]).build();
            }
            let joint = joint.limits(self.motor_axis(), limits);
            if matches!(self.motion, JointMotion::Angular) {
                joint.user_data(BOUNDED_REVOLUTE).build()
            } else {
                joint.build()
            }
        } else {
            joint.build()
        }
    }
    fn insert(
        self,
        world: &mut PhysicsWorld,
        parent: RigidBodyHandle,
        child: RigidBodyHandle,
    ) -> Handle {
        if matches!(self.motion, JointMotion::Ball | JointMotion::Universal(_)) {
            // An abrupt stop transfers load between pivot rows in the same step.
            // Resolve those transient reactions more finely on the connected island.
            world.bodies[child].set_additional_solver_iterations(4);
        }
        match self.solver {
            Solver::Impulse => {
                Handle::Impulse(world.insert_impulse_joint(parent, child, self.data()))
            }
            Solver::Reduced => {
                let handle = world
                    .insert_multibody_joint(parent, child, self.data())
                    .expect("preflighted articulation topology");
                Handle::Reduced(handle)
            }
        }
    }
}
#[derive(Clone)]
struct WorldRoot {
    handle: RigidBodyHandle,
    joint: JointId,
}
#[derive(Clone)]
struct Joint {
    handle: Handle,
    initial: Construction,
    spring: Option<ImpulseJointHandle>,
    universal_angles: [f64; 2],
    universal_stops: [Option<ImpulseJointHandle>; 2],
    previous_angle: f64,
    position: f64,
}

#[derive(Clone)]
pub(super) struct Joints {
    gravity_tree: std::sync::OnceLock<gravity::GravityTree>,
    records: BTreeMap<JointId, Joint>,
    efforts: BTreeMap<JointId, f64>,
    angular_efforts: BTreeMap<JointId, Vector>,
    universal_springs: BTreeMap<JointId, universal::SpringRows>,
    projected_universal: BTreeMap<JointId, ImpulseJointHandle>,
    universal_motors: BTreeMap<(JointId, usize), ImpulseJointHandle>,
    speed_caps: BTreeMap<JointId, speed::SpeedCaps>,
    wide_universal: std::collections::BTreeSet<JointId>,
    screws: BTreeMap<JointId, ImpulseJointHandle>,
    anchor: Option<RigidBodyHandle>,
    world_roots: BTreeMap<BodyId, WorldRoot>,
    next: u64,
    open: bool,
    pub(super) authored_welds: bool,
}
impl Default for Joints {
    fn default() -> Self {
        Self {
            gravity_tree: Default::default(),
            records: BTreeMap::new(),
            efforts: BTreeMap::new(),
            angular_efforts: BTreeMap::new(),
            universal_springs: BTreeMap::new(),
            projected_universal: BTreeMap::new(),
            universal_motors: BTreeMap::new(),
            speed_caps: BTreeMap::new(),
            wide_universal: Default::default(),
            screws: BTreeMap::new(),
            anchor: None,
            world_roots: BTreeMap::new(),
            next: 0,
            open: true,
            authored_welds: false,
        }
    }
}
#[derive(Clone)]
pub(super) struct InitialJoints {
    speed_caps: BTreeMap<JointId, [Option<f64>; 2]>,
    universal_springs: BTreeMap<JointId, [Option<RobotSpring>; 2]>,
    records: BTreeMap<JointId, Construction>,
    next: u64,
}
impl Joints {
    pub(super) fn validate_effort(
        &self,
        id: JointId,
        torque: f64,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Result<(), PhysicsError> {
        let joint = self.records.get(&id).ok_or(PhysicsError::MissingJoint)?;
        if matches!(joint.initial.motion, JointMotion::Universal(_)) {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "scalar effort on a universal joint",
            });
        }
        if matches!(joint.initial.motion, JointMotion::Ball) {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "scalar effort on a ball joint",
            });
        }
        if joint.initial.authored_weld() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "joint effort on a fixed joint",
            });
        }
        // Holding a world couple does not qualify effort on a moving joint axis.
        if joint.initial.parent.is_some_and(|parent| {
            bodies[&parent].1.mass.is_some()
                && !joint.initial.robot
                && !(matches!(joint.handle, Handle::Reduced(_))
                    && bodies[&parent].1.articulation.is_some()
                    && bodies[&joint.initial.child].1.articulation.is_some())
        }) {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "moving-parent joint effort commands",
            });
        }
        if !joint.initial.admits_effort(torque) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(())
    }

    pub(super) fn add_effort(
        &mut self,
        id: JointId,
        torque: f64,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Result<(), PhysicsError> {
        self.validate_effort(id, torque, bodies)?;
        let next = self.efforts.get(&id).copied().unwrap_or(0.0) + torque;
        if !self.records[&id].initial.admits_effort(next) {
            return Err(PhysicsError::InvalidValue);
        }
        self.efforts.insert(id, next);
        Ok(())
    }

    pub(super) fn prepare_efforts(
        &self,
        world: &PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Result<BTreeMap<BodyId, (Vector, Vector)>, PhysicsError> {
        let mut loads = BTreeMap::<BodyId, (Vector, Vector)>::new();
        for (&id, &effort) in &self.efforts {
            let joint = self.records[&id].initial;
            let parent = joint.parent.map(|id| &world.bodies[bodies[&id].0]);
            let axis = parent.map_or(Rotation::IDENTITY, |p| *p.rotation())
                * joint.frame1.rotation
                * Vector::X;
            let applied = axis * effort;
            let point = *world.bodies[bodies[&joint.child].0].position() * joint.frame2.translation;
            let dynamic_parent = joint.parent.filter(|id| bodies[id].1.mass.is_some());
            for (id, value) in
                std::iter::once((joint.child, applied)).chain(dynamic_parent.map(|p| (p, -applied)))
            {
                let (force, torque) = loads.entry(id).or_default();
                if matches!(joint.motion, JointMotion::Prismatic) {
                    *force += value;
                    *torque += (point - world.bodies[bodies[&id].0].center_of_mass()).cross(value);
                } else {
                    *torque += value;
                }
            }
        }
        for (&id, &effort) in &self.angular_efforts {
            let c = self.records[&id].initial;
            let (parent, child) = c.world_joint_frames(world, bodies);
            let torque = if matches!(c.motion, JointMotion::Ball) {
                parent * effort
            } else {
                (parent * Vector::X) * effort.x + (child * Vector::Y) * effort.y
            };
            loads.entry(c.child).or_default().1 += torque;
            if let Some(p) = c.parent.filter(|p| bodies[p].1.mass.is_some()) {
                loads.entry(p).or_default().1 -= torque;
            }
        }
        for (&id, &(force, torque)) in &loads {
            let body = &world.bodies[bodies[&id].0];
            if !(body.user_force() + force).is_finite()
                || !(body.user_torque() + torque).is_finite()
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        Ok(loads)
    }

    pub(super) fn has_effort_for(&self, members: &[(BodyId, RigidBodyHandle)]) -> bool {
        self.efforts
            .keys()
            .chain(self.angular_efforts.keys())
            .any(|id| {
                members
                    .iter()
                    .any(|&(body, _)| self.records[id].initial.child == body)
            })
    }

    pub(super) fn consume_efforts(&mut self) {
        self.efforts.clear();
        self.angular_efforts.clear();
    }

    pub(super) fn update(&mut self, world: &PhysicsWorld, bodies: &BTreeMap<BodyId, BodyEntry>) {
        for joint in self.records.values_mut() {
            if matches!(joint.initial.motion, JointMotion::Universal(_)) {
                let (parent, child) = joint.initial.world_joint_frames(world, bodies);
                joint.update_universal_angles(parent.inverse() * child);
                continue;
            }
            if !matches!(
                joint.initial.motion,
                JointMotion::Angular | JointMotion::Screw { .. }
            ) || joint.initial.exact_lock
            {
                continue;
            }
            let current = angle(world, joint.handle, joint.initial, bodies);
            if matches!(joint.handle, Handle::Reduced(_)) {
                joint.position = current;
                continue;
            }
            // Each body's full-step rotation is capped at pi/4, hence relative
            // rotation stays below pi and nearest-angle unwrapping is unambiguous.
            let delta = (current - joint.previous_angle + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            joint.position += delta;
            joint.previous_angle = current;
        }
    }
    pub(super) fn construction_open(&self) -> bool {
        self.open
    }
    pub(super) fn close(&mut self) {
        self.open = false;
    }
    pub(super) fn initial(&self) -> InitialJoints {
        InitialJoints {
            speed_caps: self
                .speed_caps
                .iter()
                .map(|(&id, caps)| {
                    (
                        id,
                        caps.each_ref()
                            .map(|cap| cap.as_ref().map(speed::AxisCap::limit)),
                    )
                })
                .collect(),
            universal_springs: self
                .universal_springs
                .iter()
                .map(|(&id, rows)| (id, rows.map(|row| row.map(|(law, _)| law))))
                .collect(),
            records: self
                .records
                .iter()
                .map(|(&id, j)| (id, j.initial))
                .collect(),
            next: self.next,
        }
    }
    pub(super) fn restore(
        initial: &InitialJoints,
        world: &mut PhysicsWorld,
        bodies: &BTreeMap<BodyId, BodyEntry>,
    ) -> Self {
        let anchor = initial
            .records
            .values()
            .any(|joint| joint.parent.is_none() && matches!(joint.solver, Solver::Impulse))
            .then(|| world.insert_body(RigidBodyBuilder::fixed()));
        let mut world_roots = BTreeMap::new();
        let mut screws = BTreeMap::new();
        let mut universal_springs = BTreeMap::new();
        let mut speed_caps = BTreeMap::new();
        let records = initial
            .records
            .iter()
            .map(|(&id, &construction)| {
                // Removal retires connected records; snapshots only contain registered endpoints.
                let child = bodies
                    .get(&construction.child)
                    .expect("joint construction retains its body")
                    .0;
                *world.bodies[child].activation_mut() = RigidBodyActivation::cannot_sleep();
                let parent = construction.parent.map_or_else(
                    || {
                        if matches!(construction.solver, Solver::Reduced) {
                            let root = world.insert_body(RigidBodyBuilder::fixed());
                            world_roots.insert(
                                construction.child,
                                WorldRoot {
                                    handle: root,
                                    joint: id,
                                },
                            );
                            root
                        } else {
                            anchor.expect("world joint owns an anchor")
                        }
                    },
                    |parent| bodies[&parent].0,
                );
                if world.bodies[parent].is_dynamic() {
                    *world.bodies[parent].activation_mut() = RigidBodyActivation::cannot_sleep();
                }
                let handle = construction.insert(world, parent, child);
                if construction.additional_pgs_iterations != 0 {
                    world.bodies[child].set_additional_pgs_iterations(usize::from(
                        construction.additional_pgs_iterations,
                    ));
                }
                if let Some(row) = construction.insert_screw_row(world, parent, child) {
                    screws.insert(id, row);
                }
                // Preserve the native insertion order from ordinary construction.
                // Constraint coloring and its finite iteration residual depend on it.
                let universal_stops = construction.insert_universal_stops(world, parent, child);
                let spring = construction.insert_spring(world, parent, child);
                if let Some(&laws) = initial.universal_springs.get(&id) {
                    universal_springs.insert(
                        id,
                        construction.insert_universal_springs(world, parent, child, laws),
                    );
                }
                if let Some(&limits) = initial.speed_caps.get(&id) {
                    speed_caps.insert(
                        id,
                        construction.insert_speed_caps(world, parent, child, limits),
                    );
                }
                (
                    id,
                    Joint {
                        handle,
                        initial: construction,
                        spring,
                        universal_angles: [0.; 2],
                        universal_stops,
                        previous_angle: angle(world, handle, construction, bodies),
                        position: 0.0,
                    },
                )
            })
            .collect();
        Self {
            authored_welds: initial.records.values().any(|c| c.authored_weld()),
            wide_universal: initial
                .records
                .iter()
                .filter_map(|(&id, c)| c.wide_universal().then_some(id))
                .collect(),
            gravity_tree: Default::default(),
            records,
            universal_springs,
            projected_universal: BTreeMap::new(),
            universal_motors: BTreeMap::new(),
            speed_caps,
            screws,
            efforts: BTreeMap::new(),
            angular_efforts: BTreeMap::new(),
            anchor,
            world_roots,
            next: initial.next,
            open: true,
        }
    }
    pub(super) fn preserve_high_water(&mut self, previous: &Self) {
        self.next = self.next.max(previous.next);
    }
    pub(super) fn forget_body(&mut self, id: BodyId, world: &mut PhysicsWorld) -> Vec<BodyId> {
        let mut survivors = Vec::new();
        self.gravity_tree.take();
        self.records.retain(|_, joint| {
            let endpoints = joint.initial;
            if endpoints.child == id {
                survivors.extend(endpoints.parent);
                false
            } else if endpoints.parent == Some(id) {
                survivors.push(endpoints.child);
                false
            } else {
                true
            }
        });
        for (_, caps) in self
            .speed_caps
            .extract_if(.., |id, _| !self.records.contains_key(id))
        {
            for cap in caps.into_iter().flatten() {
                cap.remove(world);
            }
        }
        self.projected_universal
            .retain(|id, _| self.records.contains_key(id));
        self.universal_motors
            .retain(|(id, _), _| self.records.contains_key(id));
        self.universal_springs
            .retain(|id, _| self.records.contains_key(id));
        self.screws.retain(|id, _| self.records.contains_key(id));
        self.wide_universal
            .retain(|id| self.records.contains_key(id));
        self.angular_efforts
            .retain(|id, _| self.records.contains_key(id));
        self.efforts.retain(|id, _| self.records.contains_key(id));
        survivors
    }
    pub(super) fn release_unused_anchor(&mut self, world: &mut PhysicsWorld) {
        self.world_roots.retain(|_, root| {
            let retained = self.records.contains_key(&root.joint);
            if !retained {
                world.remove_body(root.handle);
            }
            retained
        });
        if self.records.values().all(|joint| {
            joint.initial.parent.is_some() || matches!(joint.initial.solver, Solver::Reduced)
        }) && let Some(anchor) = self.anchor.take()
        {
            world.remove_body(anchor);
        }
    }
}

fn angle(
    world: &PhysicsWorld,
    handle: Handle,
    construction: Construction,
    bodies: &BTreeMap<BodyId, BodyEntry>,
) -> f64 {
    if matches!(construction.motion, JointMotion::Screw { .. }) {
        return construction.screw_twist(world, bodies);
    }
    if !matches!(construction.motion, JointMotion::Angular) || construction.exact_lock {
        return 0.;
    }
    let handle = match handle {
        Handle::Reduced(handle) => {
            let (multibody, link) = world
                .multibody_joints
                .get(handle)
                .expect("registered articulation handle");
            return multibody
                .link(link)
                .expect("registered articulation link")
                .joint
                .coords()[3];
        }
        Handle::Impulse(handle) => handle,
    };
    let parent_rotation = construction.parent.map_or(Rotation::IDENTITY, |parent| {
        *world.bodies[bodies[&parent].0].rotation()
    });
    world
        .impulse_joints
        .get(handle)
        .expect("registered joint handle")
        .data
        .as_revolute()
        .expect("registered revolute constraint")
        .angle(
            &parent_rotation,
            world.bodies[bodies[&construction.child].0].rotation(),
        )
}

impl RapierBackend {
    /// Accumulated one-step torque on a resident joint, including a queued zero.
    pub fn pending_joint_effort(&self, id: JointId) -> Option<f64> {
        self.joints.efforts.get(&id).copied()
    }

    /// Construct an unbounded revolute joint from a dynamic body to the world.
    /// Anchor/axis use world coordinates; the current construction pose is angle zero.
    /// Construction closes at the first advancing step and reopens after reset.
    /// Invalid inputs leave body, joint and anchor state unchanged.
    pub(crate) fn insert_world_revolute(
        &mut self,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.insert_revolute(None, child, anchor, axis, Solver::Impulse)
    }

    pub(crate) fn insert_revolute(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
        solver: Solver,
    ) -> Result<JointId, PhysicsError> {
        self.insert_configured_joint(
            parent,
            child,
            anchor,
            axis,
            solver,
            AngularConstraint::Revolute(None),
        )
    }

    /// Construct a reduced-coordinate joint with finite stops measured from its
    /// construction pose in radians. Retain both stops across world resets.
    pub fn add_limited_revolute(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
        limits: [f64; 2],
    ) -> Result<JointId, PhysicsError> {
        let [lower, upper] = limits;
        if !lower.is_finite() || !upper.is_finite() || lower > 0.0 || upper < 0.0 {
            return Err(PhysicsError::InvalidValue);
        }
        self.insert_configured_joint(
            parent,
            child,
            anchor,
            axis,
            Solver::Reduced,
            AngularConstraint::Revolute(Some(limits)),
        )
    }

    /// Lock all six relative coordinates at the authored construction transform.
    /// The joint frame is expressed in world coordinates. A fully locked
    /// constraint has no free axis and accepts no joint effort.
    pub fn add_fixed_joint(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
    ) -> Result<JointId, PhysicsError> {
        self.insert_configured_joint(
            parent,
            child,
            anchor,
            [1., 0., 0.],
            Solver::Reduced,
            AngularConstraint::Weld,
        )
    }

    /// Only genuinely authored weld links are eligible for public link wrenches.
    pub fn is_welded_link(&self, id: BodyId) -> bool {
        self.joints.records.values().any(|joint| {
            joint.initial.authored_weld()
                && (joint.initial.child == id || joint.initial.parent == Some(id))
        })
    }

    fn insert_configured_joint(
        &mut self,
        parent: Option<BodyId>,
        child: BodyId,
        anchor: [f64; 3],
        axis: [f64; 3],
        solver: Solver,
        constraint: AngularConstraint,
    ) -> Result<JointId, PhysicsError> {
        if !self.joints.open {
            return Err(PhysicsError::JointConstructionClosed);
        }
        let (handle, spec, ..) = self.bodies.get(&child).ok_or(PhysicsError::MissingBody)?;
        if parent == Some(child) {
            return Err(PhysicsError::InvalidJoint);
        }
        let parent_handle = parent
            .map(|id| {
                self.bodies
                    .get(&id)
                    .map(|entry| entry.0)
                    .ok_or(PhysicsError::MissingBody)
            })
            .transpose()?;
        if spec.continuous_collision
            || parent.is_some_and(|id| self.bodies[&id].1.continuous_collision)
        {
            return Err(PhysicsError::Unsupported {
                engine: self.engine(),
                capability: "joints on a continuous-collision independent body",
            });
        }
        if self.prescribed_member(child) || parent.is_some_and(|id| self.prescribed_member(id)) {
            return Err(PhysicsError::Unsupported {
                engine: self.engine(),
                capability: "topology edit on a prescribed articulation",
            });
        }
        if spec.kinematic
            || parent_handle.is_some_and(|handle| self.world.bodies[handle].is_kinematic())
        {
            return Err(PhysicsError::Unsupported {
                engine: self.engine(),
                capability: "joints on a prescribed kinematic boundary",
            });
        }
        if spec.mass.is_none() || !finite(anchor) || !finite(axis) {
            return Err(PhysicsError::InvalidJoint);
        }
        self.preflight_solver(parent, child, solver)?;
        let axis = Vector::from_array(axis)
            .try_normalize()
            .ok_or(PhysicsError::InvalidJoint)?;
        let basis = match constraint {
            AngularConstraint::Ball(rotation) | AngularConstraint::Universal(rotation, _) => {
                rotation
            }
            _ => Rotation::from_rotation_arc(Vector::X, axis),
        };
        let world_frame = Pose::from_parts(Vector::from_array(anchor), basis);
        let frame1 = parent_handle.map_or(world_frame, |parent| {
            self.world.bodies[parent].position().inverse() * world_frame
        });
        let frame2 = self.world.bodies[*handle].position().inverse() * world_frame;
        if !finite(frame1.translation.to_array())
            || !frame1.rotation.is_finite()
            || !finite(frame2.translation.to_array())
            || !frame2.rotation.is_finite()
        {
            return Err(PhysicsError::InvalidJoint);
        }
        if matches!(solver, Solver::Reduced) {
            let free_axis = matches!(constraint, AngularConstraint::Revolute(_)).then_some(axis);
            self.preflight_velocities(parent, child, Vector::from_array(anchor), free_axis)?;
        }
        let next = self
            .joints
            .next
            .checked_add(1)
            .ok_or(PhysicsError::JointIdOverflow)?;
        let construction = Construction {
            parent,
            child,
            frame1,
            frame2,
            solver,
            limits: match constraint {
                AngularConstraint::Revolute(limits)
                | AngularConstraint::Prismatic(limits)
                | AngularConstraint::Screw { limits, .. } => limits,
                AngularConstraint::Weld
                | AngularConstraint::Ball(_)
                | AngularConstraint::Universal(_, _) => None,
            },
            exact_lock: matches!(constraint, AngularConstraint::Weld),
            robot: false,
            spring: None,
            additional_pgs_iterations: 0,
            motion: match constraint {
                AngularConstraint::Prismatic(_) => JointMotion::Prismatic,
                AngularConstraint::Screw {
                    lead, effort_limit, ..
                } => JointMotion::Screw { lead, effort_limit },
                AngularConstraint::Ball(_) => JointMotion::Ball,
                AngularConstraint::Universal(_, limits) => JointMotion::Universal(limits),
                _ => JointMotion::Angular,
            },
        };
        let world = &mut self.world;
        let parent = parent_handle.unwrap_or_else(|| {
            if matches!(solver, Solver::Reduced) {
                let root = world.insert_body(RigidBodyBuilder::fixed());
                self.joints.world_roots.insert(
                    child,
                    WorldRoot {
                        handle: root,
                        joint: JointId::from_raw(self.joints.next),
                    },
                );
                root
            } else {
                *self
                    .joints
                    .anchor
                    .get_or_insert_with(|| world.insert_body(RigidBodyBuilder::fixed()))
            }
        });
        if world.bodies[parent].is_dynamic() {
            *world.bodies[parent].activation_mut() = RigidBodyActivation::cannot_sleep();
        }
        // Default sleeping would stop low-amplitude articulated motion mid-cycle.
        *world.bodies[*handle].activation_mut() = RigidBodyActivation::cannot_sleep();
        let child_handle = *handle;
        let handle = construction.insert(world, parent, child_handle);
        let id = JointId::from_raw(self.joints.next);
        if construction.wide_universal() {
            self.joints.wide_universal.insert(id);
        }
        if let Some(row) = construction.insert_screw_row(world, parent, child_handle) {
            self.joints.screws.insert(id, row);
        }
        self.joints.gravity_tree.take();
        self.joints.records.insert(
            id,
            Joint {
                handle,
                initial: construction,
                spring: None,
                universal_angles: [0.; 2],
                universal_stops: construction.insert_universal_stops(world, parent, child_handle),
                previous_angle: angle(world, handle, construction, &self.bodies),
                position: 0.0,
            },
        );
        self.joints.authored_welds |= construction.authored_weld();
        self.joints.next = next;
        Ok(id)
    }

    /// Registered child endpoint of a revolute constraint.
    pub(crate) fn world_joint_child(&self, id: JointId) -> Option<BodyId> {
        self.joints
            .records
            .get(&id)
            .map(|joint| joint.initial.child)
    }

    pub(crate) fn world_joint_state(&self, id: JointId) -> Option<JointState> {
        let joint = self.joints.records.get(&id)?;
        match joint.initial.motion {
            JointMotion::Ball | JointMotion::Universal(_) => return None,
            JointMotion::Prismatic => return Some(joint.initial.slider(&self.world, &self.bodies)),
            JointMotion::Angular | JointMotion::Screw { .. } => {}
        }
        if joint.initial.exact_lock {
            return Some(JointState {
                position: 0.,
                velocity: 0.,
            });
        }
        let handle = self.bodies.get(&joint.initial.child)?.0;
        let body = &self.world.bodies[handle];
        let (parent_rotation, parent_velocity) =
            joint
                .initial
                .parent
                .map_or((Rotation::IDENTITY, Vector::ZERO), |parent| {
                    let body = &self.world.bodies[self.bodies[&parent].0];
                    (*body.rotation(), body.angvel())
                });
        Some(JointState {
            position: joint.position,
            velocity: (body.angvel() - parent_velocity)
                .dot(parent_rotation * joint.initial.frame1.rotation * Vector::X),
        })
    }

    pub(crate) fn delete_world_joint(&mut self, id: JointId) -> bool {
        let Some(joint) = self.joints.records.remove(&id) else {
            return false;
        };
        self.joints.gravity_tree.take();
        self.resume_parked_speed_groups();
        self.joints.efforts.remove(&id);
        self.joints.angular_efforts.remove(&id);
        self.joints.wide_universal.remove(&id);
        if let Some(caps) = self.joints.speed_caps.remove(&id) {
            for cap in caps.into_iter().flatten() {
                cap.remove(&mut self.world);
            }
        }
        if let Some(row) = self.joints.projected_universal.remove(&id) {
            self.world.remove_impulse_joint(row);
        }
        for axis in 0..2 {
            if let Some(row) = self.joints.universal_motors.remove(&(id, axis)) {
                self.world.remove_impulse_joint(row);
            }
        }
        if let Some(rows) = self.joints.universal_springs.remove(&id) {
            for (_, row) in rows.into_iter().flatten() {
                self.world.remove_impulse_joint(row);
            }
        }
        for handle in joint
            .universal_stops
            .into_iter()
            .flatten()
            .chain(joint.spring)
        {
            self.world.remove_impulse_joint(handle);
        }
        if let Some(row) = self.joints.screws.remove(&id) {
            self.world.remove_impulse_joint(row);
        }
        match joint.handle {
            Handle::Impulse(handle) => {
                self.world.remove_impulse_joint(handle);
            }
            Handle::Reduced(handle) => self.world.remove_multibody_joint(handle),
        }
        for body in joint
            .initial
            .parent
            .into_iter()
            .chain([joint.initial.child])
        {
            self.release_unconstrained_body(body);
        }
        self.joints.release_unused_anchor(&mut self.world);
        if matches!(joint.handle, Handle::Reduced(_)) {
            self.joints.initialize_velocities(&mut self.world);
        }
        true
    }

    pub(super) fn release_unconstrained_body(&mut self, id: BodyId) {
        // Construction records retain registered endpoints; removal excludes its retired body.
        let handle = self.bodies[&id].0;
        if self.world.bodies[handle].additional_pgs_iterations() != 0
            && !self
                .joints
                .records
                .values()
                .any(|j| j.initial.child == id && j.initial.additional_pgs_iterations != 0)
        {
            self.world.bodies[handle].set_additional_pgs_iterations(0);
        }
        if self.world.bodies[handle].additional_solver_iterations() != 0
            && !self.joints.records.iter().any(|(key, j)| {
                j.initial.child == id
                    && (matches!(
                        j.initial.motion,
                        JointMotion::Ball | JointMotion::Universal(_)
                    ) || self.joints.speed_caps.contains_key(key))
            })
        {
            self.world.bodies[handle].set_additional_solver_iterations(0);
        }
        if self.world.bodies[handle].is_dynamic()
            && !self.has_joint(id).expect("registered endpoint")
        {
            *self.world.bodies[handle].activation_mut() = RigidBodyActivation::active();
        }
    }

    /// Whether a registered body participates in a solver constraint.
    pub(crate) fn has_joint(&self, id: BodyId) -> Option<bool> {
        let (handle, ..) = self.bodies.get(&id)?;
        Some(
            self.world.impulse_joints_with(*handle).next().is_some()
                || self.world.multibody_joints_with(*handle).next().is_some(),
        )
    }
}

#[cfg(test)]
#[path = "../tests/private/joints.rs"]
mod tests;
