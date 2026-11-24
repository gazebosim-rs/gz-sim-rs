// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Detached, local qualification of the airborne locked foot and its rising lift.
use super::*;

mod restart;
pub use restart::{LandingRestartState, RestartPairHistory};

/// A retained point of the incoming foot and pad, in the native collider pair order.
pub struct LandingCachedPoint {
    pub local_p1: [f64; 3],
    pub local_p2: [f64; 3],
    pub gap_m: f64,
    pub warm_normal_ns: f64,
    pub warm_tangent_world_ns: [f64; 3],
    pub warm_twist_nm_s: f64,
}

/// Native state verified at the cut and the observed result of each detached solve.
pub struct LandingTrial {
    /// Root, upper link and pad in that order.
    pub initial: [BodyState; 3],
    pub generalized_velocity: Vec<f64>,
    pub joint_coordinates: Vec<Vec<f64>>,
    /// Narrow-phase records on the receiving foot and pad before continuing.
    pub incoming_manifolds: usize,
    pub incoming_points: Vec<LandingCachedPoint>,
    pub incoming_warmstart: ContactWarmstart,
    pub pad_first_in_pair: Option<bool>,
    pub impulses: Vec<f64>,
    pub final_states: [BodyState; 3],
}

fn unsupported() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "rapier",
        capability: "local landing audit requires matching airborne passive locked feet and one rising cuboid support",
    }
}

/// Detached, feature-gated native continuation of a supported-foot experiment.
/// The source simulation retains its original physics state and public clock.
pub struct LandingFork {
    native: RapierBackend,
    ids: [BodyId; 3],
    outer_dt: Duration,
}

impl LandingFork {
    /// Solver-iteration count and warm-start coefficient on this detached world.
    pub fn contact_settings(&self) -> (usize, f64) {
        let p = &self.native.world.integration_parameters;
        (p.num_solver_iterations, p.warmstart_coefficient)
    }

    /// Change only the detached qualification world's native solve parameters.
    /// The public actuator clock and stored point impulses are preserved.
    pub fn set_contact_settings(
        &mut self,
        iterations: usize,
        warmstart: f64,
    ) -> Result<(), PhysicsError> {
        if !(1..=32).contains(&iterations) || !(0.0..=1.0).contains(&warmstart) {
            return Err(PhysicsError::InvalidValue);
        }
        let p = &mut self.native.world.integration_parameters;
        p.num_solver_iterations = iterations;
        p.warmstart_coefficient = warmstart;
        Ok(())
    }

    /// Foot, upper link, and lift at the latest solved native boundary.
    pub fn states(&self) -> [BodyState; 3] {
        self.ids
            .map(|id| self.native.state(id).expect("validated landing layout"))
    }

    fn tree(&self) -> &Multibody {
        let index = self
            .native
            .articulation_index(self.ids[0])
            .expect("validated landing layout");
        self.native
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("validated landing layout")
    }

    pub fn generalized_velocity(&self) -> Vec<f64> {
        self.tree().generalized_velocity().iter().copied().collect()
    }

    pub fn joint_coordinates(&self) -> Vec<Vec<f64>> {
        self.tree()
            .links()
            .map(|l| l.joint.coords().to_vec())
            .collect()
    }

    /// Solved foot–pad normal impulse on the latest native step, in N·s.
    pub fn normal_impulse(&self) -> f64 {
        let [foot, _, pad] = self.ids;
        self.native
            .rigid_contact_observations()
            .iter()
            .filter(|p| (p.a, p.b) == (foot, pad) || (p.a, p.b) == (pad, foot))
            .map(|p| p.normal_impulse)
            .sum()
    }

    /// Solved native patches on the latest native step, in public body order.
    pub fn contact_patches(&self) -> Vec<RigidContactPatch> {
        self.native.rigid_contact_patches()
    }

    /// Advance exactly one unchanged outer interval. Apply a new lift velocity
    /// once, at the start. Each substep completes a full native contact solve
    /// before its observer runs. An empty observer skips all instrumentation.
    pub fn step_outer(
        &mut self,
        substeps: u32,
        pad_velocity: Option<f64>,
        mut after_substep: impl FnMut(&LandingFork),
    ) -> Result<(), PhysicsError> {
        if !matches!(substeps, 1 | 2 | 4 | 8 | 16)
            || !self
                .outer_dt
                .as_nanos()
                .is_multiple_of(u128::from(substeps))
        {
            return Err(PhysicsError::InvalidValue);
        }
        self.step_outer_balanced(substeps, pad_velocity, |_, fork| after_substep(fork))
    }

    /// Advance a detached world on nested, nanosecond-aligned boundaries.
    /// The observer receives elapsed time since the beginning of this actuator tick.
    pub fn step_outer_balanced(
        &mut self,
        substeps: u32,
        pad_velocity: Option<f64>,
        mut after_substep: impl FnMut(Duration, &LandingFork),
    ) -> Result<(), PhysicsError> {
        let grid = gz_physics::NativeStepGrid::new(self.outer_dt, substeps)?;
        let mut elapsed = Duration::ZERO;
        if let Some(vz) = pad_velocity {
            self.native
                .set_world_velocity(self.ids[2], [0., 0., vz], [0.; 3])?;
        }
        for dt in grid.intervals() {
            self.native.step(dt)?;
            elapsed += dt;
            after_substep(elapsed, self);
        }
        Ok(())
    }
}

impl RapierBackend {
    /// Build a horizontal translating cuboid for the perfectly plastic, frictionless
    /// impact reference. The minimum combine rules pair its zero coefficients with
    /// the fixture's ordinary default-material sole. Reset retains the same surface.
    pub fn spawn_ideal_landing_pad(&mut self, spec: BodySpec) -> Result<BodyId, PhysicsError> {
        self.spawn_ideal_landing_surface(spec, RotationSpec::default())
    }

    /// Translation or rotation of a finite frictionless, perfectly inelastic support cuboid.
    pub fn spawn_ideal_landing_surface(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
    ) -> Result<BodyId, PhysicsError> {
        if !matches!(spec.shape, Shape::Box { .. }) || spec.mass.is_none() {
            return Err(PhysicsError::InvalidValue);
        }
        let id = self.spawn_velocity_boundary(spec, rotation, None)?;
        let (handle, construction, ..) = self.bodies.get_mut(&id).expect("newly spawned pad");
        let material = |collider: &mut Collider| {
            collider.set_friction(0.);
            collider.set_restitution(0.);
            collider.set_friction_combine_rule(CoefficientCombineRule::Min);
            collider.set_restitution_combine_rule(CoefficientCombineRule::Min);
        };
        material(&mut construction.collider);
        let collider = self.world.bodies[*handle].colliders()[0];
        material(&mut self.world.colliders[collider]);
        Ok(id)
    }

    /// Read the last assembled native inverse mass along four world-up impulses.
    /// Exact-locked two-link trees have six free root coordinates and no removed
    /// kinematic rows, so their cached LU factorization is in root-coordinate order.
    pub fn landing_normal_mobility(
        &self,
        root: BodyId,
        arms: [[f64; 3]; 4],
    ) -> Result<[[f64; 4]; 4], PhysicsError> {
        use rapier3d_f64::na::DMatrix;
        if arms.iter().flatten().any(|v| !v.is_finite()) || self.joints.construction_open() {
            return Err(PhysicsError::InvalidValue);
        }
        let index = self.articulation_index(root)?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .ok_or(PhysicsError::InvalidJoint)?;
        if tree.num_links() != 2
            || tree.ndofs() != 6
            || tree.root().joint.ndofs() != 6
            || tree.root().rigid_body_handle() != self.bodies[&root].0
            || tree.links().any(|l| l.joint.kinematic)
        {
            return Err(unsupported());
        }
        if let Some(p) = &self.welded_projection {
            return Ok(p.normal_mobility(
                self.world.bodies[self.bodies[&root].0].center_of_mass(),
                arms,
            ));
        }
        let j = tree.body_jacobian(0);
        let rows = DMatrix::from_fn(4, 6, |i, k| {
            j[(2, k)] + arms[i][1] * j[(3, k)] - arms[i][0] * j[(4, k)]
        });
        let weighted = tree
            .inv_augmented_mass()
            .solve(&rows.transpose())
            .ok_or(PhysicsError::InvalidValue)?;
        let matrix = rows * weighted;
        Ok(std::array::from_fn(|i| {
            std::array::from_fn(|j| matrix[(i, j)])
        }))
    }

    /// Select the number of unbiased post-integration sweeps for one audit solve.
    pub fn step_landing_with_relaxation(
        &mut self,
        dt: Duration,
        passes: usize,
    ) -> Result<(), PhysicsError> {
        if dt.is_zero() || passes > 32 {
            return Err(PhysicsError::InvalidValue);
        }
        let p = &mut self.world.integration_parameters;
        let iterations = p.num_solver_iterations;
        let saved = std::mem::replace(&mut p.num_internal_stabilization_iterations, passes);
        let result = self.step_landing_with_iterations(dt, iterations);
        self.world
            .integration_parameters
            .num_internal_stabilization_iterations = saved;
        result
    }

    /// Run the production selected-pair geometry path with an audit-only
    /// relaxation override and restore the world's iteration budget.
    pub fn step_landing_with_fresh_box_pairs(
        &mut self,
        dt: Duration,
        passes: usize,
        pairs: &[(BodyId, BodyId)],
    ) -> Result<(), PhysicsError> {
        if dt.is_zero() || passes > 32 || pairs.is_empty() {
            return Err(PhysicsError::InvalidValue);
        }
        let previous = std::mem::replace(
            &mut self
                .world
                .integration_parameters
                .num_internal_stabilization_iterations,
            passes,
        );
        let result = self.step_box_features(dt, pairs);
        self.world
            .integration_parameters
            .num_internal_stabilization_iterations = previous;
        result.map(|_| ())
    }

    /// Disable the position correction speed or warm start for a controlled native solve.
    pub fn step_landing_response_control(
        &mut self,
        dt: Duration,
        passes: usize,
        bias: bool,
        warmstart: bool,
        recycle: bool,
    ) -> Result<(), PhysicsError> {
        let p = &mut self.world.integration_parameters;
        let saved_recycle = p.contact_recycling;
        if !recycle {
            p.contact_recycling = false;
        }
        let correction = p.normalized_max_corrective_velocity;
        let coefficient = p.warmstart_coefficient;
        if !bias {
            p.normalized_max_corrective_velocity = 0.0;
        }
        if !warmstart {
            p.warmstart_coefficient = 0.0;
        }
        let result = self.step_landing_with_relaxation(dt, passes);
        let p = &mut self.world.integration_parameters;
        p.contact_recycling = saved_recycle;
        p.normalized_max_corrective_velocity = correction;
        p.warmstart_coefficient = coefficient;
        result
    }

    /// Qualification hook for the local landing fixture. Apply the native iteration
    /// budget to a single advancing solve and restore the previous setting on return.
    pub fn step_landing_with_iterations(
        &mut self,
        dt: Duration,
        iterations: usize,
    ) -> Result<(), PhysicsError> {
        if dt.is_zero() || !(1..=32).contains(&iterations) {
            return Err(PhysicsError::InvalidValue);
        }
        let saved = std::mem::replace(
            &mut self.world.integration_parameters.num_solver_iterations,
            iterations,
        );
        let result = self.step(dt);
        self.world.integration_parameters.num_solver_iterations = saved;
        result?;
        if self.last_step_quarantined() {
            return Err(PhysicsError::NonFiniteStep { engine: "rapier" });
        }
        Ok(())
    }

    fn landing_layout(&self, root: BodyId, pad: BodyId) -> Result<[BodyId; 3], PhysicsError> {
        let w = &self.world;
        let tree = w
            .multibody_joints
            .get_multibody(self.articulation_index(root)?)
            .ok_or_else(unsupported)?;
        if self.bodies.len() != 3
            || w.bodies.len() != 3
            || w.colliders.len() != 3
            || self.joints.records.len() != 1
            || tree.num_links() != 2
            || tree.ndofs() != 6
            || tree.link(1).ok_or_else(unsupported)?.joint.data.user_data != EXACT_REVOLUTE_LOCK
            || self.articulation_tier(root)? != ArticulationTier::Dynamic
            || !self.joints.efforts.is_empty()
            || !self.pending_wrenches.is_empty()
            || !self.held_wrenches.is_empty()
            || !self.held_supports.is_empty()
            || !self.cold_support_pairs.is_empty()
            || self.has_pending_support_loss()
            || self.curves.authored()
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || self.conserved_rotor.is_some()
            || self.fleet_clock != FleetClock::Coupled
            || !self.fleet_resume_positions.is_empty()
            || matches!(self.fleet_dispatch, fleet_isolation::Dispatch::Active(_))
            || !w.impulse_joints.is_empty()
            || !w.soft_bodies.is_empty()
        {
            return Err(unsupported());
        }
        #[cfg(feature = "partition-checkpoint")]
        if self.native_profile.is_some()
            || self.pair_observer.is_some()
            || self.grasp_payload_observer.is_some()
            || self.join_queries_pending
        {
            return Err(unsupported());
        }
        let child = tree.link(1).ok_or_else(unsupported)?.rigid_body_handle();
        let child_id = BodyId::from_raw(w.bodies[child].user_data as u64);
        let ids = [root, child_id, pad];
        if pad == root || pad == child_id {
            return Err(unsupported());
        }
        for id in ids {
            let handle = self.bodies.get(&id).ok_or_else(unsupported)?.0;
            let body = &w.bodies[handle];
            let [c] = body.colliders() else {
                return Err(unsupported());
            };
            if w.colliders[*c].shape().as_cuboid().is_none()
                || body.is_sleeping()
                || (id == pad
                    && (body.body_type() != RigidBodyType::KinematicVelocityBased
                        || body.linvel().z <= 0.))
            {
                return Err(unsupported());
            }
        }
        let collider = |id| &w.colliders[w.bodies[self.bodies[&id].0].colliders()[0]];
        let sole = collider(root);
        let deck = collider(pad);
        if rapier3d_f64::parry::query::distance(
            sole.position(),
            sole.shape(),
            deck.position(),
            deck.shape(),
        )
        .map_err(|_| unsupported())?
        .distance
            <= 0.
            || self.native_contact_warmstart(root, pad)?.normal_ns != 0.
        {
            return Err(unsupported());
        }
        Ok(ids)
    }

    /// Fork an airborne foot with its receiving native contact graph and the
    /// donor's motion, or clone the receiver exactly when no donor is supplied.
    /// The outer interval must match the interval used to reach the cut.
    pub fn landing_fork(
        &self,
        motion: Option<&Self>,
        root: BodyId,
        pad: BodyId,
        outer_dt: Duration,
    ) -> Result<LandingFork, PhysicsError> {
        self.copy_landing(motion, root, pad, outer_dt, 0.)
    }

    fn copy_landing(
        &self,
        motion: Option<&Self>,
        root: BodyId,
        pad: BodyId,
        outer_dt: Duration,
        pad_height_offset_m: f64,
    ) -> Result<LandingFork, PhysicsError> {
        if outer_dt.is_zero()
            || !pad_height_offset_m.is_finite()
            || outer_dt.as_secs_f64() != self.world.integration_parameters.dt
        {
            return Err(PhysicsError::InvalidValue);
        }
        let ids = self.landing_layout(root, pad)?;
        let mut trial = self.copy_passive_interval()?;
        if let Some(source) = motion {
            if source.landing_layout(root, pad)? != ids
                || source.world.gravity != self.world.gravity
                || source.world.integration_parameters != self.world.integration_parameters
                || source.articulation_index(root)? != self.articulation_index(root)?
            {
                return Err(unsupported());
            }
            let link_joint = |s: &Self| {
                s.world
                    .multibody_joints
                    .get_multibody(s.articulation_index(root).unwrap())
                    .unwrap()
                    .link(1)
                    .unwrap()
                    .joint
                    .data
            };
            if link_joint(self) != link_joint(source) {
                return Err(unsupported());
            }
            let inertia_key =
                |inertia: &Option<Inertia>| inertia.map(|i| (i.center, i.orientation, i.tensor));
            for id in ids {
                let (h, construction, rotation, inertia, group) = &self.bodies[&id];
                let (other, origin, orient, mass, filter) = &source.bodies[&id];
                let a = &self.world.colliders[self.world.bodies[*h].colliders()[0]];
                let b = &source.world.colliders[source.world.bodies[*other].colliders()[0]];
                if h != other
                    || construction.position != origin.position
                    || construction.mass != origin.mass
                    || construction.velocity != origin.velocity
                    || construction.articulation != origin.articulation
                    || construction.can_sleep != origin.can_sleep
                    || construction.kinematic != origin.kinematic
                    || rotation.orientation != orient.orientation
                    || rotation.angular_velocity != orient.angular_velocity
                    || inertia_key(inertia) != inertia_key(mass)
                    || group != filter
                    || a.shape().as_cuboid() != b.shape().as_cuboid()
                    || a.position_wrt_parent() != b.position_wrt_parent()
                    || a.is_sensor() != b.is_sensor()
                    || a.is_enabled() != b.is_enabled()
                    || a.collision_groups() != b.collision_groups()
                    || a.solver_groups() != b.solver_groups()
                    || a.active_collision_types() != b.active_collision_types()
                    || a.friction_combine_rule() != b.friction_combine_rule()
                    || a.restitution_combine_rule() != b.restitution_combine_rule()
                    || a.friction() != b.friction()
                    || a.restitution() != b.restitution()
                    || a.contact_skin() != b.contact_skin()
                {
                    return Err(unsupported());
                }
                trial.world.bodies[*h].copy_from(&source.world.bodies[*other]);
            }
            trial.world.multibody_joints = source.world.multibody_joints.clone();
            trial.joints = source.joints.clone();
        }
        if pad_height_offset_m != 0. {
            let body = &mut trial.world.bodies[trial.bodies[&pad].0];
            let mut p = body.translation();
            p.z += pad_height_offset_m;
            body.set_translation(p, true);
        }
        if motion.is_some() || pad_height_offset_m != 0. {
            trial
                .world
                .bodies
                .propagate_modified_body_positions_to_colliders(&mut trial.world.colliders);
        }
        let states = |b: &Self| ids.map(|id| b.state(id).expect("validated landing layout"));
        let initial = states(&trial);
        let mut expected = states(motion.unwrap_or(self));
        expected[2].position[2] += pad_height_offset_m;
        if initial != expected {
            return Err(PhysicsError::InvalidValue);
        }
        if pad_height_offset_m != 0. {
            trial.landing_layout(root, pad)?;
        }
        Ok(LandingFork {
            native: trial,
            ids,
            outer_dt,
        })
    }

    /// Measure a detached continuation on the original grid. An optional pad
    /// height perturbation tests the reaction's sensitivity to incoming gap.
    pub fn landing_trial(
        &self,
        motion: Option<&Self>,
        root: BodyId,
        pad: BodyId,
        steps: usize,
        dt: Duration,
        pad_height_offset_m: f64,
    ) -> Result<LandingTrial, PhysicsError> {
        if !(1..=256).contains(&steps) {
            return Err(PhysicsError::InvalidValue);
        }
        let mut fork = self.copy_landing(motion, root, pad, dt, pad_height_offset_m)?;
        let initial = fork.states();
        let generalized_velocity = fork.generalized_velocity();
        let joint_coordinates = fork.joint_coordinates();
        let trial = &fork.native;
        let foot = trial.world.bodies[trial.bodies[&root].0].colliders()[0];
        let floor = trial.world.bodies[trial.bodies[&pad].0].colliders()[0];
        let pair = trial.world.narrow_phase.contact_pair(foot, floor);
        let incoming_manifolds = pair.map_or(0, |p| p.manifolds().len());
        let incoming_points = pair
            .iter()
            .flat_map(|p| p.manifolds())
            .flat_map(|m| &m.points)
            .map(|p| LandingCachedPoint {
                local_p1: p.local_p1.to_array(),
                local_p2: p.local_p2.to_array(),
                gap_m: p.dist,
                warm_normal_ns: p.data.warmstart_impulse,
                warm_tangent_world_ns: p.data.warmstart_tangent_world.to_array(),
                warm_twist_nm_s: p.data.warmstart_twist_impulse,
            })
            .collect();
        let pad_first_in_pair = pair.map(|p| p.collider1 == floor);
        let incoming_warmstart = trial.native_contact_warmstart(root, pad)?;
        let mut impulses = Vec::with_capacity(steps);
        for _ in 0..steps {
            fork.step_outer(1, None, |f| impulses.push(f.normal_impulse()))?;
        }
        Ok(LandingTrial {
            initial,
            generalized_velocity,
            joint_coordinates,
            incoming_manifolds,
            incoming_points,
            incoming_warmstart,
            pad_first_in_pair,
            impulses,
            final_states: fork.states(),
        })
    }
}

/// Read-only pair data from the last full narrow-phase update and native solve.
/// Anchors retain Rapier's solver convention (COM-local or world-attached).
pub struct LandingSolverPoint {
    pub foot_is_first: bool,
    pub relative_dominance: i16,
    pub normal_on_foot: [f64; 3],
    pub geometric_gap: f64,
    pub solver_gap_at_update: f64,
    pub local_foot: [f64; 3],
    pub local_pad: [f64; 3],
    pub solver_foot_anchor: [f64; 3],
    pub solver_pad_anchor: [f64; 3],
    pub impulse: f64,
}
impl RapierBackend {
    /// Inspect only the named, single-collider pair. The world is left untouched.
    pub fn landing_solver_points(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Vec<LandingSolverPoint>, PhysicsError> {
        let mut rows = vec![];
        let single = |id: &BodyId| {
            let h = self.bodies.get(id).ok_or(PhysicsError::MissingBody)?.0;
            let [c] = self.world.bodies[h].colliders() else {
                return Err(PhysicsError::InvalidValue);
            };
            Ok(*c)
        };
        let f = single(&foot)?;
        let p = single(&pad)?;
        let Some(pair) = self.world.narrow_phase.contact_pair(f, p) else {
            return Ok(rows);
        };
        let first = pair.collider1 == f;
        for m in pair.solver_manifolds() {
            for sc in &m.data.solver_contacts {
                let i = (sc.contact_id[0] & !rapier3d_f64::geometry::NEW_CONTACT_BIT) as usize;
                let point = &m.points[i];
                let (a, b, xa, xb) = if first {
                    (point.local_p1, point.local_p2, sc.anchor1, sc.anchor2)
                } else {
                    (point.local_p2, point.local_p1, sc.anchor2, sc.anchor1)
                };
                rows.push(LandingSolverPoint {
                    foot_is_first: first,
                    relative_dominance: m.data.relative_dominance,
                    normal_on_foot: (if first { -m.data.normal } else { m.data.normal }).to_array(),
                    geometric_gap: point.dist,
                    solver_gap_at_update: sc.dist,
                    local_foot: a.to_array(),
                    local_pad: b.to_array(),
                    solver_foot_anchor: xa.to_array(),
                    solver_pad_anchor: xb.to_array(),
                    impulse: point.data.impulse,
                });
            }
        }
        Ok(rows)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analytic_pad_keeps_zero_friction_and_restitution_through_native_reset() {
        let mut native = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let mut spec = BodySpec {
            position: [0., 0., -0.06],
            velocity: [0., 0., 0.3],
            mass: Some(1.),
            shape: Shape::Sphere { radius: 0.1 },
        };
        assert!(matches!(
            native.spawn_ideal_landing_pad(spec),
            Err(PhysicsError::InvalidValue)
        ));
        spec.shape = Shape::Box {
            half_extents: [0.33, 0.18, 0.06],
        };
        let id = native.spawn_ideal_landing_pad(spec).unwrap();
        let check = |native: &RapierBackend| {
            let (handle, construction, ..) = &native.bodies[&id];
            let live = &native.world.colliders[native.world.bodies[*handle].colliders()[0]];
            for collider in [live, &construction.collider] {
                assert_eq!(collider.friction(), 0.);
                assert_eq!(collider.restitution(), 0.);
                assert_eq!(
                    collider.friction_combine_rule(),
                    CoefficientCombineRule::Min
                );
                assert_eq!(
                    collider.restitution_combine_rule(),
                    CoefficientCombineRule::Min
                );
            }
        };
        check(&native);
        native.step(Duration::from_micros(500)).unwrap();
        native.reset().unwrap();
        check(&native);
    }

    #[test]
    fn landing_fork_keeps_command_boundaries_and_rejects_invalid_inputs_before_mutation() {
        let mut native = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
        let cube = |z, half_extents| BodySpec {
            position: [0., 0., z],
            mass: Some(1.),
            velocity: [0.; 3],
            shape: Shape::Box { half_extents },
        };
        let root = native
            .spawn_articulation_link(cube(0.2, [0.18, 0.10, 0.06]), RotationSpec::default(), None)
            .unwrap();
        let upper = native
            .spawn_articulation_link(
                cube(0.34, [0.05, 0.06, 0.06]),
                RotationSpec::default(),
                None,
            )
            .unwrap();
        native
            .add_limited_revolute(Some(root), upper, [0., 0., 0.27], [1., 0., 0.], [0., 0.])
            .unwrap();
        native.enable_exact_revolute_locks(root).unwrap();
        let pad = native
            .spawn_velocity_boundary(
                cube(-0.06, [0.33, 0.18, 0.06]),
                RotationSpec::default(),
                None,
            )
            .unwrap();
        native
            .set_world_velocity(pad, [0., 0., 0.3], [0.; 3])
            .unwrap();
        let dt = Duration::from_micros(500);
        native.step(dt).unwrap();
        let mut fork = native.landing_fork(None, root, pad, dt).unwrap();
        let initial = fork.states();
        let original = fork.contact_settings();
        for (iterations, coefficient) in [(0, 0.), (33, 1.), (4, f64::NAN), (1, -0.1)] {
            assert!(fork.set_contact_settings(iterations, coefficient).is_err());
            assert_eq!(fork.contact_settings(), original);
        }
        fork.set_contact_settings(1, 0.).unwrap();
        assert_eq!(fork.contact_settings(), (1, 0.));
        fork.set_contact_settings(original.0, original.1).unwrap();
        let q = fork.joint_coordinates();
        let v = fork.generalized_velocity();
        assert!(native.landing_fork(None, root, pad, dt / 2).is_err());
        for (substeps, command) in [(0, Some(4.)), (3, Some(4.)), (4, Some(f64::NAN))] {
            assert!(
                fork.step_outer(substeps, command, |_| panic!("refused before first solve"))
                    .is_err()
            );
            assert_eq!(fork.states(), initial);
            assert_eq!(fork.joint_coordinates(), q);
            assert_eq!(fork.generalized_velocity(), v);
        }

        // The pad is free of contact throughout this short passage. Compare
        // each observed native boundary with direct stepping from the same cut.
        let mut reference = native.copy_passive_interval().unwrap();
        let ids = [root, upper, pad];
        for (substeps, command) in [(4, Some(0.6)), (2, None), (1, Some(0.))] {
            if let Some(vz) = command {
                reference
                    .set_world_velocity(pad, [0., 0., vz], [0.; 3])
                    .unwrap();
            }
            let mut samples = 0;
            fork.step_outer(substeps, command, |sub| {
                samples += 1;
                reference.step(dt / substeps).unwrap();
                assert_eq!(sub.states(), ids.map(|id| reference.state(id).unwrap()));
            })
            .unwrap();
            assert_eq!(samples, substeps);
        }
        assert_eq!(initial, ids.map(|id| native.state(id).unwrap()));

        let odd = dt + Duration::from_nanos(1);
        native.step(odd).unwrap();
        let mut odd_fork = native.landing_fork(None, root, pad, odd).unwrap();
        let before = odd_fork.states();
        assert!(
            odd_fork
                .step_outer(2, Some(4.), |_| panic!("non-integral split"))
                .is_err()
        );
        assert_eq!(odd_fork.states(), before);
        let original_iterations = native.world.integration_parameters.num_solver_iterations;
        assert!(
            native
                .step_landing_with_iterations(Duration::ZERO, 16)
                .is_err()
        );
        assert_eq!(
            native.world.integration_parameters.num_solver_iterations,
            original_iterations
        );
        native.step_landing_with_iterations(dt, 16).unwrap();
        assert_eq!(
            native.world.integration_parameters.num_solver_iterations,
            original_iterations
        );
    }
}
