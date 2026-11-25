// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact translations for co-moving, centered boxes beyond the reach of every
//! free rigid body. Native contact bodies and manifolds stay in one Rapier world.
use super::*;
use rapier3d_f64::parry::partitioning::{Bvh, BvhBuildStrategy};
use rapier3d_f64::parry::utils::PoseOps;
use std::collections::HashMap;

// Amortize constructing a second spatial index. Below this count the native
// solver remains the normal path. This cutoff does not affect contact admission.
pub(super) const MIN_DECKS: usize = 256;
pub(super) const MAX_COORDINATE: f64 = 1.0e6;
const MAX_QUERY_SLACK: f64 = 0.25;
// Two successive surveys must find an unguarded, contact-free deck before retirement.
const SURVEY_INTERVAL: u8 = 16;
const DISCRETE_PROFILE_DT: Duration = Duration::from_millis(10);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FleetIsolationStats {
    pub dynamic_bodies: usize,
    pub native_decks: usize,
    pub analytic_decks: usize,
    /// Cumulative returns to analytic translation in this dispatch interval.
    pub retired_decks: u64,
}
pub(super) enum Dispatch {
    Disabled,
    Candidate,
    Active(Box<Isolation>),
}
struct Row {
    id: BodyId,
    handle: RigidBodyHandle,
    start: Option<AnalyticPose>,
    last_covered: u64,
    independent_motion: bool,
    coordinates: fleet_coordinates::Keys,
}
#[derive(Clone, Copy)]
struct AnalyticPose {
    position: Vector,
    orientation: [f64; 4],
    epoch: Duration,
}
impl AnalyticPose {
    fn at(&self, velocity: Vector, time: Duration) -> Vector {
        let dt = time - self.epoch;
        if dt.is_zero() {
            self.position
        } else {
            self.position + velocity * dt.as_secs_f64()
        }
    }
}
struct Payload {
    handle: RigidBodyHandle,
    radius: f64,
    previous_center: Vector,
    previous_half_width: f64,
}
pub(super) struct Isolation {
    index: Bvh,
    coordinates: fleet_coordinates::Coordinates,
    rows: Vec<Row>,
    payloads: Vec<Payload>,
    largest_deck_radius: f64,
    pub(super) velocity: Vector,
    elapsed: Duration,
    analytic: usize,
    native_rows: Vec<usize>,
    retired_epochs: HashMap<RigidBodyHandle, Duration>,
    survey: u64,
    retired: u64,
    steps_since_survey: u8,
}
impl Isolation {
    fn build(backend: &RapierBackend) -> Option<Self> {
        let world = &backend.world;
        if backend.bodies.len() < MIN_DECKS
            || backend.bodies.len() >= u32::MAX as usize
            || world.colliders.len() != backend.bodies.len()
            || world.broad_phase.adaptive_change_detection_margin
            || !world.impulse_joints.is_empty()
            || world.multibody_joints.iter().next().is_some()
            || world.narrow_phase.contact_pairs().next().is_some()
        {
            return None;
        }
        let mut velocity = None;
        let mut payloads = Vec::new();
        let mut largest_deck_radius = 0_f64;
        let mut rows = Vec::with_capacity(backend.bodies.len());
        let mut leaves = Vec::new();
        let mut coordinates = fleet_coordinates::Coordinates::new(&world.integration_parameters);
        let mut keys = Default::default();
        for (&id, (handle, spec, _, inertia, _)) in &backend.bodies {
            let body = &world.bodies[*handle];
            let cuboid = spec.collider.shape().as_cuboid()?;
            if *spec.collider.position() != Pose::IDENTITY
                || body.colliders().len() != 1
                || !spec.can_sleep
                || spec.articulation.is_some()
                || inertia.is_some_and(|v| v.center != [0.; 3])
                || spec.mass.is_none()
                || (!spec.kinematic && !body.is_dynamic())
                || body.additional_solver_iterations() != 0
                || spec.collider.contact_skin() != 0.
                || !body.is_enabled()
                || body.translation().abs().max_element() > MAX_COORDINATE
                || cuboid.half_extents.max_element() > MAX_COORDINATE
                || body.is_ccd_enabled()
                || body.soft_ccd_prediction() != 0.
            {
                return None;
            }
            let mut coordinate_keys = fleet_coordinates::Keys::default();
            let start = if spec.kinematic {
                largest_deck_radius = largest_deck_radius.max(cuboid.half_extents.length());
                let rotation = *body.rotation();
                if (Rotation::IDENTITY * rotation)
                    .normalize()
                    .to_array()
                    .map(f64::to_bits)
                    != rotation.to_array().map(f64::to_bits)
                {
                    return None;
                }
                let extent = Pose::from_parts(Vector::ZERO, rotation)
                    .absolute_transform_vector(cuboid.half_extents);
                coordinate_keys =
                    coordinates.register(&mut keys, body.translation(), extent, body.linvel());
                // A dense publication reuses one twist. Signed zeros must also
                // match the individual native state.
                if body.angvel().to_array().map(f64::to_bits) != [0; 3]
                    || velocity
                        .get_or_insert(body.linvel())
                        .to_array()
                        .map(f64::to_bits)
                        != body.linvel().to_array().map(f64::to_bits)
                {
                    return None;
                }
                let bounds = spec.collider.shape().compute_aabb(body.position());
                leaves.push((rows.len(), bounds));
                Some(AnalyticPose {
                    position: body.translation(),
                    orientation: body.rotation().to_array(),
                    epoch: Duration::ZERO,
                })
            } else {
                payloads.push(Payload {
                    handle: *handle,
                    radius: cuboid.half_extents.length(),
                    previous_center: Vector::ZERO,
                    previous_half_width: -1.,
                });
                None
            };
            rows.push(Row {
                id,
                handle: *handle,
                start,
                last_covered: 0,
                independent_motion: false,
                coordinates: coordinate_keys,
            });
        }
        if leaves.len() < MIN_DECKS || payloads.is_empty() {
            return None;
        }
        let analytic = leaves.len();
        let index = Bvh::from_iter(BvhBuildStrategy::Binned, leaves);
        Some(Self {
            index,
            coordinates,
            rows,
            payloads,
            largest_deck_radius,
            velocity: velocity?,
            elapsed: Duration::ZERO,
            analytic,
            native_rows: Vec::new(),
            retired_epochs: HashMap::new(),
            survey: 0,
            retired: 0,
            steps_since_survey: 0,
        })
    }
    fn park(row: &Row, world: &mut PhysicsWorld) {
        let body = &mut world.bodies[row.handle];
        world.colliders[body.colliders()[0]].set_enabled(false);
        body.sleep();
    }
    fn enable_row(
        row: &mut Row,
        world: &mut PhysicsWorld,
        velocity: Vector,
        coordinates: &fleet_coordinates::Coordinates,
        boundary: Option<(Duration, &mut HashMap<RigidBodyHandle, Vector>)>,
    ) -> bool {
        let Some(start) = row.start.take() else {
            return false;
        };
        let body = &mut world.bodies[row.handle];
        let h = body.colliders()[0];
        if !world.colliders[h].is_enabled() {
            // Public state at the edit barrier uses closed-form translation.
            // Native contact detection at the next interval needs the repeated
            // substep position and fat AABB phase that an uninterrupted deck
            // accumulated. A later explicit pose edit cancels this correction.
            let native = coordinates.position(row.coordinates);
            let position = if let Some((time, pending)) = boundary {
                let published = start.at(velocity, time);
                if native.to_array().map(f64::to_bits) != published.to_array().map(f64::to_bits) {
                    pending.insert(row.handle, native);
                }
                published
            } else {
                native
            };
            body.set_translation(position, true);
            body.set_linvel(velocity, true);
            world.colliders[h].set_enabled(true);
            let leaf = coordinates.leaf(
                row.coordinates,
                velocity,
                &world.colliders[h],
                body.position(),
            );
            world
                .broad_phase
                .set_aabb(&world.integration_parameters, h, leaf);
        }
        true
    }
    fn cover(&mut self, dt: Duration, world: &mut PhysicsWorld) -> bool {
        let Some(next) = self.elapsed.checked_add(dt) else {
            return false;
        };
        let seconds = self.elapsed.as_secs_f64();
        let offset = self.velocity * seconds;
        if !offset.is_finite() || offset.abs().max_element() > MAX_COORDINATE {
            return false;
        }
        let full_speed = world.integration_parameters.max_linear_velocity();
        // Include numerical error converting a world box to deck coordinates.
        let margin = self.coordinates.contact_margin() + 32. * f64::EPSILON * MAX_COORDINATE;
        self.steps_since_survey += 1;
        let survey_due = self.steps_since_survey == SURVEY_INTERVAL;
        if survey_due {
            self.steps_since_survey = 0;
            let Some(survey) = self.survey.checked_add(1) else {
                return false;
            };
            self.survey = survey;
        }
        for payload in &mut self.payloads {
            let body = &world.bodies[payload.handle];
            let center = body.translation() - offset;
            if !center.is_finite() || center.abs().max_element() > MAX_COORDINATE {
                return false;
            }
            // For the admitted discrete bodies without CCD, Rapier runs its
            // narrow phase once, at the *start* of this public step. Internal
            // solver iterations do not discover new pairs. The circumsphere
            // and margin cover that entire detection domain in every
            // orientation. Incoming free motion prewarms future geometry;
            // constraint-generated motion is observed before the next solve.
            let seconds = dt.as_secs_f64();
            let speed = if dt <= DISCRETE_PROFILE_DT {
                let acceleration =
                    world.gravity * body.gravity_scale() + body.user_force() / body.mass();
                full_speed.min(body.linvel().length() + acceleration.length() * seconds)
            } else {
                full_speed
            };
            let half_width = payload.radius + (speed + self.velocity.length()) * seconds + margin;
            if !survey_due
                && (center - payload.previous_center).abs().max_element() + half_width
                    <= payload.previous_half_width
            {
                continue;
            }
            let guard = half_width + (0.5 * payload.radius).min(MAX_QUERY_SLACK);
            let candidates = self.index.leaves(|node| {
                let a = node.aabb();
                (center.clamp(a.mins, a.maxs) - center).abs().max_element() <= guard
            });
            for k in candidates {
                let index = k as usize;
                let row = &mut self.rows[index];
                row.last_covered = self.survey;
                if Self::enable_row(row, world, self.velocity, &self.coordinates, None) {
                    self.native_rows.push(index);
                    self.retired_epochs.remove(&row.handle);
                    self.analytic -= 1;
                }
            }
            payload.previous_center = center;
            payload.previous_half_width = guard;
        }
        if survey_due && self.survey >= 2 {
            self.retire_clear_decks(world, offset);
        }
        self.coordinates.advance(
            self.velocity,
            dt.as_secs_f64(),
            next.as_secs_f64(),
            world.integration_parameters.num_solver_iterations,
        );
        self.elapsed = next;
        true
    }
    fn retire_clear_decks(&mut self, world: &mut PhysicsWorld, offset: Vector) {
        self.native_rows.retain(|&index| {
            let row = &mut self.rows[index];
            if row.independent_motion || row.last_covered >= self.survey - 1 {
                return true;
            }
            let body = &world.bodies[row.handle];
            // A long native trajectory may accumulate more roundoff than our
            // index margin permits. Keep such a deck native until rejoin.
            let leaf = self.index.leaf_node(index as u32).expect("fleet deck leaf");
            let reference = leaf.aabb().center() + offset;
            if (body.translation() - reference).abs().max_element()
                > 8. * f64::EPSILON * MAX_COORDINATE
            {
                return true;
            }
            let collider = body.colliders()[0];
            if world
                .narrow_phase
                .contact_pairs_with(collider)
                .next()
                .is_some()
            {
                // Preserve even speculative manifolds and their warm starts.
                return true;
            }
            row.start = Some(AnalyticPose {
                position: body.translation(),
                orientation: body.rotation().to_array(),
                epoch: self.elapsed,
            });
            self.retired_epochs.insert(row.handle, self.elapsed);
            Self::park(row, world);
            self.analytic += 1;
            self.retired = self.retired.saturating_add(1);
            false
        });
    }

    pub(super) fn visit_ray_bodies(
        &self,
        ray: &ray::SolidRay,
        world: &PhysicsWorld,
        mut visit: impl FnMut(RigidBodyHandle) -> Result<(), PhysicsError>,
    ) -> Option<Result<(), PhysicsError>> {
        let bounds = ray.moving_bounds(self.velocity * self.elapsed.as_secs_f64())?;
        if crate::FLEET_BASELINE_BEAM.load(std::sync::atomic::Ordering::Relaxed) {
            return Some((|| {
                for p in &self.payloads { visit(p.handle)?; }
                for &i in &self.native_rows { visit(self.rows[i].handle)?; }
                for i in self.index.intersect_aabb(&bounds) {
                    let row = &self.rows[i as usize];
                    if row.start.is_some() { visit(row.handle)?; }
                }
                Ok(())
            })());
        }
        let beam = ray.moving_bounds(Vector::ZERO)?;
        Some((|| {
            // The index follows the common deck velocity. A centered native box,
            // even when edited before the next step, lies inside its live
            // circumsphere. Keep the exact shape test at the padded boundary.
            let mut visit_native = |handle, radius| {
                let body: &RigidBody = &world.bodies[handle];
                let center = body.translation();
                if !center.is_finite()
                    || !body.rotation().is_finite()
                    || (center - center.clamp(beam.mins, beam.maxs))
                        .abs()
                        .max_element()
                        <= radius
                {
                    visit(handle)?;
                }
                Ok::<_, PhysicsError>(())
            };
            for body in &self.payloads {
                visit_native(body.handle, body.radius)?;
            }
            for &i in &self.native_rows {
                visit_native(self.rows[i].handle, self.largest_deck_radius)?;
            }
            for i in self.index.intersect_aabb(&bounds) {
                let row = &self.rows[i as usize];
                if row.start.is_some() {
                    visit(row.handle)?;
                }
            }
            Ok(())
        })())
    }

    pub(super) fn translation(
        &self,
        handle: RigidBodyHandle,
        body: &RigidBody,
        world: &PhysicsWorld,
    ) -> Vector {
        if !body.is_kinematic()
            || world.colliders[body.colliders()[0]].is_enabled()
            || self.elapsed.is_zero()
        {
            body.translation()
        } else {
            let epoch = self
                .retired_epochs
                .get(&handle)
                .copied()
                .unwrap_or_default();
            let dt = self.elapsed - epoch;
            if dt.is_zero() {
                body.translation()
            } else {
                body.translation() + self.velocity * dt.as_secs_f64()
            }
        }
    }
    pub(super) fn read(
        &self,
        world: &PhysicsWorld,
        visit: &mut dyn FnMut(&mut dyn gz_physics::BodyStateLookup),
    ) {
        let mut next = 0;
        visit(&mut |id| {
            if self.rows.get(next).is_none_or(|r| r.id != id) {
                next = self.rows.binary_search_by_key(&id, |r| r.id).ok()?;
            }
            let row = &self.rows[next];
            next += 1;
            if let Some(start) = row.start {
                Some(BodyState {
                    position: start.at(self.velocity, self.elapsed).to_array(),
                    orientation: start.orientation,
                    velocity: self.velocity.to_array(),
                    angular_velocity: [0.; 3],
                })
            } else {
                Some(fleet::native_state(&world.bodies[row.handle]))
            }
        });
    }
}
impl RapierBackend {
    pub fn fleet_isolation_stats(&self) -> Option<FleetIsolationStats> {
        let Dispatch::Active(i) = &self.fleet_dispatch else {
            return None;
        };
        Some(FleetIsolationStats {
            dynamic_bodies: i.payloads.len(),
            native_decks: i.rows.len() - i.payloads.len() - i.analytic,
            analytic_decks: i.analytic,
            retired_decks: i.retired,
        })
    }
    /// Native contact participation of a prescribed deck in an active fleet.
    pub fn fleet_deck_is_native(&self, id: BodyId) -> Option<bool> {
        let Dispatch::Active(_) = &self.fleet_dispatch else {
            return None;
        };
        let (h, spec, ..) = self.bodies.get(&id)?;
        spec.kinematic
            .then(|| self.world.colliders[self.world.bodies[*h].colliders()[0]].is_enabled())
    }

    /// Keep a single edited deck in native integration. Its old BVH leaf
    /// still belongs to the shared coordinate frame and cannot retire it.
    /// All dynamic payloads already run in this same native world, so moving
    /// the deck across the grid can create real contacts on any later step.
    pub(super) fn detach_translating_deck(&mut self, id: BodyId) {
        let Dispatch::Active(group) = &mut self.fleet_dispatch else {
            return;
        };
        let index = group
            .rows
            .binary_search_by_key(&id, |r| r.id)
            .expect("active fleet body");
        let row = &mut group.rows[index];
        if Isolation::enable_row(
            row,
            &mut self.world,
            group.velocity,
            &group.coordinates,
            Some((group.elapsed, &mut self.fleet_resume_positions)),
        ) {
            group.analytic -= 1;
            group.native_rows.push(index);
            group.retired_epochs.remove(&row.handle);
        }
        row.independent_motion = true;
    }

    /// Materialize dormant bodies at the current barrier without replacing any
    /// native body, constraint, contact manifold or public identifier.
    pub(super) fn rejoin_fleet(&mut self) {
        if let Dispatch::Active(i) = &mut self.fleet_dispatch {
            for row in &mut i.rows {
                Isolation::enable_row(
                    row,
                    &mut self.world,
                    i.velocity,
                    &i.coordinates,
                    Some((i.elapsed, &mut self.fleet_resume_positions)),
                );
            }
            self.fleet_dispatch = Dispatch::Disabled;
        }
    }
    pub fn disable_fleet_isolation(&mut self) {
        self.rejoin_fleet();
        self.fleet_dispatch = Dispatch::Disabled;
        self.allow_isolation = false;
    }
    pub(super) fn apply_deck_resumes(&mut self) {
        // An empty map is the usual case. Release a full-fleet allocation
        // after a forced transition, since it is no longer needed.
        if self.fleet_resume_positions.is_empty() {
            return;
        }
        for (handle, position) in std::mem::take(&mut self.fleet_resume_positions) {
            if let Some(body) = self.world.bodies.get_mut(handle) {
                body.set_translation(position, true);
            }
        }
    }
    pub(super) fn cover_fleet_step(&mut self, dt: Duration) {
        if matches!(self.fleet_dispatch, Dispatch::Candidate) {
            self.fleet_dispatch = Dispatch::Disabled;
            if let Some(mut i) = Isolation::build(self)
                && i.cover(dt, &mut self.world)
                && i.analytic > 0
            {
                for row in &i.rows {
                    if row.start.is_some() {
                        Isolation::park(row, &mut self.world);
                    }
                }
                self.fleet_dispatch = Dispatch::Active(Box::new(i));
            }
            return;
        }
        if let Dispatch::Active(i) = &mut self.fleet_dispatch
            && (!i.cover(dt, &mut self.world) || i.analytic == 0)
        {
            self.rejoin_fleet();
        }
    }
}

#[cfg(test)]
mod loaded_rest_tests {
    use super::*;

    fn loaded_lane() -> (RapierBackend, BodyId, BodyId) {
        let mut world = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
        let payload = world
            .spawn(BodySpec {
                shape: Shape::Box {
                    half_extents: [0.1; 3],
                },
                position: [0., 0., 0.201],
                velocity: [0.; 3],
                mass: Some(1.),
            })
            .unwrap();
        let mut first = None;
        for i in 0..MIN_DECKS {
            let deck = world
                .spawn_velocity_boundary(
                    BodySpec {
                        shape: Shape::Box {
                            half_extents: [0.5, 0.5, 0.1],
                        },
                        position: [1.1 * i as f64, 0., 0.],
                        velocity: [0.25, 0., 0.],
                        mass: Some(1.),
                    },
                    Default::default(),
                    None,
                )
                .unwrap();
            first.get_or_insert(deck);
        }
        (world, payload, first.unwrap())
    }

    #[test]
    fn native_sleep_zeros_the_world_velocity_of_a_load_on_a_moving_deck() {
        let dt = Duration::from_millis(10);
        let (mut ordinary, payload, first) = loaded_lane();
        let (mut sleeping, other, other_first) = loaded_lane();
        for _ in 0..80 {
            ordinary.step(dt).unwrap();
            sleeping.step(dt).unwrap();
        }
        let before = ordinary.state(payload).unwrap();
        assert_eq!(before, sleeping.state(other).unwrap());
        let impulse = |backend: &RapierBackend, deck: BodyId, cargo: BodyId| {
            backend
                .rigid_contact_observations()
                .into_iter()
                .find(|p| (p.a == deck && p.b == cargo) || (p.b == deck && p.a == cargo))
                .unwrap()
                .normal_impulse
        };
        assert_eq!(
            impulse(&ordinary, first, payload),
            impulse(&sleeping, other_first, other)
        );
        assert!((before.velocity[0] - 0.25).abs() < 1e-5);
        let handle = sleeping.bodies[&other].0;
        sleeping.world.bodies[handle].sleep();
        assert_eq!(sleeping.state(other).unwrap().velocity, [0.; 3]);
        ordinary.step(dt).unwrap();
        sleeping.step(dt).unwrap();
        let current = ordinary.state(payload).unwrap();
        assert!((current.position[0] - before.position[0] - 0.0025).abs() < 1e-5);
        let forced = sleeping.state(other).unwrap();
        assert!(current.position[0] > forced.position[0]);
        println!(
            "ordinary {:?} / {:?} N s; forced native sleep {:?} / {:?} N s",
            current,
            impulse(&ordinary, first, payload),
            forced,
            impulse(&sleeping, other_first, other)
        );
    }
}
