// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Contact promotion queries use the admitted native shapes and their current body frames.
use super::*;
pub(super) mod dynamic;
#[cfg(test)]
mod pair_tests;
mod support;
use rapier3d_f64::parry::bounding_volume::BoundingVolume;
use rapier3d_f64::parry::query::{ShapeCastOptions, cast_shapes, distance};

fn unsupported(capability: &'static str) -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability,
    }
}

/// Sweep a translated copy of the starting collider along the chord of its
/// bounding-sphere center. The bound on chord interpolation error is
/// max |acceleration| dt² / 8. An orientation change also sweeps the shape
/// about that center, except when the shape itself is a sphere.
fn collider_chord(
    shape: &dyn rapier3d_f64::parry::shape::Shape,
    pose: &Pose,
    pivot: Vector,
    linear: Vector,
    spin: Vector,
    dt: f64,
) -> (Vector, f64) {
    if spin == Vector::ZERO {
        return (linear, 0.);
    }
    let speed = spin.length();
    let angle = speed * dt;
    let sphere = shape.compute_bounding_sphere(pose);
    let axis = spin / speed;
    let tangent = axis.cross(sphere.center - pivot);
    let offset = tangent * angle.sin() + axis.cross(tangent) * (2. * (0.5 * angle).sin().powi(2));
    let center_error = tangent.length() * angle.powi(2) * 0.125;
    let shape_error = if shape.as_ball().is_some() {
        0.
    } else {
        sphere.radius * angle.min(2.)
    };
    (linear + offset / dt, center_error + shape_error)
}

impl RapierBackend {
    /// Native substeps in one tick of the moving two-link worker profile.
    pub fn moving_solver_substeps() -> usize {
        IntegrationParameters::default().num_solver_iterations
    }

    /// Configure a member of the split/join profile. Keeping it awake removes sleep
    /// history from isolated motion. Recomputing contact geometry every tick makes
    /// the first impact independent of the age of a speculative broad-phase pair.
    /// Rapier still carries impulses between contact solves.
    pub fn configure_moving_body(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        self.validate_moving_body(id)?;
        self.enable_moving_body(id);
        Ok(())
    }

    pub(super) fn validate_moving_body(&self, id: BodyId) -> Result<(), PhysicsError> {
        let (handle, recipe, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        let body = &self.world.bodies[*handle];
        if self.world.gravity != Vector::ZERO
            || self.world.integration_parameters.num_solver_iterations
                != Self::moving_solver_substeps()
            || self.world.integration_parameters.max_ccd_substeps > 1
            || body.additional_solver_iterations() != 0
            || body.is_ccd_enabled()
            || !body.is_dynamic()
            || recipe.collider.shape().as_ball().is_none()
            || body.is_sleeping()
        {
            return Err(unsupported(
                "moving hinge requires awake dynamic spheres in zero gravity",
            ));
        }
        Ok(())
    }

    pub(super) fn enable_moving_body(&mut self, id: BodyId) {
        self.world.integration_parameters.contact_recycling = false;
        self.keep_body_awake(id).expect("validated moving body");
    }

    /// Preserve a dynamic body's coasting velocity when its authored model disables
    /// automatic sleep. The integration settings and contact-recycling policy stay independent.
    pub fn keep_body_awake(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        let (handle, recipe, ..) = self.bodies.get_mut(&id).ok_or(PhysicsError::MissingBody)?;
        let body = &mut self.world.bodies[*handle];
        if !body.is_dynamic() {
            return Err(unsupported("sleep policy on a non-dynamic link"));
        }
        recipe.can_sleep = false;
        let activation = body.activation_mut();
        activation.normalized_linear_threshold = -1.0;
        activation.angular_threshold = -1.0;
        if body.is_sleeping() {
            body.wake_up(true);
        }
        Ok(())
    }

    /// Admit the stationary or roundoff-scale internal rates supported by prescribed transport.
    pub fn validate_prediction_motion(&self, root: BodyId) -> Result<(), PhysicsError> {
        let index = self.articulation_index(root)?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("registered tree");
        if tree.links().any(|link| {
            !link.is_root()
                && tree
                    .joint_velocity(link)
                    .iter()
                    .any(|&rate| !rate.is_finite() || rate.abs() > 1e-8)
        }) {
            return Err(PhysicsError::TransitionDeferred {
                reason: "contact promotion requires settled internal joints",
            });
        }
        Ok(())
    }

    /// Translate the assembly along the actual native root chord. Each collider gets
    /// an additional bound for its rotation about that translating root COM.
    /// A free dynamic obstacle uses its staged velocities, forces and gravity.
    /// Angular expansion vanishes for a centered ball. A possible third contact
    /// can promote early within one native step of the payload's reachable region.
    pub fn predicts_articulation_contact(
        &self,
        root: BodyId,
        obstacle: BodyId,
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        self.predicts_contact_with_motion(root, obstacle, dt, None)
    }

    pub(super) fn predicts_contact_with_motion(
        &self,
        root: BodyId,
        obstacle: BodyId,
        dt: Duration,
        prospective: Option<&articulation::SwitchPlan>,
    ) -> Result<bool, PhysicsError> {
        let index = self.articulation_index(root)?;
        let other_handle = self
            .bodies
            .get(&obstacle)
            .ok_or(PhysicsError::MissingBody)?
            .0;
        let other = &self.world.bodies[other_handle];
        if !(other.is_dynamic()
            || other.is_fixed()
            || other.body_type() == RigidBodyType::KinematicVelocityBased)
            || self.has_joint(obstacle) != Some(false)
        {
            return Err(unsupported(
                "contact promotion requires a free dynamic, prescribed or fixed obstacle link",
            ));
        }
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("registered tree");
        if tree.root().joint.kinematic {
            self.validate_prediction_motion(root)?;
        }
        let prescribed = prospective.is_some() || tree.root().joint.kinematic;
        let spinning = match prospective {
            Some(plan) => plan.transport.as_ref(),
            None => self.spinning_assemblies.get(&root),
        };
        let (other_linear, other_spin) = if other.is_fixed() {
            (Vector::ZERO, Vector::ZERO)
        } else {
            (other.linvel(), other.angvel())
        };
        let (other_linear, curve_margin) = self
            .curves
            .chord(obstacle, dt)?
            .unwrap_or((other_linear, 0.));
        let root_body = &self.world.bodies[tree.root().rigid_body_handle()];
        let root_velocity = spinning.map_or(root_body.linvel(), |motion| {
            motion.root_velocity(&self.world, Some(dt.as_secs_f64()))
        });
        // In prescribed transport every joint retains its generalized rate. During
        // construction Rapier initializes these from the admitted link twists.
        let rate = |id: usize| {
            let link = tree.link(id).expect("registered articulation link");
            let Some(parent) = link.parent_id() else {
                return 0.;
            };
            if link.joint.ndofs() == 0 {
                return 0.;
            }
            let initial = if self.joints.construction_open() {
                let parent = tree.link(parent).expect("registered parent");
                (self.world.bodies[link.rigid_body_handle()].angvel()
                    - self.world.bodies[parent.rigid_body_handle()].angvel())
                .length()
            } else {
                0.
            };
            tree.joint_velocity(link)[0].abs().max(initial)
        };
        let internal_motion = prescribed && tree.links().any(|link| rate(link.link_id()) != 0.);
        let seconds = dt.as_secs_f64();
        let cap = self.world.integration_parameters.max_linear_velocity();
        let external_bodies = other.is_dynamic()
            && (self.world.bodies.len() > tree.num_links() + 1
                || self.world.colliders.len() > self.world.bodies.len());
        let mut deflection_could_reach = false;
        let mut hit = false;
        for handle in other.colliders() {
            let obstacle = &self.world.colliders[*handle];
            let shape = obstacle.shape();
            if !shape.is_convex() {
                return Err(unsupported(
                    "contact promotion requires convex primitive colliders",
                ));
            }
            let pose =
                other.position() * obstacle.position_wrt_parent().expect("attached obstacle");
            let (other_velocity, obstacle_margin) = if other.is_dynamic() {
                dynamic::free_sweep(other, shape, &pose, self.world.gravity, cap, seconds)
            } else {
                let (v, rotation) = collider_chord(
                    shape,
                    &pose,
                    other.center_of_mass(),
                    other_linear,
                    other_spin,
                    seconds,
                );
                (v, rotation + curve_margin)
            };
            let deflected = external_bodies
                .then(|| dynamic::deflected_aabb(other, shape, &pose, cap * seconds));
            if !other_velocity.is_finite() || !obstacle_margin.is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
            let obstacle_sweep =
                dynamic::swept_aabb(shape, &pose, other_velocity, obstacle_margin, seconds);
            for link in tree.links() {
                let body = &self.world.bodies[link.rigid_body_handle()];
                for attached in body.colliders() {
                    let collider = &self.world.colliders[*attached];
                    if !collider.shape().is_convex() {
                        return Err(unsupported(
                            "contact promotion requires convex primitive colliders",
                        ));
                    }
                    if !prescribed
                        || hit
                        || !collision::can_exchange_impulses(collider, obstacle, other.body_type())
                    {
                        continue;
                    }
                    let start =
                        body.position() * collider.position_wrt_parent().expect("attached member");
                    let (source_velocity, source_margin) = collider_chord(
                        collider.shape(),
                        &start,
                        root_body.center_of_mass(),
                        root_velocity,
                        if spinning.is_some() {
                            root_body.angvel()
                        } else {
                            Vector::ZERO
                        },
                        dt.as_secs_f64(),
                    );
                    // A small nonzero hinge rate is retained bit-for-bit at
                    // demotion. Bound its effect using the path from each pivot
                    // to this shape. Segment lengths stay fixed under upstream
                    // rotations, so the sum bounds the swept lever arm.
                    let mut internal_margin = 0.;
                    if internal_motion {
                        let sphere = collider.shape().compute_bounding_sphere(&start);
                        let mut point = sphere.center;
                        let mut reach = if collider.shape().as_ball().is_some() {
                            0.
                        } else {
                            sphere.radius
                        };
                        let mut member = link;
                        while let Some(id) = member.parent_id() {
                            let parent = tree.link(id).expect("registered parent");
                            let pivot = self.world.bodies[parent.rigid_body_handle()].position()
                                * member.joint.data.local_frame1.translation;
                            reach += (point - pivot).length();
                            internal_margin +=
                                reach * (rate(member.link_id()) * dt.as_secs_f64()).min(2.);
                            point = pivot;
                            member = parent;
                        }
                    }
                    let contact_margin = self.world.integration_parameters.prediction_distance()
                        + collider.contact_skin()
                        + obstacle.contact_skin();
                    let options = ShapeCastOptions {
                        max_time_of_impact: dt.as_secs_f64(),
                        target_distance: contact_margin
                            + obstacle_margin
                            + source_margin
                            + internal_margin,
                        ..Default::default()
                    };
                    if !options.target_distance.is_finite() || !source_velocity.is_finite() {
                        return Err(PhysicsError::InvalidValue);
                    }
                    // The same conservative source sweep serves both the free-motion
                    // query and the larger third-body deflection check. Only overlapping
                    // free sweeps require a native shape distance or time-of-impact cast.
                    let source_sweep = dynamic::swept_aabb(
                        collider.shape(),
                        &start,
                        source_velocity,
                        source_margin + internal_margin + contact_margin,
                        seconds,
                    );
                    deflection_could_reach |= deflected
                        .as_ref()
                        .is_some_and(|b| source_sweep.intersects(b));
                    if !source_sweep.intersects(&obstacle_sweep) {
                        continue;
                    }
                    // A stationary pair has no ray direction. A rotating shape
                    // can close an existing gap even if its COM is receding.
                    let stationary = source_velocity == other_velocity;
                    if stationary || obstacle_margin + source_margin + internal_margin > 0. {
                        hit = distance(&start, collider.shape(), &pose, shape)
                            .map_err(|_| unsupported("unsupported contact promotion shape pair"))?
                            .distance
                            <= options.target_distance;
                        if hit || stationary {
                            continue;
                        }
                    }
                    hit = cast_shapes(
                        &start,
                        source_velocity,
                        collider.shape(),
                        &pose,
                        other_velocity,
                        shape,
                        options,
                    )
                    .map_err(|_| unsupported("unsupported contact promotion shape pair"))?
                    .is_some();
                }
            }
        }
        Ok(hit
            || (deflection_could_reach
                && self.possible_payload_deflection(index, other_handle, dt)?))
    }
}
