// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exchange finite-mass rigid receiver state with the deformable solver.
use super::*;
mod capsule_pair;
use rapier3d_f64::parry::bounding_volume::BoundingVolume;
use rapier3d_f64::parry::query::{ShapeCastOptions, cast_shapes};

/// COM-centered principal mass properties in rigid link coordinates.
#[derive(Clone, Copy, Debug)]
pub struct SoftRigidInertia {
    pub inverse_mass: f64,
    pub inverse_principal_inertia: [f64; 3],
    /// Rotation from principal coordinates to link coordinates, xyzw.
    pub principal_orientation: [f64; 4],
}

/// Backwards-compatible principal inertia of a spherical receiver.
pub type SoftSphereInertia = SoftRigidInertia;

impl SoftRigidInertia {
    /// Scalar inverse moment when the native principal moments are isotropic.
    pub fn isotropic_inverse_inertia(self) -> Option<f64> {
        let inv = Vector::from_array(self.inverse_principal_inertia);
        (inv.max_element() - inv.min_element() <= 1e-12 * inv.min_element()).then_some(inv.x)
    }
}

#[derive(Clone, Copy, PartialEq)]
enum ReceiverShape {
    Sphere,
    Cuboid,
    Capsule,
}

impl RapierBackend {
    /// Native inverse mass and scalar inverse moment about the COM. Finite
    /// sphere feedback requires a free, isotropic, centered dynamic body.
    pub fn soft_sphere_mobility(&self, id: BodyId) -> Result<(f64, f64), PhysicsError> {
        let props = self.soft_sphere_inertia(id)?;
        let inverse = props
            .isotropic_inverse_inertia()
            .ok_or(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "scalar soft sphere mobility requires isotropic inertia",
            })?;
        Ok((props.inverse_mass, inverse))
    }

    /// The instantaneous known uniform world acceleration of a free COM. A native
    /// contact, velocity discontinuity, or damping invalidates a constant-force
    /// prediction. The scene compares it with the post-step native linear velocity.
    pub fn soft_linear_acceleration(&self, id: BodyId) -> Result<Option<[f64; 3]>, PhysicsError> {
        let (h, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        let body = &self.world.bodies[*h];
        if !body.is_dynamic()
            || !body.is_enabled()
            || body.is_sleeping()
            || body.linear_damping() != 0.
        {
            return Ok(None);
        }
        let a = self.world.gravity * body.gravity_scale()
            + body.user_force() * body.mass_properties().local_mprops.inv_mass;
        if !a.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Some(a.to_array()))
    }
    /// Native full inertia of an unconstrained, COM-centered sphere receiver.
    pub fn soft_sphere_inertia(&self, id: BodyId) -> Result<SoftSphereInertia, PhysicsError> {
        self.receiver_inertia(id, ReceiverShape::Sphere, false)
    }

    /// An isolated centered cuboid can accept a full angular pose correction.
    /// Simultaneous contact with other native colliders needs a joint solve.
    pub fn soft_cuboid_inertia(&self, id: BodyId) -> Result<SoftRigidInertia, PhysicsError> {
        self.receiver_inertia(id, ReceiverShape::Cuboid, false)
    }

    /// Native tensor for a single free capsule centered on the link and aligned with local Z.
    pub fn soft_capsule_inertia(&self, id: BodyId) -> Result<SoftRigidInertia, PhysicsError> {
        self.receiver_inertia(id, ReceiverShape::Capsule, false)
    }

    fn receiver_inertia(
        &self,
        id: BodyId,
        shape: ReceiverShape,
        enrolled_pair: bool,
    ) -> Result<SoftRigidInertia, PhysicsError> {
        let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        let body = &self.world.bodies[*handle];
        let props = &body.mass_properties().local_mprops;
        let inv = props.inv_principal_inertia;
        let eligible_geometry = match body.colliders() {
            [handle] => {
                let collider = &self.world.colliders[*handle];
                let centered = collider.position_wrt_parent().is_some_and(|p| {
                    p.translation == Vector::ZERO
                        && (shape == ReceiverShape::Sphere || p.rotation == Rotation::IDENTITY)
                });
                let eligible_shape = match shape {
                    ReceiverShape::Sphere => collider.shape().as_ball().is_some(),
                    ReceiverShape::Cuboid => {
                        self.world.colliders.len() == 1 && collider.shape().as_cuboid().is_some()
                    }
                    ReceiverShape::Capsule => {
                        (self.world.colliders.len() == 1 || enrolled_pair)
                            && collider.shape().as_capsule().is_some_and(|c| {
                                c.segment.a.x == 0.
                                    && c.segment.a.y == 0.
                                    && c.segment.a == -c.segment.b
                                    && c.segment.a.z != 0.
                            })
                    }
                };
                centered && eligible_shape
            }
            _ => false,
        };
        if self.has_joint(id) != Some(false)
            || !body.is_dynamic()
            || !body.is_enabled()
            || !body.locked_axes().is_empty()
            || props.local_com != Vector::ZERO
            || !(props.inv_mass.is_normal()
                && props.inv_mass > 0.
                && inv.is_finite()
                && inv.min_element() > 0.
                && inv.recip().is_finite())
            || !eligible_geometry
        {
            return Err(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: match shape {
                    ReceiverShape::Cuboid => {
                        "two-way soft cuboid requires an isolated single free centered box"
                    }
                    ReceiverShape::Capsule => {
                        "two-way soft capsule requires an isolated single free centered Z capsule"
                    }
                    ReceiverShape::Sphere => {
                        "two-way soft boundary requires a free centered sphere with finite principal inertia"
                    }
                },
            });
        }
        Ok(SoftRigidInertia {
            inverse_mass: props.inv_mass,
            inverse_principal_inertia: inv.to_array(),
            principal_orientation: props.principal_inertia_local_frame.to_array(),
        })
    }

    /// Commit an isolated cuboid's coupled pose and velocity together.
    pub fn commit_soft_cuboid(&mut self, id: BodyId, state: BodyState) -> Result<(), PhysicsError> {
        self.soft_cuboid_inertia(id)?;
        self.set_body_state(id, state)
    }

    /// Commit the complete coupled motion of an isolated capsule.
    pub fn commit_soft_capsule(
        &mut self,
        id: BodyId,
        state: BodyState,
    ) -> Result<(), PhysicsError> {
        self.soft_capsule_inertia(id)?;
        self.set_body_state(id, state)
    }

    /// Validate all corrections against the current rigid world before writing.
    /// A contact requiring a simultaneous third-body solve faults the step.
    pub fn commit_soft_spheres(
        &mut self,
        updates: &[(BodyId, BodyState)],
    ) -> Result<(), PhysicsError> {
        let mut motions = Vec::with_capacity(updates.len());
        for &(id, state) in updates {
            self.soft_sphere_inertia(id)?;
            let (handle, next) = body_state::prepare(self, id, state)?;
            let body = &self.world.bodies[handle];
            if motions.iter().any(|(h, _)| *h == handle) {
                return Err(PhysicsError::InvalidValue);
            }
            motions.push((handle, next.translation - body.translation()));
        }
        let simultaneous = || PhysicsError::Unsupported {
            engine: "multifidelity",
            capability: "simultaneous soft and third-body rigid contact on a finite sphere",
        };
        for &(handle, delta) in &motions {
            let body = &self.world.bodies[handle];
            let own = body.colliders()[0];
            if self
                .world
                .narrow_phase
                .contact_pairs_with(own)
                // Proximity alone may create a speculative solver contact while
                // separated jaws still have room for a sheet between them.
                .flat_map(|p| p.solver_manifolds())
                .flat_map(|m| m.points.iter())
                .any(|p| p.data.impulse != 0.0)
            {
                return Err(simultaneous());
            }
            let sphere = &self.world.colliders[own];
            let mut end = *body.position();
            end.translation += delta;
            let bounds = sphere
                .shape()
                .compute_aabb(body.position())
                .merged(&sphere.shape().compute_aabb(&end));
            for (other_handle, other) in self.world.colliders.iter() {
                if other_handle == own {
                    continue;
                }
                let parent = other.parent();
                let pose = parent.map_or_else(
                    || *other.position(),
                    |id| {
                        self.world.bodies[id].position()
                            * other.position_wrt_parent().expect("parented collider")
                    },
                );
                let other_delta = motions
                    .iter()
                    .find(|(h, _)| Some(*h) == parent)
                    .map_or(Vector::ZERO, |(_, d)| *d);
                let mut other_end = pose;
                other_end.translation += other_delta;
                let other_bounds = other
                    .shape()
                    .compute_aabb(&pose)
                    .merged(&other.shape().compute_aabb(&other_end));
                if !bounds.intersects(&other_bounds) {
                    continue;
                }
                if cast_shapes(
                    body.position(),
                    delta,
                    sphere.shape(),
                    &pose,
                    other_delta,
                    other.shape(),
                    ShapeCastOptions::with_max_time_of_impact(1.),
                )
                .map_err(|_| simultaneous())?
                .is_some()
                {
                    return Err(simultaneous());
                }
            }
        }
        for &(id, state) in updates {
            self.set_body_state(id, state)?;
        }
        Ok(())
    }
}
