// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Retain the exact SDF part transforms on one native rigid link.
use super::*;
use gz_physics::CollisionSpec;

pub(super) fn collider_for_link(
    parts: &[CollisionSpec],
    spec: BodySpec,
    orientation: Rotation,
    center_of_mass: Vector,
) -> Result<Collider, PhysicsError> {
    let body_pose = Pose::from_parts(Vector::from(spec.position), orientation);
    let mut pieces = Vec::with_capacity(parts.len());
    let mut low = Vector::splat(f64::INFINITY);
    let mut high = Vector::splat(f64::NEG_INFINITY);
    for part in parts {
        let shape = collider(part.shape)?.build();
        let local = Pose::from_parts(Vector::from(part.position), rotation(part.orientation)?)
            * *shape.position();
        let aabb = shape.shape().compute_aabb(&(body_pose * local));
        if !local.translation.is_finite()
            || !aabb.mins.is_finite()
            || !aabb.maxs.is_finite()
            || !(aabb.maxs - aabb.mins)
                .to_array()
                .into_iter()
                .all(|v| v.is_normal() && v > 0.)
        {
            return Err(PhysicsError::InvalidValue);
        }
        low = low.min(aabb.mins);
        high = high.max(aabb.maxs);
        if !high.distance(low).is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        pieces.push((local, shape.shared_shape().clone()));
    }
    // An offset single primitive must also retain its compound wrapper. Fast
    // paths for a centered box or sphere inspect the native shape type and
    // derive bounds from the link origin.
    let collider = ColliderBuilder::compound(pieces).density(0.).build();
    let sphere = collider.shape().compute_local_bounding_sphere();
    let sweep_radius = sphere.center.distance(center_of_mass) + sphere.radius;
    if !sweep_radius.is_finite() || !(sweep_radius * sweep_radius).is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(collider)
}

impl RapierBackend {
    /// Create one link with offset primitive collision geometry and an authored inertia.
    pub fn spawn_compound_link(
        &mut self,
        parts: &[CollisionSpec],
        body: BodySpec,
        rotation: RotationSpec,
        inertia: Inertia,
        motion: MaterialLinkMotion,
        friction: Option<IsotropicFriction>,
    ) -> Result<BodyId, PhysicsError> {
        if parts.is_empty() {
            return Err(PhysicsError::InvalidValue);
        }
        let mode = match motion {
            MaterialLinkMotion::Ordinary => PrimitiveMode::Ordinary,
            MaterialLinkMotion::Prescribed => PrimitiveMode::Prescribed,
            MaterialLinkMotion::Articulation => PrimitiveMode::Articulation,
        };
        self.spawn_primitive_geometry(body, rotation, Some(inertia), mode, friction, parts)
    }

    /// Set Rapier's global geometric slop in world meters. It bounds the inset
    /// used by the convex TOI sweep. Other contact uses share this tolerance.
    pub fn set_contact_linear_slop(&mut self, meters: f64) -> Result<(), PhysicsError> {
        let normalized = meters / self.world.integration_parameters.length_unit;
        if !normalized.is_finite() || normalized <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        self.world
            .integration_parameters
            .normalized_allowed_linear_error = normalized;
        Ok(())
    }

    /// Persist continuous collision detection through both native and scene resets.
    pub fn enable_continuous_collision(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        let joined = self.has_joint(id).ok_or(PhysicsError::MissingBody)?;
        let (handle, spec, ..) = self.bodies.get_mut(&id).expect("registered body");
        if spec.mass.is_none() || spec.kinematic || spec.articulation.is_some() || joined {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "continuous collision detection on an independent dynamic link",
            });
        }
        spec.continuous_collision = true;
        self.world.bodies[*handle].enable_ccd(true);
        // A live welded carrier reads the free peer's CCD flag before its
        // next solve. Keep unrelated contact patches and impulses warm.
        Ok(())
    }
}

/// A pair of material points retained by the geometric contact manifold. World
/// points use the current collider transforms. For a primitive compound, the
/// subshape index is the zero-based collision declaration on the link.
#[derive(Debug, Clone, Copy)]
pub struct RigidSurfaceWitness {
    pub body: BodyId,
    pub subshape: u32,
    pub world_point: [f64; 3],
}

impl RapierBackend {
    /// Unclustered geometric surface witnesses for active body pairs. These may
    /// include speculative separated points. Solver impulse and frozen lever
    /// arms are reported separately by `rigid_contact_patches`.
    pub fn rigid_surface_witnesses(&self) -> Vec<[RigidSurfaceWitness; 2]> {
        let id = self.contact_body_lookup();
        let mut result = Vec::new();
        for pair in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact())
        {
            let (Some(a), Some(b)) = (id(pair.collider1), id(pair.collider2)) else {
                continue;
            };
            let poses = [pair.collider1, pair.collider2].map(|h| {
                let c = &self.world.colliders[h];
                c.parent().map_or_else(
                    || *c.position(),
                    |parent| {
                        *self.world.bodies[parent].position()
                            * *c.position_wrt_parent().expect("attached collider")
                    },
                )
            });
            for manifold in pair.manifolds() {
                let child_pose = [manifold.subshape_pos1(), manifold.subshape_pos2()];
                let [p1, p2] = std::array::from_fn(|i| {
                    child_pose[i].map_or(poses[i], |child| poses[i] * *child)
                });
                for point in &manifold.points {
                    let first = RigidSurfaceWitness {
                        body: a,
                        subshape: manifold.subshape1,
                        world_point: (p1 * point.local_p1).to_array(),
                    };
                    let second = RigidSurfaceWitness {
                        body: b,
                        subshape: manifold.subshape2,
                        world_point: (p2 * point.local_p2).to_array(),
                    };
                    result.push(if a <= b {
                        [first, second]
                    } else {
                        [second, first]
                    });
                }
            }
        }
        result.sort_by_key(|[a, b]| (a.body, b.body, a.subshape, b.subshape));
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn free_peer(b: &mut RapierBackend, x: f64) -> BodyId {
        b.spawn(BodySpec {
            position: [x, 0., 0.],
            velocity: [0.; 3],
            mass: Some(1.),
            shape: Shape::Sphere { radius: 0.1 },
        })
        .unwrap()
    }

    #[test]
    fn native_ccd_and_impulse_joint_admission_agree_in_both_construction_orders() {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let parent = free_peer(&mut b, 0.);
        let child = free_peer(&mut b, 0.3);
        let joint = RobotJoint::Prismatic {
            axis: [1., 0., 0.],
            limits: None,
        };
        b.add_robot_joint(Some(parent), child, [0.15, 0., 0.], joint)
            .unwrap();
        assert!(b.enable_continuous_collision(parent).is_err());
        assert!(b.enable_continuous_collision(child).is_err());
        for ccd_on_parent in [true, false] {
            let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
            let parent = free_peer(&mut b, 0.);
            let child = free_peer(&mut b, 0.3);
            b.enable_continuous_collision(if ccd_on_parent { parent } else { child })
                .unwrap();
            assert!(
                b.add_robot_joint(Some(parent), child, [0.15, 0., 0.], joint)
                    .is_err()
            );
            assert_eq!(b.is_jointed(child), Some(false));
        }
    }
    #[test]
    fn finite_part_bounds_cannot_create_an_unrepresentable_ccd_radius() {
        // Large finite worlds can make each AABB look representable while
        // Parry's squared sweep radius overflows. Check before BVH creation.
        let shape = Shape::Sphere { radius: 1e145 };
        let spec = BodySpec {
            shape,
            position: [0.; 3],
            velocity: [0.; 3],
            mass: Some(1.),
        };
        let parts = [-1e155, 1e155].map(|x| CollisionSpec {
            shape,
            position: [x, 0., 0.],
            orientation: [0., 0., 0., 1.],
        });
        assert!(collider_for_link(&parts, spec, Rotation::IDENTITY, Vector::ZERO).is_err());
        let part = CollisionSpec {
            shape: Shape::Sphere { radius: 0.1 },
            position: [0.25, 0., 0.],
            orientation: [0., 0., 0., 1.],
        };
        assert!(
            collider_for_link(&[part], spec, Rotation::IDENTITY, Vector::splat(1e155)).is_err()
        );
    }
}
