// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One immutable unparented half-space in a distributed native solver.
use super::*;
mod patch;
pub use patch::{GroundPatchPoint, NativeGroundPatch};

#[derive(Clone, Copy, Debug)]
pub struct NativeGroundContact {
    pub body: BodyId,
    pub active: bool,
    pub normal_impulse: f64,
}

/// One immutable oriented halfspace. The solid occupies n·(x−point) ≤ 0.
#[derive(Clone, Copy, Debug)]
pub struct NativeGroundPlane {
    pub point: [f64; 3],
    pub normal: [f64; 3],
    pub friction: f64,
}

impl NativeGroundPlane {
    pub fn new(point: [f64; 3], normal: [f64; 3], friction: f64) -> Result<Self, PhysicsError> {
        let n = Vector::from(normal);
        if !Vector::from(point).is_finite()
            || !n.is_finite()
            || (n.length_squared() - 1.).abs() > 8. * f64::EPSILON
            || !friction.is_finite()
            || friction < 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        if n == Vector::Z {
            Ok(Self {
                point: [0., 0., point[2]],
                normal: [0., 0., 1.],
                friction,
            })
        } else {
            Ok(Self {
                point,
                normal,
                friction,
            })
        }
    }

    pub fn bits(self) -> [u64; 7] {
        let [x, y, z] = self.point;
        let [nx, ny, nz] = self.normal;
        [x, y, z, nx, ny, nz, self.friction].map(f64::to_bits)
    }
}

pub(super) fn collider_at(plane: NativeGroundPlane) -> Collider {
    ColliderBuilder::new(SharedShape::halfspace(Vector::from(plane.normal)))
        .translation(Vector::from(plane.point))
        .friction(plane.friction)
        .friction_combine_rule(if plane.friction != 0. {
            CoefficientCombineRule::Min
        } else {
            CoefficientCombineRule::Average
        })
        .restitution(0.)
        .build()
}

pub(super) fn initial_surface(world: &PhysicsWorld) -> Option<NativeGroundPlane> {
    world.colliders.iter().find_map(|(_, c)| {
        if c.parent().is_some() {
            return None;
        }
        let shape = c.shape().as_halfspace()?;
        NativeGroundPlane::new(
            c.translation().to_array(),
            shape.normal.to_array(),
            c.friction(),
        )
        .ok()
    })
}

pub(super) fn ground_of(
    world: &PhysicsWorld,
) -> Result<Option<(ColliderHandle, NativeGroundPlane)>, PhysicsError> {
    let mut ground = None;
    for (handle, c) in world.colliders.iter().filter(|(_, c)| c.parent().is_none()) {
        let shape = c.shape().as_halfspace().ok_or(PhysicsError::InvalidValue)?;
        let plane = NativeGroundPlane::new(
            c.translation().to_array(),
            shape.normal.to_array(),
            c.friction(),
        )?;
        let expected = collider_at(plane);
        if ground.is_some()
            || c.position() != expected.position()
            || c.is_sensor()
            || !c.is_enabled()
            || c.restitution() != 0.
            || c.contact_skin() != expected.contact_skin()
            || c.active_hooks() != expected.active_hooks()
            || c.active_events() != expected.active_events()
            || c.collision_groups() != expected.collision_groups()
            || c.solver_groups() != expected.solver_groups()
            || c.active_collision_types() != expected.active_collision_types()
            || c.friction_combine_rule() != expected.friction_combine_rule()
            || c.restitution_combine_rule() != expected.restitution_combine_rule()
        {
            return Err(PhysicsError::InvalidValue);
        }
        ground = Some((handle, plane));
    }
    Ok(ground)
}

impl RapierBackend {
    pub fn set_partition_ground(&mut self, height: f64) -> Result<(), PhysicsError> {
        self.set_partition_ground_material(height, 0.)
    }
    pub fn set_partition_ground_material(
        &mut self,
        height: f64,
        friction: f64,
    ) -> Result<(), PhysicsError> {
        self.set_partition_plane([0., 0., height], [0., 0., 1.], friction)
    }
    pub fn set_partition_plane(
        &mut self,
        point: [f64; 3],
        normal: [f64; 3],
        friction: f64,
    ) -> Result<(), PhysicsError> {
        let plane = NativeGroundPlane::new(point, normal, friction)?;
        if !self.world.colliders.is_empty() {
            return Err(PhysicsError::InvalidValue);
        }
        self.world.colliders.insert(collider_at(plane));
        Ok(())
    }
    pub fn partition_plane(&self) -> Result<Option<NativeGroundPlane>, PhysicsError> {
        Ok(ground_of(&self.world)?.map(|(_, g)| g))
    }
    pub fn partition_ground_height(&self) -> Result<Option<f64>, PhysicsError> {
        Ok(self.partition_ground_material()?.map(|(h, _)| h))
    }
    /// Legacy elevation query. A tilted surface has no single world height.
    pub fn partition_ground_material(&self) -> Result<Option<(f64, f64)>, PhysicsError> {
        self.partition_plane()?
            .map(|g| {
                if g.normal == [0., 0., 1.] {
                    Ok((g.point[2], g.friction))
                } else {
                    Err(PhysicsError::Unsupported {
                        engine: "rapier",
                        capability: "horizontal elevation of an oriented halfspace",
                    })
                }
            })
            .transpose()
    }
    fn ground_collider(&self) -> Option<ColliderHandle> {
        self.world
            .colliders
            .iter()
            .find_map(|(id, c)| c.parent().is_none().then_some(id))
    }

    fn observe_ground_pair(
        pair: &rapier3d_f64::geometry::ContactPair,
        other: ColliderHandle,
        get_id: &impl Fn(ColliderHandle) -> Option<BodyId>,
    ) -> Option<NativeGroundContact> {
        // An unbounded half-space overlaps every broad-phase proxy. A ground
        // witness requires retained contact points or an active constraint.
        let active = pair.has_any_active_contact();
        if !active && pair.solver_manifolds().iter().all(|m| m.points.is_empty()) {
            return None;
        }
        Some(NativeGroundContact {
            body: get_id(other)?,
            active,
            normal_impulse: super::partition::normal_impulse(pair),
        })
    }

    pub fn partition_ground_contacts(&self) -> Vec<NativeGroundContact> {
        let Some(ground) = self.ground_collider() else {
            return Vec::new();
        };
        let get_id = self.contact_body_lookup();
        let mut contacts: Vec<_> = self
            .world
            .narrow_phase
            .contact_pairs_with(ground)
            .filter_map(|p| {
                let other = if p.collider1 == ground {
                    p.collider2
                } else {
                    p.collider1
                };
                Self::observe_ground_pair(p, other, &get_id)
            })
            .collect();
        contacts.sort_by_key(|p| p.body);
        contacts
    }

    /// Read both receipts in one narrow-phase pass. The caller retains storage
    /// across microsteps; topology and loads always come from this solve.
    pub fn read_partition_contacts(
        &self,
        rigid: &mut Vec<RigidContactObservation>,
        mut supports: Option<&mut Vec<NativeGroundContact>>,
    ) {
        rigid.clear();
        if let Some(supports) = &mut supports {
            supports.clear();
        }
        let mut pairs = self.world.narrow_phase.contact_pairs().peekable();
        if pairs.peek().is_none() {
            return;
        }
        let get_id = self.contact_body_lookup();
        let ground = supports.as_ref().and_then(|_| self.ground_collider());
        for pair in pairs {
            let other = if ground == Some(pair.collider1) {
                Some(pair.collider2)
            } else if ground == Some(pair.collider2) {
                Some(pair.collider1)
            } else {
                None
            };
            if let Some(other) = other {
                if let Some(s) = Self::observe_ground_pair(pair, other, &get_id) {
                    supports
                        .as_mut()
                        .expect("requested ground contacts")
                        .push(s);
                }
            } else if let Some(p) = Self::observe_rigid_pair(pair, &get_id) {
                rigid.push(p);
            }
        }
        rigid.sort_unstable_by_key(|p| (p.a, p.b));
        if let Some(supports) = supports {
            supports.sort_by_key(|s| s.body);
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/terrain_join.rs"]
mod tests;
