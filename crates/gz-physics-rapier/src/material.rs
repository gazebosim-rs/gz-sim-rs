// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isotropic Coulomb friction for a newly constructed local rigid collider.
use super::*;

/// Motion ownership of a newly created finite primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialLinkMotion {
    /// A mass creates a dynamic link. An absent mass creates a static link.
    Ordinary,
    Prescribed,
    Articulation,
}

/// An authored minimum-combined, isotropic Coulomb coefficient.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IsotropicFriction(f64);
impl IsotropicFriction {
    pub fn new(mu: f64) -> Result<Self, PhysicsError> {
        if mu.is_finite() && mu >= 0. {
            Ok(Self(mu))
        } else {
            Err(PhysicsError::InvalidValue)
        }
    }
    pub fn coefficient(self) -> f64 {
        self.0
    }
    pub(super) fn apply(self, collider: &mut Collider) {
        collider.set_friction(self.0);
        collider.set_friction_combine_rule(CoefficientCombineRule::Min);
    }
}
/// Normal separation speed divided by approach speed for a fresh contact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Restitution(f64);

impl Restitution {
    pub fn new(value: f64) -> Result<Self, PhysicsError> {
        if (0.0..=1.0).contains(&value) {
            Ok(Self(value))
        } else {
            Err(PhysicsError::InvalidValue)
        }
    }

    fn apply(self, shape: &mut Collider) {
        shape.set_restitution(self.0);
        shape.set_restitution_combine_rule(CoefficientCombineRule::Max);
    }
}
#[derive(Default)]
pub(super) struct PrimitiveSurface {
    pub(super) friction: Option<IsotropicFriction>,
    pub(super) restitution: Option<Restitution>,
}

impl PrimitiveSurface {
    pub(super) fn apply(self, shape: &mut Collider) {
        if let Some(friction) = self.friction {
            friction.apply(shape);
        }
        if let Some(restitution) = self.restitution {
            restitution.apply(shape);
        }
    }
}

impl RapierBackend {
    /// Set the restitution on a link's live shape and its immutable reset recipe
    /// during world construction, before any native contact history exists.
    pub fn set_initial_restitution(
        &mut self,
        id: BodyId,
        value: Restitution,
    ) -> Result<(), PhysicsError> {
        if !self.joints.construction_open() {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "restitution edits after stepping",
            });
        }
        let (handle, recipe, ..) = self.bodies.get_mut(&id).ok_or(PhysicsError::MissingBody)?;
        value.apply(&mut recipe.collider);
        for &ch in self.world.bodies[*handle].colliders() {
            value.apply(&mut self.world.colliders[ch]);
        }
        Ok(())
    }

    /// Construct a free or fixed link with its impact material before inserting its collider.
    pub fn spawn_restituting_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Inertia,
        friction: Option<IsotropicFriction>,
        restitution: Restitution,
    ) -> Result<BodyId, PhysicsError> {
        self.spawn_primitive_geometry(
            spec,
            rotation,
            Some(inertia),
            PrimitiveMode::Ordinary,
            PrimitiveSurface {
                friction,
                restitution: Some(restitution),
            },
            &[],
        )
    }

    /// Use the same material on the live and reset construction shapes.
    pub fn spawn_material_link(
        &mut self,
        spec: BodySpec,
        rotation: RotationSpec,
        inertia: Option<Inertia>,
        motion: MaterialLinkMotion,
        friction: IsotropicFriction,
    ) -> Result<BodyId, PhysicsError> {
        let mode = match motion {
            MaterialLinkMotion::Ordinary => PrimitiveMode::Ordinary,
            MaterialLinkMotion::Prescribed => PrimitiveMode::Prescribed,
            MaterialLinkMotion::Articulation => PrimitiveMode::Articulation,
        };
        self.spawn_primitive_with_surface(spec, rotation, inertia, mode, Some(friction))
    }
    pub fn spawn_material_plane(
        &mut self,
        plane: gz_physics::PlaneSpec,
        friction: IsotropicFriction,
    ) -> Result<BodyId, PhysicsError> {
        self.horizontal_halfspace(plane, Some(friction))
    }
}
