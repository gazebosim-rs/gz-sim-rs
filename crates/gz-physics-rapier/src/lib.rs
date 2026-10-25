// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Full-f64 rigid bodies stepped by the simulator's explicit Bevy schedule.

use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, UpdateInfo};
use rapier3d_f64::prelude::*;
use std::sync::Arc;
use std::{collections::BTreeMap, time::Duration};

mod initial;
pub use initial::{InitialPhysicsState, PreparedPhysicsReset};

#[derive(Debug, Clone, Copy)]
pub enum Shape {
    Sphere { radius: f64 },
    Box { half_extents: [f64; 3] },
}

/// Initial state, in meters, kilograms, and meters per second. `None` mass is fixed.
#[derive(Debug, Clone, Copy)]
pub struct BodySpec {
    pub shape: Shape,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub mass: Option<f64>,
}

/// A link's inertial frame, relative to its body frame. Tensor order is xx,yy,zz,xy,xz,yz.
#[derive(Debug, Clone, Copy)]
pub struct Inertia {
    pub center: [f64; 3],
    pub orientation: [f64; 4],
    pub tensor: [f64; 6],
}

/// Rotational state. Quaternions use [x,y,z,w]; angular velocity is in world axes.
#[derive(Debug, Clone, Copy)]
pub struct RotationSpec {
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

impl Default for RotationSpec {
    fn default() -> Self {
        Self {
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0; 3],
        }
    }
}

/// A backend-local stable identifier. Always pair it with its owning world.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyId(u64);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    /// Body origin in world axes.
    pub position: [f64; 3],
    /// Center-of-mass velocity in world axes; differs from origin velocity for offset COM.
    pub velocity: [f64; 3],
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PhysicsReset;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum PhysicsError {
    #[error("body or gravity contains an invalid numeric value")]
    InvalidValue,
    #[error("body identifier space exhausted")]
    IdOverflow,
    #[error("Rapier is already installed in this world")]
    AlreadyInstalled,
    #[error("body identifier is not present in this backend")]
    MissingBody,
    #[error("initial physics state belongs to a different backend")]
    ForeignInitialState,
}

#[derive(Resource)]
pub struct RapierBackend {
    owner: Arc<()>,
    world: PhysicsWorld,
    bodies: BTreeMap<BodyId, (RigidBodyHandle, BodySpec, RotationSpec, Option<Inertia>)>,
    next_id: u64,
}

fn rotation(value: [f64; 4]) -> Result<Rotation, PhysicsError> {
    let rotation = Rotation::from_xyzw(value[0], value[1], value[2], value[3]);
    if !rotation.is_finite() || (rotation.length_squared() - 1.0).abs() > 1e-12 {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(rotation)
}

fn inertia_properties(inertia: Inertia, mass: f64) -> Result<MassProperties, PhysicsError> {
    if !finite(inertia.center)
        || !inertia
            .tensor
            .into_iter()
            .map(|v| v * v)
            .sum::<f64>()
            .is_finite()
    {
        return Err(PhysicsError::InvalidValue);
    }
    let q = rotation(inertia.orientation)?;
    let [xx, yy, zz, xy, xz, yz] = inertia.tensor;
    let matrix = Matrix::from_cols_array(&[xx, xy, xz, xy, yy, yz, xz, yz, zz]);
    let r = Matrix::from_quat(q);
    let properties = MassProperties::with_inertia_matrix(
        Vector::from(inertia.center),
        mass,
        r * matrix * r.transpose(),
    );
    let inverse = properties.inv_principal_inertia.to_array();
    if !inverse.into_iter().all(|v| v.is_normal() && v > 0.0) {
        return Err(PhysicsError::InvalidValue);
    }
    let principal = inverse.map(|v| 1.0 / v);
    let sum: f64 = principal.into_iter().sum();
    if !sum.is_finite() || principal.into_iter().any(|v| v > sum - v + sum * 1e-12) {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(properties)
}

fn finite(values: [f64; 3]) -> bool {
    values.into_iter().all(f64::is_finite)
}

fn collider(shape: Shape) -> Result<ColliderBuilder, PhysicsError> {
    match shape {
        Shape::Sphere { radius } if radius.is_finite() && radius > 0.0 => {
            Ok(ColliderBuilder::ball(radius))
        }
        Shape::Box {
            half_extents: [x, y, z],
        } if [x, y, z].into_iter().all(|v| v.is_finite() && v > 0.0) => {
            Ok(ColliderBuilder::cuboid(x, y, z))
        }
        _ => Err(PhysicsError::InvalidValue),
    }
}

fn validated_collider(spec: BodySpec, orientation: Rotation) -> Result<Collider, PhysicsError> {
    let shape = collider(spec.shape)?.mass(spec.mass.unwrap_or(1.0)).build();
    let mass = shape.mass_properties();
    let aabb = shape
        .shape()
        .compute_aabb(&Pose::from_parts(Vector::from(spec.position), orientation));
    let positive = |v: f64| v.is_normal() && v > 0.0;
    if !positive(mass.inv_mass)
        || !mass
            .inv_principal_inertia
            .to_array()
            .into_iter()
            .all(positive)
        || !finite(aabb.mins.to_array())
        || !finite(aabb.maxs.to_array())
        || !(aabb.maxs - aabb.mins).to_array().into_iter().all(positive)
    {
        return Err(PhysicsError::InvalidValue);
    }
    Ok(shape)
}

fn insert(
    world: &mut PhysicsWorld,
    spec: BodySpec,
    rotational: RotationSpec,
    mut shape: Collider,
    inertia: Option<MassProperties>,
) -> RigidBodyHandle {
    let body = if spec.mass.is_some() {
        RigidBodyBuilder::dynamic()
    } else {
        RigidBodyBuilder::fixed()
    };
    let mut body = body
        .pose(Pose::from_parts(
            Vector::from(spec.position),
            Rotation::from_xyzw(
                rotational.orientation[0],
                rotational.orientation[1],
                rotational.orientation[2],
                rotational.orientation[3],
            ),
        ))
        .linvel(Vector::from(spec.velocity))
        .angvel(Vector::from(rotational.angular_velocity));
    if let Some(inertia) = inertia {
        shape.set_density(0.0);
        body = body.additional_mass_properties(inertia);
    }
    let handle = world.insert(body, shape).0;
    // Impulses may arrive before the first step; initialize effective mass now.
    world.bodies[handle].recompute_mass_properties_from_colliders(&world.colliders);
    handle
}

impl RapierBackend {
    /// Gravity uses Gazebo's Z-up coordinate convention; no implicit axis conversion.
    pub fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        if !finite(gravity) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            owner: Arc::new(()),
            world: PhysicsWorld {
                gravity: Vector::from(gravity),
                ..PhysicsWorld::new()
            },
            bodies: BTreeMap::new(),
            next_id: 0,
        })
    }

    pub fn spawn(&mut self, spec: BodySpec) -> Result<BodyId, PhysicsError> {
        self.spawn_link(spec, RotationSpec::default(), None)
    }

    /// Creates one centered collider with optional explicit link inertia.
    pub fn spawn_link(
        &mut self,
        spec: BodySpec,
        rotational: RotationSpec,
        inertia: Option<Inertia>,
    ) -> Result<BodyId, PhysicsError> {
        if !finite(spec.position)
            || !finite(spec.velocity)
            || spec.mass.is_some_and(|v| !v.is_finite() || v <= 0.0)
            || (spec.mass.is_none() && spec.velocity != [0.0; 3])
            || !finite(rotational.angular_velocity)
            || (spec.mass.is_none() && rotational.angular_velocity != [0.0; 3])
        {
            return Err(PhysicsError::InvalidValue);
        }
        let shape = validated_collider(spec, rotation(rotational.orientation)?)?;
        let properties = inertia
            .map(|value| inertia_properties(value, spec.mass.unwrap_or(1.0)))
            .transpose()?;
        let next = self
            .next_id
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        let id = BodyId(self.next_id);
        let handle = insert(&mut self.world, spec, rotational, shape, properties);
        self.bodies.insert(id, (handle, spec, rotational, inertia));
        self.next_id = next;
        Ok(id)
    }

    pub fn state(&self, id: BodyId) -> Option<BodyState> {
        let (handle, ..) = self.bodies.get(&id)?;
        let body = &self.world.bodies[*handle];
        Some(BodyState {
            position: body.translation().to_array(),
            velocity: body.linvel().to_array(),
            orientation: body.rotation().to_array(),
            angular_velocity: body.angvel().to_array(),
        })
    }

    pub fn remove(&mut self, id: BodyId) -> bool {
        if let Some((handle, ..)) = self.bodies.remove(&id) {
            self.world.remove_body(handle);
            true
        } else {
            false
        }
    }

    /// Applies linear and angular impulses in world axes. Fixed bodies remain fixed.
    pub fn apply_impulse(
        &mut self,
        id: BodyId,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), PhysicsError> {
        if !finite(linear) || !finite(angular) {
            return Err(PhysicsError::InvalidValue);
        }
        let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        let body = &mut self.world.bodies[*handle];
        let mass = body.mass_properties();
        let next_linear = body.linvel() + Vector::from(linear) * mass.effective_inv_mass;
        let next_angular = body.angvel() + mass.effective_world_inv_inertia * Vector::from(angular);
        if !finite(next_linear.to_array()) || !finite(next_angular.to_array()) {
            return Err(PhysicsError::InvalidValue);
        }
        body.apply_impulse(Vector::from(linear), true);
        body.apply_torque_impulse(Vector::from(angular), true);
        Ok(())
    }

    /// Restores remaining bodies to their spawn state. Removed bodies stay removed.
    pub fn reset(&mut self) {
        self.world = PhysicsWorld {
            gravity: self.world.gravity,
            integration_parameters: self.world.integration_parameters,
            ..PhysicsWorld::new()
        };
        for (handle, spec, rotational, inertia) in self.bodies.values_mut() {
            *handle = initial::insert_initial(&mut self.world, *spec, *rotational, *inertia);
        }
    }

    fn step(&mut self, dt: Duration) {
        if !dt.is_zero() {
            self.world.integration_parameters.dt = dt.as_secs_f64();
            self.world.step();
        }
    }
}

/// Installs one backend into the simulation's physics and reset phases.
pub fn install(sim: &mut Simulation, backend: RapierBackend) -> Result<(), PhysicsError> {
    if sim.world().contains_resource::<RapierBackend>()
        || !sim.register_extension::<RapierBackend>()
    {
        return Err(PhysicsError::AlreadyInstalled);
    }
    sim.world_mut().insert_resource(backend);
    sim.add_systems(
        Phase::PhysicsStep,
        |time: Res<UpdateInfo>, mut backend: ResMut<RapierBackend>| {
            backend.step(time.dt);
        },
    );
    sim.add_systems(
        Phase::Reset,
        (|mut backend: ResMut<RapierBackend>| {
            backend.reset();
        })
        .in_set(PhysicsReset),
    );
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/ids.rs"]
mod ids;
