//! Full-f64 rigid bodies stepped by the simulator's explicit Bevy schedule.

use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, UpdateInfo};
use rapier3d_f64::prelude::*;
use std::{collections::BTreeMap, time::Duration};

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

/// A backend-local stable identifier. Always pair it with its owning world.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyId(u64);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum PhysicsError {
    #[error("body or gravity contains an invalid numeric value")]
    InvalidValue,
    #[error("body identifier space exhausted")]
    IdOverflow,
    #[error("Rapier is already installed in this world")]
    AlreadyInstalled,
}

#[derive(Resource)]
pub struct RapierBackend {
    world: PhysicsWorld,
    bodies: BTreeMap<BodyId, (RigidBodyHandle, BodySpec)>,
    next_id: u64,
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

fn validated_collider(spec: BodySpec) -> Result<Collider, PhysicsError> {
    let shape = collider(spec.shape)?.mass(spec.mass.unwrap_or(1.0)).build();
    let mass = shape.mass_properties();
    let aabb = shape
        .shape()
        .compute_aabb(&Pose::from_translation(Vector::from(spec.position)));
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

fn insert(world: &mut PhysicsWorld, spec: BodySpec, shape: Collider) -> RigidBodyHandle {
    let body = if spec.mass.is_some() {
        RigidBodyBuilder::dynamic()
    } else {
        RigidBodyBuilder::fixed()
    };
    let body = body
        .translation(Vector::from(spec.position))
        .linvel(Vector::from(spec.velocity));
    world.insert(body, shape).0
}

impl RapierBackend {
    /// Gravity uses Gazebo's Z-up coordinate convention; no implicit axis conversion.
    pub fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        if !finite(gravity) {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            world: PhysicsWorld {
                gravity: Vector::from(gravity),
                ..PhysicsWorld::new()
            },
            bodies: BTreeMap::new(),
            next_id: 0,
        })
    }

    pub fn spawn(&mut self, spec: BodySpec) -> Result<BodyId, PhysicsError> {
        if !finite(spec.position)
            || !finite(spec.velocity)
            || spec.mass.is_some_and(|v| !v.is_finite() || v <= 0.0)
            || (spec.mass.is_none() && spec.velocity != [0.0; 3])
        {
            return Err(PhysicsError::InvalidValue);
        }
        let shape = validated_collider(spec)?;
        let next = self
            .next_id
            .checked_add(1)
            .ok_or(PhysicsError::IdOverflow)?;
        let id = BodyId(self.next_id);
        let handle = insert(&mut self.world, spec, shape);
        self.bodies.insert(id, (handle, spec));
        self.next_id = next;
        Ok(id)
    }

    pub fn state(&self, id: BodyId) -> Option<BodyState> {
        let (handle, _) = self.bodies.get(&id)?;
        let body = &self.world.bodies[*handle];
        Some(BodyState {
            position: body.translation().to_array(),
            velocity: body.linvel().to_array(),
        })
    }

    pub fn remove(&mut self, id: BodyId) -> bool {
        if let Some((handle, _)) = self.bodies.remove(&id) {
            self.world.remove_body(handle);
            true
        } else {
            false
        }
    }

    /// Restores remaining bodies to their spawn state. Removed bodies stay removed.
    pub fn reset(&mut self) {
        self.world = PhysicsWorld {
            gravity: self.world.gravity,
            ..PhysicsWorld::new()
        };
        for (handle, spec) in self.bodies.values_mut() {
            // Specs are validated before insertion and remain private afterward.
            *handle = insert(
                &mut self.world,
                *spec,
                validated_collider(*spec).expect("validated shape"),
            );
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
    sim.add_systems(Phase::Reset, |mut backend: ResMut<RapierBackend>| {
        backend.reset();
    });
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/ids.rs"]
mod ids;
