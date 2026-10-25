// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

type Construction = (BodySpec, RotationSpec, Option<Inertia>);

/// Remaining bodies' spawn states and engine settings, tied to this backend instance.
/// Capture before stepping to define an initial scene; this is not a runtime snapshot.
#[derive(Clone)]
pub struct InitialPhysicsState {
    owner: Arc<()>,
    gravity: Vector,
    parameters: IntegrationParameters,
    bodies: BTreeMap<BodyId, Construction>,
    next_id: u64,
}

/// Fully constructed reset state, ready to commit without rebuilding the solver.
pub struct PreparedPhysicsReset {
    owner: Arc<()>,
    world: PhysicsWorld,
    bodies: BTreeMap<BodyId, (RigidBodyHandle, BodySpec, RotationSpec, Option<Inertia>)>,
    next_id: u64,
}

impl RapierBackend {
    /// Tests backend ownership without constructing a replacement solver.
    pub fn owns_initial_state(&self, initial: &InitialPhysicsState) -> bool {
        Arc::ptr_eq(&self.owner, &initial.owner)
    }

    pub fn initial_state(&self) -> InitialPhysicsState {
        InitialPhysicsState {
            owner: self.owner.clone(),
            gravity: self.world.gravity,
            parameters: self.world.integration_parameters,
            bodies: self
                .bodies
                .iter()
                .map(|(&id, &(_, spec, rotation, inertia))| (id, (spec, rotation, inertia)))
                .collect(),
            next_id: self.next_id,
        }
    }

    /// Builds an independent solver state before any scene or backend mutation.
    pub fn prepare_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        if !self.owns_initial_state(initial) {
            return Err(PhysicsError::ForeignInitialState);
        }
        let mut world = PhysicsWorld {
            gravity: initial.gravity,
            integration_parameters: initial.parameters,
            ..PhysicsWorld::new()
        };
        let mut bodies = BTreeMap::new();
        for (&id, &(spec, rotational, inertia)) in &initial.bodies {
            let handle = insert_initial(&mut world, spec, rotational, inertia);
            bodies.insert(id, (handle, spec, rotational, inertia));
        }
        Ok(PreparedPhysicsReset {
            owner: self.owner.clone(),
            world,
            bodies,
            next_id: initial.next_id,
        })
    }

    /// Restores initial IDs and retires runtime bodies. Later allocations remain monotonic,
    /// including bodies allocated after preparation. A foreign state leaves this unchanged.
    pub fn apply_initial(&mut self, prepared: PreparedPhysicsReset) -> Result<(), PhysicsError> {
        if !Arc::ptr_eq(&self.owner, &prepared.owner) {
            return Err(PhysicsError::ForeignInitialState);
        }
        self.pending_wrenches.clear();
        self.world = prepared.world;
        self.bodies = prepared.bodies;
        self.next_id = self.next_id.max(prepared.next_id);
        Ok(())
    }
}

pub(super) fn insert_initial(
    world: &mut PhysicsWorld,
    spec: BodySpec,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
) -> RigidBodyHandle {
    // Both callers use immutable construction records validated before insertion.
    insert(
        world,
        spec,
        rotational,
        validated_collider(
            spec,
            rotation(rotational.orientation).expect("validated rotation"),
        )
        .expect("validated shape"),
        inertia.map(|value| {
            inertia_properties(value, spec.mass.unwrap_or(1.0)).expect("validated inertia")
        }),
    )
}

#[cfg(test)]
#[path = "../tests/private/initial.rs"]
mod tests;
