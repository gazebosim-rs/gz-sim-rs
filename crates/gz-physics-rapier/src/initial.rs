// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

type Construction = (BodySpec, RotationSpec, Option<Inertia>);

/// Captured spawn states and engine settings; wrapped in [`InitialPhysicsState`].
struct InitialData {
    gravity: Vector,
    parameters: IntegrationParameters,
    bodies: BTreeMap<BodyId, Construction>,
    joints: joints::InitialJoints,
    next_id: u64,
}

/// A rebuilt solver, wrapped in [`PreparedPhysicsReset`].
struct PreparedData {
    world: PhysicsWorld,
    bodies: BTreeMap<BodyId, BodyEntry>,
    joints: joints::WorldJoints,
    next_id: u64,
}

impl RapierBackend {
    pub(crate) fn capture_initial(&self) -> InitialPhysicsState {
        InitialPhysicsState::new(
            &self.owner,
            InitialData {
                gravity: self.world.gravity,
                parameters: self.world.integration_parameters,
                bodies: self
                    .bodies
                    .iter()
                    .map(|(&id, &(_, spec, rotation, inertia))| (id, (spec, rotation, inertia)))
                    .collect(),
                joints: self.joints.initial(),
                next_id: self.next_id,
            },
        )
    }

    pub(crate) fn build_initial(
        &self,
        initial: &InitialPhysicsState,
    ) -> Result<PreparedPhysicsReset, PhysicsError> {
        let initial = initial.data::<InitialData>(&self.owner)?;
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
        let joints = joints::WorldJoints::restore(&initial.joints, &mut world, &bodies);
        Ok(PreparedPhysicsReset::new(
            &self.owner,
            PreparedData {
                world,
                bodies,
                joints,
                next_id: initial.next_id,
            },
        ))
    }

    /// A foreign state leaves this backend unchanged.
    pub(crate) fn commit_initial(
        &mut self,
        prepared: PreparedPhysicsReset,
    ) -> Result<(), PhysicsError> {
        let mut prepared = prepared.into_data::<PreparedData>(&self.owner)?;
        prepared.joints.preserve_high_water(&self.joints);
        self.joints = prepared.joints;
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
