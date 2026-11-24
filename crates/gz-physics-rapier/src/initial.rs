// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

type InitialConstruction = (Construction, RotationSpec, Option<Inertia>, Option<u64>);

/// Captured spawn states and engine settings; wrapped in [`InitialPhysicsState`].
struct InitialData {
    curves: trajectory::Curves,
    conserved_rotor: Option<BodyId>,
    #[cfg(feature = "partition-checkpoint")]
    pair_observer: Option<PairObserver>,
    #[cfg(feature = "partition-checkpoint")]
    grasp_payload_observer: Option<GraspPayloadObserver>,
    gravity: Vector,
    #[cfg(feature = "partition-checkpoint")]
    ground: Option<NativeGroundPlane>,
    parameters: IntegrationParameters,
    bodies: BTreeMap<BodyId, InitialConstruction>,
    joints: joints::InitialJoints,
    next_id: u64,
}

/// A rebuilt solver, wrapped in [`PreparedPhysicsReset`].
struct PreparedData {
    curves: trajectory::Curves,
    conserved_rotor: Option<BodyId>,
    #[cfg(feature = "partition-checkpoint")]
    pair_observer: Option<PairObserver>,
    #[cfg(feature = "partition-checkpoint")]
    grasp_payload_observer: Option<GraspPayloadObserver>,
    world: PhysicsWorld,
    bodies: BTreeMap<BodyId, BodyEntry>,
    joints: joints::Joints,
    next_id: u64,
}

impl RapierBackend {
    pub(crate) fn capture_initial(&self) -> InitialPhysicsState {
        InitialPhysicsState::new(
            &self.owner,
            InitialData {
                curves: self.curves.clone(),
                conserved_rotor: self.conserved_rotor,
                #[cfg(feature = "partition-checkpoint")]
                pair_observer: self.pair_observer,
                #[cfg(feature = "partition-checkpoint")]
                grasp_payload_observer: self.grasp_payload_observer.map(|o| GraspPayloadObserver {
                    incoming_velocity: self.bodies[&o.parts[0]].1.velocity,
                    external_impulse: [0.; 3],
                    ..o
                }),
                gravity: self.world.gravity,
                #[cfg(feature = "partition-checkpoint")]
                ground: partition_ground::initial_surface(&self.world),
                parameters: self.world.integration_parameters,
                bodies: self
                    .bodies
                    .iter()
                    .map(|(&id, (_, spec, rotation, inertia, group))| {
                        (id, (spec.clone(), *rotation, *inertia, *group))
                    })
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
        #[cfg(feature = "partition-checkpoint")]
        if let Some(h) = initial.ground {
            world.colliders.insert(partition_ground::collider_at(h));
        }
        let mut bodies = BTreeMap::new();
        for (&id, (spec, rotational, inertia, group)) in &initial.bodies {
            let handle = insert_initial(&mut world, spec, *rotational, *inertia);
            collision::assign(&mut world, handle, *group);
            bodies.insert(id, (handle, spec.clone(), *rotational, *inertia, *group));
        }
        let joints = joints::Joints::restore(&initial.joints, &mut world, &bodies);
        Ok(PreparedPhysicsReset::new(
            &self.owner,
            PreparedData {
                curves: initial.curves.clone(),
                conserved_rotor: initial.conserved_rotor,
                #[cfg(feature = "partition-checkpoint")]
                pair_observer: initial.pair_observer,
                #[cfg(feature = "partition-checkpoint")]
                grasp_payload_observer: initial.grasp_payload_observer,
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
        self.welded_projection = None;
        self.welded_exclusion = None;
        self.joints = prepared.joints;
        self.curves = prepared.curves;
        self.pending_wrenches.clear();
        self.held_wrenches.clear();
        self.spinning_assemblies.clear();
        self.parked_robots.clear();
        self.parked_speed.clear();
        self.held_supports.clear();
        self.support_watch = Default::default();
        self.cold_support_pairs.clear();
        self.free_coast = None;
        self.conserved_rotor = prepared.conserved_rotor;
        #[cfg(feature = "partition-checkpoint")]
        {
            self.join_queries_pending = false;
            self.pair_observer = prepared.pair_observer;
            self.grasp_payload_observer = prepared.grasp_payload_observer;
        }
        self.world = prepared.world;
        self.bodies = prepared.bodies;
        self.refresh_max_boundary_spin();
        self.restore_fleet_clock();
        self.next_id = self.next_id.max(prepared.next_id);
        Ok(())
    }
}

pub(super) fn insert_initial(
    world: &mut PhysicsWorld,
    spec: &Construction,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
) -> RigidBodyHandle {
    // Both callers use immutable construction records validated before insertion.
    insert(
        world,
        spec,
        rotational,
        spec.collider.clone(),
        inertia.map(|value| {
            inertia_properties(value, spec.mass.unwrap_or(1.0)).expect("validated inertia")
        }),
    )
}

#[cfg(test)]
#[path = "../tests/private/initial.rs"]
mod tests;
