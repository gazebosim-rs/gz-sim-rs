// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{BodyHandle, EntityKind};
use bevy_ecs::hierarchy::{ChildOf, Children};
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use std::collections::BTreeSet;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RemoveError {
    #[error("public entity {0} does not exist")]
    MissingEntity(u64),
    #[error("entity {0} is not a model directly owned by the world")]
    NotWorldModel(u64),
    #[error("the selected physics profile retains this model for the lifetime of the loaded world")]
    ImmutableModel,
    #[error("scene physics bindings are incomplete or inconsistent")]
    InvalidPhysics,
    #[error("scene schedule initialization failed: {0}")]
    Schedule(String),
    #[error("model was removed but system schedule retirement failed: {0}")]
    CommittedSchedule(String),
}

/// Removes a top-level model, its descendants and their physics bodies.
/// Call at a lifecycle boundary, outside an executing simulation schedule.
/// All checks precede model/physics mutation. Retiring the last IMU or joint initializes
/// pending schedules without running callbacks; arbitrary Local initializers
/// follow the core initialization contract. Returned public IDs retire here.
/// Schedule-built observers run before final validation or after body commit.
/// CommittedSchedule reports teardown failure after removal and faults the run.
/// This does not restore removed models during the existing core time reset.
pub fn remove_model(sim: &mut Simulation, id: u64) -> Result<Vec<u64>, RemoveError> {
    let removed = remove_model_checked(sim, id, false)?;
    crate::damped_sole::retire(sim.world_mut(), &removed);
    #[cfg(feature = "distributed-physics")]
    crate::passive_checkpoint::note_live_retirement(sim, id);
    Ok(removed)
}

#[derive(Debug, thiserror::Error)]
pub(super) enum SystemRetirementError {
    #[error(transparent)]
    Schedule(#[from] bevy_ecs::schedule::ScheduleError),
    #[error("a schedule observer added consumers while their callbacks were being retired")]
    AddedConsumer,
}

pub(super) fn retire_unused_systems(sim: &mut Simulation) -> Result<(), SystemRetirementError> {
    loop {
        let before = [
            sim.world()
                .contains_resource::<crate::imu_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_force_lifecycle::Installed>(),
            sim.world().contains_resource::<crate::lidar::Installed>(),
            sim.world()
                .contains_resource::<crate::diff_drive::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_controller::Installed>(),
            sim.world()
                .contains_resource::<crate::altimeter::Installed>(),
            sim.world()
                .contains_resource::<crate::air_pressure::Installed>(),
            sim.world()
                .contains_resource::<crate::magnetometer::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_state_publisher::Installed>(),
            sim.world()
                .contains_resource::<crate::air_speed::Installed>(),
            sim.world()
                .contains_resource::<crate::pose_publisher::Installed>(),
            sim.world()
                .contains_resource::<crate::logical_camera::Installed>(),
            sim.world()
                .contains_resource::<crate::odometry_publisher::Installed>(),
            sim.world()
                .contains_resource::<crate::apply_link_wrench::Installed>(),
        ];
        crate::fidelity_zone::retire_if_unused(sim)?;
        crate::contact_promotion::retire_if_unused(sim)?;
        crate::support_loss_promotion::retire_if_unused(sim)?;
        crate::imu_lifecycle::retire_if_unused(sim)?;
        crate::lidar::retire_if_unused(sim)?;
        crate::altimeter::retire_if_unused(sim)?;
        crate::air_pressure::retire_if_unused(sim)?;
        crate::magnetometer::retire_if_unused(sim)?;
        crate::joint_state_publisher::retire_if_unused(sim)?;
        crate::air_speed::retire_if_unused(sim)?;
        crate::logical_camera::retire_if_unused(sim)?;
        crate::pose_publisher::retire_if_unused(sim)?;
        crate::odometry_publisher::retire_if_unused(sim)?;
        crate::apply_link_wrench::retire_if_unused(sim)?;
        crate::diff_drive::retire_if_unused(sim)?;
        crate::joint_controller::retire_if_unused(sim)?;
        crate::joint_lifecycle::retire_if_unused(sim)?;
        crate::joint_force_lifecycle::retire_if_unused(sim)?;
        let after = [
            sim.world()
                .contains_resource::<crate::imu_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_force_lifecycle::Installed>(),
            sim.world().contains_resource::<crate::lidar::Installed>(),
            sim.world()
                .contains_resource::<crate::diff_drive::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_controller::Installed>(),
            sim.world()
                .contains_resource::<crate::altimeter::Installed>(),
            sim.world()
                .contains_resource::<crate::air_pressure::Installed>(),
            sim.world()
                .contains_resource::<crate::magnetometer::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_state_publisher::Installed>(),
            sim.world()
                .contains_resource::<crate::air_speed::Installed>(),
            sim.world()
                .contains_resource::<crate::pose_publisher::Installed>(),
            sim.world()
                .contains_resource::<crate::logical_camera::Installed>(),
            sim.world()
                .contains_resource::<crate::odometry_publisher::Installed>(),
            sim.world()
                .contains_resource::<crate::apply_link_wrench::Installed>(),
        ];
        if before == after {
            if sim.world().iter_entities().any(|entity| {
                !after[0] && entity.contains::<crate::Imu>()
                    || !after[1] && entity.contains::<crate::JointHandle>()
                    || !after[2] && entity.contains::<crate::JointForceCommand>()
                    || !after[3] && entity.contains::<crate::lidar::Lidar>()
                    || !after[4] && entity.contains::<crate::diff_drive::Drive>()
                    || !after[5] && entity.contains::<crate::joint_controller::Controller>()
                    || !after[6] && entity.contains::<crate::altimeter::Altimeter>()
                    || !after[7] && entity.contains::<crate::air_pressure::AirPressure>()
                    || !after[8] && entity.contains::<crate::magnetometer::Magnetometer>()
                    || !after[9] && entity.contains::<crate::joint_state_publisher::Publisher>()
                    || !after[10] && entity.contains::<crate::air_speed::AirSpeed>()
                    || !after[11] && entity.contains::<crate::pose_publisher::Publisher>()
                    || !after[12] && entity.contains::<crate::logical_camera::LogicalCamera>()
                    || !after[13] && entity.contains::<crate::odometry_publisher::Publisher>()
            }) || !after[14] && crate::apply_link_wrench::needs_activation(sim.world())
            {
                return Err(SystemRetirementError::AddedConsumer);
            }
            return Ok(());
        }
        // A schedule build may remove another family's last consumer.
        // Markers only retire here, so at most fifteen rounds can change them.
    }
}

fn remove_model_checked(
    sim: &mut Simulation,
    id: u64,
    schedules_ready: bool,
) -> Result<Vec<u64>, RemoveError> {
    let world = sim.world();
    let root = entity_by_id(world, id).ok_or(RemoveError::MissingEntity(id))?;
    let parent = world.get::<ChildOf>(root).map(ChildOf::parent);
    if world.get::<EntityKind>(root) != Some(&EntityKind::Model)
        || parent.and_then(|entity| world.get::<EntityKind>(entity)) != Some(&EntityKind::World)
    {
        return Err(RemoveError::NotWorldModel(id));
    }
    if !crate::joints::bindings_valid(world) {
        return Err(RemoveError::InvalidPhysics);
    }
    let backend = world.get_resource::<PhysicsBackend>();
    #[cfg(feature = "distributed-physics")]
    if let Some(backend) = backend {
        use crate::distributed_hinge::{
            DistributedFloatingTreeBackend, DistributedGraspTreeBackend,
            DistributedGroundedTreeBackend, DistributedHingeBackend, DistributedMotorTreeBackend,
            DistributedMovingHingeBackend, DistributedMultiSphereBackend,
            DistributedPayloadTreeBackend, DistributedSplitGraspBackend,
            DistributedStoppedTreeBackend,
        };
        let contains_link = |link_id| {
            entity_by_id(world, link_id)
                .and_then(|link| world.get::<ChildOf>(link))
                .is_some_and(|parent| parent.parent() == root)
        };
        let immutable = if let Some(fleet) =
            backend.downcast_ref::<crate::distributed_hinge::DistributedGraspFleetBackend>()
        {
            fleet
                .pool()
                .spec()
                .assemblies
                .iter()
                .any(|a| id == a.payload.tree.model_id || contains_link(a.payload.payload.id))
        } else if let Some(tree) = backend.downcast_ref::<DistributedHingeBackend>() {
            let spec = tree.pool().spec();
            id == spec.model_id || spec.ground.is_some_and(|g| contains_link(g.link_id))
        } else if let Some(tree) = backend.downcast_ref::<DistributedFloatingTreeBackend>() {
            id == tree.pool().spec().model_id
        } else if let Some(tree) = backend.downcast_ref::<DistributedSplitGraspBackend>() {
            id == tree.pool().topology().model_id || contains_link(tree.pool().payload_state().id)
        } else if let Some(tree) = backend.downcast_ref::<DistributedGraspTreeBackend>() {
            id == tree.pool().topology().model_id || contains_link(tree.pool().payload_state().id)
        } else if let Some(tree) = backend.downcast_ref::<DistributedPayloadTreeBackend>() {
            id == tree.pool().topology().model_id || contains_link(tree.pool().spec().payload.id)
        } else if let Some(tree) = backend.downcast_ref::<DistributedGroundedTreeBackend>() {
            id == tree.pool().topology().model_id
                || contains_link(tree.pool().spec().ground.link_id)
        } else if let Some(tree) = backend.downcast_ref::<DistributedMotorTreeBackend>() {
            id == tree.pool().topology().model_id
        } else if let Some(tree) = backend.downcast_ref::<DistributedStoppedTreeBackend>() {
            id == tree.pool().topology().model_id
        } else if let Some(moving) = backend.downcast_ref::<DistributedMovingHingeBackend>() {
            let spec = moving.pool().spec();
            id == spec.hinge.model_id || contains_link(spec.sphere.id)
        } else if let Some(pair) = backend.downcast_ref::<DistributedMultiSphereBackend>() {
            let spec = pair.pool().spec();
            id == spec.hinge.model_id || spec.spheres.iter().any(|s| contains_link(s.id))
        } else {
            false
        };
        if immutable {
            return Err(RemoveError::ImmutableModel);
        }
    }
    let distributed_pool = backend.is_some_and(|b| b.engine() == "distributed-pool");
    if backend.is_some() == world.contains_resource::<crate::tpe::TpeWorld>() {
        return Err(RemoveError::InvalidPhysics);
    }
    let mut pending = vec![root];
    let mut bodies = BTreeSet::new();
    let mut subtree = BTreeSet::new();
    let mut ids = Vec::new();
    while let Some(entity) = pending.pop() {
        if backend.is_none() {
            let valid = match world.get::<EntityKind>(entity) {
                Some(EntityKind::Model) => world.get::<gz_physics_tpe::Motion>(entity).is_some(),
                Some(EntityKind::Link) => world.get::<crate::tpe::TpeLink>(entity).is_some(),
                _ => true,
            };
            if !valid {
                return Err(RemoveError::InvalidPhysics);
            }
        }
        subtree.insert(entity);
        if let Some(id) = world.get::<EntityId>(entity) {
            ids.push(id.get());
        }
        #[cfg(feature = "distributed-physics")]
        if let Some(body) = world.get::<BodyHandle>(entity)
            && backend
                .and_then(|b| b.downcast_ref::<gz_physics_distributed::SceneBackend>())
                .is_some_and(|scene| scene.is_shared_ground(body.id()))
        {
            return Err(RemoveError::ImmutableModel);
        }
        if let Some(body) = world.get::<BodyHandle>(entity)
            && (backend.is_none_or(|backend| backend.state(body.id()).is_none())
                || !bodies.insert(body.id()))
        {
            return Err(RemoveError::InvalidPhysics);
        }
        if let Some(children) = world.get::<Children>(entity) {
            pending.extend(children.iter());
        }
    }
    // Distributed SDF admits one primitive link per top-level model. Refuse
    // externally mutated multi-body subtrees before issuing any worker command.
    if distributed_pool && bodies.len() != 1 {
        return Err(RemoveError::InvalidPhysics);
    }
    if world.iter_entities().any(|entity| {
        !subtree.contains(&entity.id())
            && entity
                .get::<BodyHandle>()
                .is_some_and(|body| bodies.contains(&body.id()))
    }) {
        return Err(RemoveError::InvalidPhysics);
    }
    let retire_imu = world.contains_resource::<crate::imu_lifecycle::Installed>()
        && !world
            .iter_entities()
            .any(|entity| entity.contains::<crate::Imu>() && !subtree.contains(&entity.id()));
    let retire_joint = world.contains_resource::<crate::joint_lifecycle::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::JointHandle>() && !subtree.contains(&entity.id())
        });
    let retire_force = world.contains_resource::<crate::joint_force_lifecycle::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::JointForceCommand>() && !subtree.contains(&entity.id())
        });
    let retire_lidar = world.contains_resource::<crate::lidar::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::lidar::Lidar>() && !subtree.contains(&entity.id())
        });
    let retire_drive = world.contains_resource::<crate::diff_drive::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::diff_drive::Drive>() && !subtree.contains(&entity.id())
        });
    let retire_controller = world.contains_resource::<crate::joint_controller::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::joint_controller::Controller>()
                && !subtree.contains(&entity.id())
        });
    let retire_magnetometer = world.contains_resource::<crate::magnetometer::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::magnetometer::Magnetometer>()
                && !subtree.contains(&entity.id())
        });
    let retire_joint_state = world.contains_resource::<crate::joint_state_publisher::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::joint_state_publisher::Publisher>()
                && !subtree.contains(&entity.id())
        });
    let retire_air_speed = world.contains_resource::<crate::air_speed::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::air_speed::AirSpeed>() && !subtree.contains(&entity.id())
        });
    let retire_logical_camera = world.contains_resource::<crate::logical_camera::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::logical_camera::LogicalCamera>()
                && !subtree.contains(&entity.id())
        });
    let retire_pose_publisher = world.contains_resource::<crate::pose_publisher::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::pose_publisher::Publisher>() && !subtree.contains(&entity.id())
        });
    let retire_ground_truth = world.contains_resource::<crate::odometry_publisher::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::odometry_publisher::Publisher>()
                && !subtree.contains(&entity.id())
        });
    let retire_link_wrench = world.contains_resource::<crate::apply_link_wrench::Installed>()
        && !crate::apply_link_wrench::has_eligible_outside(world, &subtree);
    if (retire_imu
        || retire_joint
        || retire_force
        || retire_lidar
        || retire_drive
        || retire_controller
        || retire_magnetometer
        || retire_joint_state
        || retire_air_speed
        || retire_logical_camera
        || retire_pose_publisher
        || retire_ground_truth
        || retire_link_wrench)
        && !schedules_ready
    {
        sim.initialize_schedules()
            .map_err(|error| RemoveError::Schedule(error.to_string()))?;
        // Builds synchronously notify World observers. Their mutations must
        // precede the final hierarchy, binding and native-body validation.
        return remove_model_checked(sim, id, true);
    }
    let body_order =
        crate::joints::removal_order(sim.world(), &bodies).ok_or(RemoveError::InvalidPhysics)?;
    if let Some(mut backend) = sim.world_mut().get_resource_mut::<PhysicsBackend>() {
        for body in body_order {
            let removed = backend.remove(body);
            if distributed_pool && !removed {
                return Err(RemoveError::InvalidPhysics);
            }
            assert!(removed, "preflighted bodies remain owned until commit");
        }
    }
    // ChildOf's linked-spawn relationship also removes descendants and ID hooks.
    sim.world_mut().entity_mut(root).despawn();
    ids.sort_unstable();
    // No schedule build may intervene between final validation and body commit.
    // Teardown observers now see the committed removal, not stale body handles.
    if let Err(error) = retire_unused_systems(sim) {
        let message = error.to_string();
        sim.world()
            .resource::<gz_sim_core::StepFault>()
            .report(error);
        return Err(RemoveError::CommittedSchedule(message));
    }
    Ok(ids)
}
