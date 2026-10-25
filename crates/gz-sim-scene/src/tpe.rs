// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_rapier::BodyState;
use gz_physics_tpe::{Motion, MotionReset};
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhysicsEngine {
    Rapier,
    Tpe,
}

#[derive(Resource)]
pub(super) struct TpeWorld(pub Arc<()>);

#[derive(Component)]
pub(super) struct TpeLink;

pub(super) fn add_model(
    sim: &mut Simulation,
    entity: Entity,
    pose: Transform,
) -> Result<(), LoadError> {
    let motion = Motion::new(BodyState {
        position: pose.translation,
        orientation: pose.rotation,
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
    })?;
    sim.world_mut().entity_mut(entity).insert(motion);
    Ok(())
}

pub(super) fn validate_link(spec: BodySpec, pose: Transform) -> Result<(), LoadError> {
    let valid_shape = match spec.shape {
        Shape::Sphere { radius } => radius.is_finite() && radius > 0.0,
        Shape::Box { half_extents } => half_extents.into_iter().all(|v| v.is_finite() && v > 0.0),
    };
    if !valid_shape {
        return Err(PhysicsError::InvalidValue.into());
    }
    Motion::new(BodyState {
        position: pose.translation,
        orientation: pose.rotation,
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
    })?;
    Ok(())
}

pub(super) fn install(sim: &mut Simulation) -> Result<(), LoadError> {
    gz_physics_tpe::install(sim)?;
    sim.world_mut().insert_resource(TpeWorld(Arc::new(())));
    sim.add_systems(
        Phase::PhysicsCommit,
        (sync_models, sync_links).chain().before(super::sync_models),
    );
    sim.add_systems(
        Phase::Reset,
        (sync_models, sync_links)
            .chain()
            .after(MotionReset)
            .before(super::sync_models),
    );
    Ok(())
}

fn sync_models(mut models: Query<(&Motion, &mut WorldPose)>) {
    for (motion, mut pose) in &mut models {
        pose.translation = motion.state().position;
        pose.rotation = motion.state().orientation;
    }
}

fn sync_links(
    models: Query<(&Motion, &WorldPose), Without<TpeLink>>,
    mut links: Query<(&ChildOf, &LocalPose, &mut WorldPose, &mut Velocity), With<TpeLink>>,
) {
    for (parent, local, mut pose, mut velocity) in &mut links {
        let (motion, parent_pose) = models.get(parent.parent()).expect("loader owns TPE model");
        *pose = parent_pose.compose(local.0);
        velocity.linear = motion.state().velocity;
        velocity.angular = motion.state().angular_velocity;
    }
}

#[cfg(test)]
#[path = "../tests/private/tpe.rs"]
mod tests;
