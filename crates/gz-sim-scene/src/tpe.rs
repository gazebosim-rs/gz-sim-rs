// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::BodyState;
use gz_physics_tpe::{ModelBounds, Motion, MotionReset};
use parry3d_f64::{
    bounding_volume::{Aabb, BoundingVolume},
    math::{Pose, Rotation, Vector},
};
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhysicsEngine {
    Rapier,
    Tpe,
}

#[derive(Resource)]
pub(super) struct TpeWorld(pub Arc<()>);

#[derive(Component, Clone)]
pub(super) struct TpeLink(Aabb);

pub(super) fn add_model(
    sim: &mut Simulation,
    entity: Entity,
    pose: Transform,
    is_static: bool,
) -> Result<(), LoadError> {
    let state = BodyState {
        position: pose.translation,
        orientation: pose.rotation,
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
    };
    let motion = if is_static {
        Motion::new_fixed(state)?
    } else {
        Motion::new(state)?
    };
    sim.world_mut().entity_mut(entity).insert(motion);
    Ok(())
}

pub(super) fn validate_link(spec: BodySpec, pose: Transform) -> Result<TpeLink, LoadError> {
    let valid_shape = match spec.shape {
        Shape::Sphere { radius } => radius.is_finite() && radius > 0.0,
        Shape::Cylinder {
            radius,
            half_height,
        } => [radius, half_height]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.0),
        Shape::Capsule {
            radius,
            half_length,
        } => {
            [radius, half_length]
                .into_iter()
                .all(|v| v.is_finite() && v > 0.0)
                && (radius + half_length).is_finite()
        }
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
    let half = match spec.shape {
        Shape::Sphere { radius } => [radius; 3],
        Shape::Cylinder {
            radius,
            half_height,
        } => [radius, radius, half_height],
        Shape::Capsule {
            radius,
            half_length,
        } => [radius, radius, half_length + radius],
        Shape::Box { half_extents } => half_extents,
    };
    Ok(TpeLink(Aabb::from_half_extents(
        Vector::ZERO,
        Vector::from_array(half),
    )))
}

/// Primitive collider-local bounds for the newly posed/compound TPE domain.
/// TPE still reports model AABB overlap; this does not add contact resolution.
pub(super) fn validate_collisions(
    collisions: &[CollisionSpec],
    pose: Transform,
) -> Result<TpeLink, LoadError> {
    if !(1..=16).contains(&collisions.len()) {
        return Err(PhysicsError::InvalidValue.into());
    }
    let legacy = |collision: &CollisionSpec| {
        validate_link(
            BodySpec {
                shape: collision.shape,
                position: pose.translation,
                velocity: [0.0; 3],
                mass: None,
            },
            pose,
        )
    };
    if collisions.len() == 1
        && identity(Transform {
            translation: collisions[0].position,
            rotation: collisions[0].orientation,
        })
    {
        return legacy(&collisions[0]);
    }
    let body = Pose::from_parts(
        Vector::from_array(pose.translation),
        Rotation::from_xyzw(
            pose.rotation[0],
            pose.rotation[1],
            pose.rotation[2],
            pose.rotation[3],
        ),
    );
    let mut aggregate: Option<Aabb> = None;
    for collision in collisions {
        // Reuse all existing primitive dimension and world-pose validation.
        legacy(collision)?;
        Motion::new(BodyState {
            position: collision.position,
            orientation: collision.orientation,
            velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
        })?;
        let q = collision.orientation;
        let local = Pose::from_parts(
            Vector::from_array(collision.position),
            Rotation::from_xyzw(q[0], q[1], q[2], q[3]),
        );
        // Parry cylinders are Y-axis; the retained SDF cylinder is Z-axis.
        let (shape, shape_pose) = match collision.shape {
            Shape::Sphere { radius } => (parry3d_f64::shape::SharedShape::ball(radius), local),
            Shape::Box {
                half_extents: [x, y, z],
            } => (parry3d_f64::shape::SharedShape::cuboid(x, y, z), local),
            Shape::Capsule {
                radius,
                half_length,
            } => (
                parry3d_f64::shape::SharedShape::capsule_z(half_length, radius),
                local,
            ),
            Shape::Cylinder {
                radius,
                half_height,
            } => (
                parry3d_f64::shape::SharedShape::cylinder(half_height, radius),
                local * Pose::from_rotation(Rotation::from_rotation_x(std::f64::consts::FRAC_PI_2)),
            ),
        };
        let bounds = shape.compute_aabb(&shape_pose);
        for checked in [bounds, bounds.transform_by(&body)] {
            if !checked.mins.is_finite()
                || !checked.maxs.is_finite()
                || !(checked.maxs - checked.mins).is_finite()
                || (checked.maxs - checked.mins)
                    .to_array()
                    .into_iter()
                    .any(|v| v <= 0.0)
            {
                return Err(PhysicsError::InvalidValue.into());
            }
        }
        match &mut aggregate {
            Some(current) => current.merge(&bounds),
            None => aggregate = Some(bounds),
        }
    }
    Ok(TpeLink(
        aggregate.expect("nonempty validated collision domain"),
    ))
}

pub(super) fn add_bounds(
    sim: &mut Simulation,
    model: Entity,
    is_static: bool,
) -> Result<(), LoadError> {
    let mut aggregate: Option<Aabb> = None;
    if let Some(children) = sim.world().get::<Children>(model) {
        for entity in children.iter() {
            let Some(link) = sim.world().get::<TpeLink>(entity) else {
                continue;
            };
            let local = sim
                .world()
                .get::<LocalPose>(entity)
                .expect("loader owns link poses")
                .0;
            let q = local.rotation;
            let pose = Pose::from_parts(
                Vector::from_array(local.translation),
                Rotation::from_xyzw(q[0], q[1], q[2], q[3]),
            );
            let bounds = link.0.transform_by(&pose);
            match &mut aggregate {
                Some(aggregate) => aggregate.merge(&bounds),
                None => aggregate = Some(bounds),
            }
        }
    }
    if let Some(bounds) = aggregate {
        let bounds = ModelBounds::new(
            bounds.mins.to_array(),
            bounds.maxs.to_array(),
            is_static,
            u16::MAX,
        )?;
        sim.world_mut().entity_mut(model).insert(bounds);
    }
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
