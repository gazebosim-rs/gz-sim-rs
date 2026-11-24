// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{LoadError, identity, require};
use gz_physics::PhysicsError;
use gz_physics_distributed::{SceneBackend, physics};
use sdformat::{Geometry, Sdf, Surface, World};
use std::{path::Path, time::Duration};

/// Advance an island-refining SDF simulation once with a recorded whole-island
/// placement. Admitted wrenches and native impulses participate in preflight.
/// An outstanding deferred public body edit remains untouched on refusal.
pub fn step_with_placement(
    sim: &mut gz_sim_core::Simulation,
    record: &gz_physics_distributed::pool::PlacementRecord,
) -> Result<(), PhysicsError> {
    if crate::inputs::has_pending_input(sim.world()) {
        return Err(PhysicsError::TransitionDeferred {
            reason: "placement replay requires the deferred ECS body edit to be published",
        });
    }
    SceneBackend::step_simulation_with_placement(sim, record)
}

pub(super) fn new_backend(
    sdf: &Sdf,
    world: &World,
    step: Duration,
    trusted_worker: Option<&Path>,
) -> Result<SceneBackend, LoadError> {
    require(
        physics::SimulationStep::from_duration(step).is_some(),
        &world.name,
        "distributed physics supports steps of 1, 2, 5 and 10 ms",
    )?;
    require(
        world.joints.is_empty(),
        &world.name,
        "distributed world joints",
    )?;
    require(
        world
            .models
            .iter()
            .filter(|m| {
                !m.links
                    .iter()
                    .flat_map(|l| &l.collisions)
                    .any(|c| matches!(c.geometry, Geometry::Plane { .. }))
            })
            .count()
            <= physics::MAX_PARTITION_BODIES,
        &world.name,
        "distributed body limit",
    )?;
    let mut ground_seen = false;
    for model in &world.models {
        require(
            model.models.is_empty()
                && model.links.len() == 1
                && model.joints.is_empty()
                && !model.self_collide
                && !model.enable_wind,
            &model.scoped_name,
            "distributed physics requires one primitive link per top-level model",
        )?;
        let link = &model.links[0];
        let name = &link.scoped_name;
        require(
            link.collisions.len() == 1,
            name,
            "distributed physics requires one centered collider",
        )?;
        let collision = &link.collisions[0];
        let plane = matches!(collision.geometry, Geometry::Plane { .. });
        if plane {
            require(
                model.is_static && !ground_seen,
                name,
                "distributed physics admits one shared static plane",
            )?;
            ground_seen = true;
        } else if !matches!(collision.geometry,Geometry::Box{..}|Geometry::Sphere{..}|Geometry::Capsule{..}|Geometry::Cylinder{..}) {
            return Err(LoadError::Unsupported {
                entity: name.clone(),
                feature: "distributed physics requires a box, sphere, cylinder or capsule",
            });
        };
        require(
            plane || identity(sdf.resolve_pose(&collision.pose, &model.scoped_name, name, name)?),
            name,
            "distributed primitive must be centered and aligned with the link",
        )?;
        require(
            identity(sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, name, name)?),
            name,
            "distributed primitive requires centered link-aligned inertial axes",
        )?;
        require(
            admitted_surface(&collision.surface),
            name,
            "distributed surfaces require equal nonnegative ODE mu and mu2 and default contact settings",
        )?;
        // SceneBackend validates the supplied mass and tensor against the native
        // uniform-density box when spawning each link, before starting workers.
    }
    let path = if let Some(path) = trusted_worker {
        // A bare filename is relative to the caller's working directory. Never
        // search PATH or consult a model resource, a plugin name, or SDF paths.
        std::path::absolute(path)
    } else {
        std::env::current_exe().map(|path| {
            path.with_file_name(format!("physics_worker{}", std::env::consts::EXE_SUFFIX))
        })
    }
    .map_err(|err| PhysicsError::Native {
        engine: "distributed",
        message: err.to_string(),
    })?;
    Ok(SceneBackend::new(&path, world.gravity, step)?)
}

fn admitted_surface(surface: &Surface) -> bool {
    let friction = &surface.friction;
    let ode = &friction.ode;
    let contact = &surface.contact;
    ode.mu.is_finite() && ode.mu>=0. && ode.mu==ode.mu2
        && ode.fdir1 == [0.0; 3]
        && ode.slip1 == 0.0
        && ode.slip2 == 0.0
        && friction.bullet.is_none()
        && friction.torsional.is_none()
        && contact.collide_bitmask == sdformat::Contact::default().collide_bitmask
        && contact
            .category_bitmask
            .is_none_or(|category| category == contact.collide_bitmask)
}
