// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Construct the local serial CPU owner with its authored step and IMU streams
/// keyed by scoped name, channel and advancing iteration. All resources are inline.
pub fn load_local_replay_world(
    xml: &str,
    source: &str,
    world: &str,
    seed: [u8; 32],
) -> Result<Simulation, LoadError> {
    load_local_replay_world_with_runtime(xml, source, world, seed, WorldRuntimeOptions::default())
}

/// Construct the seeded local replay world with the application's native solver settings.
pub fn load_local_replay_world_with_runtime(
    xml: &str,
    source: &str,
    world: &str,
    seed: [u8; 32],
    runtime: WorldRuntimeOptions<'_>,
) -> Result<Simulation, LoadError> {
    let mut sim = load_world_with_automatic_owner(
        xml,
        source,
        world,
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: seed,
            distributed_worker: None,
        },
        runtime,
        None,
        true,
        #[cfg(feature = "distributed-physics")]
        None,
    )?;
    // Use the same native link-origin and joint-frame poses for the initial
    // observation as for pause and reset. Converting an offset mass center back
    // to its link origin can differ by one ULP from the authored translation.
    let mut publication = Schedule::default();
    publication.add_systems(
        (
            sync_bodies,
            sync_models,
            sync_local_poses,
            sync_attached_poses,
            joints::sample,
        )
            .chain(),
    );
    publication.run(sim.world_mut());
    Ok(sim)
}

pub(super) fn validate(world: &sdformat::World) -> Result<(), LoadError> {
    use sdformat::Geometry::{Box, Capsule, Cylinder, Plane, Sphere};
    let primitive = |geometry: &sdformat::Geometry, allow_plane: bool| {
        matches!(
            geometry,
            Box { .. } | Sphere { .. } | Cylinder { .. } | Capsule { .. }
        ) || (allow_plane && matches!(geometry, Plane { .. }))
    };
    let admitted = world.plugins.iter().all(|p| {
        world_systems::is_physics_plugin(p)
            || primary_contact::sdf::is_plugin(p)
            || mounted_guide::is_plugin(p)
    }) && world.joints.is_empty()
        && world.extensions.is_empty()
        && world.models.iter().all(|model| {
            model.models.is_empty()
                && model.extensions.is_empty()
                && model.joints.iter().all(|j| {
                    matches!(
                        j.kind,
                        sdformat::JointKind::Fixed
                            | sdformat::JointKind::Ball
                            | sdformat::JointKind::Revolute
                            | sdformat::JointKind::Prismatic
                            | sdformat::JointKind::Screw
                            | sdformat::JointKind::Universal
                    )
                })
                && model.plugins.iter().all(|p| {
                    prescribed_motion::is_initial_velocity(p)
                        || contact_material::is_plugin(p)
                        || crate::articulation::is_plugin(p)
                        || joints::is_scalar_tree_plugin(p)
                        || joint_forces::is_plugin(p)
                        || diff_drive::is_plugin(p)
                        || crate::joint_state_publisher::is_plugin(p)
                        || matches!(
                            p.filename.as_str(),
                            "gz-sim-joint-position-controller-system"
                                | "gz-sim-joint-controller-system"
                        )
                })
                && model.links.iter().all(|link| {
                    link.plugins.is_empty()
                        && link.projectors.is_empty()
                        && link.particle_emitters.is_empty()
                        && link
                            .collisions
                            .iter()
                            .all(|c| primitive(&c.geometry, model.is_static))
                        && link.visuals.iter().all(|v| {
                            v.geometry
                                .as_ref()
                                .is_some_and(|geometry| primitive(geometry, true))
                        })
                        && link
                            .sensors
                            .iter()
                            .all(|s| s.kind == "imu" && s.plugins.is_empty())
                })
        });
    require(
        admitted,
        &world.name,
        "local-cpu-v1 admits static ground planes, inline rigid box, sphere, cylinder and capsule models, restitution, initial twists, incoming-face and screw-stop refinement, mounted slider stabilization, native joint controllers, held joint forces and IMUs",
    )
}
