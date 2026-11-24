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
    load_world_with_automatic_owner(
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
        WorldRuntimeOptions::default(),
        None,
        true,
        #[cfg(feature = "distributed-physics")]
        None,
    )
}

pub(super) fn validate(world: &sdformat::World) -> Result<(), LoadError> {
    use sdformat::Geometry::{Box, Sphere};
    let primitive = |geometry: &sdformat::Geometry| matches!(geometry, Box { .. } | Sphere { .. });
    let admitted = world.plugins.is_empty()
        && world.joints.is_empty()
        && world.extensions.is_empty()
        && world.models.iter().all(|model| {
            model.models.is_empty()
                && model.extensions.is_empty()
                && model.joints.iter().all(|j| {
                    matches!(
                        j.kind,
                        sdformat::JointKind::Revolute
                            | sdformat::JointKind::Prismatic
                            | sdformat::JointKind::Screw
                            | sdformat::JointKind::Universal
                    )
                })
                && model.plugins.iter().all(|p| {
                    matches!(
                        p.filename.as_str(),
                        "gz-sim-joint-position-controller-system"
                            | "gz-sim-joint-controller-system"
                    )
                })
                && model.links.iter().all(|link| {
                    link.plugins.is_empty()
                        && link.projectors.is_empty()
                        && link.particle_emitters.is_empty()
                        && link.collisions.iter().all(|c| primitive(&c.geometry))
                        && link
                            .visuals
                            .iter()
                            .all(|v| v.geometry.as_ref().is_some_and(primitive))
                        && link
                            .sensors
                            .iter()
                            .all(|s| s.kind == "imu" && s.plugins.is_empty())
                })
        });
    require(
        admitted,
        &world.name,
        "local-cpu-v1 admits inline rigid box/sphere models, native joint controllers and IMUs",
    )
}
