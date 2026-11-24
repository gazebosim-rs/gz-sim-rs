// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DVec3;
use gz_physics_distributed::moving::MovingReaction;

/// Read both native reactions and publicly committed ECS poses from the ordinary world.
pub(super) fn observe(
    world: &AuthoredHinge,
    spec: &MultiSphereSpec,
    tick: u64,
) -> Result<MultiSphereObservation> {
    let ([parent, child], hinge) = world.hinge(&spec.hinge, tick)?;
    let (a, sphere_a) = world.body(spec.spheres[0].id, tick)?;
    let (b, sphere_b) = world.body(spec.spheres[1].id, tick)?;
    let native = world.native()?;
    Ok(MultiSphereObservation {
        hinge,
        spheres: [sphere_a, sphere_b],
        reactions: [
            [
                native.pair_reaction(parent, a)?,
                native.pair_reaction(child, a)?,
            ],
            [
                native.pair_reaction(parent, b)?,
                native.pair_reaction(child, b)?,
            ],
        ],
        sphere_reaction: native.pair_reaction(a, b)?,
    })
}

fn vector(reaction: &MovingReaction) -> DVec3 {
    DVec3::from(reaction.normal) * reaction.normal_impulse + DVec3::from(reaction.tangent_impulse)
}

pub(super) fn momentum_residual(
    spec: &MultiSphereSpec,
    before: &MultiSphereObservation,
    after: &MultiSphereObservation,
) -> f64 {
    let pair = vector(&after.sphere_reaction);
    (0..2)
        .map(|i| {
            let delta =
                DVec3::from(after.spheres[i].velocity) - DVec3::from(before.spheres[i].velocity);
            let impulse = if i == 0 { pair } else { -pair }
                - after.reactions[i].iter().map(vector).sum::<DVec3>();
            (spec.spheres[i].mass * delta - impulse).length()
        })
        .try_fold(0.0_f64, |max, error| {
            error.is_finite().then(|| max.max(error))
        })
        .unwrap_or(f64::INFINITY)
}
