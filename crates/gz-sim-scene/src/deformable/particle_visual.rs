// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bind ordinary SDF visuals to measured material particles without adding a collision body.
use super::*;

pub(super) struct Binding {
    pub material: String,
    pub particle: usize,
    pub visual: u64,
}
#[expect(
    clippy::type_complexity,
    reason = "Bevy assigns separate query lifetimes to conflicting world-pose reads and writes in this ParamSet"
)]
pub(super) fn sync(
    soft: Res<Deformables>,
    lookup: EntityLookup,
    parents: Query<&ChildOf>,
    mut poses: ParamSet<(Query<&WorldPose>, Query<(&mut WorldPose, &mut LocalPose)>)>,
) {
    let mut resolve = lookup.reader();
    for binding in &soft.visuals {
        let Some(entity) = resolve(binding.visual) else {
            continue;
        };
        let Ok(parent) = parents.get(entity) else {
            continue;
        };
        let Ok(parent_pose) = poses.p0().get(parent.parent()).copied() else {
            continue;
        };
        let mut writable = poses.p1();
        let Ok((mut world, mut local)) = writable.get_mut(entity) else {
            continue;
        };
        world.translation = soft.samples[&binding.material].particles[binding.particle].position;
        // Material points determine only translation. Retain the original visual
        // rotation relative to the link, including across a fresh world or reset.
        local.0.translation = parent_pose.inverse().compose(*world).translation;
    }
}
