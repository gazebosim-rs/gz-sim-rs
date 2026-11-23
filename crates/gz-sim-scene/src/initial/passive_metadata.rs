// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the public construction data recovered from the original SDF.
use super::*;

impl InitialScene {
    pub(crate) fn passive_metadata_matches(&self, world: &World) -> bool {
        self.metadata_matches(world, |_| false)
    }
    pub(crate) fn trajectory_metadata_matches(&self, world: &World) -> bool {
        self.metadata_matches(world, |id| deformable::is_particle_visual(world, id))
    }
    fn metadata_matches(&self, world: &World, animated: impl Fn(u64) -> bool) -> bool {
        let physics = world.get_resource::<WorldPhysics>();
        if physics.map(|p| (&p.profiles, p.step_override))
            != self
                .0
                .world_physics
                .as_ref()
                .map(|p| (&p.profiles, p.step_override))
            || world
                .get_resource::<Gravity>()
                .map(|g| g.0.map(f64::to_bits))
                != Some(self.0.gravity.map(f64::to_bits))
        {
            return false;
        }
        self.0.nodes.iter().all(|node| {
            let Some(id) = entity_by_id(world, node.id.get()) else {
                // Retirement is checked independently against the sparse graph.
                return true;
            };
            let entity = world.entity(id);
            let anchor = entity.get::<ModelAnchor>();
            entity.get::<VisualGeometry>().map(|v| v.0.as_ref())
                == node.visual_geometry.as_ref().map(|v| v.0.as_ref())
                && entity.get::<VisualMaterial>().map(|v| v.0.as_ref())
                    == node.material.as_ref().map(|v| v.0.as_ref())
                && entity.get::<VisualProperties>().map(|v| &v.0)
                    == node.visual.as_ref().map(|v| &v.0)
                && entity.get::<LinkProperties>() == node.link_properties.as_ref()
                && entity.get::<CollisionDensity>().map(|d| d.0.to_bits())
                    == node.collision_density.map(|d| d.0.to_bits())
                && entity
                    .get::<LinkCenterOfMass>()
                    .map(|c| c.0.map(f64::to_bits))
                    == node.center.map(|c| c.0.map(f64::to_bits))
                && anchor.map(|a| a.link) == node.anchor.as_ref().map(|a| a.link)
                && anchor.zip(node.anchor.as_ref()).is_none_or(|(a, b)| {
                    crate::passive_checkpoint::same_pose(a.link_to_model, b.link_to_model)
                })
                && entity.contains::<AttachedPose>() == node.attached
                && (!node.attached
                    || animated(node.id.get())
                    || entity
                        .get::<LocalPose>()
                        .is_some_and(|p| crate::passive_checkpoint::same_pose(p.0, node.local.0)))
        })
    }
}
