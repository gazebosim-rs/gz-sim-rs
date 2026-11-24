// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::EntityKind;
/// Physical collision bounds from the SDF and the committed Bevy collision pose.
pub(super) struct BoxShape {
    entity: Entity,
    half: DVec3,
}
impl BoxShape {
    pub(super) fn find(sim: &Simulation, sdf: &sdformat::Sdf, link_name: &str) -> Result<Self> {
        let link = sdf.worlds[0]
            .models
            .iter()
            .flat_map(|m| &m.links)
            .find(|l| l.scoped_name == format!("default::{link_name}"))
            .ok_or("missing SDF link")?;
        let [c] = link.collisions.as_slice() else {
            return Err("expected one collision per measured link".into());
        };
        let sdformat::Geometry::Box { size } = c.geometry else {
            return Err("expected collision box".into());
        };
        let name = format!("{}::{}", link.scoped_name, c.name);
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<EntityKind>() == Some(&EntityKind::Collision)
                    && e.get::<Name>().is_some_and(|n| n.0 == name)
            })
            .ok_or("missing collision pose")?
            .id();
        Ok(Self {
            entity,
            half: DVec3::from(size) * 0.5,
        })
    }
    pub(super) fn at(&self, sim: &Simulation) -> Result<Placed> {
        let p = sim
            .world()
            .get::<WorldPose>(self.entity)
            .ok_or("missing current collision pose")?;
        Ok(Placed {
            center: p.translation.into(),
            inverse: DQuat::from_array(p.rotation).inverse(),
            half: self.half,
        })
    }
}
pub(super) struct Placed {
    center: DVec3,
    inverse: DQuat,
    half: DVec3,
}
impl Placed {
    pub(super) fn box_gap(&self, other: &Self) -> Result<f64> {
        use parry3d_f64::{
            math::{Pose, Rotation, Vector},
            query,
            shape::Cuboid,
        };
        let pose = |p: &Self| {
            Pose::from_parts(
                Vector::from_array(p.center.to_array()),
                Rotation::from_array(p.inverse.inverse().to_array()),
            )
        };
        let shape = |p: &Self| Cuboid::new(Vector::from_array(p.half.to_array()));
        Ok(query::distance(&pose(self), &shape(self), &pose(other), &shape(other))?.distance)
    }
    pub(super) fn distances(&self, p: DVec3) -> DVec3 {
        (self.inverse * (p - self.center)).abs() - self.half
    }
    pub(super) fn clearance(&self, p: DVec3) -> f64 {
        let q = self.distances(p);
        q.max(DVec3::ZERO).length() + q.max_element().min(0.) - RADIUS
    }
}
