// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Quasi-static whole-body torque compensation about the measured joint axes.
//! The planned foot wrenches are used only to compute internal actuator torques.
use crate::{BodyHandle, JointHandle, LoadError, Name, joint_posture::PostureController};
use glam::{DQuat, DVec2, DVec3};
use gz_physics::{BodyId, JointId, PhysicsBackend, PhysicsError};
use gz_sim_core::Simulation;
use std::collections::BTreeMap;

#[derive(Clone)]
struct Link {
    id: BodyId,
    mass: f64,
    local_com: DVec3,
}
#[derive(Clone)]
struct Axis {
    id: JointId,
    parent: usize,
    local_anchor: DVec3,
    local_axis: DVec3,
    children: Vec<usize>,
}
#[derive(Clone)]
pub(super) struct SupportControl {
    axes: Vec<Axis>,
    links: Vec<Link>,
    feet: [usize; 2],
}
impl SupportControl {
    pub fn bind(
        sdf: &sdformat::Sdf,
        model: &sdformat::Model,
        sim: &Simulation,
    ) -> Result<Self, LoadError> {
        let invalid = |cause: &str| LoadError::JointPosture {
            entity: model.scoped_name.clone(),
            reason: cause.into(),
        };
        let ids: BTreeMap<_, _> = sim
            .world()
            .iter_entities()
            .filter_map(|e| Some((e.get::<Name>()?.0.to_owned(), e.id())))
            .collect();
        let mut links = Vec::new();
        let mut index = BTreeMap::new();
        for link in &model.links {
            let name = &link.scoped_name;
            index.insert(name.as_str(), links.len());
            let rel = sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, name, name)?;
            links.push(Link {
                id: sim
                    .world()
                    .get::<BodyHandle>(
                        *ids.get(name)
                            .ok_or_else(|| invalid("missing gait link entity"))?,
                    )
                    .ok_or_else(|| invalid("missing gait link"))?
                    .id(),
                mass: link.inertial.mass,
                local_com: DVec3::from_array(rel.translation),
            });
        }
        let mut axes = Vec::new();
        let mut parents = vec![None; links.len()];
        let mut children = Vec::new();
        for j in &model.joints {
            let scoped = format!("{}::{}", model.scoped_name, j.name);
            let name = &scoped;
            let parent = sdf.resolve_name(&model.scoped_name, &j.parent)?;
            let child = sdf.resolve_name(&model.scoped_name, &j.child)?;
            let p = *index
                .get(parent.as_str())
                .ok_or_else(|| invalid("unbound parent in gait tree"))?;
            let c = *index
                .get(child.as_str())
                .ok_or_else(|| invalid("unbound child in gait tree"))?;
            if parents[c].is_some() || p == c {
                return Err(invalid("invalid gait tree ancestry"));
            }
            let frame = sdf.pose_of(name, &parent)?;
            let direction = sdf.resolve_axis(
                j.axis
                    .as_ref()
                    .ok_or_else(|| invalid("missing gait joint axis"))?,
                name,
                &parent,
            )?;
            axes.push(Axis {
                id: sim
                    .world()
                    .get::<JointHandle>(
                        *ids.get(name)
                            .ok_or_else(|| invalid("missing gait joint entity"))?,
                    )
                    .ok_or_else(|| invalid("missing native gait joint"))?
                    .id(),
                parent: p,
                local_anchor: DVec3::from_array(frame.translation),
                local_axis: DVec3::from_array(direction),
                children: vec![],
            });
            children.push(c);
            parents[c] = Some(p);
        }
        for (axis, child) in axes.iter_mut().zip(children) {
            for i in 0..links.len() {
                let mut current = Some(i);
                for _ in 0..=links.len() {
                    let Some(node) = current else { break };
                    if node == child {
                        axis.children.push(i);
                        current = None;
                        break;
                    }
                    current = parents[node];
                }
                if current.is_some() {
                    return Err(invalid("cyclic gait ancestry"));
                }
            }
        }
        let foot = |s| {
            index
                .get(format!("{}::{s}_foot", model.scoped_name).as_str())
                .copied()
                .ok_or_else(|| invalid("missing named left or right foot"))
        };
        Ok(Self {
            axes,
            links,
            feet: [foot("left")?, foot("right")?],
        })
    }
    pub(super) fn foot_centers(&self, be: &PhysicsBackend) -> Result<[DVec2; 2], PhysicsError> {
        let point = |i: usize| {
            be.state(self.links[i].id)
                .map(|s| DVec3::from_array(s.position).truncate())
                .ok_or(PhysicsError::MissingBody)
        };
        Ok([point(self.feet[0])?, point(self.feet[1])?])
    }
    pub fn apply(
        &self,
        ctl: &mut PostureController,
        be: &PhysicsBackend,
        target: DVec2,
        gravity: f64,
        swing_foot: Option<usize>,
    ) -> Result<(), PhysicsError> {
        let (com, v) = ctl.center_of_mass(be)?;
        let desired_a = (12. * (target - com.truncate()) - 6.5 * v.truncate()).clamp_length_max(2.);
        let apparent_g = (-desired_a).extend(-gravity);
        let mut positions = [DVec3::ZERO; 31];
        let mut rotations = [DQuat::IDENTITY; 31];
        let mut centers = [DVec3::ZERO; 31];
        for (i, link) in self.links.iter().enumerate() {
            let s = be.state(link.id).ok_or(PhysicsError::MissingBody)?;
            let p = DVec3::from_array(s.position);
            let q = DQuat::from_array(s.orientation);
            positions[i] = p;
            rotations[i] = q;
            centers[i] = p + q * link.local_com;
        }
        let f = self.feet.map(|i| positions[i].truncate());
        let projected = com.truncate() - com.z / gravity * desired_a;
        let across = f[0] - f[1];
        let weight = if let Some(swing) = swing_foot {
            if swing == 1 { 1. } else { 0. }
        } else {
            ((projected - f[1]).dot(across) / across.length_squared().max(1e-8)).clamp(0., 1.)
        };
        let shares = [weight, 1. - weight];
        let center = f[0] * weight + f[1] * (1. - weight);
        let shift =
            (projected - center).clamp(DVec2::new(-0.110, -0.052), DVec2::new(0.110, 0.052));
        let floor_points = f.map(|p| (p + shift).extend(0.));
        for axis in &self.axes {
            let rot = rotations[axis.parent];
            let pivot = positions[axis.parent] + rot * axis.local_anchor;
            let mut moment = DVec3::ZERO;
            for &i in &axis.children {
                moment += (centers[i] - pivot).cross(apparent_g * self.links[i].mass);
                for (side, &foot) in self.feet.iter().enumerate() {
                    if i == foot {
                        moment += (floor_points[side] - pivot)
                            .cross(-apparent_g * (ctl.mass() * shares[side]));
                    }
                }
            }
            ctl.set_feedforward(axis.id, -(rot * axis.local_axis).dot(moment))?;
        }
        Ok(())
    }
}
