// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;

#[derive(Clone)]
pub(super) struct HeadingControl {
    pelvis: BodyId,
    pelvis_start: f64,
    hips: [JointId; 2],
    feet: [BodyId; 2],
    feet_start: [f64; 2],
    bounds: [[f64; 2]; 2],
    gain: f64,
}

fn heading(be: &PhysicsBackend, id: BodyId) -> Result<f64, PhysicsError> {
    let state = be.state(id).ok_or(PhysicsError::MissingBody)?;
    let forward = DQuat::from_array(state.orientation) * DVec3::X;
    Ok(forward.y.atan2(forward.x))
}

fn error(current: f64, reference: f64) -> f64 {
    let delta = current - reference;
    delta.sin().atan2(delta.cos())
}

impl HeadingControl {
    pub(super) fn bind(
        sdf: &sdformat::Sdf,
        model: &sdformat::Model,
        world: &sdformat::World,
        sim: &Simulation,
        pelvis: BodyId,
        feet: [BodyId; 2],
        gain: f64,
    ) -> Result<Self, LoadError> {
        let hip = |side| -> Result<(JointId, [f64; 2]), LoadError> {
            let name = format!("{side}_hip_yaw");
            let joint = model
                .joints
                .iter()
                .find(|j| j.name == name)
                .ok_or(PhysicsError::MissingJoint)?;
            let axis = joint.axis.as_ref().ok_or(PhysicsError::MissingJoint)?;
            let scoped = format!("{}::{name}", model.scoped_name);
            let direction = DVec3::from_array(sdf.resolve_axis(axis, &scoped, &world.name)?);
            if (direction - DVec3::Z).length() > 1e-8 || axis.lower > 0. || axis.upper < 0. {
                return Err(PhysicsError::InvalidValue.into());
            }
            let handle = sim
                .world()
                .iter_entities()
                .find_map(|e| {
                    (e.get::<Name>()?.0 == scoped)
                        .then(|| e.get::<JointHandle>().map(|h| h.id()))
                        .flatten()
                })
                .ok_or(PhysicsError::MissingJoint)?;
            Ok((handle, [axis.lower, axis.upper]))
        };
        let (left, lb) = hip("left")?;
        let (right, rb) = hip("right")?;
        let be = sim.world().resource::<PhysicsBackend>();
        Ok(Self {
            pelvis,
            pelvis_start: heading(be, pelvis)?,
            hips: [left, right],
            feet,
            feet_start: [heading(be, feet[0])?, heading(be, feet[1])?],
            bounds: [lb, rb],
            gain,
        })
    }
    pub(super) fn apply(
        &self,
        controller: &mut PostureController,
        be: &PhysicsBackend,
        target: &trajectory::Target,
    ) -> Result<(), PhysicsError> {
        let root = error(heading(be, self.pelvis)?, self.pelvis_start);
        for (side, joint) in self.hips.into_iter().enumerate() {
            let airborne = target.heading_swing_weight(side);
            let foot = error(heading(be, self.feet[side])?, self.feet_start[side]);
            // The grounded child's yaw acts back on the pelvis. In swing, direct
            // the unloaded foot toward its authored forward vector before landing.
            let correction = self.gain * ((1. - airborne) * root - airborne * foot);
            let q = be.joint_state(joint).ok_or(PhysicsError::MissingJoint)?.position;
            let [lo, hi] = self.bounds[side];
            controller.set_target(joint, (q + correction).clamp(lo, hi))?;
        }
        Ok(())
    }
}
