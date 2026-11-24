// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{FaceBearingAdvance, FaceOverlap, FrictionlessFaceSupport};
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceBearingReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub normal: [f64; 3],
    pub normal_force: f64,
    pub normal_impulse: f64,
    /// Pressure centroid is constant in material sole coordinates throughout.
    pub normalized_sole_center: [f64; 2],
    pub start_pressure_world: [f64; 3],
    pub end_pressure_world: [f64; 3],
    /// Intersection polygons in the finite pad's own top-face coordinates.
    pub start_overlap: FaceOverlap,
    pub end_overlap: FaceOverlap,
    pub angular_impulse_about_com: [f64; 3],
}
impl Frame {
    pub(super) fn face(
        self,
        b: &RapierBackend,
        t0: Duration,
        dt: Duration,
        corners: [Vector; 4],
    ) -> Result<Option<PreparedBearing>, PhysicsError> {
        let axes = [Vector::X, Vector::Y].map(|a| self.boxes.pad_pose.rotation * a);
        let model = FrictionlessFaceSupport {
            mass: self.mass,
            inertia_body: self.inertia,
            corners,
            pad_axes: axes,
            pad_half: self.boxes.pad_half.truncate(),
            gravity: self.gravity,
        };
        let Some(bound) = model.bind(self.start)? else {
            return Ok(None);
        };
        let seconds = dt.as_secs_f64();
        let FaceBearingAdvance::Supported(end) = bound.advance(seconds)? else {
            return Ok(None);
        };
        let begin = bound.sample(0.)?;
        let center_speed = self.start.velocity.length() + seconds * self.gravity.length();
        let tree = b
            .world
            .multibody_joints
            .get_multibody(self.owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        if !b.swept_locked_neighborhood(
            tree,
            self.owner,
            self.boxes,
            self.com,
            self.gravity,
            self.tolerance,
            center_speed,
            self.start.angular_velocity.length(),
            dt,
        )? {
            return Ok(None);
        }
        let receipt = FaceBearingReceipt {
            foot: self.foot,
            support: self.pad,
            start: t0,
            duration: dt,
            normal: self.normal.to_array(),
            normal_force: end.normal_force,
            normal_impulse: end.normal_impulse,
            normalized_sole_center: end.sole_center.to_array(),
            start_pressure_world: (self.top + begin.center_of_pressure).to_array(),
            end_pressure_world: (self.top + self.deck_velocity * seconds + end.center_of_pressure)
                .to_array(),
            start_overlap: begin.overlap,
            end_overlap: end.overlap,
            angular_impulse_about_com: end.angular_impulse_about_com.to_array(),
        };
        Ok(Some(PreparedBearing {
            colliders: self.boxes.colliders,
            motion: self.motion(end.state, seconds),
            receipt: BearingResult::Face(Box::new(receipt)),
        }))
    }
}
