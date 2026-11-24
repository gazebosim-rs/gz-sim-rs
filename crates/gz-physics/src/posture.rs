// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Torque-limited revolute posture and projected mass-center feedback shared by ordinary and managed worlds.
use crate::{BodyState, JointState};
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PostureMode {
    #[default]
    Balance,
    Posture,
    Passive,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PostureFeedback {
    #[default]
    None,
    X,
    NegativeY,
}
/// An actuator's cold SDF law. Joint identity is the stable public SDF entity ID.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PostureAxisLaw {
    pub joint_id: u64,
    pub reference_rad: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub max_effort_nm: f64,
    pub limits: [f64; 2],
    pub feedback: PostureFeedback,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MassCenterFeedback {
    pub reference: [f64; 3],
    pub gains: [f64; 2],
    pub max_correction: [f64; 2],
}
/// The order of links and actuators preserves authored floating-point summation order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PostureLaw {
    pub link_order: Vec<u64>,
    pub axes: Vec<PostureAxisLaw>,
    pub balance: MassCenterFeedback,
}
/// All command overrides are explicit for the interval. An absent row uses the cold SDF reference.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PostureSetpoint {
    pub joint_id: u64,
    pub target_rad: f64,
    pub feedforward_nm: f64,
    pub direct_effort_nm: Option<f64>,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PostureTargets {
    pub mode: PostureMode,
    pub balance_offset: [f64; 2],
    pub axes: Vec<PostureSetpoint>,
}
/// Spatial sum over actual native body COMs and COM velocities.
#[derive(Default)]
pub struct MassCenterSum {
    position: DVec3,
    velocity: DVec3,
}
impl MassCenterSum {
    pub fn add(&mut self, mass: f64, local_com: [f64; 3], state: BodyState) {
        let com = DVec3::from_array(state.position)
            + DQuat::from_array(state.orientation) * DVec3::from_array(local_com);
        self.position += mass * com;
        self.velocity += mass * DVec3::from_array(state.velocity);
    }
    pub fn finish(self, mass: f64) -> ([f64; 3], [f64; 3]) {
        (
            (self.position / mass).to_array(),
            (self.velocity / mass).to_array(),
        )
    }
}
impl MassCenterFeedback {
    pub fn correction(&self, position: [f64; 3], velocity: [f64; 3], offset: [f64; 2]) -> [f64; 2] {
        let e = DVec3::from_array(position)
            - DVec3::from_array(self.reference)
            - DVec3::new(offset[0], offset[1], 0.);
        let [kp, kd] = self.gains;
        let [xlim, ylim] = self.max_correction;
        [
            (kp * e.x + kd * velocity[0]).clamp(-xlim, xlim),
            (-kp * e.y - kd * velocity[1]).clamp(-ylim, ylim),
        ]
    }
}
impl PostureAxisLaw {
    pub fn command(
        &self,
        state: JointState,
        correction: [f64; 2],
        setting: Option<&PostureSetpoint>,
    ) -> [f64; 2] {
        let offset = match self.feedback {
            PostureFeedback::None => 0.,
            PostureFeedback::X => correction[0],
            PostureFeedback::NegativeY => correction[1],
        };
        let goal = setting.map_or(self.reference_rad, |s| s.target_rad);
        let target = (goal + offset).clamp(self.limits[0], self.limits[1]);
        let ff = setting.map_or(0., |s| s.feedforward_nm);
        let effort = setting
            .and_then(|s| s.direct_effort_nm)
            .unwrap_or(
                self.stiffness * (target - state.position) - self.damping * state.velocity + ff,
            )
            .clamp(-self.max_effort_nm, self.max_effort_nm);
        [target, effort]
    }
}
