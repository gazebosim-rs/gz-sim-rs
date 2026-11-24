// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
/// No material kinematics, obstacle or station channel enters the observer.
#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize)]
pub(super) struct JawReadings {
    pub squeeze_n: [f64; 2],
    pub force_on_payload_n: [f64; 3],
    pub rail_y_m: f64,
    pub rail_speed_m_s: f64,
    pub spindle_rad: f64,
}
impl JawReadings {
    pub(super) fn read(sim: &Simulation, robot: &Robot) -> Result<Self> {
        let mut s = Self::default();
        let mut sum = DVec3::ZERO;
        for (i, pad) in robot.pads.iter().enumerate() {
            let w = deformable::articulated_pad_wrench(sim, pad.id).ok_or("missing pad wrench")?;
            let rot = sim
                .world()
                .get::<WorldPose>(pad.entity)
                .ok_or("missing pad frame")?
                .rotation;
            let f = DVec3::from(w.force_n);
            let sign = if i == 0 { -1. } else { 1. };
            s.squeeze_n[i] = f.dot(sign * (DQuat::from_array(rot) * DVec3::X));
            sum -= f;
        }
        let rail = sim
            .world()
            .get::<JointSample>(robot.rail.entity)
            .ok_or("missing rail encoder")?
            .state;
        s.force_on_payload_n = sum.into();
        s.rail_y_m = rail.position;
        s.rail_speed_m_s = rail.velocity;
        s.spindle_rad = sim
            .world()
            .get::<JointSample>(robot.spindle.entity)
            .ok_or("missing screw encoder")?
            .state
            .position;
        Ok(s)
    }
}
#[derive(Default)]
pub(super) struct Observer {
    previous_rail_speed: f64,
    filtered_drag: f64,
    loss_ticks: usize,
}
#[derive(Clone, Copy, Serialize, PartialEq, Default)]
pub(super) struct Signal {
    pub drag_n: f64,
    pub raw_drag_n: f64,
    pub rail_acceleration: f64,
    pub pad_loss_ticks: usize,
}
pub(super) const SNAG_N: f64 = 0.040;
impl Observer {
    pub(super) fn update(&mut self, r: &JawReadings) -> (controller::Feedback, Signal) {
        let a = (r.rail_speed_m_s - self.previous_rail_speed) / DT;
        self.previous_rail_speed = r.rail_speed_m_s;
        let raw = r.force_on_payload_n[1] - (WEIGHT / 9.81) * a;
        self.filtered_drag += (raw - self.filtered_drag) * 0.08;
        let vertical = r.force_on_payload_n[2];
        let min = r.squeeze_n[0].min(r.squeeze_n[1]);
        if min < 0.3 {
            self.loss_ticks += 1;
        } else {
            self.loss_ticks = 0;
        }
        (
            controller::Feedback {
                carried: min > 0.3 && (vertical - WEIGHT).abs() < 0.05,
                snag: self.filtered_drag > SNAG_N,
                clear: self.filtered_drag.abs() < 0.015,
                supported: min > 0.3 && vertical < 0.08 * WEIGHT,
                lost: self.loss_ticks >= 8,
                load_n: self.filtered_drag,
                spindle_rad: r.spindle_rad,
            },
            Signal {
                drag_n: self.filtered_drag,
                raw_drag_n: raw,
                rail_acceleration: a,
                pad_loss_ticks: self.loss_ticks,
            },
        )
    }
}
