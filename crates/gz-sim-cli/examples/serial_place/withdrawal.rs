// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

// The shelf first moves down and sideways, waits clear of the pickup tray,
// then returns below the parcel and rises back to its original transfer height.
pub(super) const PATH: &str = r#"
        <waypoint time_ns="9224000000" displacement="-0.30 0 0" velocity="0 0 0"/>
        <waypoint time_ns="9324000000" displacement="-0.28 0 -0.05" velocity="0 0 0"/>
        <waypoint time_ns="9624000000" displacement="-0.1 0 -0.05" velocity="0 0 0"/>
        <waypoint time_ns="10300000000" displacement="-0.1 0 -0.05" velocity="0 0 0"/>
        <waypoint time_ns="10600000000" displacement="-0.30 0 -0.05" velocity="0 0 0"/>
        <waypoint time_ns="10900000000" displacement="-0.30 0 0" velocity="0 0 0"/>
"#;

#[derive(Debug, Default, PartialEq, Serialize)]
pub(super) struct Receipt {
    pub loss_tick: Option<usize>,
    pub close_tick: Option<usize>,
    pub shelf_return_tick: Option<usize>,
    pub retry_tick: Option<usize>,
    pub final_open_tick: Option<usize>,
    pub max_drop_before_return_mm: f64,
    pub unsupported_before_return_ms: usize,
    pub peak_downward_before_return_m_s: f64,
    pub pad_loaded_at_retract: bool,
    pub suspended_on_pads_ms: usize,
    pub cleared_receiver_on_pads_ms: usize,
    pub held_clear_at_return: bool,
    pub recovery_spindle_target_rad: Option<f64>,
}

pub(super) struct GraspSample {
    pub drop_mm: f64,
    pub vertical_speed: f64,
    pub bottom_z: f64,
    pub normal: [f64; 4],
    pub spindle_rad: f64,
}

pub(super) struct Recovery {
    pub receipt: Receipt,
    delay: usize,
    closed_loop: bool,
    unloaded_ticks: usize,
    seen_shelf_clear: bool,
    clear_hold_ticks: usize,
}

impl Recovery {
    pub fn new(trial: Trial, delay: usize) -> Self {
        Self {
            receipt: Receipt::default(),
            delay,
            closed_loop: trial == Trial::Withdrawal,
            unloaded_ticks: 0,
            seen_shelf_clear: false,
            clear_hold_ticks: 0,
        }
    }
    pub fn retrying(&self) -> bool {
        self.receipt.retry_tick.is_some() && self.receipt.final_open_tick.is_none()
    }

    pub fn observe(
        &mut self,
        tick: usize,
        start: usize,
        sample: &GraspSample,
        shelf: ([f64; 3], [f64; 3]),
    ) {
        if self.receipt.shelf_return_tick.is_some() {
            return;
        }
        let normal = sample.normal;
        let clear = shelf.0[0] > 0.12;
        self.seen_shelf_clear |= clear;
        let held = normal[0] + normal[1] == 0. && normal[2] > 0. && normal[3] > 0.;
        let above = held && sample.bottom_z > 0.585;
        if clear && held {
            self.receipt.suspended_on_pads_ms += 1;
            self.receipt.cleared_receiver_on_pads_ms += usize::from(above);
        }
        if self.seen_shelf_clear && above {
            self.clear_hold_ticks += 1;
        } else {
            self.clear_hold_ticks = 0;
        }
        let parked = shelf.0[0].abs() < 0.0005
            && (shelf.0[2] - 0.574).abs() < 0.0005
            && glam::DVec3::from(shelf.1).length() < 0.002;
        if self.seen_shelf_clear && parked {
            self.receipt.shelf_return_tick = Some(tick);
            self.receipt.held_clear_at_return = self.clear_hold_ticks >= 100;
            return;
        }
        self.receipt.peak_downward_before_return_m_s = self
            .receipt
            .peak_downward_before_return_m_s
            .max(-sample.vertical_speed);
        self.receipt.max_drop_before_return_mm =
            self.receipt.max_drop_before_return_mm.max(sample.drop_mm);
        self.receipt.unsupported_before_return_ms += usize::from(normal.iter().all(|j| *j == 0.));
        if normal[0] < 0.0002 * WEIGHT_N {
            self.unloaded_ticks += 1;
        } else {
            self.unloaded_ticks = 0;
        }
        if tick > start && self.unloaded_ticks >= 2 {
            self.receipt.loss_tick.get_or_insert(tick);
        }
    }
    pub fn commands(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        start: usize,
        joints: [u64; 2],
        spindle: u64,
        sample: &GraspSample,
    ) -> Result<()> {
        let normal = sample.normal;
        let age = tick - start;
        if let Some(final_tick) = self.receipt.final_open_tick {
            if tick == final_tick + 350 {
                // A low catch seats the load at a higher spindle angle. Return
                // to at least the height that already cleared the caught load.
                let retract = self
                    .receipt
                    .recovery_spindle_target_rad
                    .unwrap_or(1.6)
                    .max(1.6);
                set_joint_controller_command(sim, spindle, retract)?;
            }
            return Ok(());
        }
        if let Some(close) = self.receipt.close_tick {
            if tick == close + 100 {
                self.receipt.pad_loaded_at_retract = normal[2] > 0. && normal[3] > 0.;
                if self.receipt.pad_loaded_at_retract {
                    // Lift a slipped but still gripped parcel clear of the
                    // returning top. Keep the target within the screw travel.
                    let lead_m_per_rad = 0.5 / std::f64::consts::TAU;
                    let target = (sample.spindle_rad + (0.60 - sample.bottom_z) / lead_m_per_rad)
                        .clamp(1.4, 1.95);
                    self.receipt.recovery_spindle_target_rad = Some(target);
                    set_joint_controller_command(sim, spindle, target)?;
                }
            }
            if self.receipt.shelf_return_tick == Some(tick) && self.receipt.held_clear_at_return {
                self.receipt.retry_tick = Some(tick);
                set_joint_controller_command(sim, spindle, -0.018)?;
            }
            return Ok(());
        }
        let react = self.closed_loop
            && self
                .receipt
                .loss_tick
                .is_some_and(|t| tick >= t + self.delay);
        if react || age <= 100 {
            let force = if react { 0. } else { -20. * age as f64 / 100. };
            JointForceCommand::set_batch(sim, joints.map(|id| (id, force)))?;
            if react {
                self.receipt.close_tick = Some(tick);
            }
        }
        if !self.closed_loop && age == 350 {
            set_joint_controller_command(sim, spindle, 1.6)?;
        }
        Ok(())
    }
}
