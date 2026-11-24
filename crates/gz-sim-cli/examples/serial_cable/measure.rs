// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[derive(Debug, Serialize)]
pub(super) struct Report {
    cable_weight_n: f64,
    initial_source_weight_n: f64,
    mean_travel_jaw_up_n: [f64; 2],
    mean_travel_squeeze_n: [f64; 2],
    min_travel_squeeze_n: f64,
    travel_peak_station_n: f64,
    min_crossing_cable_gap_m: Option<f64>,
    ticks_over_bar: usize,
    peak_bar_n: f64,
    minimum_rigid_arm_bar_gap_m: f64,
    peak_travel_friction_bound_excess_n: f64,
    max_strain: f64,
    peak_momentum_residual_ns: f64,
    minimum_other_collider_gap_m: f64,
    pickup_lift_m: f64,
    max_hold_tip_sag_m: f64,
    max_spindle_rad: f64,
    peak_wrist_angles_rad: [f64; 2],
    travel_cm_y_m: f64,
    release_tick: Option<usize>,
    release_shelf_mean_n: Option<f64>,
    final_source_n: f64,
    final_shelf_n: f64,
    final_pad_up_n: [f64; 2],
    final_max_squeeze_n: f64,
    final_peak_particle_speed_m_s: f64,
    final_shelf_footprint_margin_m: f64,
    final_center_m: [f64; 3],
}
impl Report {
    pub(super) fn from_run(r: &[Sample]) -> Self {
        // The last 400 ms precede rail travel. The final 500 ms follow withdrawal.
        let hold = &r[5000..5400];
        let travel = &r[5400..9400];
        let final_window = &r[END - 500..END];
        let mean = |a: &[Sample], f: &dyn Fn(&Sample) -> f64| {
            a.iter().map(f).sum::<f64>() / a.len() as f64
        };
        let peak = |a: &[Sample], f: &dyn Fn(&Sample) -> f64| a.iter().map(f).fold(0., f64::max);
        let minimum = |a: &[Sample], f: &dyn Fn(&Sample) -> f64| {
            a.iter().map(f).fold(f64::INFINITY, f64::min)
        };
        let release = r.iter().position(|s| s.release_commanded);
        Self {
            cable_weight_n: WEIGHT,
            initial_source_weight_n: mean(&r[..50], &|s| s.source),
            mean_travel_jaw_up_n: std::array::from_fn(|i| mean(travel, &|s| s.jaw_up[i])),
            mean_travel_squeeze_n: std::array::from_fn(|i| mean(travel, &|s| s.squeeze[i])),
            min_travel_squeeze_n: minimum(travel, &|s| s.squeeze[0].min(s.squeeze[1])),
            travel_peak_station_n: peak(travel, &|s| s.source.max(s.shelf)),
            min_crossing_cable_gap_m: travel
                .iter()
                .filter(|s| s.near_bar)
                .map(|s| s.clearance)
                .reduce(f64::min),
            ticks_over_bar: travel.iter().filter(|s| s.near_bar).count(),
            minimum_rigid_arm_bar_gap_m: minimum(r, &|s| s.robot_bar_gap),
            peak_travel_friction_bound_excess_n: peak(travel, &|s| s.jaw_friction_excess_n),
            peak_bar_n: peak(r, &|s| s.bar),
            max_strain: peak(r, &|s| s.strain),
            peak_momentum_residual_ns: peak(r, &|s| s.residual),
            minimum_other_collider_gap_m: minimum(r, &|s| s.other_gap),
            pickup_lift_m: r[5399].center[2] - 0.525,
            max_hold_tip_sag_m: peak(hold, &|s| s.tip_sag),
            max_spindle_rad: peak(r, &|s| s.spindle),
            peak_wrist_angles_rad: std::array::from_fn(|i| peak(r, &|s| s.wrist[i].abs())),
            travel_cm_y_m: r[9399].center[1],
            release_tick: release.map(|i| i + 1),
            release_shelf_mean_n: release.map(|i| mean(&r[i - 30..i], &|s| s.shelf)),
            final_source_n: mean(final_window, &|s| s.source),
            final_shelf_n: mean(final_window, &|s| s.shelf),
            final_pad_up_n: std::array::from_fn(|i| mean(final_window, &|s| s.jaw_up[i])),
            final_max_squeeze_n: peak(final_window, &|s| s.squeeze[0].max(s.squeeze[1])),
            final_peak_particle_speed_m_s: peak(final_window, &|s| s.peak_speed),
            final_shelf_footprint_margin_m: minimum(final_window, &|s| s.shelf_margin),
            final_center_m: r.last().unwrap().center,
        }
    }
    pub(super) fn qualify(&self, name: &str) -> Result<()> {
        let common = (self.initial_source_weight_n - WEIGHT).abs() < 0.003
            && self.minimum_rigid_arm_bar_gap_m > 0.002
            && self.minimum_other_collider_gap_m > 0.005
            && self.max_strain < 0.02
            && self.peak_momentum_residual_ns < 1e-8
            && self.max_spindle_rad > 1.80
            && self.peak_wrist_angles_rad[1] > 0.02
            && self.final_peak_particle_speed_m_s < 0.001;
        let rested = (self.final_shelf_n - WEIGHT).abs() < 0.003
            && self.final_source_n.abs() < 1e-6
            && self.final_max_squeeze_n < 1e-4
            && self.final_shelf_footprint_margin_m > 0.
            && self.release_shelf_mean_n.is_some_and(|v| v > 0.9 * WEIGHT);
        let carried = self.pickup_lift_m > 0.10
            && self.max_hold_tip_sag_m > 0.01
            && self.min_travel_squeeze_n > 0.05
            && (self.travel_cm_y_m - 0.46).abs() < 0.012
            && self.ticks_over_bar > 100;
        let accepted = match name {
            "clear" => {
                rested
                    && carried
                    && self.min_crossing_cable_gap_m.is_some_and(|g| g > 0.02)
                    && self.peak_travel_friction_bound_excess_n < 0.005
                    && self.peak_bar_n < 1e-7
                    && self.travel_peak_station_n < 1e-7
                    && (self.mean_travel_jaw_up_n.iter().sum::<f64>() - WEIGHT).abs()
                        < 0.01 * WEIGHT
            }
            "low" => rested && carried && self.peak_bar_n > 0.01,
            "open" => {
                self.release_tick.is_none()
                    && self.final_shelf_n.abs() < 1e-6
                    && (self.final_source_n - WEIGHT).abs() < 0.003
                    && self.travel_cm_y_m.abs() < 0.05
                    && self.pickup_lift_m.abs() < 0.01
                    && self.final_max_squeeze_n < 1e-5
            }
            _ => false,
        };
        if common && accepted {
            Ok(())
        } else {
            Err(format!("cable qualification failed for {name}: {self:?}").into())
        }
    }
}
