// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[derive(Serialize)]
struct Snag {
    detection_tick: usize,
    first_retreat_command_tick: usize,
    peak_force_n: f64,
    cumulative_force_magnitude_ns: f64,
    last_loaded_tick: usize,
    max_forward_overshoot_m: f64,
    retreat_distance_m: f64,
}
#[derive(Serialize)]
pub(super) struct Measurement {
    snags: Vec<Snag>,
    max_spindle_rad: f64,
    minimum_robot_bar_gap_m: f64,
    minimum_unenrolled_box_gap_m: f64,
    maximum_axial_strain: f64,
    peak_momentum_residual_ns: f64,
    minimum_retained_squeeze_n: f64,
    mean_airborne_gravity_support_n: [f64; 2],
    second_pass_minimum_crossing_gap_m: Option<f64>,
    opening_tick: usize,
    opening_support_dwell_mean_n: f64,
    final_center_m: [f64; 3],
    final_support_load_n: [f64; 2],
    final_pad_squeeze_n: f64,
    final_peak_material_speed_m_s: f64,
}
pub(super) fn measure(scene: Scene, rows: &[Trace], events: &[Event]) -> Result<Measurement> {
    let mut snags = vec![];
    for w in events.windows(2).filter(|w| w[0].stage == Stage::Backoff) {
        let start = w[0];
        let end = w[1];
        let ticks = &rows[start.tick - 1..end.tick];
        let last = ticks
            .iter()
            .rfind(|r| r.sample.bar > 1e-9)
            .ok_or("recovery lacks a physical trigger")?;
        snags.push(Snag {
            detection_tick: start.tick,
            first_retreat_command_tick: start.tick + 1,
            peak_force_n: ticks.iter().map(|v| v.sample.bar).fold(0., f64::max),
            cumulative_force_magnitude_ns: ticks.iter().map(|v| v.sample.bar * DT).sum(),
            last_loaded_tick: last.tick,
            max_forward_overshoot_m: ticks
                .iter()
                .map(|v| v.y - start.carriage_y_m)
                .fold(0., f64::max),
            retreat_distance_m: start.carriage_y_m - end.carriage_y_m,
        });
    }
    let retry = events.iter().filter(|e| e.stage == Stage::Transport).nth(1);
    let gap = retry.and_then(|e| {
        rows[e.tick..]
            .iter()
            .take_while(|r| r.stage == Stage::Transport)
            .filter(|r| r.sample.near_bar)
            .map(|r| r.sample.clearance)
            .reduce(f64::min)
    });
    let source = matches!(scene, Scene::TooHigh);
    let open = events
        .iter()
        .find(|e| e.stage == Stage::Opening)
        .ok_or("load was never set down")?
        .tick;
    let dwell = rows[open - 30..open].iter().map(|r| {
        if source {
            r.sample.source
        } else {
            r.sample.shelf
        }
    });
    if dwell.clone().any(|v| v <= 0.9 * WEIGHT) {
        return Err("opened before the receiving support took the load".into());
    }
    let last = &rows[rows.len() - 500..];
    let result = Measurement {
        snags,
        max_spindle_rad: rows.iter().map(|v| v.sample.spindle).fold(0., f64::max),
        minimum_robot_bar_gap_m: rows
            .iter()
            .map(|v| v.sample.robot_bar_gap)
            .fold(f64::INFINITY, f64::min),
        minimum_unenrolled_box_gap_m: rows
            .iter()
            .map(|v| v.sample.other_gap)
            .fold(f64::INFINITY, f64::min),
        maximum_axial_strain: rows.iter().map(|v| v.sample.strain).fold(0., f64::max),
        peak_momentum_residual_ns: rows.iter().map(|v| v.sample.residual).fold(0., f64::max),
        minimum_retained_squeeze_n: rows
            .iter()
            .filter(|r| {
                matches!(
                    r.stage,
                    Stage::Transport | Stage::Backoff | Stage::Raising | Stage::Returning
                )
            })
            .flat_map(|v| v.sample.squeeze)
            .fold(f64::INFINITY, f64::min),
        mean_airborne_gravity_support_n: std::array::from_fn(|i| {
            rows[5000..5400]
                .iter()
                .map(|r| r.sample.jaw_up[i])
                .sum::<f64>()
                / 400.
        }),
        second_pass_minimum_crossing_gap_m: gap,
        opening_tick: open + 1,
        opening_support_dwell_mean_n: dwell.sum::<f64>() / 30.,
        final_center_m: last.last().unwrap().sample.center,
        final_support_load_n: [
            last.iter().map(|r| r.sample.source).sum::<f64>() / 500.,
            last.iter().map(|r| r.sample.shelf).sum::<f64>() / 500.,
        ],
        final_pad_squeeze_n: last
            .iter()
            .flat_map(|r| r.sample.squeeze)
            .fold(0., f64::max),
        final_peak_material_speed_m_s: last.iter().map(|r| r.sample.peak_speed).fold(0., f64::max),
    };
    let expected = match scene {
        Scene::Clear => 0,
        Scene::Snag => 1,
        Scene::TooHigh => 2,
    };
    let expected_y = if source { 0. } else { 0.46 };
    let target = usize::from(!source);
    let valid = result.snags.len() == expected
        && (result.final_center_m[1] - expected_y).abs() < 0.025
        && (result.final_support_load_n[target] - WEIGHT).abs() < 0.001
        && result.final_support_load_n[1 - target].abs() < 1e-5
        && result.final_pad_squeeze_n < 1e-4
        && result.final_peak_material_speed_m_s < 1e-4
        && result.maximum_axial_strain < 0.005
        && result.peak_momentum_residual_ns < 1e-8
        && result.minimum_robot_bar_gap_m > 0.002
        && result.minimum_unenrolled_box_gap_m > 0.01
        && result.minimum_retained_squeeze_n > 0.1
        && result.max_spindle_rad < 2.81
        && (result.mean_airborne_gravity_support_n.iter().sum::<f64>() - WEIGHT).abs() < 0.002
        && result
            .snags
            .iter()
            .all(|r| r.peak_force_n > controller::FORCE_TRIGGER && r.retreat_distance_m > 0.075)
        && (!matches!(scene, Scene::Snag) || gap.is_some_and(|g| g > 0.003))
        && (!source || events.iter().any(|e| e.stage == Stage::Returning));
    if !valid {
        return Err(format!(
            "snag qualification failed: {}",
            serde_json::to_string(&result)?
        )
        .into());
    }
    Ok(result)
}
