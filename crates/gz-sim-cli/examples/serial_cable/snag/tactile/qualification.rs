// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Serialize)]
struct Alarm {
    first_obstacle_tick: usize,
    detection_tick: usize,
    first_retreat_command_tick: usize,
    filtered_drag_n: f64,
    latency_ms: usize,
}

pub(super) fn measure(fixture: Fixture, rows: &[Log], baseline: &[String]) -> Result<Value> {
    let last = &rows[rows.len() - 500..];
    let first_contact = rows.iter().find(|r| r.truth.bar > 1e-8).map(|r| r.tick);
    let mut alarms = vec![];
    let mut open = None;
    let mut lost = None;
    let mut traversal_start = 0;
    for pair in rows.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        if a.stage == b.stage {
            continue;
        }
        match b.stage {
            Stage::Transport => traversal_start = a.tick,
            Stage::Backoff => {
                let onset = rows[traversal_start..a.tick]
                    .iter()
                    .find(|r| r.truth.bar > 1e-8)
                    .ok_or("gripper alarm without an independently measured contact")?;
                alarms.push(Alarm {
                    first_obstacle_tick: onset.tick,
                    detection_tick: a.tick,
                    first_retreat_command_tick: b.tick,
                    filtered_drag_n: a.signal.drag_n,
                    latency_ms: a.tick - onset.tick,
                });
            }
            Stage::Opening => open = Some(a.tick),
            Stage::LostGrip => lost = Some(a.tick),
            _ => {}
        }
    }
    let prefix = match fixture {
        Fixture::Clear => rows.len(),
        Fixture::Fast => 5400,
        Fixture::Lost => 6049,
        _ => first_contact.ok_or("fixture missed the physical cable")? - 1,
    };
    if !rows[..prefix]
        .iter()
        .map(|r| &r.truth.particle_state_sha256)
        .eq(baseline[..prefix].iter())
    {
        return Err("unmatched incoming cable particles".into());
    }
    let transport = || rows.iter().filter(|r| r.stage == Stage::Transport);
    let peak_drag = transport().map(|r| r.signal.drag_n).fold(0., f64::max);
    let peak_accel = transport()
        .map(|r| r.signal.rail_acceleration.abs())
        .fold(0., f64::max);
    let peak_speed = transport()
        .map(|r| r.jaws.rail_speed_m_s.abs())
        .fold(0., f64::max);
    let lift_drag = rows[..5400]
        .iter()
        .map(|r| r.signal.drag_n)
        .fold(0., f64::max);
    let bar_ticks = rows.iter().filter(|r| r.truth.bar > 1e-8).count();
    let impulse = rows.iter().map(|r| r.truth.bar * DT).sum::<f64>();
    let peak_bar = rows.iter().map(|r| r.truth.bar).fold(0., f64::max);
    let strain = rows.iter().map(|r| r.truth.strain).fold(0., f64::max);
    let momentum = rows.iter().map(|r| r.truth.residual).fold(0., f64::max);
    let robot_gap = rows
        .iter()
        .map(|r| r.truth.robot_bar_gap)
        .fold(f64::INFINITY, f64::min);
    let pickup_load = rows[5000..5400]
        .iter()
        .map(|r| r.jaws.force_on_payload_n[2])
        .sum::<f64>()
        / 400.;
    let return_to_source = matches!(fixture, Fixture::Limit | Fixture::Lost);
    let support = |r: &Log| {
        if return_to_source {
            r.truth.source
        } else {
            r.truth.shelf
        }
    };
    let rest_load = last.iter().map(support).sum::<f64>() / 500.;
    let rest_speed = last.iter().map(|r| r.truth.peak_speed).fold(0., f64::max);
    let support_dwell = open.map(|i| rows[i - 30..i].iter().map(support).sum::<f64>() / 30.);
    let dwell_min = open.map(|i| {
        rows[i - 30..i]
            .iter()
            .map(support)
            .fold(f64::INFINITY, f64::min)
    });
    let stop_distance = lost.map(|i| {
        rows[i..]
            .iter()
            .map(|r| r.jaws.rail_y_m - rows[i - 1].jaws.rail_y_m)
            .fold(0., f64::max)
    });
    let retry_gap = transport()
        .filter(|r| r.tick > traversal_start && r.truth.near_bar)
        .map(|r| r.truth.clearance)
        .reduce(f64::min);
    let expected_alarms = match fixture {
        Fixture::Snag => 1,
        Fixture::Limit => 2,
        _ => 0,
    };
    let fixture_ok = match fixture {
        Fixture::Clear => bar_ticks == 0 && peak_drag < 0.02,
        Fixture::Fast => bar_ticks == 0 && peak_speed > 0.4 && peak_accel > 1.0 && peak_drag < 0.02,
        Fixture::Rub => bar_ticks >= 50 && impulse > 0.001 && peak_drag < sensor::SNAG_N,
        Fixture::Snag => retry_gap.is_some_and(|g| g > 0.003),
        Fixture::Limit => rows.iter().any(|r| r.stage == Stage::Returning),
        Fixture::Lost => {
            lost.is_some_and(|t| rows[t - 1].signal.pad_loss_ticks == 8 && t < 6075)
                && stop_distance.is_some_and(|d| d < 0.01)
                && open.is_none()
                && bar_ticks == 0
        }
    };
    let physically_placed = match fixture {
        Fixture::Lost => true,
        _ => {
            dwell_min.is_some_and(|n| n > 0.85 * WEIGHT)
                && support_dwell.is_some_and(|n| n > 0.90 * WEIGHT)
                && lost.is_none()
        }
    };
    if !fixture_ok
        || !physically_placed
        || alarms.len() != expected_alarms
        || alarms
            .iter()
            .any(|a| a.latency_ms > 180 || a.filtered_drag_n <= sensor::SNAG_N)
        || (rest_load - WEIGHT).abs() > 0.002
        || rest_speed > 1e-4
        || (pickup_load - WEIGHT).abs() > 0.002
        || robot_gap < 0.002
        || strain > 0.005
        || momentum > 1e-8
    {
        return Err(format!("gripper case {} failed qualification; {} alarms, {bar_ticks} obstacle ticks, {support_dwell:?} N setdown, {rest_load} N rest, {strain} strain",fixture.name(),alarms.len()).into());
    }
    Ok(
        json!({"case":fixture.name(),"reset_identical_ticks":rows.len(),"matched_clear_prefix_ticks":prefix,"alarms":alarms,
      "peak_transport_drag_n":peak_drag,"peak_lift_drag_n":lift_drag,"peak_transport_rail_acceleration_m_s2":peak_accel,"peak_transport_speed_m_s":peak_speed,
      "first_obstacle_tick":first_contact,"obstacle_loaded_ticks":bar_ticks,"obstacle_impulse_magnitude_sum_ns":impulse,"peak_obstacle_n":peak_bar,
      "minimum_rigid_robot_obstacle_gap_m":robot_gap,"last_pass_crossing_gap_m":retry_gap,"peak_axial_strain":strain,"peak_momentum_residual_ns":momentum,
      "held_weight_n":pickup_load,"first_opening_command_tick":open.map(|t|t+1),"placement_dwell_mean_n":support_dwell,"placement_dwell_min_n":dwell_min,
      "first_failed_squeeze_tick":lost.map(|t|t-7),"lost_grip_detection_tick":lost,"first_halt_command_tick":lost.map(|t|t+1),"maximum_forward_stop_distance_m":stop_distance,
      "final_center_m":rows.last().unwrap().truth.center,"rest_support":if return_to_source{"source"}else{"shelf"},"rest_mean_load_n":rest_load,"rest_peak_particle_speed_m_s":rest_speed}),
    )
}
