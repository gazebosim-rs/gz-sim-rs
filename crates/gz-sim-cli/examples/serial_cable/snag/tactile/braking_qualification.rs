// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

pub(super) fn measure(name: &str, rows: &[Log]) -> Result<Value> {
    let mut arrests = vec![];
    let mut begin = 0;
    let mut opening = None;
    let mut alarms = 0;
    let mut grip_losses = 0;
    let mut retry_gap = None;
    let mut return_command = false;
    for (i, p) in rows.windows(2).enumerate() {
        let (before, after) = (&p[0], &p[1]);
        if before.stage == after.stage {
            continue;
        }
        match after.stage {
            Stage::Transport => begin = i + 1,
            Stage::Returning => return_command = true,
            Stage::Opening => opening = Some(i + 1),
            Stage::Backoff | Stage::LostGrip => {
                let onset = rows[begin..=i]
                    .iter()
                    .find(|r| r.truth.bar > 1e-8)
                    .map(|r| r.tick);
                if after.stage == Stage::Backoff {
                    alarms += 1;
                    if onset.is_none() {
                        return Err("retreat without physical cable contact".into());
                    }
                } else {
                    grip_losses += 1;
                }
                let rest = rows[i + 1..].iter().take_while(|r| r.stage == after.stage);
                let reverse = rest
                    .clone()
                    .find(|r| r.jaws.rail_speed_m_s <= 0.)
                    .ok_or("rail did not arrest its forward speed")?;
                let max_overshoot = rest
                    .clone()
                    .map(|r| r.jaws.rail_y_m - before.jaws.rail_y_m)
                    .fold(0., f64::max);
                arrests.push(json!({"mode":after.stage,"first_cable_contact_tick":onset,"detection_tick":before.tick,"first_command_tick":after.tick,"first_command_n":after.rail_command_n,
           "delay_after_cable_contact_ms":onset.map(|t|before.tick-t),"detection_rigid_gap_m":before.truth.robot_bar_gap,"forward_speed_at_detection_m_s":before.jaws.rail_speed_m_s,"peak_forward_overshoot_m":max_overshoot,
           "first_nonpositive_rail_speed_tick":reverse.tick,"jaw_payload_up_n":before.jaws.force_on_payload_n[2],"filtered_drag_n":before.signal.drag_n,"unloaded_travel_m":before.unloaded_travel_m}));
            }
            Stage::Placing => {
                retry_gap = rows[begin..=i]
                    .iter()
                    .filter(|r| r.stage == Stage::Transport && r.truth.near_bar)
                    .map(|r| r.truth.clearance)
                    .reduce(f64::min);
            }
            _ => {}
        }
    }
    let clearance = rows
        .iter()
        .map(|r| r.truth.robot_bar_gap)
        .fold(f64::INFINITY, f64::min);
    let mut first_rigid = None;
    let mut pad_totals = [0.; 2];
    for r in rows {
        let impulses = r
            .rigid_pad_bar_impulse_ns
            .ok_or("missing rigid pad/bar measurements")?;
        if impulses[0] + impulses[1] > 1e-12 {
            first_rigid.get_or_insert(r.tick);
        }
        for (sum, j) in pad_totals.iter_mut().zip(impulses) {
            *sum += j;
        }
    }
    let rigid_ns = pad_totals.iter().sum::<f64>();
    let max_strain = rows.iter().map(|r| r.truth.strain).fold(0., f64::max);
    let residual = rows.iter().map(|r| r.truth.residual).fold(0., f64::max);
    let cable_contact_ticks = rows.iter().filter(|r| r.truth.bar > 1e-8).count();
    let source = matches!(
        name,
        "tall" | "fast-tall" | "original" | "original-fast" | "lost"
    );
    let support = |r: &Log| {
        if source {
            r.truth.source
        } else {
            r.truth.shelf
        }
    };
    let rest = &rows[rows.len() - 500..];
    let final_mean = rest.iter().map(support).sum::<f64>() / 500.;
    let dwell = opening.map(|k| rows[k - 30..k].iter().map(support).sum::<f64>() / 30.);
    let final_speed = rest.iter().map(|r| r.truth.peak_speed).fold(0., f64::max);
    let min_squeeze = rows
        .iter()
        .filter(|r| {
            matches!(
                r.stage,
                Stage::Transport | Stage::Backoff | Stage::Returning | Stage::Raising
            )
        })
        .flat_map(|r| r.jaws.squeeze_n)
        .fold(f64::INFINITY, f64::min);
    let valid = match name {
        "original" | "original-fast" => {
            alarms == 2 && return_command && rigid_ns > 1e-5 && clearance < 1e-8
        }
        "tall" | "fast-tall" => {
            alarms == 2
                && return_command
                && clearance > 0.002
                && rigid_ns == 0.
                && min_squeeze > 0.3
        }
        "snag" => {
            alarms == 1 && retry_gap.is_some_and(|g| g > 0.003) && rigid_ns == 0. && !return_command
        }
        "rub" => alarms == 0 && cable_contact_ticks > 100,
        "lost" => grip_losses == 1 && opening.is_none() && alarms == 0 && rigid_ns == 0.,
        _ => alarms == 0 && cable_contact_ticks == 0 && rigid_ns == 0.,
    };
    if !valid
        || (final_mean - WEIGHT).abs() > 0.002
        || final_speed > 1e-4
        || max_strain > 0.005
        || residual > 1e-8
        || (name != "lost" && !dwell.is_some_and(|n| n > 0.9 * WEIGHT))
    {
        return Err(format!("{name}: {alarms} snags, {rigid_ns} N s jaw impact, {clearance} m gap, {dwell:?} N placement, {max_strain} strain").into());
    }
    Ok(
        json!({"case":name,"reset_identical_ticks":rows.len(),"arrests":arrests,"minimum_rigid_clearance_m":clearance,"first_rigid_pad_impact_tick":first_rigid,"rigid_pad_impulse_sum_ns":rigid_ns,"rigid_pad_impulse_left_right_ns":pad_totals,
    "cable_contact_ticks":cable_contact_ticks,"minimum_transport_squeeze_n":min_squeeze,"peak_cable_strain":max_strain,"peak_cable_momentum_residual_ns":residual,
    "retry_crossing_clearance_m":retry_gap,"returned_to_source":return_command,"first_open_tick":opening.map(|x|x+1),"release_dwell_mean_n":dwell,"rest_support":if source{"source"}else{"shelf"},"rest_support_mean_n":final_mean,"rest_peak_speed_m_s":final_speed}),
    )
}
