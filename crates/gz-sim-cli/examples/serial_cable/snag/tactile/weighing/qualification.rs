// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use identify::Refusal;
pub(super) fn measure(
    name: &str,
    trial: Trial,
    request: usize,
    rows: &[Log],
    estimate: &Estimate,
) -> Result<Value> {
    // Authored mass and external supports are withheld until the evaluator runs.
    let truth_mass = trial.parameters().0 * 11.;
    let expected = match trial {
        Trial::Slippery | Trial::ProbeSlip => Some(Refusal::NoBilateralGrasp),
        Trial::Assisted => Some(Refusal::MassDisagreement),
        Trial::NoLift => Some(Refusal::UnsettledLift),
        Trial::StalledProbe => Some(Refusal::InsufficientProbe),
        Trial::Pulsed => Some(Refusal::InconsistentWeight),
        _ => None,
    };
    if estimate.refused != expected || estimate.mass_kg.is_none() != expected.is_some() {
        return Err(format!("unexpected identification for {name}: {estimate:?}").into());
    }
    let error = estimate.mass_kg.map(|v| (v - truth_mass) / truth_mass);
    let events = rows
        .windows(2)
        .filter(|w| w[0].stage != w[1].stage)
        .map(|w| (w[0].tick, w[1].stage))
        .collect::<Vec<_>>();
    let trips = events.iter().filter(|(_, s)| *s == Stage::Backoff).count();
    let carries = events.iter().any(|(_, s)| *s == Stage::Transport);
    let released = events
        .iter()
        .find(|(_, s)| *s == Stage::Opening)
        .map(|e| e.0);
    let first = events.first().ok_or("missing pickup decision")?.0;
    let rigid_ns = rows
        .iter()
        .filter_map(|r| r.rigid_pad_bar_impulse_ns)
        .flatten()
        .sum::<f64>();
    let gap = rows
        .iter()
        .map(|r| r.truth.robot_bar_gap)
        .fold(f64::INFINITY, f64::min);
    let strain = rows.iter().map(|r| r.truth.strain).fold(0., f64::max);
    let residual = rows.iter().map(|r| r.truth.residual).fold(0., f64::max);
    let stationary = &rows[rows.len() - 500..];
    let final_load = [
        stationary.iter().map(|r| r.truth.source).sum::<f64>() / 500.,
        stationary.iter().map(|r| r.truth.shelf).sum::<f64>() / 500.,
    ];
    let contact = rows.iter().filter(|r| r.truth.bar > 1e-8).count();
    let tests = match trial {
        Trial::Nominal | Trial::Heavy => {
            trips == 2
                && events.iter().any(|e| e.1 == Stage::Returning)
                && final_load[0] > 0.99 * truth_mass * 9.81
                && gap > 0.002
        }
        Trial::Rub => trips == 0 && contact >= 50 && final_load[1] > 0.99 * truth_mass * 9.81,
        Trial::Light => trips == 0 && contact == 0 && final_load[1] > 0.99 * truth_mass * 9.81,
        Trial::Lost => {
            trips == 0
                && released.is_none()
                && events.iter().any(|e| e.1 == Stage::LostGrip)
                && final_load[0] > 0.99 * truth_mass * 9.81
        }
        _ => !carries && released.is_none() && events == [(first, Stage::Unidentified)],
    };
    if !tests
        || error.is_some_and(|v| v.abs() > 0.02)
        || rigid_ns != 0.
        || strain > 0.01
        || residual > 1e-8
    {
        return Err(format!("{name} failed: {events:?}, {gap} m clearance, {contact} bar ticks, {rigid_ns} N s rigid, {error:?} mass error, {residual} momentum residual").into());
    }
    let max_speed = rows
        .iter()
        .filter(|r| r.stage == Stage::Transport)
        .map(|r| r.jaws.rail_speed_m_s.abs())
        .fold(0., f64::max);
    let max_accel = rows
        .iter()
        .filter(|r| r.stage == Stage::Transport)
        .map(|r| r.signal.rail_acceleration.abs())
        .fold(0., f64::max);
    Ok(
        json!({"case":name,"authored_mass_kg":truth_mass,"friction":trial.parameters().1,"requested_travel_ticks":request,"estimate":estimate,"mass_relative_error":error,
      "decision_tick":first,"events":events,"reset_identical_ticks":rows.len(),"bar_contact_ticks":contact,"minimum_rigid_clearance_m":gap,"rigid_pad_bar_impulse_ns":rigid_ns,"peak_strain":strain,
      "peak_transport_speed_m_s":max_speed,"peak_transport_accel_m_s2":max_accel,"peak_momentum_residual_ns":residual,"final_support_loads_source_shelf_n":final_load}),
    )
}
