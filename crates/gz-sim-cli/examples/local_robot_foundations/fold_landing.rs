// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! First external contact of a sheet after it has completely escaped the fingers.
use super::*;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum End {
    Escape,
    FirstLanding,
    Horizon,
}

pub(super) fn particles(grasp: &Grasp) -> Result<Vec<[f64; 2]>> {
    Ok(arm_sheet::cloth(&grasp.recording.sim)?
        .particles()
        .iter()
        .map(|p| [p.inverse_mass.recip(), p.velocity[2]])
        .collect())
}

fn momentum(particles: &[[f64; 2]]) -> f64 {
    particles
        .iter()
        .map(|[mass, velocity]| mass * velocity)
        .sum()
}

pub(super) fn impact(
    tick: usize,
    incoming: &[[f64; 2]],
    outgoing: &[[f64; 2]],
    previous: &Tick,
    now: &Tick,
    weight: f64,
) -> Value {
    let incoming_pz = momentum(incoming);
    let outgoing_pz = momentum(outgoing);
    let bench = now.bench_up_n * DT;
    let jaws = now.jaw_up_n.iter().sum::<f64>() * DT;
    let gravity = -weight * DT;
    json!({"tick":tick,"incoming_mass_kg_vz_m_s":incoming,"outgoing_mass_kg_vz_m_s":outgoing,"incoming_clearance_m":previous.min_bench_clearance_m,
        "outgoing_clearance_m":now.min_bench_clearance_m,"incoming_vertical_momentum_kg_m_s":incoming_pz,
        "outgoing_vertical_momentum_kg_m_s":outgoing_pz,"bench_upward_impulse_ns":bench,"jaw_upward_impulse_ns":jaws,
        "gravity_impulse_ns":gravity,"vertical_balance_error_ns":outgoing_pz-incoming_pz-gravity-jaws-bench,
        "bench_lateral_impulse_ns":-now.bench_side_reaction_n*DT,
        "lateral_balance_error_ns":now.momentum_y_kg_m_s-previous.momentum_y_kg_m_s
            -DT*(now.force_y_n-now.bench_side_reaction_n-now.jaw_side_reaction_n.iter().sum::<f64>())})
}
