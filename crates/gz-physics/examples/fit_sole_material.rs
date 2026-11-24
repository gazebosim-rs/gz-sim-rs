// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fit two layer stiffnesses and damping from a centered, fully covered flat drop.
use gz_physics::sole_calibration::{DropFit, ImpactMeasurement, LayerPair};
use std::error::Error;
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 8 {
        return Err("usage: fit_sole_material MASS_KG AREA_M2 INCOMING_M_S REBOUND_M_S depth|time|force VALUE foot-share|known-deck VALUE".into());
    }
    let value = |i: usize| args[i].parse::<f64>();
    let instrument = match args[4].as_str() {
        "depth" => ImpactMeasurement::MaximumCompressionM(value(5)?),
        "time" => ImpactMeasurement::ContactDurationS(value(5)?),
        "force" => ImpactMeasurement::PeakForceN(value(5)?),
        _ => return Err("select depth in m, time in s or force in N".into()),
    };
    let fit = DropFit::measure(value(0)?, value(1)?, value(2)?, value(3)?, instrument)
        .ok_or("measurements do not define a finite passive material")?;
    let pair = match args[6].as_str() {
        "foot-share" => LayerPair::with_foot_share(fit.effective_n_m3, value(7)?),
        "known-deck" => LayerPair::with_known_deck(fit.effective_n_m3, value(7)?),
        _ => return Err("select foot-share or known-deck (N/m³)".into()),
    }
    .ok_or("layer measurement is inconsistent with the fitted stiffness")?;
    println!(
        "<foot_stiffness>{}</foot_stiffness>\n<deck_stiffness>{}</deck_stiffness>\n<damping>{}</damping>",
        pair.foot_n_m3, pair.deck_n_m3, fit.damping_s_m
    );
    Ok(())
}
