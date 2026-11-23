// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/inclined_sheet.rs"]
mod sheet;
#[allow(dead_code)]
#[path = "support/slip_transition.rs"]
mod transition;
use std::error::Error;
use transition::*;
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1).collect::<Vec<_>>();
    let source = args
        .iter()
        .position(|a| a == "--write-sdf")
        .map(|i| {
            if i + 1 >= args.len() {
                return Err("missing SDF output path");
            }
            let file = args.remove(i + 1);
            args.remove(i);
            Ok(file)
        })
        .transpose()?;
    let cfg = Run {
        speed: args.first().map_or(Ok(-0.42), |v| v.parse())?,
        mu: args.get(1).map_or(Ok(0.15), |v| v.parse())?,
        width: args.get(2).map_or(Ok(250000), |v| v.parse())?,
        transition: args
            .get(3)
            .map(|v| v.parse::<u64>())
            .transpose()?
            .filter(|n| *n > 0),
        yaw: args.get(4).map_or(Ok(0.), |v| v.parse())?,
        extra_mass: args.get(5).map_or(Ok(0.), |v| v.parse())?,
    };
    if let Some(file) = source {
        use std::io::Write;
        let mut out = std::fs::File::create_new(file)?;
        out.write_all(cfg.sdf().as_bytes())?;
        return Ok(());
    }
    let ms = args.get(6).map_or(Ok(480), |v| v.parse())?;
    let r = run(cfg, ms)?;
    println!(
        "speed={} mu={} width_ns={} transition_ns={:?} yaw={} extra_mass={} ms={ms} x_m={:.14e} v_m_s={:.14e} max_x_error_m={:.14e} max_v_error_m_s={:.14e} cuts={} trials={} rejected_slip_trials={} reversal_bracket_ns={:?} max_P_residual_Ns={:.12e} max_Lz_residual_Nms={:.12e} max_spin_rad_s={:.12e} elapsed_sum_ms={:.4} wall_p50_us={:.3} wall_p95_us={:.3} analytic_turn_tick_us={:.3}",
        cfg.speed,
        cfg.mu,
        cfg.width,
        cfg.transition,
        cfg.yaw,
        cfg.extra_mass,
        r.x,
        r.v,
        r.worst_x,
        r.worst_v,
        r.cuts,
        r.trials,
        r.slip_refinements,
        r.crossing,
        r.p,
        r.l,
        r.spin,
        r.time_ns as f64 * 1e-6,
        r.wall_p50 as f64 * 1e-3,
        r.wall_p95 as f64 * 1e-3,
        r.transition_tick_ns as f64 * 1e-3
    );
    Ok(())
}
