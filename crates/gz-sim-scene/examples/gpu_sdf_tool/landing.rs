// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the first loaded public tick at successively finer native clocks.
use super::*;
use std::num::NonZeroUsize;

#[derive(serde::Deserialize)]
struct LandingFrame {
    #[serde(flatten)]
    sample: Published,
    velocity_rad_s: [f64; 2],
    motor_demand_rad_s: [f64; 2],
}

/// Replay one loaded tick from the same incoming state. Vary the contact
/// iterations separately from the external sample and motor clocks.
fn native_contact_iterations(
    lane: &PlanarToolLane,
    limits: [Option<[f64; 2]>; 2],
    steps: NonZeroUsize,
) -> Vec<serde_json::Value> {
    let mut values = vec![];
    for (iterations, unbiased) in [
        (1, 1),
        (2, 1),
        (4, 1),
        (8, 1),
        (16, 1),
        (1, 4),
        (4, 4),
        (8, 8),
    ] {
        let mut model = native_tool::NativeTool::new_with_limits(lane, limits);
        model.set_steps_per_ms(steps);
        let settings = &mut model.arm.world.integration_parameters;
        settings.num_internal_pgs_iterations = iterations;
        settings.num_internal_stabilization_iterations = unbiased;
        let sample = model.step(lane);
        let q = model.arm.angles();
        let w = model.arm.rates();
        let distal = lane.arm.length[1] * (q[0] + q[1]).cos();
        let vertical = (lane.arm.length[0] * q[0].cos() + distal) * w[0] + distal * w[1];
        values.push(json!({"contact_iterations":iterations,"stabilization_iterations":unbiased,
            "normal_impulse_ns":sample.n,"first_internal_normal_impulse_ns":sample.first_n,
            "tip_vertical_velocity_m_s":vertical,"landing_offset_s":sample.first,"tangent_impulse_ns":sample.t}));
    }
    values
}

pub(super) fn run(trace: &Path, out: &Path) -> Result<(), Box<dyn Error>> {
    let frames: Vec<LandingFrame> = serde_json::from_slice(&fs::read(trace)?)?;
    let landing = frames
        .iter()
        .position(|f| f.sample.normal_impulse_ns > 0.)
        .filter(|&i| i > 0)
        .ok_or("a free incoming frame and a landing are required")?;
    if frames[..=landing]
        .iter()
        .enumerate()
        .any(|(i, f)| f.sample.tick_ms != i + 1 || f.sample.stop_impulse_nms != [0.; 2])
    {
        return Err(
            "first landing must precede all joint stops on a continuous public clock".into(),
        );
    }
    let incoming = &frames[landing - 1];
    let hit = &frames[landing];
    let origin = [0.48, -0.62];
    let cut: [f64; 2] = std::array::from_fn(|i| origin[i] + incoming.sample.position_rad[i]);
    let mut results = vec![];
    let mut iteration_sweep = vec![];
    for substeps in [8, 16, 32, 64, 128, 256, 512] {
        let mut rows = vec![];
        let mut analytic = None;
        for matched in [false, true] {
            let mut lane = reference_fixture();
            lane.arm.damping = [6., 1.5];
            lane.velocity_mode = [1.; 2];
            if matched {
                lane.arm.angle = cut.map(turn);
                lane.arm.speed = incoming.velocity_rad_s;
            }
            // No stop is loaded before or during this impact. Keeping the same
            // physical limits in both runs also checks for an intervening crossing.
            let limits = [[-0.36, 0.02], [-0.15, 1.20]];
            let limits = std::array::from_fn(|i| {
                let offset = if matched {
                    incoming.sample.position_rad[i]
                } else {
                    0.
                };
                Some(limits[i].map(|q| q - offset))
            });
            let mut native = native_tool::NativeTool::new_with_limits(&lane, limits);
            native.set_steps_per_ms(NonZeroUsize::new(substeps).unwrap());
            if !matched {
                for f in &frames[..landing] {
                    let q = native.arm.angles();
                    lane.velocity_target = std::array::from_fn(|i| {
                        let cap = [2.4, 3.2][i];
                        ((f.sample.command_rad[i] - (q[i] - origin[i])) / 0.001).clamp(-cap, cap)
                    });
                    if native.step(&lane).n > 0. {
                        return Err("continuous native arm landed before the matched cut".into());
                    }
                }
            }
            let q = native.arm.angles();
            let w = native.arm.rates();
            if matched
                && (0..2).any(|i| {
                    (cut[i] - q[i]).abs() > 1e-12
                        || (incoming.velocity_rad_s[i] - w[i]).abs() > 1e-12
                })
            {
                return Err("failed to initialize the native incoming state".into());
            }
            lane.velocity_target = if matched {
                hit.motor_demand_rad_s
            } else {
                std::array::from_fn(|i| {
                    let cap = [2.4, 3.2][i];
                    ((hit.sample.command_rad[i] - (q[i] - origin[i])) / 0.001).clamp(-cap, cap)
                })
            };
            if matched {
                if substeps == 512 {
                    iteration_sweep = native_contact_iterations(
                        &lane,
                        limits,
                        NonZeroUsize::new(substeps).unwrap(),
                    );
                }
                let mut state = reference::State::new(&lane.arm);
                let obs = tool_reference::refined_step(
                    &mut state,
                    &lane,
                    NonZeroUsize::new(substeps).unwrap(),
                );
                analytic = Some(
                    json!({"normal_impulse_ns":obs.n, "tangent_impulse_ns":obs.t,
                    "first_internal_normal_impulse_ns":obs.first_n, "landing_offset_s":obs.first,
                    "rates_after_rad_s":state.w, "tip_m":obs.tip}),
                );
            }
            let observation = native.step(&lane);
            rows.push(json!({"matched":matched,"incoming_angles_rad":q,"incoming_rates_rad_s":w,
                "landing_offset_s":observation.first,"normal_impulse_ns":observation.n,
                "tangent_impulse_ns":observation.t,"first_internal_normal_impulse_ns":observation.first_n,
                "tip_m":observation.tip,"rates_after_rad_s":native.arm.rates()}));
        }
        results.push(json!({"native_steps_per_ms":substeps,"runs":rows,"matched_cartesian_reference":analytic}));
    }
    let report = json!({"public_tick_ms":hit.sample.tick_ms,"incoming_angles_rad":cut,"incoming_rates_rad_s":incoming.velocity_rad_s,
        "gpu_normal_impulse_ns":hit.sample.normal_impulse_ns,"gpu_rates_after_rad_s":hit.velocity_rad_s,"native":results,"matched_native_iterations_at_512_steps":iteration_sweep});
    fs::write(out, serde_json::to_vec_pretty(&report)?)?;
    Ok(())
}
