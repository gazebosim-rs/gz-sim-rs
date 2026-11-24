// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/reference.rs"]
mod reference;

use gz_physics_gpu::{GpuBatch, GpuError, Lane};
use reference::{DT, GRAVITY, Reference};
use std::error::Error;

#[test]
fn construction_rejects_interpenetration_and_reversed_initial_order() {
    let base = reference::fixture(1)[0];
    for invalid in [
        Lane {
            rigid_z: base.bob_z + 0.5 * base.combined_radius,
            ..base
        },
        Lane {
            rigid_z: base.bob_z - base.combined_radius,
            ..base
        },
        Lane {
            bob_z: base.anchor_z - 0.01,
            ..base
        },
    ] {
        assert!(matches!(
            GpuBatch::new("unused", vec![base, invalid]),
            Err(GpuError::InvalidLane(1))
        ));
    }
}

fn bob_prediction(lane: &Lane) -> (f64, f64) {
    let h = DT.as_secs_f64();
    let v = lane.bob_v + h * GRAVITY;
    let z = lane.bob_z + h * v;
    let length = z - (lane.anchor_z + h * lane.anchor_v);
    let correction = lane.bob_inverse_mass * (lane.rest_length - length)
        / (lane.bob_inverse_mass + lane.compliance / h.powi(2));
    (z + correction, v + correction / h)
}

fn edge_cases() -> Vec<Lane> {
    let h = DT.as_secs_f64();
    let mut lanes = reference::fixture(129);
    for (i, lane) in lanes.iter_mut().enumerate() {
        lane.anchor_z = -4.;
        lane.bob_z = 0.;
        lane.rest_length = 4.;
        lane.anchor_v = 0.07;
        lane.bob_v = 0.02;
        lane.compliance = 1e-5;
        lane.bob_inverse_mass = 1.;
        lane.rigid_inverse_mass = [0.01, 1., 100.][(i / 6) % 3];
        lane.rigid_z = lane.combined_radius + 0.01;
        match i % 6 {
            // Without swept contact the rigid sphere passes through both faces.
            0 => lane.rigid_v = -300.,
            // Put the free rigid endpoint 100 nm inside or outside first touch.
            side @ (1 | 2) => {
                let epsilon = if side == 1 { -1e-7 } else { 1e-7 };
                let target = bob_prediction(lane).0 + lane.combined_radius + epsilon;
                lane.rigid_v = (target - lane.rigid_z) / h - 0.625 * h * GRAVITY;
            }
            3 => lane.rigid_v = 20.,
            // Exact initial touch is legal. Check both departure and approach.
            side => {
                lane.rigid_z = lane.combined_radius;
                lane.rigid_v = if side == 4 { 1. } else { -1. };
            }
        }
    }
    lanes
}

#[test]
#[ignore = "requires an explicitly selected Vulkan device with shader f64"]
fn finite_mass_sweeps_and_grazing_endpoints_match_cpu() -> Result<(), Box<dyn Error>> {
    let selector = std::env::var("GZ_GPU_ADAPTER")?;
    let initial = edge_cases();
    let mut cpu = Reference::new(&initial)?;
    let mut device = GpuBatch::new(&selector, initial.clone())?;
    let mut errors = [0_f64; 3];
    for tick in 0..3 {
        cpu.step()?;
        device.step()?;
        for (i, (gpu, cpu)) in device.lanes().iter().zip(&cpu.lanes).enumerate() {
            for (max, error) in errors.iter_mut().zip([
                (gpu.rigid_z - cpu.rigid_z)
                    .abs()
                    .max((gpu.bob_z - cpu.bob_z).abs()),
                (gpu.rigid_v - cpu.rigid_v)
                    .abs()
                    .max((gpu.bob_v - cpu.bob_v).abs()),
                (gpu.last_receiver_impulse_z - cpu.last_receiver_impulse_z).abs(),
            ]) {
                *max = max.max(error);
            }
            assert!(gpu.rigid_z - gpu.bob_z >= gpu.combined_radius - 1e-12);
            if tick == 0 {
                let lane = initial[i];
                let j = gpu.last_receiver_impulse_z;
                if matches!(i % 6, 0 | 1 | 5) {
                    assert!(j > 0., "missed initial contact in lane {i}");
                } else {
                    assert_eq!(j, 0., "unexpected initial contact in lane {i}");
                }
                let rigid_dv = gpu.rigid_v - (lane.rigid_v + DT.as_secs_f64() * GRAVITY);
                let bob_dv = gpu.bob_v - bob_prediction(&lane).1;
                assert!((rigid_dv / lane.rigid_inverse_mass - j).abs() < 1e-9);
                assert!((bob_dv / lane.bob_inverse_mass + j).abs() < 1e-9);
            }
        }
    }
    eprintln!("129 cells, 3 steps, max [m, m/s, N s]: {errors:?}");
    assert!(errors[0] < 1e-10 && errors[1] < 1e-8 && errors[2] < 1e-8);
    Ok(())
}
