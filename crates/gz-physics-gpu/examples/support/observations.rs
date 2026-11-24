// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::Lane;

pub fn dynamic(lane: &Lane) -> [f64; 7] {
    [
        lane.anchor_z,
        lane.bob_z,
        lane.rigid_z,
        lane.anchor_v,
        lane.bob_v,
        lane.rigid_v,
        lane.last_receiver_impulse_z,
    ]
}

pub fn parameters(lane: &Lane) -> [f64; 5] {
    [
        lane.rest_length,
        lane.compliance,
        lane.bob_inverse_mass,
        lane.rigid_inverse_mass,
        lane.combined_radius,
    ]
}

/// Fingerprint all published binary64 values in stable lane order. Comparing
/// independent reruns at each tick detects even a changed signed zero.
pub fn fingerprint(lanes: &[Lane]) -> u64 {
    lanes.iter().fold(0xcbf29ce484222325, |hash, lane| {
        dynamic(lane)
            .into_iter()
            .chain(parameters(lane))
            .fold(hash, |hash, x| {
                (hash ^ x.to_bits()).wrapping_mul(0x100000001b3)
            })
    })
}

#[derive(Default)]
pub struct MaxError {
    pub value: f64,
    pub lane: usize,
    pub tick: usize,
}

impl MaxError {
    pub fn update(&mut self, error: f64, lane: usize, tick: usize) {
        assert!(error.is_finite(), "nonfinite lane {lane} at tick {tick}");
        if error > self.value {
            self.value = error;
            self.lane = lane;
            self.tick = tick;
        }
    }
}

pub struct Agreement {
    pub position: MaxError,
    pub velocity: MaxError,
    pub impulse: MaxError,
    pub min_support_ratio: f64,
    pub max_support_ratio: f64,
    pub max_momentum_residual: f64,
    impacts: Vec<f64>,
    span_min: Vec<f64>,
    span_max: Vec<f64>,
    last_motion: Vec<i8>,
    reversals: Vec<usize>,
    hold_impulses: Vec<f64>,
    hold_start_v: Vec<f64>,
}

impl Agreement {
    pub fn new(n: usize) -> Self {
        Self {
            position: Default::default(),
            velocity: Default::default(),
            impulse: Default::default(),
            min_support_ratio: f64::INFINITY,
            max_support_ratio: 0.,
            max_momentum_residual: 0.,
            impacts: vec![0.; n],
            span_min: vec![f64::INFINITY; n],
            span_max: vec![f64::NEG_INFINITY; n],
            last_motion: vec![0; n],
            reversals: vec![0; n],
            hold_impulses: vec![0.; n],
            hold_start_v: vec![0.; n],
        }
    }
    pub fn compare(&mut self, tick: usize, actual: &[Lane], reference: &[Lane]) {
        assert_eq!(actual.len(), reference.len());
        for (i, (gpu, cpu)) in actual.iter().zip(reference).enumerate() {
            for (j, (x, r)) in dynamic(gpu).into_iter().zip(dynamic(cpu)).enumerate() {
                let error = (x - r).abs();
                match j {
                    0..=2 => self.position.update(error, i, tick),
                    3..=5 => self.velocity.update(error, i, tick),
                    _ => self.impulse.update(error, i, tick),
                }
            }
            assert_eq!(
                parameters(gpu),
                parameters(cpu),
                "material/admission changed at tick {tick}, lane {i}"
            );
            self.impacts[i] = self.impacts[i].max(gpu.last_receiver_impulse_z);
            // A moving length after impact alone would also count monotone creep.
            // Require a change between resolved shortening and lengthening speeds.
            if i % 2 == 1 && self.impacts[i] > 0.1 {
                let relative = gpu.bob_v - gpu.anchor_v;
                let sign = if relative > 0.005 {
                    1
                } else if relative < -0.005 {
                    -1
                } else {
                    0
                };
                if sign != 0 {
                    if self.last_motion[i] != 0 && self.last_motion[i] != sign {
                        self.reversals[i] += 1;
                    }
                    self.last_motion[i] = sign;
                }
            }
            if tick == 800 {
                self.hold_start_v[i] = gpu.rigid_v;
            }
            let span = gpu.bob_z - gpu.anchor_z;
            assert!(span > 0., "strut inverted at tick {tick}, lane {i}");
            if tick > 800 {
                self.span_min[i] = self.span_min[i].min(span);
                self.span_max[i] = self.span_max[i].max(span);
                self.hold_impulses[i] += gpu.last_receiver_impulse_z;
            }
        }
    }

    pub fn verify(&mut self, final_state: &[Lane], implementation: &str) {
        const HOLD_SECONDS: f64 = 0.2;
        let mut soft_amplitude = (f64::INFINITY, 0.0_f64);
        let mut min_reversals = usize::MAX;
        for (i, lane) in final_state.iter().enumerate() {
            assert!(self.impacts[i] > 0.1, "no resolved impact in lane {i}");
            let mass = lane.rigid_inverse_mass.recip();
            let gravity_impulse = mass * 9.81 * HOLD_SECONDS;
            let support = self.hold_impulses[i] / gravity_impulse;
            if i % 2 == 0 {
                self.min_support_ratio = self.min_support_ratio.min(support);
                self.max_support_ratio = self.max_support_ratio.max(support);
            } else {
                min_reversals = min_reversals.min(self.reversals[i]);
                let excursion = self.span_max[i] - self.span_min[i];
                soft_amplitude.0 = soft_amplitude.0.min(excursion);
                soft_amplitude.1 = soft_amplitude.1.max(excursion);
            }
            let p = mass * (lane.rigid_v - self.hold_start_v[i]);
            self.max_momentum_residual = self
                .max_momentum_residual
                .max((p + gravity_impulse - self.hold_impulses[i]).abs());
        }
        for (field, error, limit) in [
            ("position (m)", &self.position, 1e-8),
            ("velocity (m/s)", &self.velocity, 1e-6),
            ("impulse (N s)", &self.impulse, 1e-6),
        ] {
            println!(
                "{implementation} maximum {field} error {:.5e} at lane {}, tick {} (limit {limit:.0e})",
                error.value, error.lane, error.tick
            );
            assert!(
                error.value < limit,
                "{implementation}/production CPU {field} mismatch"
            );
        }
        println!(
            "{implementation} stiff cohort final 200 ms support / weight impulse range [{:.6}, {:.6}]; maximum rigid momentum-balance residual {:.2e} N s",
            self.min_support_ratio, self.max_support_ratio, self.max_momentum_residual
        );
        println!(
            "{implementation} compliant strut peak-to-peak length range over last 200 ms [{:.6}, {:.6}] m",
            soft_amplitude.0, soft_amplitude.1
        );
        assert!(
            soft_amplitude.0 > 0.001 && soft_amplitude.1 > 0.02,
            "compliant cohort does not oscillate"
        );
        println!(
            "{implementation}: after first impact every compliant lane reversed axial velocity at least {min_reversals} times (dead band ±0.005 m/s)"
        );
        assert!(
            min_reversals >= 2,
            "missing post-impact compression/rebound cycle"
        );
        assert!(self.max_momentum_residual < 1e-9);
        assert!(
            self.min_support_ratio > 0.99 && self.max_support_ratio < 1.01,
            "unsettled support load"
        );
    }
}

#[path = "times.rs"]
mod times;
pub use times::Times;
