// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use rand_chacha::{ChaCha8Rng, rand_core::SeedableRng};
use rand_distr::{Distribution, StandardNormal, StandardUniform};

/// An independently owned sensor stream. Its seed and stable stream ID define replay.
/// Cloning preserves its exact position; simulation reset policy belongs to the caller.
#[derive(Debug, Clone)]
pub struct NoiseStream(ChaCha8Rng);

impl NoiseStream {
    pub fn new(seed: [u8; 32], stream: u64) -> Self {
        let mut rng = ChaCha8Rng::from_seed(seed);
        rng.set_stream(stream);
        Self(rng)
    }

    fn normal(&mut self, mean: f64, stddev: f64) -> f64 {
        // Native Rand::DblNormal returns the mean at this zero threshold.
        if stddev <= f64::EPSILON {
            mean
        } else {
            let standard: f64 = StandardNormal.sample(&mut self.0);
            mean + standard * stddev
        }
    }
}

/// Native Gaussian/quantized parameters, in the measured channel's SI units.
/// Nonpositive deviations disable their random contribution, as in native sensors.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GaussianParameters {
    pub mean: f64,
    pub stddev: f64,
    pub bias_mean: f64,
    pub bias_stddev: f64,
    pub dynamic_bias_stddev: f64,
    pub dynamic_bias_correlation_time: f64,
    /// Positive precision above 1e-6 enables rounding for either Gaussian kind.
    pub precision: f64,
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
#[error(
    "noise parameters, input, elapsed time and derived values must be finite; elapsed time must be nonnegative"
)]
pub struct InvalidNoise;

/// One channel's fixed/dynamic bias; its caller owns the random stream.
#[derive(Debug, Clone)]
pub struct GaussianNoise {
    parameters: GaussianParameters,
    bias: f64,
}

impl GaussianNoise {
    pub fn new(
        parameters: GaussianParameters,
        stream: &mut NoiseStream,
    ) -> Result<Self, InvalidNoise> {
        let p = parameters;
        if [
            p.mean,
            p.stddev,
            p.bias_mean,
            p.bias_stddev,
            p.dynamic_bias_stddev,
            p.dynamic_bias_correlation_time,
            p.precision,
        ]
        .iter()
        .any(|x| !x.is_finite())
        {
            return Err(InvalidNoise);
        }
        let mut bias = if p.bias_stddev > 0.0 {
            stream.normal(p.bias_mean, p.bias_stddev)
        } else {
            p.bias_mean
        };
        let sign: f64 = StandardUniform.sample(&mut stream.0);
        if sign < 0.5 {
            bias = -bias;
        }
        if !bias.is_finite() {
            return Err(InvalidNoise);
        }
        Ok(Self { parameters, bias })
    }

    pub fn parameters(&self) -> GaussianParameters {
        self.parameters
    }

    pub fn bias(&self) -> f64 {
        self.bias
    }

    /// Applies white noise, evolving bias, then native half-away-from-zero rounding.
    /// Failed output validation retains the prior bias; consumed randomness is not rewound.
    pub fn apply(
        &mut self,
        input: f64,
        dt: f64,
        stream: &mut NoiseStream,
    ) -> Result<f64, InvalidNoise> {
        if !input.is_finite() || !dt.is_finite() || dt < 0.0 {
            return Err(InvalidNoise);
        }
        let p = self.parameters;
        let white = if p.stddev > 0.0 {
            stream.normal(p.mean, p.stddev)
        } else {
            p.mean
        };
        let mut bias = self.bias;
        if p.dynamic_bias_stddev > 0.0 && p.dynamic_bias_correlation_time > 0.0 && dt > 0.0 {
            let sigma = p.dynamic_bias_stddev;
            let tau = p.dynamic_bias_correlation_time;
            let deviation = (-sigma * sigma * tau / 2.0 * (-2.0 * dt / tau).exp_m1()).sqrt();
            bias = (-dt / tau).exp() * bias + stream.normal(0.0, deviation);
        }
        let mut output = input + bias + white;
        if p.precision > 1e-6 {
            output = (output / p.precision).round() * p.precision;
        }
        if !bias.is_finite() || !output.is_finite() {
            return Err(InvalidNoise);
        }
        self.bias = bias;
        Ok(output)
    }
}
