// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::{GaussianNoise, GaussianParameters, ImuReading, InvalidNoise, NoiseStream};
use sdformat::{Imu, NoiseKind};
use std::time::Duration;

#[derive(Clone)]
pub(super) struct ImuNoise {
    channels: [Option<GaussianNoise>; 6],
    covariance: [f64; 6],
    stream: NoiseStream,
    seed: [u8; 32],
    identity: u64,
    previous: Option<Duration>,
}

impl ImuNoise {
    pub(super) fn new(
        config: &Imu,
        seed: [u8; 32],
        identity: u64,
    ) -> Result<Option<Box<Self>>, InvalidNoise> {
        let configurations = config
            .linear_acceleration
            .iter()
            .chain(&config.angular_velocity);
        if configurations
            .clone()
            .all(|noise| noise.kind == NoiseKind::None)
        {
            return Ok(None);
        }
        let mut result = Self {
            channels: std::array::from_fn(|_| None),
            covariance: [0.0; 6],
            stream: NoiseStream::new(seed, identity),
            seed,
            identity,
            previous: None,
        };
        for (index, noise) in configurations.enumerate() {
            if noise.kind == NoiseKind::None {
                continue;
            }
            // Native IMU covariance is rounded through float before publication.
            let covariance = (noise.stddev * noise.stddev) as f32 as f64;
            if !covariance.is_finite() {
                return Err(InvalidNoise);
            }
            result.covariance[index] = covariance;
            result.channels[index] = Some(GaussianNoise::new(
                GaussianParameters {
                    mean: noise.mean,
                    stddev: noise.stddev,
                    bias_mean: noise.bias_mean,
                    bias_stddev: noise.bias_stddev,
                    dynamic_bias_stddev: noise.dynamic_bias_stddev,
                    dynamic_bias_correlation_time: noise.dynamic_bias_correlation_time,
                    precision: noise.precision,
                },
                &mut result.stream,
            )?);
        }
        Ok(Some(Box::new(result)))
    }

    pub(super) fn apply(
        &mut self,
        reading: &mut ImuReading,
        time: Duration,
    ) -> Result<(), InvalidNoise> {
        // chrono duration_cast<duration<float>> casts the integer ns count before dividing.
        let dt = self
            .previous
            .filter(|previous| *previous <= time)
            .map_or(0.0, |previous| {
                ((time - previous).as_nanos() as f32 / 1e9f32) as f64
            });
        for (value, noise) in reading
            .specific_force
            .iter_mut()
            .chain(&mut reading.angular_velocity)
            .zip(&mut self.channels)
        {
            if let Some(noise) = noise {
                *value = noise.apply(*value, dt, &mut self.stream)?;
            }
        }
        reading
            .angular_velocity_covariance
            .copy_from_slice(&self.covariance[3..]);
        reading
            .linear_acceleration_covariance
            .copy_from_slice(&self.covariance[..3]);
        self.previous = Some(time);
        Ok(())
    }

    pub(super) fn reset(&mut self) {
        self.stream = NoiseStream::new(self.seed, self.identity);
        for noise in self.channels.iter_mut().flatten() {
            *noise = GaussianNoise::new(noise.parameters(), &mut self.stream)
                .expect("the same seed and validated parameters reproduce the initial finite bias");
        }
        self.previous = None;
    }
}
