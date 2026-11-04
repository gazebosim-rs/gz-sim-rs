// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded ideal planar lidar measurement geometry; acquisition and IO are external.

pub const MAX_PLANAR_BEAMS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarLidarConfig {
    pub samples: usize,
    pub angle_min: f64,
    pub angle_max: f64,
    pub range_min: f64,
    pub range_max: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "expected 2–128 planar beams, a finite positive span up to one turn, finite ordered nonnegative ranges and nonnegative finite hits"
)]
pub struct InvalidLidarInput;

#[derive(Debug, Clone)]
pub struct PlanarLidar {
    config: PlanarLidarConfig,
    step: f64,
    directions: Vec<[f64; 3]>,
}

impl PlanarLidar {
    pub fn new(config: PlanarLidarConfig) -> Result<Self, InvalidLidarInput> {
        let span = config.angle_max - config.angle_min;
        if !(2..=MAX_PLANAR_BEAMS).contains(&config.samples)
            || ![
                config.angle_min,
                config.angle_max,
                config.range_min,
                config.range_max,
            ]
            .iter()
            .all(|x| x.is_finite())
            || !span.is_finite()
            || span <= 0.0
            || span > std::f64::consts::TAU
            || config.range_min < 0.0
            || config.range_max <= config.range_min
        {
            return Err(InvalidLidarInput);
        }
        let step = span / (config.samples - 1) as f64;
        if !step.is_finite() || step <= 0.0 {
            return Err(InvalidLidarInput);
        }
        let mut directions = Vec::with_capacity(config.samples);
        let mut previous = None;
        for index in 0..config.samples {
            let angle = if index + 1 == config.samples {
                config.angle_max
            } else {
                config.angle_min + index as f64 * step
            };
            if !angle.is_finite() || previous.is_some_and(|last| angle <= last) {
                return Err(InvalidLidarInput);
            }
            let (sin, cos) = angle.sin_cos();
            directions.push([cos, sin, 0.0]);
            previous = Some(angle);
        }
        Ok(Self {
            config,
            step,
            directions,
        })
    }

    pub fn config(&self) -> &PlanarLidarConfig {
        &self.config
    }
    pub fn angle_step(&self) -> f64 {
        self.step
    }
    /// Unit directions in the sensor's local XY plane; +X is zero angle.
    pub fn directions(&self) -> &[[f64; 3]] {
        &self.directions
    }

    /// Keep continuous metric distances. Range resolution is metadata, not an
    /// invented quantization rule. A miss remains positive infinity.
    pub fn range(&self, hit: Option<f64>) -> Result<f64, InvalidLidarInput> {
        match hit {
            None => Ok(f64::INFINITY),
            Some(value) if value.is_finite() && value >= 0.0 => {
                Ok(value.clamp(self.config.range_min, self.config.range_max))
            }
            Some(_) => Err(InvalidLidarInput),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> PlanarLidarConfig {
        PlanarLidarConfig {
            samples: 3,
            angle_min: 0.0,
            angle_max: std::f64::consts::FRAC_PI_2,
            range_min: 0.1,
            range_max: 10.0,
        }
    }
    #[test]
    fn inclusive_planar_directions_and_continuous_clipping_have_physical_units() {
        let model = PlanarLidar::new(config()).unwrap();
        let expected = [
            [1.0, 0.0, 0.0],
            [
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
                0.0,
            ],
            [0.0, 1.0, 0.0],
        ];
        for (actual, expected) in model.directions().iter().zip(expected) {
            for (a, b) in actual.iter().zip(expected) {
                assert!((a - b).abs() < 1e-12);
            }
        }
        for direction in model.directions() {
            let norm: f64 = direction.iter().map(|x| x * x).sum();
            assert!((norm - 1.0).abs() <= 1e-12);
        }
        assert_eq!(model.angle_step(), std::f64::consts::FRAC_PI_4);
        assert_eq!(*model.config(), config());
        for (hit, expected) in [
            (None, f64::INFINITY),
            (Some(0.0), 0.1),
            (Some(0.05), 0.1),
            (Some(4.123456789), 4.123456789),
            (Some(10.0), 10.0),
            (Some(11.0), 10.0),
        ] {
            assert_eq!(model.range(hit).unwrap(), expected);
        }
    }
    #[test]
    fn invalid_domains_are_refused_without_returning_a_scan() {
        let mut invalid = Vec::new();
        for samples in [0, 1, MAX_PLANAR_BEAMS + 1, usize::MAX] {
            invalid.push(PlanarLidarConfig {
                samples,
                ..config()
            });
        }
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            invalid.extend([
                PlanarLidarConfig {
                    angle_min: value,
                    ..config()
                },
                PlanarLidarConfig {
                    angle_max: value,
                    ..config()
                },
                PlanarLidarConfig {
                    range_min: value,
                    ..config()
                },
                PlanarLidarConfig {
                    range_max: value,
                    ..config()
                },
            ]);
        }
        invalid.push(PlanarLidarConfig {
            samples: MAX_PLANAR_BEAMS,
            angle_max: f64::from_bits(1),
            ..config()
        });
        invalid.push(PlanarLidarConfig {
            samples: MAX_PLANAR_BEAMS,
            angle_min: 9_007_199_254_740_992.0,
            angle_max: 9_007_199_254_740_994.0,
            ..config()
        });
        invalid.extend([
            PlanarLidarConfig {
                angle_max: 0.0,
                ..config()
            },
            PlanarLidarConfig {
                angle_max: -1.0,
                ..config()
            },
            PlanarLidarConfig {
                angle_max: std::f64::consts::TAU + 1.0,
                ..config()
            },
            PlanarLidarConfig {
                range_min: -1.0,
                ..config()
            },
            PlanarLidarConfig {
                range_max: 0.1,
                ..config()
            },
        ]);
        for input in invalid {
            assert!(matches!(PlanarLidar::new(input), Err(InvalidLidarInput)));
        }
        let model = PlanarLidar::new(config()).unwrap();
        for hit in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.01] {
            assert_eq!(model.range(Some(hit)), Err(InvalidLidarInput));
        }
        assert_eq!(
            PlanarLidar::new(PlanarLidarConfig {
                samples: MAX_PLANAR_BEAMS,
                ..config()
            })
            .unwrap()
            .directions()
            .len(),
            MAX_PLANAR_BEAMS
        );
    }
}
