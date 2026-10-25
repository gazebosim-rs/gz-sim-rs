// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::{GaussianNoise, GaussianParameters, InvalidNoise, NoiseStream};
use rand_chacha::{ChaCha8Rng, rand_core::SeedableRng};
use rand_distr::{Distribution, Uniform};

fn stream(id: u64) -> NoiseStream {
    NoiseStream::new([42; 32], id)
}

fn moments(values: &[f64]) -> (f64, f64) {
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / values.len() as f64;
    (mean, variance)
}

fn gaussian(p: GaussianParameters, random: &mut NoiseStream) {
    let mut noise = GaussianNoise::new(p, random).unwrap();
    assert_eq!(noise.parameters(), p);
    let expected_mean = 42.0 + p.mean + noise.bias();
    let values: Vec<_> = (0..100)
        .map(|_| noise.apply(42.0, 0.0, random).unwrap())
        .collect();
    let (mean, variance) = moments(&values);
    // Native NoiseTest uses 100 samples and the same five-sigma bounds.
    assert!((mean - expected_mean).abs() <= 5.0 * p.stddev / 100f64.sqrt());
    assert!((variance - p.stddev.powi(2)).abs() <= 5.0 * (2.0 * p.stddev.powi(4) / 99.0).sqrt());
}

// Measurement cases from gz-sensors src/Noise_TEST.cc (2018 OSRF, Apache-2.0).
// Custom callbacks/factory and image noise remain separate, unported contracts.
#[test]
fn upstream_gaussian_zero_mean_white_and_bias_assertions() {
    let mut random = stream(0);
    let mut zero = GaussianNoise::new(GaussianParameters::default(), &mut random).unwrap();
    let mut inputs = ChaCha8Rng::from_seed([42; 32]);
    let uniform = Uniform::new(-1e6, 1e6).unwrap();
    for _ in 0..100 {
        let input: f64 = uniform.sample(&mut inputs);
        assert!((zero.apply(input, 0.0, &mut random).unwrap() - input).abs() < 1e-6);
    }
    for bias_mean in [0.0, 100.0] {
        gaussian(
            GaussianParameters {
                mean: 10.0,
                stddev: 5.0,
                bias_mean,
                ..Default::default()
            },
            &mut random,
        );
    }
    let biases: Vec<_> = (0..100)
        .map(|_| {
            GaussianNoise::new(
                GaussianParameters {
                    bias_stddev: 5.0,
                    ..Default::default()
                },
                &mut random,
            )
            .unwrap()
            .bias()
        })
        .collect();
    let (mean, variance) = moments(&biases);
    assert!(mean.abs() <= 5.0 * 5.0 / 100f64.sqrt());
    assert!((variance - 25.0).abs() <= 5.0 * (2.0 * 625.0 / 99.0f64).sqrt());
}

#[test]
fn upstream_quantized_inputs_and_native_precision_threshold() {
    let mut random = stream(1);
    let mut noise = GaussianNoise::new(
        GaussianParameters {
            precision: 0.3,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    for input in [0.32, 0.31, 0.30, 0.29, 0.28] {
        assert!((noise.apply(input, 0.0, &mut random).unwrap() - 0.3).abs() < 1e-6);
    }
    for input in [-12.92, -12.91, -12.90, -12.89, -12.88] {
        assert!((noise.apply(input, 0.0, &mut random).unwrap() + 12.9).abs() < 1e-6);
    }
    for precision in [-1.0, 0.0, 1e-6] {
        let mut noise = GaussianNoise::new(
            GaussianParameters {
                precision,
                ..Default::default()
            },
            &mut random,
        )
        .unwrap();
        assert_eq!(
            noise.apply(0.123456789, 0.0, &mut random).unwrap(),
            0.123456789
        );
    }
    let mut halves = GaussianNoise::new(
        GaussianParameters {
            precision: 0.5,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    assert_eq!(halves.apply(0.25, 0.0, &mut random).unwrap(), 0.5);
    assert_eq!(halves.apply(-0.25, 0.0, &mut random).unwrap(), -0.5);
}

#[test]
fn streams_replay_independently_and_cloning_preserves_bias_and_position() {
    let parameters = GaussianParameters {
        mean: 0.1,
        stddev: 0.2,
        bias_stddev: 0.3,
        dynamic_bias_stddev: 0.4,
        dynamic_bias_correlation_time: 2.0,
        ..Default::default()
    };
    let mut first = stream(7);
    let mut replay = stream(7);
    let mut unrelated = stream(8);
    let mut a = GaussianNoise::new(parameters, &mut first).unwrap();
    let mut b = GaussianNoise::new(parameters, &mut replay).unwrap();
    let mut c = GaussianNoise::new(parameters, &mut unrelated).unwrap();
    assert_eq!(a.bias(), b.bias());
    let mut different = false;
    for _ in 0..100 {
        let value = a.apply(42.0, 0.001, &mut first).unwrap();
        assert_eq!(value, b.apply(42.0, 0.001, &mut replay).unwrap());
        different |= value != c.apply(42.0, 0.001, &mut unrelated).unwrap();
        c.apply(0.0, 0.1, &mut unrelated).unwrap();
    }
    assert!(different);
    let mut snapshot = a.clone();
    let mut random_snapshot = first.clone();
    for _ in 0..10 {
        assert_eq!(
            a.apply(42.0, 1.0, &mut first).unwrap(),
            snapshot.apply(42.0, 1.0, &mut random_snapshot).unwrap()
        );
    }
}

#[test]
fn dynamic_bias_uses_the_native_exact_discrete_innovation_distribution() {
    let mut random = stream(9);
    let sigma: f64 = 0.7;
    let tau: f64 = 2.0;
    let dt: f64 = 0.001f32 as f64;
    let mut noise = GaussianNoise::new(
        GaussianParameters {
            dynamic_bias_stddev: sigma,
            dynamic_bias_correlation_time: tau,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    assert_eq!(noise.apply(42.0, 0.0, &mut random).unwrap(), 42.0);
    let phi = (-dt / tau).exp();
    let expected_variance = sigma.powi(2) * tau / 2.0 * (1.0 - (-2.0 * dt / tau).exp());
    let mut innovations = Vec::new();
    for _ in 0..10_000 {
        let before = noise.bias();
        let value = noise.apply(42.0, dt, &mut random).unwrap();
        assert_eq!(value, 42.0 + noise.bias());
        innovations.push(noise.bias() - phi * before);
    }
    let (mean, variance) = moments(&innovations);
    assert!(mean.abs() <= 5.0 * (expected_variance / 10_000.0).sqrt());
    assert!(
        (variance - expected_variance).abs()
            <= 5.0 * (2.0 * expected_variance.powi(2) / 9999.0).sqrt()
    );
    for (dynamic, correlation) in [(-1.0, 1.0), (1.0, 0.0), (1.0, -1.0)] {
        let mut noise = GaussianNoise::new(
            GaussianParameters {
                bias_mean: 10.0,
                dynamic_bias_stddev: dynamic,
                dynamic_bias_correlation_time: correlation,
                ..Default::default()
            },
            &mut random,
        )
        .unwrap();
        let bias = noise.bias();
        assert_eq!(noise.apply(42.0, 1.0, &mut random).unwrap(), 42.0 + bias);
        assert_eq!(noise.bias(), bias);
    }
}

#[test]
fn tiny_deviations_and_nonpositive_native_deviations_preserve_the_mean() {
    let mut random = stream(10);
    for deviation in [-1.0, 0.0, f64::EPSILON] {
        let mut noise = GaussianNoise::new(
            GaussianParameters {
                mean: 2.0,
                stddev: deviation,
                bias_mean: 10.0,
                bias_stddev: deviation,
                ..Default::default()
            },
            &mut random,
        )
        .unwrap();
        assert_eq!(noise.bias().abs(), 10.0);
        assert_eq!(
            noise.apply(42.0, 0.0, &mut random).unwrap(),
            44.0 + noise.bias()
        );
    }
}

#[test]
fn invalid_parameters_inputs_and_derived_overflow_are_explicit_errors() {
    let mut random = stream(11);
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let bad = [
            GaussianParameters {
                mean: value,
                ..Default::default()
            },
            GaussianParameters {
                stddev: value,
                ..Default::default()
            },
            GaussianParameters {
                bias_mean: value,
                ..Default::default()
            },
            GaussianParameters {
                bias_stddev: value,
                ..Default::default()
            },
            GaussianParameters {
                dynamic_bias_stddev: value,
                ..Default::default()
            },
            GaussianParameters {
                dynamic_bias_correlation_time: value,
                ..Default::default()
            },
            GaussianParameters {
                precision: value,
                ..Default::default()
            },
        ];
        for p in bad {
            assert!(matches!(
                GaussianNoise::new(p, &mut random),
                Err(InvalidNoise)
            ));
        }
    }
    let mut noise = GaussianNoise::new(
        GaussianParameters {
            mean: f64::MAX,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    let bias = noise.bias();
    for (input, dt) in [
        (f64::NAN, 0.0),
        (0.0, f64::INFINITY),
        (0.0, -1.0),
        (f64::MAX, 0.0),
    ] {
        assert_eq!(noise.apply(input, dt, &mut random), Err(InvalidNoise));
        assert_eq!(noise.bias(), bias);
    }
    let mut dynamic = GaussianNoise::new(
        GaussianParameters {
            dynamic_bias_stddev: f64::MAX,
            dynamic_bias_correlation_time: 1.0,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    assert_eq!(dynamic.apply(0.0, 1.0, &mut random), Err(InvalidNoise));
    assert_eq!(dynamic.bias(), 0.0);
    let mut quantized = GaussianNoise::new(
        GaussianParameters {
            precision: 1e-5,
            ..Default::default()
        },
        &mut random,
    )
    .unwrap();
    assert_eq!(
        quantized.apply(f64::MAX, 0.0, &mut random),
        Err(InvalidNoise)
    );
}

#[test]
fn sampled_bias_overflow_is_reported_instead_of_storing_nonfinite_state() {
    let mut random = stream(12);
    let parameters = GaussianParameters {
        bias_mean: f64::MAX,
        bias_stddev: f64::MAX,
        ..Default::default()
    };
    let mut rejected = 0;
    for _ in 0..100 {
        match GaussianNoise::new(parameters, &mut random) {
            Ok(noise) => assert!(noise.bias().is_finite()),
            Err(InvalidNoise) => rejected += 1,
        }
    }
    assert!(rejected > 0);
}
