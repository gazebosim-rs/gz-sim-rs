// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::lidar::{PlanarLidar, PlanarLidarConfig, SpatialLidar, VerticalLidarConfig};
fn horizontal() -> PlanarLidarConfig {
    PlanarLidarConfig {
        samples: 3,
        angle_min: -std::f64::consts::FRAC_PI_4,
        angle_max: std::f64::consts::FRAC_PI_4,
        range_min: 0.1,
        range_max: 10.0,
    }
}
#[test]
fn spatial_rows_are_vertical_major_with_independent_spherical_directions() {
    let v = VerticalLidarConfig {
        samples: 3,
        angle_min: -std::f64::consts::FRAC_PI_6,
        angle_max: std::f64::consts::FRAC_PI_6,
    };
    let model = SpatialLidar::new(horizontal(), v).unwrap();
    assert_eq!(model.directions().len(), 9);
    assert_eq!(model.vertical(), v);
    assert_eq!(model.vertical_angle_step(), std::f64::consts::FRAC_PI_6);
    for (row, elevation) in [
        -std::f64::consts::FRAC_PI_6,
        0.0,
        std::f64::consts::FRAC_PI_6,
    ]
    .into_iter()
    .enumerate()
    {
        for (column, azimuth) in [
            -std::f64::consts::FRAC_PI_4,
            0.0,
            std::f64::consts::FRAC_PI_4,
        ]
        .into_iter()
        .enumerate()
        {
            let actual = model.directions()[row * 3 + column];
            let expected = [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
            for (a, b) in actual.into_iter().zip(expected) {
                assert!((a - b).abs() < 1e-12);
            }
            assert!((actual.into_iter().map(|x| x * x).sum::<f64>() - 1.0).abs() < 1e-12);
        }
    }
    assert_eq!(model.range(None).unwrap(), f64::INFINITY);
    for (hit, expected) in [(0.0, 0.1), (4.125, 4.125), (11.0, 10.0)] {
        assert_eq!(model.range(Some(hit)).unwrap(), expected);
    }
    for hit in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(model.range(Some(hit)).is_err());
    }
}
#[test]
fn default_spatial_row_preserves_every_legacy_planar_direction_bit() {
    for count in [2, 3, 16, 128] {
        let config = PlanarLidarConfig {
            samples: count,
            ..horizontal()
        };
        let old = PlanarLidar::new(config).unwrap();
        let new = SpatialLidar::new(config, VerticalLidarConfig::default()).unwrap();
        assert_eq!(old.angle_step().to_bits(), new.angle_step().to_bits());
        assert_eq!(new.vertical_angle_step().to_bits(), 0.0_f64.to_bits());
        for (a, b) in old.directions().iter().zip(new.directions()) {
            assert_eq!(a.map(f64::to_bits), b.map(f64::to_bits));
        }
        let negative_zero = SpatialLidar::new(
            config,
            VerticalLidarConfig {
                samples: 1,
                angle_min: -0.0,
                angle_max: -0.0,
            },
        )
        .unwrap();
        assert_eq!(
            negative_zero.vertical().angle_min.to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(
            negative_zero.vertical().angle_max.to_bits(),
            0.0_f64.to_bits()
        );
        for hit in [None, Some(0.0), Some(4.125), Some(11.0)] {
            assert_eq!(
                old.range(hit).unwrap().to_bits(),
                new.range(hit).unwrap().to_bits()
            );
        }
    }
}
#[test]
fn spatial_limits_order_underflow_and_product_overflow_are_closed_before_allocation() {
    let valid = VerticalLidarConfig {
        samples: 8,
        angle_min: -0.5,
        angle_max: 0.5,
    };
    assert_eq!(
        SpatialLidar::new(
            PlanarLidarConfig {
                samples: 16,
                ..horizontal()
            },
            valid
        )
        .unwrap()
        .directions()
        .len(),
        128
    );
    for v in [
        VerticalLidarConfig {
            samples: 0,
            ..valid
        },
        VerticalLidarConfig {
            samples: usize::MAX,
            ..valid
        },
        VerticalLidarConfig {
            samples: 1,
            ..valid
        },
        VerticalLidarConfig {
            samples: 9,
            ..valid
        },
        VerticalLidarConfig {
            angle_min: f64::NAN,
            ..valid
        },
        VerticalLidarConfig {
            angle_max: f64::INFINITY,
            ..valid
        },
        VerticalLidarConfig {
            angle_min: 0.6,
            ..valid
        },
        VerticalLidarConfig {
            angle_min: -2.0,
            ..valid
        },
        VerticalLidarConfig {
            angle_max: 2.0,
            ..valid
        },
        VerticalLidarConfig {
            angle_min: 0.0,
            angle_max: f64::from_bits(1),
            ..valid
        },
    ] {
        assert!(
            SpatialLidar::new(
                PlanarLidarConfig {
                    samples: 16,
                    ..horizontal()
                },
                v
            )
            .is_err(),
            "{v:?}"
        );
    }
    assert!(
        SpatialLidar::new(
            PlanarLidarConfig {
                samples: usize::MAX,
                ..horizontal()
            },
            valid
        )
        .is_err()
    );
}
