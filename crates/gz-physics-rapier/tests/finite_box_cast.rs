// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent Parry shape intersection at either side of finite-box events.
use gz_physics::{FiniteBoxArrival, FiniteBoxFlight, FlightBox, SeparatingFeature};
use rapier3d_f64::parry::{query, shape::Cuboid};
use rapier3d_f64::prelude::{Pose, Rotation, Vector};
use std::time::Duration;
#[test]
fn finite_foot_cast_cross_checks_three_feature_geometries_and_off_deck_flights() {
    let mut counts = [0; 5];
    let half = Vector::new(0.18, 0.10, 0.06);
    let pad = Vector::new(0.33, 0.18, 0.06);
    for k in 0..4096 {
        let signed = |n: i32| 2. * f64::from((k * n + 17) % 997) / 996. - 1.;
        let rot = Rotation::from_scaled_axis(Vector::new(
            signed(93) * 0.16,
            signed(107) * 0.12,
            signed(297) * 0.7,
        ));
        let extent = half.x * (rot * Vector::X).z.abs()
            + half.y * (rot * Vector::Y).z.abs()
            + half.z * (rot * Vector::Z).z.abs();
        let body = FlightBox {
            center: Vector::new(signed(83) * 0.51, signed(31) * 0.32, extent + 0.00018),
            orientation: rot,
            half_extent: half,
        };
        let support = FlightBox {
            center: Vector::Z * (-pad.z),
            orientation: Rotation::IDENTITY,
            half_extent: pad,
        };
        let flight = FiniteBoxFlight {
            moving: body,
            support,
            center_of_mass: body.center + rot * (0.07 * Vector::Z),
            relative_velocity: Vector::new(signed(101), signed(43), -1.6),
            angular_velocity: Vector::new(signed(67) * 2., signed(19) * 3., signed(317) * 8.),
            acceleration: Vector::new(signed(139) * 5., 0., -9.81),
        };
        let intersects = |t: f64| {
            let f = flight.at(t);
            query::intersection_test(
                &Pose::from_parts(f.center, f.orientation),
                &Cuboid::new(half),
                &Pose::from_parts(support.center, support.orientation),
                &Cuboid::new(pad),
            )
            .unwrap()
            .intersecting
        };
        let dt = Duration::from_micros(if k % 2 == 0 { 500 } else { 25_000 });
        match flight.first_crossing(dt).unwrap() {
            FiniteBoxArrival::Crossing(c) => {
                let t = c.earliest.as_secs_f64();
                assert!(!intersects((t - 1e-7).max(0.)), "early {k}");
                assert!(intersects(c.latest.as_secs_f64() + 1e-7), "late {k}");
                counts[match c.feature {
                    SeparatingFeature::SupportFace(_) => 0,
                    SeparatingFeature::MovingFace(_) => 1,
                    SeparatingFeature::EdgePair(_, _) => 2,
                }] += 1;
            }
            FiniteBoxArrival::Separated => {
                for t in 0..=10 {
                    assert!(
                        !intersects(dt.as_secs_f64() * f64::from(t) / 10.),
                        "missed {k}"
                    );
                }
                counts[3] += 1;
            }
            FiniteBoxArrival::Unresolved { .. } => counts[4] += 1,
            FiniteBoxArrival::InitiallyTouching => panic!("unexpected initial overlap"),
        }
    }
    assert!(counts[..4].iter().all(|n| *n > 10), "{counts:?}");
    eprintln!("Parry vertex/face, face/vertex, edge/edge, separated, unresolved: {counts:?}");
}
