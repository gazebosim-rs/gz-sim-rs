// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec2;
use gz_physics::{DeckDirection, EdgeMotion, NormalApproach, PlanarEdgeArc, PlanarEdgeContact};
use std::time::Duration;

#[test]
fn a_loaded_pivot_on_either_edge_preserves_energy_and_resolves_world_impulse() {
    let dt = Duration::from_millis(3);
    let t = dt.as_secs_f64();
    for (sign, deck) in [
        (1., DeckDirection::Negative),
        (-1., DeckDirection::Positive),
    ] {
        let angle: f64 = sign * 0.19;
        let (sn, cs) = angle.sin_cos();
        let c = PlanarEdgeContact {
            mass: 1.25,
            pitch_inertia: 0.04,
            height: 0.10,
            contact_x: sign * 0.03,
            pitch_rate: 0.40 * sign,
            tangent_center_speed: 0.10 * (0.40 * sign),
            acceleration: [9.81 * sn, -9.81 * cs],
            external_torque: 0.,
            friction: 0.8,
        };
        let arc = PlanarEdgeArc {
            contact: c,
            angle,
            sole_bounds: [-0.14, 0.14],
            deck,
        };
        let advance = arc.advance(dt).unwrap().unwrap();
        let start = DVec2::new(
            -c.contact_x * cs + c.height * sn,
            c.contact_x * sn + c.height * cs,
        );
        let vel = c.pitch_rate * DVec2::new(start.y, -start.x);
        let end = DVec2::from(advance.relative_center);
        let energy = |w: f64, z: f64| {
            0.5 * (c.pitch_inertia + c.mass * (c.contact_x.powi(2) + c.height.powi(2))) * w * w
                + c.mass * 9.81 * z
        };
        assert_eq!(advance.motion, EdgeMotion::Sticking);
        assert_eq!(advance.next.contact.contact_x, c.contact_x);
        assert_eq!(advance.lost_energy, 0.);
        assert!(
            (energy(advance.next.contact.pitch_rate, end.y) - energy(c.pitch_rate, start.y)).abs()
                < 1e-12
        );
        let impulse = DVec2::from(advance.normal_impulse) + DVec2::from(advance.tangent_impulse);
        let momentum_change = c.mass * (DVec2::from(advance.relative_velocity) - vel);
        assert!(momentum_change.distance(impulse - DVec2::Y * (9.81 * c.mass * t)) < 1e-12);
        assert!(advance.compressive_impulse > 0.);
        assert!((end.length() - start.length()).abs() < 1e-15);
        // A pivot beyond the physical sole or over the interior face cannot carry this interval.
        assert!(
            PlanarEdgeArc {
                sole_bounds: [0.09, 0.14],
                ..arc
            }
            .advance(dt)
            .unwrap()
            .is_none()
        );
        assert!(
            PlanarEdgeArc {
                deck: if sign > 0. {
                    DeckDirection::Positive
                } else {
                    DeckDirection::Negative
                },
                ..arc
            }
            .advance(dt)
            .unwrap()
            .is_none()
        );
    }
}

#[test]
fn touchdown_clock_selects_the_first_approaching_root_and_rounds_once() {
    let horizon = Duration::from_secs(4);
    let approach = |gap, closing_speed, closing_acceleration| NormalApproach {
        gap,
        closing_speed,
        closing_acceleration,
    };
    // Starts moving away; gravity brings the sole back through its initial gap.
    assert_eq!(
        approach(3., -2., 4.).touchdown(horizon).unwrap(),
        Some(Duration::from_secs_f64((1. + 7_f64.sqrt()) / 2.))
    );
    // Two positive roots (1 and 3 s); the first is the loading event.
    assert_eq!(
        approach(3., 4., -2.).touchdown(horizon).unwrap(),
        Some(Duration::from_secs(1))
    );
    assert_eq!(
        approach(3., 4., -2.)
            .touchdown(Duration::from_millis(999))
            .unwrap(),
        None
    );
    assert_eq!(approach(1., -2., -1.).touchdown(horizon).unwrap(), None);
    assert_eq!(approach(2., 1., -1.).touchdown(horizon).unwrap(), None);
    assert_eq!(approach(1., 0., 0.).touchdown(horizon).unwrap(), None);
    assert_eq!(
        approach(-0.1, -0.2, 0.).touchdown(horizon).unwrap(),
        Some(Duration::ZERO)
    );
    let subnanosecond = approach(0.6e-9, 1., 0.);
    assert_eq!(
        subnanosecond.touchdown_seconds(horizon).unwrap(),
        Some(0.6e-9)
    );
    assert_eq!(
        subnanosecond.touchdown(horizon).unwrap(),
        Some(Duration::from_nanos(1))
    );
    assert!(approach(1., f64::NAN, 1.).touchdown(horizon).is_err());
    assert!(approach(f64::MAX, 1., 2.).touchdown(horizon).is_err());
}
