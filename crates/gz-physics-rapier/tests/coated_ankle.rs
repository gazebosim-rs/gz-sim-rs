// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::sole_contact::SoleMaterial;
use gz_physics_rapier::{BodyId, BodySpec, CoatedAnkle, DynamicBackend, RapierBackend, Shape};
use std::time::Duration;

fn fixture(upper_shape: Shape) -> (RapierBackend, CoatedAnkle) {
    let mut b = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let mut make = |shape, z, mass| {
        b.spawn(BodySpec {
            shape,
            position: [0., 0., z],
            velocity: [0.; 3],
            mass,
        })
        .unwrap()
    };
    let foot = make(
        Shape::Box {
            half_extents: [0.18, 0.1, 0.025],
        },
        0.0248,
        Some(1.),
    );
    let shank = make(upper_shape, 0.27, Some(1.));
    let floor = make(
        Shape::Box {
            half_extents: [0.5, 0.5, 0.02],
        },
        -0.02,
        None,
    );
    let c = CoatedAnkle {
        foot,
        shank,
        floor,
        material: SoleMaterial::new(2e6, 3e6, 2.5, 0.32, 0.015).unwrap(),
        depth: 0,
        substeps: 4,
    };
    (b, c)
}
#[test]
fn only_a_floating_reduced_ankle_and_eligible_plantar_collision_exchange_load() {
    let mut dt = Duration::from_micros(500);
    let (mut b, c) = fixture(Shape::Box {
        half_extents: [0.04, 0.05, 0.17],
    });
    assert!(b.validate_coated_ankle(c, dt).is_err());
    b.add_reduced_revolute(Some(c.foot), c.shank, [0., 0., 0.07], [0., 1., 0.])
        .unwrap();
    for bad in [
        CoatedAnkle { foot: c.shank, ..c },
        CoatedAnkle { floor: c.foot, ..c },
        CoatedAnkle {
            floor: BodyId::from_raw(999),
            ..c
        },
        CoatedAnkle { depth: 7, ..c },
    ] {
        assert!(b.step_coated_ankle(bad, dt).is_err());
    }
    let before = [c.foot, c.shank].map(|id| b.state(id).unwrap());
    b.set_self_collision_group(c.foot, Some(7)).unwrap();
    b.set_self_collision_group(c.floor, Some(7)).unwrap();
    assert!(b.step_coated_ankle(c, dt).is_err());
    b.set_self_collision_group(c.floor, None).unwrap();
    assert_eq!(before, [c.foot, c.shank].map(|id| b.state(id).unwrap()));
    let row = b.step_coated_ankle(c, dt).unwrap();
    assert!(row.lower.normal_impulse > 0.);
    dt = Duration::ZERO;
    assert!(b.step_coated_ankle(c, dt).is_err());
    let (mut b, c) = fixture(Shape::Sphere { radius: 0.02 });
    b.add_reduced_revolute(Some(c.foot), c.shank, [0., 0., 0.07], [0., 1., 0.])
        .unwrap();
    assert!(
        b.validate_coated_ankle(c, Duration::from_micros(500))
            .is_err()
    );
}
