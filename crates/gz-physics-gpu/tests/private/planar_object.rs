// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn all_loaded_and_released_floor_sets_satisfy_complementarity() {
    let mass = ContactMass {
        floor: [2., 1.5],
        coupling: [-0.4, 0.3],
        pair: 2.2,
    };
    for a in [false, true] {
        for b in [false, true] {
            for c in [false, true] {
                let expected = [
                    if a { 0.8 } else { 0. },
                    if b { 0.2 } else { 0. },
                    if c { 1.1 } else { 0. },
                ];
                let [x, y, p] = expected;
                let incoming = [
                    (if a { 0. } else { 0.4 }) - 2. * x + 0.4 * p,
                    (if b { 0. } else { 0.6 }) - 1.5 * y - 0.3 * p,
                    (if c { 0. } else { 0.1 }) + 0.4 * x - 0.3 * y - 2.2 * p,
                ];
                let actual = mass.impulses([0.; 3], incoming, 0., 1.);
                for (got, want) in actual.into_iter().zip(expected) {
                    assert!((got - want).abs() < 1e-12, "{actual:?} {expected:?}");
                }
            }
        }
    }
    assert_eq!(mass.impulses([1.; 3], [-5.; 3], 0., 1.), [0.; 3]);
    let immovable = ContactMass {
        floor: [0., 0.],
        coupling: [0., 0.],
        pair: 0.,
    };
    assert_eq!(immovable.impulses([0.; 3], [-1.; 3], 0., 1.), [0.; 3]);
}

#[test]
fn rejects_nonphysical_and_unrepresentable_initial_states_before_device_lookup() {
    let mut tool = PlanarToolLane::zeroed();
    let arm = &mut tool.arm;
    arm.angle = [[1., 0.]; 2];
    arm.target = arm.angle;
    arm.mass = [1.; 2];
    arm.inertia = [0.02; 2];
    arm.length = [0.2; 2];
    arm.center = [0.1; 2];
    tool.radius = 0.02;
    tool.ground_z = -0.1;
    tool.first_contact_offset = -1.;
    let initial = PlanarObjectLane {
        tool,
        object: FreeSphere {
            position: [0.45, -0.08],
            velocity: [0.; 2],
            mass: 0.08,
            radius: 0.02,
            normal_impulse: 0.,
            tool_impulse: [0.; 2],
            floor_impulse: 0.,
            surface: Default::default(),
        },
    };
    assert!(initial.valid());
    for change in [
        (|l: &mut PlanarObjectLane| l.object.position[1] -= 1e-4) as fn(&mut PlanarObjectLane),
        |l| l.object.position = [0.4, 0.],
        |l| l.object.radius = 0.,
        |l| l.object.mass = -1.,
        |l| l.object.mass = f64::from_bits(1),
        |l| l.object.normal_impulse = 1.,
        |l| l.object.tool_impulse[1] = 1.,
        |l| l.object.floor_impulse = 1.,
        |l| l.object.velocity[0] = f64::NAN,
        |l| l.object.position[0] = f64::MAX,
        |l| l.tool.friction = 1.3,
        |l| l.tool.stops[0] = crate::PlanarJointStops::from_relative(0., -0.2, 0.2).unwrap(),
    ] {
        let mut lane = initial;
        change(&mut lane);
        assert!(matches!(
            PlanarObjectBatch::new("absent", vec![lane]),
            Err(GpuError::InvalidLane(0))
        ));
    }
}

#[test]
fn coupled_sphere_floor_and_tool_recover_every_coulomb_mode() {
    let floor = [2., 3.];
    let pair = [[3.5, 0.15], [0.15, 2.5]];
    let cross = [[0.2, 0.1], [-0.4, 0.3]];
    let block = coupled::PairBlock { floor, pair, cross };
    // Store a target impulse and a complementary outgoing contact speed.
    let modes = [
        ([0., 0.], [0.6, 0.2]),
        ([0.8, 0.09], [0., 0.]),
        ([0.8, 0.24], [0., -0.5]),
        ([0.8, -0.24], [0., 0.5]),
    ];
    for (a, va) in modes {
        for (b, vb) in modes {
            let free = [
                std::array::from_fn(|i| {
                    va[i] - floor[i] * a[i] - cross[0][i] * b[0] - cross[1][i] * b[1]
                }),
                std::array::from_fn(|i| vb[i] - dot(pair[i], b) - dot(cross[i], a)),
            ];
            let [fa, fb] = block.solve([0.; 2], free, [0.3; 2], 0., 1.);
            for (got, want) in fa.into_iter().chain(fb).zip(a.into_iter().chain(b)) {
                assert!((got - want).abs() < 1e-12, "{fa:?} {fb:?} vs {a:?} {b:?}");
            }
        }
    }
    assert_eq!(
        block.solve([1.; 2], [[-1.; 2]; 2], [0.3; 2], 0., 1.),
        [[0.; 2]; 2]
    );
}
