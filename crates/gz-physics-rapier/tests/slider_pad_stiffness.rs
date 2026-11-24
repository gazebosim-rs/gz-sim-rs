// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, JointId, PhysicsError, RotationSpec, Shape};
use gz_physics_rapier::{RapierBackend, RobotJoint};
use std::time::Duration;

const SIZE: f64 = 0.02;
const FORCE: f64 = 3.;
const STEP: Duration = Duration::from_micros(500);

fn body(b: &mut RapierBackend, shape: Shape, x: f64, lane: f64, mass: f64) -> BodyId {
    b.spawn_link(
        BodySpec {
            shape,
            position: [x, lane, 0.],
            velocity: [0.; 3],
            mass: Some(mass),
        },
        RotationSpec::default(),
        None,
    )
    .unwrap()
}

#[derive(Clone, Copy)]
struct Pair {
    pads: [BodyId; 2],
    slides: [JointId; 2],
}

impl Pair {
    fn new(b: &mut RapierBackend, k: [Option<f64>; 2], flat: bool, lane: f64) -> Self {
        Self::with_gap(b, k, flat, lane, 0.)
    }

    fn with_gap(
        b: &mut RapierBackend,
        k: [Option<f64>; 2],
        flat: bool,
        lane: f64,
        gap: f64,
    ) -> Self {
        let offset = SIZE + gap / 2.;
        let pads = std::array::from_fn(|i| {
            let x = if i == 0 { -offset } else { offset };
            let shape = if flat || i == 1 {
                Shape::Box {
                    half_extents: [SIZE; 3],
                }
            } else {
                Shape::Sphere { radius: SIZE }
            };
            body(b, shape, x, lane, if i == 0 { 0.08 } else { 0.2 })
        });
        let slides = std::array::from_fn(|i| {
            let axis = if i == 0 { 1. } else { -1. };
            let id = b
                .add_reduced_scalar_robot_joint(
                    None,
                    pads[i],
                    [-axis * offset, lane, 0.],
                    RobotJoint::Prismatic {
                        axis: [axis, 0., 0.],
                        limits: Some([-0.01, 0.01]),
                    },
                )
                .unwrap();
            if let Some(k) = k[i] {
                b.set_initial_pad_stiffness(pads[i], k).unwrap();
            }
            id
        });
        Self { pads, slides }
    }

    fn compression(self, b: &RapierBackend) -> f64 {
        let x = self.pads.map(|id| b.state(id).unwrap().position[0]);
        2.0 * SIZE - (x[1] - x[0])
    }

    fn squeeze(self, b: &mut RapierBackend) {
        for j in self.slides {
            b.add_joint_effort(j, FORCE).unwrap();
        }
    }
}

#[test]
fn one_or_two_coatings_set_the_total_point_or_flat_patch_stiffness_after_reset() {
    for (k, effective) in [
        ([Some(15000.), None], 15000.),
        ([None, Some(15000.)], 15000.),
        ([Some(10000.), Some(30000.)], 7500.),
    ] {
        for flat in [false, true] {
            let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
            let pair = Pair::new(&mut b, k, flat, 0.);
            let mut repeats = Vec::new();
            for _ in 0..2 {
                for _ in 0..8000 {
                    pair.squeeze(&mut b);
                    b.step(STEP).unwrap();
                }
                let patches = b.rigid_contact_patches();
                assert_eq!(patches.len(), 1);
                assert_eq!(patches[0].points.len(), if flat { 4 } else { 1 });
                let force = patches[0]
                    .points
                    .iter()
                    .map(|p| p.normal_impulse)
                    .sum::<f64>()
                    / STEP.as_secs_f64();
                assert!((force - FORCE).abs() < 1e-5);
                let compression = pair.compression(&b);
                assert!(
                    (compression - FORCE / effective).abs() < 1e-7,
                    "{k:?} flat={flat} x={compression} predicted={}",
                    FORCE / effective
                );
                repeats.push((compression, force));
                b.reset().unwrap();
            }
            assert_eq!(repeats[0], repeats[1]);
        }
    }
}

#[test]
fn a_coating_only_changes_the_selected_links_in_its_own_grasp() {
    let run = |k| {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let changed = Pair::new(&mut b, [k, None], false, 0.);
        let ordinary = Pair::new(&mut b, [None, None], true, 0.5);
        let mut trace = Vec::new();
        for _ in 0..500 {
            changed.squeeze(&mut b);
            ordinary.squeeze(&mut b);
            b.step(STEP).unwrap();
            trace.push((
                ordinary.compression(&b),
                b.state(ordinary.pads[1]).unwrap().velocity,
            ));
        }
        (changed.compression(&b), trace)
    };
    let baseline = run(None);
    let coated = run(Some(25000.));
    assert_ne!(baseline.0, coated.0);
    assert_eq!(baseline.1, coated.1);
}

#[test]
fn only_bounded_reduced_sliders_accept_a_positive_construction_stiffness() {
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let shape = Shape::Sphere { radius: SIZE };
    let unjoined = body(&mut b, shape, 0., 1., 0.1);
    assert_eq!(
        b.set_initial_pad_stiffness(unjoined, 1.),
        Err(PhysicsError::InvalidJoint)
    );
    let bounded = Pair::new(&mut b, [None; 2], false, 0.);
    for k in [-1., 0., f64::NAN, f64::INFINITY] {
        assert_eq!(
            b.set_initial_pad_stiffness(bounded.pads[0], k),
            Err(PhysicsError::InvalidValue)
        );
    }
    for (i, joint) in [
        RobotJoint::Prismatic {
            axis: [1., 0., 0.],
            limits: Some([-0.01, 0.01]),
        },
        RobotJoint::Prismatic {
            axis: [1., 0., 0.],
            limits: None,
        },
        RobotJoint::Revolute {
            axis: [0., 0., 1.],
            limits: None,
        },
    ]
    .into_iter()
    .enumerate()
    {
        let lane = 2. + i as f64;
        let id = body(&mut b, shape, 0., lane, 0.1);
        if i == 0 {
            b.add_robot_joint(None, id, [0., lane, 0.], joint).unwrap();
        } else {
            b.add_reduced_scalar_robot_joint(None, id, [0., lane, 0.], joint)
                .unwrap();
        }
        assert_eq!(
            b.set_initial_pad_stiffness(id, 1.),
            Err(PhysicsError::InvalidJoint)
        );
    }
    for k in [f64::from_bits(1), f64::MAX] {
        b.set_initial_pad_stiffness(bounded.pads[0], k).unwrap();
        bounded.squeeze(&mut b);
        b.step(STEP).unwrap();
        assert!(bounded.compression(&b).is_finite());
        b.reset().unwrap();
    }
    b.step(STEP).unwrap();
    assert_eq!(
        b.set_initial_pad_stiffness(bounded.pads[0], 1.),
        Err(PhysicsError::JointConstructionClosed)
    );
}

#[test]
fn two_coated_slides_follow_the_same_normal_response_on_a_point_and_a_face() {
    // With two equal opposing forces, relative compression obeys
    // m*x'' + c*x' + k*x = F, where m is the reduced mass of the two pads.
    let m: f64 = 1. / (1. / 0.08 + 1. / 0.2);
    let k: f64 = 7500.; // 10,000 and 30,000 N/m in series.
    let omega = (k / m).sqrt();
    let root = (10_f64.powi(2) - 1.).sqrt();
    let slow = -omega / (10. + root);
    let fast = -omega * (10. + root);
    let expected = |t: f64| {
        FORCE / k * (1. + (fast * (slow * t).exp() - slow * (fast * t).exp()) / (slow - fast))
    };
    for flat in [false, true] {
        let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let pair = Pair::new(&mut b, [Some(10000.), Some(30000.)], flat, 0.);
        let mut error: f64 = 0.;
        for tick in 1..=2000 {
            pair.squeeze(&mut b);
            b.step(Duration::from_micros(50)).unwrap();
            let t = f64::from(tick) * 50e-6;
            error = error.max((pair.compression(&b) - expected(t)).abs());
        }
        eprintln!("flat={flat} peak_position_error_um={:.5}", error * 1e6);
        assert!(error < 1e-6);
    }
}

#[test]
fn separated_pads_carry_no_normal_load_at_rest_or_in_free_approach() {
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let pair = Pair::with_gap(&mut b, [Some(30000.), None], false, 0., 50e-6);
    let dt = Duration::from_micros(25);
    let normal_impulse = |b: &RapierBackend| {
        b.rigid_contact_patches()
            .iter()
            .flat_map(|p| &p.points)
            .map(|p| p.normal_impulse)
            .sum::<f64>()
    };
    for _ in 0..10 {
        b.step(dt).unwrap();
        assert_eq!(normal_impulse(&b), 0.);
    }
    let mut free_ticks = 0;
    let accel = FORCE * (1. / 0.08 + 1. / 0.2);
    for _ in 0..100 {
        let gap = -pair.compression(&b);
        let [vl, vr] = pair.pads.map(|id| b.state(id).unwrap().velocity[0]);
        // Using end-of-tick speed bounds the free advance for all substeps.
        let free_travel = ((vl - vr) + accel * dt.as_secs_f64()) * dt.as_secs_f64();
        pair.squeeze(&mut b);
        b.step(dt).unwrap();
        if gap > free_travel {
            free_ticks += 1;
            assert_eq!(normal_impulse(&b), 0.);
        }
    }
    assert!(free_ticks > 40);
    assert!(normal_impulse(&b) > 0.);
}
