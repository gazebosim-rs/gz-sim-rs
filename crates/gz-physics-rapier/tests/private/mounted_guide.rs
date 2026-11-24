// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn body(b: &mut RapierBackend, x: f64) -> BodyId {
    b.spawn(BodySpec {
        shape: gz_physics::Shape::Sphere { radius: 0.01 },
        position: [x, 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
    })
    .unwrap()
}
const SLIDER: RobotJoint = RobotJoint::Prismatic {
    axis: [1., 0., 0.],
    limits: Some([-0.02, 0.02]),
};

#[test]
fn only_the_mounted_base_row_changes_even_with_active_motors_and_zero_frequency_stops() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let rail = body(&mut b, 0.);
    let wrist = body(&mut b, 0.5);
    let rail_joint = b.add_robot_joint(None, rail, [0.; 3], SLIDER).unwrap();
    let screw = b
        .add_robot_joint(
            Some(rail),
            wrist,
            [0.5, 0., 0.],
            RobotJoint::Screw {
                axis: [0., 0., 1.],
                pitch: 0.03,
                effort_limit: 3.,
                limits: Some([-0.5, 0.5]),
            },
        )
        .unwrap();
    let spare = body(&mut b, 2.);
    let reduced = b
        .add_limited_revolute(None, spare, [2., 0., 0.], [0., 0., 1.], [-0.2, 0.2])
        .unwrap();
    assert!(matches!(
        b.set_mounted_guide_frequency(reduced, 100000.),
        Err(PhysicsError::InvalidJoint)
    ));
    b.set_joint_angular_speed_limits(screw, [Some(0.8), None])
        .unwrap();
    b.set_joint_velocity_motor(
        rail_joint,
        Some(JointVelocityMotor {
            target_velocity: 0.2,
            max_force: 0.8,
        }),
    )
    .unwrap();
    let records = |b: &RapierBackend| {
        b.world
            .impulse_joints()
            .map(|(h, j)| (h, j.data))
            .collect::<Vec<_>>()
    };
    let baseline = records(&b);
    let Handle::Impulse(guide) = b.joints.records[&rail_joint].handle else {
        panic!()
    };
    assert!(
        baseline
            .iter()
            .any(|(_, d)| d.softness.natural_frequency == 0.)
    );
    for hz in [0., -1., 1e-300, 1e-180, 1_000_001., f64::NAN, f64::INFINITY] {
        assert!(matches!(
            b.set_mounted_guide_frequency(rail_joint, hz),
            Err(PhysicsError::InvalidValue)
        ));
    }
    assert!(matches!(
        b.set_mounted_guide_frequency(screw, 100000.),
        Err(PhysicsError::InvalidJoint)
    ));
    assert!(matches!(
        b.set_mounted_guide_frequency(JointId::from_raw(u64::MAX), 100000.),
        Err(PhysicsError::MissingJoint)
    ));
    assert_eq!(records(&b), baseline);
    b.set_mounted_guide_frequency(rail_joint, 100000.).unwrap();
    let mut expected = baseline;
    expected
        .iter_mut()
        .find(|(h, _)| *h == guide)
        .unwrap()
        .1
        .softness
        .natural_frequency = 100000.;
    assert_eq!(records(&b), expected);
    b.step(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        b.set_mounted_guide_frequency(rail_joint, 100000.),
        Err(PhysicsError::JointConstructionClosed)
    ));
}

#[test]
fn mounted_slider_holds_side_load_and_reaches_both_authored_travel_stops_after_reset() {
    for sign in [-1., 1.] {
        let mut b = RapierBackend::new([0., 0., -9.81]).unwrap();
        let rail = body(&mut b, 0.);
        let j = b.add_robot_joint(None, rail, [0.; 3], SLIDER).unwrap();
        b.set_mounted_guide_frequency(j, 100000.).unwrap();
        let mut previous = None;
        for run in 0..2 {
            b.set_joint_velocity_motor(
                j,
                Some(JointVelocityMotor {
                    target_velocity: sign * 0.5,
                    max_force: 4.,
                }),
            )
            .unwrap();
            for _ in 0..300 {
                b.add_world_wrench(rail, [0., 20., 0.], [0.; 3]).unwrap();
                b.step(Duration::from_millis(1)).unwrap();
            }
            let s = b.state(rail).unwrap();
            let locked_error = s.position[1].hypot(s.position[2]);
            println!(
                "sign={sign} ended x={} m with transverse error {locked_error:e} m",
                s.position[0]
            );
            assert!((s.position[0] - sign * 0.02).abs() < 1e-5);
            assert!(locked_error < 1e-6 && s.velocity[0].abs() < 1e-6);
            if let Some(p) = previous {
                assert_eq!(s, p);
            }
            previous = Some(s);
            if run == 0 {
                b.reset().unwrap();
            }
        }
    }
}
