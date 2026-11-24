// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::f64::consts::FRAC_PI_2;

// Both masses are centered on the spherical pivot. The wrist can carry its
// own weight, but a slider at its lower stop can carry only one sign of load.
#[test]
fn the_slider_stop_sees_the_actual_orientation_of_its_spherical_parent() {
    for (angle, supported) in [(FRAC_PI_2, false), (-FRAC_PI_2, true)] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let spec = BodySpec {
            shape: Shape::Sphere { radius: 0.02 },
            position: [0.; 3],
            velocity: [0.; 3],
            mass: Some(1.),
        };
        let palm = b.spawn(spec).unwrap();
        let tool = b.spawn(spec).unwrap();
        b.add_robot_joint(
            None,
            palm,
            [0.; 3],
            RobotJoint::Ball {
                orientation: Rotation::from_rotation_z(0.23).to_array(),
            },
        )
        .unwrap();
        let slider = b
            .add_robot_joint(
                Some(palm),
                tool,
                [0.; 3],
                RobotJoint::Prismatic {
                    axis: [1., 0., 0.],
                    limits: Some([0., 0.08]),
                },
            )
            .unwrap();
        let dt = Duration::from_millis(1);
        b.step(dt).unwrap();
        // A consistent, motionless pose at each of the two reachable turns.
        let turn = Rotation::from_rotation_y(angle);
        for id in [palm, tool] {
            b.world.bodies[b.bodies[&id].0].set_rotation(turn, true);
        }
        b.world.gravity = Vector::NEG_Z * 9.81;
        let axis = b.mounted_axis(b.joints.records[&slider].initial, Vector::X);
        assert!((axis - turn * Vector::X).length() < 1e-13);
        let result = b.switch_articulation(palm, ArticulationTier::Prescribed, dt);
        if supported {
            result.unwrap();
        } else {
            assert!(
                matches!(result,Err(PhysicsError::TransitionDeferred{reason}) if reason.contains("gravity"))
            );
        }
    }
}
