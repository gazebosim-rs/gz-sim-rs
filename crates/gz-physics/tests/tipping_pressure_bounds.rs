// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec2, DVec3};
use gz_physics::{EdgeSupportState, FrictionlessFaceSupport};

#[test]
fn a_yawing_asymmetric_foot_retains_a_compressive_load_as_its_pressure_moves_along_the_edge() {
    let foot = FrictionlessFaceSupport {
        mass: 2.,
        inertia_body: DMat3::from_cols(
            DVec3::new(0.041, 0.001, -0.008),
            DVec3::new(0.001, 0.052, 0.006),
            DVec3::new(-0.008, 0.006, 0.057),
        ),
        corners: std::array::from_fn(|i| {
            DVec3::new(
                if i < 2 { -0.18 } else { 0.18 },
                if i % 2 == 0 { -0.10 } else { 0.10 },
                -0.13,
            )
        }),
        pad_axes: [DVec3::X, DVec3::Y],
        pad_half: DVec2::new(0.41, 0.20),
        gravity: DVec3::new(0.03, -0.04, -9.81),
    };
    let tilt = DQuat::from_rotation_y(0.076);
    let normal = tilt * DVec3::Z;
    let tangent = tilt * DVec3::X;
    let along_sole = 0.062;
    let roll_rate = -1.865;
    let initial = EdgeSupportState {
        orientation: tilt * DQuat::from_rotation_z(0.758),
        center: 0.41 * DVec3::X + 0.13 * normal + along_sole * tangent + 0.10 * DVec3::Y,
        velocity: 0.124 * tangent - 0.15 * DVec3::Y - along_sole * roll_rate * normal,
        angular_velocity: roll_rate * DVec3::Y - 4.45 * normal,
    };
    let edge = foot.bind_deck_edge(initial, 1).unwrap().unwrap();
    let duration = 0.035;
    let bounds = edge.enclose(duration).unwrap().unwrap();
    assert!(bounds.domains > 1);
    assert!(bounds.normal_force[0] > 0.);
    assert!(bounds.center_along_edge[0] > -foot.pad_half.y);
    assert!(bounds.center_along_edge[1] < foot.pad_half.y);
    let mut trajectory = edge.trajectory();
    for i in 1..=70 {
        let dt = duration / 70.;
        let sample = trajectory.advance(dt, dt / 8.).unwrap();
        assert!((bounds.normal_force[0]..=bounds.normal_force[1]).contains(&sample.normal_force));
        assert!(
            (bounds.center_along_edge[0]..=bounds.center_along_edge[1])
                .contains(&sample.center_along_edge)
        );
        assert!(sample.state.velocity.length() <= bounds.center_speed_bound);
        assert!(sample.state.angular_velocity.length() <= bounds.angular_speed_bound);
        assert!(sample.equivalent_endpoint_force.into_iter().all(|f| f > 0.));
        let elapsed = f64::from(i) * dt;
        let expected_impulse =
            foot.mass * (sample.state.velocity - initial.velocity - elapsed * foot.gravity);
        assert!(sample.integrated_force.distance(expected_impulse) < 1e-12);
    }
    // At this longer horizon the original supporting edge cannot be certified.
    assert!(edge.enclose(0.1).unwrap().is_none());
}
