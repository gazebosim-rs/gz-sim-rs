// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn sample() -> PlanarToolLane {
    PlanarToolLane {
        arm: PlanarArmLane {
            angle: [[1., 0.], [0., -1.]],
            speed: [0.; 2],
            length: [0.4, 0.2],
            center: [0.2, 0.1],
            mass: [0.8, 0.4],
            inertia: [0.02, 0.006],
            tip_mass: 0.1,
            gravity: 0.,
            target: [[1., 0.], [0., -1.]],
            stiffness: [0.; 2],
            damping: [0.; 2],
            effort_limit: [8., 3.],
            effort: [0.; 2],
        },
        ground_z: -0.22,
        radius: 0.02,
        friction: 0.3,
        normal_impulse: 0.,
        tangent_impulse: 0.,
        first_contact_offset: -1.,
        first_normal_impulse: 0.,
        first_tangent_impulse: 0.,
        turns: [0.; 2],
    }
}
fn advance(mut v: PlanarToolLane) -> PlanarToolLane {
    let words = bytemuck::cast_mut::<_, [f64; WIDTH]>(&mut v);
    for (i, x) in solve(*words, &[0.0005, 0., 1., 0.5]) {
        words[i] = x;
    }
    v
}
#[test]
fn explicit_coulomb_impulses_satisfy_coupled_stick_and_sliding() {
    for (vn, vt, mu) in [
        (-1., 0., 0.8),
        (-1., 0.3, 0.7),
        (-1., -4., 0.3),
        (-1., 4., 0.3),
        (-1., 2., 0.),
    ] {
        let [nn, nt, tt] = [2., 0.4, 1.2];
        let [n, t] = impact([vn, vt], [nn, nt, tt], mu, 0., 1.);
        let slip = vt + nt * n + tt * t;
        assert!(n > 0. && (vn + n * nn + t * nt).abs() < 1e-13 && t.abs() <= mu * n + 1e-14);
        if slip.abs() > 1e-12 {
            assert!((t + slip.signum() * mu * n).abs() < 1e-13);
        }
    }
    assert_eq!(impact([-1., -2.], [1., 2., 4.], 1., 0., 1.), [1., 0.]);
    assert_eq!(impact([1., 0.], [1., 0., 1.], 0.5, 0., 1.), [0.; 2]);
    assert_eq!(impact([-1., 0.], [0., 0., 1.], 0.5, 0., 1.), [0.; 2]);
    assert_eq!(impact([-1., 4.], [1., 0., 0.], 0., 0., 1.), [1., 0.]);
}
#[test]
fn free_swing_never_reports_stale_floor_impulse() {
    let mut v = sample();
    v.ground_z = -2.;
    v.arm.speed = [-1., 1.];
    v.normal_impulse = 3.;
    v.tangent_impulse = -0.2;
    v.first_normal_impulse = 1.;
    v.first_contact_offset = 0.;
    let result = advance(v);
    assert_eq!(
        [
            result.normal_impulse,
            result.tangent_impulse,
            result.first_normal_impulse,
            result.first_tangent_impulse
        ],
        [0.; 4]
    );
    assert_eq!(result.first_contact_offset, -1.);
    assert_ne!(result.arm.angle, v.arm.angle);
}
#[test]
fn first_closing_impact_redirects_articulated_velocity_and_liftoff_unloads() {
    let mut v = sample();
    v.arm.speed = [-1., 0.];
    v.ground_z = v.arm.points()[1][1] - v.radius - 0.0001;
    let after = advance(v);
    assert!(after.normal_impulse > 0. && after.first_normal_impulse > 0.);
    assert!((0.00020..0.00030).contains(&after.first_contact_offset));
    assert!(after.tangent_impulse.abs() <= v.friction * after.normal_impulse + 1e-14);
    assert!(after.gap() > -1e-7);
    let mut lift = after;
    lift.arm.speed = [1., 0.];
    let released = advance(lift);
    assert!(
        released.gap() > after.gap()
            && released.normal_impulse == 0.
            && released.first_contact_offset == -1.
    );
}
#[test]
fn invalid_floor_or_impulse_history_fails_before_device_lookup() {
    let mut l = sample();
    l.ground_z = -0.23;
    assert!(l.valid());
    let changes: [fn(&mut PlanarToolLane); 10] = [
        |v| v.arm.speed[0] = f64::NAN,
        |v| v.radius = f64::NAN,
        |v| v.radius = 0.,
        |v| v.ground_z = 1.,
        |v| v.ground_z = f64::INFINITY,
        |v| v.friction = -1.,
        |v| v.normal_impulse = 1.,
        |v| v.tangent_impulse = 0.1,
        |v| v.first_contact_offset = 0.,
        |v| v.first_normal_impulse = 0.1,
    ];
    for edit in changes {
        let mut wrong = l;
        edit(&mut wrong);
        assert!(matches!(
            PlanarToolBatch::new("fake", vec![wrong]),
            Err(GpuError::InvalidLane(0))
        ));
    }
}
#[test]
#[ignore = "requires GZ_GPU_ADAPTER selecting a hardware f64 Vulkan device"]
fn contact_target_changes_are_transactional_and_reset_restores_initial_frame() {
    let mut l = sample();
    l.ground_z = -0.23;
    let mut sim = crate::simulation::planar_tool_simulation(
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        vec![l],
    )
    .unwrap();
    sim.step_exact(1).unwrap();
    let mut batch = sim.world_mut().resource_mut::<PlanarToolBatch>();
    let before = batch.lanes()[0];
    assert!(batch.set_target_angles(1, [1.; 2]).is_err());
    assert!(batch.set_target_angles(0, [f64::NAN, 0.]).is_err());
    assert_eq!(
        bytemuck::bytes_of(&before),
        bytemuck::bytes_of(&batch.lanes()[0])
    );
    batch.set_target_angles(0, [1., -0.4]).unwrap();
    assert_eq!(batch.lanes()[0].arm.angle, before.arm.angle);
    sim.reset().unwrap();
    assert_eq!(
        bytemuck::bytes_of(&l),
        bytemuck::bytes_of(&sim.world().resource::<PlanarToolBatch>().lanes()[0])
    );
}

#[test]
fn the_public_clock_retains_multiple_hinge_turns_in_either_direction() {
    for speed in [-10000., 10000.] {
        let mut v = sample();
        v.ground_z = -4.;
        v.arm.angle = [[1., 0.]; 2];
        v.arm.speed = [speed, 0.];
        let increment = 4. * (0.00025_f64 * speed).atan();
        assert!(increment.abs() > std::f64::consts::PI);
        for tick in 1..=25 {
            v = advance(v);
            assert!((v.joint_angles()[0] - f64::from(tick) * increment).abs() < 1e-11);
        }
    }
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with hardware f64 Vulkan"]
fn device_tracks_opposite_fast_revolutions_across_public_ticks() {
    let speeds = [-10000., 10000.];
    let initial = speeds.map(|speed| {
        let mut l = sample();
        l.ground_z = -4.;
        l.arm.angle = [[1., 0.]; 2];
        l.arm.speed = [speed, 0.];
        l
    });
    let mut sim = crate::simulation::planar_tool_simulation(
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        initial.to_vec(),
    )
    .unwrap();
    sim.step_exact(25).unwrap();
    for (arm, speed) in sim
        .world()
        .resource::<PlanarToolBatch>()
        .lanes()
        .iter()
        .zip(speeds)
    {
        let expected = 100. * (0.00025_f64 * speed).atan();
        assert!((arm.joint_angles()[0] - expected).abs() < 1e-11);
        assert_eq!(arm.joint_angles()[1], 0.);
    }
}
