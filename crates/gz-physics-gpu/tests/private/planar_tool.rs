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
        velocity_mode: [0.; 2],
        velocity_target: [0.; 2],
        stops: Default::default(),
        stop_impulse: [0.; 2],
    }
}
fn advance(mut v: PlanarToolLane) -> PlanarToolLane {
    let words = bytemuck::cast_mut::<_, [f64; WIDTH]>(&mut v);
    for (i, x) in solve::<_, true>(*words, &[0.0005, 0., 1., 0.5]) {
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

#[test]
#[ignore = "requires GZ_GPU_ADAPTER selecting a hardware f64 Vulkan device"]
fn velocity_drives_are_axis_owned_finite_and_clear_at_reset() {
    let mut lane = sample();
    for (mode, speed) in [([2., 0.], [0., 0.]), ([1., 0.], [f64::NAN, 0.])] {
        lane.velocity_mode = mode;
        lane.velocity_target = speed;
        assert!(PlanarToolBatch::new("absent adapter", vec![lane]).is_err());
    }
    lane.velocity_mode = [1., 0.];
    lane.velocity_target = [0.; 2];
    let mut sim = crate::simulation::planar_tool_simulation(
        &std::env::var("GZ_GPU_ADAPTER").unwrap(),
        vec![lane],
    )
    .unwrap();
    let mut batch = sim.world_mut().resource_mut::<PlanarToolBatch>();
    for (row, axis, speed) in [(1, 0, 1.), (0, 1, 1.), (0, 2, 1.), (0, 0, f64::NAN)] {
        assert!(batch.set_motor_speed(row, axis, speed).is_err());
    }
    batch.set_motor_speed(0, 0, 2.).unwrap();
    batch.set_motor_speed(0, 0, 2.).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<PlanarToolBatch>().lanes()[0].velocity_target,
        [2., 0.]
    );
    sim.reset().unwrap();
    assert_eq!(
        sim.world().resource::<PlanarToolBatch>().lanes()[0].velocity_target,
        [0., 0.]
    );
}

#[test]
fn authored_limits_cover_both_sides_and_preserve_the_free_coordinate_momentum() {
    for i in 0..2 {
        for inward in [-1., 1.] {
            let mut l = sample();
            l.arm.angle = [[1., 0.]; 2];
            l.ground_z = -10.;
            l.arm.gravity = 0.;
            l.arm.stiffness = [0.; 2];
            l.arm.damping = [0.; 2];
            l.arm.speed = [0.; 2];
            l.arm.speed[i] = -inward;
            let limits = if inward > 0. {
                [0., f64::INFINITY]
            } else {
                [f64::NEG_INFINITY, 0.]
            };
            l.stops[i] = PlanarJointStops::from_relative(0., limits[0], limits[1]).unwrap();
            let words: &[f64] = bytemuck::cast_slice(std::slice::from_ref(&l));
            let mass = ArmInertia::from_lane(words);
            let mut unit = [0.; 2];
            unit[i] = 1.;
            let col = mass.inverse(1., 0., unit);
            let expected = inward / col[i];
            let hit = advance(l);
            assert!((hit.stop_impulse[i] - expected).abs() < 2e-6);
            assert_eq!(hit.normal_impulse, 0.);
            assert!(hit.joint_angles()[i].abs() < 1e-9);
            assert!((hit.arm.speed[1 - i] - col[1 - i] * expected).abs() < 2e-5);
            l.arm.speed[i] = inward;
            let released = advance(l);
            assert_eq!(released.stop_impulse, [0.; 2]);
        }
    }
}
#[test]
fn paired_stops_and_floor_balance_generalized_momentum_and_respect_unilateral_directions() {
    let l = sample();
    let words: &[f64] = bytemuck::cast_slice(std::slice::from_ref(&l));
    let m = ArmInertia::from_lane(words);
    let (c, s) = (0.8, 0.6);
    for normal in [[0.5, 0.16], [0.2, -0.11], [0.3, 0.]] {
        let tangent = [0.06, -0.03];
        let an = m.inverse(c, s, normal);
        let at = m.inverse(c, s, tangent);
        for side0 in [-1., 0., 1.] {
            for side1 in [-1., 0., 1.] {
                for w0 in [-2., -0.1, 0., 0.4, 1.2] {
                    for w1 in [-1.1, -0.2, 0., 0.6, 1.7] {
                        let speed = [w0, w1];
                        let sides = [side0, side1];
                        for floor in [false, true] {
                            let floor_only = if floor {
                                impact(
                                    [dot(normal, speed), dot(tangent, speed)],
                                    [dot(normal, an), dot(tangent, an), dot(tangent, at)],
                                    0.3,
                                    0.,
                                    1.,
                                )
                            } else {
                                [0.; 2]
                            };
                            let (w, [n, t], p) = planar_stops::solve(
                                &m, c, s, speed, sides, [0.; 2], floor, normal, tangent, 0.3,
                                floor_only, 0., 1.,
                            );
                            let delta = m.momentum(c, [w[0] - w0, w[1] - w1]);
                            assert!(n >= 0. && t.abs() <= 0.3 * n + 1e-12);
                            assert!(!floor || dot(normal, w) >= -1e-11);
                            for i in 0..2 {
                                assert!(
                                    sides[i] * w[i] >= -1e-11,
                                    "sides={sides:?}, sample={speed:?}, result={w:?}"
                                );
                                assert!(sides[i] * p[i] >= -1e-11);
                                assert!(
                                    (delta[i] - p[i] - normal[i] * n - tangent[i] * t).abs()
                                        < 1e-11
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}
#[test]
fn finite_stops_reject_invalid_ranges_and_retain_multiturn_order() {
    let tau = std::f64::consts::TAU;
    for (origin, lo, hi) in [
        (f64::NAN, 0., 0.),
        (0., 0.1, 0.2),
        (0., -1., -0.1),
        (0., f64::NAN, 1.),
        (0., 0., f64::MAX),
    ] {
        assert!(PlanarJointStops::from_relative(origin, lo, hi).is_none());
    }
    for dir in [-1., 1.] {
        let mut l = sample();
        l.arm.angle = [[1., 0.]; 2];
        l.arm.speed = [dir * 1000., 0.];
        l.ground_z = -10.;
        l.arm.gravity = 0.;
        l.arm.damping = [0.; 2];
        l.arm.stiffness = [0.; 2];
        let (lo, hi) = if dir > 0. {
            (-tau, 1.25 * tau)
        } else {
            (-1.25 * tau, tau)
        };
        l.stops[0] = PlanarJointStops::from_relative(0., lo, hi).unwrap();
        for _ in 0..30 {
            l = advance(l);
            assert!(l.joint_angles()[0] >= lo - 1e-12 && l.joint_angles()[0] <= hi + 1e-12);
        }
    }
}

#[test]
fn measured_arm_landing_integrates_the_remaining_load_on_the_same_public_tick() {
    let mut l = sample();
    let q = [0.48_f64 - 0.1352207501553569, -0.62 + 0.11955202500386763];
    l.arm.angle = q.map(|q| [q.cos(), q.sin()]);
    l.arm.speed = [-2.7371816473033457, 0.2431650870780966];
    l.arm.length = [0.30, 0.21];
    l.arm.center = [0.15, 0.12];
    l.arm.inertia = [0.0062, 0.0034];
    l.arm.tip_mass = 0.;
    l.arm.gravity = 9.81;
    l.arm.damping = [6., 1.5];
    l.arm.effort_limit = [8., 2.8];
    l.radius = 0.018;
    l.ground_z = 0.05;
    l.velocity_mode = [1.; 2];
    l.velocity_target = [-2.4, 0.4479749961323609];
    let result = advance(l);
    // The independent Cartesian integration uses 512 steps from this incoming cut.
    assert!((result.first_contact_offset - 0.00064909209).abs() < 2e-6);
    assert!((result.first_normal_impulse - 0.29729368).abs() < 0.00012);
    assert!((result.normal_impulse - 0.30321932).abs() < 0.00035);
    assert!((result.tangent_impulse + 0.0909658).abs() < 0.00012);
}
