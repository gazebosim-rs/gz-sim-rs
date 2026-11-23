// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent orbit, subscribed sensor and post-travel impact controls.
use super::*;
use gz_physics::{BodySpec, PhysicsBackend, RotationSpec, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{
    ImuPublication, ImuSample, LidarPublication, LidarSample, set_imu_interest, set_lidar_interest,
};

// Taylor recurrence in signed Q60. For |x| <= 2.01 the i128 intermediates
// stay below 2^124. Terms through degree 37 leave <1e-32 truncation error;
// quantization of each term is below 2^-60. The inputs and final geometry
// use binary64. This reference does not call the engine quaternion integrator.
fn sin_cos_q60(x: f64) -> (f64, f64) {
    const ONE: i128 = 1 << 60;
    assert!(x.abs() <= 2.01);
    let angle = (x * ONE as f64).round() as i128;
    let square = angle * angle / ONE;
    let (mut st, mut ct) = (angle, ONE);
    let (mut sin, mut cos) = (st, ct);
    for k in 1..=18 {
        st = -(st * square / ONE) / ((2 * k) * (2 * k + 1));
        ct = -(ct * square / ONE) / ((2 * k - 1) * (2 * k));
        sin += st;
        cos += ct;
    }
    (sin as f64 / ONE as f64, cos as f64 / ONE as f64)
}
fn spin(v: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    let (sin, cos) = sin_cos_q60(angle);
    v * cos + axis.cross(v) * sin + axis * axis.dot(v) * (1. - cos)
}

#[test]
fn circular_solution_matches_every_link_watched_imu_and_planar_lidar_at_three_step_sizes() {
    let sensors = r#"<sensor name="orbit_imu" type="imu"><always_on>true</always_on><update_rate>100</update_rate></sensor>
<sensor name="orbit_laser" type="lidar"><update_rate>100</update_rate><topic>/scan</topic><frame_id>array</frame_id>
<lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal></scan><range><min>0.01</min><max>4</max><resolution>0.01</resolution></range></lidar></sensor>"#;
    for step_us in [500u64, 1000, 2000] {
        let xml = ARRAY
            .replace(
                "0.001</max_step_size>",
                &format!("{}</max_step_size>", step_us as f64 * 1e-6),
            )
            .replace(
                "<link name=\"left_panel\">",
                &format!("<link name=\"left_panel\">{sensors}"),
            );
        let mut sim = load(&xml).unwrap();
        let rig = Mechanism::bind(&sim, &xml, "satellite");
        let imu = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<ImuPublication>())
            .unwrap()
            .id();
        set_imu_interest(&mut sim, [imu]);
        let lidar = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<LidarPublication>())
            .unwrap()
            .id();
        set_lidar_interest(&mut sim, [lidar]);
        // Keep initial torque bounded as the physical step duration changes.
        rig.prepare(&mut sim, 1., 1_500_000 / step_us, |_, _| {})
            .unwrap();
        let start = rig.sample(&sim);
        begin_model_free_coast(&mut sim, rig.model).unwrap();
        let w = DVec3::from_array(start.links[0].angular_velocity);
        let axis = w.normalize();
        let rate = w.length();
        let dt = step_us as f64 * 1e-6;
        let velocity = start.momentum / rig.mass();
        let offset = DVec3::from_array(start.links[3].position) - start.center;
        let first_scan = sim
            .world()
            .get::<LidarSample>(lidar)
            .unwrap()
            .ranges
            .clone();
        let begin = sim.elapsed();
        let mut last = begin;
        let mut samples = 0;
        let mut worst = [0_f64; 5];
        for tick in 1..=2_000_000 / step_us {
            sim.step_exact(1).unwrap();
            let t = tick as f64 * dt;
            let state = rig.sample(&sim);
            for (actual, initial) in state.links.iter().zip(&start.links) {
                let radial = spin(
                    DVec3::from_array(initial.position) - start.center,
                    axis,
                    rate * t,
                );
                let pos = start.center + t * velocity + radial;
                let vel = velocity + w.cross(radial);
                worst[0] = worst[0].max(pos.distance(DVec3::from_array(actual.position)));
                worst[1] = worst[1].max(vel.distance(DVec3::from_array(actual.velocity)));
            }
            let imu_read = sim.world().get::<ImuSample>(imu).unwrap();
            if imu_read.time <= last {
                continue;
            }
            last = imu_read.time;
            assert_eq!(last, sim.elapsed());
            samples += 1;
            let (s, c) = sin_cos_q60(rate * t * 0.5);
            let q = DQuat::from_xyzw(axis.x * s, axis.y * s, axis.z * s, c)
                * DQuat::from_array(start.links[3].orientation);
            let read = imu_read.reading.as_ref().unwrap();
            let reported = DQuat::from_array(read.orientation);
            worst[2] = worst[2].max((reported - q * reported.dot(q).signum()).length());
            let accel =
                w.cross(spin(offset, axis, rate * t) - spin(offset, axis, rate * (t - dt))) / dt;
            worst[3] = worst[3]
                .max((q.inverse() * accel).distance(DVec3::from_array(read.specific_force)));
            assert!((q.inverse() * w).distance(DVec3::from_array(read.angular_velocity)) < 1e-10);
            let distance = sim
                .world()
                .resource::<PhysicsBackend>()
                .cast_ray(
                    state.links[3].position,
                    (q * DVec3::X).to_array(),
                    4.,
                    Some(rig.links[3].1),
                )
                .unwrap()
                .unwrap();
            let scan = sim.world().get::<LidarSample>(lidar).unwrap();
            assert_eq!(scan.time, imu_read.time);
            assert!((distance - scan.ranges[1]).abs() < 1e-10);
            worst[4] = worst[4].max((distance - 0.245).abs());
            for (&observed, &initial) in scan.ranges.iter().zip(&first_scan) {
                if initial.is_finite() {
                    assert!(observed.is_finite());
                    worst[4] = worst[4].max((observed - initial).abs());
                } else {
                    assert_eq!(observed, initial);
                }
            }
        }
        eprintln!(
            "Q60 orbit at {step_us} us, {samples} watched samples, max |p,v,q,IMU accel,lidar| {worst:?}"
        );
        assert_eq!(samples, 200);
        assert!(worst.into_iter().all(|error| error < 1e-9));
    }
}

#[test]
fn spawned_hazard_hits_the_rotated_panel_from_its_committed_pose() {
    let setup = || {
        let mut sim = load(ARRAY).unwrap();
        let rig = Mechanism::bind(&sim, ARRAY, "satellite");
        rig.prepare(&mut sim, 1., 1000, |_, _| {}).unwrap();
        (sim, rig)
    };
    let (mut coast, rig) = setup();
    let (mut native, nrig) = setup();
    assert_eq!(rig, nrig);
    begin_model_free_coast(&mut coast, rig.model).unwrap();
    let mut impulses = [0.; 2];
    for (i, s) in [&mut coast, &mut native].into_iter().enumerate() {
        s.step_exact(1400).unwrap();
        let panel = rig.sample(s).links[5];
        let position = DVec3::from_array(panel.position) + DVec3::Z * 0.104;
        let v = DVec3::from_array(panel.velocity) - 2. * DVec3::Z;
        s.world_mut()
            .resource_mut::<PhysicsBackend>()
            .spawn_link(
                BodySpec {
                    position: position.to_array(),
                    velocity: v.to_array(),
                    mass: Some(0.8),
                    shape: Shape::Sphere { radius: 0.08 },
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        for _ in 0..20 {
            s.step_exact(1).unwrap();
            assert_eq!(
                model_fidelity(s, rig.model).unwrap(),
                ArticulationTier::Dynamic
            );
            let b = s
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            impulses[i] += b
                .rigid_contact_observations()
                .iter()
                .filter(|p| p.a == rig.links[5].1 || p.b == rig.links[5].1)
                .map(|p| p.normal_impulse)
                .sum::<f64>();
        }
    }
    eprintln!("Post-travel spawned impact impulses (fast, native): {impulses:?}");
    assert!(impulses[0] > 0.3 && (impulses[0] - impulses[1]).abs() < 1e-8);
    let (a, b) = (rig.sample(&coast), rig.sample(&native));
    for (l, r) in a.links.iter().zip(b.links) {
        assert!(DVec3::from_array(l.velocity).distance(DVec3::from_array(r.velocity)) < 1e-8);
    }
}

#[test]
fn tilted_axis_and_offset_link_centers_match_the_exact_orbit_across_variable_steps() {
    use gz_physics::{DynamicBackend, Inertia};
    use std::time::Duration;
    let mut b = MultiFidelityBackend::new([0.; 3]).unwrap();
    let q = DQuat::from_euler(glam::EulerRot::ZYX, 0.45, -0.28, 0.73);
    let axis = q * DVec3::Z;
    let w = 0.8 * axis;
    let origin = DVec3::new(6., -4., 2.);
    let specs = [(0., 0.12, 2.0), (-0.63, 0.08, 1.1), (0.81, -0.13, 0.7)];
    let mass: f64 = specs.iter().map(|v| v.2).sum();
    let center0 =
        origin + q * DVec3::X * (specs.iter().map(|(x, r, m)| (x + r) * m).sum::<f64>() / mass);
    let system_velocity = DVec3::new(0.23, -0.15, 0.07);
    let ids = specs.map(|(x, offset, m)| {
        let p = origin + q * DVec3::X * x;
        let center = p + q * DVec3::X * offset;
        b.spawn_articulation_link(
            BodySpec {
                position: p.to_array(),
                velocity: (system_velocity + w.cross(center - center0)).to_array(),
                mass: Some(m),
                shape: Shape::Sphere { radius: 0.07 },
            },
            RotationSpec {
                orientation: q.to_array(),
                angular_velocity: w.to_array(),
            },
            Some(Inertia {
                center: [offset, 0., 0.],
                orientation: [0., 0., 0., 1.],
                tensor: [0.03, 0.04, 0.05, 0., 0., 0.],
            }),
        )
        .unwrap()
    });
    for (&child, anchor) in ids[1..].iter().zip([-0.32, 0.41]) {
        b.add_reduced_revolute(
            Some(ids[0]),
            child,
            (origin + q * DVec3::X * anchor).to_array(),
            axis.to_array(),
        )
        .unwrap();
    }
    let dt = Duration::from_millis(1);
    b.step(dt).unwrap();
    let initial = ids.map(|id| b.state(id).unwrap());
    let centers = std::array::from_fn::<_, 3, _>(|i| {
        DVec3::from_array(initial[i].position)
            + DQuat::from_array(initial[i].orientation) * DVec3::X * specs[i].1
    });
    let center = centers
        .iter()
        .zip(specs)
        .map(|(c, (_, _, m))| m * c)
        .sum::<DVec3>()
        / mass;
    let v = initial
        .iter()
        .zip(specs)
        .map(|(s, (_, _, m))| m * DVec3::from_array(s.velocity))
        .sum::<DVec3>()
        / mass;
    let w = DVec3::from_array(initial[0].angular_velocity);
    b.begin_articulation_free_coast(ids[0], dt).unwrap();
    let mut time = Duration::ZERO;
    let mut worst = 0_f64;
    for i in 0..2000 {
        let step = Duration::from_micros([250, 500, 1750][i % 3]);
        time += step;
        b.step(step).unwrap();
        let t = time.as_secs_f64();
        for (j, id) in ids.iter().enumerate() {
            let state = b.state(*id).unwrap();
            let radius = spin(centers[j] - center, w.normalize(), w.length() * t);
            let actual_center = DVec3::from_array(state.position)
                + DQuat::from_array(state.orientation) * DVec3::X * specs[j].1;
            worst = worst.max(actual_center.distance(center + t * v + radius));
            worst = worst.max(DVec3::from_array(state.velocity).distance(v + w.cross(radius)));
        }
    }
    eprintln!("Tilted offset-COM orbit at changing dt: max position/COM-speed error {worst:e}");
    assert!(worst < 5e-10);
}

#[test]
fn overflowing_analytic_clock_is_recoverable_before_any_pose_or_joint_commit() {
    use std::time::Duration;
    let mut sim = load(ARRAY).unwrap();
    let rig = Mechanism::bind(&sim, ARRAY, "satellite");
    sim.step_exact(1).unwrap();
    begin_model_free_coast(&mut sim, rig.model).unwrap();
    let mut b = sim.world_mut().resource_mut::<PhysicsBackend>();
    b.step(Duration::MAX).unwrap();
    let poses: Vec<_> = rig
        .links
        .iter()
        .map(|(_, id)| b.state(*id).unwrap())
        .collect();
    let joints: Vec<_> = rig
        .joints
        .iter()
        .map(|(_, id)| b.joint_state(*id).unwrap())
        .collect();
    assert!(matches!(
        b.step(Duration::from_nanos(1)),
        Err(gz_physics::PhysicsError::InvalidValue)
    ));
    for (i, (_, id)) in rig.links.iter().enumerate() {
        assert_eq!(b.state(*id).unwrap(), poses[i]);
    }
    for (i, (_, id)) in rig.joints.iter().enumerate() {
        assert_eq!(b.joint_state(*id).unwrap(), joints[i]);
    }
}

#[test]
fn free_hinge_without_axial_torque_cannot_hold_a_nonprincipal_base_spin() {
    use gz_physics::DynamicBackend;
    use std::time::Duration;
    let mut b = MultiFidelityBackend::new([0.; 3]).unwrap();
    let angular = DVec3::new(0.5, 0., 1.);
    let ids = [-0.5, 0.5].map(|x| {
        let p = DVec3::X * x;
        b.spawn_articulation_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: p.to_array(),
                velocity: angular.cross(p).to_array(),
                mass: Some(1.),
            },
            RotationSpec {
                orientation: [0., 0., 0., 1.],
                angular_velocity: angular.to_array(),
            },
            None,
        )
        .unwrap()
    });
    b.add_reduced_revolute(Some(ids[0]), ids[1], [0.; 3], [1., 0., 0.])
        .unwrap();
    let dt = Duration::from_nanos(1);
    b.step(dt).unwrap();
    let before = ids.map(|id| b.state(id));
    assert!(matches!(
        b.begin_articulation_free_coast(ids[0], Duration::ZERO),
        Err(gz_physics::PhysicsError::InvalidValue)
    ));
    assert!(matches!(
        b.begin_articulation_free_coast(ids[0], dt),
        Err(gz_physics::PhysicsError::TransitionDeferred {
            reason: "free coast spin is not an aggregate principal axis"
        })
    ));
    assert_eq!(before, ids.map(|id| b.state(id)));
}
