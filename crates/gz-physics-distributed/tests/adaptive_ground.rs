// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    adaptive_substeps_with_ground,
    physics::{
        Body, BodyWrench, GroundPlane, Island, PendingImpulse, Primitive, SimulationStep, Snapshot,
        Tier,
    },
};
use std::error::Error;

type TestResult<T = ()> = Result<T, Box<dyn Error>>;

fn body(primitive: Primitive, size: [f64; 3]) -> Body {
    Body {
        id: 1,
        tier: Tier::Dynamic,
        primitive,
        size,
        mass: 1.,
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
        friction: 0.,
    }
}

fn select_ground(b: &Body, plane: GroundPlane) -> TestResult<u8> {
    Ok(adaptive_substeps_with_ground(
        std::slice::from_ref(b),
        SimulationStep::Ms10,
        Some(plane),
        &[],
        [0.; 3],
        &[],
    )?)
}

#[test]
fn oriented_halfspace_resolves_each_native_support_shape() -> TestResult {
    let n = [0.6, 0., 0.8];
    let point = [-3., 2., 7.];
    let plane = GroundPlane::oriented(point, n, 0.8)?;
    for (primitive, size, support) in [
        (Primitive::Box, [0.2, 2., 0.6], 0.6 * 0.1 + 0.8 * 0.3),
        (Primitive::Sphere, [0.6; 3], 0.3),
        (Primitive::Cylinder, [0.2, 0.2, 1.2], 0.6 * 0.1 + 0.8 * 0.6),
        (Primitive::Capsule, [0.2, 0.2, 1.2], 0.1 + 0.8 * 0.6),
    ] {
        let mut b = body(primitive, size);
        b.position = std::array::from_fn(|i| point[i] + (support + 0.03) * n[i]);
        // Tangential translation does not close the gap.
        b.linear = [8., -2., -6.];
        assert_eq!(select_ground(&b, plane)?, 1, "{primitive:?}");
        b.linear = n.map(|x| -5. * x);
        assert_eq!(select_ground(&b, plane)?, 10, "{primitive:?}");
        // Use ten ordinary 1 ms native steps as an independent contact witness.
        let mut fine = Island::restore(&Snapshot {
            bodies: vec![b.clone()],
            ground: Some(plane),
            step: SimulationStep::Ms1,
            ..Default::default()
        })?;
        let mut observed_impulse = 0.;
        for _ in 0..10 {
            fine.step()?;
            observed_impulse += fine
                .snapshot()
                .supports
                .iter()
                .map(|s| s.normal_impulse)
                .sum::<f64>();
        }
        assert!(observed_impulse > 0., "{primitive:?}");
        b.linear = [0.; 3];
        b.position = std::array::from_fn(|i| point[i] + support * n[i]);
        assert_eq!(select_ground(&b, plane)?, 10);
    }
    Ok(())
}

#[test]
fn angular_motion_and_torque_can_bring_a_horizontal_rod_to_ground() -> TestResult {
    let plane = GroundPlane::new(2.)?;
    let half = std::f64::consts::FRAC_1_SQRT_2;
    for primitive in [Primitive::Box, Primitive::Cylinder, Primitive::Capsule] {
        let mut rod = body(primitive, [0.08, 0.08, 2.]);
        rod.orientation = [0., half, 0., half]; // local Z is world X
        rod.position[2] = 2.3;
        assert_eq!(select_ground(&rod, plane)?, 1);
        rod.angular = [0., 50., 0.];
        assert_eq!(select_ground(&rod, plane)?, 10);
        rod.position[2] = 3.2;
        assert_eq!(select_ground(&rod, plane)?, 1);
        rod.position[2] = 2.3;
        rod.angular = [0.; 3];
        let wrench = BodyWrench {
            id: rod.id,
            force: [0.; 3],
            torque: [0., 3000., 0.],
        };
        assert_eq!(
            adaptive_substeps_with_ground(
                &[rod],
                SimulationStep::Ms10,
                Some(plane),
                &[],
                [0.; 3],
                &[wrench]
            )?,
            10,
        );
    }
    let mut sphere = body(Primitive::Sphere, [0.1; 3]);
    sphere.angular = [0., 100., 0.];
    sphere.position[2] = 2.08;
    assert_eq!(select_ground(&sphere, plane)?, 1);
    Ok(())
}

#[test]
fn field_and_impulses_can_close_or_remove_a_predicted_ground_gap() -> TestResult {
    let plane = GroundPlane::oriented([1., 2., 3.], [-1., 0., 0.], 0.)?;
    let mut sphere = body(Primitive::Sphere, [0.4; 3]);
    sphere.position = [0.76, 0., 0.];
    let select = |b: &Body, pending: &[PendingImpulse], gravity, forces: &[BodyWrench]| {
        adaptive_substeps_with_ground(
            std::slice::from_ref(b),
            SimulationStep::Ms10,
            Some(plane),
            pending,
            gravity,
            forces,
        )
    };
    assert_eq!(select(&sphere, &[], [0.; 3], &[])?, 1);
    assert_eq!(select(&sphere, &[], [600., 0., 0.], &[])?, 10);
    assert_eq!(select(&sphere, &[], [-600., 0., 0.], &[])?, 1);
    let mut load = BodyWrench {
        id: sphere.id,
        force: [600., 0., 0.],
        torque: [0.; 3],
    };
    assert_eq!(
        select(&sphere, &[], [0.; 3], std::slice::from_ref(&load))?,
        10
    );
    load.force[0] = -600.;
    assert_eq!(select(&sphere, &[], [600., 0., 0.], &[load])?, 1);
    sphere.linear = [5., 0., 0.];
    assert_eq!(select(&sphere, &[], [0.; 3], &[])?, 10);
    let stop = PendingImpulse {
        id: sphere.id,
        impulse: [-5., 0., 0.],
    };
    assert_eq!(select(&sphere, &[stop], [0.; 3], &[])?, 1);
    sphere.tier = Tier::Prescribed;
    assert_eq!(select(&sphere, &[], [0.; 3], &[])?, 1);
    sphere.tier = Tier::Fixed;
    sphere.mass = 0.;
    sphere.linear = [0.; 3];
    sphere.position[0] = 1.;
    assert_eq!(select(&sphere, &[], [0.; 3], &[])?, 1);
    Ok(())
}

#[test]
fn ground_prediction_skin_and_clamped_travel_bound_the_decision() -> TestResult {
    let plane = GroundPlane::new(-0.5)?;
    let skin = gz_physics_multifidelity::partition_prediction_distance();
    let limit = gz_physics_multifidelity::partition_linear_speed_limit();
    let mut sphere = body(Primitive::Sphere, [1.; 3]);
    sphere.position[2] = skin * 0.5;
    assert_eq!(select_ground(&sphere, plane)?, 10);
    sphere.position[2] = skin * 2.;
    assert_eq!(select_ground(&sphere, plane)?, 1);

    let select = |b: &Body| {
        adaptive_substeps_with_ground(
            std::slice::from_ref(b),
            SimulationStep::Ms10,
            Some(plane),
            &[],
            [0., 0., -2. * limit / 0.01],
            &[],
        )
    };
    sphere.position[2] = 0.5 * limit * 0.01;
    assert_eq!(select(&sphere)?, 10);
    sphere.position[2] = limit * 0.01 + skin * 2.;
    assert_eq!(select(&sphere)?, 1);
    Ok(())
}

#[test]
fn near_unit_pose_keeps_a_grazing_rod_within_the_ground_skin() -> TestResult {
    let plane = GroundPlane::oriented([0.; 3], [1., 0., 0.], 0.)?;
    let skin = gz_physics_multifidelity::partition_prediction_distance();
    let mut rod = body(Primitive::Box, [0.02, 0.02, 2.]);
    rod.position[0] = 1. + skin;
    for scale in [1. - 4e-13, 1. + 4e-13] {
        let h = scale * std::f64::consts::FRAC_1_SQRT_2;
        rod.orientation = [0., h, 0., h];
        assert_eq!(select_ground(&rod, plane)?, 10);
    }
    rod.position[0] += 1e-8;
    assert_eq!(select_ground(&rod, plane)?, 1);
    Ok(())
}
