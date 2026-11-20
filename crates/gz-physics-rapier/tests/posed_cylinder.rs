// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, PhysicsError, PosedCylinderBodySpec,
    RapierBackend, RotationSpec, Shape,
};
use std::{
    f64::consts::{FRAC_1_SQRT_2, FRAC_PI_4, PI},
    time::Duration,
};

const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

fn spec() -> PosedCylinderBodySpec {
    PosedCylinderBodySpec {
        radius: 0.6,
        half_height: 1.4,
        collision_position: [-2.0, 1.0, 0.75],
        collision_orientation: [0.0, FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2],
        position: [0.0, 0.0, 4.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

fn inertia() -> Inertia {
    Inertia {
        center: [0.0; 3],
        orientation: IDENTITY,
        tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
    }
}

fn ball(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.25 },
        position,
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

fn step(backend: &mut RapierBackend, count: usize) {
    for _ in 0..count {
        backend.step(Duration::from_millis(1)).unwrap();
    }
}

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite() && expected.is_finite());
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

fn same_state(actual: BodyState, expected: BodyState) {
    for (actual, expected) in actual
        .position
        .into_iter()
        .zip(expected.position)
        .chain(actual.velocity.into_iter().zip(expected.velocity))
        .chain(actual.orientation.into_iter().zip(expected.orientation))
        .chain(
            actual
                .angular_velocity
                .into_iter()
                .zip(expected.angular_velocity),
        )
    {
        close(actual, expected);
    }
}

fn ray(
    backend: &RapierBackend,
    origin: [f64; 3],
    direction: [f64; 3],
    exclude: Option<BodyId>,
) -> Option<f64> {
    backend.cast_ray(origin, direction, 10.0, exclude).unwrap()
}

// Independent fixed-angle trigonometry: no Rapier or production transform helper.
fn body_y([x, y, z]: [f64; 3]) -> [f64; 3] {
    let (s, c) = (PI / 6.0).sin_cos();
    [c * x + s * z, y, -s * x + c * z]
}

fn local_x([x, y, z]: [f64; 3]) -> [f64; 3] {
    let (s, c) = FRAC_PI_4.sin_cos();
    [x, c * y - s * z, s * y + c * z]
}

fn along(point: [f64; 3], axis: [f64; 3], distance: f64) -> [f64; 3] {
    std::array::from_fn(|i| point[i] + distance * axis[i])
}

#[test]
fn posed_cylinder_analytic_axial_radial_and_tangent_rays_compose_both_frames() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let position = [3.0, -4.0, 5.0];
    let local_position = [-3.0, 2.0, 1.0];
    let rotation = RotationSpec {
        orientation: [0.0, (PI / 12.0).sin(), 0.0, (PI / 12.0).cos()],
        ..RotationSpec::default()
    };
    let id = backend
        .spawn_posed_cylinder_link(
            PosedCylinderBodySpec {
                radius: 0.5,
                half_height: 1.25,
                collision_position: local_position,
                collision_orientation: [(PI / 8.0).sin(), 0.0, 0.0, (PI / 8.0).cos()],
                position,
                mass: None,
                ..spec()
            },
            rotation,
            None,
        )
        .unwrap();
    let offset = body_y(local_position);
    let center = std::array::from_fn(|i| position[i] + offset[i]);
    for (axis, extent) in [
        ([1.0, 0.0, 0.0], 0.5),
        ([0.0, 1.0, 0.0], 0.5),
        ([0.0, 0.0, 1.0], 1.25),
    ] {
        let normal = body_y(local_x(axis));
        for sign in [-1.0, 1.0] {
            let direction = normal.map(|value| value * sign);
            let origin = along(center, direction, -extent - 2.0);
            close(ray(&backend, origin, direction, None).unwrap(), 2.0);
            assert_eq!(ray(&backend, origin, direction, Some(id)), None);
        }
    }
    assert_eq!(ray(&backend, center, [1.0, 0.0, 0.0], None), Some(0.0));
    let state = backend.state(id).unwrap();
    assert_eq!(state.position, position);
    assert_eq!(state.orientation, rotation.orientation);
    assert_eq!(state.velocity, [0.0; 3]);
    assert_eq!(state.angular_velocity, [0.0; 3]);
    assert_eq!(backend.is_dynamic(id), Some(false));
    // Tangent generators meet the end cap at the same height as interior rays.
    // A just-outside parallel ray must miss, including before any physics step.
    let mut tangent = RapierBackend::new([0.0; 3]).unwrap();
    tangent
        .spawn_posed_cylinder_link(
            PosedCylinderBodySpec {
                radius: 0.5,
                half_height: 1.25,
                collision_position: [1.0, -2.0, 0.5],
                collision_orientation: IDENTITY,
                position: [2.0, -2.0, 4.5],
                mass: None,
                ..spec()
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    for x in [3.0, 3.49, 3.5] {
        close(
            ray(&tangent, [x, -4.0, 1.75], [0.0, 0.0, 1.0], None).unwrap(),
            2.0,
        );
    }
    assert_eq!(
        ray(&tangent, [3.51, -4.0, 1.75], [0.0, 0.0, 1.0], None),
        None
    );
    let mut centered = RapierBackend::new([0.0; 3]).unwrap();
    centered
        .spawn_link(
            BodySpec {
                shape: Shape::Cylinder {
                    radius: 0.5,
                    half_height: 1.25,
                },
                position,
                velocity: [0.0; 3],
                mass: None,
            },
            rotation,
            None,
        )
        .unwrap();
    let direction = body_y(local_x([0.0, 0.0, 1.0]));
    let origin = along(center, direction, -3.25);
    assert_eq!(ray(&centered, origin, direction, None), None);
    close(ray(&backend, origin, direction, None).unwrap(), 2.0);
}

fn support(backend: &mut RapierBackend) -> BodyId {
    backend
        .spawn_posed_cylinder_link(
            PosedCylinderBodySpec {
                radius: 0.6,
                half_height: 1.4,
                collision_position: [2.0, 0.0, 1.0],
                collision_orientation: IDENTITY,
                position: [0.0, 0.0, 2.0],
                mass: None,
                ..spec()
            },
            RotationSpec::default(),
            None,
        )
        .unwrap()
}

#[test]
fn posed_static_cylinder_contacts_at_analytic_height_without_moving_link_origin() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let floor = support(&mut backend);
    let floor_state = backend.state(floor).unwrap();
    let body = backend.spawn(ball([2.0, 0.0, 5.0])).unwrap();
    let mut old = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    old.spawn(BodySpec {
        shape: Shape::Cylinder {
            radius: 0.6,
            half_height: 1.4,
        },
        position: [0.0, 0.0, 2.0],
        velocity: [0.0; 3],
        mass: None,
    })
    .unwrap();
    let unsupported = old.spawn(ball([2.0, 0.0, 5.0])).unwrap();
    step(&mut backend, 1500);
    step(&mut old, 1500);
    let landed = backend.state(body).unwrap();
    // The translated cylinder cap is at 2+1+half-height1.4; ball radius is .25.
    assert!((landed.position[2] - 4.65).abs() < 0.01, "{landed:?}");
    assert!(landed.velocity[2].abs() < 0.01, "{landed:?}");
    assert_eq!(backend.state(floor), Some(floor_state));
    assert!(old.state(unsupported).unwrap().position[2] < 0.0);
    backend.reset().unwrap();
    assert_eq!(backend.state(floor), Some(floor_state));
    assert_eq!(backend.state(body).unwrap().position, [2.0, 0.0, 5.0]);
    assert!(backend.remove(floor));
    assert!(!backend.remove(floor));
    step(&mut backend, 1500);
    assert!(backend.state(body).unwrap().position[2] < 0.0);
}

#[test]
fn posed_dynamic_cylinder_keeps_explicit_centered_inertia_and_world_impulses() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let spec = PosedCylinderBodySpec {
        velocity: [0.5, -0.25, 0.75],
        ..spec()
    };
    let rotation = RotationSpec {
        orientation: [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2],
        angular_velocity: [0.1, 0.2, 0.3],
    };
    let rotated_inertia = Inertia {
        orientation: [FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2],
        ..inertia()
    };
    let id = backend
        .spawn_posed_cylinder_link(spec, rotation, Some(rotated_inertia))
        .unwrap();
    let old = control
        .spawn_link(
            BodySpec {
                shape: Shape::Cylinder {
                    radius: spec.radius,
                    half_height: spec.half_height,
                },
                position: spec.position,
                velocity: spec.velocity,
                mass: spec.mass,
            },
            rotation,
            Some(rotated_inertia),
        )
        .unwrap();
    let initial = backend.state(id).unwrap();
    assert_eq!(initial.position, spec.position);
    assert_eq!(initial.orientation, rotation.orientation);
    assert_eq!(initial.velocity, spec.velocity);
    for (world, body) in [(&mut backend, id), (&mut control, old)] {
        world
            .apply_impulse(body, [2.0, 4.0, -2.0], [4.0, 2.0, 3.0])
            .unwrap();
        let after = world.state(body).unwrap();
        for (actual, expected) in after.velocity.into_iter().zip([1.5, 1.75, -0.25]) {
            close(actual, expected);
        }
        // Rbody Z90 * Rinertia X90 maps [2,3,4] to world principal moments [4,2,3].
        for (actual, expected) in after.angular_velocity.into_iter().zip([1.1, 1.2, 1.3]) {
            close(actual, expected);
        }
        world
            .add_world_wrench(body, [0.5, -1.0, 2.0], [0.1, 0.2, -0.3])
            .unwrap();
    }
    for _ in 0..100 {
        step(&mut backend, 1);
        step(&mut control, 1);
        same_state(backend.state(id).unwrap(), control.state(old).unwrap());
    }
    backend.reset().unwrap();
    assert_eq!(backend.state(id), Some(initial));
    let mut no_spin = RapierBackend::new([0.0; 3]).unwrap();
    let body = no_spin
        .spawn_posed_cylinder_link(
            PosedCylinderBodySpec {
                position: [0.0; 3],
                velocity: [0.0; 3],
                ..spec
            },
            RotationSpec::default(),
            Some(inertia()),
        )
        .unwrap();
    assert_eq!(no_spin.state(body).unwrap().velocity, [0.0; 3]);
    no_spin
        .add_world_wrench(body, [4.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    step(&mut no_spin, 1);
    close(no_spin.state(body).unwrap().velocity[0], 0.002);
    for angular in no_spin.state(body).unwrap().angular_velocity {
        close(angular, 0.0);
    }
}

#[test]
fn posed_cylinder_identity_matches_centered_static_and_explicit_dynamic_construction() {
    for mass in [None, Some(2.0)] {
        let mut posed = RapierBackend::new([0.0; 3]).unwrap();
        let mut old = RapierBackend::new([0.0; 3]).unwrap();
        let spec = PosedCylinderBodySpec {
            collision_position: [0.0; 3],
            collision_orientation: IDENTITY,
            mass,
            ..spec()
        };
        let inertia = Some(inertia());
        let id = posed
            .spawn_posed_cylinder_link(spec, RotationSpec::default(), inertia)
            .unwrap();
        let control = old
            .spawn_link(
                BodySpec {
                    shape: Shape::Cylinder {
                        radius: spec.radius,
                        half_height: spec.half_height,
                    },
                    position: spec.position,
                    velocity: spec.velocity,
                    mass,
                },
                RotationSpec::default(),
                inertia,
            )
            .unwrap();
        assert_eq!(posed.state(id), old.state(control));
        assert_eq!(
            ray(&posed, [0.0; 3], [0.0, 0.0, 1.0], None),
            ray(&old, [0.0; 3], [0.0, 0.0, 1.0], None)
        );
        for (world, body) in [(&mut posed, id), (&mut old, control)] {
            world
                .apply_impulse(body, [1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
                .unwrap();
        }
        step(&mut posed, 10);
        step(&mut old, 10);
        assert_eq!(posed.state(id), old.state(control));
    }
}

#[test]
fn posed_cylinder_groups_and_owned_geometry_survive_both_reset_paths_and_removal() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let floor = support(&mut backend);
    let body = backend.spawn(ball([2.0, 0.0, 5.0])).unwrap();
    backend.set_self_collision_group(floor, Some(7)).unwrap();
    backend.set_self_collision_group(body, Some(7)).unwrap();
    let captured = backend.initial_state();
    let initial = backend.state(body).unwrap();
    for _ in 0..2 {
        step(&mut backend, 1500);
        assert!(backend.state(body).unwrap().position[2] < 0.0);
        backend.reset().unwrap();
        assert_eq!(backend.state(body), Some(initial));
        close(
            ray(&backend, [2.0, 0.0, 0.0], [0.0, 0.0, 1.0], Some(body)).unwrap(),
            1.6,
        );
    }
    let prepared = backend.prepare_initial(&captured).unwrap();
    assert!(backend.remove(floor));
    assert!(backend.remove(body));
    let runtime = backend.spawn(ball([10.0, 0.0, 5.0])).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(runtime), None);
    assert_eq!(backend.state(body), Some(initial));
    close(
        ray(&backend, [2.0, 0.0, 0.0], [0.0, 0.0, 1.0], Some(body)).unwrap(),
        1.6,
    );
    step(&mut backend, 1500);
    assert!(backend.state(body).unwrap().position[2] < 0.0);
    backend.reset().unwrap();
    backend.set_self_collision_group(body, None).unwrap();
    step(&mut backend, 1500);
    assert!((backend.state(body).unwrap().position[2] - 4.65).abs() < 0.01);
    let next = backend.spawn(ball([20.0, 0.0, 5.0])).unwrap();
    assert!(next > runtime);
}

fn rejected(
    spec: PosedCylinderBodySpec,
    rotational: RotationSpec,
    inertia: Option<Inertia>,
    expected: PhysicsError,
) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(ball([-20.0, 0.0, 0.0])).unwrap();
    assert_eq!(control.spawn(ball([-20.0, 0.0, 0.0])).unwrap(), id);
    for world in [&mut backend, &mut control] {
        world
            .add_world_wrench(id, [2.0, 4.0, 6.0], [0.1, 0.2, 0.3])
            .unwrap();
    }
    let before = backend.state(id);
    assert_eq!(
        backend.spawn_posed_cylinder_link(spec, rotational, inertia),
        Err(expected),
        "spec={spec:?}, rotation={rotational:?}, inertia={inertia:?}"
    );
    assert_eq!(backend.state(id), before);
    assert_eq!(backend.state(BodyId::from_raw(1)), None);
    let next = backend.spawn(ball([20.0, 0.0, 0.0])).unwrap();
    assert_eq!(next, BodyId::from_raw(1));
    assert_eq!(control.spawn(ball([20.0, 0.0, 0.0])).unwrap(), next);
    backend.step(Duration::ZERO).unwrap();
    assert_eq!(backend.state(id), before);
    step(&mut backend, 1);
    step(&mut control, 1);
    same_state(backend.state(id).unwrap(), control.state(id).unwrap());
    for (actual, expected) in backend
        .state(id)
        .unwrap()
        .velocity
        .into_iter()
        .zip([0.001, 0.002, 0.003])
    {
        close(actual, expected);
    }
    let velocity = backend.state(id).unwrap().velocity;
    step(&mut backend, 1);
    step(&mut control, 1);
    same_state(backend.state(id).unwrap(), control.state(id).unwrap());
    for (actual, expected) in backend
        .state(id)
        .unwrap()
        .velocity
        .into_iter()
        .zip(velocity)
    {
        close(actual, expected);
    }
}

#[test]
fn posed_cylinder_invalid_numbers_dimensions_and_composed_bounds_refuse_atomically() {
    for value in [
        PosedCylinderBodySpec {
            position: [f64::NAN; 3],
            ..spec()
        },
        PosedCylinderBodySpec {
            collision_position: [f64::INFINITY; 3],
            ..spec()
        },
        PosedCylinderBodySpec {
            velocity: [f64::NAN; 3],
            ..spec()
        },
        PosedCylinderBodySpec {
            collision_orientation: [0.0; 4],
            ..spec()
        },
        PosedCylinderBodySpec {
            collision_orientation: [0.0, 0.0, 0.1, 1.0],
            ..spec()
        },
        PosedCylinderBodySpec {
            collision_orientation: [f64::NAN; 4],
            ..spec()
        },
        PosedCylinderBodySpec {
            position: [f64::MAX, 0.0, 0.0],
            collision_position: [f64::MAX, 0.0, 0.0],
            ..spec()
        },
        // A cancelling world translation must not rescue collapsed local bounds.
        PosedCylinderBodySpec {
            position: [-f64::MAX, 0.0, 0.0],
            collision_position: [f64::MAX, 0.0, 0.0],
            ..spec()
        },
        PosedCylinderBodySpec {
            collision_position: [f64::MAX, 0.0, 0.0],
            ..spec()
        },
        PosedCylinderBodySpec {
            position: [f64::MAX, 0.0, 0.0],
            ..spec()
        },
        PosedCylinderBodySpec {
            mass: None,
            velocity: [0.1, 0.0, 0.0],
            ..spec()
        },
    ] {
        rejected(
            value,
            RotationSpec::default(),
            Some(inertia()),
            PhysicsError::InvalidValue,
        );
    }
    for dimension in [
        0.0,
        -0.5,
        f64::NAN,
        f64::INFINITY,
        f64::MAX,
        f64::MIN_POSITIVE,
        f64::from_bits(1),
    ] {
        for value in [
            PosedCylinderBodySpec {
                radius: dimension,
                ..spec()
            },
            PosedCylinderBodySpec {
                half_height: dimension,
                ..spec()
            },
        ] {
            rejected(
                value,
                RotationSpec::default(),
                Some(inertia()),
                PhysicsError::InvalidValue,
            );
        }
    }
    for mass in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::MAX,
        f64::from_bits(1),
    ] {
        rejected(
            PosedCylinderBodySpec {
                mass: Some(mass),
                ..spec()
            },
            RotationSpec::default(),
            Some(inertia()),
            PhysicsError::InvalidValue,
        );
    }
    for rotational in [
        RotationSpec {
            orientation: [0.0; 4],
            ..RotationSpec::default()
        },
        RotationSpec {
            orientation: [f64::NAN; 4],
            ..RotationSpec::default()
        },
        RotationSpec {
            orientation: [0.0, 0.1, 0.0, 1.0],
            ..RotationSpec::default()
        },
        RotationSpec {
            angular_velocity: [f64::INFINITY; 3],
            ..RotationSpec::default()
        },
    ] {
        rejected(
            spec(),
            rotational,
            Some(inertia()),
            PhysicsError::InvalidValue,
        );
    }
    rejected(
        PosedCylinderBodySpec {
            mass: None,
            ..spec()
        },
        RotationSpec {
            angular_velocity: [0.0, 0.0, 1.0],
            ..RotationSpec::default()
        },
        Some(inertia()),
        PhysicsError::InvalidValue,
    );
}

#[test]
fn posed_cylinder_inertia_refusals_preserve_ids_state_and_pending_wrenches() {
    rejected(
        spec(),
        RotationSpec::default(),
        None,
        PhysicsError::Unsupported {
            engine: "rapier",
            capability: "dynamic posed cylinder without explicit inertia",
        },
    );
    for mass in [None, Some(2.0)] {
        rejected(
            PosedCylinderBodySpec { mass, ..spec() },
            RotationSpec::default(),
            Some(Inertia {
                center: [-0.25, 0.0, 0.0],
                ..inertia()
            }),
            PhysicsError::Unsupported {
                engine: "rapier",
                capability: "posed cylinder offset inertia",
            },
        );
    }
    for value in [
        Inertia {
            center: [f64::NAN; 3],
            ..inertia()
        },
        Inertia {
            orientation: [0.0; 4],
            ..inertia()
        },
        Inertia {
            tensor: [f64::MAX; 6],
            ..inertia()
        },
        Inertia {
            tensor: [-1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            ..inertia()
        },
        Inertia {
            tensor: [10.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            ..inertia()
        },
    ] {
        rejected(
            spec(),
            RotationSpec::default(),
            Some(value),
            PhysicsError::InvalidValue,
        );
    }
}
