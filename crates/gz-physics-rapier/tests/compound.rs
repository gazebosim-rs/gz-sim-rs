// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia,
    PhysicsError, RapierBackend, RotationSpec, Shape,
};
use std::{f64::consts::FRAC_1_SQRT_2, time::Duration};

const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
const Y90: [f64; 4] = [0.0, FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2];
const Z90: [f64; 4] = [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2];

fn child(shape: Shape, position: [f64; 3]) -> CollisionSpec {
    CollisionSpec {
        shape,
        position,
        orientation: IDENTITY,
    }
}
fn mixed() -> [CollisionSpec; 4] {
    [
        Shape::Box {
            half_extents: [0.25, 0.5, 0.75],
        },
        Shape::Cylinder {
            radius: 0.5,
            half_height: 1.5,
        },
        Shape::Capsule {
            radius: 0.5,
            half_length: 1.5,
        },
        Shape::Sphere { radius: 0.5 },
    ]
    .map(|shape| child(shape, [0.0; 3]))
}
fn spec(collisions: &[CollisionSpec]) -> CompoundBodySpec<'_> {
    CompoundBodySpec {
        collisions,
        position: [0.0; 3],
        velocity: [0.0; 3],
        mass: None,
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
    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
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
fn distance(backend: &RapierBackend, origin: [f64; 3], direction: [f64; 3], expected: f64) {
    close(
        backend
            .cast_ray(origin, direction, 20.0, None)
            .unwrap()
            .unwrap(),
        expected,
    );
}

#[test]
fn compound_mixed_primitives_keep_noncommuting_frames_strict_chords_and_concave_gaps() {
    let mut children = mixed();
    for (index, collision) in children.iter_mut().enumerate() {
        collision.position = [-6.0 + 4.0 * index as f64, 0.0, 0.0];
        collision.orientation = Y90;
    }
    // Link Z90 maps local child X to world Y. Child Y90 then maps each native
    // Gazebo-Z axis to world Y; rotating the translation again gives wrong centers.
    let rotation = RotationSpec {
        orientation: Z90,
        ..RotationSpec::default()
    };
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn_compound_link(
            CompoundBodySpec {
                position: [3.0, -4.0, 5.0],
                ..spec(&children)
            },
            rotation,
            None,
        )
        .unwrap();
    for (y, center_hit, chord_hit) in [
        (-10.0, 3.75, 3.75),
        (-6.0, 3.5, 3.6),
        (-2.0, 3.5, 3.6),
        (2.0, 3.5, 3.6),
    ] {
        distance(&backend, [3.0, y, 9.0], [0.0, 0.0, -1.0], center_hit);
        distance(&backend, [3.3, y, 9.0], [0.0, 0.0, -1.0], chord_hit);
        assert_eq!(
            backend.cast_ray([3.0, y, 9.0], [0.0, 0.0, -1.0], 20.0, Some(id)),
            Ok(None)
        );
        assert_eq!(
            backend.cast_ray([3.0, y, 5.0], [1.0, 0.0, 0.0], 20.0, None),
            Ok(Some(0.0))
        );
    }
    // This line passes through the compound AABB, but through neither child.
    assert_eq!(
        backend.cast_ray([3.0, -8.0, 9.0], [0.0, 0.0, -1.0], 20.0, None),
        Ok(None)
    );
    distance(&backend, [3.0, -13.0, 5.0], [0.0, 1.0, 0.0], 2.25);
    // The capsule's hemispherical tip is y=0, not the cylinder's flat y=-.5.
    distance(&backend, [3.0, 1.0, 5.0], [0.0, -1.0, 0.0], 1.0);
    let state = backend.state(id).unwrap();
    assert_eq!(state.position, [3.0, -4.0, 5.0]);
    assert_eq!(state.orientation, Z90);
    assert_eq!(backend.is_dynamic(id), Some(false));
    // Public rays use the current body pose immediately after an explicit edit.
    let edited = BodyState {
        position: [4.0, -4.0, 5.0],
        ..state
    };
    backend.set_body_state(id, edited).unwrap();
    distance(&backend, [4.3, -6.0, 9.0], [0.0, 0.0, -1.0], 3.6);
    backend.reset().unwrap();
    assert_eq!(backend.state(id), Some(state));
    distance(&backend, [3.3, -6.0, 9.0], [0.0, 0.0, -1.0], 3.6);
}

#[test]
fn compound_rays_keep_endpoints_nearest_whole_owner_exclusion_and_child_errors() {
    let children = [
        child(Shape::Sphere { radius: 0.5 }, [-2.0, 0.0, 0.0]),
        child(Shape::Sphere { radius: 0.5 }, [2.0, 0.0, 0.0]),
    ];
    for reverse in [false, true] {
        let mut ordered = children;
        if reverse {
            ordered.reverse();
        }
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let owner = backend
            .spawn_compound_link(spec(&ordered), RotationSpec::default(), None)
            .unwrap();
        assert_eq!(
            backend.cast_ray([-2.0, 0.0, 4.0], [0.0, 0.0, -1.0], 3.499, None),
            Ok(None)
        );
        for max in [3.5, 3.501] {
            close(
                backend
                    .cast_ray([-2.0, 0.0, 4.0], [0.0, 0.0, -1.0], max, None)
                    .unwrap()
                    .unwrap(),
                3.5,
            );
        }
        distance(&backend, [-2.0, 0.5, 4.0], [0.0, 0.0, -1.0], 4.0);
        assert_eq!(
            backend.cast_ray([-2.0, 0.501, 4.0], [0.0, 0.0, -1.0], 20.0, None),
            Ok(None)
        );
        assert_eq!(
            backend.cast_ray([-2.0, 0.0, 0.5], [0.0, 0.0, 1.0], 20.0, None),
            Ok(Some(0.0))
        );
        assert_eq!(
            backend.cast_ray([-2.0, 0.0, 4.0], [0.0, 0.0, 1.0], 20.0, None),
            Ok(None)
        );
        let behind = backend
            .spawn(BodySpec {
                position: [-2.0, 0.0, -3.0],
                mass: None,
                ..ball([0.0; 3])
            })
            .unwrap();
        distance(&backend, [-2.0, 0.0, 4.0], [0.0, 0.0, -1.0], 3.5);
        close(
            backend
                .cast_ray([-2.0, 0.0, 4.0], [0.0, 0.0, -1.0], 20.0, Some(owner))
                .unwrap()
                .unwrap(),
            6.75,
        );
        assert_eq!(
            backend.cast_ray([2.0, 0.0, 4.0], [0.0, 0.0, -1.0], 20.0, Some(owner)),
            Ok(None)
        );
        assert!(backend.remove(owner));
        assert!(!backend.remove(owner));
        assert!(backend.state(behind).is_some());
        assert_eq!(
            backend.cast_ray([0.0; 3], [1.0, 0.0, 0.0], 20.0, Some(owner)),
            Err(PhysicsError::MissingBody)
        );
    }
    // The ordinary cylinder is admitted, but this tiny transverse query offset
    // underflows its unchanged kernel's squared cross product. A prior inside
    // hit on the sphere cannot hide that error, in either declaration order.
    for reverse in [false, true] {
        let mut parts = [
            child(Shape::Sphere { radius: 1.0 }, [0.0; 3]),
            child(
                Shape::Cylinder {
                    radius: 0.5,
                    half_height: 1.5,
                },
                [0.0; 3],
            ),
        ];
        if reverse {
            parts.reverse();
        }
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let owner = backend
            .spawn_compound_link(spec(&parts), RotationSpec::default(), None)
            .unwrap();
        assert_eq!(
            backend.cast_ray([1e-200, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, None),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(
            backend.cast_ray([1e-200, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, Some(owner)),
            Ok(None)
        );
    }
}

#[test]
fn compound_explicit_centered_inertia_and_world_impulses_ignore_asymmetric_child_centers() {
    let mut children = mixed();
    for (i, member) in children.iter_mut().enumerate() {
        member.position = [2.0 + i as f64, -1.0, 0.5];
    }
    let rotational = RotationSpec {
        orientation: Z90,
        angular_velocity: [0.1, 0.2, 0.3],
    };
    let rotated_inertia = Inertia {
        orientation: [FRAC_1_SQRT_2, 0.0, 0.0, FRAC_1_SQRT_2],
        ..inertia()
    };
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let owner = backend
        .spawn_compound_link(
            CompoundBodySpec {
                position: [0.0, 0.0, 5.0],
                velocity: [0.5, -0.25, 0.75],
                mass: Some(2.0),
                ..spec(&children)
            },
            rotational,
            Some(rotated_inertia),
        )
        .unwrap();
    let centered = control
        .spawn_link(
            BodySpec {
                position: [0.0, 0.0, 5.0],
                velocity: [0.5, -0.25, 0.75],
                mass: Some(2.0),
                ..ball([0.0; 3])
            },
            rotational,
            Some(rotated_inertia),
        )
        .unwrap();
    let initial = backend.state(owner).unwrap();
    for (world, id) in [(&mut backend, owner), (&mut control, centered)] {
        world
            .apply_impulse(id, [2.0, 4.0, -2.0], [4.0, 2.0, 3.0])
            .unwrap();
        let after = world.state(id).unwrap();
        for (actual, expected) in after.velocity.into_iter().zip([1.5, 1.75, -0.25]) {
            close(actual, expected);
        }
        // Body Z90 * inertia X90 maps principal moments[2,3,4] to[4,2,3].
        for (actual, expected) in after.angular_velocity.into_iter().zip([1.1, 1.2, 1.3]) {
            close(actual, expected);
        }
        world
            .add_world_wrench(id, [0.5, -1.0, 2.0], [0.1, 0.2, -0.3])
            .unwrap();
    }
    for _ in 0..100 {
        step(&mut backend, 1);
        step(&mut control, 1);
        same_state(
            backend.state(owner).unwrap(),
            control.state(centered).unwrap(),
        );
    }
    backend.reset().unwrap();
    assert_eq!(backend.state(owner), Some(initial));
    let mut no_spin = RapierBackend::new([0.0; 3]).unwrap();
    let id = no_spin
        .spawn_compound_link(
            CompoundBodySpec {
                mass: Some(2.0),
                ..spec(&children)
            },
            RotationSpec::default(),
            Some(inertia()),
        )
        .unwrap();
    assert_eq!(no_spin.state(id).unwrap().velocity, [0.0; 3]);
    no_spin
        .add_world_wrench(id, [4.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    step(&mut no_spin, 1);
    close(no_spin.state(id).unwrap().velocity[0], 0.002);
    for angular in no_spin.state(id).unwrap().angular_velocity {
        close(angular, 0.0);
    }
}

fn support(backend: &mut RapierBackend) -> BodyId {
    let children = [-2.0, 2.0].map(|x| {
        child(
            Shape::Box {
                half_extents: [1.0, 1.0, 0.5],
            },
            [x, 0.0, 0.5],
        )
    });
    backend
        .spawn_compound_link(spec(&children), RotationSpec::default(), None)
        .unwrap()
}

#[test]
fn compound_contacts_and_self_groups_distinguish_member_surfaces_from_the_gap() {
    for group in [None, Some(7)] {
        let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
        let floor = support(&mut backend);
        let original = backend.state(floor).unwrap();
        let on_member = backend.spawn(ball([2.0, 0.0, 3.0])).unwrap();
        let in_gap = backend.spawn(ball([0.0, 0.0, 3.0])).unwrap();
        backend.set_self_collision_group(floor, group).unwrap();
        backend.set_self_collision_group(on_member, group).unwrap();
        step(&mut backend, 1500);
        let landed = backend.state(on_member).unwrap();
        if group.is_none() {
            assert!((landed.position[2] - 1.25).abs() < 0.01, "{landed:?}");
            assert!(landed.velocity[2].abs() < 0.01, "{landed:?}");
        } else {
            assert!(landed.position[2] < 0.0);
        }
        assert!(backend.state(in_gap).unwrap().position[2] < 0.0);
        assert_eq!(backend.state(floor), Some(original));
        assert!(backend.remove(floor));
        assert_eq!(backend.state(floor), None);
    }
}

#[test]
fn compound_ordinary_and_prepared_resets_restore_geometry_groups_and_id_high_water() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let floor = support(&mut backend);
    let body = backend.spawn(ball([2.0, 0.0, 3.0])).unwrap();
    let survivor = backend.spawn(ball([20.0, 0.0, 3.0])).unwrap();
    backend.set_self_collision_group(floor, Some(7)).unwrap();
    backend.set_self_collision_group(body, Some(7)).unwrap();
    let initial = backend.state(body).unwrap();
    let captured = backend.initial_state();
    step(&mut backend, 1500);
    assert!(backend.state(body).unwrap().position[2] < 0.0);
    backend.reset().unwrap();
    assert_eq!(backend.state(body), Some(initial));
    close(
        backend
            .cast_ray([2.0, 0.0, 3.0], [0.0, 0.0, -1.0], 10.0, Some(body))
            .unwrap()
            .unwrap(),
        2.0,
    );
    let prepared = backend.prepare_initial(&captured).unwrap();
    assert!(backend.remove(floor));
    backend.reset().unwrap();
    assert_eq!(backend.state(floor), None);
    assert_eq!(
        backend.cast_ray([2.0, 0.0, 3.0], [0.0, 0.0, -1.0], 10.0, Some(body)),
        Ok(None)
    );
    let runtime = backend.spawn(ball([30.0, 0.0, 3.0])).unwrap();
    backend
        .add_world_wrench(body, [100.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(runtime), None);
    assert_eq!(backend.state(body), Some(initial));
    assert!(backend.state(survivor).is_some());
    close(
        backend
            .cast_ray([2.0, 0.0, 3.0], [0.0, 0.0, -1.0], 10.0, Some(body))
            .unwrap()
            .unwrap(),
        2.0,
    );
    step(&mut backend, 1500);
    assert!(backend.state(body).unwrap().position[2] < 0.0);
    backend.reset().unwrap();
    backend.set_self_collision_group(body, None).unwrap();
    step(&mut backend, 1500);
    assert!((backend.state(body).unwrap().position[2] - 1.25).abs() < 0.01);
    close(backend.state(body).unwrap().velocity[0], 0.0);
    let next = backend.spawn(ball([40.0, 0.0, 3.0])).unwrap();
    assert!(next > runtime);
    let foreign = RapierBackend::new([0.0; 3]).unwrap();
    let state = backend.state(body);
    assert!(backend.prepare_initial(&foreign.initial_state()).is_err());
    let foreign_prepared = foreign.prepare_initial(&foreign.initial_state()).unwrap();
    assert!(backend.apply_initial(foreign_prepared).is_err());
    assert_eq!(backend.state(body), state);
}

fn rejected(
    collisions: &[CollisionSpec],
    position: [f64; 3],
    velocity: [f64; 3],
    mass: Option<f64>,
    rotation: RotationSpec,
    inertia: Option<Inertia>,
    expected: PhysicsError,
) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(ball([-20.0, 0.0, 0.0])).unwrap();
    assert_eq!(control.spawn(ball([-20.0, 0.0, 0.0])).unwrap(), id);
    for world in [&mut backend, &mut control] {
        world
            .add_world_wrench(id, [2.0, 4.0, 6.0], [0.0; 3])
            .unwrap();
    }
    let before = backend.state(id);
    assert_eq!(
        backend.spawn_compound_link(
            CompoundBodySpec {
                collisions,
                position,
                velocity,
                mass
            },
            rotation,
            inertia
        ),
        Err(expected)
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
    step(&mut backend, 1);
    step(&mut control, 1);
    same_state(backend.state(id).unwrap(), control.state(id).unwrap());
}

#[test]
fn compound_two_and_sixteen_members_are_owned_while_bad_counts_and_last_child_are_atomic() {
    let children = [child(Shape::Sphere { radius: 0.5 }, [0.0; 3]); 17];
    for count in [2, 16] {
        for mass in [None, Some(2.0)] {
            let mut backend = RapierBackend::new([0.0; 3]).unwrap();
            let mut control = RapierBackend::new([0.0; 3]).unwrap();
            let mut owned = children[..count].to_vec();
            let id = backend
                .spawn_compound_link(
                    CompoundBodySpec {
                        mass,
                        ..spec(&owned)
                    },
                    RotationSpec::default(),
                    Some(inertia()),
                )
                .unwrap();
            let single = control
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.5 },
                        mass,
                        ..ball([0.0; 3])
                    },
                    RotationSpec::default(),
                    Some(inertia()),
                )
                .unwrap();
            assert_eq!(backend.state(id), control.state(single));
            owned.fill(child(Shape::Sphere { radius: f64::NAN }, [f64::NAN; 3]));
            distance(&backend, [0.0, 0.0, 4.0], [0.0, 0.0, -1.0], 3.5);
            backend.reset().unwrap();
            distance(&backend, [0.0, 0.0, 4.0], [0.0, 0.0, -1.0], 3.5);
            assert_eq!(id, BodyId::from_raw(0));
            assert_eq!(
                backend.spawn(ball([20.0, 0.0, 0.0])).unwrap(),
                BodyId::from_raw(1)
            );
            backend
                .apply_impulse(id, [1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
                .unwrap();
            control
                .apply_impulse(single, [1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
                .unwrap();
            step(&mut backend, 10);
            step(&mut control, 10);
            same_state(backend.state(id).unwrap(), control.state(single).unwrap());
        }
    }
    for count in [0, 1, 17] {
        rejected(
            &children[..count],
            [0.0; 3],
            [0.0; 3],
            None,
            RotationSpec::default(),
            None,
            PhysicsError::InvalidValue,
        );
    }
    let mut bad = children[..16].to_vec();
    bad[15].shape = Shape::Capsule {
        radius: 0.5,
        half_length: 0.0,
    };
    rejected(
        &bad,
        [0.0; 3],
        [0.0; 3],
        None,
        RotationSpec::default(),
        None,
        PhysicsError::InvalidValue,
    );
}

#[test]
fn compound_numeric_and_centered_inertia_refusals_preserve_commands_and_identifiers() {
    let original = mixed();
    for invalid in [
        f64::NAN,
        f64::INFINITY,
        0.0,
        -1.0,
        f64::MIN_POSITIVE,
        f64::MAX,
    ] {
        let mut children = original;
        children[3].shape = Shape::Sphere { radius: invalid };
        rejected(
            &children,
            [0.0; 3],
            [0.0; 3],
            Some(2.0),
            RotationSpec::default(),
            Some(inertia()),
            PhysicsError::InvalidValue,
        );
    }
    for fault in 0..5 {
        let mut children = original;
        match fault {
            0 => children[3].position = [f64::INFINITY; 3],
            1 => children[3].position = [f64::MAX; 3],
            2 => children[3].orientation = [0.0; 4],
            3 => children[3].orientation = [0.0, 0.0, 0.1, 1.0],
            _ => children[3].orientation = [f64::NAN; 4],
        }
        rejected(
            &children,
            [0.0; 3],
            [0.0; 3],
            Some(2.0),
            RotationSpec::default(),
            Some(inertia()),
            PhysicsError::InvalidValue,
        );
    }
    // Every child has representable dimensions and bounds; the union volume
    // overflows the BVH cost arithmetic before any native body is inserted.
    let enormous = [
        child(
            Shape::Box {
                half_extents: [1e95, 1e-20, 1e-20],
            },
            [1e110, 0.0, 0.0],
        ),
        child(
            Shape::Box {
                half_extents: [1e-20, 1e95, 1e-20],
            },
            [0.0, 1e110, 0.0],
        ),
        child(
            Shape::Box {
                half_extents: [1e-20, 1e-20, 1e95],
            },
            [0.0, 0.0, 1e110],
        ),
    ];
    rejected(
        &enormous,
        [0.0; 3],
        [0.0; 3],
        None,
        RotationSpec::default(),
        None,
        PhysicsError::InvalidValue,
    );
    // Finite union volume alone does not make the maintained shifted-tensor
    // eigen arithmetic representable. These slender children keep valid local moments.
    let separated = [-1e70, 1e70].map(|x| {
        child(
            Shape::Box {
                half_extents: [1e55, 1e-10, 1e-10],
            },
            [x, 0.0, 0.0],
        )
    });
    rejected(
        &separated,
        [0.0; 3],
        [0.0; 3],
        None,
        RotationSpec::default(),
        None,
        PhysicsError::InvalidValue,
    );
    for (position, velocity, mass, rotation) in [
        ([f64::NAN; 3], [0.0; 3], Some(2.0), RotationSpec::default()),
        ([f64::MAX; 3], [0.0; 3], Some(2.0), RotationSpec::default()),
        ([0.0; 3], [f64::NAN; 3], Some(2.0), RotationSpec::default()),
        ([0.0; 3], [0.0; 3], Some(0.0), RotationSpec::default()),
        (
            [0.0; 3],
            [0.0; 3],
            Some(f64::INFINITY),
            RotationSpec::default(),
        ),
        ([0.0; 3], [1.0, 0.0, 0.0], None, RotationSpec::default()),
        (
            [0.0; 3],
            [0.0; 3],
            None,
            RotationSpec {
                angular_velocity: [0.0, 0.0, 1.0],
                ..RotationSpec::default()
            },
        ),
        (
            [0.0; 3],
            [0.0; 3],
            Some(2.0),
            RotationSpec {
                orientation: [0.0; 4],
                ..RotationSpec::default()
            },
        ),
        (
            [0.0; 3],
            [0.0; 3],
            Some(2.0),
            RotationSpec {
                angular_velocity: [f64::NAN; 3],
                ..RotationSpec::default()
            },
        ),
    ] {
        rejected(
            &original,
            position,
            velocity,
            mass,
            rotation,
            Some(inertia()),
            PhysicsError::InvalidValue,
        );
    }
    rejected(
        &original,
        [0.0; 3],
        [0.0; 3],
        Some(2.0),
        RotationSpec::default(),
        None,
        PhysicsError::Unsupported {
            engine: "rapier",
            capability: "dynamic primitive compound without explicit inertia",
        },
    );
    for mass in [None, Some(2.0)] {
        for value in [
            Inertia {
                center: [0.1, 0.0, 0.0],
                ..inertia()
            },
            Inertia {
                center: [-0.1, 0.0, 0.0],
                ..inertia()
            },
        ] {
            // Preserve each exact former refusal input. The matched centered
            // body starts at the same world COM; its measured finite-step
            // trajectory is independent of the shifted body's origin frame.
            let mut offset = RapierBackend::new([0.0; 3]).unwrap();
            let mut centered = RapierBackend::new([0.0; 3]).unwrap();
            let id = offset
                .spawn_compound_link(
                    CompoundBodySpec {
                        mass,
                        ..spec(&original)
                    },
                    RotationSpec::default(),
                    Some(value),
                )
                .unwrap();
            let twin = centered
                .spawn_compound_link(
                    CompoundBodySpec {
                        position: value.center,
                        mass,
                        ..spec(&original)
                    },
                    RotationSpec::default(),
                    Some(inertia()),
                )
                .unwrap();
            assert_eq!(id, twin);
            let initial = offset.state(id).unwrap();
            assert_eq!(initial.position, [0.0; 3]);
            assert_eq!(initial.velocity, [0.0; 3]);
            assert_eq!(initial.angular_velocity, [0.0; 3]);
            assert_eq!(initial.orientation, IDENTITY);
            distance(&offset, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.5);
            if mass.is_some() {
                offset
                    .apply_impulse(id, [2.0, 0.0, 0.0], [0.0, 0.0, 4.0])
                    .unwrap();
                centered
                    .apply_impulse(twin, [2.0, 0.0, 0.0], [0.0, 0.0, 4.0])
                    .unwrap();
            }
            step(&mut offset, 7);
            step(&mut centered, 7);
            let actual = offset.state(id).unwrap();
            let measured = centered.state(twin).unwrap();
            if mass.is_some() {
                close(measured.velocity[0], 1.0);
                close(measured.angular_velocity[2], 1.0);
                let [x, y, z, scalar] = measured.orientation;
                let c = value.center[0];
                let arm = [
                    c * (1.0 - 2.0 * (y * y + z * z)),
                    c * 2.0 * (x * y + scalar * z),
                    c * 2.0 * (x * z - scalar * y),
                ];
                let omega = measured.angular_velocity;
                for axis in 0..3 {
                    close(actual.position[axis], measured.position[axis] - arm[axis]);
                    // Native velocity is COM velocity; only the reported
                    // origin position differs between these matched bodies.
                    close(actual.velocity[axis], measured.velocity[axis]);
                    close(actual.angular_velocity[axis], omega[axis]);
                }
                for (actual, expected) in actual.orientation.into_iter().zip(measured.orientation) {
                    close(actual, expected);
                }
            } else {
                same_state(actual, initial);
                distance(&offset, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.5);
            }
            offset.reset().unwrap();
            assert_eq!(offset.state(id).unwrap(), initial);
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
                tensor: [0.0; 6],
                ..inertia()
            },
            Inertia {
                tensor: [10.0, 1.0, 1.0, 0.0, 0.0, 0.0],
                ..inertia()
            },
        ] {
            rejected(
                &original,
                [0.0; 3],
                [0.0; 3],
                mass,
                RotationSpec::default(),
                Some(value),
                PhysicsError::InvalidValue,
            );
        }
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        assert!(
            backend
                .spawn_compound_link(
                    CompoundBodySpec {
                        mass,
                        ..spec(&original)
                    },
                    RotationSpec::default(),
                    Some(inertia())
                )
                .is_ok()
        );
    }
}
