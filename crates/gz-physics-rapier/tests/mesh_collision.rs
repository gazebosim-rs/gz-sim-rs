// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, MeshBodySpec, PhysicsError,
    RapierBackend, RotationSpec, Shape,
};
use std::time::Duration;

const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
const CUBE: [[f64; 3]; 8] = [
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.5, 0.5, -0.5],
    [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5],
    [0.5, 0.5, 0.5],
    [-0.5, 0.5, 0.5],
];
const FACES: [[u32; 3]; 12] = [
    [0, 2, 1],
    [0, 3, 2],
    [4, 5, 6],
    [4, 6, 7],
    [0, 1, 5],
    [0, 5, 4],
    [3, 7, 6],
    [3, 6, 2],
    [0, 4, 7],
    [0, 7, 3],
    [1, 2, 6],
    [1, 6, 5],
];
const SURFACE: [[f64; 3]; 4] = [
    [-2.0, -2.0, 0.0],
    [2.0, -2.0, 0.0],
    [2.0, 2.0, 0.0],
    [-2.0, 2.0, 0.0],
];
const SURFACE_FACES: [[u32; 3]; 2] = [[0, 1, 2], [0, 2, 3]];

fn mesh<'a>(vertices: &'a [[f64; 3]], triangles: &'a [[u32; 3]]) -> MeshBodySpec<'a> {
    MeshBodySpec {
        vertices,
        triangles,
        collision_position: [0.0; 3],
        collision_orientation: IDENTITY,
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
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

fn transformed_floor(
    backend: &mut RapierBackend,
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> BodyId {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    backend
        .spawn_mesh_link(
            MeshBodySpec {
                position: [3.0, 4.0, 5.0],
                collision_position: [0.0, 2.0, 1.0],
                collision_orientation: [s, 0.0, 0.0, s],
                mass: None,
                ..mesh(vertices, triangles)
            },
            RotationSpec {
                orientation: [-s, 0.0, 0.0, s],
                ..RotationSpec::default()
            },
            None,
        )
        .unwrap()
}

#[test]
fn triangle_surface_contacts_use_both_link_and_local_collision_frames() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let floor = transformed_floor(&mut backend, &SURFACE, &SURFACE_FACES);
    let floor_state = backend.state(floor).unwrap();
    assert_eq!(backend.is_dynamic(floor), Some(false));
    let sphere = backend.spawn(ball([3.0, 5.0, 5.0])).unwrap();
    step(&mut backend, 1500);
    let landed = backend.state(sphere).unwrap();
    assert!((landed.position[2] - 3.25).abs() < 0.01, "{landed:?}");
    assert!(landed.velocity[2].abs() < 0.01, "{landed:?}");
    assert_eq!(backend.state(floor), Some(floor_state));
    backend.reset().unwrap();
    assert_eq!(backend.state(sphere).unwrap().position, [3.0, 5.0, 5.0]);
    assert_eq!(backend.state(floor), Some(floor_state));
    // Removing the finite mesh surface releases the otherwise unchanged primitive.
    assert!(backend.remove(floor));
    assert!(!backend.remove(floor));
    backend.reset().unwrap();
    assert_eq!(backend.state(floor), None);
    step(&mut backend, 1500);
    assert!(backend.state(sphere).unwrap().position[2] < 0.0);
}

#[test]
fn dynamic_convex_mesh_uses_explicit_inertia_and_world_twist_like_a_primitive() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rotation = RotationSpec {
        orientation: [0.0, 0.0, s, s],
        angular_velocity: [0.1, 0.2, 0.3],
    };
    let inertia = Inertia {
        center: [0.25, -0.125, 0.5],
        orientation: [s, 0.0, 0.0, s],
        tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
    };
    let spec = MeshBodySpec {
        velocity: [0.5, -0.25, 0.75],
        collision_position: [0.75, 0.0, 0.0],
        collision_orientation: [0.0, s, 0.0, s],
        ..mesh(&CUBE, &FACES)
    };
    let id = backend
        .spawn_mesh_link(spec, rotation, Some(inertia))
        .unwrap();
    // Separate worlds avoid contacts between the two construction controls.
    let mut primitive = RapierBackend::new([0.0; 3]).unwrap();
    let control = primitive
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.5; 3],
                },
                position: spec.position,
                velocity: spec.velocity,
                mass: spec.mass,
            },
            rotation,
            Some(inertia),
        )
        .unwrap();
    let initial = backend.state(id).unwrap();
    assert_eq!(initial.position, spec.position);
    assert_eq!(initial.orientation, rotation.orientation);
    assert_eq!(initial.velocity, spec.velocity);
    assert_eq!(initial.angular_velocity, rotation.angular_velocity);
    assert_eq!(backend.is_dynamic(id), Some(true));
    for (world, body) in [(&mut backend, id), (&mut primitive, control)] {
        world
            .apply_impulse(body, [2.0, 4.0, -2.0], [4.0, 2.0, 3.0])
            .unwrap();
        let state = world.state(body).unwrap();
        for (actual, expected) in state.velocity.into_iter().zip([1.5, 1.75, -0.25]) {
            close(actual, expected);
        }
        // Body Rz(π/2) * inertial Rx(π/2) maps the diagonal to world [4,2,3].
        for (actual, expected) in state.angular_velocity.into_iter().zip([1.1, 1.2, 1.3]) {
            close(actual, expected);
        }
        world
            .add_world_wrench(body, [0.5, -1.0, 2.0], [0.1, 0.2, -0.3])
            .unwrap();
    }
    for _ in 0..100 {
        step(&mut backend, 1);
        step(&mut primitive, 1);
        same_state(
            backend.state(id).unwrap(),
            primitive.state(control).unwrap(),
        );
    }
    backend.reset().unwrap();
    primitive.reset().unwrap();
    same_state(backend.state(id).unwrap(), initial);
    same_state(
        backend.state(id).unwrap(),
        primitive.state(control).unwrap(),
    );
}

#[test]
fn convex_contact_support_retains_collision_translation_and_orientation_through_reset() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ground = backend
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let vertices = CUBE.map(|[x, y, z]| [x * 2.0, y * 4.0, z * 6.0]);
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let id = backend
        .spawn_mesh_link(
            MeshBodySpec {
                position: [0.0, 0.0, 4.0],
                collision_position: [0.0, 2.0, 0.0],
                collision_orientation: [0.0, 0.0, s, s],
                ..mesh(&vertices, &FACES)
            },
            RotationSpec {
                orientation: [s, 0.0, 0.0, s],
                ..RotationSpec::default()
            },
            None,
        )
        .unwrap();
    let initial = backend.state(id).unwrap();
    let fixed = backend.state(ground);
    for _ in 0..2 {
        step(&mut backend, 2000);
        let landed = backend.state(id).unwrap();
        // Rbody * Rcollision maps the local X half-extent to world Z (one metre).
        // Rbody maps the two-metre collision offset to +Z: the body origin is -1.
        assert!((landed.position[2] + 1.0).abs() < 0.03, "{landed:?}");
        assert!(landed.velocity[2].abs() < 0.02, "{landed:?}");
        assert_eq!(backend.state(ground), fixed);
        backend.reset().unwrap();
        assert_eq!(backend.state(id), Some(initial));
    }
}

#[test]
fn reset_and_captured_restore_own_meshes_after_source_mutation_and_drop() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let (floor, body, captured, floor_state, body_state) = {
        let mut vertices = SURFACE.to_vec();
        let mut faces = SURFACE_FACES.to_vec();
        let floor = transformed_floor(&mut backend, &vertices, &faces);
        let mut solid = CUBE.to_vec();
        let mut solid_faces = FACES.to_vec();
        let body = backend
            .spawn_mesh_link(
                MeshBodySpec {
                    position: [3.0, 5.0, 6.0],
                    ..mesh(&solid, &solid_faces)
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let captured = backend.initial_state();
        let floor_state = backend.state(floor).unwrap();
        let body_state = backend.state(body).unwrap();
        vertices.fill([f64::NAN; 3]);
        faces.fill([u32::MAX; 3]);
        solid.fill([1e6; 3]);
        solid_faces.clear();
        (floor, body, captured, floor_state, body_state)
    };
    for _ in 0..2 {
        step(&mut backend, 2000);
        let landed = backend.state(body).unwrap();
        assert!((landed.position[2] - 3.5).abs() < 0.02, "{landed:?}");
        assert!(landed.velocity[2].abs() < 0.02, "{landed:?}");
        backend.reset().unwrap();
        assert_eq!(backend.state(body), Some(body_state));
        assert_eq!(backend.state(floor), Some(floor_state));
    }
    assert!(backend.remove(floor));
    assert!(backend.remove(body));
    let runtime = backend.spawn(ball([20.0, 0.0, 0.0])).unwrap();
    let runtime_state = backend.state(runtime);
    let prepared = backend.prepare_initial(&captured).unwrap();
    assert_eq!(backend.state(floor), None);
    assert_eq!(backend.state(body), None);
    assert_eq!(backend.state(runtime), runtime_state);
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(runtime), None);
    assert_eq!(backend.state(body), Some(body_state));
    assert_eq!(backend.state(floor), Some(floor_state));
    assert!(backend.spawn(ball([20.0, 0.0, 0.0])).unwrap() > runtime);
    step(&mut backend, 2000);
    let landed = backend.state(body).unwrap();
    assert!((landed.position[2] - 3.5).abs() < 0.02, "{landed:?}");
    assert!(landed.velocity[2].abs() < 0.02, "{landed:?}");
}

fn rejected_spawn_is_atomic(
    spec: MeshBodySpec<'_>,
    rotation: RotationSpec,
    inertia: Option<Inertia>,
    diagnostic: Option<&str>,
) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let seed = ball([-20.0, 0.0, 0.0]);
    let id = backend.spawn(seed).unwrap();
    assert_eq!(control.spawn(seed).unwrap(), id);
    for world in [&mut backend, &mut control] {
        world
            .add_world_wrench(id, [2.0, 4.0, 6.0], [0.1, 0.2, 0.3])
            .unwrap();
    }
    let before = backend.state(id);
    let error = backend
        .spawn_mesh_link(spec, rotation, inertia)
        .unwrap_err();
    if let Some(diagnostic) = diagnostic {
        match error {
            PhysicsError::Native { engine, message } => {
                assert_eq!(engine, "rapier");
                assert!(message.contains(diagnostic), "{message}");
            }
            other => panic!("expected Rapier mesh diagnostic, got {other:?}"),
        }
    } else {
        assert_eq!(error, PhysicsError::InvalidValue);
    }
    assert_eq!(backend.state(id), before);
    assert_eq!(backend.state(BodyId::from_raw(1)), None);
    // Failure consumes neither the public ID nor a pending one-step wrench.
    let next = backend
        .spawn_mesh_link(mesh(&CUBE, &FACES), RotationSpec::default(), None)
        .unwrap();
    assert_eq!(next, BodyId::from_raw(1));
    assert_eq!(
        control
            .spawn_mesh_link(mesh(&CUBE, &FACES), RotationSpec::default(), None)
            .unwrap(),
        next
    );
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
    let after_force = backend.state(id).unwrap().velocity;
    step(&mut backend, 1);
    step(&mut control, 1);
    same_state(backend.state(id).unwrap(), control.state(id).unwrap());
    for (actual, expected) in backend
        .state(id)
        .unwrap()
        .velocity
        .into_iter()
        .zip(after_force)
    {
        close(actual, expected);
    }
}

#[test]
fn invalid_mesh_state_frames_and_inertia_preserve_ids_state_and_pending_work() {
    let base = mesh(&CUBE, &FACES);
    for spec in [
        MeshBodySpec {
            position: [f64::NAN; 3],
            ..base
        },
        MeshBodySpec {
            velocity: [f64::INFINITY; 3],
            ..base
        },
        MeshBodySpec {
            collision_position: [f64::NAN; 3],
            ..base
        },
        MeshBodySpec {
            collision_orientation: [0.0; 4],
            ..base
        },
        MeshBodySpec {
            position: [f64::MAX, 0.0, 0.0],
            collision_position: [f64::MAX, 0.0, 0.0],
            ..base
        },
        MeshBodySpec {
            mass: None,
            velocity: [1.0, 0.0, 0.0],
            ..base
        },
    ] {
        rejected_spawn_is_atomic(spec, RotationSpec::default(), None, None);
    }
    for mass in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::MAX,
        f64::MIN_POSITIVE,
        f64::from_bits(1),
    ] {
        rejected_spawn_is_atomic(
            MeshBodySpec {
                mass: Some(mass),
                ..base
            },
            RotationSpec::default(),
            None,
            None,
        );
    }
    for rotation in [
        RotationSpec {
            orientation: [0.0; 4],
            ..RotationSpec::default()
        },
        RotationSpec {
            angular_velocity: [f64::NAN; 3],
            ..RotationSpec::default()
        },
    ] {
        rejected_spawn_is_atomic(base, rotation, None, None);
    }
    rejected_spawn_is_atomic(
        MeshBodySpec { mass: None, ..base },
        RotationSpec {
            angular_velocity: [0.0, 1.0, 0.0],
            ..RotationSpec::default()
        },
        None,
        None,
    );
    let valid = Inertia {
        center: [0.0; 3],
        orientation: IDENTITY,
        tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
    };
    for inertia in [
        Inertia {
            center: [f64::NAN; 3],
            ..valid
        },
        Inertia {
            orientation: [0.0; 4],
            ..valid
        },
        Inertia {
            tensor: [f64::MAX; 6],
            ..valid
        },
        Inertia {
            tensor: [-1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            ..valid
        },
        Inertia {
            tensor: [10.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            ..valid
        },
    ] {
        rejected_spawn_is_atomic(base, RotationSpec::default(), Some(inertia), None);
    }
}

#[test]
fn malformed_or_nonconvex_meshes_refuse_without_allocating_or_consuming_work() {
    let rotation = RotationSpec::default();
    rejected_spawn_is_atomic(
        mesh(&[], &FACES),
        rotation,
        None,
        Some("vertex/triangle count"),
    );
    rejected_spawn_is_atomic(
        mesh(&CUBE, &[]),
        rotation,
        None,
        Some("vertex/triangle count"),
    );
    let mut vertices = CUBE;
    vertices[0][0] = f64::NAN;
    rejected_spawn_is_atomic(
        mesh(&vertices, &FACES),
        rotation,
        None,
        Some("vertices must be finite"),
    );
    let mut faces = FACES;
    faces[0][0] = 8;
    rejected_spawn_is_atomic(
        mesh(&CUBE, &faces),
        rotation,
        None,
        Some("index is outside"),
    );
    faces = FACES;
    faces[0] = [0, 0, 1];
    rejected_spawn_is_atomic(mesh(&CUBE, &faces), rotation, None, Some("nonzero area"));
    rejected_spawn_is_atomic(
        mesh(&CUBE, &FACES[..11]),
        rotation,
        None,
        Some("must be closed"),
    );
    // Open surfaces remain valid statically, but never become dynamic solids by a hull repair.
    rejected_spawn_is_atomic(
        mesh(&SURFACE, &SURFACE_FACES),
        rotation,
        None,
        Some("must be closed"),
    );
    faces = FACES;
    faces[0].swap(0, 1);
    rejected_spawn_is_atomic(
        mesh(&CUBE, &faces),
        rotation,
        None,
        Some("opposite face winding"),
    );
    let mut duplicate = FACES.to_vec();
    duplicate.push(FACES[0]);
    rejected_spawn_is_atomic(
        mesh(&CUBE, &duplicate),
        rotation,
        None,
        Some("duplicate face"),
    );
    vertices = CUBE;
    vertices[6] = [0.0; 3];
    rejected_spawn_is_atomic(
        mesh(&vertices, &FACES),
        rotation,
        None,
        Some("convex and outward-oriented"),
    );
    let inward = FACES.map(|[a, b, c]| [a, c, b]);
    rejected_spawn_is_atomic(
        mesh(&CUBE, &inward),
        rotation,
        None,
        Some("convex and outward-oriented"),
    );
    let mut unused = CUBE.to_vec();
    unused.push([0.0; 3]);
    rejected_spawn_is_atomic(mesh(&unused, &FACES), rotation, None, Some("unused vertex"));
}
