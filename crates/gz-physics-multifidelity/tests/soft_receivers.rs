// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;

#[test]
fn swept_corrections_are_validated_for_every_free_sphere_before_the_batch_is_applied() {
    let mut backend = MultiFidelityBackend::new([0.; 3]).unwrap();
    let sphere = |x| BodySpec {
        position: [x, 0., 0.],
        velocity: [0.; 3],
        mass: Some(2.),
        shape: Shape::Sphere { radius: 0.5 },
    };
    let a = backend
        .spawn_link(sphere(-4.), Default::default(), None)
        .unwrap();
    let b = backend
        .spawn_link(sphere(0.), Default::default(), None)
        .unwrap();
    let (w, i) = backend.soft_sphere_mobility(a).unwrap();
    assert!((w - 0.5).abs() < 1e-12 && (i - 5.).abs() < 1e-12);
    let wall = backend
        .spawn_link(
            BodySpec {
                position: [2., 0., 0.],
                mass: None,
                shape: Shape::Box {
                    half_extents: [0.01, 0.6, 0.6],
                },
                ..sphere(0.)
            },
            Default::default(),
            None,
        )
        .unwrap();
    assert!(backend.soft_sphere_mobility(wall).is_err());
    let before_a = backend.state(a).unwrap();
    let before_b = backend.state(b).unwrap();
    let mut updated_a = before_a;
    let mut updated_b = before_b;
    updated_a.position[0] = -3.;
    updated_b.position[0] = 3.;
    assert!(
        backend
            .commit_soft_spheres(&[(a, updated_a), (b, updated_b)])
            .is_err()
    );
    assert_eq!(backend.state(a), Some(before_a));
    assert_eq!(backend.state(b), Some(before_b));
    backend.remove(wall);
    backend
        .commit_soft_spheres(&[(a, updated_a), (b, updated_b)])
        .unwrap();
    assert_eq!(backend.state(a), Some(updated_a));
    assert_eq!(backend.state(b), Some(updated_b));
    assert!(
        backend
            .commit_soft_spheres(&[(a, updated_b), (b, updated_a)])
            .is_err()
    );
    assert_eq!(backend.state(a), Some(updated_a));
    assert_eq!(backend.state(b), Some(updated_b));
}

#[test]
fn principal_mass_frame_and_third_body_refusal_are_preserved_for_tensor_receivers() {
    use gz_physics::{Inertia, RotationSpec};
    let mut backend = MultiFidelityBackend::new([0.; 3]).unwrap();
    let properties = Inertia {
        center: [0.; 3],
        orientation: [0., 0., (0.25f64).sin(), (0.25f64).cos()],
        tensor: [0.06, 0.09, 0.11, 0., 0., 0.],
    };
    let spec = BodySpec {
        position: [0.; 3],
        velocity: [0.; 3],
        mass: Some(2.),
        shape: Shape::Sphere { radius: 0.5 },
    };
    let a = backend
        .spawn_link(spec, RotationSpec::default(), Some(properties))
        .unwrap();
    let b = backend
        .spawn_link(
            BodySpec {
                position: [2., 0., 0.],
                ..spec
            },
            RotationSpec::default(),
            Some(properties),
        )
        .unwrap();
    let mass = backend.soft_sphere_inertia(a).unwrap();
    let mut moments = mass.inverse_principal_inertia.map(f64::recip);
    moments.sort_by(f64::total_cmp);
    assert!((mass.inverse_mass - 0.5).abs() < 1e-14);
    for (measured, expected) in moments.into_iter().zip([0.06, 0.09, 0.11]) {
        assert!((measured - expected).abs() < 1e-12);
    }
    assert!(backend.soft_sphere_mobility(a).is_err());
    let old = [backend.state(a).unwrap(), backend.state(b).unwrap()];
    let mut new = old;
    new[0].angular_velocity = [0.8, -0.5, 0.1];
    new[1].position = [0.3, 0., 0.];
    assert!(
        backend
            .commit_soft_spheres(&[(a, new[0]), (b, new[1])])
            .is_err()
    );
    assert_eq!([backend.state(a).unwrap(), backend.state(b).unwrap()], old);
}

#[test]
fn cuboid_commit_revalidates_native_isolation_before_pose_and_twist_writes() {
    let mut backend = MultiFidelityBackend::new([0.; 3]).unwrap();
    let spec = BodySpec {
        shape: Shape::Box {
            half_extents: [0.5, 0.25, 0.15],
        },
        position: [0.; 3],
        velocity: [0.; 3],
        mass: Some(2.),
    };
    let id = backend.spawn_link(spec, Default::default(), None).unwrap();
    assert!(backend.soft_cuboid_inertia(id).is_ok() && backend.soft_sphere_inertia(id).is_err());
    let before = backend.state(id).unwrap();
    let mut next = before;
    next.position = [1., 2., 3.];
    next.angular_velocity = [0.2, 0.1, 0.3];
    let neighbor = backend
        .spawn_link(
            BodySpec {
                position: [20.; 3],
                ..spec
            },
            Default::default(),
            None,
        )
        .unwrap();
    assert!(backend.commit_soft_cuboid(id, next).is_err());
    assert_eq!(backend.state(id), Some(before));
    backend.remove(neighbor);
    backend.commit_soft_cuboid(id, next).unwrap();
    assert_eq!(backend.state(id), Some(next));
}
