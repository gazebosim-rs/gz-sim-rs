// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Every C++ exception the bridge can raise returns as a typed error.
use super::*;

fn sphere() -> BodyConfig {
    BodyConfig {
        shape: 0,
        dimensions: [0.5, 0.0, 0.0],
        position: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
        mobile: true,
        mass: 1.0,
        center: [0.0; 3],
        moment: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
        principal: [0.1; 3],
        axes: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        velocity: Twist {
            linear: [0.0; 3],
            angular: [0.0; 3],
        },
    }
}

fn message<T>(result: Result<T, PhysicsError>) -> String {
    match result {
        Err(PhysicsError::Native { engine, message }) => {
            assert_eq!(engine, crate::ENGINE);
            message
        }
        Err(other) => panic!("unexpected {other:?}"),
        Ok(_) => panic!("native call unexpectedly succeeded"),
    }
}

#[test]
fn native_exceptions_become_native_errors() {
    let mut world = NativeWorld::new([0.0; 3]).unwrap();
    let twist = Twist {
        linear: [0.0; 3],
        angular: [0.0, 0.0, 1.0],
    };
    let unknown = message(world.add_body(&BodyConfig {
        shape: 9,
        ..sphere()
    }));
    assert!(unknown.contains("shape code 9"), "{unknown}");
    // The rejected body leaves no skeleton behind in the DART world.
    assert_eq!(world.skeleton_count().unwrap(), 0);
    assert!(!message(world.read(0)).is_empty());
    let slot = world.add_body(&sphere()).unwrap();
    assert!(message(world.remove_revolute(slot)).contains("no world revolute"));
    world.add_revolute(slot, &twist).unwrap();
    assert!(message(world.add_revolute(slot, &twist)).contains("already has a parent joint"));
    assert!(world.remove_revolute(slot).unwrap());
    world.remove_body(slot).unwrap();
    for error in [
        message(world.remove_body(slot)),
        message(world.set_velocity(slot, &twist)),
        message(world.apply_impulse(slot, &twist)),
        message(world.step(
            0.001,
            &[Wrench {
                slot,
                force: [1.0; 3],
                torque: [0.0; 3],
            }],
        )),
        message(world.add_revolute(slot, &twist)),
        message(world.read(slot)),
    ] {
        assert!(error.contains("removed body slot"), "{error}");
    }
    assert_eq!(world.step(0.001, &[]).unwrap(), 0);
    assert_eq!(world.skeleton_count().unwrap(), 0);
}

#[test]
fn std_and_foreign_exceptions_both_return_as_errors() {
    assert!(crate::ffi::bridge::exception_probe(0).is_ok());
    let error = crate::ffi::bridge::exception_probe(1).unwrap_err();
    assert_eq!(error.what(), "probe std::exception");
    // cxx's default handler would std::terminate on a non-std exception.
    let error = crate::ffi::bridge::exception_probe(2).unwrap_err();
    assert_eq!(error.what(), "unknown C++ exception");
    assert_eq!(
        message::<()>(Err(crate::ffi::native(error))),
        "unknown C++ exception"
    );
}

#[test]
fn freeing_a_hinged_body_into_deep_contact_is_refused_unchanged() {
    use gz_physics::{BodySpec, Inertia, RotationSpec, Shape};
    let mut world = NativeWorld::new([0.0; 3]).unwrap();
    let block = crate::body::config(
        BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 2.0],
            },
            position: [0.0, 0.0, -2.0],
            velocity: [0.0; 3],
            mass: None,
        },
        RotationSpec::default(),
        None,
    )
    .unwrap();
    world.add_body(&block).unwrap();
    // A light cube buried 1.5 m deep, as a hinge can hold a body against
    // contact; its records bypass the backend's spawn-time contact check.
    let cube = crate::body::config(
        BodySpec {
            shape: Shape::Box {
                half_extents: [0.05; 3],
            },
            position: [0.0, 0.0, -1.5],
            velocity: [0.0; 3],
            mass: Some(2e-5),
        },
        RotationSpec::default(),
        Some(Inertia {
            center: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            tensor: [1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0],
        }),
    )
    .unwrap();
    let slot = world.add_body(&cube).unwrap();
    let hinge = Twist {
        linear: [10.0, 0.0, -1.5],
        angular: [0.0, 1.0, 0.0],
    };
    assert!(world.add_revolute(slot, &hinge).unwrap());
    // Hinged 10 m away, a unit impulse at the buried contacts meets about
    // |r|^2/I_axis = 100/2e-3; freed, 1/m + |r|^2/lambda exceeds 4e5.
    assert!(world.contacts_conditioned().unwrap());
    world
        .set_velocity(
            slot,
            &Twist {
                linear: [0.0, 0.0, 0.1],
                angular: [0.0, 0.01, 0.0],
            },
        )
        .unwrap();
    let before = world.read(slot).unwrap();
    assert!(!world.remove_revolute(slot).unwrap());
    let after = world.read(slot).unwrap();
    assert_eq!(after.position, before.position);
    assert_eq!(after.orientation, before.orientation);
    assert_eq!(after.velocity, before.velocity);
    assert_eq!(after.joint_position, before.joint_position);
    assert_eq!(after.joint_velocity, before.joint_velocity);
    assert!(world.contacts_conditioned().unwrap());
    assert_eq!(world.step(0.001, &[]).unwrap(), 0);
}

#[test]
fn native_body_joint_preflight_and_retirement_preserve_skeleton_ownership() {
    let mut world = NativeWorld::new([0.0; 3]).unwrap();
    let parent = world.add_body(&sphere()).unwrap();
    let child = world
        .add_body(&BodyConfig {
            position: [3.0, 0.0, 0.0],
            ..sphere()
        })
        .unwrap();
    let axis = Twist {
        linear: [0.0; 3],
        angular: [0.0, 0.0, 1.0],
    };
    let initial = [world.read(parent).unwrap(), world.read(child).unwrap()];
    assert!(world.add_body_revolute(parent, child, &axis).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 1);
    for (slot, before) in [parent, child].into_iter().zip(initial) {
        let state = world.read(slot).unwrap();
        assert_eq!(state.position, before.position);
        assert_eq!(state.velocity, before.velocity);
    }
    assert!(world.remove_revolute(child).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 2);
    assert!(world.add_revolute(parent, &axis).unwrap());
    assert!(world.add_body_revolute(parent, child, &axis).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 1);
    assert!(
        message(world.add_body_revolute(child, parent, &axis))
            .contains("unsupported body revolute topology")
    );
    let before = world.read(parent).unwrap();
    assert!(!world.remove_revolute(parent).unwrap());
    let after = world.read(parent).unwrap();
    assert_eq!(after.position, before.position);
    assert_eq!(after.velocity, before.velocity);
    assert_eq!(world.skeleton_count().unwrap(), 1);
    assert!(world.remove_revolute(child).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 2);
    assert!(world.remove_revolute(parent).unwrap());
    world.remove_body(parent).unwrap();
    world.remove_body(child).unwrap();
    assert_eq!(world.skeleton_count().unwrap(), 0);
}

#[test]
fn prepared_native_free_pair_exact_old_input_preserves_ownership_on_retirement() {
    let mut world = NativeWorld::new([0.0; 3]).unwrap();
    let parent = world.add_body(&sphere()).unwrap();
    let child = world
        .add_body(&BodyConfig {
            position: [3.0, 0.0, 0.0],
            ..sphere()
        })
        .unwrap();
    let axis = Twist {
        linear: [0.0; 3],
        angular: [0.0, 0.0, 1.0],
    };
    let before = [world.read(parent).unwrap(), world.read(child).unwrap()];
    assert!(world.add_body_revolute(parent, child, &axis).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 1);
    for (slot, initial) in [parent, child].into_iter().zip(before) {
        let state = world.read(slot).unwrap();
        assert_eq!(state.position, initial.position);
        assert_eq!(state.velocity, initial.velocity);
    }
    assert!(world.remove_revolute(child).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 2);
    assert!(world.add_revolute(parent, &axis).unwrap());
    assert!(world.add_body_revolute(parent, child, &axis).unwrap());
    assert_eq!(world.skeleton_count().unwrap(), 1);
    assert!(!world.remove_revolute(parent).unwrap());
    assert!(world.remove_revolute(child).unwrap());
    assert!(world.remove_revolute(parent).unwrap());
    world.remove_body(parent).unwrap();
    world.remove_body(child).unwrap();
    assert_eq!(world.skeleton_count().unwrap(), 0);
}
