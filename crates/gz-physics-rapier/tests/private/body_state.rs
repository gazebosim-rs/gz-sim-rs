// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn spec(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position,
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

#[test]
fn preflight_is_read_only_and_success_preserves_construction_and_native_pending_force() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn_link(
            spec([0.0; 3]),
            RotationSpec::default(),
            Some(Inertia {
                center: [0.25, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let other = backend.spawn(spec([10.0; 3])).unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    backend
        .add_world_wrench(id, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0])
        .unwrap();
    backend
        .add_world_wrench(other, [7.0, 8.0, 9.0], [1.0, 0.0, 0.0])
        .unwrap();
    let handle = backend.bodies[&id].0;
    let construction = backend.bodies[&id].1.clone();
    let old_rotation = backend.bodies[&id].2;
    let before = (backend.state(id), backend.state(other));
    let pending = backend.pending_wrenches.clone();
    let next = backend.next_id;
    let native_work = |b: &RapierBackend, id: BodyId| {
        let body = &b.world.bodies[b.bodies[&id].0];
        (body.user_force(), body.user_torque())
    };
    let work = (native_work(&backend, id), native_work(&backend, other));
    let requested = BodyState {
        position: [2.0, 3.0, 4.0],
        velocity: [1.0, 2.0, 3.0],
        orientation: [0.0, 0.0, 1.0, 0.0],
        angular_velocity: [4.0, 5.0, 6.0],
    };
    backend.validate_body_state(id, requested).unwrap();
    assert_eq!((backend.state(id), backend.state(other)), before);
    assert_eq!(
        (native_work(&backend, id), native_work(&backend, other)),
        work
    );
    backend.set_body_state(id, requested).unwrap();
    assert_eq!(backend.bodies[&id].0, handle);
    assert_eq!(backend.state(id), Some(requested));
    assert_eq!(backend.state(other), before.1);
    assert_eq!(backend.pending_wrenches, pending);
    assert_eq!(
        (native_work(&backend, id), native_work(&backend, other)),
        work
    );
    assert_eq!(backend.next_id, next);
    assert!(!backend.joints.construction_open());
    let entry = &backend.bodies[&id];
    assert_eq!(entry.1.position, construction.position);
    assert_eq!(entry.1.velocity, construction.velocity);
    assert_eq!(entry.1.mass, construction.mass);
    assert_eq!(
        entry.1.collider.position(),
        construction.collider.position()
    );
    assert_eq!(entry.2.orientation, old_rotation.orientation);
    assert_eq!(entry.2.angular_velocity, old_rotation.angular_velocity);
    assert_eq!(
        backend.world.bodies[handle]
            .mass_properties()
            .world_com
            .to_array(),
        [1.75, 3.0, 4.0]
    );
}

#[test]
fn rejected_jointed_edit_preserves_pending_generalized_and_world_work_through_next_step() {
    fn setup(reduced: bool) -> (RapierBackend, BodyId) {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let body = backend.spawn(spec([0.0; 3])).unwrap();
        let joint = if reduced {
            backend
                .add_reduced_revolute(None, body, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap()
        } else {
            backend
                .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap()
        };
        backend.step(Duration::from_millis(1)).unwrap();
        backend.add_joint_effort(joint, 0.5).unwrap();
        backend
            .add_world_wrench(body, [0.0; 3], [0.0, 0.25, 0.0])
            .unwrap();
        (backend, body)
    }
    for reduced in [false, true] {
        let (mut edited, body) = setup(reduced);
        let (mut control, control_body) = setup(reduced);
        let request = BodyState {
            position: [1.0; 3],
            velocity: [2.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [3.0; 3],
        };
        let before = edited.state(body);
        assert_eq!(
            edited.set_body_state(body, request),
            Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "jointed body-state replacement",
            })
        );
        assert_eq!(edited.state(body), before);
        assert_eq!(edited.pending_wrenches, control.pending_wrenches);
        edited.step(Duration::from_millis(1)).unwrap();
        control.step(Duration::from_millis(1)).unwrap();
        assert_eq!(edited.state(body), control.state(control_body));
        assert!(edited.state(body).unwrap().angular_velocity[1] > 0.0);
    }
}
