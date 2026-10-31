// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prepared positives; existing rejection assertions remain pending review.
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, PhysicsError, RotationSpec,
    Shape,
};
use gz_physics_dart::DartBackend;
use std::time::Duration;

fn sphere(
    b: &mut DartBackend,
    position: [f64; 3],
    velocity: [f64; 3],
    angular: [f64; 3],
    mass: Option<f64>,
) -> BodyId {
    b.spawn_link(
        BodySpec {
            shape: Shape::Sphere { radius: 0.1 },
            position,
            velocity,
            mass,
        },
        RotationSpec {
            angular_velocity: angular,
            ..Default::default()
        },
        Some(Inertia {
            center: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
        }),
    )
    .unwrap()
}
fn setup(
    scenario: usize,
    obstacle: bool,
    dynamic: bool,
) -> (DartBackend, [BodyId; 2], Option<BodyId>, Option<JointId>) {
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let shifted = scenario == 2 || scenario == 4;
    let fixed = scenario == 1 || shifted;
    let z = if shifted { -1.0 } else { 0.0 };
    let moving = scenario != 2 && scenario != 3;
    let (linear, child_linear) = if moving {
        if shifted { (-1.0, -5.0) } else { (0.2, -3.8) }
    } else {
        (0.0, 0.0)
    };
    let a = sphere(
        &mut b,
        [0.0, 0.0, z],
        [linear, 0.0, 0.0],
        [0.0, if moving { 1.0 } else { 0.0 }, 0.0],
        Some(1.0),
    );
    let c = sphere(
        &mut b,
        [0.0, 0.0, z - 2.0],
        [child_linear, 0.0, 0.0],
        [0.0, if moving { 3.0 } else { 0.0 }, 0.0],
        Some(1.0),
    );
    if scenario < 2 {
        b.add_world_wrench(a, [0.3, 0.0, 0.0], [0.0, 0.2, 0.0])
            .unwrap();
    }
    let root = if fixed {
        Some(b.add_world_revolute(a, [0.0; 3], [0.0, 1.0, 0.0]).unwrap())
    } else {
        None
    };
    let contact = if obstacle {
        Some(sphere(
            &mut b,
            match scenario {
                2 => [0.0, 0.0, -1.15],
                3 => [0.205, 0.0, 0.0],
                4 => [-0.205, 0.0, -3.0],
                _ => [0.15, 0.0, 0.0],
            },
            [0.0; 3],
            [0.0; 3],
            if dynamic { Some(1.0) } else { None },
        ))
    } else {
        None
    };
    (b, [a, c], contact, root)
}
fn attach(b: &mut DartBackend, bodies: [BodyId; 2], scenario: usize) -> JointId {
    b.add_revolute(
        bodies[0],
        bodies[1],
        [
            0.0,
            0.0,
            if scenario == 2 || scenario == 4 {
                -2.0
            } else {
                -1.0
            },
        ],
        [0.0, 1.0, 0.0],
    )
    .unwrap()
}
fn near(a: BodyState, b: BodyState) {
    for (x, y) in a
        .position
        .into_iter()
        .chain(a.orientation)
        .chain(a.velocity)
        .chain(a.angular_velocity)
        .zip(
            b.position
                .into_iter()
                .chain(b.orientation)
                .chain(b.velocity)
                .chain(b.angular_velocity),
        )
    {
        assert!((x - y).abs() < 1e-12, "{a:?} != {b:?}");
    }
}
#[test]
fn exact_old_static_contact_inputs_match_native_plugin_and_keep_lifecycle() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/static_contacts.json")).unwrap();
    for scenario in [0, 3] {
        let (mut b, bodies, _, root) = setup(scenario, true, false);
        let before = bodies.map(|id| b.state(id).unwrap());
        let leaf = attach(&mut b, bodies, scenario);
        for (id, state) in bodies.into_iter().zip(before) {
            let after = b.state(id).unwrap();
            assert_eq!(after.position, state.position);
            assert_eq!(after.orientation, state.orientation);
        }
        assert_eq!(leaf.raw(), u64::from(root.is_some()));
        if scenario == 3 {
            b.set_world_velocity(bodies[0], [1.0, 0.0, 0.0], [0.0; 3])
                .unwrap();
        }
        let initial = b.initial_state();
        let mut tick = 0;
        for row in fixture["observations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|r| r["scenario"].as_u64().unwrap() == scenario as u64)
        {
            while tick < row["tick"].as_u64().unwrap() {
                tick += 1;
                if scenario != 2 && (scenario >= 3 || tick == 1) {
                    b.add_joint_effort(leaf, 0.7).unwrap();
                }
                b.step(Duration::from_millis(1)).unwrap();
            }
            for (i, id) in bodies.iter().enumerate() {
                let state = b.state(*id).unwrap();
                for (field, actual) in [
                    ("position", state.position.to_vec()),
                    ("orientation", state.orientation.to_vec()),
                    ("velocity", state.velocity.to_vec()),
                    ("angular_velocity", state.angular_velocity.to_vec()),
                ] {
                    for (a, reference) in actual
                        .into_iter()
                        .zip(row["bodies"][i][field].as_array().unwrap())
                    {
                        let expected = reference.as_f64().unwrap();
                        assert!(
                            (a - expected).abs() < 1e-12,
                            "scenario={scenario} tick={tick} body={i} {field}: {a} != {expected}"
                        );
                    }
                }
            }
            let state = b.joint_state(leaf).unwrap();
            for (field, a) in [("q", state.position), ("dq", state.velocity)] {
                assert!(
                    (a - row[field].as_f64().unwrap()).abs() < 1e-12,
                    "scenario={scenario} tick={tick} {field}"
                );
            }
        }
        let before = bodies.map(|id| b.state(id).unwrap());
        assert!(b.remove_joint(leaf));
        for (id, state) in bodies.into_iter().zip(before) {
            near(b.state(id).unwrap(), state);
        }
        let prepared = b.prepare_initial(&initial).unwrap();
        b.apply_initial(prepared).unwrap();
        assert_eq!(b.joint_child(leaf), Some(bodies[1]));
        b.step(Duration::from_millis(1)).unwrap();
        assert!(b.remove(bodies[1]));
        b.step(Duration::from_millis(1)).unwrap();
    }
}
#[test]
fn dynamic_contact_attachment_refusal_preserves_native_future_motion() {
    for scenario in 0..3 {
        let (mut failed, bodies, contact, _) = setup(scenario, true, true);
        let (mut clean, controls, _, _) = setup(scenario, false, false);
        let before = bodies.map(|id| failed.state(id));
        for _ in 0..2 {
            assert_eq!(
                failed.add_revolute(
                    bodies[0],
                    bodies[1],
                    [0.0, 0.0, if scenario == 2 { -2.0 } else { -1.0 }],
                    [0.0, 1.0, 0.0]
                ),
                Err(PhysicsError::InvalidJoint)
            );
            assert_eq!(bodies.map(|id| failed.state(id)), before);
        }
        assert!(failed.remove(contact.unwrap()));
        let leaf = attach(&mut failed, bodies, scenario);
        let control = attach(&mut clean, controls, scenario);
        failed.add_joint_effort(leaf, 0.7).unwrap();
        clean.add_joint_effort(control, 0.7).unwrap();
        for _ in 0..3 {
            failed.step(Duration::from_millis(1)).unwrap();
            clean.step(Duration::from_millis(1)).unwrap();
            for (a, b) in bodies.into_iter().zip(controls) {
                near(failed.state(a).unwrap(), clean.state(b).unwrap());
            }
        }
    }
}
#[test]
fn approaching_dynamic_contact_refusal_preserves_pending_effort_and_future_motion() {
    for scenario in [3, 4] {
        let (mut failed, bodies, contact, _) = setup(scenario, true, true);
        let (mut clean, controls, _, _) = setup(scenario, false, false);
        let leaf = attach(&mut failed, bodies, scenario);
        let control = attach(&mut clean, controls, scenario);
        if scenario == 3 {
            for (b, root) in [(&mut failed, bodies[0]), (&mut clean, controls[0])] {
                b.set_world_velocity(root, [1.0, 0.0, 0.0], [0.0; 3])
                    .unwrap();
            }
        }
        let mut refused = false;
        for _ in 0..20 {
            failed.add_joint_effort(leaf, 0.7).unwrap();
            clean.add_joint_effort(control, 0.7).unwrap();
            let before = bodies.map(|id| failed.state(id));
            let rate = failed.joint_state(leaf);
            if let Err(error) = failed.step(Duration::from_millis(1)) {
                assert!(error.to_string().contains("contact"), "{error}");
                assert_eq!(bodies.map(|id| failed.state(id)), before);
                assert_eq!(failed.joint_state(leaf), rate);
                assert!(failed.step(Duration::from_millis(1)).is_err());
                assert_eq!(bodies.map(|id| failed.state(id)), before);
                assert!(failed.remove(contact.unwrap()));
                refused = true;
                break;
            }
            clean.step(Duration::from_millis(1)).unwrap();
        }
        assert!(refused);
        for _ in 0..3 {
            failed.step(Duration::from_millis(1)).unwrap();
            clean.step(Duration::from_millis(1)).unwrap();
            for (a, b) in bodies.into_iter().zip(controls) {
                near(failed.state(a).unwrap(), clean.state(b).unwrap());
            }
        }
    }
}

fn ill_conditioned_pair(ground: bool) -> (DartBackend, [BodyId; 2], JointId, Option<BodyId>) {
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let floor = if ground {
        Some(
            b.spawn_static_plane(gz_physics::PlaneSpec {
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                size: [100.0; 2],
            })
            .unwrap(),
        )
    } else {
        None
    };
    let mut bodies = Vec::new();
    for (mass, moment, z) in [(0.001, 0.0001, 0.105), (1e8, 1e7, 0.405)] {
        bodies.push(
            b.spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.1 },
                    position: [0.0, 0.0, z],
                    velocity: [0.0, 0.0, -0.1],
                    mass: Some(mass),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.0; 3],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [moment, moment, moment, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap(),
        );
    }
    let bodies = [bodies[0], bodies[1]];
    let joint = b
        .add_revolute(bodies[0], bodies[1], [0.0, 0.0, 0.255], [0.0, 0.0, 1.0])
        .unwrap();
    (b, bodies, joint, floor)
}
#[test]
fn scaled_matrix_contact_refusal_restores_all_rates_and_future_motion() {
    let (mut failed, bodies, joint, floor) = ill_conditioned_pair(true);
    let (mut clean, controls, control, _) = ill_conditioned_pair(false);
    let mut refused = false;
    for _ in 0..100 {
        failed.add_joint_effort(joint, 0.00001).unwrap();
        clean.add_joint_effort(control, 0.00001).unwrap();
        let before = bodies.map(|id| failed.state(id));
        let rate = failed.joint_state(joint);
        if let Err(error) = failed.step(Duration::from_millis(1)) {
            assert!(error.to_string().contains("contact"), "{error}");
            assert_eq!(bodies.map(|id| failed.state(id)), before);
            assert_eq!(failed.joint_state(joint), rate);
            assert!(failed.step(Duration::from_millis(1)).is_err());
            assert_eq!(bodies.map(|id| failed.state(id)), before);
            assert!(failed.remove(floor.unwrap()));
            refused = true;
            break;
        }
        clean.step(Duration::from_millis(1)).unwrap();
    }
    assert!(refused);
    for _ in 0..3 {
        failed.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near(failed.state(a).unwrap(), clean.state(b).unwrap());
        }
    }
}

#[test]
fn exact_free_root_active_limit_input_still_refuses_and_preserves_future_motion() {
    let (mut failed, bodies, _, _) = setup(3, false, false);
    let (mut clean, controls, _, _) = setup(3, false, false);
    let leaf = attach(&mut failed, bodies, 3);
    let control = attach(&mut clean, controls, 3);
    failed
        .set_world_velocity(bodies[1], [-1e20, 0.0, 0.0], [0.0, 1e20, 0.0])
        .unwrap();
    failed.add_joint_effort(leaf, 0.7).unwrap();
    clean.add_joint_effort(control, 0.7).unwrap();
    let before = bodies.map(|id| failed.state(id));
    let rate = failed.joint_state(leaf);
    for _ in 0..2 {
        let error = failed.step(Duration::from_millis(1)).unwrap_err();
        assert!(error.to_string().contains("active joint limit"), "{error}");
        assert_eq!(bodies.map(|id| failed.state(id)), before);
        assert_eq!(failed.joint_state(leaf), rate);
    }
    failed
        .set_world_velocity(bodies[1], [0.0; 3], [0.0; 3])
        .unwrap();
    for _ in 0..3 {
        failed.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near(failed.state(a).unwrap(), clean.state(b).unwrap());
        }
    }
}
