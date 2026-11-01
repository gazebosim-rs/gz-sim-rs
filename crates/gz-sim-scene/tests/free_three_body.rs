// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Scene composition with the accepted contact-free native backend contract.
use gz_physics::{BodySpec, DynamicBackend, Inertia, PhysicsBackend, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    queue_joint_effort, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::{collections::BTreeSet, time::Duration};

fn xml(reverse: bool) -> String {
    let links = [("root", "0 0 0"), ("middle", "0 0 -2"), ("leaf", "10 0 0")]
        .map(|(name, pose)| format!("<link name='{name}'><pose>{pose} 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>")).join("");
    let first = "<joint name='first' type='revolute'><parent>root</parent><child>middle</child><pose relative_to='__model__'>0 0 -1 0 0 0</pose><axis><xyz expressed_in='__model__'>0 1 0</xyz></axis></joint>";
    let second = "<joint name='second' type='revolute'><parent>middle</parent><child>leaf</child><pose relative_to='__model__'>0 0 0 0 0 0</pose><axis><xyz expressed_in='__model__'>0 1 0</xyz></axis></joint>";
    let joints = if reverse {
        format!("{second}{first}")
    } else {
        format!("{first}{second}")
    };
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='chain'>{links}{joints}</model></world></sdf>"
    )
}
fn public_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == format!("default::{name}"))
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn close(a: impl IntoIterator<Item = f64>, b: impl IntoIterator<Item = f64>) {
    let a: Vec<_> = a.into_iter().collect();
    let b: Vec<_> = b.into_iter().collect();
    assert_eq!(a.len(), b.len());
    for (a, b) in a.into_iter().zip(b) {
        assert!((a - b).abs() < 1e-12, "{a} != {b}");
    }
}
#[test]
fn serial_chain_declaration_order_sampling_reset_and_removal() {
    for reverse in [false, true] {
        let mut sim = load_world_with_sdf_step(
            &xml(reverse),
            "free-three-body.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Dart,
        )
        .unwrap();
        let bodies = ["chain::root", "chain::middle", "chain::leaf"].map(|n| public_id(&sim, n));
        let joints = ["chain::first", "chain::second"].map(|n| public_id(&sim, n));
        assert_eq!(joints[0] < joints[1], !reverse);
        for epoch in 0..2 {
            let mut reference = DartBackend::new([0.; 3]).unwrap();
            let controls = [[0.; 3], [0., 0., -2.], [10., 0., 0.]].map(|position| {
                reference
                    .spawn_link(
                        BodySpec {
                            shape: Shape::Sphere { radius: 0.1 },
                            position,
                            velocity: [0.; 3],
                            mass: Some(1.),
                        },
                        RotationSpec::default(),
                        Some(Inertia {
                            center: [0.; 3],
                            orientation: [0., 0., 0., 1.],
                            tensor: [0.004, 0.004, 0.004, 0., 0., 0.],
                        }),
                    )
                    .unwrap()
            });
            let hinges = [
                reference
                    .add_revolute(controls[0], controls[1], [0., 0., -1.], [0., 1., 0.])
                    .unwrap(),
                reference
                    .add_revolute(controls[1], controls[2], [0.; 3], [0., 1., 0.])
                    .unwrap(),
            ];
            for tick in 1..=10 {
                for (id, hinge, effort) in
                    [(joints[0], hinges[0], 0.7), (joints[1], hinges[1], -0.2)]
                {
                    queue_joint_effort(&mut sim, id, effort).unwrap();
                    reference.add_joint_effort(hinge, effort).unwrap();
                }
                reference.step(Duration::from_millis(1)).unwrap();
                sim.step_exact(1).unwrap();
                for (id, control) in bodies.into_iter().zip(controls) {
                    let e = entity_by_id(sim.world(), id).unwrap();
                    let handle = sim.world().get::<BodyHandle>(e).unwrap();
                    let actual = sim
                        .world()
                        .resource::<PhysicsBackend>()
                        .state(handle.id())
                        .unwrap();
                    let expected = reference.state(control).unwrap();
                    close(
                        actual
                            .position
                            .into_iter()
                            .chain(actual.orientation)
                            .chain(actual.velocity)
                            .chain(actual.angular_velocity),
                        expected
                            .position
                            .into_iter()
                            .chain(expected.orientation)
                            .chain(expected.velocity)
                            .chain(expected.angular_velocity),
                    );
                    let pose = sim.world().get::<WorldPose>(e).unwrap();
                    close(pose.translation, expected.position);
                    close(pose.rotation, expected.orientation);
                }
                for (id, hinge, child) in [
                    (joints[0], hinges[0], bodies[1]),
                    (joints[1], hinges[1], bodies[2]),
                ] {
                    let e = entity_by_id(sim.world(), id).unwrap();
                    let handle = sim.world().get::<JointHandle>(e).unwrap();
                    assert_eq!(handle.child(), child);
                    let sample = sim.world().get::<JointSample>(e).unwrap();
                    let expected = reference.joint_state(hinge).unwrap();
                    close(
                        [sample.state.position, sample.state.velocity],
                        [expected.position, expected.velocity],
                    );
                    assert_eq!(sample.time, Duration::from_millis(tick));
                    assert_eq!(sample.epoch, epoch);
                }
            }
            if epoch == 0 {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
            }
        }
        let model = public_id(&sim, "chain");
        remove_model(&mut sim, model).unwrap();
        for id in bodies.into_iter().chain(joints) {
            assert!(entity_by_id(sim.world(), id).is_none());
        }
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn free_three_body_scope_keeps_unqualified_topologies_and_limits_closed() {
    let base = xml(false);
    let cases = [
        base.replace("<parent>middle</parent>", "<parent>root</parent>"),
        base.replace("<parent>root</parent>", "<parent>leaf</parent>"),
        base.replace("<parent>root</parent>", "<parent>world</parent>"),
        base.replace("</model>", "<link name='extra'><pose>100 0 0 0 0 0</pose><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>"),
        base.replace("</model>", "<model name='nested'/></model>"),
        base.replace(
            "<model name='chain'>",
            "<model name='chain'><static>true</static>",
        ),
        base.replace("</axis>", "</axis><axis2><xyz>1 0 0</xyz></axis2>"),
        base.replacen("<child>middle</child>", "<child>leaf</child>", 1),
        base.replace(
            "</axis>",
            "<limit><lower>0</lower><upper>1</upper></limit></axis>",
        ),
    ];
    for xml in cases {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "unsupported-three-body.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Dart
            )
            .is_err()
        );
    }
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Mujoco] {
        assert!(
            load_world_with_sdf_step(
                &base,
                "unsupported-engine.sdf",
                "default",
                &NoResources,
                engine
            )
            .is_err()
        );
    }
}

#[test]
fn exact_native_scene_inputs_match_body_joint_and_semantic_frame_observations() {
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/free_three_body_scene.json"
    ))
    .unwrap();
    let cases = fixture["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 2);
    assert_eq!(
        cases
            .iter()
            .map(|case| case["reverse_declarations"].as_bool().unwrap())
            .collect::<BTreeSet<_>>(),
        BTreeSet::from([false, true]),
    );
    for case in cases {
        let reverse = case["reverse_declarations"].as_bool().unwrap();
        assert_eq!(case["xml"].as_str().unwrap(), xml(reverse));
        let mut sim = load_world_with_sdf_step(
            &xml(reverse),
            "native-free-three-body.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Dart,
        )
        .unwrap();
        let values = |row: &serde_json::Value| {
            row.as_array()
                .unwrap()
                .iter()
                .map(|value| value.as_f64().unwrap())
                .collect::<Vec<_>>()
        };
        let semantic_joints = case["semantic_joints"].as_array().unwrap();
        assert_eq!(semantic_joints.len(), 2);
        assert_eq!(
            semantic_joints
                .iter()
                .map(|row| row["name"].as_str().unwrap())
                .collect::<BTreeSet<_>>(),
            BTreeSet::from(["first", "second"])
        );
        for joint in semantic_joints {
            let id = public_id(&sim, &format!("chain::{}", joint["name"].as_str().unwrap()));
            let entity = entity_by_id(sim.world(), id).unwrap();
            let pose = sim.world().get::<WorldPose>(entity).unwrap();
            let expected = values(&joint["resolved_model_frame_pose"]);
            assert_eq!(expected.len(), 7);
            close(pose.translation.into_iter().chain(pose.rotation), expected);
        }
        for (key, count) in [("commands", 20), ("body_states", 30), ("joint_states", 20)] {
            assert_eq!(case[key].as_array().unwrap().len(), count);
        }
        for tick in 1..=10 {
            let at_tick = |key: &str| {
                case[key]
                    .as_array()
                    .unwrap()
                    .iter()
                    .filter(|row| row["tick"].as_u64().unwrap() == tick)
                    .collect::<Vec<_>>()
            };
            let commands = at_tick("commands");
            assert_eq!(commands.len(), 2);
            assert_eq!(
                commands
                    .iter()
                    .map(|row| row["name"].as_str().unwrap())
                    .collect::<BTreeSet<_>>(),
                BTreeSet::from(["first", "second"])
            );
            for command in commands {
                let effort = if command["name"] == "first" {
                    0.7
                } else {
                    -0.2
                };
                assert_eq!(command["effort"].as_f64(), Some(effort));
                let id = public_id(
                    &sim,
                    &format!("chain::{}", command["name"].as_str().unwrap()),
                );
                queue_joint_effort(&mut sim, id, command["effort"].as_f64().unwrap()).unwrap();
            }
            sim.step_exact(1).unwrap();
            let bodies = at_tick("body_states");
            assert_eq!(bodies.len(), 3);
            assert_eq!(
                bodies
                    .iter()
                    .map(|row| row["name"].as_str().unwrap())
                    .collect::<BTreeSet<_>>(),
                BTreeSet::from(["root", "middle", "leaf"])
            );
            for body in bodies {
                let id = public_id(&sim, &format!("chain::{}", body["name"].as_str().unwrap()));
                let entity = entity_by_id(sim.world(), id).unwrap();
                let handle = sim.world().get::<BodyHandle>(entity).unwrap();
                let state = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .state(handle.id())
                    .unwrap();
                let expected = values(&body["values"]);
                assert_eq!(expected.len(), 13);
                close(
                    state
                        .position
                        .into_iter()
                        .chain(state.orientation)
                        .chain(state.velocity)
                        .chain(state.angular_velocity),
                    expected.iter().copied(),
                );
                let pose = sim.world().get::<WorldPose>(entity).unwrap();
                close(
                    pose.translation.into_iter().chain(pose.rotation),
                    expected[..7].iter().copied(),
                );
            }
            let joints = at_tick("joint_states");
            assert_eq!(joints.len(), 2);
            assert_eq!(
                joints
                    .iter()
                    .map(|row| row["name"].as_str().unwrap())
                    .collect::<BTreeSet<_>>(),
                BTreeSet::from(["first", "second"])
            );
            for joint in joints {
                let id = public_id(&sim, &format!("chain::{}", joint["name"].as_str().unwrap()));
                let entity = entity_by_id(sim.world(), id).unwrap();
                let sample = sim.world().get::<JointSample>(entity).unwrap();
                let expected = values(&joint["values"]);
                assert_eq!(expected.len(), 2);
                close([sample.state.position, sample.state.velocity], expected);
                assert_eq!(sample.time, Duration::from_millis(tick));
                assert_eq!(sample.epoch, 0);
            }
        }
    }
}
