// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual selected-plugin final-tree parity; data stays absent until one reviewed capture.
use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, PlaneSpec, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use std::{
    io::Write,
    process::{Command, Stdio},
    time::Duration,
};
const FIXTURE: &str = include_str!("fixtures/serial_assembly_native.json");
const SOURCE: &[u8] = include_bytes!("fixtures/serial_assembly.cc");
const PROVENANCE: &[u8] = include_bytes!("../../../reference/serial_assembly.json");
fn checksum(bytes: &[u8]) -> String {
    let mut child = Command::new("sha256sum")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child.stdin.take().unwrap().write_all(bytes).unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success() && output.stderr.is_empty());
    String::from_utf8(output.stdout)
        .unwrap()
        .split_whitespace()
        .next()
        .unwrap()
        .to_owned()
}
fn xml(scenario: usize) -> String {
    let z = if scenario == 0 { "1" } else { "0.19" };
    let floor = if scenario == 0 {
        ""
    } else {
        "<model name='floor'><static>true</static><link name='body'><collision name='plane'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>"
    };
    let links=["root","middle","leaf"].map(|name|format!("<link name='{name}'><pose>0 0 {z} 0 0 0</pose><self_collide>false</self_collide><inertial><pose>0 0 0.1 0 0 0</pose><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='sphere'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link>")).join("");
    format!(
        "<sdf version='1.11'><world name='w'><gravity>0 0 {}</gravity>{floor}<model name='chain'><self_collide>false</self_collide>{links}<joint name='first' type='revolute'><parent>root</parent><child>middle</child><axis><xyz>0 0 1</xyz></axis></joint><joint name='second' type='revolute'><parent>middle</parent><child>leaf</child><axis><xyz>0 0 1</xyz></axis></joint></model></world></sdf>",
        if scenario == 0 { "0" } else { "-9.81" }
    )
}
fn close(actual: &[f64], expected: &Value, scenario: usize, tick: u64, name: &str) {
    let expected: Vec<_> = expected
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
    assert_eq!(actual.len(), expected.len());
    for (index, (a, b)) in actual.iter().zip(&expected).enumerate() {
        assert!(
            a.is_finite() && b.is_finite() && (a - b).abs() <= 1e-12,
            "case{scenario} tick{tick} {name}[{index}]: {a:.17e} != {b:.17e}"
        );
    }
}
#[test]
fn atomic_serial_final_tree_matches_actual_plugin_shapes_com_wrench_and_joint_motion() {
    let fixture: Value = serde_json::from_str(FIXTURE).unwrap();
    let provenance: Value = serde_json::from_slice(PROVENANCE).unwrap();
    assert_eq!(fixture["schema_version"], 1);
    assert_eq!(fixture["generator_sha256"], checksum(SOURCE));
    assert_eq!(provenance["generator_sha256"], checksum(SOURCE));
    assert_eq!(
        fixture["capture"]["provenance_sha256"],
        checksum(PROVENANCE)
    );
    assert_eq!(
        fixture["capture"]["stdout_sha256"],
        checksum(fixture["raw_stdout"].as_str().unwrap().as_bytes())
    );
    assert_eq!(fixture["capture"]["stderr_sha256"], checksum(b""));
    assert_eq!(
        provenance["state"],
        "Actual plugin observations collected; no Rust/native comparison claimed"
    );
    assert!(provenance.get("failure").is_none() && provenance.get("traceback").is_none());
    assert_eq!(
        fixture["time_scope"],
        "requested dt times counted ForwardStep calls; not queried simulator clock"
    );
    let raw: Vec<Value> = fixture["raw_stdout"]
        .as_str()
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    let cases = fixture["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 2);
    assert_eq!(raw.len(), 8);
    let materialized: Vec<_> = cases
        .iter()
        .flat_map(|case| case["rows"].as_array().unwrap().iter().cloned())
        .collect();
    assert_eq!(raw, materialized);
    for (scenario, case) in cases.iter().enumerate() {
        assert_eq!(case["scenario"], scenario);
        assert_eq!(case["xml"], xml(scenario));
        assert_eq!(case["xml_sha256"], checksum(xml(scenario).as_bytes()));
        let mut backend = DartBackend::new(if scenario == 0 {
            [0.; 3]
        } else {
            [0., 0., -9.81]
        })
        .unwrap();
        if scenario == 1 {
            backend
                .spawn_static_plane(PlaneSpec {
                    position: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    size: [100.; 2],
                })
                .unwrap();
        }
        let z = if scenario == 0 { 1. } else { 0.19 };
        let bodies: [BodyId; 3] = std::array::from_fn(|_| {
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.2 },
                        position: [0., 0., z],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec::default(),
                    Some(Inertia {
                        center: [0., 0., 0.1],
                        orientation: [0., 0., 0., 1.],
                        tensor: [1., 1., 1., 0., 0., 0.],
                    }),
                )
                .unwrap()
        });
        let joints = backend
            .add_serial_revolutes(bodies, [[0., 0., z]; 2], [[0., 0., 1.]; 2])
            .unwrap();
        if scenario == 1 {
            let initial = case["rows"][0]["bodies"][0]["values"][18].as_f64().unwrap();
            let final_z = case["rows"][3]["bodies"][0]["values"][18].as_f64().unwrap();
            let ballistic = initial - 9.81 * 0.001_f64.powi(2) * 100. * 101. / 2.;
            assert!(
                final_z > ballistic + 0.01,
                "actual native floor response required"
            );
        }
        let before = backend.initial_state();
        // Same immutable data qualifies fresh assembly and prepared reconstruction.
        for epoch in 0..2 {
            let rows = case["rows"].as_array().unwrap();
            assert_eq!(rows.len(), 4);
            let mut matched = 0;
            for tick in 0..=100_u64 {
                if tick > 0 {
                    if tick == 1 {
                        backend
                            .add_world_wrench(bodies[0], [1., 2., 3.], [0.2, -0.1, 0.3])
                            .unwrap();
                    }
                    backend.step(Duration::from_millis(1)).unwrap();
                }
                let Some(row) = rows.iter().find(|r| r["tick"] == tick) else {
                    continue;
                };
                assert_eq!(row["scenario"], scenario);
                assert_eq!(row["requested_time_ns"], tick * 1_000_000);
                matched += 1;
                let states = row["bodies"].as_array().unwrap();
                assert_eq!(states.len(), 3);
                for ((id, name), expected) in bodies
                    .into_iter()
                    .zip(["root", "middle", "leaf"])
                    .zip(states)
                {
                    assert_eq!(expected["name"], name);
                    let state = backend.state(id).unwrap();
                    let r = UnitQuaternion::from_quaternion(Quaternion::new(
                        state.orientation[3],
                        state.orientation[0],
                        state.orientation[1],
                        state.orientation[2],
                    ));
                    let local = Vector3::new(0., 0., 0.1);
                    let com = Vector3::from(state.position) + r * local;
                    let origin = Vector3::from(state.velocity)
                        - Vector3::from(state.angular_velocity).cross(&(r * local));
                    let values: Vec<_> = state
                        .position
                        .into_iter()
                        .chain(state.orientation)
                        .chain(state.velocity)
                        .chain(state.angular_velocity)
                        .chain(origin.iter().copied())
                        .chain(com.iter().copied())
                        .chain(local.iter().copied())
                        .collect();
                    close(&values, &expected["values"], scenario, tick, name);
                }
                let rates = row["joints"].as_array().unwrap();
                assert_eq!(rates.len(), 2);
                for (id, expected) in joints.into_iter().zip(rates) {
                    let state = backend.joint_state(id).unwrap();
                    close(
                        &[state.position, state.velocity],
                        expected,
                        scenario,
                        tick,
                        "joint",
                    );
                }
            }
            assert_eq!(matched, 4);
            if epoch == 0 {
                let prepared = backend.prepare_initial(&before).unwrap();
                backend.apply_initial(prepared).unwrap();
            }
        }
    }
}

/// Compare actual readouts, not a newly synthesized trajectory or relaxed bound.
fn same_body_readout(a: gz_physics::BodyState, b: gz_physics::BodyState) {
    for (a, b) in a
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
        assert!(a.is_finite() && b.is_finite() && (a - b).abs() <= 1e-12);
    }
}

#[test]
fn cold_rotated_parent_frames_preserve_state_and_reconstruct_against_retained_single_edges() {
    // A free noncontact tree admits the retained two single-edge calls. Their
    // live state is an independent construction control, not a native golden.
    let mut atomic = DartBackend::new([0.; 3]).unwrap();
    let mut sequential = DartBackend::new([0.; 3]).unwrap();
    let make = |backend: &mut DartBackend| {
        [
            ([0., 0., 2.], [0.2, -0.3, 0.4]),
            ([2., 1., 3.], [-0.5, 0.2, -0.1]),
            ([4., 2., 4.], [0.3, 0.1, -0.2]),
        ]
        .map(|(position, [roll, pitch, yaw])| {
            let q = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.2 },
                        position,
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec {
                        orientation: [q.i, q.j, q.k, q.w],
                        angular_velocity: [0.; 3],
                    },
                    Some(Inertia {
                        center: [0., 0., 0.1],
                        orientation: [0., 0., 0., 1.],
                        tensor: [1., 1., 1., 0., 0., 0.],
                    }),
                )
                .unwrap()
        })
    };
    let bodies = make(&mut atomic);
    let controls = make(&mut sequential);
    let before = bodies.map(|id| atomic.state(id).unwrap());
    assert!(
        before
            .iter()
            .all(|state| state.orientation != [0., 0., 0., 1.])
    );
    let anchors = [[1., 0.5, 2.5], [3., 1.5, 3.5]];
    let axes = [[0., 0., 1.], [0., 1., 0.]];
    let joints = atomic.add_serial_revolutes(bodies, anchors, axes).unwrap();
    let old = [
        sequential
            .add_revolute(controls[0], controls[1], anchors[0], axes[0])
            .unwrap(),
        sequential
            .add_revolute(controls[1], controls[2], anchors[1], axes[1])
            .unwrap(),
    ];
    assert_eq!(joints.map(|id| id.raw()), [0, 1]);
    assert_eq!(old.map(|id| id.raw()), [0, 1]);
    for ((id, control), before) in bodies.into_iter().zip(controls).zip(before) {
        same_body_readout(atomic.state(id).unwrap(), before);
        same_body_readout(
            atomic.state(id).unwrap(),
            sequential.state(control).unwrap(),
        );
    }
    for id in joints {
        assert_eq!(
            atomic.joint_state(id).unwrap(),
            gz_physics::JointState {
                position: 0.,
                velocity: 0.
            }
        );
    }
    let initial = atomic.initial_state();
    let control_initial = sequential.initial_state();
    for epoch in 0..2 {
        if epoch == 1 {
            // Real queued work must be discarded by the prepared initial reset.
            atomic
                .add_world_wrench(bodies[2], [3., 2., 1.], [0.3, 0.2, 0.1])
                .unwrap();
            let prepared = atomic.prepare_initial(&initial).unwrap();
            atomic.apply_initial(prepared).unwrap();
            let prepared = sequential.prepare_initial(&control_initial).unwrap();
            sequential.apply_initial(prepared).unwrap();
            for (id, before) in bodies.into_iter().zip(before) {
                same_body_readout(atomic.state(id).unwrap(), before);
            }
        }
        // Same actual COM wrench as the existing captured reference protocol,
        // queued once. Existing eight-row/560-value comparator above is intact.
        for (backend, ids) in [(&mut atomic, bodies), (&mut sequential, controls)] {
            backend
                .add_world_wrench(ids[0], [1., 2., 3.], [0.2, -0.1, 0.3])
                .unwrap();
        }
        for _ in 0..3 {
            atomic.step(Duration::from_millis(1)).unwrap();
            sequential.step(Duration::from_millis(1)).unwrap();
            for (id, control) in bodies.into_iter().zip(controls) {
                same_body_readout(
                    atomic.state(id).unwrap(),
                    sequential.state(control).unwrap(),
                );
            }
            for (id, control) in joints.into_iter().zip(old) {
                let (a, b) = (
                    atomic.joint_state(id).unwrap(),
                    sequential.joint_state(control).unwrap(),
                );
                for (a, b) in [(a.position, b.position), (a.velocity, b.velocity)] {
                    assert!(a.is_finite() && b.is_finite() && (a - b).abs() <= 1e-12);
                }
            }
        }
    }
    for id in bodies {
        assert!(atomic.remove(id));
        assert!(atomic.state(id).is_none());
    }
    assert!(
        joints
            .into_iter()
            .all(|id| atomic.joint_state(id).is_none())
    );
}

#[test]
fn child_local_serial_frames_match_the_unchanged_actual_plugin_reference_and_reset() {
    let fixture: Value = serde_json::from_str(FIXTURE).unwrap();
    let provenance: Value = serde_json::from_slice(PROVENANCE).unwrap();
    assert_eq!(fixture["schema_version"], 1);
    assert_eq!(fixture["generator_sha256"], checksum(SOURCE));
    assert_eq!(provenance["generator_sha256"], checksum(SOURCE));
    assert_eq!(
        fixture["capture"]["provenance_sha256"],
        checksum(PROVENANCE)
    );
    assert_eq!(
        fixture["capture"]["stdout_sha256"],
        checksum(fixture["raw_stdout"].as_str().unwrap().as_bytes())
    );
    assert_eq!(
        provenance["state"],
        "Actual plugin observations collected; no Rust/native comparison claimed"
    );
    assert!(provenance.get("failure").is_none() && provenance.get("traceback").is_none());
    let cases = fixture["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 2);
    for (scenario, case) in cases.iter().enumerate() {
        assert_eq!(case["scenario"], scenario);
        assert_eq!(case["xml"], xml(scenario));
        assert_eq!(case["xml_sha256"], checksum(xml(scenario).as_bytes()));
        let mut backend = DartBackend::new(if scenario == 0 {
            [0.; 3]
        } else {
            [0., 0., -9.81]
        })
        .unwrap();
        if scenario == 1 {
            backend
                .spawn_static_plane(PlaneSpec {
                    position: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    size: [100.; 2],
                })
                .unwrap();
        }
        let z = if scenario == 0 { 1. } else { 0.19 };
        let bodies = std::array::from_fn(|_| {
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.2 },
                        position: [0., 0., z],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec::default(),
                    Some(Inertia {
                        center: [0., 0., 0.1],
                        orientation: [0., 0., 0., 1.],
                        tensor: [1., 1., 1., 0., 0., 0.],
                    }),
                )
                .unwrap()
        });
        // Both original XML joints have identity child-relative pose. Unlike
        // world anchors [0,0,z], these are zero CHILD translations/local Z axes.
        let joints = backend
            .add_serial_revolutes_in_child_frames(bodies, [[0.; 3]; 2], [[0., 0., 1.]; 2])
            .unwrap();
        let initial = backend.initial_state();
        for epoch in 0..2 {
            let rows = case["rows"].as_array().unwrap();
            assert_eq!(rows.len(), 4);
            let mut matched = 0;
            for tick in 0..=100_u64 {
                if tick != 0 {
                    if tick == 1 {
                        backend
                            .add_world_wrench(bodies[0], [1., 2., 3.], [0.2, -0.1, 0.3])
                            .unwrap();
                    }
                    backend.step(Duration::from_millis(1)).unwrap();
                }
                let Some(row) = rows.iter().find(|row| row["tick"] == tick) else {
                    continue;
                };
                assert_eq!(row["scenario"], scenario);
                assert_eq!(row["requested_time_ns"], tick * 1_000_000);
                let expected = row["bodies"].as_array().unwrap();
                assert_eq!(expected.len(), 3);
                for ((id, name), expected) in bodies
                    .into_iter()
                    .zip(["root", "middle", "leaf"])
                    .zip(expected)
                {
                    assert_eq!(expected["name"], name);
                    let state = backend.state(id).unwrap();
                    let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
                        state.orientation[3],
                        state.orientation[0],
                        state.orientation[1],
                        state.orientation[2],
                    ));
                    let local = Vector3::new(0., 0., 0.1);
                    let arm = rotation * local;
                    let origin = Vector3::from(state.velocity)
                        - Vector3::from(state.angular_velocity).cross(&arm);
                    let com = Vector3::from(state.position) + arm;
                    let values: Vec<_> = state
                        .position
                        .into_iter()
                        .chain(state.orientation)
                        .chain(state.velocity)
                        .chain(state.angular_velocity)
                        .chain(origin.iter().copied())
                        .chain(com.iter().copied())
                        .chain(local.iter().copied())
                        .collect();
                    close(&values, &expected["values"], scenario, tick, name);
                }
                let expected = row["joints"].as_array().unwrap();
                assert_eq!(expected.len(), 2);
                for (id, expected) in joints.into_iter().zip(expected) {
                    let state = backend.joint_state(id).unwrap();
                    close(
                        &[state.position, state.velocity],
                        expected,
                        scenario,
                        tick,
                        "joint",
                    );
                }
                matched += 1;
            }
            assert_eq!(matched, 4);
            if epoch == 0 {
                backend
                    .add_world_wrench(bodies[2], [3., 2., 1.], [0.3, 0.2, 0.1])
                    .unwrap();
                let prepared = backend.prepare_initial(&initial).unwrap();
                backend.apply_initial(prepared).unwrap();
            }
        }
    }
}

#[test]
fn nonidentity_child_frames_keep_local_axis_and_anchor_semantics_through_reset() {
    let mut local = DartBackend::new([0.; 3]).unwrap();
    let mut world = DartBackend::new([0.; 3]).unwrap();
    let make = |backend: &mut DartBackend| -> [BodyId; 3] {
        std::array::from_fn(|i| {
            let q = UnitQuaternion::from_euler_angles(0.2 + i as f64 * 0.1, -0.1, 0.3);
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.2 },
                        position: [i as f64 * 2., 0.5, 2.],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec {
                        orientation: [q.i, q.j, q.k, q.w],
                        angular_velocity: [0.; 3],
                    },
                    Some(Inertia {
                        center: [0., 0., 0.1],
                        orientation: [0., 0., 0., 1.],
                        tensor: [1., 1., 1., 0., 0., 0.],
                    }),
                )
                .unwrap()
        })
    };
    let bodies = make(&mut local);
    let controls = make(&mut world);
    let before = bodies.map(|id| local.state(id).unwrap());
    let anchors = [[0.3, -0.2, 0.1], [-0.2, 0.1, 0.4]];
    let axes = [[1., 0., 0.], [0., 1., 0.]];
    // Resolve the control through actual preconstruction readouts. The control
    // uses the preserved world API; no independently captured native data is
    // claimed for these nonidentity frames.
    let frames: [(Vector3<f64>, Vector3<f64>); 2] = std::array::from_fn(|i| {
        let state = world.state(controls[i + 1]).unwrap();
        let q = UnitQuaternion::from_quaternion(Quaternion::new(
            state.orientation[3],
            state.orientation[0],
            state.orientation[1],
            state.orientation[2],
        ));
        (
            Vector3::from(state.position) + q * Vector3::from(anchors[i]),
            q * Vector3::from(axes[i]),
        )
    });
    assert!(frames[0].1[1].abs() > 0.1 && frames[1].1[0].abs() > 0.1);
    let joints = local
        .add_serial_revolutes_in_child_frames(bodies, anchors, axes)
        .unwrap();
    let expected = world
        .add_serial_revolutes(
            controls,
            frames.map(|f| f.0.into()),
            frames.map(|f| f.1.into()),
        )
        .unwrap();
    assert_eq!(joints.map(|id| id.raw()), [0, 1]);
    for (id, state) in bodies.into_iter().zip(before) {
        same_body_readout(local.state(id).unwrap(), state);
    }
    let initial = local.initial_state();
    let control_initial = world.initial_state();
    for epoch in 0..2 {
        if epoch == 1 {
            local
                .add_world_wrench(bodies[2], [3., 2., 1.], [0.3, 0.2, 0.1])
                .unwrap();
            local.add_joint_effort(joints[1], 2.).unwrap();
            let prepared = local.prepare_initial(&initial).unwrap();
            local.apply_initial(prepared).unwrap();
            let prepared = world.prepare_initial(&control_initial).unwrap();
            world.apply_initial(prepared).unwrap();
            for (id, state) in bodies.into_iter().zip(before) {
                same_body_readout(local.state(id).unwrap(), state);
            }
        }
        for (backend, ids) in [(&mut local, bodies), (&mut world, controls)] {
            backend
                .add_world_wrench(ids[0], [1., 2., 3.], [0.2, -0.1, 0.3])
                .unwrap();
        }
        for _ in 0..3 {
            local.step(Duration::from_millis(1)).unwrap();
            world.step(Duration::from_millis(1)).unwrap();
            for (id, control) in bodies.into_iter().zip(controls) {
                same_body_readout(local.state(id).unwrap(), world.state(control).unwrap());
            }
            for (joint, control) in joints.into_iter().zip(expected) {
                let (a, b) = (
                    local.joint_state(joint).unwrap(),
                    world.joint_state(control).unwrap(),
                );
                for (a, b) in [(a.position, b.position), (a.velocity, b.velocity)] {
                    assert!(a.is_finite() && b.is_finite() && (a - b).abs() <= 1e-12);
                }
            }
        }
    }
    for id in bodies {
        assert!(local.remove(id));
    }
    assert!(joints.into_iter().all(|id| local.joint_state(id).is_none()));
}
