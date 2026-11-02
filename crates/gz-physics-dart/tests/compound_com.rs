// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Offset-COM source controls; parent negative dispositions remain separate.
use gz_physics::{
    BodyId, BodySpec, BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia,
    PhysicsError, RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
const SPHERE: CollisionSpec = CollisionSpec {
    shape: Shape::Sphere { radius: 0.2 },
    position: [0.0; 3],
    orientation: IDENTITY,
};
fn q(rpy: [f64; 3]) -> [f64; 4] {
    let q = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
    [q.i, q.j, q.k, q.w]
}
fn rotation(q: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2]))
}
fn inertia(center: [f64; 3]) -> Inertia {
    Inertia {
        center,
        orientation: IDENTITY,
        tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
    }
}
fn spawn(
    backend: &mut DartBackend,
    shapes: &[CollisionSpec],
    position: [f64; 3],
    center: [f64; 3],
    velocity: [f64; 3],
    rotational: RotationSpec,
) -> Result<BodyId, PhysicsError> {
    backend.spawn_compound_link(
        CompoundBodySpec {
            collisions: shapes,
            position,
            velocity,
            mass: Some(1.0),
        },
        rotational,
        inertia(center),
    )
}
fn near_values(a: impl IntoIterator<Item = f64>, b: impl IntoIterator<Item = f64>) {
    let a: Vec<_> = a.into_iter().collect();
    let b: Vec<_> = b.into_iter().collect();
    assert_eq!(a.len(), b.len());
    for (a, b) in a.into_iter().zip(b) {
        assert!((a - b).abs() <= 1e-12, "{a:.17e} != {b:.17e}");
    }
}
fn near(a: BodyState, b: BodyState) {
    near_values(
        a.position
            .into_iter()
            .chain(a.orientation)
            .chain(a.velocity)
            .chain(a.angular_velocity),
        b.position
            .into_iter()
            .chain(b.orientation)
            .chain(b.velocity)
            .chain(b.angular_velocity),
    );
}
fn center_position(state: BodyState, center: [f64; 3]) -> Vector3<f64> {
    Vector3::from(state.position) + rotation(state.orientation) * Vector3::from(center)
}

#[test]
fn one_compound_shape_reuses_single_body_com_twist_impulse_and_inertia_semantics() {
    let center = [0.03, -0.02, 0.05];
    let velocity = [0.2, -0.1, 0.3];
    let rotational = RotationSpec {
        orientation: q([0.2, 0.1, -0.3]),
        angular_velocity: [0.1, 0.2, -0.3],
    };
    let explicit = Inertia {
        orientation: q([0.1, 0.2, 0.3]),
        tensor: [1.0, 1.5, 2.0, 0.0, 0.0, 0.0],
        ..inertia(center)
    };
    let mut old = DartBackend::new([0.0; 3]).unwrap();
    let mut new = DartBackend::new([0.0; 3]).unwrap();
    let a = old
        .spawn_link(
            BodySpec {
                shape: SPHERE.shape,
                position: [1.0, 2.0, 3.0],
                velocity,
                mass: Some(2.0),
            },
            rotational,
            Some(explicit),
        )
        .unwrap();
    let b = new
        .spawn_compound_link(
            CompoundBodySpec {
                collisions: &[SPHERE],
                position: [1.0, 2.0, 3.0],
                velocity,
                mass: Some(2.0),
            },
            rotational,
            explicit,
        )
        .unwrap();
    near(old.state(a).unwrap(), new.state(b).unwrap());
    near_values(new.state(b).unwrap().velocity, velocity);
    let origin_velocity = Vector3::from(velocity)
        - Vector3::from(rotational.angular_velocity)
            .cross(&(rotation(rotational.orientation) * Vector3::from(center)));
    assert!((origin_velocity - Vector3::from(velocity)).norm() > 0.001);
    for (backend, id) in [(&mut old, a), (&mut new, b)] {
        backend
            .apply_impulse(id, [0.3, -0.2, 0.1], [0.2, -0.1, 0.3])
            .unwrap();
        backend
            .add_world_wrench(id, [3.0, -2.0, 1.0], [0.1, 0.2, 0.3])
            .unwrap();
    }
    for _ in 0..10 {
        old.step(DT).unwrap();
        new.step(DT).unwrap();
        near(old.state(a).unwrap(), new.state(b).unwrap());
    }
}

#[test]
fn changing_body_origin_matches_each_native_representation_and_consumes_one_wrench() {
    let center = [0.1, -0.05, 0.2];
    let origin = [1.0, 2.0, 3.0];
    let rotational = RotationSpec {
        orientation: q([0.2, 0.1, -0.3]),
        angular_velocity: [0.1, 0.2, -0.3],
    };
    let mut shapes = [SPHERE; 3];
    shapes[0].position = [-0.4, 0.0, 0.0];
    shapes[1].shape = Shape::Box {
        half_extents: [0.3, 0.2, 0.4],
    };
    shapes[1].orientation = q([0.0, "1.5708".parse().unwrap(), 0.3]);
    shapes[2].position = [0.4, 0.0, 0.0];
    let shifted = shapes.map(|shape| CollisionSpec {
        position: (Vector3::from(shape.position) - Vector3::from(center)).into(),
        ..shape
    });
    let mut a = DartBackend::new([0.0; 3]).unwrap();
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let first = spawn(
        &mut a,
        &shapes,
        origin,
        center,
        [0.2, -0.1, 0.3],
        rotational,
    )
    .unwrap();
    let com = Vector3::from(origin) + rotation(rotational.orientation) * Vector3::from(center);
    let second = spawn(
        &mut b,
        &shifted,
        com.into(),
        [0.0; 3],
        [0.2, -0.1, 0.3],
        rotational,
    )
    .unwrap();
    fn checksum(bytes: &[u8]) -> String {
        use std::io::Write;
        use std::process::{Command, Stdio};
        let mut child = Command::new("sha256sum")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        child.stdin.take().unwrap().write_all(bytes).unwrap();
        let output = child.wait_with_output().unwrap();
        assert!(output.status.success());
        assert!(output.stderr.is_empty());
        String::from_utf8(output.stdout)
            .unwrap()
            .split_whitespace()
            .next()
            .unwrap()
            .to_owned()
    }
    fn values(row: &serde_json::Value, key: &str, count: usize) -> Vec<f64> {
        let result: Vec<_> = row[key]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        assert_eq!(result.len(), count, "{key}: {row}");
        assert!(result.iter().all(|v| v.is_finite()), "{key}: {row}");
        result
    }
    fn observed(
        actual: impl IntoIterator<Item = f64>,
        expected: Vec<f64>,
        scenario: usize,
        tick: usize,
        field: &str,
    ) {
        let actual: Vec<_> = actual.into_iter().collect();
        assert_eq!(actual.len(), expected.len());
        for (index, (a, b)) in actual.iter().zip(&expected).enumerate() {
            assert!(
                a.is_finite() && (*a - *b).abs() <= 1e-12,
                "case{scenario} tick{tick} {field}[{index}]: {a:.17e} != {b:.17e}; Rust {actual:?}, native {expected:?}"
            );
        }
    }
    const PROVENANCE: &[u8] = include_bytes!("../../../reference/compound_com_gauge.json");
    const GENERATOR: &[u8] = include_bytes!("fixtures/compound_com_gauge.cc");
    const RAW: &str = include_str!("fixtures/compound_com_gauge.jsonl");
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/compound_com_gauge.json")).unwrap();
    let provenance: serde_json::Value = serde_json::from_slice(PROVENANCE).unwrap();
    assert_eq!(fixture["schema_version"], 1);
    assert_eq!(fixture["generator_sha256"], checksum(GENERATOR));
    assert_eq!(provenance["generator_sha256"], checksum(GENERATOR));
    assert_eq!(
        fixture["capture"]["provenance_sha256"],
        checksum(PROVENANCE)
    );
    assert_eq!(
        fixture["capture"]["stdout_sha256"],
        checksum(RAW.as_bytes())
    );
    assert_eq!(fixture["capture"]["stderr_sha256"], checksum(b""));
    assert_eq!(
        provenance["state"],
        "Actual plugin observations collected; no Rust/native comparison claimed"
    );
    assert!(provenance.get("failure").is_none() && provenance.get("traceback").is_none());
    assert_eq!(
        fixture["time_scope"],
        "Row time_ns is requested dt times counted steps, not queried native clock"
    );
    assert_eq!(fixture["time_scope"], provenance["time_scope"]);
    let inputs_raw = fixture["inputs_raw"].as_str().unwrap();
    assert_eq!(
        fixture["capture"]["inputs_sha256"],
        checksum(inputs_raw.as_bytes())
    );
    let parsed_inputs: serde_json::Value = serde_json::from_str(inputs_raw).unwrap();
    assert_eq!(parsed_inputs, fixture["inputs"]);
    let inputs = &fixture["inputs"];
    assert_eq!(
        inputs["source_test"],
        "changing_body_origin_preserves_world_shapes_com_motion_and_pending_wrenches"
    );
    assert_eq!(inputs["dt_ns"], 1_000_000);
    assert_eq!(inputs["steps"], 10);
    assert_eq!(inputs["mass"], 1);
    near_values(values(inputs, "gravity", 3), [0.0; 3]);
    near_values(values(inputs, "inertia", 6), [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]);
    near_values(values(inputs, "inertial_orientation", 4), IDENTITY);
    near_values(
        values(inputs, "raw_body_quaternion", 4),
        rotational.orientation,
    );
    near_values(values(inputs, "body_quaternion", 4), rotational.orientation);
    near_values(values(inputs, "box_quaternion", 4), shapes[1].orientation);
    near_values(values(inputs, "com_velocity", 3), [0.2, -0.1, 0.3]);
    near_values(
        values(inputs, "angular_velocity", 3),
        rotational.angular_velocity,
    );
    near_values(values(inputs, "first_step_world_force", 3), [1.0, 2.0, 3.0]);
    near_values(
        values(inputs, "first_step_world_torque", 3),
        [0.2, -0.1, 0.3],
    );
    let cases = fixture["cases"].as_array().unwrap();
    let declared_cases = inputs["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 2);
    assert_eq!(declared_cases.len(), 2);
    let raw: Vec<serde_json::Value> = RAW
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(raw.len(), 22);
    let materialized: Vec<_> = cases
        .iter()
        .flat_map(|case| case["rows"].as_array().unwrap().iter().cloned())
        .collect();
    assert_eq!(raw, materialized);
    for scenario in 0..2 {
        assert_eq!(cases[scenario]["scenario"], scenario);
        assert_eq!(declared_cases[scenario]["scenario"], scenario);
        let xml = cases[scenario]["xml"].as_str().unwrap();
        assert_eq!(cases[scenario]["xml_sha256"], checksum(xml.as_bytes()));
        assert_eq!(cases[scenario]["rows"].as_array().unwrap().len(), 11);
        near_values(
            values(&declared_cases[scenario], "position", 3),
            if scenario == 0 { origin } else { com.into() },
        );
        near_values(
            values(&declared_cases[scenario], "local_com", 3),
            if scenario == 0 { center } else { [0.0; 3] },
        );
    }
    // The two physical representations agree before integration. The selected
    // SDK does not preserve that cross-gauge equality after a finite step.
    let initial = [a.state(first).unwrap(), b.state(second).unwrap()];
    near_values(
        center_position(initial[0], center).iter().copied(),
        initial[1].position,
    );
    near_values(initial[0].orientation, initial[1].orientation);
    near_values(initial[0].velocity, initial[1].velocity);
    near_values(initial[0].angular_velocity, initial[1].angular_velocity);
    for (one, two) in shapes.into_iter().zip(shifted) {
        near_values(
            (Vector3::from(initial[0].position)
                + rotation(initial[0].orientation) * Vector3::from(one.position))
            .iter()
            .copied(),
            (Vector3::from(initial[1].position)
                + rotation(initial[1].orientation) * Vector3::from(two.position))
            .iter()
            .copied(),
        );
        near_values(
            (rotation(initial[0].orientation) * rotation(one.orientation))
                .coords
                .iter()
                .copied(),
            (rotation(initial[1].orientation) * rotation(two.orientation))
                .coords
                .iter()
                .copied(),
        );
    }
    let native_com_delta = cases[0]["rows"][1]["com_position"][0].as_f64().unwrap()
        - cases[1]["rows"][1]["com_position"][0].as_f64().unwrap();
    assert!(
        native_com_delta > 1e-12,
        "captured SDK cross-gauge difference disappeared"
    );
    for (backend, id) in [(&mut a, first), (&mut b, second)] {
        backend
            .add_world_wrench(id, [1.0, 2.0, 3.0], [0.2, -0.1, 0.3])
            .unwrap();
    }
    let mut first_centered_twist = None;
    for tick in 0..=10 {
        if tick > 0 {
            a.step(DT).unwrap();
            b.step(DT).unwrap();
        }
        for (scenario, state) in [a.state(first).unwrap(), b.state(second).unwrap()]
            .into_iter()
            .enumerate()
        {
            let local = if scenario == 0 { center } else { [0.0; 3] };
            let row = &cases[scenario]["rows"][tick];
            assert_eq!(row["scenario"], scenario);
            assert_eq!(row["tick"], tick);
            assert_eq!(row["time_ns"], tick * 1_000_000);
            assert_eq!(row["wrench_this_step"], tick == 1);
            observed(
                state.position,
                values(row, "position", 3),
                scenario,
                tick,
                "position",
            );
            observed(
                state.orientation,
                values(row, "orientation", 4),
                scenario,
                tick,
                "orientation",
            );
            observed(
                state.velocity,
                values(row, "velocity", 3),
                scenario,
                tick,
                "COM velocity",
            );
            observed(
                state.angular_velocity,
                values(row, "angular_velocity", 3),
                scenario,
                tick,
                "angular velocity",
            );
            observed(
                local,
                values(row, "local_com", 3),
                scenario,
                tick,
                "local COM",
            );
            observed(
                center_position(state, local).iter().copied(),
                values(row, "com_position", 3),
                scenario,
                tick,
                "COM position",
            );
            let origin_v = Vector3::from(state.velocity)
                - Vector3::from(state.angular_velocity)
                    .cross(&(rotation(state.orientation) * Vector3::from(local)));
            observed(
                origin_v.iter().copied(),
                values(row, "origin_velocity", 3),
                scenario,
                tick,
                "origin velocity",
            );
            let world_shapes = row["world_shapes"].as_array().unwrap();
            assert_eq!(world_shapes.len(), 3);
            for (shape, native) in (if scenario == 0 { shapes } else { shifted })
                .into_iter()
                .zip(world_shapes)
            {
                observed(
                    (Vector3::from(state.position)
                        + rotation(state.orientation) * Vector3::from(shape.position))
                    .iter()
                    .copied(),
                    values(native, "position", 3),
                    scenario,
                    tick,
                    "world shape position",
                );
                observed(
                    (rotation(state.orientation) * rotation(shape.orientation))
                        .coords
                        .iter()
                        .copied(),
                    values(native, "orientation", 4),
                    scenario,
                    tick,
                    "world shape orientation",
                );
            }
            if tick == 0 {
                near_values(
                    origin_v.iter().copied(),
                    values(&declared_cases[scenario], "commanded_origin_velocity", 3),
                );
                near_values(
                    values(row, "com_position", 3),
                    values(&cases[1 - scenario]["rows"][0], "com_position", 3),
                );
                near_values(
                    values(row, "velocity", 3),
                    values(&cases[1 - scenario]["rows"][0], "velocity", 3),
                );
                near_values(
                    values(row, "orientation", 4),
                    values(&cases[1 - scenario]["rows"][0], "orientation", 4),
                );
                near_values(
                    values(row, "angular_velocity", 3),
                    values(&cases[1 - scenario]["rows"][0], "angular_velocity", 3),
                );
                for (shape, other) in world_shapes.iter().zip(
                    cases[1 - scenario]["rows"][0]["world_shapes"]
                        .as_array()
                        .unwrap(),
                ) {
                    near_values(values(shape, "position", 3), values(other, "position", 3));
                    near_values(
                        values(shape, "orientation", 4),
                        values(other, "orientation", 4),
                    );
                }
            }
            // The zero-COM representation is also a direct witness that the
            // queued wrench is consumed once, not repeated on later steps.
            if scenario == 1 && tick == 1 {
                near_values(state.velocity, [0.201, -0.098, 0.303]);
                near_values(state.angular_velocity, [0.1002, 0.1999, -0.2997]);
                first_centered_twist = Some((state.velocity, state.angular_velocity));
            } else if scenario == 1 && tick > 1 {
                let (linear, angular) = first_centered_twist.unwrap();
                near_values(state.velocity, linear);
                near_values(state.angular_velocity, angular);
            }
        }
        if tick == 1 {
            let actual = center_position(a.state(first).unwrap(), center).x
                - b.state(second).unwrap().position[0];
            assert!((actual - native_com_delta).abs() <= 1e-12);
        }
    }
}

#[test]
fn collision_response_is_measured_from_com_not_from_body_origin() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let shapes = [-0.1, 0.0, 0.1].map(|x| CollisionSpec {
        position: [250.0 + x, 0.0, 0.0],
        ..SPHERE
    });
    // Origin spatial conditioning is62500<100000, but a response bound that
    // incorrectly measures these shapes from origin would exceed100000.
    let id = spawn(
        &mut backend,
        &shapes,
        [0.0; 3],
        [250.0, 0.0, 0.0],
        [0.0; 3],
        RotationSpec::default(),
    )
    .unwrap();
    let before = backend.state(id).unwrap();
    backend.step(DT).unwrap();
    near(backend.state(id).unwrap(), before);
    assert!(backend.remove(id));
}

#[test]
fn final_shape_com_lever_origin_conditioning_and_origin_velocity_fail_atomically() {
    for case in 0..5 {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        let mut control = DartBackend::new([0.0; 3]).unwrap();
        let existing = spawn(
            &mut backend,
            &[SPHERE],
            [10.0, 0.0, 0.0],
            [0.0; 3],
            [0.0; 3],
            RotationSpec::default(),
        )
        .unwrap();
        let reference = spawn(
            &mut control,
            &[SPHERE],
            [10.0, 0.0, 0.0],
            [0.0; 3],
            [0.0; 3],
            RotationSpec::default(),
        )
        .unwrap();
        for (world, id) in [(&mut backend, existing), (&mut control, reference)] {
            world
                .add_world_wrench(id, [3.0, -2.0, 1.0], [0.1, 0.2, 0.3])
                .unwrap();
        }
        let before = backend.state(existing).unwrap();
        let mut center = [0.0; 3];
        let mut shapes = [SPHERE; 3];
        let mut velocity = [0.0; 3];
        let mut rotational = RotationSpec::default();
        match case {
            0 => center[0] = f64::NAN,
            1 => center[0] = 1e61,
            2 => {
                center[0] = 400.0;
                for shape in &mut shapes {
                    shape.position[0] = 400.0;
                }
            }
            3 => {
                center[0] = -250.0;
                shapes[0].position[0] = -250.0;
                shapes[1].position[0] = -250.0;
            }
            4 => {
                center[0] = 1.0;
                velocity[0] = 6e59;
                rotational.angular_velocity[0] = 6e59;
            }
            _ => unreachable!(),
        }
        assert_eq!(
            spawn(
                &mut backend,
                &shapes,
                [0.0; 3],
                center,
                velocity,
                rotational
            ),
            Err(PhysicsError::InvalidValue),
            "case{case}"
        );
        assert_eq!(backend.state(existing), Some(before));
        let next = spawn(
            &mut backend,
            &[SPHERE],
            [-10.0, 0.0, 0.0],
            [0.0; 3],
            [0.0; 3],
            RotationSpec::default(),
        )
        .unwrap();
        assert_eq!(next.raw(), existing.raw() + 1);
        backend.step(DT).unwrap();
        control.step(DT).unwrap();
        near(
            backend.state(existing).unwrap(),
            control.state(reference).unwrap(),
        );
    }
}

#[test]
fn captured_offset_com_reset_replays_whole_shape_and_twist_state_after_removal() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let center = [0.03, -0.02, 0.05];
    let mut shapes = [SPHERE; 3];
    shapes[0].position = [-0.4, 0.0, 0.0];
    shapes[2].position = [0.4, 0.0, 0.0];
    let id = spawn(
        &mut backend,
        &shapes,
        [0.0, 0.0, 2.0],
        center,
        [0.2, -0.1, 0.3],
        RotationSpec {
            orientation: q([0.2, 0.1, -0.3]),
            angular_velocity: [0.1, 0.2, -0.3],
        },
    )
    .unwrap();
    let initial = backend.initial_state();
    let before = backend.state(id).unwrap();
    let mut observations = Vec::new();
    for _ in 0..10 {
        backend.step(DT).unwrap();
        observations.push(backend.state(id).unwrap());
    }
    let prepared = backend.prepare_initial(&initial).unwrap();
    assert!(backend.remove(id));
    let extra = spawn(
        &mut backend,
        &[SPHERE],
        [20.0, 0.0, 0.0],
        [0.0; 3],
        [0.0; 3],
        RotationSpec::default(),
    )
    .unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(extra), None);
    near(backend.state(id).unwrap(), before);
    for expected in observations {
        backend.step(DT).unwrap();
        near(backend.state(id).unwrap(), expected);
    }
    backend.reset().unwrap();
    near(backend.state(id).unwrap(), before);
    assert!(backend.remove(id));
    let later = spawn(
        &mut backend,
        &shapes,
        [0.0, 0.0, 2.0],
        center,
        [0.0; 3],
        RotationSpec::default(),
    )
    .unwrap();
    assert!(later.raw() > extra.raw());
}
