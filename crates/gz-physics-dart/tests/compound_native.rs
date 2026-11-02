// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reads only actual C++ observations; the fixture is materialized after review.
use gz_physics::{
    BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia, PlaneSpec, RotationSpec,
    Shape,
};
use gz_physics_dart::DartBackend;
use nalgebra::UnitQuaternion;
use serde_json::Value;
use std::{
    collections::BTreeSet,
    io::Write,
    process::{Command, Stdio},
    time::Duration,
};

const FIXTURE: &str = include_str!("fixtures/compound_collisions.json");
const GENERATOR: &[u8] = include_bytes!("fixtures/compound_collisions.cc");
const PROVENANCE: &[u8] = include_bytes!("../../../reference/compound_collisions.json");
const TICKS: [u64; 5] = [0, 1, 10, 100, 600];
const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
const EPSILON: f64 = 1e-12;

fn keys(value: &Value, expected: &[&str]) {
    let actual: BTreeSet<_> = value
        .as_object()
        .unwrap()
        .keys()
        .map(String::as_str)
        .collect();
    assert_eq!(actual, expected.iter().copied().collect());
}

fn checksum(bytes: &[u8]) -> String {
    let mut child = Command::new("sha256sum")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child.stdin.take().unwrap().write_all(bytes).unwrap();
    let result = child.wait_with_output().unwrap();
    assert!(result.status.success());
    assert!(result.stderr.is_empty());
    let output = String::from_utf8(result.stdout).unwrap();
    output.split_whitespace().next().unwrap().to_owned()
}

fn hash(value: &Value) -> &str {
    let hash = value.as_str().unwrap();
    assert_eq!(hash.len(), 64);
    assert!(
        hash.bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    );
    hash
}

fn quaternion(rpy: [f64; 3]) -> [f64; 4] {
    let q = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
    [q.i, q.j, q.k, q.w]
}

fn collisions(count: usize) -> Vec<CollisionSpec> {
    let sphere = CollisionSpec {
        shape: Shape::Sphere { radius: 0.2 },
        position: [0.0; 3],
        orientation: IDENTITY,
    };
    if count == 1 {
        return vec![sphere];
    }
    let mut values = vec![
        CollisionSpec {
            shape: Shape::Box {
                half_extents: [0.3, 0.2, 0.4],
            },
            position: [-0.4, 0.0, 0.0],
            orientation: IDENTITY,
        },
        CollisionSpec {
            shape: Shape::Cylinder {
                radius: 0.2,
                half_height: 0.5,
            },
            position: [if count == 2 { 0.4 } else { 0.0 }, 0.0, 0.0],
            // Preserve the SDF fixture's decimal angle, rather than snapping it to pi/2.
            orientation: quaternion([0.0, "1.5708".parse().unwrap(), 0.3]),
        },
    ];
    if count == 3 {
        values.push(CollisionSpec {
            shape: Shape::Capsule {
                radius: 0.2,
                half_length: 0.4,
            },
            position: [0.4, 0.0, 0.0],
            orientation: quaternion([0.2, 0.4, -0.1]),
        });
    }
    values
}

fn xml(count: usize, contact: bool) -> String {
    let collision = |index: usize, pose: &str, geometry: &str| {
        format!(
            "<collision name='c{index}'><pose>{pose}</pose><geometry>{geometry}</geometry></collision>"
        )
    };
    let sphere = "<sphere><radius>0.2</radius></sphere>";
    let box_shape = "<box><size>0.6 0.4 0.8</size></box>";
    let cylinder = "<cylinder><radius>0.2</radius><length>1</length></cylinder>";
    let capsule = "<capsule><radius>0.2</radius><length>0.8</length></capsule>";
    let shapes = match count {
        1 => collision(0, "0 0 0 0 0 0", sphere),
        2 => {
            collision(0, "-0.4 0 0 0 0 0", box_shape)
                + &collision(1, "0.4 0 0 0 1.5708 0.3", cylinder)
        }
        3 => {
            collision(0, "-0.4 0 0 0 0 0", box_shape)
                + &collision(1, "0 0 0 0 1.5708 0.3", cylinder)
                + &collision(2, "0.4 0 0 0.2 0.4 -0.1", capsule)
        }
        _ => panic!("unsupported fixture collision count"),
    };
    let floor = if contact {
        "<model name='floor'><static>true</static><link name='body'><collision name='plane'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>"
    } else {
        ""
    };
    format!(
        "<sdf version='1.11'><world name='w'><gravity>0 0 {}</gravity>{floor}<model name='compound'><link name='body'><pose>0 0 {} 0.2 0.1 -0.3</pose><inertial><pose>0 0 0 0.1 0.2 0.3</pose><mass>2</mass><inertia><ixx>1</ixx><iyy>1.5</iyy><izz>2</izz></inertia></inertial>{shapes}</link></model></world></sdf>",
        if contact { "-9.81" } else { "0" },
        if contact { "1" } else { "2" },
    )
}

fn vector(row: &Value, key: &str, length: usize) -> Vec<f64> {
    let values = row[key].as_array().unwrap();
    assert_eq!(values.len(), length, "{key}");
    values
        .iter()
        .map(|value| {
            let value = value.as_f64().unwrap();
            assert!(value.is_finite(), "{key}");
            value
        })
        .collect()
}

fn validate_row(row: &Value, scenario: u64, count: u64, contact: bool, tick: u64) {
    keys(
        row,
        &[
            "scenario",
            "count",
            "contact",
            "tick",
            "position",
            "orientation",
            "velocity",
            "angular_velocity",
        ],
    );
    assert_eq!(row["scenario"].as_u64(), Some(scenario));
    assert_eq!(row["count"].as_u64(), Some(count));
    assert_eq!(row["contact"].as_u64(), Some(u64::from(contact)));
    assert_eq!(row["tick"].as_u64(), Some(tick));
    assert!(vector(row, "position", 3).iter().all(|v| v.abs() <= 5e8));
    for key in ["velocity", "angular_velocity"] {
        assert!(vector(row, key, 3).iter().all(|v| v.abs() <= 1e60));
    }
    let norm: f64 = vector(row, "orientation", 4).iter().map(|v| v * v).sum();
    assert!((norm - 1.0).abs() <= EPSILON);
}

fn compare(state: BodyState, row: &Value) {
    let actual: Vec<_> = state
        .position
        .into_iter()
        .chain(state.orientation)
        .chain(state.velocity)
        .chain(state.angular_velocity)
        .collect();
    let expected: Vec<_> = vector(row, "position", 3)
        .into_iter()
        .chain(vector(row, "orientation", 4))
        .chain(vector(row, "velocity", 3))
        .chain(vector(row, "angular_velocity", 3))
        .collect();
    assert_eq!(actual.len(), 13);
    assert_eq!(expected.len(), 13);
    for (field, (actual, expected)) in actual.into_iter().zip(expected).enumerate() {
        assert!(
            (actual - expected).abs() <= EPSILON,
            "scenario{} tick{} field{field}: Rust{actual:.17e}, native{expected:.17e}",
            row["scenario"],
            row["tick"]
        );
    }
}

#[test]
fn six_actual_native_inputs_match_all_thirty_complete_body_states() {
    let fixture: Value = serde_json::from_str(FIXTURE).unwrap();
    keys(
        &fixture,
        &[
            "schema_version",
            "copyright",
            "license",
            "scope",
            "generator_sha256",
            "capture",
            "cases",
        ],
    );
    assert_eq!(fixture["schema_version"].as_u64(), Some(1));
    assert_eq!(
        fixture["copyright"],
        "Copyright (C) 2026 Open Source Robotics Foundation"
    );
    assert_eq!(fixture["license"], "Apache-2.0");
    assert_eq!(
        fixture["scope"],
        "Native DART compound-link reference; no B03 acceptance."
    );
    assert_eq!(hash(&fixture["generator_sha256"]), checksum(GENERATOR));
    let capture = &fixture["capture"];
    keys(
        capture,
        &[
            "provenance_sha256",
            "stdout_sha256",
            "stderr_sha256",
            "maps_sha256",
        ],
    );
    for key in ["provenance_sha256", "stdout_sha256", "stderr_sha256"] {
        hash(&capture[key]);
    }
    assert_eq!(hash(&capture["provenance_sha256"]), checksum(PROVENANCE));
    assert_eq!(hash(&capture["stderr_sha256"]), checksum(b""));
    keys(&capture["maps_sha256"], &["before", "after"]);
    for key in ["before", "after"] {
        hash(&capture["maps_sha256"][key]);
    }
    let cases = fixture["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 6);
    let mut observations = 0;
    for (index, case) in cases.iter().enumerate() {
        let count = index / 2 + 1;
        let contact = index % 2 == 1;
        keys(
            case,
            &["scenario", "count", "contact", "xml", "xml_sha256", "rows"],
        );
        assert_eq!(case["scenario"].as_u64(), Some(index as u64));
        assert_eq!(case["count"].as_u64(), Some(count as u64));
        assert_eq!(case["contact"].as_u64(), Some(u64::from(contact)));
        let input = xml(count, contact);
        assert_eq!(case["xml"].as_str(), Some(input.as_str()));
        assert_eq!(hash(&case["xml_sha256"]), checksum(input.as_bytes()));
        let rows = case["rows"].as_array().unwrap();
        assert_eq!(rows.len(), TICKS.len());
        for (row, tick) in rows.iter().zip(TICKS) {
            validate_row(row, index as u64, count as u64, contact, tick);
        }
        if contact {
            let ballistic_z = 1.0 - 9.81 * 0.001_f64.powi(2) * 600.0 * 601.0 / 2.0;
            assert!(vector(&rows[4], "position", 3)[2] > ballistic_z + 0.01);
            assert!(vector(&rows[4], "velocity", 3)[2] > -9.81 * 0.6 + 0.01);
        }
        let mut backend = DartBackend::new([0.0, 0.0, if contact { -9.81 } else { 0.0 }]).unwrap();
        let floor = contact.then(|| {
            backend
                .spawn_static_plane(PlaneSpec {
                    position: [0.0; 3],
                    orientation: IDENTITY,
                    size: [100.0; 2],
                })
                .unwrap()
        });
        let floor_state = floor.map(|id| backend.state(id).unwrap());
        let shapes = collisions(count);
        assert_eq!(shapes.len(), count);
        let body = backend
            .spawn_compound_link(
                CompoundBodySpec {
                    collisions: &shapes,
                    position: [0.0, 0.0, if contact { 1.0 } else { 2.0 }],
                    velocity: [0.0; 3],
                    mass: Some(2.0),
                },
                RotationSpec {
                    orientation: quaternion([0.2, 0.1, -0.3]),
                    ..Default::default()
                },
                Inertia {
                    center: [0.0; 3],
                    orientation: quaternion([0.1, 0.2, 0.3]),
                    tensor: [1.0, 1.5, 2.0, 0.0, 0.0, 0.0],
                },
            )
            .unwrap();
        if !contact {
            backend
                .set_world_velocity(body, [0.2, -0.1, 0.3], [0.1, 0.2, -0.3])
                .unwrap();
        }
        let mut cursor = 0;
        for tick in 0..=600 {
            // Both implementations receive exactly600 independent1ms steps,
            // the same gravity and no applied external force or joint command.
            if tick != 0 {
                backend.step(Duration::from_millis(1)).unwrap();
            }
            if cursor < TICKS.len() && tick == TICKS[cursor] {
                compare(backend.state(body).unwrap(), &rows[cursor]);
                cursor += 1;
                observations += 1;
            }
        }
        assert_eq!(cursor, 5);
        if let Some(id) = floor {
            assert_eq!(backend.state(id), floor_state);
        }
    }
    assert_eq!(observations, 30);
}
