// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise the shipped input replay command in independent processes.
#![cfg(all(target_os = "linux", target_arch = "x86_64"))]
use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

const XML: &str = include_str!("../../../examples/worlds/local_robot_replay.sdf");
const JOURNAL: &str = include_str!("../../../examples/replay/local_serial.json");

fn run(world: &Path, journal: &Path, extra: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_gz-sim-local-replay"))
        .arg(world)
        .arg(journal)
        .args(extra)
        .output()
        .unwrap()
}
fn success(output: Output) -> Vec<Value> {
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout)
        .unwrap()
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect()
}
fn file(dir: &Path, name: &str, content: &str) -> PathBuf {
    let path = dir.join(name);
    fs::write(&path, content).unwrap();
    path
}
fn altered(dir: &Path, base: &Value, name: &str, change: impl FnOnce(&mut Value)) -> PathBuf {
    let mut journal = base.clone();
    change(&mut journal);
    file(dir, name, &journal.to_string())
}
fn record(rows: &[Value], command: u64, step: u64) -> &Value {
    rows.iter()
        .find(|v| v["command"] == command && v["step"] == step)
        .unwrap()
}
fn shared(a: &Value, b: &Value, keys: &[&str]) {
    for key in keys {
        assert_eq!(a[*key], b[*key], "{key}");
    }
}
const STATE: &[&str] = &["scene", "physics", "contacts", "inputs", "imu"];

#[test]
fn independent_processes_reconstruct_lifecycle_warm_contact_and_pending_inputs() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let xml = file(dir, "robot.sdf", XML);
    let source = file(dir, "journal.json", JOURNAL);
    let trace = dir.join("trace.jsonl");
    let trace = trace.to_str().unwrap();
    let archive = dir.join("recording");
    let first = run(
        &xml,
        &source,
        &[
            "--trace",
            trace,
            "--threads",
            "1",
            "--record",
            archive.to_str().unwrap(),
        ],
    );
    assert!(String::from_utf8_lossy(&first.stderr).contains("1 threads"));
    success(first);
    let check = run(&xml, &source, &["--verify", trace, "--threads", "8"]);
    assert!(String::from_utf8_lossy(&check.stderr).contains("8 threads"));
    success(check);
    let rows: Vec<Value> = fs::read_to_string(trace)
        .unwrap()
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    assert_eq!(rows[0]["profile"], "local-cpu-v1");
    assert_eq!(rows[0]["binary_sha256"].as_str().unwrap().len(), 64);
    assert_eq!(rows[0]["step_ns"], 1_000_000);
    let settled = record(&rows, 2, 180);
    let queued = record(&rows, 3, 180);
    let paused = record(&rows, 4, 180);
    assert_eq!(settled["pending_wrenches"], 0);
    assert_eq!(paused["pending_wrenches"], 1);
    assert_ne!(settled["inputs"], queued["inputs"]);
    shared(settled, queued, &["scene", "physics", "contacts", "imu"]);
    shared(queued, paused, STATE);
    assert!(paused["contact_points"].as_u64().unwrap() > 0);
    assert!(paused["stored_normal_ns"].as_f64().unwrap() > 0.);
    assert!(paused["delivered_normal_ns"].as_f64().unwrap() > 0.);
    assert_eq!(paused["imu_samples"], 1);
    assert_eq!(paused["held_targets"], 2);
    let retargeted = record(&rows, 5, 180);
    assert_ne!(retargeted["inputs"], paused["inputs"]);
    shared(retargeted, paused, &["physics", "contacts", "imu"]);
    let resumed = record(&rows, 6, 181);
    assert_eq!(resumed["pending_wrenches"], 0);
    let spawn = record(&rows, 7, 200);
    assert_eq!(
        spawn["entities"].as_u64().unwrap(),
        settled["entities"].as_u64().unwrap() + 4
    );
    let loads = |row: &Value| -> std::collections::BTreeMap<(u64, u64), f64> {
        row["pair_reactions_ns"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| {
                (
                    (p[0].as_u64().unwrap(), p[1].as_u64().unwrap()),
                    p[2].as_f64().unwrap(),
                )
            })
            .collect()
    };
    let landed = loads(record(&rows, 8, 380));
    let removed = loads(record(&rows, 9, 380));
    let retired: Vec<f64> = landed
        .iter()
        .filter_map(|(pair, load)| (!removed.contains_key(pair)).then_some(*load))
        .collect();
    assert_eq!(retired.len(), 1);
    // The new free parcel settles on its own part of the tray before removal.
    assert!((retired[0] / 0.001 - 0.2 * 9.81).abs() < 0.02);
    assert_eq!(record(&rows, 9, 380)["entities"], settled["entities"]);
    let reset = record(&rows, 13, 0);
    assert_eq!(reset["epoch"], 1);
    assert_eq!(reset["entities"], settled["entities"]);
    assert_eq!(reset["pending_wrenches"], 0);
    assert_eq!(reset["stored_normal_ns"], 0.);
    assert_eq!(reset["imu_samples"], 0);
    shared(reset, record(&rows, 0, 0), STATE);
    for (before, after, steps) in [(2, 15, 1..181), (6, 19, 181..201)] {
        for step in steps {
            shared(
                record(&rows, before, step),
                record(&rows, after, step),
                STATE,
            );
        }
    }
    for epoch in 0..2 {
        let m: Value = serde_json::from_slice(
            &fs::read(archive.join(format!("epoch-{epoch}/manifest.json"))).unwrap(),
        )
        .unwrap();
        assert_eq!(m["complete"], true);
        assert!(m["frames"].as_u64().unwrap() > 10);
    }
    let base: Value = serde_json::from_str(JOURNAL).unwrap();
    let sham = altered(dir, &base, "sham.json", |j| {
        j["commands"][2]["force"] = json!([0, 0, 0]);
        j["commands"][2]["torque"] = json!([0, 0, 0]);
    });
    let control = success(run(&xml, &sham, &[]));
    shared(
        record(&control, 4, 180),
        paused,
        &["scene", "physics", "contacts", "imu"],
    );
    assert_ne!(record(&control, 4, 180)["inputs"], paused["inputs"]);
    assert_ne!(record(&control, 6, 181)["physics"], resumed["physics"]);
    assert_ne!(record(&control, 6, 181)["contacts"], resumed["contacts"]);
    let seed = altered(dir, &base, "seed.json", |j| {
        j["seed"] = json!("independent instrument")
    });
    let independent = success(run(&xml, &seed, &[]));
    for (a, b) in rows.iter().zip(&independent).skip(1) {
        shared(a, b, &["scene", "physics", "contacts", "inputs"]);
    }
    assert_ne!(record(&independent, 2, 180)["imu"], settled["imu"]);
    let mut buf = fs::read_to_string(trace).unwrap();
    buf.push_str("extra\n");
    let long = file(dir, "long", &buf);
    assert!(
        String::from_utf8_lossy(&run(&xml, &source, &["--verify", long.to_str().unwrap()]).stderr)
            .contains("extra data")
    );
    let truncated = file(
        dir,
        "short",
        &buf.lines().take(3).collect::<Vec<_>>().join("\n"),
    );
    assert!(
        String::from_utf8_lossy(
            &run(&xml, &source, &["--verify", truncated.to_str().unwrap()]).stderr
        )
        .contains("line 4")
    );
}

#[test]
fn unsupported_profiles_and_invalid_lifecycle_requests_fail() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let xml = file(dir, "robot.sdf", XML);
    let journal = file(dir, "j", JOURNAL);
    let original = include_str!("../../../examples/worlds/local_robot_serial_transfer.sdf");
    let plain = file(dir, "original", original);
    let accepted = file(
        dir,
        "empty",
        r#"{"seed":"dry","commands":[{"op":"step","count":2}]}"#,
    );
    assert_eq!(success(run(&plain, &accepted, &[])).len(), 4);
    for (s, err) in [
        (
            "<sdf version='1.12'><model name='loose'><link name='body'/></model></sdf>",
            "exactly one",
        ),
        (
            r#"<sdf version="1.12"><world name="default"><model name="box"><link name="body"><sensor name="field" type="magnetometer"><magnetometer/></sensor></link></model></world></sdf>"#,
            "admits inline",
        ),
    ] {
        let world = file(dir, "unsupported.sdf", s);
        let output = run(&world, &journal, &[]);
        assert!(!output.status.success());
        assert!(
            String::from_utf8_lossy(&output.stderr).contains(err),
            "{:?}",
            output
        );
    }
    let spawn =
        json!({"op":"spawn","template":"default::parcel","alias":"copy","position":[0.1,0.1,0.55]});
    for (ops, err) in [
        (
            json!([{"op":"target","joint":"missing","value":1}]),
            "unknown scoped",
        ),
        (
            json!([{"op":"spawn","template":"missing","alias":"a","position":[0,0,0]}]),
            "unknown startup",
        ),
        (json!([spawn, spawn]), "duplicate live"),
        (json!([{"op":"remove","alias":"absent"}]), "unknown live"),
        (
            json!([{"op":"wrench","link":"default::gripper::palm","force":[0,0,1],"torque":[0,0,0]}]),
            "only on ordinary free links",
        ),
    ] {
        let j = file(
            dir,
            "bad",
            &json!({"seed":"test","commands":ops}).to_string(),
        );
        let out = run(&xml, &j, &[]);
        assert!(!out.status.success());
        assert!(
            String::from_utf8_lossy(&out.stderr).contains(err),
            "{:?}",
            out
        );
    }
}

#[test]
fn report_corrupt_inputs_missing_files_and_capture_the_last_unsampled_step() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let world = file(dir, "robot.sdf", XML);
    let journal = file(
        dir,
        "input.json",
        r#"{"seed":"a","commands":[{"op":"step","count":3}]}"#,
    );
    let archive = dir.join("run");
    success(run(
        &world,
        &journal,
        &["--record", archive.to_str().unwrap()],
    ));
    let frames = fs::read_to_string(archive.join("epoch-0/frames.jsonl")).unwrap();
    let projected: Vec<Value> = frames
        .lines()
        .map(|s| serde_json::from_str::<Value>(s).unwrap())
        .filter(|v| v["kind"] == "frame")
        .map(|v| serde_json::from_str(v["scene"].as_str().unwrap()).unwrap())
        .collect();
    assert_eq!(projected.len(), 2);
    assert_eq!(projected[1]["nanoseconds"], 3_000_000);
    for args in [
        vec!["--record", archive.to_str().unwrap()],
        vec!["--trace", world.to_str().unwrap()],
        vec!["--verify", dir.join("missing").to_str().unwrap()],
    ] {
        assert!(!run(&world, &journal, &args).status.success());
    }
    let missing = dir.join("missing");
    let broken = file(dir, "broken", "{");
    assert!(!run(&missing, &journal, &[]).status.success());
    assert!(!run(&world, &missing, &[]).status.success());
    assert!(!run(&world, &broken, &[]).status.success());
    assert!(!run(&broken, &journal, &[]).status.success());
    let invalid_noise = file(
        dir,
        "bad_noise",
        &XML.replace("<stddev>0.015</stddev>", "<stddev>1e200</stddev>"),
    );
    assert!(!run(&invalid_noise, &journal, &[]).status.success());
    let bad_ops = [
        json!([{"op":"target","joint":"default::parcel::box","value":0.1}]),
        json!([{"op":"step","count":1},{"op":"step","count":u64::MAX}]),
        json!([{"op":"spawn","template":"default::gripper","alias":"robot","position":[0,0,0]}]),
        json!([{"op":"spawn","template":"default::parcel","alias":"copy","position":[1e200,0,0]}]),
    ];
    for commands in bad_ops {
        let j = file(
            dir,
            "bad_cmd",
            &json!({"seed":"a","commands":commands}).to_string(),
        );
        assert!(!run(&world, &j, &[]).status.success(), "{commands}");
    }
}

#[test]
fn spherical_wrist_commands_accumulate_survive_pause_and_replay_after_reset() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let xml = file(
        dir,
        "wrist.sdf",
        include_str!("../../../examples/worlds/gravity_wrist.sdf"),
    );
    let journal = file(
        dir,
        "wrist.json",
        include_str!("../../../examples/replay/ball_wrist.json"),
    );
    let rows = success(run(&xml, &journal, &["--threads", "1"]));
    assert_eq!(rows, success(run(&xml, &journal, &["--threads", "8"])));
    let queued = record(&rows, 2, 0);
    let paused = record(&rows, 3, 0);
    shared(queued, paused, STATE);
    let wrists = paused["wrists"].as_array().unwrap();
    let held = wrists
        .iter()
        .position(|j| !j["pending_torque_nm"].is_null())
        .unwrap();
    let free = 1 - held;
    assert_eq!(wrists.len(), 2);
    assert_eq!(wrists[held]["pending_torque_nm"], json!([18., -24., 12.]));
    assert_ne!(record(&rows, 1, 0)["inputs"], paused["inputs"]);
    assert_eq!(
        record(&rows, 7, 500)["wrists"][held]["pending_torque_nm"],
        json!([0., 0., 0.])
    );
    let reset = record(&rows, 9, 0);
    assert!(reset["wrists"][held]["pending_torque_nm"].is_null());
    for step in 1..=240 {
        let before = record(&rows, 4, step);
        let replayed = record(&rows, 13, step);
        assert!(before["wrists"][held]["pending_torque_nm"].is_null());
        shared(
            before,
            replayed,
            &["wrists", "physics", "contacts", "inputs", "scene"],
        );
    }
    let first = &record(&rows, 4, 1)["wrists"];
    assert_ne!(
        first[held]["angular_velocity_rad_s"],
        first[free]["angular_velocity_rad_s"]
    );
    let miss = file(
        dir,
        "wrong-kind",
        r#"{"seed":"w","commands":[{"op":"ball_torque","joint":"default::held::clamp","torque_nm":[1,2,3]}]}"#,
    );
    assert!(!run(&xml, &miss, &[]).status.success());
}

#[test]
fn spindle_and_universal_inputs_replay_on_the_moving_wrist() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let xml = file(
        dir,
        "wrist.sdf",
        include_str!("../../../examples/worlds/local_robot_screw_wrist.sdf"),
    );
    let json = include_str!("../../../examples/replay/screw_universal.json");
    let journal = file(dir, "axis.json", json);
    let rows = success(run(&xml, &journal, &["--threads", "1"]));
    assert_eq!(rows, success(run(&xml, &journal, &["--threads", "8"])));
    let queued = record(&rows, 3, 0);
    shared(queued, record(&rows, 4, 0), STATE);
    assert_eq!(queued["scalar_joints"][0]["pending_effort"], 2.);
    assert_eq!(
        queued["universal_joints"][0]["pending_effort"],
        json!([0.8, -1.25])
    );
    let reset = record(&rows, 9, 0);
    for key in ["scalar_joints", "universal_joints"] {
        assert!(reset[key][0]["pending_effort"].is_null());
        assert!(record(&rows, 5, 1)[key][0]["pending_effort"].is_null());
    }
    assert_eq!(
        record(&rows, 8, 180)["scalar_joints"][0]["pending_effort"],
        0.
    );
    assert_eq!(
        record(&rows, 8, 180)["universal_joints"][0]["pending_effort"],
        json!([0., 0.])
    );
    for tick in 1..=180 {
        shared(
            record(&rows, 5, tick),
            record(&rows, 14, tick),
            &[
                "physics",
                "inputs",
                "scene",
                "contacts",
                "scalar_joints",
                "universal_joints",
            ],
        );
    }

    let mut control: Value = serde_json::from_str(json).unwrap();
    for c in control["commands"].as_array_mut().unwrap() {
        match c["op"].as_str().unwrap() {
            "universal_torque" => c["torque_nm"] = json!([0., 0.]),
            "joint_effort" => c["effort"] = json!(0.),
            _ => {}
        }
    }
    let baseline = file(dir, "unloaded.json", &control.to_string());
    let free = success(run(&xml, &baseline, &[]));
    let first = record(&rows, 5, 1);
    let passive = record(&free, 5, 1);
    assert_ne!(
        first["scalar_joints"][0]["velocity"],
        passive["scalar_joints"][0]["velocity"]
    );
    for i in 0..2 {
        assert_ne!(
            first["universal_joints"][0]["velocity"][i],
            passive["universal_joints"][0]["velocity"][i]
        );
    }
    for command in [
        json!({"op":"joint_effort","joint":"default::robot::wrist","effort":1.}),
        json!({"op":"universal_torque","joint":"default::robot::spindle","torque_nm":[0.,1.]}),
        json!({"op":"universal_torque","joint":"default::robot::wrist","torque_nm":[0.,3.]}),
    ] {
        let path = file(
            dir,
            "invalid.json",
            &json!({"seed":"invalid","commands":[command]}).to_string(),
        );
        assert!(!run(&xml, &path, &[]).status.success());
    }
}

#[test]
fn tilted_curved_shapes_hit_at_their_own_support_height_and_bundle_their_surfaces() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path();
    let xml = file(
        dir,
        "curves.sdf",
        include_str!("../../../examples/worlds/local_curved_drop.sdf"),
    );
    let commands = file(
        dir,
        "drop.json",
        include_str!("../../../examples/replay/curved_drop.json"),
    );
    let archive = dir.join("capture");
    let rows = success(run(
        &xml,
        &commands,
        &["--record", archive.to_str().unwrap()],
    ));
    assert_eq!(rows, success(run(&xml, &commands, &["--threads", "8"])));
    let mut first = std::collections::BTreeMap::new();
    for tick in 1..=600 {
        let state = record(&rows, 1, tick);
        shared(state, record(&rows, 3, tick), STATE);
        for pair in state["pair_reactions_ns"].as_array().unwrap() {
            if pair[2].as_f64().unwrap() > 0. {
                first
                    .entry((pair[0].as_u64().unwrap(), pair[1].as_u64().unwrap()))
                    .or_insert(tick);
            }
        }
    }
    // Both centers start at 320 mm and both shapes have 45 mm radius
    // and 210 mm overall axial length. The capsule's straight section is 120 mm.
    let tilt = std::f64::consts::PI / 6.;
    let height = [
        0.105 * tilt.cos() + 0.045 * tilt.sin(),
        0.06 * tilt.cos() + 0.045,
    ];
    assert_eq!(first.len(), 2);
    for (h, tick) in height.into_iter().zip(first.values()) {
        let ballistic_ms = 1000. * (2. * (0.32 - h) / 9.81).sqrt();
        assert!((*tick as f64 - ballistic_ms).abs() < 1.);
    }
    for pair in record(&rows, 1, 600)["pair_reactions_ns"]
        .as_array()
        .unwrap()
    {
        assert!((pair[2].as_f64().unwrap() * 1000. - 0.25 * 9.81).abs() < 0.01);
    }
    let manifest = |epoch| -> Value {
        serde_json::from_slice(
            &fs::read(archive.join(format!("epoch-{epoch}/manifest.json"))).unwrap(),
        )
        .unwrap()
    };
    let initial = manifest(0);
    assert_eq!(initial["assets"], manifest(1)["assets"]);
    let assets = initial["assets"].as_array().unwrap();
    assert_eq!(assets.len(), 2);
    for entry in assets {
        let route = entry["route"].as_str().unwrap().trim_start_matches('/');
        let bytes = fs::read(archive.join("epoch-0").join(route)).unwrap();
        assert_eq!(&bytes[..4], b"glTF");
        assert_eq!(
            bytes,
            fs::read(archive.join("epoch-1").join(route)).unwrap()
        );
    }
}
