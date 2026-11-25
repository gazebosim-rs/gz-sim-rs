// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use std::{path::Path, process::Command};

fn run(world: &Path, extra: &[&str]) -> String {
    let output = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(world)
        .args(["--physics-engine", "multifidelity", "--iterations", "1000"])
        .args(extra)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap()
}

#[test]
fn stock_pendulums_use_sdf_default_inertia_about_the_authored_mass_centers() {
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/worlds/upstream_double_pendulum.sdf");
    let stock = run(&source, &[]);
    assert_eq!(
        stock.lines().next(),
        Some("simulation_ns=1000000000 iterations=1000")
    );
    let xml = std::fs::read_to_string(&source).unwrap();
    let offset = "<pose>0 0 0.5 0 0 0</pose>";
    let inertial = format!("<inertial>\n          {offset}");
    assert_eq!(xml.matches(&inertial).count(), 4);
    let tensor = "<inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia>";
    let dir = tempfile::tempdir().unwrap();
    let variant = dir.path().join("variant.sdf");
    std::fs::write(
        &variant,
        xml.replace(&inertial, &format!("{inertial}<mass>1</mass>{tensor}")),
    )
    .unwrap();
    assert_eq!(run(&variant, &[]), stock);

    // Leave every collider and hinge where it was and move only the centers of mass.
    std::fs::write(&variant, xml.replace(&inertial, "<inertial>")).unwrap();
    let centered = run(&variant, &[]);
    for model in ["double_pendulum_with_base0", "double_pendulum_with_base1"] {
        let name = format!("default::{model}::lower_link ");
        let height = |text: &str| {
            text.lines()
                .find_map(|line| line.strip_prefix(&name))
                .unwrap()
                .split_whitespace()
                .nth(2)
                .unwrap()
                .parse::<f64>()
                .unwrap()
        };
        assert!((height(&centered) - height(&stock)).abs() > 0.01);
    }
}

#[test]
fn contact_precision_reduces_base_sink_and_cannot_be_ignored_by_another_engine() {
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/worlds/upstream_double_pendulum.sdf");
    for (args, range) in [
        (&[][..], -0.0013..-0.0010),
        (&["--stiff-static-contacts"][..], -0.0005..0.0),
    ] {
        let state = run(&source, args);
        for line in state.lines().filter(|line| line.contains("::base ")) {
            let z = line
                .split_whitespace()
                .nth(3)
                .unwrap()
                .parse::<f64>()
                .unwrap();
            assert!(range.contains(&z), "base z = {z}, expected {range:?}");
        }
        assert_eq!(
            state
                .lines()
                .filter(|line| line.contains("::base "))
                .count(),
            2
        );
    }
    let refused = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&source)
        .args(["--physics-engine", "rapier", "--stiff-static-contacts"])
        .output()
        .unwrap();
    assert!(!refused.status.success());
    assert!(
        String::from_utf8_lossy(&refused.stderr).contains("require local multi-fidelity physics")
    );
}

#[test]
#[cfg(all(target_os = "linux", target_arch = "x86_64"))]
fn verification_rejects_a_different_contact_response_before_the_first_step() {
    let dir = tempfile::tempdir().unwrap();
    let journal = dir.path().join("journal.json");
    let trace = dir.path().join("tick.jsonl");
    std::fs::write(
        &journal,
        r#"{"seed":"contact-response","commands":[{"op":"step","count":2}]}"#,
    )
    .unwrap();
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/worlds/upstream_double_pendulum.sdf");
    let replay = |flag: &str, extra: &[&str]| {
        Command::new(env!("CARGO_BIN_EXE_gz-sim-local-replay"))
            .arg(&source)
            .arg(&journal)
            .arg(flag)
            .arg(&trace)
            .args(extra)
            .output()
            .unwrap()
    };
    let captured = replay("--trace", &["--stiff-static-contacts"]);
    assert!(captured.status.success(), "{captured:?}");
    let profile: serde_json::Value = serde_json::from_str(
        std::fs::read_to_string(&trace)
            .unwrap()
            .lines()
            .next()
            .unwrap(),
    )
    .unwrap();
    assert_eq!(profile["static_contact_frequency_hz"], 120);
    assert!(
        replay("--verify", &["--stiff-static-contacts", "--threads", "2"])
            .status
            .success()
    );
    let different = replay("--verify", &[]);
    assert!(!different.status.success());
    assert!(
        String::from_utf8_lossy(&different.stderr)
            .contains("physics replay differs at trace line 1")
    );
}

#[test]
#[cfg(all(target_os = "linux", target_arch = "x86_64"))]
fn viewer_lighting_still_rejects_a_ground_visual_too_large_to_encode() {
    let source = include_str!("../../../examples/worlds/upstream_double_pendulum.sdf");
    let (before, after) = source.rsplit_once("<size>100 100</size>").unwrap();
    let dir = tempfile::tempdir().unwrap();
    let world = dir.path().join("oversize-ground.sdf");
    std::fs::write(&world, format!("{before}<size>1e100 100</size>{after}")).unwrap();
    // Only the authored visual changes. The same physical model must still step.
    run(&world, &[]);
    let recording = dir.path().join("recording");
    let output = Command::new(env!("CARGO_BIN_EXE_gz-sim-local-replay"))
        .arg(world)
        .arg(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../examples/replay/upstream_double_pendulum.json"),
        )
        .arg("--record")
        .arg(&recording)
        .arg("--viewer-lighting")
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("invalid plane size"),
        "{output:?}"
    );
    assert!(!recording.join("epoch-0/manifest.json").exists());
}
