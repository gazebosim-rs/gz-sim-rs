// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::floating_tree::crash_tests::terminate_probe;
use std::{error::Error, fs, process::Command};
type Test<T = ()> = std::result::Result<T, Box<dyn Error>>;
const CABLE: &str = include_str!("../../../../../examples/articulated_transient_cable.sdf");
const CLOTH: &str = include_str!("../../../../../examples/articulated_first_impact.sdf");
const CHILD: &str = "floating_tree::automatic::crash_tests::automatic_controller_process";
pub(super) fn cold(label: &str) -> Test<SoftBody> {
    let xml = if label == "cloth" {
        CLOTH
            .replace("0.001</max_step_size>", "0.01</max_step_size>")
            .replace("friction=\"0.9\"", "friction=\"0\"")
    } else {
        CABLE.to_owned()
    };
    Ok(gz_sim_scene::deformable::articulated_impact::distributed_automatic_material_contact_tree_spec(&xml,"automatic.sdf","default")?.material)
}
fn operation(p: &mut AutomaticPool<impl AutomaticConstruction>, op: &str) -> Result<()> {
    match op {
        "step" => p.advance_at(p.epoch(), p.tick()),
        "transfer" => p.transfer((p.owner() + 1) % p.width()),
        "reset" => p.reset(),
        _ => panic!("unknown automatic recovery operation"),
    }
}
#[test]
#[ignore = "controller subprocess, drives a real private geometry, impulse or commit boundary"]
fn automatic_controller_process() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let action = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let (label, op) = action.split_once('-').unwrap();
    fn run<S: AutomaticConstruction>(exe: &Path, dir: &Path, body: SoftBody, op: &str) -> Test {
        let mut p = AutomaticPool::<S>::resume(exe, dir, body)?;
        let pid = p.worker_pid(p.owner()).unwrap();
        operation(&mut p, op)?;
        assert_ne!(
            pid,
            p.worker_pid(p.owner()).unwrap(),
            "controller passed its interlock without a worker loss"
        );
        Ok(())
    }
    if matches!(label, "rotor" | "membrane") {
        return run::<AutomaticRotorMaterialSpec>(
            Path::new(&exe),
            Path::new(&dir),
            rotor_body(label)?,
            op,
        );
    }
    run::<AutomaticMaterialSpec>(Path::new(&exe), Path::new(&dir), cold(label)?, op)?;
    Ok(())
}
#[test]
#[ignore = "actual controller and worker SIGKILLs on ordinary-validated off-grid material cuts"]
fn verified_automatic_subintervals_survive_partial_head_impulse_tail_transfer_and_reset() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let root = std::env::var("GZ_AUTOMATIC_WITNESS")?;
    for label in ["cable", "cloth"] {
        let body = cold(label)?;
        let dir = Path::new(&root).join(label);
        audit::<AutomaticMaterialSpec>(exe, &dir, label, body, "gzautmaterial")?;
    }
    Ok(())
}
fn rotor_body(label: &str) -> Test<SoftBody> {
    let xml = match label {
        "rotor" => include_str!("../../../../../examples/rotating_pad_cable.sdf").to_owned(),
        "membrane" => include_str!("../../../../../examples/distributed_rotating_pad_cloth.sdf")
            .replace("0.011</max_step_size>", "0.20</max_step_size>"),
        _ => return Err("unknown rotating material fixture".into()),
    };
    Ok(
        gz_sim_scene::deformable::articulated_impact::distributed_rotor_material_contact_spec(
            &xml,
            "rotor.sdf",
            "default",
        )?
        .material,
    )
}
#[test]
#[ignore = "requires checked rotor material snapshots and real process termination"]
fn rotating_material_private_head_impulse_tail_and_recovery() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_ROTOR_WITNESS")?;
    audit::<AutomaticRotorMaterialSpec>(
        Path::new(&exe),
        Path::new(&dir),
        "rotor",
        rotor_body("rotor")?,
        "gzrotmaterial",
    )
}
#[test]
#[ignore = "requires checked four-node face snapshots and real process termination"]
fn rotating_membrane_keeps_the_three_weighted_impulses_and_elastic_tail_exactly_once() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_ROTOR_CLOTH_WITNESS")?;
    audit::<AutomaticRotorMaterialSpec>(
        Path::new(&exe),
        Path::new(&dir),
        "membrane",
        rotor_body("membrane")?,
        "gzrotmaterial",
    )
}
fn audit<S: AutomaticConstruction>(
    exe: &Path,
    dir: &Path,
    label: &str,
    body: SoftBody,
    extension: &str,
) -> Test {
    let prior = fs::read(dir.join(format!("prior.{extension}")))?;
    let after = fs::read(dir.join(format!("accepted.{extension}")))?;
    for (action, original, edges) in [
        (
            "step",
            &prior,
            &[
                "automatic-native-head",
                "automatic-material-head",
                "automatic-impulses-prepared",
                "automatic-native-impulses",
                "automatic-material-impulses",
                "automatic-native-tail",
                "automatic-material-tail",
                "automatic-complete-proposal",
                "before-rename",
                "after-rename",
                "after-sync",
                "GZ_AUTOMATIC_HEAD_KILL_ONCE",
                "GZ_AUTOMATIC_IMPULSE_KILL_ONCE",
            ][..],
        ),
        (
            "transfer",
            &after,
            &[
                "automatic-destination-staged",
                "automatic-source-fenced",
                "automatic-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &after,
            &[
                "automatic-destination-staged",
                "automatic-source-fenced",
                "automatic-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
    ] {
        for &edge in edges {
            let broken = tempfile::tempdir()?;
            let control = tempfile::tempdir()?;
            let file = broken.path().join(S::FILE);
            let reference = control.path().join(S::FILE);
            fs::write(&file, original)?;
            fs::write(&reference, original)?;
            let mut direct = AutomaticPool::<S>::resume(exe, control.path(), body.clone())?;
            let command = format!("{label}-{action}");
            let killed_worker = edge.starts_with("GZ_");
            if killed_worker {
                let marker = broken.path().join("native-killed");
                assert!(
                    Command::new(std::env::current_exe()?)
                        .args(["--exact", "--ignored", CHILD])
                        .env("GZ_FLOATING_TREE_TEST_SESSION", broken.path())
                        .env("GZ_FLOATING_TREE_TEST_ACTION", &command)
                        .env(edge, &marker)
                        .status()?
                        .success()
                );
                assert!(marker.is_file());
            } else {
                terminate_probe(CHILD, broken.path(), &command, edge, direct.tick())?;
            }
            let accepted = killed_worker || matches!(edge, "after-rename" | "after-sync");
            if accepted {
                operation(&mut direct, action)?;
            }
            assert_eq!(fs::read(&file)?, fs::read(&reference)?);
            let mut recovered = AutomaticPool::<S>::resume(exe, broken.path(), body.clone())?;
            if !accepted {
                operation(&mut recovered, action)?;
                operation(&mut direct, action)?;
            }
            if action == "step" {
                assert_eq!(fs::read(&file)?, after);
            }
            for _ in 0..32 {
                direct.advance()?;
                recovered.advance()?;
                assert_eq!(fs::read(&file)?, fs::read(&reference)?);
            }
            eprintln!(
                "material={label} operation={action} killed_at={edge} already_advanced={accepted} exact_continuation=32"
            );
        }
    }
    Ok(())
}
