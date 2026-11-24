// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{error::Error, fs};
type Test<T = ()> = std::result::Result<T, Box<dyn Error>>;
const PROBE: &str = "floating_tree::material::crash_tests::material_global_controller_process";
const CLOTH: &str = include_str!("../../../../../examples/articulated_material_cloth_impact.sdf");
const CABLE: &str = include_str!("../../../../../examples/articulated_material_cable_impact.sdf");
fn recipe(xml: &str) -> Test<(ArticulatedMaterialSpec, SoftBody)> {
    let a = gz_sim_scene::deformable::articulated_impact::distributed_material_contact_tree_spec(
        xml,
        "world.sdf",
        "default",
    )?;
    let tree = ContactTreeSpec {
        tree: serde_json::from_value(serde_json::to_value(a.tree)?)?,
        limits: serde_json::from_value(serde_json::to_value(a.limits)?)?,
    };
    Ok((
        ArticulatedMaterialSpec {
            tree,
            material_name: a.material_name,
            barriers: a
                .barriers
                .into_iter()
                .map(|b| MaterialImpactBarrier {
                    tick: b.time.as_millis() as u64,
                    links: b.links,
                    restitution: b.restitution,
                })
                .collect(),
        },
        a.material,
    ))
}
fn command(p: &mut ArticulatedMaterialPool, action: &str, tick: u64) -> Result<()> {
    match action {
        "advance" => p.advance_with_efforts_at(0, tick, &[]),
        "transfer" => p.transfer(1),
        "reset" => p.reset(),
        _ => panic!("unknown material command"),
    }
}
#[test]
#[ignore = "controller child for real signals at the coupled publication boundary"]
fn material_global_controller_process() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let action = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let (kind, op) = action.split_once('-').unwrap();
    let (_, cold) = recipe(if kind == "cloth" { CLOTH } else { CABLE })?;
    let mut p = ArticulatedMaterialPool::resume(Path::new(&worker), Path::new(&dir), cold)?;
    command(&mut p, op, tick)?;
    panic!("controller missed coupled material crash point")
}
#[test]
#[ignore = "real coordinator SIGKILL between separately applied rigid and XPBD impulses"]
fn recovered_material_controller_applies_both_physical_impulses_exactly_once() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    for (kind, xml) in [("cloth", CLOTH), ("cable", CABLE)] {
        let (s, cold) = recipe(xml)?;
        let seed = tempfile::tempdir()?;
        let mut p = ArticulatedMaterialPool::create(exe, seed.path(), 3, s, cold.clone())?;
        for _ in 0..99 {
            p.advance()?;
        }
        let prior = fs::read(seed.path().join(FILE))?;
        p.advance()?;
        let after = fs::read(seed.path().join(FILE))?;
        drop(p);
        for (action, base, tick, edges) in [
            (
                "advance",
                &prior,
                99,
                &[
                    "material-rigid-advanced",
                    "material-xpbd-advanced",
                    "material-signed-impulses-prepared",
                    "contact-tree-native-prepared",
                    "material-native-impulses-applied",
                    "material-particle-impulses-applied",
                    "material-both-prepared",
                    "before-rename",
                    "after-rename",
                    "after-sync",
                ][..],
            ),
            (
                "transfer",
                &after,
                100,
                &[
                    "tree-destination-staged",
                    "tree-source-fenced",
                    "tree-destination-active",
                    "material-native-transferred",
                    "material-both-prepared",
                    "before-rename",
                    "after-rename",
                    "after-sync",
                ][..],
            ),
            (
                "reset",
                &after,
                100,
                &[
                    "material-both-prepared",
                    "before-rename",
                    "after-rename",
                    "after-sync",
                ][..],
            ),
        ] {
            for &edge in edges {
                let a = tempfile::tempdir()?;
                let b = tempfile::tempdir()?;
                for d in [&a, &b] {
                    fs::write(d.path().join(FILE), base)?;
                }
                crate::floating_tree::crash_tests::terminate_probe(
                    PROBE,
                    a.path(),
                    &format!("{kind}-{action}"),
                    edge,
                    tick,
                )?;
                let mut r = ArticulatedMaterialPool::resume(exe, a.path(), cold.clone())?;
                let mut q = ArticulatedMaterialPool::resume(exe, b.path(), cold.clone())?;
                let accepted = matches!(edge, "after-rename" | "after-sync");
                if accepted {
                    command(&mut q, action, tick)?;
                }
                assert_eq!(
                    fs::read(a.path().join(FILE))?,
                    fs::read(b.path().join(FILE))?
                );
                if !accepted {
                    command(&mut r, action, tick)?;
                    command(&mut q, action, tick)?;
                } else if action == "advance" {
                    assert!(command(&mut r, action, tick).is_err());
                }
                for _ in 0..72 {
                    r.advance()?;
                    q.advance()?;
                    assert_eq!(
                        fs::read(a.path().join(FILE))?,
                        fs::read(b.path().join(FILE))?
                    );
                }
                assert_eq!(r.last_impact().is_some(), action != "reset");
                eprintln!(
                    "material={kind} action={action} sigkill={edge} published={accepted} identical_whole_native_material_files=72"
                );
            }
        }
    }
    Ok(())
}

#[test]
#[ignore = "requires generated ordinary matched material witnesses and worker"]
fn altered_portable_impulses_and_wrong_cold_material_are_rejected_before_resume() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&exe);
    let witness = std::env::var("GZ_MATERIAL_WITNESSES")?;
    for (kind, xml, other) in [("cloth", CLOTH, CABLE), ("cable", CABLE, CLOTH)] {
        let source = fs::read(
            Path::new(&witness)
                .join(kind)
                .join("accepted.gzmaterialtree"),
        )?;
        let dir = tempfile::tempdir()?;
        let file = dir.path().join(FILE);
        fs::write(&file, &source)?;
        let (spec, cold) = recipe(xml)?;
        assert!(ArticulatedMaterialPool::resume(exe, dir.path(), recipe(other)?.1).is_err());
        assert_eq!(fs::read(&file)?, source);
        let mut invalid_spec = spec.clone();
        invalid_spec.barriers[0].restitution = f64::INFINITY;
        let wrong = tempfile::tempdir()?;
        assert!(
            ArticulatedMaterialPool::create(exe, wrong.path(), 2, invalid_spec, cold.clone())
                .is_err()
        );
        assert!(!wrong.path().join(FILE).exists());
        for variant in 0..6 {
            let (s, hash, mut saved): (ArticulatedMaterialSpec, [u8; 32], Saved) =
                codec().deserialize(&source[40..])?;
            let r = saved.last.as_mut().unwrap();
            match variant {
                0 => r.loads[0].on_particles[0].1[0] += 0.0001,
                1 => r.loads[0].weights[0] += 0.1,
                2 => r.loads[0].point[1] += 0.2,
                3 => r.kinetic_energy_change += 0.1,
                4 => r.loads[0].normal = [0.; 3],
                _ => saved.last = None,
            }
            let blob = codec().serialize(&(s, hash, saved))?;
            let encoded = [MAGIC.as_slice(), Sha256::digest(&blob).as_slice(), &blob].concat();
            fs::write(&file, &encoded)?;
            assert!(ArticulatedMaterialPool::resume(exe, dir.path(), cold.clone()).is_err());
            assert_eq!(fs::read(&file)?, encoded);
        }
        fs::write(&file, &source)?;
        let p = ArticulatedMaterialPool::resume(exe, dir.path(), cold)?;
        assert_eq!(p.last_impact().unwrap().tick, 100);
    }
    Ok(())
}

#[test]
#[ignore = "child for actual native worker loss during atomic material transaction"]
fn material_native_kill_after_private_step() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let kind = std::env::var("GZ_MATERIAL_KIND")?;
    let (_, cold) = recipe(if kind == "cloth" { CLOTH } else { CABLE })?;
    let mut p = ArticulatedMaterialPool::resume(Path::new(&exe), Path::new(&dir), cold)?;
    let original = p.worker_pid(p.owner());
    p.advance_with_efforts_at(0, 99, &[])?;
    assert_eq!(p.tick(), 100);
    assert_ne!(original, p.worker_pid(p.owner()));
    assert!(p.last_impact().is_some());
    Ok(())
}
#[test]
#[ignore = "requires generated ordinary matched material witnesses and worker"]
fn native_loss_between_private_step_and_signed_response_retries_both_solvers() -> Test {
    const CHILD: &str =
        "floating_tree::material::crash_tests::material_native_kill_after_private_step";
    let root = std::env::var("GZ_MATERIAL_WITNESSES")?;
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&exe);
    for (kind, xml) in [("cloth", CLOTH), ("cable", CABLE)] {
        let witness = Path::new(&root).join(kind);
        for edge in [
            "GZ_MATERIAL_GEOMETRY_KILL_ONCE",
            "GZ_MATERIAL_CONTACT_KILL_ONCE",
        ] {
            let dir = tempfile::tempdir()?;
            let file = dir.path().join(FILE);
            fs::copy(witness.join("prior.gzmaterialtree"), &file)?;
            let control = tempfile::tempdir()?;
            fs::copy(&file, control.path().join(FILE))?;
            let marker = dir.path().join("lost-native-worker");
            assert!(
                std::process::Command::new(std::env::current_exe()?)
                    .args(["--exact", CHILD, "--ignored"])
                    .env("GZ_MATERIAL_KIND", kind)
                    .env("GZ_FLOATING_TREE_TEST_SESSION", dir.path())
                    .env(edge, &marker)
                    .status()?
                    .success()
            );
            assert!(marker.is_file());
            let cold = recipe(xml)?.1;
            let mut direct = ArticulatedMaterialPool::resume(exe, control.path(), cold.clone())?;
            direct.advance()?;
            assert_eq!(fs::read(&file)?, fs::read(control.path().join(FILE))?);
            let mut resumed = ArticulatedMaterialPool::resume(exe, dir.path(), cold)?;
            for _ in 0..40 {
                resumed.advance()?;
                direct.advance()?;
                assert_eq!(fs::read(&file)?, fs::read(control.path().join(FILE))?);
            }
            eprintln!(
                "material={kind} actual_native_death={edge} automatic_retries_complete=true exact_physical_files=41"
            );
        }
    }
    Ok(())
}
