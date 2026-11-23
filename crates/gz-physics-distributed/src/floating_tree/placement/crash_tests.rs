// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{collections::BTreeMap, error::Error, fs, path::Path};
type TestResult = std::result::Result<(), Box<dyn Error>>;
fn force(t: u64) -> [f64; 3] {
    [
        0.,
        0.,
        match t {
            650..850 | 1250..1450 => 1.2,
            850..1250 => -1.2,
            _ => 0.,
        },
    ]
}
fn run(p: &mut SplitGraspPool, op: &str, tick: u64) -> Result<()> {
    match op {
        "release" | "rejoin" | "split-step" => {
            p.advance_with_payload_force_at(0, tick, &[], force(tick))
        }
        "tree" | "part" => {
            let free = (0..p.width())
                .find(|i| !p.owners().contains(i))
                .expect("unused destination");
            if op == "tree" {
                p.transfer(free)
            } else {
                p.transfer_payload(free)
            }
        }
        "reset" => p.reset(),
        _ => panic!("unknown selective grasp fault probe"),
    }
}
#[test]
#[ignore = "child of the selective grasp crash matrix"]
fn placement_controller_process() -> TestResult {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = SplitGraspPool::resume(Path::new(&exe), Path::new(&dir))?;
    run(&mut p, &op, tick)?;
    panic!("selective grasp coordinator missed its cut")
}
#[test]
#[ignore = "requires GZSPWK01 worker and real SIGKILLs on prepared release, approach and ownership changes"]
fn controller_kills_at_split_and_rejoin_publish_one_physical_interval() -> TestResult {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let xml = include_str!("../../../../../examples/distributed_gripper_payload.sdf");
    let external = gz_sim_scene::distributed_payload_tree_spec(xml, "gripper.sdf", "default")?;
    let spec: PayloadTreeSpec = serde_json::from_value(serde_json::to_value(&external)?)?;
    let seed = tempfile::tempdir()?;
    let mut p = SplitGraspPool::create(
        exe,
        seed.path(),
        3,
        GraspTreeSpec {
            payload: spec.clone(),
        },
        GraspPlacement::Adaptive { release_dwell: 6 },
    )?;
    let mut seeds = BTreeMap::new();
    for tick in 0..2400 {
        if matches!(tick, 0 | 600 | 1450) {
            for j in &spec.tree.joints {
                let child = spec.tree.links.iter().find(|l| l.id == j.child_id).unwrap();
                let sign = -child.position[1].signum();
                p.set_velocity_motor(
                    j.joint_id,
                    Some(gz_physics::JointVelocityMotor {
                        target_velocity: sign * if tick == 600 { -6. } else { 6. },
                        max_force: 4.,
                    }),
                )?;
            }
        }
        let before = fs::read(seed.path().join(FILE))?;
        let owners = p.owners();
        let old_split = owners[0] != owners[1];
        p.advance_with_payload_force_at(0, tick, &[], force(tick))?;
        let new = p.owners();
        let split = new[0] != new[1];
        if !old_split && split {
            seeds.entry("release").or_insert((tick, before));
        } else if old_split && !split {
            seeds.entry("rejoin").or_insert((tick, before));
        } else if split {
            seeds.entry("split-step").or_insert((tick, before));
        }
        if seeds.len() == 3 {
            break;
        }
    }
    assert_eq!(
        seeds.len(),
        3,
        "no physical release, separate motion or contact approach"
    );
    drop(p);
    for (op, edges) in [
        (
            "release",
            &[
                "grasp-destinations-staged",
                "grasp-sources-fenced",
                "grasp-destinations-active",
                "grasp-step-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "rejoin",
            &[
                "grasp-private-tree-advanced",
                "grasp-private-preview-rejected",
                "grasp-destinations-staged",
                "grasp-sources-fenced",
                "grasp-destinations-active",
                "grasp-step-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "split-step",
            &[
                "grasp-private-tree-advanced",
                "grasp-step-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "tree",
            &[
                "grasp-destinations-staged",
                "grasp-sources-fenced",
                "grasp-destinations-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "part",
            &[
                "grasp-destinations-staged",
                "grasp-sources-fenced",
                "grasp-destinations-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &[
                "grasp-destinations-staged",
                "grasp-sources-fenced",
                "grasp-destinations-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
    ] {
        let (tick, original) = seeds.get(op).unwrap_or(&seeds["split-step"]);
        for &edge in edges {
            let a = tempfile::tempdir()?;
            let b = tempfile::tempdir()?;
            for d in [&a, &b] {
                fs::write(d.path().join(FILE), original)?;
            }
            super::super::crash_tests::terminate_probe(
                "floating_tree::placement::crash_tests::placement_controller_process",
                a.path(),
                op,
                edge,
                *tick,
            )?;
            let mut recovered = SplitGraspPool::resume(exe, a.path())?;
            let mut control = SplitGraspPool::resume(exe, b.path())?;
            let sent = matches!(edge, "after-rename" | "after-sync");
            if sent {
                run(&mut control, op, *tick)?;
            }
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?,
                "unexpected disk outcome op={op} edge={edge}"
            );
            if !sent {
                run(&mut recovered, op, *tick)?;
                run(&mut control, op, *tick)?;
            } else if matches!(op, "release" | "rejoin" | "split-step") {
                assert!(matches!(
                    run(&mut recovered, op, *tick),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            for _ in 0..32 {
                recovered.advance_with_payload_force(&[], force(recovered.tick()))?;
                control.advance_with_payload_force(&[], force(control.tick()))?;
                assert_eq!(
                    fs::read(a.path().join(FILE))?,
                    fs::read(b.path().join(FILE))?
                );
            }
            eprintln!(
                "selective_grasp_controller_death op={op} edge={edge} seed_tick={tick} published={sent} continuation_frames=32"
            );
        }
    }
    Ok(())
}
