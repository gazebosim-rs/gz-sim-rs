// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{error::Error, fs, path::Path};
type Test = std::result::Result<(), Box<dyn Error>>;
#[path = "../../../tests/support/tree_payload_fixture.rs"]
mod fixture;
const PROBE: &str = "floating_tree::cluster::crash_tests::coupled_controller_process";
fn update(p: &mut CoupledGraspFleetPool, op: &str, t: u64) -> Result<()> {
    let id = p.spec().assemblies[0].payload.tree.joints[0].joint_id;
    let loads = p
        .spec()
        .assemblies
        .iter()
        .map(|s| {
            (
                s.payload.payload.id,
                [0., -4. * s.payload.payload.position[1].signum(), 0.02],
            )
        })
        .collect::<Vec<_>>();
    match op {
        "step" => p.advance_with_inputs_at(0, t, &[(id, -0.001)], &loads),
        "motor" => p.set_velocity_motor(
            id,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: 0.15,
                max_force: 0.08,
            }),
        ),
        "queue" => p.queue_effort(id, 0.021),
        "transfer" => p.transfer(2),
        "reset" => p.reset(),
        _ => panic!("unknown coupled fault operation"),
    }
}
#[test]
#[ignore = "entry point for a real killed coupled graph controller"]
fn coupled_controller_process() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let t = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = CoupledGraspFleetPool::resume(Path::new(&exe), Path::new(&dir))?;
    update(&mut p, &op, t)?;
    panic!("coupled graph fault marker missing")
}
fn two_cages() -> GraspFleetSpec {
    let mut assemblies = Vec::new();
    for k in 0..2 {
        let d = k * 2000;
        let y = (k * 4) as f64;
        let mut s = fixture::cage(8, false, 0.24, [0.26, 0.074, -0.021]);
        s.exact_revolute_locks = true;
        s.payload_can_sleep = false;
        s.tree.model_id += d;
        for l in &mut s.tree.links {
            l.id += d;
            l.position[1] += y;
        }
        for j in &mut s.tree.joints {
            j.joint_id += d;
            j.parent_id += d;
            j.child_id += d;
            j.anchor[1] += y;
        }
        for l in &mut s.limits {
            l.joint_id += d;
        }
        s.payload.id += d;
        s.payload.position[1] += y;
        assemblies.push(GraspTreeSpec { payload: s });
    }
    GraspFleetSpec { assemblies }
}
fn matrix(exe: &Path, dir: &Path, base: &[u8], tick: u64, op: &str, edges: &[&str]) -> Test {
    for &edge in edges {
        let a = tempfile::tempdir_in(dir)?;
        let b = tempfile::tempdir_in(dir)?;
        for x in [&a, &b] {
            fs::write(x.path().join(FILE), base)?;
        }
        crate::floating_tree::crash_tests::terminate_probe(PROBE, a.path(), op, edge, tick)?;
        let mut p = CoupledGraspFleetPool::resume(exe, a.path())?;
        let mut q = CoupledGraspFleetPool::resume(exe, b.path())?;
        let published = matches!(edge, "after-rename" | "after-sync");
        if published {
            update(&mut q, op, tick)?;
        }
        assert_eq!(
            fs::read(a.path().join(FILE))?,
            fs::read(b.path().join(FILE))?
        );
        if !published {
            update(&mut p, op, tick)?;
            update(&mut q, op, tick)?;
        } else if op == "step" {
            assert!(matches!(
                update(&mut p, op, tick),
                Err(ArticulationError::StaleInterval { .. })
            ));
        }
        for n in 0..32 {
            let id = p.spec().assemblies[0].payload.tree.joints[0].joint_id;
            let forces = p
                .spec()
                .assemblies
                .iter()
                .map(|g| {
                    (
                        g.payload.payload.id,
                        [0., -4. * g.payload.payload.position[1].signum(), 0.02],
                    )
                })
                .collect::<Vec<_>>();
            let e = (n as f64 - 11.) * 0.00001;
            p.advance_with_inputs(&[(id, e)], &forces)?;
            q.queue_effort(id, e)?;
            q.advance_with_inputs(&[], &forces)?;
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?
            );
        }
        eprintln!(
            "coupled two-assembly native op={op} edge={edge} published={published} seed_tick={tick} exactly_once_continuation=32"
        );
    }
    Ok(())
}
#[test]
#[ignore = "requires the two-assembly worker and real controller subprocess kills"]
fn coupled_cross_contacts_and_paged_graphs_recover_after_controller_sigkill() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let scratch = tempfile::tempdir()?;
    let xml = include_str!("../../../../../examples/distributed_grasp_fleet.sdf")
        .replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58");
    let groups = gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
        &xml,
        "fleet.sdf",
        "default",
    )?;
    let s = GraspFleetSpec {
        assemblies: serde_json::from_value(serde_json::to_value(groups)?)?,
    };
    let dir = tempfile::tempdir_in(scratch.path())?;
    let mut p = CoupledGraspFleetPool::create(exe, dir.path(), 3, s)?;
    let mut count = 0;
    for t in 0..850 {
        update(&mut p, "step", t)?;
        if p.cross_reactions().iter().any(|r| {
            r.reaction.active
                && r.reaction.normal_impulse > 0.
                && r.reaction.warmstart_normal_impulse > 0.
        }) {
            count += 1;
            if count == 12 {
                break;
            }
        }
    }
    assert_eq!(count, 12);
    let ids = p
        .spec()
        .assemblies
        .iter()
        .map(|g| g.payload.tree.joints[1].joint_id)
        .collect::<Vec<_>>();
    for id in ids {
        p.queue_effort(id, -0.015)?;
    }
    let tick = p.tick();
    let old = fs::read(dir.path().join(FILE))?;
    drop(p);
    for (op, edges) in [
        (
            "step",
            &[
                "cluster-step-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "queue",
            &[
                "cluster-command-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "motor",
            &[
                "cluster-command-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "transfer",
            &[
                "cluster-destination-staged",
                "cluster-source-fenced",
                "cluster-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &[
                "cluster-destination-staged",
                "cluster-source-fenced",
                "cluster-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
    ] {
        matrix(exe, scratch.path(), &old, tick, op, edges)?;
    }
    let d = tempfile::tempdir_in(scratch.path())?;
    let mut p = CoupledGraspFleetPool::create(exe, d.path(), 3, two_cages())?;
    for _ in 0..10 {
        p.advance()?;
    }
    assert!(p.image().unwrap().len() > 65536);
    let tick = p.tick();
    let img = fs::read(d.path().join(FILE))?;
    drop(p);
    matrix(
        exe,
        scratch.path(),
        &img,
        tick,
        "transfer",
        &["cluster-transfer-upload-page"],
    )?;
    matrix(
        exe,
        scratch.path(),
        &img,
        tick,
        "step",
        &["cluster-receipt-page"],
    )?;
    Ok(())
}
