// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{error::Error, fs, path::Path};
type Test = std::result::Result<(), Box<dyn Error>>;
type FleetInputs = (Vec<(u64, f64)>, Vec<(u64, [f64; 3])>);

fn controls(p: &GraspFleetPool) -> FleetInputs {
    let mut e = vec![];
    let mut f = vec![];
    for (i, s) in p.spec().assemblies.iter().enumerate() {
        e.push((
            s.payload.tree.joints[0].joint_id,
            if i == 0 { 0.035 } else { -0.023 },
        ));
        f.push((
            s.payload.payload.id,
            [0.01, -0.005, if i == 0 { 1.2 } else { -0.82 }],
        ));
    }
    (e, f)
}
fn act(p: &mut GraspFleetPool, op: &str, tick: u64) -> Result<()> {
    let model = p.spec().assemblies[0].payload.tree.model_id;
    let id = p.spec().assemblies[0].payload.tree.joints[0].joint_id;
    match op {
        "step" => {
            let (e, f) = controls(p);
            p.advance_with_inputs_at(0, tick, &e, &f)
        }
        "cross" => {
            let f = p
                .spec()
                .assemblies
                .iter()
                .map(|a| {
                    (
                        a.payload.payload.id,
                        [0., -4. * a.payload.payload.position[1].signum(), 0.],
                    )
                })
                .collect::<Vec<_>>();
            p.advance_with_inputs_at(0, tick, &[], &f)
        }
        "motor" => p.set_velocity_motor(
            id,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: -2.,
                max_force: 1.5,
            }),
        ),
        "queue" => p.queue_effort(id, 0.025),
        "transfer" => p.transfer(model, 2),
        "reset" => p.reset(),
        _ => panic!("unknown fleet fault operation"),
    }
}
#[test]
#[ignore = "subprocess for the global fleet transaction matrix"]
fn fleet_controller_process() -> Test {
    let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let op = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = GraspFleetPool::resume(Path::new(&exe), Path::new(&dir))?;
    act(&mut p, &op, tick)?;
    panic!("fleet process missed selected barrier")
}
#[test]
#[ignore = "requires the fleet worker and actual killed coordinators"]
fn fleet_controller_deaths_retain_both_grasps_and_one_clock() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let xml = include_str!("../../../../../../examples/distributed_grasp_fleet.sdf");
    let scenes = gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
        xml,
        "fleet.sdf",
        "default",
    )?;
    let spec = GraspFleetSpec {
        assemblies: serde_json::from_value(serde_json::to_value(scenes)?)?,
    };
    let seed = tempfile::tempdir()?;
    let mut p = GraspFleetPool::create(
        exe,
        seed.path(),
        3,
        spec.clone(),
        GraspFleetPlacement::RoundRobin,
    )?;
    for s in &spec.assemblies {
        let palm_y = s.payload.tree.links[0].position[1];
        for j in &s.payload.tree.joints {
            let child = s
                .payload
                .tree
                .links
                .iter()
                .find(|l| l.id == j.child_id)
                .unwrap();
            p.set_velocity_motor(
                j.joint_id,
                Some(gz_physics::JointVelocityMotor {
                    target_velocity: -(child.position[1] - palm_y).signum() * 6.,
                    max_force: 4.,
                }),
            )?;
        }
    }
    let (_, forces) = controls(&p);
    for t in 0..675 {
        p.advance_with_inputs(&[], if t < 650 { &[] } else { &forces })?;
    }
    for s in &spec.assemblies {
        let m = s.payload.tree.model_id;
        assert_eq!(
            p.payload_reactions(m)
                .unwrap()
                .iter()
                .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                .count(),
            2
        );
        p.queue_effort(s.payload.tree.joints[1].joint_id, -0.049)?;
    }
    let tick = p.tick();
    let base = fs::read(seed.path().join(FILE))?;
    drop(p);
    for (op, edges) in [
        (
            "step",
            &[
                "grasp-fleet-worker-prepared",
                "grasp-fleet-all-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "motor",
            &[
                "grasp-fleet-command-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "queue",
            &[
                "grasp-fleet-command-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "transfer",
            &[
                "grasp-fleet-destination-staged",
                "grasp-fleet-source-fenced",
                "grasp-fleet-destination-active",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &[
                "grasp-fleet-destination-staged",
                "grasp-fleet-source-fenced",
                "grasp-fleet-destination-active",
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
                fs::write(d.path().join(FILE), &base)?;
            }
            crate::floating_tree::crash_tests::terminate_probe(
                "floating_tree::placement::fleet::crash_tests::fleet_controller_process",
                a.path(),
                op,
                edge,
                tick,
            )?;
            let mut r = GraspFleetPool::resume(exe, a.path())?;
            let mut q = GraspFleetPool::resume(exe, b.path())?;
            let published = matches!(edge, "after-rename" | "after-sync");
            if published {
                act(&mut q, op, tick)?;
            }
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?
            );
            if !published {
                act(&mut r, op, tick)?;
                act(&mut q, op, tick)?;
            } else if op == "step" {
                assert!(matches!(
                    act(&mut r, op, tick),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            for _ in 0..32 {
                let (e, f) = controls(&r);
                r.advance_with_inputs(&e, &f)?;
                for (id, x) in e {
                    q.queue_effort(id, x)?;
                }
                q.advance_with_inputs(&[], &f)?;
                assert_eq!(
                    fs::read(a.path().join(FILE))?,
                    fs::read(b.path().join(FILE))?
                );
            }
            eprintln!(
                "fleet op={op} edge={edge} published={published} both_bilateral=true seed_tick={tick} exact_continuation=32"
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "requires a real worker and controller death during a rejected approach"]
fn fleet_rejected_crossing_controller_death_preserves_all_committed_bodies() -> Test {
    let exe_s = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&exe_s);
    let xml = include_str!("../../../../../../examples/distributed_grasp_fleet.sdf")
        .replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58");
    let scenes = gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
        &xml,
        "fleet.sdf",
        "default",
    )?;
    let s = GraspFleetSpec {
        assemblies: serde_json::from_value(serde_json::to_value(scenes)?)?,
    };
    let dir = tempfile::tempdir()?;
    let mut p = GraspFleetPool::create(exe, dir.path(), 2, s, GraspFleetPlacement::RoundRobin)?;
    let j = p.spec().assemblies[0].payload.tree.joints[0].joint_id;
    let tick = loop {
        p.queue_effort(j, 0.0002)?;
        let old = p.tick();
        assert!(old < 800);
        match act(&mut p, "cross", old) {
            Ok(()) => (),
            Err(ArticulationError::PossibleCrossAssemblyContact { .. }) => break old,
            Err(e) => return Err(e.into()),
        }
    };
    let original = fs::read(dir.path().join(FILE))?;
    drop(p);
    crate::floating_tree::crash_tests::terminate_probe(
        "floating_tree::placement::fleet::crash_tests::fleet_controller_process",
        dir.path(),
        "cross",
        "grasp-fleet-crossing-rejected",
        tick,
    )?;
    assert_eq!(fs::read(dir.path().join(FILE))?, original);
    let mut p = GraspFleetPool::resume(exe, dir.path())?;
    assert_eq!(p.tick(), tick);
    assert_eq!(p.pending_joint_effort(j), Some(0.0002));
    assert!(matches!(
        act(&mut p, "cross", tick),
        Err(ArticulationError::PossibleCrossAssemblyContact { .. })
    ));
    eprintln!(
        "two independently prepared workers; real SIGKILL while rejecting cross-assembly contact; original clock {tick}, queued effort, both payloads and every archive byte recovered"
    );
    Ok(())
}
