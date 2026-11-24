// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{error::Error, fs, path::Path};
type Test = std::result::Result<(), Box<dyn Error>>;
const PROBE: &str = "floating_tree::adaptive::crash_tests::adaptive_grasp_controller_process";
fn load(p: &AdaptiveGraspFleetPool) -> Vec<(u64, [f64; 3])> {
    p.spec()
        .assemblies
        .iter()
        .map(|g| {
            let sign = -g.payload.payload.position[1].signum();
            (g.payload.payload.id, [0., 4. * sign, 0.03 * sign])
        })
        .collect()
}
fn command(p: &mut AdaptiveGraspFleetPool, action: &str, tick: u64) -> Result<()> {
    let j = p.spec().assemblies[0].payload.tree.joints[0].joint_id;
    match action {
        "join" => p.advance_with_inputs_at(0, tick, &[(j, -0.0001)], &load(p)),
        "warm_join" => {
            let f = p
                .spec()
                .assemblies
                .iter()
                .map(|g| {
                    (
                        g.payload.payload.id,
                        [0., -3. * g.payload.payload.position[1].signum(), 0.],
                    )
                })
                .collect::<Vec<_>>();
            p.advance_with_inputs_at(0, tick, &[], &f)
        }
        "queue" => p.queue_effort(j, 0.0002),
        "reset" => p.reset(),
        _ => panic!("unexpected adaptive action"),
    }
}
#[test]
#[ignore = "controller subprocess for global promotion crashes"]
fn adaptive_grasp_controller_process() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
    let action = std::env::var("GZ_FLOATING_TREE_TEST_ACTION")?;
    let tick = std::env::var("GZ_TREE_PROBE_TICK")?.parse()?;
    let mut p = AdaptiveGraspFleetPool::resume(Path::new(&worker), Path::new(&dir))?;
    command(&mut p, &action, tick)?;
    panic!("failed to encounter adaptive crash edge")
}
#[test]
#[ignore = "requires actual managed workers and SIGKILL across both native owner protocols"]
fn adaptive_grasp_controller_kills_preserve_one_join_and_upcoming_impact() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let xml = include_str!("../../../../../examples/distributed_grasp_fleet.sdf")
        .replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58");
    let groups = gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
        &xml,
        "approach.sdf",
        "default",
    )?;
    let s = GraspFleetSpec {
        assemblies: serde_json::from_value(serde_json::to_value(groups)?)?,
    };
    let seed = tempfile::tempdir()?;
    let mut p = AdaptiveGraspFleetPool::create(exe, seed.path(), 3, s)?;
    let j = p.spec().assemblies[0].payload.tree.joints[0].joint_id;
    let (before, after, tick) = loop {
        let tick = p.tick();
        assert!(tick < 700);
        p.queue_effort(j, 0.0003)?;
        let old = fs::read(seed.path().join(FILE))?;
        command(&mut p, "join", tick)?;
        if p.is_coupled() {
            break (old, fs::read(seed.path().join(FILE))?, tick);
        }
    };
    drop(p);
    for (action, base, t, edges) in [
        (
            "join",
            &before,
            tick,
            &[
                "adaptive-crossing-trials-discarded",
                "adaptive-clear-members-prepared",
                "adaptive-independent-workers-closed",
                "adaptive-coupled-prior-installed",
                "adaptive-coupled-interval-prepared",
                "before-rename",
                "after-rename",
                "after-sync",
            ][..],
        ),
        (
            "reset",
            &after,
            tick + 1,
            &[
                "adaptive-cold-roles-installed",
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
            crate::floating_tree::crash_tests::terminate_probe(PROBE, a.path(), action, edge, t)?;
            let mut r = AdaptiveGraspFleetPool::resume(exe, a.path())?;
            let mut q = AdaptiveGraspFleetPool::resume(exe, b.path())?;
            let published = matches!(edge, "after-rename" | "after-sync");
            if published {
                command(&mut q, action, t)?;
            }
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?
            );
            if !published {
                command(&mut r, action, t)?;
                command(&mut q, action, t)?;
            } else if action == "join" {
                assert!(matches!(
                    command(&mut r, action, t),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            let mut contact = 0;
            for _ in 0..80 {
                let f = load(&r);
                r.advance_with_inputs(&[(j, 0.0002)], &f)?;
                q.queue_effort(j, 0.0002)?;
                q.advance_with_inputs(&[], &f)?;
                assert_eq!(
                    fs::read(a.path().join(FILE))?,
                    fs::read(b.path().join(FILE))?
                );
                if r.cross_reactions().iter().any(|row| {
                    row.reaction.normal_impulse > 0. && row.reaction.warmstart_normal_impulse > 0.
                }) {
                    contact += 1;
                }
            }
            if action == "join" {
                assert!(contact >= 25);
            }
            eprintln!(
                "adaptive {action} edge={edge} compiled_stdin_and_coupled_workers=true old_or_advanced={published} seed={t} exact_steps=80 impact_warm={contact}"
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "extract the first managed promotion's native inputs for an independent physics audit"]
fn extract_grasp_promotion_inputs_at_captured_boundary() -> Test {
    let path = std::env::var("GZ_ADAPTIVE_GRASP_WITNESS")?;
    let root = Path::new(&path);
    let dir = tempfile::tempdir()?;
    fs::copy(
        root.join("prior.gzadaptivegraspfleet"),
        dir.path().join(FILE),
    )?;
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let mut p = AdaptiveGraspFleetPool::resume(Path::new(&worker), dir.path())?;
    let Active::Separate(separate) = p.active.as_mut().unwrap() else {
        panic!("expected separate members")
    };
    let (bytes, _) = separate.export_members(false, &[])?;
    for (i, img) in bytes.iter().enumerate() {
        fs::write(root.join(format!("member-{i}.gzfclr")), img)?;
    }
    fs::write(
        root.join("construction.json"),
        serde_json::to_vec_pretty(&p.spec)?,
    )?;
    eprintln!(
        "actual independent owner exports epoch={} tick={} packet_bytes={:?}",
        p.epoch(),
        p.tick(),
        bytes.each_ref().map(Vec::len)
    );
    command(&mut p, "join", 500).map_err(|e| format!("repeating promotion: {e:?}"))?;
    Ok(())
}

#[test]
#[ignore = "requires real worker and controller SIGKILL, including a history at sustained bilateral load"]
fn verified_warm_replay_controller_kills_preserve_history_and_first_cross_contact() -> Test {
    held_faults(GraspFleetMerge::VerifiedReplay(GraspReplayBudget {
        max_intervals: 2048,
        max_events: 4096,
    }))
}
#[test]
#[ignore = "real warm member export, destination solve, controller crashes and durable graph replacement"]
fn native_warm_member_controller_kills_preserve_contact_and_graph() -> Test {
    held_faults(GraspFleetMerge::WarmMembers)
}
fn held_prior(
    exe: &Path,
    seed: &Path,
    policy: GraspFleetMerge,
) -> std::result::Result<AdaptiveGraspFleetPool, Box<dyn Error>> {
    let xml = include_str!("../../../../../examples/distributed_grasp_fleet.sdf")
        .replace("-0.95 0.58", "-0.52 0.58")
        .replace(" 0.95 0.58", " 0.52 0.58")
        .replace("-0.946 0.58", "-0.516 0.58")
        .replace(" 0.954 0.58", " 0.524 0.58");
    let groups = gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
        &xml,
        "grasp.sdf",
        "default",
    )?;
    let spec = GraspFleetSpec {
        assemblies: serde_json::from_value(serde_json::to_value(groups)?)?,
    };
    let mut p = AdaptiveGraspFleetPool::create_with_policy(exe, seed, 3, spec.clone(), policy)?;
    for g in &spec.assemblies {
        for j in &g.payload.tree.joints {
            let y = g
                .payload
                .tree
                .links
                .iter()
                .find(|l| l.id == j.child_id)
                .unwrap()
                .position[1];
            p.set_velocity_motor(
                j.joint_id,
                Some(JointVelocityMotor {
                    target_velocity: -6. * (y - g.payload.tree.links[0].position[1]).signum(),
                    max_force: 4.,
                }),
            )?;
        }
    }
    for t in 0..999 {
        if t < 650 {
            p.advance()?;
        } else {
            command(&mut p, "queue", t)?;
            command(&mut p, "warm_join", t)?;
        }
    }
    Ok(p)
}
fn held_faults(policy: GraspFleetMerge) -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let seed = tempfile::tempdir()?;
    let mut p = held_prior(exe, seed.path(), policy)?;
    let spec = p.spec().clone();
    let queue = fs::read(seed.path().join(FILE))?;
    command(&mut p, "queue", 999)?;
    let prior = fs::read(seed.path().join(FILE))?;
    assert!(!p.is_coupled());
    command(&mut p, "warm_join", 999)?;
    assert_eq!(p.joined_at(), Some(999));
    let joined = fs::read(seed.path().join(FILE))?;
    drop(p);
    let warm_members = policy == GraspFleetMerge::WarmMembers;
    let prepared = if warm_members {
        "adaptive-cut-prepared"
    } else {
        "adaptive-journal-prepared"
    };
    let mut join_edges = vec![
        "adaptive-crossing-trials-discarded",
        if warm_members {
            "adaptive-warm-members-prepared"
        } else {
            "adaptive-replay-history-verified"
        },
        "adaptive-independent-workers-closed",
        "adaptive-coupled-prior-installed",
    ];
    if warm_members {
        if crate::floating_tree::pool::worker_supports(exe, "--probe-grasp-fleet-prior-v3")? {
            join_edges.push("adaptive-prior-proof-verified");
        }
        join_edges.push("adaptive-warm-continuation-verified");
    }
    join_edges.extend([
        "adaptive-coupled-interval-prepared",
        prepared,
        "before-rename",
        "after-rename",
        "after-sync",
    ]);
    for (action, bytes, tick, edges) in [
        ("warm_join", &prior, 999, &join_edges[..]),
        (
            "queue",
            &queue,
            999,
            &[prepared, "before-rename", "after-rename", "after-sync"][..],
        ),
        (
            "reset",
            &joined,
            1000,
            &[
                "adaptive-cold-roles-installed",
                prepared,
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
                fs::write(d.path().join(FILE), bytes)?;
            }
            crate::floating_tree::crash_tests::terminate_probe(
                PROBE,
                a.path(),
                action,
                edge,
                tick,
            )?;
            let mut r = AdaptiveGraspFleetPool::resume(exe, a.path())?;
            let mut q = AdaptiveGraspFleetPool::resume(exe, b.path())?;
            let published = matches!(edge, "after-rename" | "after-sync");
            if published {
                command(&mut q, action, tick)?;
            }
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?
            );
            if !published {
                command(&mut q, action, tick)?;
                command(&mut r, action, tick)?;
            }
            if action == "warm_join" && published {
                assert!(matches!(
                    command(&mut r, action, tick),
                    Err(ArticulationError::StaleInterval { .. })
                ));
            }
            let mut warm = 0;
            for _ in 0..255 {
                let t = r.tick();
                if action == "reset" {
                    r.advance()?;
                    q.advance()?;
                } else {
                    command(&mut r, "warm_join", t)?;
                    command(&mut q, "warm_join", t)?;
                }
                assert_eq!(
                    fs::read(a.path().join(FILE))?,
                    fs::read(b.path().join(FILE))?
                );
                if r.cross_reactions().iter().any(|r| {
                    r.reaction.normal_impulse > 0. && r.reaction.warmstart_normal_impulse > 0.
                }) {
                    assert!(spec.assemblies.iter().all(|g| {
                        r.payload_reactions(g.payload.tree.model_id)
                            .unwrap()
                            .iter()
                            .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                            .count()
                            >= 2
                    }));
                    warm += 1;
                }
            }
            if action != "reset" {
                assert!(warm >= 60);
            }
            eprintln!(
                "policy={policy:?} phase={action} killed={edge} published={published} exact_native_archives=255 bilateral_and_foreign={warm}"
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "kill either actual worker after private trials, before proof export"]
fn killed_speculative_grasp_owner_recovers_retained_prior_without_double_applying_load() -> Test {
    let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
    let exe = Path::new(&worker);
    let seed = tempfile::tempdir()?;
    assert!(crate::floating_tree::pool::worker_supports(
        exe,
        "--probe-grasp-fleet-prior-v3"
    )?);
    let mut original = held_prior(exe, seed.path(), GraspFleetMerge::WarmMembers)?;
    command(&mut original, "queue", 999)?;
    let bytes = fs::read(seed.path().join(FILE))?;
    drop(original);
    for owner in 0..2 {
        let a = tempfile::tempdir()?;
        let b = tempfile::tempdir()?;
        for d in [&a, &b] {
            fs::write(d.path().join(FILE), &bytes)?;
        }
        let mut fault = AdaptiveGraspFleetPool::resume(exe, a.path())?;
        let mut control = AdaptiveGraspFleetPool::resume(exe, b.path())?;
        let slot = fault.owners()[owner];
        fault.kill_before_proof = Some(slot);
        command(&mut fault, "warm_join", 999)?;
        command(&mut control, "warm_join", 999)?;
        assert!(!fault.member_timing().unwrap().reused_source_workers);
        assert!(control.member_timing().unwrap().reused_source_workers);
        let mut warm = 0;
        for t in 1000..1256 {
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?
            );
            command(&mut fault, "warm_join", t)?;
            command(&mut control, "warm_join", t)?;
            if fault
                .cross_reactions()
                .iter()
                .any(|r| r.reaction.normal_impulse > 0. && r.reaction.warmstart_normal_impulse > 0.)
            {
                warm += 1;
            }
        }
        assert!(warm >= 70);
        eprintln!(
            "proof-export owner={owner} slot={slot} killed_between_trial_and_export=true fallback_reconstructed=true byte_identical_after=256 subsequent_warm_foreign={warm}"
        );
    }
    Ok(())
}
