// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::moving::crash_tests::await_boundary;
use std::{fs, process::Command as Process};

const WORKER: &str = "GZ_MULTI_TEST_WORKER";
const ARCHIVE: &str = "barrier.gzmultisphere";
const CHILD: &str = "multi_sphere::crash_tests::coordinator_child";
const DRIVE: f64 = 0.01;

fn apply(pool: &mut MultiSpherePool, operation: &str) -> Result<()> {
    match operation {
        "step" => pool.advance(),
        "atomic" => pool.advance_with_effort(pool.spec().hinge.joint_id, DRIVE / 2.),
        "handoff" => pool.transfer((pool.owners()[0] + 1) % pool.width()),
        "pair handoff" => pool.transfer_pair(pool.owners()[0]),
        "torque" => pool.queue_effort(pool.spec().hinge.joint_id, DRIVE),
        _ => panic!("unknown test operation"),
    }
}

#[test]
#[ignore = "private SIGKILL harness child"]
fn coordinator_child() {
    let exe = PathBuf::from(std::env::var_os(WORKER).unwrap());
    let dir = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_SESSION").unwrap());
    let mut pool = MultiSpherePool::resume(&exe, &dir).unwrap();
    apply(
        &mut pool,
        &std::env::var("GZ_MULTI_TEST_OPERATION").unwrap(),
    )
    .unwrap();
    panic!("controller failed to reach the requested fault boundary")
}

fn check(exe: &Path, seed: &[u8], op: &str, edge: &str) -> Result<()> {
    let dir = tempfile::tempdir()?;
    let session = dir.path().join("interrupted");
    let reference = dir.path().join("reference");
    for path in [&session, &reference] {
        fs::create_dir(path)?;
        fs::write(path.join(ARCHIVE), seed)?;
    }
    let mut expected = MultiSpherePool::resume(exe, &reference)?;
    let epoch = expected.epoch();
    let tick = expected.observation().hinge.tick;
    let committed = matches!(edge, "after-rename" | "after-sync");
    if committed {
        if op == "atomic" {
            expected.queue_effort(expected.spec().hinge.joint_id, DRIVE / 2.)?;
            expected.advance()?;
        } else {
            apply(&mut expected, op)?;
        }
    }
    let marker = dir.path().join("boundary");
    let mut child = Process::new(std::env::current_exe()?)
        .args(["--exact", CHILD, "--ignored"])
        .env("GZ_MULTI_TEST_SESSION", &session)
        .env("GZ_MULTI_TEST_OPERATION", op)
        .env("GZ_MOVING_TEST_GATE", edge)
        .env("GZ_MOVING_TEST_MARKER", &marker)
        .spawn()?;
    await_boundary(&mut child, &marker, edge)?;
    assert!(
        matches!(MultiSpherePool::resume(exe,&session), Err(ArticulationError::Io(e)) if e.kind()==io::ErrorKind::WouldBlock)
    );
    child.kill()?;
    child.wait()?;
    let mut recovered = MultiSpherePool::resume(exe, &session)?;
    assert!(!session.join("barrier.pending.gzhinge").exists());
    println!(
        "recovered {op} at {edge}: tick {}, owners {:?}",
        recovered.observation().hinge.tick,
        recovered.owners()
    );
    if op == "atomic" && committed {
        let joint = recovered.spec().hinge.joint_id;
        assert!(matches!(
            recovered.advance_with_effort_at(epoch, tick, joint, DRIVE / 2.),
            Err(ArticulationError::StaleInterval { .. })
        ));
    }
    for n in 0..=4 {
        if n == 1 && op == "atomic" && !committed {
            // The reopened tick still names the pre-operation cut. Only this
            // case should resubmit the new effort. Compare to the older API.
            expected.queue_effort(expected.spec().hinge.joint_id, DRIVE / 2.)?;
            expected.advance()?;
            let joint = recovered.spec().hinge.joint_id;
            recovered.advance_with_effort_at(epoch, tick, joint, DRIVE / 2.)?;
        } else if n > 0 {
            for pool in [&mut expected, &mut recovered] {
                match pool.pending_joint_effort() {
                    Some(pending) => assert_eq!(pending, DRIVE),
                    None => apply(pool, "torque")?,
                }
                pool.advance()?;
            }
        }
        assert_eq!(recovered.owners(), expected.owners());
        assert_eq!(
            recovered.pending_joint_effort(),
            expected.pending_joint_effort()
        );
        assert_eq!(
            worker::encode(recovered.observation())?,
            worker::encode(expected.observation())?
        );
        assert_eq!(recovered.image(), expected.image());
        assert_eq!(recovered.pair_image(), expected.pair_image());
        assert_eq!(
            fs::read(session.join(ARCHIVE))?,
            fs::read(reference.join(ARCHIVE))?
        );
    }
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock worker"]
fn controller_kill_during_membership_changes_and_double_loaded_transfer() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os(WORKER).expect("stock worker path"));
    let dir = tempfile::tempdir()?;
    let spec = gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec(
        include_str!("../../../../examples/distributed_hinge_two_spheres.sdf"),
        "distributed_hinge_two_spheres.sdf",
        "default",
    )
    .unwrap();
    // The scene dev-dependency also links the non-test build of this crate.
    let spec: MultiSphereSpec =
        serde_json::from_value(serde_json::to_value(spec).unwrap()).unwrap();
    let mut pool = MultiSpherePool::create(&exe, dir.path(), 4, spec)?;
    let path = dir.path().join(ARCHIVE);
    let mut shared = false;
    let mut released = false;
    let mut changes = Vec::new();
    for tick in 0..1800 {
        let before = pool.owners();
        if !shared
            && pool
                .observation()
                .reactions
                .iter()
                .all(|r| r.iter().any(|r| r.normal_impulse > 0.))
        {
            let bytes = fs::read(&path)?;
            for gate in ["before-rename", "after-rename"] {
                check(&exe, &bytes, "torque", gate)?;
            }
            apply(&mut pool, "torque")?;
            let bytes = fs::read(&path)?;
            for gate in ["regroup-fenced", "before-rename", "after-rename"] {
                check(&exe, &bytes, "handoff", gate)?;
            }
            shared = true;
        } else {
            apply(&mut pool, "torque")?;
        }
        let bytes = fs::read(&path)?;
        if tick == 0 {
            for gate in ["after-hinge", "after-sphere-A", "after-rename"] {
                check(&exe, &bytes, "step", gate)?;
            }
        }
        pool.advance()?;
        let now = pool.owners();
        if before != now {
            changes.push((owner_mask(before), owner_mask(now)));
            for gate in ["regroup-fenced", "before-rename", "after-rename"] {
                check(&exe, &bytes, "step", gate)?;
            }
            if shared && owner_mask(now) == 0 {
                released = true;
                break;
            }
        }
    }
    assert!(shared && released);
    assert!(
        changes
            .windows(2)
            .any(|s| s[0].0 == 0 && s[0].1.count_ones() == 1 && s[1].1 == 3)
    );
    assert!(
        changes
            .windows(2)
            .any(|s| s[0].0 == 3 && s[0].1.count_ones() == 1 && s[1].1 == 0)
    );
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock worker"]
fn controller_kill_during_pair_formation_warm_transfer_and_merge() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os(WORKER).expect("stock worker path"));
    let dir = tempfile::tempdir()?;
    let spec = gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec(
        include_str!("../../../../examples/distributed_sphere_pair_island.sdf"),
        "distributed_sphere_pair_island.sdf",
        "default",
    )
    .unwrap();
    let spec: MultiSphereSpec =
        serde_json::from_value(serde_json::to_value(spec).unwrap()).unwrap();
    let mut p = MultiSpherePool::create(&exe, dir.path(), 4, spec)?;
    let mut joined = false;
    let mut loaded = false;
    let mut merged = false;
    let path = dir.path().join(ARCHIVE);
    for _ in 0..800 {
        apply(&mut p, "torque")?;
        let before = fs::read(&path)?;
        let pair = p.pair_image().is_some();
        let reaction = p.observation().sphere_reaction;
        if !loaded && pair && reaction.normal_impulse > 0. && reaction.warmstart_normal_impulse > 0.
        {
            for gate in ["regroup-fenced", "before-rename", "after-rename"] {
                check(&exe, &before, "pair handoff", gate)?;
            }
            loaded = true;
        }
        p.advance()?;
        if !joined && !pair && p.pair_image().is_some() {
            for gate in [
                "regroup-fenced",
                "pair-previewed",
                "after-hinge",
                "after-pair",
                "before-rename",
                "after-rename",
            ] {
                check(&exe, &before, "step", gate)?;
            }
            joined = true;
        }
        if pair && p.owners() == [p.owners()[0]; 3] {
            for gate in [
                "pair-merge-prepared",
                "regroup-fenced",
                "before-rename",
                "after-rename",
            ] {
                check(&exe, &before, "step", gate)?;
            }
            merged = true;
            break;
        }
    }
    assert!(joined && loaded && merged);

    // This frozen v1 barrier has a warm sphere-sphere manifold and no hinge
    // contact. It must be possible to recover either side of extracting the
    // whole pair while the next torque is pending.
    let old = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../artifacts/multifidelity-two-sphere-hinge/four/cuts/tick-1292")
        .join(ARCHIVE);
    let seed = fs::read(old)?;
    let legacy = tempfile::tempdir()?;
    fs::write(legacy.path().join(ARCHIVE), &seed)?;
    let mut restored = MultiSpherePool::resume(&exe, legacy.path())?;
    assert_eq!(restored.owners(), [1; 3]);
    assert!(
        restored
            .observation()
            .sphere_reaction
            .warmstart_normal_impulse
            > 0.
    );
    restored.advance()?;
    assert!(restored.pair_image().is_some());
    for gate in [
        "regroup-fenced",
        "after-hinge",
        "after-pair",
        "before-rename",
        "after-rename",
    ] {
        check(&exe, &seed, "step", gate)?;
    }

    // This oblique impact eventually leaves two freely moving bodies with
    // enough clearance to fence the pair and reinstall both single-body worlds.
    let mut scattering = spec;
    scattering.spheres[0].position = [4., 0., 0.5];
    scattering.spheres[1].position = [4.35, 0.17, 0.5];
    scattering.spheres[0].velocity = [3., 0., 0.];
    scattering.spheres[1].velocity = [-3., 0., 0.];
    let dir = tempfile::tempdir()?;
    let mut p = MultiSpherePool::create(&exe, dir.path(), 3, scattering)?;
    let mut scattered = false;
    for _ in 0..250 {
        apply(&mut p, "torque")?;
        let was_pair = p.pair_image().is_some();
        let seed = fs::read(dir.path().join(ARCHIVE))?;
        p.advance()?;
        if was_pair && p.pair_image().is_none() {
            let [h, a, b] = p.owners();
            assert!(h != a && h != b && a != b);
            for gate in [
                "regroup-fenced",
                "after-sphere-A",
                "before-rename",
                "after-rename",
            ] {
                check(&exe, &seed, "step", gate)?;
            }
            scattered = true;
            break;
        }
    }
    assert!(scattered);
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock worker"]
fn controller_kill_during_atomic_effort_and_advance() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os(WORKER).expect("stock worker path"));
    let dir = tempfile::tempdir()?;
    let spec = gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec(
        include_str!("../../../../examples/distributed_sphere_pair_island.sdf"),
        "distributed_sphere_pair_island.sdf",
        "default",
    )
    .unwrap();
    let spec: MultiSphereSpec =
        serde_json::from_value(serde_json::to_value(spec).unwrap()).unwrap();
    let mut p = MultiSpherePool::create(&exe, dir.path(), 4, spec)?;
    let joint = spec.hinge.joint_id;
    let path = dir.path().join(ARCHIVE);
    for tick in 0..=1213 {
        let gates: &[&str] = match tick {
            0 => &["atomic-effort-accepted", "after-hinge", "after-sphere-A"],
            124 => &["regroup-fenced", "pair-previewed", "after-pair"],
            149 => &[
                "atomic-effort-accepted",
                "pair-previewed",
                "after-hinge",
                "after-pair",
            ],
            150 => &["pair-merge-prepared", "regroup-fenced", "after-hinge"],
            1213 => &["regroup-fenced", "pair-previewed", "after-pair"],
            _ => &[],
        };
        if gates.is_empty() {
            p.advance_with_effort(joint, DRIVE)?;
            continue;
        }
        if tick != 0 {
            p.queue_effort(joint, DRIVE / 2.)?;
        }
        let seed = fs::read(&path)?;
        for gate in gates
            .iter()
            .copied()
            .chain(["before-rename", "after-rename", "after-sync"])
        {
            check(&exe, &seed, "atomic", gate)?;
        }
        // Generate the following crash seed on the original driven trajectory.
        p.advance_with_effort(joint, if tick == 0 { DRIVE } else { DRIVE / 2. })?;
        if tick == 149 {
            assert!(p.pair_image().is_some());
            assert!(p.observation().sphere_reaction.warmstart_normal_impulse > 0.);
        } else if tick == 1213 {
            assert!(p.pair_image().is_some());
            assert!(p.observation().sphere_reaction.active);
        }
    }
    Ok(())
}
