// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check signed effort at a recovered separation against the ordinary SDF world.
#[path = "../examples/moving_hinge/reference.rs"]
mod reference;

use gz_physics_distributed::moving::{MovingObservation, MovingPool, MovingSpec};
use gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec;
use reference::Reference;
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOURCE: &str = "distributed_hinge_impact.sdf";
const SDF: &str = include_str!("../../../examples/distributed_hinge_impact.sdf");
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
const ARCHIVE: &str = "barrier.gzmoving";

fn spec() -> Result<MovingSpec> {
    Ok(distributed_moving_hinge_spec(SDF, SOURCE, "default")?)
}

#[test]
#[ignore = "requires GZ_MOVING_LEGACY_ARTIFACTS pointing to the retained artifacts directory"]
fn previous_moving_archives_preserve_ownership_and_pending_torque_on_reopen() -> Result<()> {
    let root = std::env::var_os("GZ_MOVING_LEGACY_ARTIFACTS").ok_or("missing archive location")?;
    let spec = spec()?;
    for (tick, archive, owners, next_separate) in [
        (
            4,
            "multifidelity-hinge-release/legacy/split-effort-4.gzmoving",
            [0, 1],
            // The saved −1125 N·m exceeds the driven speed certificate on the next tick.
            false,
        ),
        (
            205,
            "multifidelity-moving-hinge/four-forward/cuts/tick-205/barrier.gzmoving",
            [1, 1],
            false,
        ),
        (
            700,
            "multifidelity-hinge-release/legacy/still-joined-700.gzmoving",
            [1, 1],
            true,
        ),
        (
            1271,
            "multifidelity-moving-hinge/four-forward/cuts/tick-1271/barrier.gzmoving",
            [1, 2],
            true,
        ),
    ] {
        let directory = tempfile::tempdir()?;
        let before = fs::read(Path::new(&root).join(archive))?;
        let path = directory.path().join(ARCHIVE);
        fs::write(&path, &before)?;
        let mut pool = MovingPool::open_scene(Path::new(WORKER), directory.path(), None, spec)?;
        let mut native = Reference::load(&spec)?;
        for t in 1..=tick {
            native.advance()?;
            if t == 205 {
                native.queue(spec.hinge.joint_id, 2.25)?;
            }
        }
        if tick == 4 {
            native.queue(spec.hinge.joint_id, -1125.)?;
        }
        assert_eq!(fs::read(path)?, before, "opening mutated the legacy file");
        assert_eq!(*pool.observation(), native.observe(&spec, tick)?);
        assert_eq!(pool.owners(), owners);
        assert_eq!(
            pool.pending_joint_effort(),
            match tick {
                4 => Some(-1125.),
                205 => Some(2.25),
                _ => None,
            }
        );
        for step in 1..=16 {
            pool.advance()?;
            native.advance()?;
            assert_eq!(*pool.observation(), native.observe(&spec, tick + step)?);
            if step == 1 {
                assert_eq!(pool.pending_joint_effort(), None);
                let [hinge, sphere] = pool.owners();
                println!(
                    "legacy tick {tick}, owners {owners:?} -> [{hinge}, {sphere}], exact first step"
                );
                assert_eq!(hinge != sphere, next_separate);
            }
        }
    }
    Ok(())
}

#[test]
fn recovered_split_torque_causes_a_second_loaded_impact_and_handoff() -> Result<()> {
    const KICK_AT: u64 = 500;
    const KICK_NM: f64 = -10_000.;
    let directory = tempfile::tempdir()?;
    let exe = Path::new(WORKER);
    let spec = spec()?;
    let mut native = Reference::load(&spec)?;
    let mut coasting = Reference::load(&spec)?;
    let mut pool = MovingPool::create(exe, directory.path(), 3, spec)?;
    let mut second_impact = None;
    let mut split = None;
    let mut joined = None;
    for tick in 1..=780 {
        let owners_before = pool.owners();
        pool.advance()?;
        native.advance()?;
        coasting.advance()?;
        let expected = native.observe(&spec, tick)?;
        assert_eq!(*pool.observation(), expected, "tick {tick}");
        let owners = pool.owners();
        let coupled = owners[0] == owners[1];
        if owners_before != owners {
            println!("tick {tick}: owners {owners_before:?} -> {owners:?}");
            if !coupled && tick > 205 && split.is_none() {
                split = Some(tick);
            }
            if coupled && tick > KICK_AT {
                joined = Some(tick);
            }
        }
        if tick == 205 {
            // Reproduce the qualified first impact and its queued drive.
            assert!(expected.reactions[1].normal_impulse > 1.);
            for world in [&mut native, &mut coasting] {
                world.queue(spec.hinge.joint_id, 2.25)?;
            }
            pool.queue_effort(spec.hinge.joint_id, 2.25)?;
        }
        if tick == KICK_AT {
            assert!(
                !coupled,
                "expected the tightened bound to have released the hinge"
            );
            assert!(split.is_some());
            pool.drop_next_acknowledgement(owners[0])?;
            pool.queue_effort(spec.hinge.joint_id, KICK_NM)?;
            native.queue(spec.hinge.joint_id, KICK_NM)?;
            drop(pool);
            pool = MovingPool::resume(exe, directory.path())?;
            assert_eq!(pool.owners(), owners);
            assert_eq!(*pool.observation(), expected);
        }
        if tick > KICK_AT {
            let unforced = coasting.observe(&spec, tick)?;
            assert!(
                unforced
                    .reactions
                    .iter()
                    .all(|r| !r.active && r.normal_impulse == 0.)
            );
            if tick == KICK_AT + 1 {
                assert!((expected.hinge.qdot - unforced.hinge.qdot).abs() > 100.);
            }
            if expected.reactions.iter().any(|r| r.normal_impulse > 0.) && second_impact.is_none() {
                assert!(coupled && joined.is_some_and(|t| t < tick));
                assert!(expected.reactions[1].warmstart_normal_impulse > 0.);
                let image = pool.image().unwrap().to_owned();
                let destination = (owners[0] + 1) % pool.width();
                pool.transfer(destination)?;
                assert_eq!(pool.image(), Some(image.as_slice()));
                drop(pool);
                pool = MovingPool::resume(exe, directory.path())?;
                assert_eq!(pool.owners(), [destination; 2]);
                assert_eq!(pool.image(), Some(image.as_slice()));
                assert_eq!(*pool.observation(), expected);
                second_impact = Some(tick);
                println!(
                    "second impact tick {tick}: normal {} N·s, cached {} N·s, migrated to {destination}, restored {} bytes",
                    expected.reactions[1].normal_impulse,
                    expected.reactions[1].warmstart_normal_impulse,
                    image.len()
                );
            }
        }
    }
    assert!(
        second_impact.is_some_and(|t| t <= 764),
        "expected at least 16 exact post-impact steps"
    );
    Ok(())
}

#[test]
fn cancelling_efforts_and_rejected_overflow_survive_reopen() -> Result<()> {
    let spec = spec()?;
    let dir = tempfile::tempdir()?;
    let archive = dir.path().join(ARCHIVE);
    let worker = Path::new(WORKER);
    let joint = spec.hinge.joint_id;
    let mut pool = MovingPool::create(worker, dir.path(), 3, spec)?;
    let mut ordinary = Reference::load(&spec)?;
    let mut loaded = false;
    for tick in 0..240 {
        let has_load = pool
            .observation()
            .reactions
            .iter()
            .any(|r| r.normal_impulse > 0.);
        if tick < 2 || has_load {
            // Exercise the cold first image, the compact warm image and the coupled image.
            assert_eq!(pool.pending_joint_effort(), None);
            pool.queue_effort(joint, f64::MAX)?;
            let before = fs::read(&archive)?;
            assert!(pool.queue_effort(joint, f64::MAX).is_err());
            assert_eq!(pool.pending_joint_effort(), Some(f64::MAX));
            assert_eq!(fs::read(&archive)?, before);
            pool.queue_effort(joint, -f64::MAX)?;
            assert_eq!(pool.pending_joint_effort(), Some(0.));
            drop(pool);
            pool = MovingPool::resume(worker, dir.path())?;
            assert_eq!(pool.pending_joint_effort(), Some(0.));
            if has_load {
                let destination = (pool.owners()[0] + 1) % pool.width();
                pool.transfer(destination)?;
                assert_eq!(pool.pending_joint_effort(), Some(0.));
                loaded = true;
            }
            ordinary.queue(joint, 0.)?;
        }
        pool.advance()?;
        ordinary.advance()?;
        assert_eq!(pool.pending_joint_effort(), None);
        assert_eq!(
            bincode::serialize(pool.observation())?,
            bincode::serialize(&ordinary.observe(&spec, tick + 1)?)?
        );
        if loaded {
            break;
        }
    }
    assert!(loaded, "expected a coupled loaded receipt");
    Ok(())
}
