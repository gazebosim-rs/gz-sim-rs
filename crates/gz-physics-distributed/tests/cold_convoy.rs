// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cold attachment binds an operator-selected cut to its original writer.
#![cfg(unix)]
use gz_physics_distributed::articulation::ArticulationError;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    LocalPose, Velocity, WorldPose,
    distributed_hinge::{
        ColdOrigin, ColdSelection, HingeWorkerOptions, MultiSpherePool, WorkerTransport,
        distributed_multi_sphere_spec, inspect_restart_cut, preview_selected_hinge_world,
        resume_selected_hinge_world,
    },
};
use std::{error::Error, fs, io, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";

#[test]
fn origin_and_digest_bind_inspection_and_install_without_discarding_staging() -> Result<()> {
    let root = tempfile::tempdir()?;
    let dir = root.path().join("original");
    let other = root.path().join("copy");
    let staging = dir.join("barrier.pending.gzhinge");
    let archive = dir.join(ARCHIVE);
    let claim = dir.join("controller.lock");
    let spec = distributed_multi_sphere_spec(SDF, "cold.sdf", "default")?;
    let worker = Path::new(WORKER);
    let mut live = MultiSpherePool::create(worker, &dir, 3, spec)?;
    let origin = ColdOrigin::capture(&dir)?;
    let cold = fs::read(&archive)?;
    live.queue_effort(spec.hinge.joint_id, 0.015)?;
    let warm = fs::read(&archive)?;
    assert_ne!(cold, warm);
    live.verify_live_cut(&origin, &warm)?;
    assert!(live.verify_live_cut(&origin, &[]).is_err());
    // Both exact valid files describe the same epoch and physical tick. Only
    // the second contains the new torque acknowledged by this live pool.
    fs::write(&archive, &cold)?;
    assert!(live.verify_live_cut(&origin, &fs::read(&archive)?).is_err());
    assert_eq!(fs::read(&archive)?, cold);
    fs::write(&archive, &warm)?;
    let cut = inspect_restart_cut(warm.as_slice())?;
    let digest = cut.sha256.as_str();
    assert_eq!(ColdOrigin::capture(&dir)?, origin);
    assert!(
        matches!(ColdSelection::inspect(&dir, digest, &origin), Err(ArticulationError::Io(e)) if e.kind()==io::ErrorKind::WouldBlock)
    );
    drop(live);
    fs::write(&staging, b"unconfirmed successor")?;
    fs::create_dir(&other)?;
    fs::write(other.join(ARCHIVE), &warm)?;
    fs::copy(&claim, other.join("controller.lock"))?;
    assert!(ColdSelection::inspect(&other, digest, &origin).is_err());
    assert!(ColdSelection::inspect(&dir, "wrong", &origin).is_err());
    let selected = ColdSelection::inspect(&dir, digest, &origin)?;
    let try_attach = |dir: &Path, width, spec| {
        MultiSpherePool::resume_scene_selected(
            worker,
            dir,
            width,
            spec,
            WorkerTransport::Pipe,
            &selected,
        )
    };
    let busy = fs::File::open(&claim)?;
    busy.try_lock()?;
    assert!(
        matches!(try_attach(&dir,None,spec),Err(ArticulationError::Io(e)) if e.kind()==io::ErrorKind::WouldBlock)
    );
    drop(busy);
    assert!(try_attach(&other, None, spec).is_err());
    assert!(try_attach(&dir, Some(4), spec).is_err());
    let mut wrong = spec;
    wrong.hinge.joint_id += 50;
    assert!(try_attach(&dir, None, wrong).is_err());
    let mut signed = spec;
    let axis = &mut signed.hinge.axis[0];
    assert_eq!(*axis, 0.);
    *axis = -*axis;
    assert_eq!(signed, spec);
    assert!(!spec.same_recipe(&signed)?);
    assert!(try_attach(&dir, None, signed).is_err());
    fs::write(&archive, &cold)?;
    assert!(try_attach(&dir, None, spec).is_err());
    assert_eq!(fs::read(&archive)?, cold);
    assert_eq!(fs::read(&staging)?, b"unconfirmed successor");
    fs::write(&archive, &warm)?;
    let original_archive = dir.join("original-cut");
    fs::rename(&archive, &original_archive)?;
    std::os::unix::fs::symlink(&original_archive, &archive)?;
    assert!(ColdSelection::inspect(&dir, digest, &origin).is_err());
    assert!(try_attach(&dir, None, spec).is_err());
    assert_eq!(fs::read(&staging)?, b"unconfirmed successor");
    fs::remove_file(&archive)?;
    fs::rename(&original_archive, &archive)?;
    let original_claim = dir.join("old.lock");
    fs::rename(&claim, &original_claim)?;
    fs::copy(&original_claim, &claim)?;
    assert!(ColdSelection::inspect(&dir, digest, &origin).is_err());
    assert!(try_attach(&dir, None, spec).is_err());
    fs::remove_file(&claim)?;
    fs::rename(&original_claim, &claim)?;
    fs::remove_file(other.join("controller.lock"))?;
    fs::hard_link(&claim, other.join("controller.lock"))?;
    let after_link = ColdOrigin::capture(&dir)?;
    assert!(ColdSelection::inspect(&other, digest, &after_link).is_err());
    assert_eq!(fs::read(&staging)?, b"unconfirmed successor");

    let missing = root.path().join("missing");
    assert!(ColdSelection::inspect(&missing, digest, &origin).is_err());
    assert!(!missing.exists());
    let bare = root.path().join("bare");
    fs::create_dir(&bare)?;
    fs::copy(&archive, bare.join(ARCHIVE))?;
    assert!(ColdSelection::inspect(&bare, digest, &origin).is_err());
    assert!(!bare.join("controller.lock").exists());
    Ok(())
}

#[allow(deprecated)]
fn compare_poses(preview: &Simulation, live: &Simulation) {
    assert_eq!(live.epoch(), preview.epoch());
    assert_eq!(live.elapsed(), preview.elapsed());
    let mut total = 0;
    for entity in live.world().iter_entities() {
        let Some(id) = entity.get::<EntityId>() else {
            continue;
        };
        let other = preview
            .world()
            .entity(entity_by_id(preview.world(), id.get()).unwrap());
        if let Some(pose) = entity.get::<WorldPose>() {
            let actual = other.get::<WorldPose>().unwrap();
            assert_eq!(
                (actual.translation, actual.rotation),
                (pose.translation, pose.rotation)
            );
            total += 1;
        }
        if let Some(pose) = entity.get::<LocalPose>() {
            let actual = other.get::<LocalPose>().unwrap().0;
            assert_eq!(
                (actual.translation, actual.rotation),
                (pose.0.translation, pose.0.rotation)
            );
        }
        if let Some(velocity) = entity.get::<Velocity>() {
            let actual = other.get::<Velocity>().unwrap();
            assert_eq!(
                (actual.linear, actual.angular),
                (velocity.linear, velocity.angular)
            );
        }
    }
    assert!(total >= 12);
}

#[test]
fn loaded_offset_preview_matches_live_ecs_and_queued_successor() -> Result<()> {
    // A nonzero authored offset and rotated visual exercise link-local propagation.
    let sdf = SDF.replace(
        "<visual name=\"tool_visual\">",
        "<visual name=\"tool_visual\"><pose>0.08 0.02 0 0 0 0.31</pose>",
    );
    let root = tempfile::tempdir()?;
    let dir = root.path().join("live");
    let reference = root.path().join("reference");
    let worker = Path::new(WORKER);
    let spec = distributed_multi_sphere_spec(&sdf, "offset.sdf", "default")?;
    let mut control = MultiSpherePool::create(worker, &reference, 3, spec)?;
    let mut original = MultiSpherePool::create(worker, &dir, 3, spec)?;
    for (session, pool) in [(&reference, &mut control), (&dir, &mut original)] {
        for t in 0..150 {
            pool.advance_with_effort_at(0, t, spec.hinge.joint_id, -0.01)?;
        }
        assert!(pool.observation().sphere_reaction.normal_impulse > 0.49);
        let origin = ColdOrigin::capture(session)?;
        let before_transfer = fs::read(session.join(ARCHIVE))?;
        pool.verify_live_cut(&origin, &before_transfer)?;
        let physical = *pool.observation();
        pool.transfer_pair(pool.owners()[0])?;
        assert_eq!(*pool.observation(), physical);
        assert!(pool.verify_live_cut(&origin, &before_transfer).is_err());
        pool.queue_effort(spec.hinge.joint_id, 0.015)?;
    }
    let archive = dir.join(ARCHIVE);
    let warm = fs::read(&archive)?;
    let other_origin = ColdOrigin::capture(&reference)?;
    assert!(original.verify_live_cut(&other_origin, &warm).is_err());
    original.verify_live_cut(&ColdOrigin::capture(&dir)?, &warm)?;
    let origin = ColdOrigin::capture(&dir)?;
    let inspection = inspect_restart_cut(warm.as_slice())?;
    let mut preview = preview_selected_hinge_world(&sdf, "offset.sdf", "default", &inspection)?;
    assert!(preview.is_paused());
    assert!(
        preview_selected_hinge_world(
            &sdf.replace("<mass>2</mass>", "<mass>3</mass>"),
            "bad.sdf",
            "default",
            &inspection
        )
        .is_err()
    );
    drop(original);
    let selected = ColdSelection::inspect(&dir, &inspection.sha256, &origin)?;
    let mut live = resume_selected_hinge_world(
        &sdf,
        "offset.sdf",
        "default",
        worker,
        &dir,
        HingeWorkerOptions {
            width: Some(3),
            ..Default::default()
        },
        &selected,
    )?;
    assert!(live.is_paused());
    compare_poses(&preview, &live);
    assert_eq!(fs::read(&archive)?, warm);
    assert!(preview.step_exact(1).is_err());
    assert_eq!(fs::read(&archive)?, warm);
    live.step_exact(1)?;
    control.advance()?;
    assert_eq!(fs::read(&archive)?, fs::read(reference.join(ARCHIVE))?);
    Ok(())
}

#[test]
fn named_scene_with_rotated_local_visuals_previews_construction_and_reset_epochs() -> Result<()> {
    use gz_sim_scene::{distributed_hinge::load_distributed_hinge_world, reset_loaded_world};
    let xml = SDF
        .replace("<world name=\"default\">", "<world name=\"cell_a\">")
        .replace(
            "<visual name=\"base_visual\">",
            "<visual name=\"base_visual\"><pose>0.011 -0.05 0.03 0.11 -0.07 0.95</pose>",
        )
        .replace(
            "<visual name=\"ball2_visual\">",
            "<visual name=\"ball2_visual\"><pose>-0.02 0.08 0.01 0.07 -0.21 0.54</pose>",
        );
    let dir = tempfile::tempdir()?;
    let mut live = load_distributed_hinge_world(
        &xml,
        "named.sdf",
        "cell_a",
        Path::new(WORKER),
        dir.path(),
        Some(3),
    )?;
    for epoch in [0, 1] {
        if epoch == 1 {
            live.step_exact(151)?;
            reset_loaded_world(&mut live, |_| ())?;
        }
        let inspector = inspect_restart_cut(fs::File::open(dir.path().join(ARCHIVE))?)?;
        assert_eq!(inspector.cut.epoch, epoch);
        assert_eq!(inspector.cut.observation.hinge.tick, 0);
        let preview = preview_selected_hinge_world(&xml, "named.sdf", "cell_a", &inspector)?;
        assert_eq!(preview.epoch(), epoch);
        assert!(preview.is_paused());
        compare_poses(&preview, &live);
    }
    Ok(())
}
