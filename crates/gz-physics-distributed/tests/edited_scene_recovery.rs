// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "private/passive_pose_proof.rs"]
mod passive_pose_proof;
#[path = "../examples/edited_scene_recovery/scenario.rs"]
mod scenario;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{SceneBackend, pool::PoolNativeRecovery};
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, Name, WorldRuntimeOptions, load_world_with_runtime,
    passive_checkpoint::{PassivePoolScene, RestoredTemplates},
    prepare_model_template, remove_model, reset_loaded_world, spawn_model_template,
};
use scenario::{Result, SDF};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{fs, path::Path};
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn resume(w: u8, p: &Path) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::resume(
        SDF,
        "moved.sdf",
        "default",
        scenario::options(worker(), w),
        Default::default(),
        p,
    )?)
}
fn name(sim: &Simulation, scoped: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == scoped))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}

// Drive the preexisting public Bevy lifecycle directly, without the checkpoint
// owner's recipe, allocator, template or restoration implementation.
fn ordinary(width: u8, worker_path: &Path) -> Result<Simulation> {
    let mut sim = load_world_with_runtime(
        SDF,
        "stack.sdf",
        "default",
        &sdformat::NoResources,
        scenario::options(worker_path, width),
        WorldRuntimeOptions::default(),
    )?;
    let sdf = sdformat::load_str(SDF, "stack.sdf", &sdformat::NoResources)?;
    let first = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[1])?;
    let second = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[2])?;
    assert_eq!(
        spawn_model_template(&mut sim, &first, scenario::pose(-3., 0., 2.))?,
        14
    );
    assert_eq!(
        spawn_model_template(&mut sim, &second, scenario::pose(3., 0., 2.))?,
        18
    );
    sim.step_exact(4)?;
    remove_model(&mut sim, 14)?;
    reset_loaded_world(&mut sim, |_| ())?;
    sim.step_exact(55)?;
    assert_eq!(
        spawn_model_template(&mut sim, &first, scenario::pose(2., -2., 4.))?,
        22
    );
    remove_model(&mut sim, 22)?;
    assert_eq!(
        spawn_model_template(&mut sim, &first, scenario::pose(0., 0., 4.5))?,
        26
    );
    sim.step_exact(30)?;
    remove_model(&mut sim, 6)?;
    sim.step_exact(25)?;
    let mut birth = scenario::pose(1.7, 1.2, 2.25);
    birth.rotation = [0., 0., 0.175_f64.sin(), 0.175_f64.cos()];
    assert_eq!(spawn_model_template(&mut sim, &first, birth)?, 30);
    let body = sim
        .world()
        .get::<BodyHandle>(entity_by_id(sim.world(), 27).unwrap())
        .unwrap()
        .id();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .queue_linear_impulse(body, [0., 0.0125, 0.])?;
    Ok(sim)
}

#[test]
fn sparse_active_clones_loaded_retirements_and_original_blueprint_match_unrestricted_bevy()
-> Result<()> {
    let dir = tempfile::tempdir()?;
    let baseline = std::env::var_os("GZ_BEFORE_SCENE_WORKER").map(std::path::PathBuf::from);
    for width in [3, 4, 8] {
        let (mut source, receipt) = scenario::build(worker(), width)?;
        assert_eq!(
            (
                receipt.prior_epoch,
                receipt.retired_runtime,
                receipt.airborne_clone,
                receipt.newborn_clone,
                receipt.next_public,
                receipt.next_native
            ),
            ([14, 18], 22, 26, 30, 34, 8)
        );
        let mut reference = ordinary(width, baseline.as_deref().unwrap_or(worker()))?;
        assert_eq!(
            passive_pose_proof::proof(source.simulation()),
            passive_pose_proof::proof(&reference)
        );
        let path = dir.path().join(width.to_string());
        source.save(&path)?;
        assert_eq!(&fs::read(&path)?[..8], b"GZSCN002");
        assert!(PoolNativeRecovery::open_scene(&path).is_err());
        drop(source);
        let mut s = resume(width, &path)?;
        let mut hash = Sha256::new();
        let mut pair_receipts = 0usize;
        for n in 0..=160 {
            let current = passive_pose_proof::proof(s.simulation());
            assert_eq!(
                current,
                passive_pose_proof::proof(&reference),
                "width {width} successor {n}"
            );
            let a = backend(s.simulation());
            let b = backend(&reference);
            assert_eq!(a.last_substeps(), b.last_substeps());
            assert_eq!(
                a.last_island_refined_bodies(),
                b.last_island_refined_bodies()
            );
            hash.update(current);
            pair_receipts += a
                .pool_frame()
                .unwrap()
                .workers
                .iter()
                .map(|w| w.snapshot.impulses.len())
                .sum::<usize>();
            if n < 160 {
                s.step_exact(1)?;
                reference.step_exact(1)?;
            }
        }
        assert!(pair_receipts > 0);
        assert!(entity_by_id(s.simulation().world(), 6).is_none());
        assert_eq!(s.spawn(6, scenario::pose(4., -1., 4.))?, 34);
        s.retire(34)?;
        s.reset_loaded()?;
        assert_eq!(s.simulation().epoch(), 2);
        assert_eq!(name(s.simulation(), "default::first"), 6);
        for id in [14, 18, 22, 26, 30, 34] {
            assert!(entity_by_id(s.simulation().world(), id).is_none());
        }
        assert_eq!(next_entity_id(s.simulation().world())?, 38);
        // All native workers have fresh unedited histories, but the ECS retains
        // the higher external body/public high-water marks across this reset.
        let floor = dir.path().join(format!("reset-{width}"));
        s.save(&floor)?;
        drop(s);
        let mut s = resume(width, &floor)?;
        assert_eq!(backend(s.simulation()).next_body_handle(), 9);
        assert_eq!(next_entity_id(s.simulation().world())?, 38);
        s.retire(10)?;
        let retired = dir.path().join(format!("retired-after-reset-{width}"));
        s.save(&retired)?;
        drop(s);
        let s = resume(width, &retired)?;
        let mut sim = s.into_simulation();
        let original = sim
            .world()
            .resource::<RestoredTemplates>()
            .by_name("default::second")
            .unwrap();
        assert!(entity_by_id(sim.world(), 10).is_none());
        let fresh = spawn_model_template(&mut sim, &original, scenario::pose(4., 0., 3.))?;
        assert_eq!(fresh, 38);
        assert_eq!(backend(&sim).next_body_handle(), 10);
        println!(
            "edited width={width} states=161 pair_receipts={pair_receipts} next_control=38 restored_epoch=2 trajectory_sha256={:x}",
            hash.finalize()
        );
    }
    Ok(())
}

fn mutation(
    bytes: &[u8],
    file: &Path,
    edit: impl FnOnce(&mut Value, &mut Vec<u8>, &mut Value, &mut Vec<u8>),
) -> Result<()> {
    let n = u32::from_le_bytes(bytes[40..44].try_into()?) as usize;
    let mut outer = bytes[..8].to_vec();
    let mut manifest: Value = serde_json::from_slice(&bytes[44..44 + n])?;
    let mut tag = bytes[44 + n..52 + n].to_vec();
    let mut native: Value = serde_json::from_slice(&bytes[52 + n..])?;
    edit(&mut manifest, &mut tag, &mut native, &mut outer);
    let m = serde_json::to_vec(&manifest)?;
    let mut payload = (m.len() as u32).to_le_bytes().to_vec();
    payload.extend(m);
    payload.extend(tag);
    payload.extend(serde_json::to_vec(&native)?);
    outer.extend(Sha256::digest(&payload));
    outer.extend(payload);
    fs::write(file, outer)?;
    Ok(())
}

#[test]
fn rejects_resealed_recipe_allocator_and_template_inconsistencies_before_publication() -> Result<()>
{
    let dir = tempfile::tempdir()?;
    let good = dir.path().join("valid");
    let (mut source, _) = scenario::build(worker(), 4)?;
    source.save(&good)?;
    drop(source);
    let bytes = fs::read(&good)?;
    let bad = dir.path().join("bad");
    for case in 0..13 {
        mutation(&bytes, &bad, |m, tag, native, outer| match case {
            0 => m["population"]["next_native"] = 7.into(),
            1 => m["population"]["epoch_start_native"] = 9.into(),
            2 => m["population"]["clones"][0]["model"] = 25.into(),
            3 => m["population"]["clones"][0]["source_visual"] = 13.into(),
            4 => m["population"]["clones"][0]["birth_tick"] = 111.into(),
            5 => m["population"]["clones"][0]["source_model"] = 2.into(),
            6 => m["population"]["clones"][1]["translation"][0] = 3.into(),
            7 => m["population"]["retired"] = serde_json::json!([6, 6]),
            8 => m["population"]["retired"] = serde_json::json!([14]),
            9 => m["bindings"]["next_public_id"] = 38.into(),
            10 => *outer = b"GZSCN001".to_vec(),
            11 => {
                *tag = b"GZPOOL01".to_vec();
                native
                    .as_object_mut()
                    .unwrap()
                    .remove("population_high_water");
            }
            12 => native["population_high_water"] = 8.into(),
            _ => unreachable!(),
        })?;
        assert!(resume(4, &bad).is_err(), "case {case}");
    }
    assert!(resume(4, &good).is_ok());
    Ok(())
}

#[test]
fn births_separated_by_a_paused_ecs_publication_preserve_each_local_transform() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let (mut s, _) = scenario::build(worker(), 4)?;
    s.tick()?;
    let mut another = scenario::pose(-2.25, 1.45, 3.1);
    another.rotation = [0., 0., 0.117_f64.sin(), 0.117_f64.cos()];
    assert_eq!(s.spawn(6, another)?, 34);
    let path = dir.path().join("between");
    let before = passive_pose_proof::proof(s.simulation());
    s.save(&path)?;
    drop(s);
    let mut restored = resume(4, &path)?;
    assert_eq!(passive_pose_proof::proof(restored.simulation()), before);
    restored.tick()?;
    let next = dir.path().join("published");
    let published = passive_pose_proof::proof(restored.simulation());
    restored.save(&next)?;
    drop(restored);
    assert_eq!(
        passive_pose_proof::proof(resume(4, &next)?.simulation()),
        published
    );
    Ok(())
}

#[test]
fn sparse_allocator_at_last_complete_u32_model_survives_restart_and_loaded_reset() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let mut center = scenario::build(worker(), 4)?.0;
    center.reset_loaded()?;
    let reset = dir.path().join("ordinary-reset");
    center.save(&reset)?;
    drop(center);
    // A seed's model roots are 14 modulo 4. This is the last root whose
    // four public identities fit in the viewer protocol's u32 space.
    let last_model = (u64::from(u32::MAX) - 3) - (u64::from(u32::MAX) - 3 - 14) % 4;
    let next_native = 3 + (last_model - 14) / 4;
    let far = dir.path().join("sparse-floor");
    mutation(&fs::read(reset)?, &far, |m, _, _, _| {
        m["population"]["next_native"] = next_native.into();
        m["population"]["epoch_start_native"] = next_native.into();
        m["bindings"]["next_public_id"] = last_model.into();
    })?;
    let mut s = resume(4, &far)?;
    assert_eq!(next_entity_id(s.simulation().world())?, last_model);
    assert_eq!(backend(s.simulation()).next_body_handle(), next_native);
    assert_eq!(s.spawn(6, scenario::pose(2., 2., 3.))?, last_model);
    let next_public = last_model + 4;
    assert_eq!(next_entity_id(s.simulation().world())?, next_public);
    let sealed = dir.path().join("last-live-model");
    s.save(&sealed)?;
    drop(s);
    let mut s = resume(4, &sealed)?;
    let before = passive_pose_proof::proof(s.simulation());
    assert!(s.spawn(6, scenario::pose(-2., 2., 3.)).is_err());
    assert_eq!(passive_pose_proof::proof(s.simulation()), before);
    assert_eq!(backend(s.simulation()).next_body_handle(), next_native + 1);
    assert!(entity_by_id(s.simulation().world(), last_model + 3).is_some());
    s.retire(last_model)?;
    s.reset_loaded()?;
    let reset = dir.path().join("final-reset");
    s.save(&reset)?;
    drop(s);
    let mut s = resume(4, &reset)?;
    assert_eq!(next_entity_id(s.simulation().world())?, next_public);
    assert!(s.spawn(6, scenario::pose(2., 1., 3.)).is_err());
    assert!(entity_by_id(s.simulation().world(), 6).is_some());
    assert!(entity_by_id(s.simulation().world(), last_model).is_none());
    println!(
        "last model={} visual={} next_public={} next_native={} archive_bytes={}",
        last_model,
        last_model + 3,
        next_public,
        next_native + 1,
        fs::metadata(sealed)?.len()
    );
    Ok(())
}
