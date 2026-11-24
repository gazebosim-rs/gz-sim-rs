// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{SceneBackend, pool::PoolFrame};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    passive_checkpoint::PassivePoolScene,
};
#[path = "private/passive_archive.rs"]
mod passive_archive;
use passive_archive::patch;
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const STACK: &str = include_str!("../../../examples/distributed_stack.sdf");
const GROUND: &str = include_str!("../../../examples/distributed_ground.sdf");

fn options(engine: PhysicsEngine) -> WorldLoadOptions<'static> {
    WorldLoadOptions {
        engine,
        step: None,
        distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        noise_seed: [0; 32],
    }
}
fn runtime() -> WorldRuntimeOptions<'static> {
    WorldRuntimeOptions {
        distributed_packed_handoffs: true,
        distributed_local_helper_limit: Some(0),
        ..Default::default()
    }
}
fn load(xml: &str, engine: PhysicsEngine) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::load(
        xml,
        "world.sdf",
        "default",
        options(engine),
        runtime(),
    )?)
}
fn resume(xml: &str, engine: PhysicsEngine, file: &Path) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::resume(
        xml,
        "relocated/world.sdf",
        "default",
        options(engine),
        runtime(),
        file,
    )?)
}
fn frame(scene: &PassivePoolScene) -> &PoolFrame {
    scene
        .simulation()
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .pool_frame()
        .unwrap()
}
fn link(scene: &PassivePoolScene, name: &str) -> u64 {
    scene
        .simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[path = "private/passive_pose_proof.rs"]
mod passive_pose_proof;
fn proof(s: &PassivePoolScene) -> [u8; 32] {
    passive_pose_proof::proof(s.simulation())
}

#[test]
fn continuation_preserves_clock_pause_jointless_bodies_contacts_inputs_and_loaded_reset()
-> Result<()> {
    let dir = tempfile::tempdir()?;
    for (xml, engine, target) in [
        (
            STACK,
            PhysicsEngine::DistributedIslands(4),
            "default::second::body",
        ),
        (
            STACK,
            PhysicsEngine::DistributedAdaptive(3),
            "default::second::body",
        ),
        (
            GROUND,
            PhysicsEngine::DistributedPool(8),
            "default::blue::body",
        ),
    ] {
        for paused in [true, false] {
            let mut source = load(xml, engine)?;
            let mut uninterrupted = load(xml, engine)?;
            let public_link = link(&source, target);
            for s in [&mut source, &mut uninterrupted] {
                // Cross reset once before capture so restored epoch is nonzero.
                s.step_exact(8)?;
                s.reset_loaded()?;
                s.step_exact(110)?;
                s.set_paused(paused);
                s.queue_impulse(public_link, [0.05, 0., 0.])?;
                s.queue_impulse(public_link, [0.125, 0.01, 0.])?;
            }
            assert_eq!(proof(&source), proof(&uninterrupted));
            let file = dir.path().join(format!("{engine:?}-{paused}"));
            source.save(&file)?;
            let cut = gz_physics_distributed::pool::NativeContactSnapshot::open(&file)?;
            assert_eq!(cut.frame, *frame(&source));
            assert_eq!(cut.population, Some(gz_physics_distributed::pool::ScenePopulation::Loaded));
            assert!(cut.patches.iter().any(|p| !p.finite.is_empty() || !p.ground.is_empty()));
            let original = fs::read(&file)?;
            assert_eq!(&original[..8], b"GZSCN001");
            assert!(source.save(&file).is_err());
            assert_eq!(original, fs::read(&file)?);
            drop(source);
            let mut recovered = resume(xml, engine, &file)?;
            assert_eq!(proof(&recovered), proof(&uninterrupted));
            assert_eq!(recovered.simulation().is_paused(), paused);
            if paused {
                for s in [&mut recovered, &mut uninterrupted] {
                    s.tick()?;
                }
                assert_eq!(proof(&recovered), proof(&uninterrupted));
            }
            let mut history = Sha256::new();
            let mut contacts = 0;
            let mut supports = 0;
            for _ in 0..104 {
                recovered.step_exact(1)?;
                uninterrupted.step_exact(1)?;
                assert_eq!(proof(&recovered), proof(&uninterrupted));
                for w in &frame(&recovered).workers {
                    contacts += w.snapshot.impulses.len();
                    supports += w.snapshot.supports.len();
                }
                history.update(proof(&recovered));
            }
            assert!(contacts + supports > 0, "empty loaded contact scenario");
            println!(
                "passive {engine:?} paused={paused} successor_ticks=104 pair_receipts={contacts} ground_receipts={supports} sha256={:x}",
                history.finalize()
            );
            for s in [&mut recovered, &mut uninterrupted] {
                s.reset_loaded()?;
            }
            assert_eq!(recovered.simulation().epoch(), 2);
            assert_eq!(proof(&recovered), proof(&uninterrupted));
            recovered.step_exact(8)?;
            uninterrupted.step_exact(8)?;
            assert_eq!(proof(&recovered), proof(&uninterrupted));
        }
    }
    Ok(())
}

#[test]
fn pending_public_commands_fail_before_publication_and_survive_refusal() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let engine = PhysicsEngine::DistributedIslands(4);
    let mut scene = load(STACK, engine)?;
    let id = link(&scene, "default::second::body");
    let mut reference = load(STACK, engine)?;
    for s in [&mut scene, &mut reference] {
        s.queue_velocity(id, [0.017, 0., 0.], [0.; 3])?;
    }
    let path = dir.path().join("pending");
    assert!(scene.save(&path).is_err());
    assert!(!path.exists());
    scene.tick()?;
    assert!(scene.save(&path).is_err());
    for s in [&mut scene, &mut reference] {
        s.step_exact(1)?;
    }
    assert_eq!(proof(&scene), proof(&reference));
    for s in [&mut scene, &mut reference] {
        s.queue_wrench(id, [2., 0., 0.], [0.; 3])?;
    }
    // Distributed forces are admitted into the bounded scene field immediately.
    // They survive a fresh launch, whereas a deferred public velocity above does not.
    let held = dir.path().join("held");
    scene.save(&held)?;
    drop(scene);
    let mut scene = resume(STACK, engine, &held)?;
    for s in [&mut scene, &mut reference] {
        s.step_exact(1)?;
    }
    assert_eq!(proof(&scene), proof(&reference));
    scene.save(&path)?;
    drop(scene);
    assert_eq!(proof(&resume(STACK, engine, &path)?), proof(&reference));
    Ok(())
}


#[test]
fn refuses_changed_sdf_execution_settings_public_bindings_clocks_and_unsupported_domains()
-> Result<()> {
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("checkpoint");
    let engine = PhysicsEngine::DistributedIslands(3);
    let mut source = load(STACK, engine)?;
    source.step_exact(63)?;
    source.save(&file)?;
    drop(source);
    assert!(resume(&format!("{STACK}\n"), engine, &file).is_err());
    assert!(resume(STACK, PhysicsEngine::DistributedAdaptive(3), &file).is_err());
    assert!(
        PassivePoolScene::resume(
            STACK,
            "world.sdf",
            "default",
            options(engine),
            Default::default(),
            &file
        )
        .is_err()
    );
    for tag in [
        "<include/>",
        "<sensor name='x'/>",
        "<plugin name='x' filename='x'/>",
        "<joint/>",
        "<mesh/>",
    ] {
        assert!(
            load(
                &STACK.replace("</world>", &format!("{tag}</world>")),
                engine
            )
            .is_err()
        );
    }
    assert!(load(STACK, PhysicsEngine::Distributed).is_err());
    let bytes = fs::read(&file)?;
    let bad = dir.path().join("bad");
    for case in 0..5 {
        patch(&bytes, &bad, |meta, _native| match case {
            0 => meta["bindings"]["nodes"][2]["body"] = 999.into(),
            1 => meta["bindings"]["nodes"][2]["parent"] = 7.into(),
            2 => meta["clock"]["iteration"] = 62.into(),
            3 => meta["clock"]["sim_time"]["secs"] = 99.into(),
            4 => {
                meta["public_pose_sha256"][0] =
                    ((meta["public_pose_sha256"][0].as_u64().unwrap() + 1) % 256).into()
            }
            _ => unreachable!(),
        })?;
        assert!(resume(STACK, engine, &bad).is_err(), "case {case}");
    }
    for length in [0u32, 512 * 1024 + 1, u32::MAX] {
        let mut altered = bytes.clone();
        altered[40..44].copy_from_slice(&length.to_le_bytes());
        let h = Sha256::digest(&altered[40..]);
        altered[8..40].copy_from_slice(&h);
        fs::write(&bad, altered)?;
        assert!(resume(STACK, engine, &bad).is_err());
    }
    Ok(())
}

#[cfg(target_os = "linux")]
fn at_crash_barrier() -> Result<PassivePoolScene> {
    let mut s = load(STACK, PhysicsEngine::DistributedIslands(4))?;
    s.step_exact(3)?;
    s.reset_loaded()?;
    s.step_exact(110)?;
    s.set_paused(true);
    let id = link(&s, "default::second::body");
    s.queue_impulse(id, [0.125, 0., 0.])?;
    Ok(s)
}
#[cfg(target_os = "linux")]
fn pids(scene: &PassivePoolScene) -> Vec<u32> {
    scene
        .simulation()
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .local_worker_pids()
        .unwrap()
}
#[cfg(target_os = "linux")]
fn suffix(mut scene: PassivePoolScene) -> Result<Vec<[u8; 32]>> {
    let mut hashes = vec![proof(&scene)];
    for _ in 0..128 {
        scene.step_exact(1)?;
        hashes.push(proof(&scene));
    }
    scene.reset_loaded()?;
    hashes.push(proof(&scene));
    for _ in 0..8 {
        scene.step_exact(1)?;
        hashes.push(proof(&scene));
    }
    Ok(hashes)
}
#[cfg(target_os = "linux")]
#[test]
#[ignore = "separate process launched by the SIGKILL integration"]
fn crash_role() -> Result<()> {
    let dir = std::path::PathBuf::from(std::env::var_os("GZ_PASSIVE_CHECKPOINT_TEST_DIR").unwrap());
    let path = dir.join("scene.barrier");
    match std::env::var("GZ_PASSIVE_CHECKPOINT_TEST_ROLE")?.as_str() {
        "original" => {
            let mut scene = at_crash_barrier()?;
            scene.save(&path)?;
            // The original coordinator continues one unsealed interval beyond the disk image.
            scene.step_exact(1)?;
            // A fresh independent body can accept a deferred public velocity. Neither
            // this unsealed birth nor its pending command belongs to the saved cut.
            let model = scene.spawn(10, gz_sim_scene::WorldPose {
                translation: [8., 0., 3.],
                rotation: [0., 0., 0., 1.],
            })?;
            scene.queue_velocity(model + 1, [-0.25, 0., 0.125], [0.; 3])?;
            let refused = dir.join("not-committed");
            assert!(scene.save(&refused).is_err());
            assert!(!refused.exists());
            fs::write(
                dir.join("ready.part"),
                serde_json::to_vec(&(frame(&scene).tick, pids(&scene)))?,
            )?;
            fs::rename(dir.join("ready.part"), dir.join("ready"))?;
            loop {
                std::thread::park();
            }
        }
        "replacement" => {
            let scene = resume(STACK, PhysicsEngine::DistributedIslands(4), &path)?;
            assert_eq!(frame(&scene).tick, 110);
            assert_eq!(scene.simulation().epoch(), 1);
            assert!(scene.simulation().is_paused());
            let owners = pids(&scene);
            let hashes = suffix(scene)?;
            fs::write(
                dir.join("successors"),
                serde_json::to_vec(&(owners, hashes))?,
            )?;
            Ok(())
        }
        _ => unreachable!(),
    }
}

#[cfg(target_os = "linux")]
#[test]
fn sigkill_coordinator_then_resume_into_new_process_and_restore_loaded_reset() -> Result<()> {
    use std::{
        process::{Child, Command},
        thread,
        time::{Duration, Instant},
    };
    struct Guard(Child);
    impl Drop for Guard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    let dir = tempfile::tempdir()?;
    let test = std::env::current_exe()?;
    let command = |role: &str| {
        let mut c = Command::new(&test);
        c.args(["--exact", "--ignored", "crash_role", "--nocapture"])
            .env("GZ_PASSIVE_CHECKPOINT_TEST_DIR", dir.path())
            .env("GZ_PASSIVE_CHECKPOINT_TEST_ROLE", role);
        c
    };
    let mut original = Guard(command("original").spawn()?);
    let deadline = Instant::now() + Duration::from_secs(30);
    while !dir.path().join("ready").exists() {
        if let Some(s) = original.0.try_wait()? {
            return Err(format!("coordinator died before checkpoint: {s}").into());
        }
        if Instant::now() > deadline {
            return Err("source did not publish a barrier".into());
        }
        thread::sleep(Duration::from_millis(10));
    }
    let (tick, old): (u64, Vec<u32>) =
        serde_json::from_slice(&fs::read(dir.path().join("ready"))?)?;
    assert_eq!(tick, 111);
    assert_eq!(old.len(), 4);
    let original_pid = original.0.id();
    original.0.kill()?;
    use std::os::unix::process::ExitStatusExt;
    assert_eq!(original.0.wait()?.signal(), Some(9));
    for pid in &old {
        // A reparented zombie has exited and cannot serve its endpoint.
        while fs::read_to_string(format!("/proc/{pid}/stat")).is_ok_and(|s| {
            s.rsplit_once(") ")
                .is_some_and(|(_, fields)| !fields.starts_with("Z "))
        }) {
            if Instant::now() > deadline {
                return Err("an original worker still runs".into());
            }
            thread::sleep(Duration::from_millis(10));
        }
    }
    let mut replacement = Guard(command("replacement").spawn()?);
    let fresh_pid = replacement.0.id();
    assert_ne!(fresh_pid, original_pid);
    assert!(replacement.0.wait()?.success());
    let (new, observed): (Vec<u32>, Vec<[u8; 32]>) =
        serde_json::from_slice(&fs::read(dir.path().join("successors"))?)?;
    assert_eq!(new.len(), 4);
    assert!(new.iter().all(|pid| !old.contains(pid)));
    let expected = suffix(at_crash_barrier()?)?;
    assert_eq!(observed, expected);
    let mut hash = Sha256::new();
    for h in observed {
        hash.update(h);
    }
    println!(
        "SIGKILL source={original_pid} old_workers={old:?} replacement={fresh_pid} fresh_workers={new:?} lost_unsealed_tick=111 restored_tick=110 epoch=1 paused=true compared_barriers={} trajectory_sha256={:x}",
        expected.len(),
        hash.finalize()
    );
    Ok(())
}

#[test]
fn loaded_templates_and_live_catalog_can_inspect_resting_sources_without_editing_them() -> Result<()>
{
    let engine = PhysicsEngine::DistributedIslands(4);
    let mut scene = load(STACK, engine)?;
    scene.step_exact(110)?;
    let dir = tempfile::tempdir()?;
    let checkpoint = dir.path().join("stack");
    scene.save(&checkpoint)?;
    drop(scene);
    let restored = resume(STACK, engine, &checkpoint)?;
    let before = proof(&restored);
    let sdf = sdformat::load_str(STACK, "stack.sdf", &sdformat::NoResources)?;
    for model in &sdf.worlds[0].models {
        gz_sim_scene::prepare_model_template(restored.simulation(), &sdf, model)?;
    }
    let loaded = link(&restored, "default::first");
    assert!(
        gz_sim_scene::model_state(restored.simulation(), loaded).is_err(),
        "contact still fences same-tick state mutations"
    );
    assert_eq!(before, proof(&restored));
    Ok(())
}

#[test]
fn clock_zero_preserves_oriented_loaded_geometry_and_pause_after_reset() -> Result<()> {
    let sdf = include_str!("../../../examples/distributed_oriented_plane.sdf");
    let dir = tempfile::tempdir()?;
    let engine = PhysicsEngine::DistributedPool(3);
    for epoch in [0, 1] {
        let mut source = load(sdf, engine)?;
        let mut reference = load(sdf, engine)?;
        for s in [&mut source, &mut reference] {
            if epoch == 1 {
                s.set_paused(false);
                s.step_exact(1)?;
                s.reset_loaded()?;
            }
        }
        let path = dir.path().join(epoch.to_string());
        source.save(&path)?;
        drop(source);
        let mut recovered = resume(sdf, engine, &path)?;
        assert_eq!(proof(&recovered), proof(&reference));
        for _ in 0..128 {
            recovered.step_exact(1)?;
            reference.step_exact(1)?;
            assert_eq!(proof(&recovered), proof(&reference));
        }
    }
    Ok(())
}

#[test]
fn bounded_sdf_domain_and_manifest_envelope_check_every_cut_before_workers() -> Result<()> {
    use gz_physics_distributed::pool::{MAX_SCENE_METADATA_BYTES, PoolNativeRecovery};

    let dir = tempfile::tempdir()?;
    let engine = PhysicsEngine::DistributedIslands(3);
    let mut longest = STACK.to_owned();
    longest.push_str(&" ".repeat(1024 * 1024 - longest.len()));
    let scene = load(&longest, engine)?;
    longest.push(' ');
    assert!(load(&longest, engine).is_err());
    drop(scene);
    for (xml, world) in [
        (
            "<sdf><world name='default'/><world name='other'/></sdf>",
            "default",
        ),
        ("<world name='default'/>", "default"),
        (STACK, "other"),
        (
            "<sdf><world name='default'><n:box xmlns:n='urn:test'/></world></sdf>",
            "default",
        ),
        ("<sdf>", "default"),
    ] {
        assert!(
            PassivePoolScene::load(xml, "stack.sdf", world, options(engine), runtime()).is_err()
        );
    }
    for settings in [
        WorldRuntimeOptions {
            distributed_native_64: true,
            ..Default::default()
        },
        WorldRuntimeOptions {
            gpu_adapter: Some("unavailable"),
            ..Default::default()
        },
    ] {
        assert!(
            PassivePoolScene::load(STACK, "stack.sdf", "default", options(engine), settings)
                .is_err()
        );
    }
    let mut sim = load(STACK, engine)?.into_simulation();
    let p = dir.path().join("bounded-metadata");
    let metadata = vec![b'a'; MAX_SCENE_METADATA_BYTES];
    {
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = backend.downcast_mut::<SceneBackend>().unwrap();
        assert!(native.save_loaded_pool_checkpoint(&p, 0, &[]).is_err());
        assert!(
            native
                .save_loaded_pool_checkpoint(&p, 0, &vec![b'a'; MAX_SCENE_METADATA_BYTES + 1])
                .is_err()
        );
        assert!(
            native
                .save_loaded_pool_checkpoint(&p, 1, &metadata)
                .is_err()
        );
        assert!(!p.exists());
        native.save_loaded_pool_checkpoint(&p, 0, &metadata)?;
    }
    let (opened, native) = PoolNativeRecovery::open_scene(&p)?;
    assert_eq!(opened, metadata);
    assert_eq!(native.frame().tick, 0);
    drop(sim);

    // These are properly hashed, bounded files with incomplete lengths or native tags.
    let bad = dir.path().join("truncated");
    for payload in [
        vec![1],
        vec![9, 0, 0, 0, b'a'],
        vec![1, 0, 0, 0, b'a', b'G', b'Z'],
        vec![
            1, 0, 0, 0, b'a', b'G', b'Z', b'P', b'O', b'O', b'L', b'0', b'1',
        ],
    ] {
        let mut file = b"GZSCN001".to_vec();
        file.extend(Sha256::digest(&payload));
        file.extend(payload);
        fs::write(&bad, file)?;
        assert!(PoolNativeRecovery::open_scene_manifest(&bad).is_err());
    }
    Ok(())
}

#[test]
fn nested_native_definition_must_still_match_the_rebound_sdf_and_manifest_schema() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("good");
    let engine = PhysicsEngine::DistributedIslands(3);
    let mut scene = load(STACK, engine)?;
    scene.step_exact(7)?;
    scene.save(&path)?;
    drop(scene);
    let bytes = fs::read(path)?;
    let bad = dir.path().join("resealed");
    let changed = STACK.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.25</mu><mu2>0.25</mu2>");
    // Both SDFs are admitted and their public hierarchies are identical.
    drop(load(&changed, engine)?);
    patch(&bytes, &bad, |m, _| {
        m["domain"]["sdf_sha256"] = serde_json::json!(Sha256::digest(changed.as_bytes()).to_vec());
    })?;
    assert!(
        resume(&changed, engine, &bad)
            .err()
            .expect("invalid manifest")
            .to_string()
            .contains("native pool differs")
    );
    for (section, field) in [
        ("", "unknown"),
        ("bindings", "unknown"),
        ("clock", "unknown"),
        ("domain", "unknown"),
    ] {
        patch(&bytes, &bad, |m, _| {
            let object = if section.is_empty() {
                m
            } else {
                &mut m[section]
            };
            object[field] = true.into();
        })?;
        assert!(
            resume(STACK, engine, &bad)
                .err()
                .expect("invalid manifest")
                .to_string()
                .contains("unknown field")
        );
    }
    patch(&bytes, &bad, |m, _| {
        m["bindings"]["nodes"][0]["unknown"] = true.into()
    })?;
    assert!(
        resume(STACK, engine, &bad)
            .err()
            .expect("invalid manifest")
            .to_string()
            .contains("unknown field")
    );
    Ok(())
}

#[test]
fn periodic_sdf_checkpoint_retains_policy_and_starts_fresh_timing() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("timed.gzscn");
    let engine = PhysicsEngine::DistributedIslands(4);
    // This test isolates the timing lifecycle from a machine's measured move cost.
    let runtime = WorldRuntimeOptions {
        distributed_periodic_placement: Some("4,8,1,4294967295".parse()?),
        ..runtime()
    };
    let status = |s: &PassivePoolScene| {
        s.simulation()
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap()
            .placement_status()
            .unwrap()
            .clone()
    };
    // Eight separated links make at least two native worlds share a worker.
    let models = (0..8).map(|i| format!(r#"<model name="b{i}"><pose>{} 0 4 0 0 0</pose><link name="body"><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name="c"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>"#,i*5)).collect::<String>();
    let xml = format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity><physics name="ten" type="ignored"><max_step_size>0.01</max_step_size></physics>{models}</world></sdf>"#
    );
    let parallel = std::thread::available_parallelism()?.get() > 1;
    let mut source =
        PassivePoolScene::load(&xml, "scene.sdf", "default", options(engine), runtime)?;
    source.step_exact(2)?;
    assert_eq!(status(&source).windows_started, u64::from(parallel));
    source.save(&file)?;
    let mismatched = WorldRuntimeOptions {
        distributed_periodic_placement: None,
        ..runtime
    };
    assert!(
        PassivePoolScene::resume(
            &xml,
            "scene.sdf",
            "default",
            options(engine),
            mismatched,
            &file
        )
        .is_err()
    );
    let mut restored = PassivePoolScene::resume(
        &xml,
        "scene.sdf",
        "default",
        options(engine),
        runtime,
        &file,
    )?;
    assert_eq!(status(&restored), Default::default());
    for _ in 0..13 {
        source.step_exact(1)?;
        restored.step_exact(1)?;
        assert_eq!(proof(&source), proof(&restored));
    }
    assert_eq!(status(&restored).windows_completed > 0, parallel);
    assert_eq!(status(&restored).committed_moves, 0);
    restored.reset_loaded()?;
    restored.step_exact(1)?;
    assert_eq!(frame(&restored).tick, 1);
    assert_eq!(status(&restored).windows_started, u64::from(parallel));
    assert_eq!(status(&restored).windows_completed, 0);
    Ok(())
}
