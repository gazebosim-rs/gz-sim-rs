// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "private/passive_archive.rs"]
mod passive_archive;
#[path = "private/passive_pose_proof.rs"]
mod passive_pose_proof;
#[path = "../examples/held_wrench_scene/scenario.rs"]
mod scenario;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{SceneBackend, physics::BodyWrench};
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle,
    passive_checkpoint::{PassivePoolScene, save_paused_runtime},
    queue_link_world_velocity,
};
use scenario::{Result, SDF};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn resume(width: u8, path: &Path) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::resume(
        SDF,
        "stack.sdf",
        "default",
        scenario::options(worker(), width),
        Default::default(),
        path,
    )?)
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn id(sim: &Simulation, link: u64) -> u64 {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), link).unwrap())
        .unwrap()
        .id()
        .raw()
}
fn body(sim: &Simulation, link: u64) -> (u8, &gz_physics_distributed::physics::Body) {
    backend(sim)
        .pool_frame()
        .unwrap()
        .body(id(sim, link))
        .unwrap()
}
fn metadata(path: &Path) -> Result<Value> {
    let bytes = fs::read(path)?;
    let len = u32::from_le_bytes(bytes[40..44].try_into()?) as usize;
    Ok(serde_json::from_slice(&bytes[44..44 + len])?)
}
#[test]
fn fresh_resume_consumes_ordered_wrenches_and_acked_impulse_once_even_after_worker_loss()
-> Result<()> {
    let dir = tempfile::tempdir()?;
    let old_worker = std::env::var_os("GZ_HELD_REFERENCE_WORKER").map(std::path::PathBuf::from);
    for width in 3..=8 {
        let mut source = scenario::load(worker(), width)?;
        let mut control = scenario::load(old_worker.as_deref().unwrap_or(worker()), width)?;
        for s in [&mut source, &mut control] {
            s.step_exact(110)?;
            scenario::queue(s)?;
        }
        let path = dir.path().join(format!("held-{width}"));
        source.save(&path)?;
        let input = backend(source.simulation()).held_scene_wrenches().to_vec();
        assert_eq!(input.len(), 3);
        assert_eq!(
            serde_json::from_value::<Vec<BodyWrench>>(metadata(&path)?["held_wrenches"].clone())?,
            input
        );
        drop(source);
        let mut sim = resume(width, &path)?.into_checkpointable_runtime();
        assert_eq!(backend(&sim).held_scene_wrenches(), input);
        assert_eq!(
            passive_pose_proof::proof(&sim),
            passive_pose_proof::proof(control.simulation())
        );
        for _ in 0..3 {
            sim.tick()?;
            control.tick()?;
        }
        assert_eq!(backend(&sim).held_scene_wrenches(), input);
        assert_eq!(
            passive_pose_proof::proof(&sim),
            passive_pose_proof::proof(control.simulation())
        );
        let (owner, cyan) = body(&sim, 11);
        let before_v = cyan.linear[0];
        let before_w = cyan.angular[1];
        let old_pid = backend(&sim).local_worker_pids().unwrap()[usize::from(owner)];
        if width.is_multiple_of(2) {
            // The request executes, then its reply is withheld on this real local endpoint.
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<SceneBackend>()
                .unwrap()
                .drop_next_acknowledgement(owner)?;
        } else {
            #[cfg(target_os = "linux")]
            {
                use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
                pidfd_send_signal(
                    pidfd_open(Pid::from_raw(old_pid as i32).unwrap(), PidfdFlags::empty())?,
                    Signal::KILL,
                )?;
            }
        }
        let mut digest = Sha256::new();
        for tick in 1..=48 {
            sim.step_exact(1)?;
            control.step_exact(1)?;
            let state = passive_pose_proof::proof(&sim);
            assert_eq!(
                state,
                passive_pose_proof::proof(control.simulation()),
                "width {width} tick {tick}"
            );
            digest.update(state);
            assert!(backend(&sim).held_scene_wrenches().is_empty());
            let cyan = body(&sim, 11).1;
            if tick <= 2 {
                // This airborne 2 kg cube receives 0.25 N s plus 4 N s in X.
                // Its inertia about Y is 0.12 kg m² and its torque integral is 0.24 N m s.
                let dp = cyan.mass * (cyan.linear[0] - before_v);
                let dl = 0.12 * (cyan.angular[1] - before_w);
                assert!((dp - 4.25).abs() < 1e-10, "tick={tick} delta_p={dp}");
                assert!((dl - 0.24).abs() < 1e-10, "tick={tick} delta_L={dl}");
            }
        }
        let new_pid = backend(&sim).local_worker_pids().unwrap()[usize::from(owner)];
        #[cfg(target_os = "linux")]
        if !width.is_multiple_of(2) {
            assert_ne!(old_pid, new_pid);
        }
        let consumed = dir.path().join(format!("consumed-{width}"));
        save_paused_runtime(&mut sim, &consumed)?;
        assert!(metadata(&consumed)?.get("held_wrenches").is_none());
        println!(
            "width={width} original_worker={old_pid} continuing_worker={new_pid} delta_p_x=4.25 delta_L_y=0.24 exact_ticks=48 digest={:x}",
            digest.finalize()
        );
    }
    Ok(())
}
#[test]
fn held_sequence_bounds_order_retirement_reset_and_queued_public_input() -> Result<()> {
    let d = tempfile::tempdir()?;
    let mut scene = scenario::load(worker(), 4)?;
    let mut control = scenario::load(worker(), 4)?;
    scene.step_exact(5)?;
    control.step_exact(5)?;
    for n in 0..32 {
        let f = match n % 3 {
            0 => 1.,
            1 => 2.0f64.powi(-53),
            _ => -1.,
        };
        scene.queue_wrench(11, [f, -0.0, 0.0], [0.; 3])?;
        control.queue_wrench(11, [f, -0.0, 0.0], [0.; 3])?;
    }
    let held = backend(scene.simulation()).held_scene_wrenches().to_vec();
    assert!(scene.queue_wrench(11, [0.; 3], [0.; 3]).is_err());
    let cut = d.path().join("full");
    scene.save(&cut)?;
    drop(scene);
    let mut restored = resume(4, &cut)?;
    assert_eq!(backend(restored.simulation()).held_scene_wrenches(), held);
    assert!(restored.queue_wrench(11, [0.; 3], [0.; 3]).is_err());
    restored.step_exact(1)?;
    control.step_exact(1)?;
    assert_eq!(
        passive_pose_proof::proof(restored.simulation()),
        passive_pose_proof::proof(control.simulation())
    );
    drop(restored);
    let mut source = resume(4, &cut)?.into_checkpointable_runtime();
    queue_link_world_velocity(&mut source, 11, [0.1, 0., 0.], [0.; 3])?;
    let refused = d.path().join("uncommitted");
    assert!(save_paused_runtime(&mut source, &refused).is_err());
    assert!(!refused.exists());
    assert_eq!(backend(&source).held_scene_wrenches(), held);
    drop(source);
    let mut r = resume(4, &cut)?;
    r.retire(10)?;
    assert!(backend(r.simulation()).held_scene_wrenches().is_empty());
    r.queue_wrench(7, [0.1, 0., 0.], [0.; 3])?;
    r.reset_loaded()?;
    assert!(backend(r.simulation()).held_scene_wrenches().is_empty());
    Ok(())
}
#[test]
fn malformed_resealed_held_fields_are_rejected_before_worker_start() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let p = dir.path().join("good");
    let mut s = scenario::load(worker(), 4)?;
    s.step_exact(110)?;
    scenario::queue(&mut s)?;
    s.save(&p)?;
    let manifest = metadata(&p)?;
    let ary = manifest["held_wrenches"].as_array().unwrap();
    let mut unknown = ary.clone();
    unknown[0]["id"] = json!(18446744073709551615u64);
    let mut fixed = ary.clone();
    fixed[0]["id"] = json!(id(s.simulation(), 3));
    let mut extra = ary.clone();
    extra[0]["duration"] = json!(2);
    let mut overflow = ary[0].clone();
    overflow["force"] = json!([f64::MAX, 0., 0.]);
    for (i, value) in [
        json!(unknown),
        json!(fixed),
        json!(extra),
        json!(vec![ary[0].clone(); 33]),
        json!([overflow.clone(), overflow]),
        json!("invalid"),
    ]
    .into_iter()
    .enumerate()
    {
        let broken = dir.path().join(format!("bad-{i}"));
        passive_archive::patch(&fs::read(&p)?, &broken, |m, _| m["held_wrenches"] = value)?;
        let missing = dir.path().join("worker-must-not-be-started");
        let err = PassivePoolScene::resume(
            SDF,
            "stack.sdf",
            "default",
            scenario::options(&missing, 4),
            Default::default(),
            &broken,
        )
        .err()
        .expect("invalid scene field was admitted")
        .to_string();
        assert!(
            !err.contains("No such file"),
            "worker launched before admission: {err}"
        );
    }
    Ok(())
}

#[test]
fn old_no_wrench_archives_reseal_byte_for_byte() -> Result<()> {
    // Complete historical process archives are optional local qualification artifacts.
    // The ordinary gate always checks the same property on a fresh force-free cut.
    let d = tempfile::tempdir()?;
    let mut s = scenario::load(worker(), 4)?;
    s.step_exact(24)?;
    let fresh = d.path().join("fresh");
    s.save(&fresh)?;
    drop(s);
    let mut cuts = vec![fresh];
    if let Some(paths) = std::env::var_os("GZ_HELD_EXACT_ARCHIVES") {
        cuts.extend(std::env::split_paths(&paths));
    }
    for (n, cut) in cuts.iter().enumerate() {
        let original = fs::read(cut)?;
        assert!(metadata(cut)?.get("held_wrenches").is_none());
        let mut restored = resume(4, cut)?;
        assert!(
            backend(restored.simulation())
                .held_scene_wrenches()
                .is_empty()
        );
        let copy = d.path().join(format!("copy-{n}"));
        restored.save(&copy)?;
        let copied = fs::read(copy)?;
        let offset = original.iter().zip(&copied).position(|(x, y)| x != y);
        assert!(
            original == copied,
            "{} first changed byte {offset:?}",
            cut.display()
        );
        println!(
            "retained {} original bytes from {} {:x}",
            original.len(),
            cut.display(),
            Sha256::digest(original)
        );
    }
    Ok(())
}

#[cfg(target_os = "linux")]
#[test]
fn failed_capture_is_retryable_without_consuming_or_reordering_the_field() -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let d = tempfile::tempdir()?;
    let mut s = scenario::load(worker(), 4)?;
    let mut control = scenario::load(worker(), 4)?;
    for scene in [&mut s, &mut control] {
        scene.step_exact(110)?;
        scenario::queue(scene)?;
    }
    let before = backend(s.simulation()).held_scene_wrenches().to_vec();
    let owner = body(s.simulation(), 11).0;
    let backend = backend(s.simulation());
    let pid = backend.local_worker_pids().unwrap()[usize::from(owner)];
    pidfd_send_signal(
        pidfd_open(Pid::from_raw(pid as i32).unwrap(), PidfdFlags::empty())?,
        Signal::KILL,
    )?;
    let cut = d.path().join("retry-capture");
    let initial = s.save(&cut);
    if initial.is_err() {
        assert!(!cut.exists());
        assert_eq!(
            s.simulation()
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<SceneBackend>()
                .unwrap()
                .held_scene_wrenches(),
            before
        );
        s.save(&cut)?;
    }
    drop(s);
    let mut recovered = resume(4, &cut)?;
    for _ in 0..32 {
        recovered.step_exact(1)?;
        control.step_exact(1)?;
        assert_eq!(
            passive_pose_proof::proof(recovered.simulation()),
            passive_pose_proof::proof(control.simulation())
        );
    }
    println!(
        "capture after worker {pid} died, first_attempt_success={}, resumed 32 identical frames",
        initial.is_ok()
    );
    Ok(())
}

#[test]
fn jointly_projected_wrench_and_impulse_cross_owner_on_first_restored_interval() -> Result<()> {
    let d = tempfile::tempdir()?;
    // Two 0.6 m diameter spheres at rest have 0.3 m clearance, with zero gravity.
    // A 20 N s impulse or a 2000 N held force alone is insufficient to reach
    // the other sphere within this 10 ms interval.
    let sdf = SDF
        .replace("0 0 -9.81", "0 0 0")
        .replace("0 0 0.15 0 0 0", "-10 0 0.15 0 0 0")
        .replace("0 0 1.8 0 0 0", "0 0 0 0 0 0")
        .replace("0 0 9.2 0 0 0", "0.9 0 0 0 0 0")
        .replace(
            "<box><size>0.6 0.6 0.6</size></box>",
            "<sphere><radius>0.3</radius></sphere>",
        )
        .replace(">0.06<", ">0.036<")
        .replace(">0.12<", ">0.072<");
    let fresh = || {
        PassivePoolScene::load(
            &sdf,
            "crossing.sdf",
            "default",
            scenario::options(worker(), 4),
            Default::default(),
        )
    };
    let mut source = fresh()?;
    let mut control = fresh()?;
    for s in [&mut source, &mut control] {
        s.step_exact(1)?;
        assert_ne!(body(s.simulation(), 7).0, body(s.simulation(), 11).0);
        s.queue_wrench(7, [1500., 0., 0.], [0.; 3])?;
        s.queue_impulse(7, [20., 0., 0.])?;
        s.queue_wrench(7, [500., 0., 0.], [0.; 3])?;
    }
    let cut = d.path().join("crossing");
    source.save(&cut)?;
    drop(source);
    // The input is individually valid, but the combined movement also needs a
    // complete handoff journal. A resealed under-budget barrier must stay unpublished.
    let under_budget = d.path().join("under-budget");
    passive_archive::patch(&fs::read(&cut)?, &under_budget, |_, native| {
        native["journal_limit"] = json!(2);
    })?;
    let error = PassivePoolScene::resume(
        &sdf,
        "crossing.sdf",
        "default",
        scenario::options(worker(), 4),
        Default::default(),
        &under_budget,
    )
    .err()
    .expect("under-budget exchange became an authoritative scene");
    assert!(
        matches!(
            error,
            gz_sim_scene::passive_checkpoint::Error::Recovery(
                gz_physics_distributed::coordinator::RecoveryError::Coordinator(
                    gz_physics_distributed::coordinator::Error::Capacity
                )
            )
        ),
        "{error}"
    );
    let mut recovered = PassivePoolScene::resume(
        &sdf,
        "crossing.sdf",
        "default",
        scenario::options(worker(), 4),
        Default::default(),
        &cut,
    )?
    .into_checkpointable_runtime();
    let starts = [body(&recovered, 7).0, body(&recovered, 11).0];
    // The first handoff response/step for this resident may go missing.
    recovered
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .drop_next_acknowledgement(starts[0])?;
    let mut contacts = 0;
    for n in 0..12 {
        recovered.step_exact(1)?;
        control.step_exact(1)?;
        assert_eq!(
            passive_pose_proof::proof(&recovered),
            passive_pose_proof::proof(control.simulation())
        );
        let (owner_a, a) = body(&recovered, 7);
        let (owner_b, b) = body(&recovered, 11);
        if n == 0 {
            assert_eq!(owner_a, owner_b);
            assert!((a.mass * a.linear[0] + b.mass * b.linear[0] - 40.).abs() < 1e-9);
        }
        contacts += backend(&recovered)
            .pool_frame()
            .unwrap()
            .workers
            .iter()
            .map(|w| w.snapshot.impulses.len())
            .sum::<usize>();
    }
    assert!(contacts > 0);
    println!(
        "first recovered interval migrated initial owners {starts:?}, total delta_p_x=40, {contacts} committed pair receipts, 12 native/public frames exact"
    );
    Ok(())
}

#[cfg(target_os = "linux")]
#[test]
#[ignore = "requires archived pre-extension and new production CLI binaries plus retained historical archives"]
fn previous_and_current_cli_reseal_every_historical_force_free_cut_identically() -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    use std::{
        path::PathBuf,
        process::{Child, Command, Stdio},
        thread,
        time::{Duration, Instant},
    };
    struct Owned(Child);
    impl Drop for Owned {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    fn wait(child: &mut Child, file: &Path, msg: &str) -> Result<()> {
        let start = Instant::now();
        loop {
            let log = fs::read_to_string(file)?;
            if log.contains(msg) {
                return Ok(());
            }
            if start.elapsed() > Duration::from_secs(15) || child.try_wait()?.is_some() {
                return Err(format!("missing {msg}: {log}").into());
            }
            thread::sleep(Duration::from_millis(2));
        }
    }
    let d = tempfile::tempdir()?;
    let sdf = d.path().join("scene.sdf");
    fs::write(&sdf, SDF)?;
    let config = d.path().join("network.json5");
    fs::write(
        &config,
        r#"{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#,
    )?;
    let cuts =
        std::env::var_os("GZ_HELD_LEGACY_ARCHIVES").ok_or("provide original archive paths")?;
    for (i, input) in std::env::split_paths(&cuts).enumerate() {
        let mut receipts = Vec::new();
        for arm in ["GZ_HELD_BEFORE_CLI", "GZ_HELD_CURRENT_CLI"] {
            let bin = PathBuf::from(std::env::var_os(arm).ok_or("provide frozen CLI paths")?);
            let out = d.path().join(format!("{i}-{arm}"));
            let log = out.with_extension("log");
            let mut cli = Owned(
                Command::new(bin)
                    .arg(&sdf)
                    .args([
                        "--physics-engine",
                        "distributed",
                        "--physics-workers",
                        "4",
                        "--adaptive-contacts",
                        "--physics-worker",
                    ])
                    .arg(worker())
                    .args(["--server", "--resume-pool-checkpoint"])
                    .arg(&input)
                    .arg("--live-pool-checkpoint")
                    .arg(&out)
                    .env("GZ_PARTITION", format!("held-archive-{arm}-{i}"))
                    .env("ZENOH_CONFIG", &config)
                    .stdout(Stdio::null())
                    .stderr(fs::File::create(&log)?)
                    .spawn()?,
            );
            wait(&mut cli.0, &log, "checkpoint armed")?;
            pidfd_send_signal(
                pidfd_open(Pid::from_child(&cli.0), PidfdFlags::empty())?,
                Signal::USR1,
            )?;
            wait(&mut cli.0, &log, "checkpoint saved")?;
            receipts.push(fs::read(out)?);
            drop(cli);
        }
        assert!(
            receipts[0] == receipts[1],
            "no-wrench CLI archive changed: {}",
            input.display()
        );
        println!(
            "before/after CLI output {} bytes {:x} from {}",
            receipts[0].len(),
            Sha256::digest(&receipts[0]),
            input.display()
        );
    }
    Ok(())
}

#[test]
fn fresh_input_after_save_is_only_applied_to_the_timelines_that_receive_it() -> Result<()> {
    let d = tempfile::tempdir()?;
    let cut = d.path().join("before-late-force");
    let mut source = scenario::load(worker(), 4)?;
    source.step_exact(110)?;
    scenario::queue(&mut source)?;
    source.save(&cut)?;
    let sealed = fs::read(&cut)?;
    source.queue_wrench(11, [200., 0., 0.], [0.; 3])?;
    source.step_exact(1)?;
    let wanted = passive_pose_proof::proof(source.simulation());
    let fast = body(source.simulation(), 11).1.linear[0];
    drop(source);
    let mut original_cut = resume(4, &cut)?;
    original_cut.step_exact(1)?;
    let slow = body(original_cut.simulation(), 11).1.linear[0];
    assert!((fast - slow - 1.).abs() < 1e-12);
    drop(original_cut);
    let mut replayed = resume(4, &cut)?;
    replayed.queue_wrench(11, [200., 0., 0.], [0.; 3])?;
    replayed.step_exact(1)?;
    assert_eq!(passive_pose_proof::proof(replayed.simulation()), wanted);
    assert_eq!(fs::read(&cut)?, sealed);
    Ok(())
}
