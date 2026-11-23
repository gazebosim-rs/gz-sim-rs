// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
use gz_physics::PhysicsBackend;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    JointHandle,
    distributed_hinge::{DistributedFloatingTreeBackend, load_distributed_floating_tree_world},
    queue_joint_effort, remove_model, reset_loaded_world,
};
use serde_json::Value;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

const CLI: &str = env!("CARGO_BIN_EXE_gz-sim-tree");
const ARCHIVE: &str = "barrier.gzfloatingtree";
const XML: &str = include_str!("../../../examples/distributed_branched_tree.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[test]
fn rejects_nonfinite_invalid_commands_before_session_creation() {
    let temporary = tempfile::tempdir().unwrap();
    let session = temporary.path().join("invalid");
    for args in [
        vec!["queue", "1", "NaN"],
        vec!["step", "--effort", "1=inf"],
        vec!["step", "--expect-tick", "0"],
        vec!["step", "--expect-epoch", "0"],
        vec!["step", "--effort", "bad"],
    ] {
        assert!(
            !Command::new(CLI)
                .arg(&session)
                .args(args)
                .output()
                .unwrap()
                .status
                .success()
        );
        assert!(!session.exists());
    }
}

#[test]
#[ignore = "requires a stock or frozen scalar floating tree worker via GZ_MULTI_TEST_WORKER"]
fn command_line_and_staged_vectors_consume_each_saved_joint_once() -> Result<()> {
    let worker = PathBuf::from(
        std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing GZ_MULTI_TEST_WORKER")?,
    );
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let a = root.path().join("operator");
    let b = root.path().join("split");
    let sdf =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/distributed_branched_tree.sdf");
    let raw = |dir: &Path, args: &[String]| -> Result<Output> {
        Ok(Command::new(CLI)
            .arg(dir)
            .arg("--worker")
            .arg(&worker)
            .args(args)
            .output()?)
    };
    let good = |dir: &Path, args: &[String]| -> Result<Value> {
        let out = raw(dir, args)?;
        if !out.status.success() {
            return Err(String::from_utf8_lossy(&out.stderr).into_owned().into());
        }
        Ok(serde_json::from_slice(&out.stdout)?)
    };
    let create = [
        "create".into(),
        sdf.display().to_string(),
        "--workers=3".into(),
    ];
    let initial = good(&a, &create)?;
    assert_eq!(initial, good(&b, &create)?);
    let ids: Vec<u64> = initial["state"]["joints"]
        .as_array()
        .unwrap()
        .iter()
        .map(|j| j["joint_id"].as_u64().unwrap())
        .collect();
    let cut = |dir: &Path| fs::read(dir.join(ARCHIVE));
    let vector = |v0: f64, v1: f64| {
        vec![
            "step".into(),
            "--expect-epoch=0".into(),
            "--expect-tick=0".into(),
            format!("--effort={}={v0}", ids[0]),
            format!("--effort={}={v1}", ids[1]),
        ]
    };
    for dir in [&a, &b] {
        good(dir, &["queue".into(), ids[0].to_string(), "-.03".into()])?;
        good(dir, &["queue".into(), ids[2].to_string(), ".09".into()])?;
    }
    let before = cut(&a)?;
    for refused in [
        vec![
            "step".into(),
            format!("--effort={}=.02", ids[0]),
            format!("--effort={}=.06", ids[0]),
        ],
        vec![
            "step".into(),
            format!("--effort={}=.02", ids[0]),
            "--effort=99999=.01".into(),
        ],
        vec![
            "step".into(),
            format!("--effort={}=.02", ids[0]),
            "--expect-epoch=0".into(),
            "--expect-tick=1".into(),
        ],
    ] {
        assert!(!raw(&a, &refused)?.status.success());
        assert_eq!(cut(&a)?, before);
    }
    let report = good(&a, &vector(0.12, -0.07))?;
    good(&b, &["queue".into(), ids[0].to_string(), ".12".into()])?;
    good(&b, &["queue".into(), ids[1].to_string(), "-.07".into()])?;
    assert_eq!(report, good(&b, &["step".into()])?);
    assert_eq!(cut(&a)?, cut(&b)?);
    assert!(!raw(&a, &vector(0.12, -0.07))?.status.success());
    assert_eq!(cut(&a)?, cut(&b)?);
    for action in [["transfer", "2"], ["recover", "2"]] {
        let args = action.map(str::to_owned);
        assert_eq!(good(&a, &args)?, good(&b, &args)?);
    }
    let open = |d: &Path| {
        load_distributed_floating_tree_world(XML, "branched.sdf", "default", &worker, d, Some(3))
    };
    let mut host = open(&a)?;
    let mut reference = open(&b)?;
    let native_id = |public| {
        host.world()
            .get::<JointHandle>(entity_by_id(host.world(), public).unwrap())
            .unwrap()
            .id()
    };
    let [j0, j1] = [native_id(ids[0]), native_id(ids[1])];
    let model = initial["model_id"].as_u64().unwrap();
    assert!(remove_model(&mut host, model).is_err());
    let stage = |sim: &mut gz_sim_core::Simulation, e, t, data: &[(gz_physics::JointId, f64)]| {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<DistributedFloatingTreeBackend>()
            .unwrap()
            .stage_step_efforts_at(e, t, data)
    };
    for sim in [&mut host, &mut reference] {
        queue_joint_effort(sim, ids[0], 0.2)?;
    }
    let saved = cut(&a)?;
    for data in [vec![(j0, 1.), (j0, 2.)], vec![(j1, 0.1), (j0, f64::NAN)]] {
        assert!(stage(&mut host, 0, 1, &data).is_err());
    }
    assert!(stage(&mut host, 0, 0, &[(j0, -0.5)]).is_err());
    stage(&mut host, 0, 1, &[(j0, f64::MAX)])?;
    assert!(queue_joint_effort(&mut host, ids[0], f64::MAX).is_err());
    assert_eq!(cut(&a)?, saved);
    drop(host);
    host = open(&a)?;
    stage(&mut host, 0, 1, &[(j0, -0.5), (j1, 0.23)])?;
    assert!(stage(&mut host, 0, 1, &[]).is_err());
    host.tick()?;
    assert_eq!(cut(&a)?, saved);
    for (id, torque) in [(ids[0], -0.5), (ids[1], 0.23)] {
        queue_joint_effort(&mut reference, id, torque)?;
    }
    host.step_exact(1)?;
    reference.step_exact(1)?;
    assert_eq!(cut(&a)?, cut(&b)?);
    stage(&mut host, 0, 2, &[(j0, 1.)])?;
    reset_loaded_world(&mut host, |_| ())?;
    reset_loaded_world(&mut reference, |_| ())?;
    assert!(stage(&mut host, 0, 0, &[(j0, 1.)]).is_err());
    host.step_exact(1)?;
    reference.step_exact(1)?;
    assert_eq!(cut(&a)?, cut(&b)?);
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to a finite-stop worker"]
fn stopped_session_resolves_by_archive_and_preserves_controls_at_a_loaded_cut() -> Result<()> {
    use gz_sim_scene::distributed_hinge::{
        DistributedStoppedTreeBackend, load_distributed_articulation_world,
    };
    const STOP: &str = include_str!("../../../examples/distributed_stopped_tree.sdf");
    let worker = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let session = root.path().join("stops");
    let run = |args: &[&str]| -> Result<Value> {
        let result = Command::new(CLI)
            .arg(&session)
            .arg("--worker")
            .arg(&worker)
            .args(args)
            .output()?;
        if !result.status.success() {
            return Err(String::from_utf8_lossy(&result.stderr).into_owned().into());
        }
        Ok(serde_json::from_slice(&result.stdout)?)
    };
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/distributed_stopped_tree.sdf");
    let cold = run(&["create", path.to_str().unwrap(), "--workers", "3"])?;
    let limits = cold["joint_limits"].as_array().unwrap();
    assert_eq!(limits.len(), 3);
    let id = limits[0]["joint_id"].as_u64().unwrap();
    let public = id.to_string();
    let mut sim = load_distributed_articulation_world(
        STOP,
        "stops.sdf",
        "default",
        &worker,
        &session,
        Some(3),
    )?;
    let model = cold["model_id"].as_u64().unwrap();
    assert!(remove_model(&mut sim, model).is_err());
    for _ in 0..220 {
        let n = sim
            .world()
            .get::<JointHandle>(entity_by_id(sim.world(), id).unwrap())
            .unwrap()
            .id();
        let mut p = sim.world_mut().resource_mut::<PhysicsBackend>();
        let t = p.downcast_mut::<DistributedStoppedTreeBackend>().unwrap();
        t.stage_step_effort_at(t.pool().epoch(), t.pool().observation().tick, n, 0.65)?;
        sim.step_exact(1)?;
    }
    drop(sim);
    let before = run(&["inspect"])?;
    let joint = |v: &Value| {
        let j = v["state"]["joints"]
            .as_array()
            .unwrap()
            .iter()
            .find(|j| j["joint_id"].as_u64() == Some(id))
            .unwrap();
        (
            j["q"].as_f64().unwrap(),
            j["qdot"].as_f64().unwrap(),
            j["pending_effort"].as_f64(),
        )
    };
    assert!(joint(&before).0 >= limits[0]["upper"].as_f64().unwrap() - 1e-6);
    let held = run(&["queue", &public, "-0.73"])?;
    assert_eq!(joint(&held).2, Some(-0.73));
    let moved = run(&["transfer", "1"])?;
    let recovered = run(&["recover", "1"])?;
    assert_eq!(joint(&held), joint(&moved));
    assert_eq!(joint(&held), joint(&recovered));
    let accepted = run(&["step", "--expect-epoch=0", "--expect-tick=220"])?;
    assert!(joint(&accepted).0 < joint(&held).0 && joint(&accepted).2.is_none());
    let bytes = fs::read(session.join("barrier.gzstoppedtree"))?;
    assert!(run(&["step", "--expect-epoch=0", "--expect-tick=220"]).is_err());
    assert_eq!(fs::read(session.join("barrier.gzstoppedtree"))?, bytes);
    let reset = run(&["reset"])?;
    assert_eq!(reset["epoch"], 1);
    assert_eq!(joint(&reset), (0.0, 0.0, None));
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER supporting the velocity motor profile"]
fn motor_commands_replace_native_config_without_a_step_and_reset_clears_it() -> Result<()> {
    use gz_sim_scene::distributed_hinge::{
        DistributedMotorTreeBackend, load_distributed_articulation_world,
    };
    let worker =
        PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing motor worker")?);
    let temp = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let session = temp.path().join("motors");
    let run = |args: &[&str]| -> Result<Value> {
        let r = Command::new(CLI)
            .arg(&session)
            .arg("--worker")
            .arg(&worker)
            .args(args)
            .output()?;
        if !r.status.success() {
            return Err(String::from_utf8_lossy(&r.stderr).into_owned().into());
        }
        Ok(serde_json::from_slice(&r.stdout)?)
    };
    let sdf =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/distributed_branched_tree.sdf");
    let ready = run(&[
        "create",
        sdf.to_str().unwrap(),
        "--motorized",
        "--workers",
        "3",
    ])?;
    let index = 0;
    let id = ready["velocity_motors"][index]["joint_id"]
        .as_u64()
        .unwrap();
    let name = id.to_string();
    let armed = run(&["motor", &name, "-1.26", "0.15"])?;
    assert_eq!(armed["state"], ready["state"]);
    assert_eq!(armed["velocity_motors"][index]["target_velocity"], -1.26);
    let held = run(&["queue", &name, "0.28"])?;
    let path = session.join("barrier.gzmotorizedtree");
    let before = fs::read(&path)?;
    for bad in [
        ["motor", &name, "inf", "0.3"],
        ["motor", &name, "1", "-0.1"],
        ["motor", "99999", "1", "0.3"],
    ] {
        assert!(run(&bad).is_err());
        assert_eq!(fs::read(&path)?, before);
    }
    let slowed = run(&["motor", &name, "0.2", "0"])?;
    assert_eq!(slowed["state"], held["state"]);
    let mut sim =
        load_distributed_articulation_world(XML, "motors.sdf", "default", &worker, &session, None)?;
    let joint = sim
        .world()
        .get::<JointHandle>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .id();
    let mut be = sim.world_mut().resource_mut::<PhysicsBackend>();
    let original = fs::read(&path)?;
    assert!(
        be.set_joint_velocity_motor(
            joint,
            Some(gz_physics::JointVelocityMotor {
                target_velocity: f64::NAN,
                max_force: 1.
            })
        )
        .is_err()
    );
    assert_eq!(fs::read(&path)?, original);
    be.set_joint_velocity_motor(
        joint,
        Some(gz_physics::JointVelocityMotor {
            target_velocity: -0.41,
            max_force: 0.27,
        }),
    )?;
    sim.step_exact(2)?;
    drop(sim);
    let moved = run(&["transfer", "2"])?;
    let recovered = run(&["recover", "2"])?;
    assert_eq!(moved["state"], recovered["state"]);
    assert_eq!(moved["velocity_motors"], recovered["velocity_motors"]);
    assert_eq!(
        recovered["velocity_motors"][index]["target_velocity"],
        -0.41
    );
    let off = run(&["motor-off", &name])?;
    assert_eq!(off["state"], recovered["state"]);
    assert!(off["velocity_motors"][index]["target_velocity"].is_null());
    run(&["motor", &name, "1.2", "0.4"])?;
    let mut host =
        load_distributed_articulation_world(XML, "motors.sdf", "default", &worker, &session, None)?;
    assert!(remove_model(&mut host, ready["model_id"].as_u64().unwrap()).is_err());
    reset_loaded_world(&mut host, |_| ())?;
    assert!(
        host.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedMotorTreeBackend>()
            .unwrap()
            .pool()
            .velocity_motor(id)
            .is_none()
    );
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to a motorized grasp worker"]
fn payload_force_cli_and_bevy_consume_one_guarded_interval() -> Result<()> {
    use gz_sim_scene::{
        distributed_hinge::{DistributedGraspTreeBackend, load_distributed_articulation_world},
        queue_link_world_wrench,
    };
    const GRIPPER: &str = include_str!("../../../examples/distributed_gripper_payload.sdf");
    let worker = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let a = root.path().join("cli");
    let b = root.path().join("bevy");
    let sdf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/distributed_gripper_payload.sdf");
    let call = |dir: &Path, args: &[&str]| -> Result<Output> {
        Ok(Command::new(CLI)
            .arg(dir)
            .arg("--worker")
            .arg(&worker)
            .args(args)
            .output()?)
    };
    let good = |dir: &Path, args: &[&str]| -> Result<Value> {
        let result = call(dir, args)?;
        if !result.status.success() {
            return Err(String::from_utf8_lossy(&result.stderr).into_owned().into());
        }
        Ok(serde_json::from_slice(&result.stdout)?)
    };
    let cut = |d: &Path| fs::read(d.join("barrier.gzgrasptree"));
    let mut seed = Value::Null;
    for dir in [&a, &b] {
        seed = good(dir, &["create", sdf.to_str().unwrap(), "--motorized"])?;
        good(dir, &["motor", "15", "6", "4"])?;
        good(dir, &["queue", "15", "-0.10"])?;
    }
    let prior = cut(&a)?;
    let step = [
        "step",
        "--expect-epoch=0",
        "--expect-tick=0",
        "--effort=15=0.35",
        "--payload-force=0,0,1.2",
    ];
    for extra in [
        "--payload-force=NaN,0,0",
        "--payload-force=1,0",
        "--payload-force=1,2,3,4",
        "--expect-tick=3",
    ] {
        let args = if extra.starts_with("--expect") {
            vec!["step", extra, "--expect-epoch=0", "--payload-force=0,0,1"]
        } else {
            vec!["step", extra]
        };
        assert!(!call(&a, &args)?.status.success());
        assert_eq!(prior, cut(&a)?);
    }
    let first = good(&a, &step)?;
    assert!(!call(&a, &step)?.status.success());
    assert_eq!(first, good(&a, &["inspect"])?);
    let mut host =
        load_distributed_articulation_world(GRIPPER, "gripper.sdf", "default", &worker, &b, None)?;
    let part = seed["payload"]["state"]["id"].as_u64().unwrap();
    assert!(queue_link_world_wrench(&mut host, part, [0.; 3], [0., 1., 0.]).is_err());
    queue_link_world_wrench(&mut host, part, [0., 0., 0.5], [0.; 3])?;
    queue_link_world_wrench(&mut host, part, [0., 0., 0.7], [0.; 3])?;
    let j = host
        .world()
        .get::<JointHandle>(entity_by_id(host.world(), 15).unwrap())
        .unwrap()
        .id();
    host.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<DistributedGraspTreeBackend>()
        .unwrap()
        .stage_step_effort_at(0, 0, j, 0.35)?;
    host.tick()?;
    assert_eq!(cut(&b)?, prior);
    host.step_exact(1)?;
    drop(host);
    assert_eq!(cut(&a)?, cut(&b)?);
    let bevy = good(&b, &["inspect"])?;
    let vy = |row: &Value| row["payload"]["state"]["velocity"][2].as_f64().unwrap();
    assert!((vy(&first) - vy(&bevy)).abs() < 1e-17);
    assert!((vy(&first) - 0.001 * 1.2 / 0.72).abs() < 1e-15);
    let coast = good(&a, &["step"])?;
    assert_eq!(vy(&coast).to_bits(), vy(&first).to_bits());
    good(&a, &["reset"])?;
    assert!(!call(&a, &step)?.status.success());
    Ok(())
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER with selective-grasp placement"]
fn separated_grasp_cli_recovers_both_owners_and_rejects_stale_steps() -> Result<()> {
    use gz_sim_scene::distributed_hinge::{
        DistributedSplitGraspBackend, GraspPlacement, load_distributed_articulation_world,
        load_distributed_split_grasp_world,
    };
    let xml = include_str!("../../../examples/distributed_gripper_payload.sdf")
        .replace("0.317 0.004 0.58", "0.317 0.004 2.58");
    let worker = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let sdf = root.path().join("separated.sdf");
    fs::write(&sdf, &xml)?;
    let session = root.path().join("session");
    let call = |args: &[&str]| -> Result<Value> {
        let out = Command::new(CLI)
            .arg(&session)
            .arg("--worker")
            .arg(&worker)
            .args(args)
            .output()?;
        if !out.status.success() {
            return Err(String::from_utf8_lossy(&out.stderr).to_string().into());
        }
        Ok(serde_json::from_slice(&out.stdout)?)
    };
    let create = |placement| {
        call(&[
            "create",
            sdf.to_str().unwrap(),
            "--workers=3",
            "--motorized",
            "--placement",
            placement,
        ])
    };
    for invalid in ["adaptive=0", "adaptive=65536", "typo"] {
        assert!(create(invalid).is_err());
        assert!(!session.exists());
    }
    let cold = create("adaptive=1")?;
    assert_eq!(cold["owners"], serde_json::json!([0, 0]));
    let joint = cold["state"]["joints"][0]["joint_id"].as_u64().unwrap();
    let joint_name = joint.to_string();
    call(&["motor", &joint_name, "0.3", "0.15"])?;
    let mut warm = call(&["step"])?;
    for _ in 0..4 {
        if warm["owners"][0] != warm["owners"][1] {
            break;
        }
        warm = call(&["step"])?;
    }
    assert_ne!(warm["owners"][0], warm["owners"][1]);
    let spare = (0..3)
        .find(|slot| {
            !warm["owners"]
                .as_array()
                .unwrap()
                .contains(&serde_json::json!(slot))
        })
        .unwrap()
        .to_string();
    let moved = call(&["transfer-payload", &spare])?;
    let recovered = call(&["recover", &spare])?;
    assert_eq!(moved, recovered);
    assert_eq!(warm["state"], recovered["state"]);
    assert_eq!(warm["payload"], recovered["payload"]);
    assert_eq!(warm["velocity_motors"], recovered["velocity_motors"]);
    call(&["queue", &joint_name, "-0.125"])?;
    let path = session.join("barrier.gzsplitgrasp");
    let saved = fs::read(&path)?;
    assert!(
        call(&[
            "step",
            "--expect-epoch=0",
            "--expect-tick=0",
            "--payload-force=0,0,1.2"
        ])
        .is_err()
    );
    assert!(
        load_distributed_split_grasp_world(
            &xml,
            "separated.sdf",
            "default",
            &worker,
            &session,
            None,
            GraspPlacement::KeepTogether
        )
        .is_err()
    );
    assert_eq!(fs::read(&path)?, saved);
    let mut host = load_distributed_articulation_world(
        &xml,
        "separated.sdf",
        "default",
        &worker,
        &session,
        None,
    )?;
    let model = cold["model_id"].as_u64().unwrap();
    assert!(remove_model(&mut host, model).is_err());
    assert_eq!(
        host.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<DistributedSplitGraspBackend>()
            .unwrap()
            .pool()
            .pending_joint_effort(joint),
        Some(-0.125)
    );
    host.step_exact(1)?;
    drop(host);
    let clear = call(&["inspect"])?;
    assert!(clear["state"]["joints"][0]["pending_effort"].is_null());
    let reset = call(&["reset"])?;
    assert_eq!(reset["epoch"], 1);
    assert_eq!(reset["state"], cold["state"]);
    assert_eq!(reset["payload"], cold["payload"]);
    assert_eq!(reset["owners"][0], reset["owners"][1]);
    Ok(())
}

#[test]
#[ignore = "requires the managed gripper fleet worker via GZ_MULTI_TEST_WORKER"]
fn fleet_cli_routes_atomic_inputs_and_preserves_the_clock_through_owner_changes() -> Result<()> {
    let worker = PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let root = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let sdf =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/distributed_grasp_fleet.sdf");
    for (layout, archive) in [
        ("packed", "barrier.gzgraspfleet"),
        ("round-robin", "barrier.gzgraspfleet"),
        ("coupled", "barrier.gzgraspcluster"),
        ("adaptive", "barrier.gzadaptivegraspfleet"),
        ("warm", "barrier.gzadaptivegraspfleet"),
        ("replay=1500,2400", "barrier.gzadaptivegraspfleet"),
    ] {
        let session = root.path().join(layout);
        let call = |args: &[&str]| -> Result<Value> {
            let out = Command::new(CLI)
                .arg(&session)
                .arg("--worker")
                .arg(&worker)
                .args(args)
                .output()?;
            if !out.status.success() {
                return Err(String::from_utf8_lossy(&out.stderr).to_string().into());
            }
            Ok(serde_json::from_slice(&out.stdout)?)
        };
        let born = call(&[
            "create",
            sdf.to_str().unwrap(),
            "--workers=3",
            "--fleet",
            layout,
        ])?;
        let groups = born["assemblies"].as_array().unwrap();
        let model = groups[0]["model_id"].as_u64().unwrap().to_string();
        let joint = groups[0]["state"]["joints"][0]["joint_id"]
            .as_u64()
            .unwrap()
            .to_string();
        let part = groups[1]["payload"]["state"]["id"]
            .as_u64()
            .unwrap()
            .to_string();
        let force = format!("{part}=0,.3,0");
        call(&["queue", &joint, ".017"])?;
        let bytes = fs::read(session.join(archive))?;
        for invalid in [
            vec!["step", "--load", &force, "--load", &force],
            vec![
                "step",
                "--load",
                &force,
                "--expect-epoch=0",
                "--expect-tick=1",
            ],
            vec!["step", "--load", "99999=1,2,3"],
        ] {
            assert!(call(&invalid).is_err());
            assert_eq!(fs::read(session.join(archive))?, bytes);
        }
        let accepted = call(&[
            "step",
            "--load",
            &force,
            "--expect-epoch=0",
            "--expect-tick=0",
        ])?;
        assert_eq!(accepted["tick"], 1);
        assert_eq!(
            accepted["assemblies"][0]["state"]["joints"][0]["pending_effort"],
            Value::Null
        );
        assert_ne!(
            accepted["assemblies"][1]["payload"]["state"]["velocity"][1],
            0.
        );
        let migrated = if layout == "coupled" {
            assert!(call(&["transfer", "2", "--model", &model]).is_err());
            call(&["transfer", "2"])?
        } else {
            assert!(call(&["transfer", "2"]).is_err());
            call(&["transfer", "2", "--model", &model])?
        };
        assert_eq!(migrated["tick"], 1);
        assert_eq!(migrated["owners"][0], 2);
        assert_eq!(
            migrated["assemblies"][1]["payload"],
            accepted["assemblies"][1]["payload"]
        );
        assert_eq!(call(&["inspect"])?, migrated);
        let reset = call(&["reset"])?;
        assert_eq!(
            (reset["epoch"].as_u64(), reset["tick"].as_u64()),
            (Some(1), Some(0))
        );
        assert!(call(&["step", "--expect-epoch=0", "--expect-tick=0"]).is_err());
        assert_eq!(call(&["inspect"])?, reset);
    }
    Ok(())
}
