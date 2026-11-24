// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Whole-tree capability transfer and durable, per-joint input over private worker links.
use gz_physics_distributed::{
    articulation::{ArticulationError, ArticulationLinkSpec, ArticulationPool},
    floating_tree::{FloatingTreeJointSpec, FloatingTreePool, FloatingTreeSpec},
    moving::MovingPool,
    multi_sphere::MultiSpherePool,
};
use std::{error::Error, fs, path::Path, process::Command};
const ARCHIVE: &str = "barrier.gzfloatingtree";
const EXE: &str = env!("CARGO_BIN_EXE_physics_worker");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn serial(n: usize) -> FloatingTreeSpec {
    let mut links = Vec::new();
    let mut joints = Vec::new();
    for i in 0..n {
        let x = 0.18 * i as f64;
        let id = 200 + i as u64;
        links.push(ArticulationLinkSpec {
            id,
            radius: 0.06,
            mass: 1.2,
            position: [x, 0.02, 0.7],
            orientation: [0., 0., 0., 1.],
            velocity: [0.11, -0.02, 0.05],
            angular_velocity: [0.; 3],
            inertia_center: [0.007, 0.001, -0.001],
            inertia_orientation: [0., 0., 0., 1.],
            inertia_tensor: [0.011, 0.013, 0.014, 0., 0., 0.],
        });
        if i > 0 {
            // Declaration identities need not follow the native topological insertion order.
            joints.push(FloatingTreeJointSpec {
                joint_id: 1000 - i as u64,
                parent_id: id - 1,
                child_id: id,
                anchor: [x - 0.09, 0.02, 0.7],
                axis: if i % 2 == 0 {
                    [0., 1., 0.]
                } else {
                    [0.6, 0., 0.8]
                },
            });
        }
    }
    FloatingTreeSpec {
        model_id: 51,
        links,
        joints,
    }
}
fn kill(pid: u32) -> Result<()> {
    assert!(
        Command::new("kill")
            .args(["-KILL", &pid.to_string()])
            .status()?
            .success()
    );
    Ok(())
}

#[test]
fn cold_warm_all_joint_inputs_survive_real_worker_loss_and_loaded_transfer() -> Result<()> {
    let exe = Path::new(EXE);
    let spec = serial(6);
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut atomic = FloatingTreePool::create(exe, a.path(), 3, spec.clone())?;
    let mut separate = FloatingTreePool::create(exe, b.path(), 3, spec.clone())?;
    let joints = spec.joints.iter().map(|j| j.joint_id).collect::<Vec<_>>();
    for (index, &joint) in joints.iter().enumerate() {
        let torque = index as f64 * 0.01;
        atomic.queue_effort(joint, torque)?;
        separate.queue_effort(joint, torque)?;
    }
    assert!(atomic.image().is_none());
    drop(atomic);
    atomic = FloatingTreePool::resume(exe, a.path())?;
    assert_eq!(atomic.pending_joint_effort(joints[0]), Some(0.0));
    assert_eq!(atomic.pending_joint_effort(joints[4]), Some(0.04));
    assert_eq!(atomic.observation(), separate.observation());
    for tick in 0..256 {
        if matches!(tick, 0 | 60 | 188) {
            let slot = (atomic.owner() + 1) % atomic.width();
            let old = atomic.worker_pid(slot).unwrap();
            kill(old)?;
            atomic.transfer(slot)?;
            separate.transfer(slot)?;
            assert_ne!(atomic.worker_pid(slot), Some(old));
        }
        if tick == 75 {
            let owner = atomic.owner();
            let other = (owner + 1) % 3;
            let pid = atomic.worker_pid(owner).unwrap();
            let idle = atomic.worker_pid(other).unwrap();
            atomic.replace_worker(other)?;
            assert_eq!(atomic.worker_pid(owner), Some(pid));
            assert_ne!(atomic.worker_pid(other), Some(idle));
            kill(pid)?;
        }
        if tick == 145 {
            atomic.drop_next_acknowledgement(atomic.owner())?;
        }
        let joint = joints[tick as usize % joints.len()];
        let torque = if tick % 8 < 4 { 0.075 } else { -0.096 };
        separate.queue_effort(joint, torque)?;
        separate.advance()?;
        atomic.advance_with_effort_at(0, tick, joint, torque)?;
        assert!(
            atomic
                .observation()
                .joints
                .iter()
                .all(|s| s.pending_effort.is_none())
        );
        assert_eq!(atomic.image(), separate.image());
        assert_eq!(atomic.observation(), separate.observation());
        assert_eq!(
            fs::read(a.path().join(ARCHIVE))?,
            fs::read(b.path().join(ARCHIVE))?
        );
        if matches!(tick, 65 | 201) {
            drop(atomic);
            atomic = FloatingTreePool::open_scene(exe, a.path(), None, spec.clone())?;
        }
    }
    assert!(atomic.observation().joints.iter().any(|s| s.q.abs() > 1e-4));
    assert!(
        atomic
            .observation()
            .joints
            .windows(2)
            .all(|w| w[0].joint_id < w[1].joint_id)
    );
    Ok(())
}

#[test]
fn largest_supported_tree_survives_warm_migration_and_per_joint_reopen() -> Result<()> {
    let exe = Path::new(EXE);
    let spec = serial(gz_physics_multifidelity::MAX_FLOATING_TREE_LINKS);
    let dir = tempfile::tempdir()?;
    let mut p = FloatingTreePool::create(exe, dir.path(), 2, spec.clone())?;
    for (i, j) in spec.joints.iter().enumerate() {
        p.queue_effort(j.joint_id, i as f64 * 0.00003)?;
    }
    p.advance()?;
    assert!(p.image().unwrap().len() <= gz_physics_multifidelity::FLOATING_TREE_IMAGE_LIMIT);
    for (i, j) in spec.joints.iter().enumerate() {
        p.queue_effort(j.joint_id, -(i as f64) * 0.00004)?;
    }
    let expected = p.observation().clone();
    let image = p.image().unwrap().to_owned();
    p.transfer(1)?;
    drop(p);
    let mut p = FloatingTreePool::resume(exe, dir.path())?;
    assert_eq!(&expected, p.observation());
    assert_eq!(p.image().unwrap(), image);
    p.advance()?;
    assert_eq!(p.observation().links.len(), 64);
    assert_eq!(p.observation().joints.len(), 63);
    assert!(
        p.observation()
            .joints
            .iter()
            .all(|j| j.pending_effort.is_none())
    );
    Ok(())
}

#[test]
fn failed_publication_fences_and_other_profile_openers_preserve_tree_barrier() -> Result<()> {
    let exe = Path::new(EXE);
    let dir = tempfile::tempdir()?;
    let mut p = FloatingTreePool::create(exe, dir.path(), 2, serial(3))?;
    let joint = p.spec().joints[0].joint_id;
    p.queue_effort(joint, -0.01)?;
    let file = dir.path().join(ARCHIVE);
    let before = fs::read(&file)?;
    let staging = dir.path().join("barrier.pending.gzhinge");
    fs::create_dir(&staging)?;
    assert!(matches!(
        p.advance_with_effort_at(0, 0, joint, 0.05),
        Err(ArticulationError::Publication(_))
    ));
    assert!(p.check_interval(0, 0).is_err());
    assert_eq!(fs::read(&file)?, before);
    drop(p);
    fs::remove_dir(&staging)?;
    assert!(ArticulationPool::resume(exe, dir.path()).is_err());
    assert!(MovingPool::resume(exe, dir.path()).is_err());
    assert!(MultiSpherePool::resume(exe, dir.path()).is_err());
    let mut other = serial(3);
    other.links[0].inertia_center[2] = 0.02;
    assert!(FloatingTreePool::open_scene(exe, dir.path(), None, other).is_err());
    assert_eq!(fs::read(&file)?, before);
    let mut recovered = FloatingTreePool::resume(exe, dir.path())?;
    assert_eq!(recovered.pending_joint_effort(joint), Some(-0.01));
    recovered.advance_with_effort_at(0, 0, joint, 0.05)?;
    recovered.reset()?;
    assert_eq!(recovered.epoch(), 1);
    assert_eq!(recovered.observation().tick, 0);
    assert!(matches!(
        recovered.advance_with_effort_at(0, 0, joint, 0.025),
        Err(ArticulationError::StaleInterval { .. })
    ));
    Ok(())
}

#[test]
fn all_sixty_three_motor_settings_and_independent_torques_survive_one_warm_cut() -> Result<()> {
    use gz_physics::JointVelocityMotor;
    use gz_physics_distributed::floating_tree::{MotorTreePool, MotorTreeSpec};
    let exe = Path::new(EXE);
    let dir = tempfile::tempdir()?;
    let spec = MotorTreeSpec {
        tree: serial(64),
        limits: Vec::new(),
    };
    let mut p = MotorTreePool::create(exe, dir.path(), 2, spec.clone())?;
    p.advance()?;
    let mut inputs = Vec::new();
    for (idx, joint) in spec.tree.joints.iter().enumerate() {
        let m = JointVelocityMotor {
            target_velocity: (idx as f64 - 31.) * 0.0015,
            max_force: idx as f64 * 0.00045,
        };
        let held = -(idx as f64) * 0.0002;
        p.set_velocity_motor(joint.joint_id, Some(m))?;
        p.queue_effort(joint.joint_id, held)?;
        inputs.push((joint.joint_id, m, held));
    }
    let image = p.image().unwrap().to_owned();
    let actual = image.len();
    assert!(actual <= gz_physics_multifidelity::MOTOR_TREE_IMAGE_LIMIT);
    let archive = dir.path().join("barrier.gzmotorizedtree");
    let before = fs::read(&archive)?;
    let bad = |target_velocity, max_force| {
        Some(JointVelocityMotor {
            target_velocity,
            max_force,
        })
    };
    for (joint, m) in [
        (inputs[0].0, bad(f64::NAN, 0.1)),
        (u64::MAX, None),
        (inputs[0].0, bad(1., -0.01)),
    ] {
        assert!(p.set_velocity_motor(joint, m).is_err());
        assert_eq!(fs::read(&archive)?, before);
    }
    kill(p.worker_pid(p.owner()).unwrap())?;
    p.transfer(1)?;
    drop(p);
    p = MotorTreePool::resume(exe, dir.path())?;
    assert_eq!(p.image().unwrap(), image);
    for (j, m, held) in &inputs {
        assert_eq!(p.velocity_motor(*j), Some(*m));
        assert_eq!(p.pending_joint_effort(*j), Some(*held));
    }
    let staging = dir.path().join("barrier.pending.gzhinge");
    fs::create_dir(&staging)?;
    assert!(matches!(
        p.set_velocity_motor(inputs[0].0, None),
        Err(ArticulationError::Publication(_))
    ));
    assert!(p.check_interval(0, 1).is_err());
    drop(p);
    fs::remove_dir(&staging)?;
    p = MotorTreePool::resume(exe, dir.path())?;
    assert_eq!(p.image().unwrap(), image);
    p.advance()?;
    for (j, m, _) in &inputs {
        assert_eq!(p.pending_joint_effort(*j), None);
        assert_eq!(p.velocity_motor(*j), Some(*m));
    }
    p.reset()?;
    assert!(
        inputs
            .iter()
            .all(|(j, _, _)| p.velocity_motor(*j).is_none())
    );
    eprintln!(
        "maximum motors=63 held_efforts=63 native_bytes={actual} native_limit={} archive_bytes={}",
        gz_physics_multifidelity::MOTOR_TREE_IMAGE_LIMIT,
        before.len()
    );
    Ok(())
}

#[test]
fn eight_link_loaded_floor_fits_wire_and_archive_and_preserves_every_warm_reaction() -> Result<()> {
    use gz_physics_distributed::{
        articulation::ArticulationGroundSpec,
        floating_tree::{GroundedTreePool, GroundedTreeSpec},
    };
    let exe = Path::new(EXE);
    let dir = tempfile::tempdir()?;
    let reference = tempfile::tempdir()?;
    let mut tree = serial(8);
    for link in &mut tree.links {
        link.position[2] = link.radius + 0.003;
        link.velocity = [0.12, -0.07, -0.18];
    }
    for joint in &mut tree.joints {
        joint.anchor[2] = tree.links[0].position[2];
    }
    let spec = GroundedTreeSpec {
        tree,
        limits: Vec::new(),
        ground: ArticulationGroundSpec {
            link_id: 4,
            height: 0.,
            size: [20.; 2],
            gravity_z: -9.81,
        },
    };
    let mut p = GroundedTreePool::create(exe, dir.path(), 3, spec.clone())?;
    let mut control = GroundedTreePool::create(exe, reference.path(), 3, spec.clone())?;
    assert_eq!(p.ground_reactions().len(), 8);
    let file = dir.path().join("barrier.gzgroundedtree");
    let file_ref = reference.path().join("barrier.gzgroundedtree");
    let mut maximum_contacts = 0;
    let mut maximum_bytes = 0;
    let mut maximum_archive = 0;
    let mut recoveries = 0;
    let mut saw_friction = false;
    for tick in 0..230 {
        let j = spec.tree.joints[(tick as usize) % 7].joint_id;
        p.advance_with_effort_at(0, tick, j, 0.002)?;
        control.queue_effort(j, 0.002)?;
        control.advance()?;
        let active = p
            .ground_reactions()
            .iter()
            .filter(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
            .count();
        maximum_contacts = maximum_contacts.max(active);
        saw_friction |= p
            .ground_reactions()
            .iter()
            .any(|r| r.tangent_impulse.iter().any(|v| v.abs() > 1e-8));
        assert_eq!(p.ground_reactions(), control.ground_reactions());
        assert_eq!(p.observation(), control.observation());
        assert_eq!(p.image(), control.image());
        let bytes = fs::read(&file)?;
        assert_eq!(bytes, fs::read(&file_ref)?);
        maximum_bytes = maximum_bytes.max(p.image().unwrap().len());
        maximum_archive = maximum_archive.max(bytes.len());
        if active >= 6 && tick % 40 == 0 {
            let destination = (p.owner() + 1) % p.width();
            let held = spec.tree.joints[0].joint_id;
            for pool in [&mut p, &mut control] {
                pool.queue_effort(held, 0.071)?;
            }
            kill(p.worker_pid(destination).unwrap())?;
            p.transfer(destination)?;
            control.transfer(destination)?;
            drop(p);
            p = GroundedTreePool::resume(exe, dir.path())?;
            assert_eq!(p.ground_reactions(), control.ground_reactions());
            assert_eq!(p.pending_joint_effort(held), Some(0.071));
            recoveries += 1;
        }
    }
    eprintln!(
        "eight-link loaded floor: maximum_simultaneous_warm_loaded_links={maximum_contacts} maximum_native_bytes={maximum_bytes} maximum_archive_bytes={maximum_archive} native_limit={} recoveries={recoveries} tangential_impulse={saw_friction}",
        gz_physics_multifidelity::GROUNDED_TREE_IMAGE_LIMIT
    );
    assert_eq!(maximum_contacts, 8);
    assert!(recoveries >= 2 && saw_friction);
    assert!(maximum_bytes <= gz_physics_multifidelity::GROUNDED_TREE_IMAGE_LIMIT);
    assert!(maximum_archive <= 192 * 1024 + 40);
    Ok(())
}

use gz_physics_distributed::floating_tree::{
    FloatingTreeJointLimit, PayloadTreePool, PayloadTreeSpec,
};
#[path = "support/tree_payload_fixture.rs"]
mod payload_fixture;

#[test]
fn eight_links_exchange_two_way_impulses_with_a_mobile_payload_in_either_authored_order()
-> Result<()> {
    use glam::DVec3;
    let exe = Path::new(EXE);
    for (payload_first, mass) in [(true, 0.24), (false, 1.15)] {
        let mut spec = payload_fixture::cage(8, payload_first, mass, [0.26, 0.074, -0.021]);
        spec.exact_revolute_locks = true;
        spec.payload_can_sleep = false;
        let a = tempfile::tempdir()?;
        let b = tempfile::tempdir()?;
        let mut pool = PayloadTreePool::create(exe, a.path(), 3, spec.clone())?;
        let mut control = PayloadTreePool::create(exe, b.path(), 3, spec.clone())?;
        let diagnostics = std::env::var_os("GZ_TREE_PAYLOAD_PROBE").map(|d| {
            Path::new(&d).join(if payload_first {
                "light-first"
            } else {
                "heavy-last"
            })
        });
        let mut previous_cut = fs::read(a.path().join("barrier.gzpayloadtree"))?;
        let mut rows=String::from("tick,active,warm,tree_px,tree_py,tree_pz,payload_px,payload_py,payload_pz,impulse_x,impulse_y,impulse_z,total_drift
");
        if let Some(d) = &diagnostics {
            fs::create_dir_all(d)?;
            fs::write(d.join("initial.gzpayloadtree"), &previous_cut)?;
            fs::write(
                d.join("construction.json"),
                serde_json::to_vec_pretty(&spec)?,
            )?;
        }
        let p0 = DVec3::from_array(spec.payload.velocity) * mass;
        let mut cumulative = DVec3::ZERO;
        let mut max_payload_error = 0f64;
        let mut max_total_error = 0f64;
        let mut max_payload_delta = 0f64;
        let mut max_image = 0;
        let mut max_file = 0;
        let mut peak = 0;
        let mut peak_warm = 0;
        let mut transfers = 0;
        for tick in 0..280 {
            let id = spec.tree.joints[tick % 7].joint_id;
            let e = if tick % 15 < 7 { 0.036 } else { -0.024 };
            pool.advance_with_effort_at(0, tick as u64, id, e)?;
            control.queue_effort(id, e)?;
            control.advance()?;
            assert_eq!(pool.observation(), control.observation());
            assert_eq!(pool.payload_state(), control.payload_state());
            assert_eq!(pool.payload_reactions(), control.payload_reactions());
            assert_eq!(pool.image(), control.image());
            let file = fs::read(a.path().join("barrier.gzpayloadtree"))?;
            assert_eq!(file, fs::read(b.path().join("barrier.gzpayloadtree"))?);
            max_file = max_file.max(file.len());
            max_image = max_image.max(pool.image().unwrap().len());
            let active = pool
                .payload_reactions()
                .iter()
                .filter(|r| r.active && r.normal_impulse > 0.)
                .count();
            let warm = pool
                .payload_reactions()
                .iter()
                .filter(|r| r.active && r.warmstart_normal_impulse > 0. && r.normal_impulse > 0.)
                .count();
            peak = peak.max(active);
            peak_warm = peak_warm.max(warm);
            for r in pool.payload_reactions() {
                cumulative += DVec3::from_array(r.normal) * r.normal_impulse
                    + DVec3::from_array(r.tangent_impulse);
            }
            let payload = DVec3::from_array(pool.payload_state().velocity) * mass;
            let tree: DVec3 = pool
                .observation()
                .links
                .iter()
                .zip(&spec.tree.links)
                .map(|(s, l)| DVec3::from_array(s.velocity) * l.mass)
                .sum();
            max_payload_delta = max_payload_delta.max((payload - p0).length());
            max_payload_error = max_payload_error.max((payload - p0 + cumulative).length());
            let residual = (payload + tree - p0).length();
            if let Some(d) = &diagnostics {
                use std::fmt::Write;
                writeln!(
                    rows,
                    "{},{active},{warm},{},{},{},{},{},{},{},{},{},{}",
                    tick + 1,
                    tree.x,
                    tree.y,
                    tree.z,
                    payload.x,
                    payload.y,
                    payload.z,
                    cumulative.x,
                    cumulative.y,
                    cumulative.z,
                    residual
                )?;
                if residual > max_total_error {
                    fs::write(d.join("peak-before.gzpayloadtree"), &previous_cut)?;
                    fs::write(d.join("peak.gzpayloadtree"), &file)?;
                }
                previous_cut = file;
            }
            max_total_error = max_total_error.max(residual);
            if warm >= 2 && tick % 35 == 0 && transfers < 4 {
                let destination = (pool.owner() + 1) % pool.width();
                let killed = if transfers % 2 == 0 {
                    pool.owner()
                } else {
                    destination
                };
                kill(pool.worker_pid(killed).unwrap())?;
                pool.transfer(destination)?;
                control.transfer(destination)?;
                drop(pool);
                pool = PayloadTreePool::resume(exe, a.path())?;
                assert_eq!(pool.payload_reactions(), control.payload_reactions());
                assert_eq!(pool.payload_state(), control.payload_state());
                transfers += 1;
            }
        }
        eprintln!(
            "payload_first={payload_first} mass={mass} frames=280 max_active={peak} max_warm={peak_warm} transfers={transfers} native_bytes={max_image} archive_bytes={max_file} max_payload_dp={max_payload_delta} impulse_closure={max_payload_error} total_momentum_drift={max_total_error}"
        );
        if let Some(d) = diagnostics {
            fs::write(d.join("history.csv"), rows)?;
            fs::copy(
                a.path().join("barrier.gzpayloadtree"),
                d.join("final.gzpayloadtree"),
            )?;
        }
        assert!(peak >= 2 && peak_warm >= 2 && transfers >= 1);
        assert!(max_image <= gz_physics_multifidelity::PAYLOAD_TREE_IMAGE_LIMIT);
        assert!(max_payload_delta > 1e-3 && max_payload_error < 1e-8 && max_total_error < 1e-6);
        assert!(
            pool.observation()
                .joints
                .iter()
                .all(|j| j.q == 0. && j.qdot == 0.)
        );
    }
    Ok(())
}

#[test]
fn frozen_v2_payload_archive_retains_default_locked_stop_behavior() -> Result<()> {
    let exe = Path::new(EXE);
    let archive = include_bytes!(
        "fixtures/payload-v2-control.gzpayloadtree"
    );
    let a = tempfile::tempdir()?;
    fs::write(a.path().join("barrier.gzpayloadtree"), archive)?;
    let mut p = PayloadTreePool::resume(exe, a.path())?;
    assert!(!p.spec().exact_revolute_locks);
    assert!(p.spec().payload_can_sleep);
    assert_eq!(
        p.worker_profile(),
        gz_physics_distributed::floating_tree::FloatingTreeWorkerProfile::PayloadV2
    );
    assert_eq!(p.observation().tick, 15);
    let original = std::env::var_os("GZ_PAYLOAD_V2_WORKER");
    let b = tempfile::tempdir()?;
    fs::write(b.path().join("barrier.gzpayloadtree"), archive)?;
    let mut legacy = if let Some(old) = &original {
        Some(PayloadTreePool::resume(Path::new(old), b.path())?)
    } else {
        None
    };
    for tick in 15..70 {
        let id = p.spec().tree.joints[tick % 7].joint_id;
        p.advance_with_effort(id, 0.036)?;
        if let Some(q) = &mut legacy {
            q.advance_with_effort(id, 0.036)?;
            assert_eq!(p.image(), q.image());
            assert_eq!(p.payload_state(), q.payload_state());
            assert_eq!(p.payload_reactions(), q.payload_reactions());
            assert_eq!(
                fs::read(a.path().join("barrier.gzpayloadtree"))?,
                fs::read(b.path().join("barrier.gzpayloadtree"))?
            );
        }
    }
    assert_eq!(
        &fs::read(a.path().join("barrier.gzpayloadtree"))?[..8],
        b"GZFPCU02"
    );
    let mut different = p.spec().clone();
    different.exact_revolute_locks = true;
    drop(p);
    assert!(PayloadTreePool::open_scene(exe, a.path(), Some(2), different).is_err());
    eprintln!(
        "v2_reopened_tick=15 continued=55 matched_frozen_v2_binary={}",
        legacy.is_some()
    );
    Ok(())
}
