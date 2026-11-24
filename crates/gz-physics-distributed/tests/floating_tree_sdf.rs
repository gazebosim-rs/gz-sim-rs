// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A regular authored Bevy world supplies the independent native and public-state reference.
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_physics_distributed::floating_tree::{
    FloatingTreePool, FloatingTreeWorkerProfile, MotorTreePool, StoppedTreePool, TreeConstruction,
    TreePool,
};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, PhysicsEngine, distributed_floating_tree_spec,
    distributed_motor_tree_spec, distributed_stopped_tree_spec, load_world_with_sdf_step,
    queue_joint_effort, set_model_fidelity,
};
use sha2::{Digest, Sha256};
use std::{error::Error, path::Path, process::Command};

fn compare<S: TreeConstruction>(
    ordinary: &Simulation,
    pool: &TreePool<S>,
    name: &str,
    image_hash: &mut Sha256,
) -> Result<(), Box<dyn Error>> {
    let world = ordinary.world();
    let backend = world.resource::<PhysicsBackend>();
    let solver = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
    let expected = match pool.worker_profile() {
        FloatingTreeWorkerProfile::MotorV1 => solver.checkpoint_motor_revolute_tree()?,
        FloatingTreeWorkerProfile::GroundedV1 => solver.checkpoint_grounded_revolute_tree()?,
        FloatingTreeWorkerProfile::PayloadV2 | FloatingTreeWorkerProfile::ExactPayloadV3 => {
            solver.checkpoint_payload_revolute_tree()?
        }
        _ => solver.checkpoint_floating_revolute_tree()?,
    };
    assert_eq!(
        pool.image(),
        Some(expected.as_slice()),
        "{} native tick {}",
        name,
        pool.observation().tick
    );
    image_hash.update(expected);
    for body in &pool.observation().links {
        let ent = entity_by_id(world, body.id).unwrap();
        let handle = world.get::<BodyHandle>(ent).unwrap();
        let other = backend.state(handle.id()).unwrap();
        assert_eq!(
            body.position.map(f64::to_bits),
            other.position.map(f64::to_bits)
        );
        assert_eq!(
            body.orientation.map(f64::to_bits),
            other.orientation.map(f64::to_bits)
        );
        assert_eq!(
            body.velocity.map(f64::to_bits),
            other.velocity.map(f64::to_bits)
        );
        assert_eq!(
            body.angular_velocity.map(f64::to_bits),
            other.angular_velocity.map(f64::to_bits)
        );
    }
    for joint in &pool.observation().joints {
        let ent = entity_by_id(world, joint.joint_id).unwrap();
        let handle = world.get::<JointHandle>(ent).unwrap();
        let other = backend.joint_state(handle.id()).unwrap();
        assert_eq!(joint.q.to_bits(), other.position.to_bits());
        assert_eq!(joint.qdot.to_bits(), other.velocity.to_bits());
    }
    Ok(())
}

fn qualify(xml: &str, name: &str) -> Result<(), Box<dyn Error>> {
    let spec = distributed_floating_tree_spec(xml, name, "default")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir()?;
    let mut pool = FloatingTreePool::create(exe, dir.path(), 3, spec.clone())?;
    let mut ordinary = load_world_with_sdf_step(
        xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    set_model_fidelity(&mut ordinary, spec.model_id, ArticulationTier::Dynamic)?;
    let mut image_hash = Sha256::new();
    let mut checkpoints = 0;
    for tick in 0..360 {
        let idx = tick as usize % spec.joints.len();
        let joint = spec.joints[idx].joint_id;
        let effort = if tick % 80 < 35 { 0.06 } else { -0.025 };
        if matches!(tick, 0 | 95 | 226) {
            for (n, j) in spec.joints.iter().enumerate() {
                let held = (n as f64 - 2.) * 0.007;
                pool.queue_effort(j.joint_id, held)?;
                queue_joint_effort(&mut ordinary, j.joint_id, held)?;
            }
            pool.transfer((pool.owner() + 1) % pool.width())?;
            drop(pool);
            pool = FloatingTreePool::resume(exe, dir.path())?;
            checkpoints += 1;
        }
        if tick == 131 || tick == 300 {
            let pid = pool.worker_pid(pool.owner()).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
        }
        if tick == 67 {
            pool.drop_next_acknowledgement(pool.owner())?;
        }
        queue_joint_effort(&mut ordinary, joint, effort)?;
        ordinary.step_exact(1)?;
        pool.advance_with_effort_at(0, tick, joint, effort)?;
        compare(&ordinary, &pool, name, &mut image_hash)?;
    }
    assert!(pool.observation().joints.iter().all(|j| j.q.abs() > 1e-6));
    eprintln!(
        "scene={name} links={} joints={} exact_native_frames=360 exact_public_frames=360 worker_deaths=2 transfers_and_reopens={checkpoints} native_sequence_sha256={:x}",
        spec.links.len(),
        spec.joints.len(),
        image_hash.finalize()
    );
    Ok(())
}
#[test]
fn branched_and_serial_sdf_trees_match_the_ordinary_bevy_solver_through_recovery()
-> Result<(), Box<dyn Error>> {
    qualify(
        include_str!("../../../examples/distributed_branched_tree.sdf"),
        "distributed_branched_tree.sdf",
    )?;
    qualify(
        include_str!("../../../examples/distributed_serial_tree.sdf"),
        "distributed_serial_tree.sdf",
    )
}

#[test]
fn ordinary_bevy_stop_loading_reversal_and_release_survive_worker_transfer()
-> Result<(), Box<dyn Error>> {
    let xml = include_str!("../../../examples/distributed_stopped_tree.sdf");
    let name = "distributed_stopped_tree.sdf";
    let spec = distributed_stopped_tree_spec(xml, name, "default")?;
    let dir = tempfile::tempdir()?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut pool = StoppedTreePool::create(exe, dir.path(), 3, spec.clone())?;
    assert_eq!(pool.worker_profile(), FloatingTreeWorkerProfile::StoppedV1);
    let mut ordinary = load_world_with_sdf_step(
        xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    set_model_fidelity(&mut ordinary, spec.tree.model_id, ArticulationTier::Dynamic)?;
    let limit = spec.limits[0];
    let mut hash = Sha256::new();
    let mut upper_hold = 0;
    let mut lower_hold = 0;
    let mut released = false;
    let mut transferred = [false; 2];
    let mut range = [f64::INFINITY, f64::NEG_INFINITY];
    for tick in 0..900 {
        let sign = if !(260..650).contains(&tick) { 1. } else { -1. };
        let efforts = [
            (limit.joint_id, 3.8 * sign),
            (spec.limits[1].joint_id, -2.1 * sign),
        ];
        for &(id, e) in &efforts {
            queue_joint_effort(&mut ordinary, id, e)?;
        }
        ordinary.step_exact(1)?;
        pool.advance_with_efforts_at(0, tick, &efforts)?;
        compare(&ordinary, &pool, name, &mut hash)?;
        let state = pool
            .observation()
            .joints
            .iter()
            .find(|s| s.joint_id == limit.joint_id)
            .unwrap();
        range[0] = range[0].min(state.q);
        range[1] = range[1].max(state.q);
        let upper = (state.q - limit.upper).abs() < 1e-3 && state.qdot.abs() < 1e-2;
        let lower = (state.q - limit.lower).abs() < 1e-3 && state.qdot.abs() < 1e-2;
        upper_hold += usize::from(upper);
        lower_hold += usize::from(lower);
        released |= transferred[0] && tick > 260 && state.q < limit.upper - 0.006;
        for (index, loaded) in [upper, lower].into_iter().enumerate() {
            if loaded && !transferred[index] {
                let held = if index == 0 { 0.055 } else { -0.042 };
                pool.queue_effort(limit.joint_id, held)?;
                queue_joint_effort(&mut ordinary, limit.joint_id, held)?;
                let expected_image = pool.image().unwrap().to_owned();
                let destination = (pool.owner() + 1) % pool.width();
                let killed = pool.worker_pid(destination).unwrap();
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &killed.to_string()])
                        .status()?
                        .success()
                );
                pool.transfer(destination)?;
                assert_ne!(pool.worker_pid(destination), Some(killed));
                drop(pool);
                pool = StoppedTreePool::open_scene(exe, dir.path(), None, spec.clone())?;
                assert_eq!(pool.image().unwrap(), expected_image);
                assert_eq!(pool.pending_joint_effort(limit.joint_id), Some(held));
                transferred[index] = true;
            }
        }
    }
    eprintln!(
        "stopped branch: all_native_and_public_frames=900 joint={} min_q={} max_q={} stop_bounds=[{},{}] upper_held_ticks={upper_hold} lower_held_ticks={lower_hold} released={released} transfers={transferred:?} sha256={:x}",
        limit.joint_id,
        range[0],
        range[1],
        limit.lower,
        limit.upper,
        hash.finalize()
    );
    assert!(upper_hold > 10 && lower_hold > 10 && released && transferred == [true, true]);
    Ok(())
}

fn motor_bits(m: Option<JointVelocityMotor>) -> Option<[u64; 2]> {
    m.map(|m| [m.target_velocity.to_bits(), m.max_force.to_bits()])
}
fn motor_sdf(xml: &str, name: &str) -> Result<(), Box<dyn Error>> {
    let spec = distributed_motor_tree_spec(xml, name, "default")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir()?;
    let mut pool = MotorTreePool::create(exe, dir.path(), 3, spec.clone())?;
    let mut ordinary = load_world_with_sdf_step(
        xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    set_model_fidelity(&mut ordinary, spec.tree.model_id, ArticulationTier::Dynamic)?;
    let driven = spec
        .limits
        .first()
        .map_or(spec.tree.joints[0].joint_id, |l| l.joint_id);
    let second = spec
        .tree
        .joints
        .iter()
        .find(|j| j.joint_id != driven)
        .unwrap()
        .joint_id;
    let motor = |target_velocity, max_force| {
        Some(JointVelocityMotor {
            target_velocity,
            max_force,
        })
    };
    let mut hash = Sha256::new();
    let mut ranges = [f64::INFINITY, f64::NEG_INFINITY];
    let mut angles = [f64::INFINITY, f64::NEG_INFINITY];
    let mut changes = 0;
    let mut reads = 0;
    for tick in 0..960 {
        let commands = match tick {
            0 => vec![(driven, motor(2.5, 3.25)), (second, motor(-1.25, 1.5))],
            240 => vec![(driven, motor(-4.5, 2.8))],
            485 => vec![(second, None)],
            570 => vec![(driven, None)],
            650 => vec![(driven, motor(3.1, 1.7)), (second, motor(-0.0, 0.0))],
            795 => vec![(second, motor(1.7, 0.67))],
            _ => vec![],
        };
        for (id, setting) in commands {
            let entity = entity_by_id(ordinary.world(), id).unwrap();
            let handle = ordinary.world().get::<JointHandle>(entity).unwrap().id();
            ordinary
                .world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_joint_velocity_motor(handle, setting)?;
            pool.set_velocity_motor(id, setting)?;
            assert_eq!(pool.observation().tick, tick);
            assert_eq!(motor_bits(pool.velocity_motor(id)), motor_bits(setting));
            changes += 1;
        }
        if matches!(tick, 0 | 240 | 485 | 650) {
            let held = 0.007;
            queue_joint_effort(&mut ordinary, second, held)?;
            pool.queue_effort(second, held)?;
            let prior = pool.image().map(<[u8]>::to_vec);
            let destination = (pool.owner() + 1) % 3;
            let dead = pool.worker_pid(destination).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &dead.to_string()])
                    .status()?
                    .success()
            );
            pool.transfer(destination)?;
            drop(pool);
            pool = MotorTreePool::resume(exe, dir.path())?;
            assert_eq!(pool.image(), prior.as_deref());
            assert_eq!(pool.pending_joint_effort(second), Some(held));
        }
        let extra = if tick % 9 == 0 { 0.003 } else { -0.001 };
        queue_joint_effort(&mut ordinary, second, extra)?;
        ordinary.step_exact(1)?;
        pool.advance_with_effort_at(0, tick, second, extra)?;
        compare(&ordinary, &pool, name, &mut hash)?;
        let world = ordinary.world();
        let native = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        for joint in &pool.observation().joints {
            let entity = entity_by_id(world, joint.joint_id).unwrap();
            let handle = world.get::<JointHandle>(entity).unwrap().id();
            assert_eq!(
                motor_bits(pool.velocity_motor(joint.joint_id)),
                motor_bits(native.motor_revolute_velocity(handle)?)
            );
            reads += 1;
            if joint.joint_id == driven {
                angles[0] = angles[0].min(joint.q);
                angles[1] = angles[1].max(joint.q);
                ranges[0] = ranges[0].min(joint.qdot);
                ranges[1] = ranges[1].max(joint.qdot);
            }
        }
    }
    assert!(ranges[0] < -0.011 && ranges[1] > 0.011);
    pool.reset()?;
    assert!(
        pool.observation()
            .joints
            .iter()
            .all(|j| pool.velocity_motor(j.joint_id).is_none())
    );
    eprintln!(
        "motor scene={name} exact_native_and_public_frames=960 persistent_motor_reads={reads} changes={changes} transfers_and_reopens=4 min_q={} max_q={} min_qdot={} max_qdot={} sha256={:x}",
        angles[0],
        angles[1],
        ranges[0],
        ranges[1],
        hash.finalize()
    );
    Ok(())
}

#[test]
fn persistent_velocity_targets_match_ordinary_bevy_through_clear_reverse_and_worker_loss()
-> Result<(), Box<dyn Error>> {
    motor_sdf(
        include_str!("../../../examples/distributed_stopped_tree.sdf"),
        "stopped_motor.sdf",
    )?;
    motor_sdf(
        include_str!("../../../examples/distributed_branched_tree.sdf"),
        "free_motor.sdf",
    )
}

fn reaction_bits(r: &gz_physics_multifidelity::GroundReaction) -> (bool, [u64; 5]) {
    (
        r.active,
        [
            r.normal_impulse.to_bits(),
            r.warmstart_normal_impulse.to_bits(),
            r.tangent_impulse[0].to_bits(),
            r.tangent_impulse[1].to_bits(),
            r.tangent_impulse[2].to_bits(),
        ],
    )
}

#[test]
fn authored_grounded_leg_matches_native_contacts_through_loaded_handoff_and_release()
-> Result<(), Box<dyn Error>> {
    use gz_physics_distributed::floating_tree::GroundedTreePool;
    let xml = include_str!("../../../examples/distributed_grounded_leg.sdf");
    let name = "distributed_grounded_leg.sdf";
    let spec = gz_sim_scene::distributed_grounded_tree_spec(xml, name, "default")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir()?;
    let mut pool = GroundedTreePool::create(exe, dir.path(), 3, spec.clone())?;
    let mut ordinary = load_world_with_sdf_step(
        xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    set_model_fidelity(&mut ordinary, spec.tree.model_id, ArticulationTier::Dynamic)?;
    let mut history = Sha256::new();
    let mut previous = vec![false; spec.tree.links.len()];
    let mut releases = 0;
    let mut entries = 0;
    let mut warm_ticks = 0;
    let mut loaded_migrations = 0;
    let mut tangent = 0f64;
    let mut maximum_native = 0;
    for tick in 0..1150 {
        let sign = if (tick / 170) % 2 == 0 { 1. } else { -1. };
        let inputs = spec
            .tree
            .joints
            .iter()
            .enumerate()
            .map(|(i, j)| {
                (
                    j.joint_id,
                    if i % 2 == 0 { 1.2 * sign } else { -0.8 * sign },
                )
            })
            .collect::<Vec<_>>();
        for &(j, e) in &inputs {
            queue_joint_effort(&mut ordinary, j, e)?;
        }
        ordinary.step_exact(1)?;
        pool.advance_with_efforts_at(0, tick, &inputs)?;
        compare(&ordinary, &pool, name, &mut history)?;
        let world = ordinary.world();
        let floor = world
            .get::<BodyHandle>(entity_by_id(world, spec.ground.link_id).unwrap())
            .unwrap()
            .id();
        let solver = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut loaded = false;
        for (i, (link, reaction)) in spec
            .tree
            .links
            .iter()
            .zip(pool.ground_reactions())
            .enumerate()
        {
            let body = world
                .get::<BodyHandle>(entity_by_id(world, link.id).unwrap())
                .unwrap()
                .id();
            let expected = solver.ground_reaction(body, floor)?;
            assert_eq!(reaction_bits(reaction), reaction_bits(&expected));
            let contact = reaction.active && reaction.normal_impulse > 0.;
            entries += usize::from(contact && !previous[i]);
            releases += usize::from(!contact && previous[i]);
            previous[i] = contact;
            loaded |= contact && reaction.warmstart_normal_impulse > 0.;
            tangent = tangent.max(reaction.tangent_impulse.iter().map(|x| x.abs()).sum());
        }
        warm_ticks += usize::from(loaded);
        maximum_native = maximum_native.max(pool.image().unwrap().len());
        if loaded && tick % 110 == 0 && loaded_migrations < 5 {
            let dest = (pool.owner() + 1) % pool.width();
            let j = inputs[0].0;
            pool.queue_effort(j, -0.017)?;
            queue_joint_effort(&mut ordinary, j, -0.017)?;
            let reactions: Vec<_> = pool.ground_reactions().iter().map(reaction_bits).collect();
            let image = pool.image().unwrap().to_owned();
            let kill_slot = if loaded_migrations % 2 == 0 {
                pool.owner()
            } else {
                dest
            };
            let pid = pool.worker_pid(kill_slot).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            pool.transfer(dest)?;
            drop(pool);
            pool = GroundedTreePool::resume(exe, dir.path())?;
            assert_eq!(pool.image().unwrap(), image);
            assert_eq!(
                reactions,
                pool.ground_reactions()
                    .iter()
                    .map(reaction_bits)
                    .collect::<Vec<_>>()
            );
            assert_eq!(pool.pending_joint_effort(j), Some(-0.017));
            loaded_migrations += 1;
        }
    }
    eprintln!(
        "grounded leg: exact_native_public_and_reaction_frames=1150 entries={entries} releases={releases} warm_ticks={warm_ticks} loaded_migrations_and_kills={loaded_migrations} max_tangent_l1={tangent} max_native_bytes={maximum_native} sha256={:x}",
        history.finalize()
    );
    assert!(
        entries >= 2
            && releases >= 1
            && warm_ticks > 100
            && loaded_migrations >= 3
            && tangent > 1e-8
    );
    Ok(())
}

#[test]
fn authored_free_gripper_exchanges_impulse_with_both_fingers_then_releases_after_handoff()
-> Result<(), Box<dyn Error>> {
    use glam::DVec3;
    use gz_physics_distributed::floating_tree::PayloadTreePool;
    let xml = include_str!("../../../examples/distributed_gripper_payload.sdf");
    let name = "distributed_gripper_payload.sdf";
    let spec = gz_sim_scene::distributed_payload_tree_spec(xml, name, "default")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let dir = tempfile::tempdir()?;
    let mut pool = PayloadTreePool::create(exe, dir.path(), 3, spec.clone())?;
    let mut ordinary = load_world_with_sdf_step(
        xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    set_model_fidelity(&mut ordinary, spec.tree.model_id, ArticulationTier::Dynamic)?;
    let world = ordinary.world();
    let joints: Vec<_> = spec
        .tree
        .joints
        .iter()
        .map(|j| {
            let ent = entity_by_id(world, j.joint_id).unwrap();
            let def = world.get::<gz_sim_scene::JointDefinition>(ent).unwrap();
            (
                j.joint_id,
                match def.0.name.as_str() {
                    "right_close" => 0.8,
                    "left_close" => -0.8,
                    x => panic!("unexpected joint {x}"),
                },
            )
        })
        .collect();
    let payload_handle = world
        .get::<BodyHandle>(entity_by_id(world, spec.payload.id).unwrap())
        .unwrap()
        .id();
    let handles: Vec<_> = spec
        .tree
        .links
        .iter()
        .map(|l| {
            world
                .get::<BodyHandle>(entity_by_id(world, l.id).unwrap())
                .unwrap()
                .id()
        })
        .collect();
    let mut hash = Sha256::new();
    let mut impulse = DVec3::ZERO;
    let mut max_payload_closure = 0f64;
    let mut max_system_drift = 0f64;
    let mut max_dp = 0f64;
    let mut warm_ticks = 0;
    let mut released_ticks = 0;
    let mut migrations = 0;
    let mut max_native = 0;
    let mut first_warm = None;
    let mut first_release = None;
    let mut relative_motion = 0f64;
    for tick in 0..1000 {
        let sign = if tick < 470 { 1. } else { -1. };
        let torques: Vec<_> = joints.iter().map(|&(j, e)| (j, sign * e)).collect();
        for &(j, e) in &torques {
            queue_joint_effort(&mut ordinary, j, e)?;
        }
        ordinary.step_exact(1)?;
        pool.advance_with_efforts_at(0, tick, &torques)?;
        compare(&ordinary, &pool, name, &mut hash)?;
        let b = ordinary.world().resource::<PhysicsBackend>();
        let native = b.downcast_ref::<MultiFidelityBackend>().unwrap();
        let p = b.state(payload_handle).unwrap();
        let q = pool.payload_state();
        assert_eq!(q.position.map(f64::to_bits), p.position.map(f64::to_bits));
        assert_eq!(
            q.orientation.map(f64::to_bits),
            p.orientation.map(f64::to_bits)
        );
        assert_eq!(q.velocity.map(f64::to_bits), p.velocity.map(f64::to_bits));
        assert_eq!(
            q.angular_velocity.map(f64::to_bits),
            p.angular_velocity.map(f64::to_bits)
        );
        let mut active = Vec::new();
        for (&body, r) in handles.iter().zip(pool.payload_reactions()) {
            assert_eq!(*r, native.payload_tree_reaction(body, payload_handle)?);
            impulse += DVec3::from_array(r.normal) * r.normal_impulse
                + DVec3::from_array(r.tangent_impulse);
            if r.active && r.warmstart_normal_impulse > 0. && r.normal_impulse > 0. {
                active.push(DVec3::from_array(r.normal));
            }
        }
        let opposing = active
            .iter()
            .enumerate()
            .any(|(i, n)| active[i + 1..].iter().any(|m| n.dot(*m) < -0.65));
        if opposing {
            warm_ticks += 1;
            first_warm.get_or_insert(tick + 1);
        }
        if tick > 475
            && pool
                .payload_reactions()
                .iter()
                .all(|r| !r.active || r.normal_impulse == 0.)
        {
            released_ticks += 1;
            first_release.get_or_insert(tick + 1);
        }
        let payload_delta = (DVec3::from_array(q.velocity)
            - DVec3::from_array(spec.payload.velocity))
            * spec.payload.mass;
        let tree_delta: DVec3 = pool
            .observation()
            .links
            .iter()
            .zip(&spec.tree.links)
            .map(|(state, link)| {
                (DVec3::from_array(state.velocity) - DVec3::from_array(link.velocity)) * link.mass
            })
            .sum();
        max_dp = max_dp.max(payload_delta.length());
        max_payload_closure = max_payload_closure.max((payload_delta + impulse).length());
        max_system_drift = max_system_drift.max((payload_delta + tree_delta).length());
        relative_motion = relative_motion.max(
            (DVec3::from_array(q.position) - DVec3::from_array(spec.payload.position)).length(),
        );
        max_native = max_native.max(pool.image().unwrap().len());
        if opposing && (migrations == 0 || tick % 43 == 0) && migrations < 5 {
            let dest = (pool.owner() + 1) % pool.width();
            let dead = if migrations % 2 == 0 {
                pool.owner()
            } else {
                dest
            };
            let reactions = pool.payload_reactions().to_vec();
            let state = *pool.payload_state();
            let image = pool.image().unwrap().to_owned();
            let pid = pool.worker_pid(dead).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            pool.transfer(dest)?;
            drop(pool);
            pool = PayloadTreePool::resume(exe, dir.path())?;
            assert_eq!(pool.image().unwrap(), image);
            assert_eq!(pool.payload_reactions(), reactions);
            assert_eq!(pool.payload_state(), &state);
            migrations += 1;
        }
    }
    eprintln!(
        "scene={name} exact_native_and_public_and_reaction_frames=1000 first_warm={first_warm:?} opposing_warm_ticks={warm_ticks} first_release={first_release:?} released_ticks={released_ticks} loaded_transfers_worker_kills_reopens={migrations} max_native_bytes={max_native} max_payload_dp={max_dp} max_payload_displacement={relative_motion} payload_impulse_closure={max_payload_closure} total_native_momentum_drift={max_system_drift} sha256={:x}",
        hash.finalize()
    );
    assert!(warm_ticks > 50 && released_ticks > 50 && migrations >= 3);
    assert!(max_dp > 1e-3 && max_payload_closure < 1e-9 && max_system_drift < 5e-3);
    Ok(())
}
