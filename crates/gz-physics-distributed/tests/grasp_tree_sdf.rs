// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Match every public receipt and complete future solver graph to an uninterrupted Bevy gripper.
use glam::{DQuat, DVec3};
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_physics_distributed::floating_tree::{GraspTreePool, GraspTreeSpec};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::entity_by_id;
use gz_sim_scene::{BodyHandle, JointDefinition, JointHandle, PhysicsEngine};
use sha2::{Digest, Sha256};
use std::{error::Error, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_gripper_payload.sdf");

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
enum Grip {
    Strong,
    Weak,
    PreloadedWeak,
    Open,
}
#[derive(Default, serde::Serialize)]
struct Measures {
    both: usize,
    free: usize,
    warm_transfers: usize,
    slip: f64,
    transferred: f64,
    closure: f64,
    drift: f64,
    travel: f64,
    extra_deaths: usize,
    reopens: usize,
    digest: String,
}
#[path = "support/grasp_campaign.rs"]
mod campaign;
use campaign::{Journal, Program};
fn trial(config: &Program) -> Result<Measures> {
    let grip = config.grip;
    let mass = config.mass;
    let load = config.load[2];
    let xml = config.xml();
    let spec = gz_sim_scene::distributed_payload_tree_spec(&xml, "gripper.sdf", "default")?;
    let exe = config.worker();
    let exe = exe.as_path();
    let dir = tempfile::tempdir()?;
    let mut journal = Journal::new(dir.path(), config, &xml)?;
    let mut pool = GraspTreePool::create(
        exe,
        dir.path(),
        3,
        GraspTreeSpec {
            payload: spec.clone(),
        },
    )?;
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "gripper.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    gz_sim_scene::set_model_fidelity(&mut sim, spec.tree.model_id, ArticulationTier::Dynamic)?;
    let world = sim.world();
    let handle = |id| {
        world
            .get::<BodyHandle>(entity_by_id(world, id).unwrap())
            .unwrap()
            .id()
    };
    let payload_handle = handle(spec.payload.id);
    let links: Vec<_> = spec.tree.links.iter().map(|l| handle(l.id)).collect();
    let joints: Vec<_> = spec
        .tree
        .joints
        .iter()
        .map(|j| {
            let e = entity_by_id(world, j.joint_id).unwrap();
            let sign = if world.get::<JointDefinition>(e).unwrap().0.name == "right_close" {
                1.
            } else {
                -1.
            };
            (j.joint_id, world.get::<JointHandle>(e).unwrap().id(), sign)
        })
        .collect();
    let mut measure = Measures::default();
    let mut contact = DVec3::ZERO;
    let mut external = DVec3::ZERO;
    let mut at_load = DVec3::ZERO;
    let mut hash = Sha256::new();
    let mut load_requested = 0.;
    let mut transferred = DVec3::ZERO;
    for tick in 0..config.end {
        journal.pin(tick)?;
        if tick == 0 || tick == config.open_at || tick == config.load_at {
            for &(id, handle, sign) in &joints {
                let native = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                if native.is_exact_revolute_lock(handle)? {
                    assert!(
                        pool.set_velocity_motor(
                            id,
                            Some(JointVelocityMotor {
                                target_velocity: sign,
                                max_force: 1.
                            })
                        )
                        .is_err()
                    );
                    continue;
                }
                let motor = config.motor(tick, sign);
                pool.set_velocity_motor(id, motor)?;
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .set_joint_velocity_motor(handle, motor)?;
                assert_eq!(pool.velocity_motor(id), motor);
            }
        }
        let force = config.force(tick);
        let torques = if tick % config.torque_period == 0 {
            vec![(joints[tick % joints.len()].0, 0.001)]
        } else {
            vec![]
        };
        if tick == config.load_at {
            let pending = 0.004;
            pool.queue_effort(joints[0].0, pending)?;
            gz_sim_scene::queue_joint_effort(&mut sim, joints[0].0, pending)?;
            let cut = std::fs::read(dir.path().join("barrier.gzgrasptree"))?;
            for rejected in [[f64::NAN, 0., 0.], [0., f64::INFINITY, 0.]] {
                assert!(
                    pool.advance_with_payload_force_at(0, tick as u64, &[], rejected)
                        .is_err()
                );
            }
            assert!(
                pool.advance_with_payload_force_at(0, tick as u64 + 1, &[], force)
                    .is_err()
            );
            assert!(
                pool.advance_with_payload_force_at(
                    0,
                    tick as u64,
                    &[(joints[0].0, 0.1), (joints[0].0, -0.1)],
                    force
                )
                .is_err()
            );
            assert_eq!(cut, std::fs::read(dir.path().join("barrier.gzgrasptree"))?);
            assert_eq!(pool.pending_joint_effort(joints[0].0), Some(pending));
            pool.drop_next_acknowledgement(pool.owner())?;
        }
        for &(id, effort) in &torques {
            gz_sim_scene::queue_joint_effort(&mut sim, id, effort)?;
        }
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(payload_handle, force, [0.; 3])?;
        sim.step_exact(1)?;
        pool.advance_with_payload_force_at(0, tick as u64, &torques, force)?;
        let backend = sim.world().resource::<PhysicsBackend>();
        let native = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
        let graph = native.checkpoint_grasp_revolute_tree()?;
        if pool.image().unwrap() != graph {
            journal.oracle(&graph)?;
            panic!("native graph mismatch seed={} tick={tick}", config.seed);
        }
        hash.update(graph);
        let p = backend.state(payload_handle).unwrap();
        let current = pool.payload_state();
        assert_eq!(
            current.position.map(f64::to_bits),
            p.position.map(f64::to_bits)
        );
        assert_eq!(
            current.orientation.map(f64::to_bits),
            p.orientation.map(f64::to_bits)
        );
        assert_eq!(
            current.velocity.map(f64::to_bits),
            p.velocity.map(f64::to_bits)
        );
        assert_eq!(
            current.angular_velocity.map(f64::to_bits),
            p.angular_velocity.map(f64::to_bits)
        );
        let mut tick_impulse = DVec3::ZERO;
        let mut normals = Vec::new();
        let mut tree_momentum = DVec3::ZERO;
        for ((&id, l), (r, s)) in links.iter().zip(&spec.tree.links).zip(
            pool.payload_reactions()
                .iter()
                .zip(&pool.observation().links),
        ) {
            assert_eq!(*r, native.payload_tree_reaction(id, payload_handle)?);
            let link = backend.state(id).unwrap();
            assert_eq!(
                s.position.map(f64::to_bits),
                link.position.map(f64::to_bits)
            );
            assert_eq!(
                s.orientation.map(f64::to_bits),
                link.orientation.map(f64::to_bits)
            );
            assert_eq!(
                s.velocity.map(f64::to_bits),
                link.velocity.map(f64::to_bits)
            );
            assert_eq!(
                s.angular_velocity.map(f64::to_bits),
                link.angular_velocity.map(f64::to_bits)
            );
            tick_impulse += DVec3::from_array(r.normal) * r.normal_impulse
                + DVec3::from_array(r.tangent_impulse);
            if r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0. {
                normals.push(DVec3::from_array(r.normal));
            }
            tree_momentum += DVec3::from_array(s.velocity) * l.mass;
        }
        for &(id, handle, _) in &joints {
            let j = pool
                .observation()
                .joints
                .iter()
                .find(|s| s.joint_id == id)
                .unwrap();
            let state = backend.joint_state(handle).unwrap();
            assert_eq!(j.q.to_bits(), state.position.to_bits());
            assert_eq!(j.qdot.to_bits(), state.velocity.to_bits());
            assert_eq!(
                pool.velocity_motor(id),
                native.motor_revolute_velocity(handle)?
            );
        }
        contact += tick_impulse;
        external += DVec3::from_array(force) * 0.001;
        let part = DVec3::from_array(current.velocity) * mass;
        measure.closure = measure.closure.max((part + contact - external).length());
        measure.drift = measure
            .drift
            .max((part + tree_momentum - external).length());
        let palm = &pool.observation().links[0];
        let relative = DQuat::from_array(palm.orientation).inverse()
            * (DVec3::from_array(current.position) - DVec3::from_array(palm.position));
        if tick == config.load_at - 1 {
            at_load = relative;
        }
        let both = normals
            .iter()
            .enumerate()
            .any(|(i, n)| normals[i + 1..].iter().any(|m| n.dot(*m) < -0.65));
        if (config.load_at..config.open_at).contains(&tick) {
            measure.both += usize::from(both);
            measure.slip = measure.slip.max((relative - at_load).length());
            load_requested += force[2] * 0.001;
            transferred += tick_impulse;
            if both
                && (measure.warm_transfers == 0 || tick % config.transfer_period == 0)
                && measure.warm_transfers < 4
            {
                let dest = (pool.owner() + 1) % pool.width();
                let kill_slot = if measure.warm_transfers % 2 == 0 {
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
                let saved = pool.image().unwrap().to_owned();
                let reactions = pool.payload_reactions().to_vec();
                pool.transfer(dest)?;
                drop(pool);
                pool = GraspTreePool::resume(exe, dir.path())?;
                assert_eq!(pool.image().unwrap(), saved);
                assert_eq!(pool.payload_reactions(), reactions);
                measure.warm_transfers += 1;
            }
        }
        if config.seed != 0 && (tick == config.death_at || tick == config.open_at + 19) {
            let slot = if tick == config.death_at {
                pool.owner()
            } else {
                (pool.owner() + 1) % pool.width()
            };
            let pid = pool.worker_pid(slot).unwrap();
            assert!(
                Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
            pool.replace_worker(slot)?;
            measure.extra_deaths += 1;
        }
        if config.seed != 0 && (tick == config.load_at + 7 || tick == config.open_at + 31) {
            let expected = pool.image().unwrap().to_vec();
            drop(pool);
            pool = GraspTreePool::resume(exe, dir.path())?;
            assert_eq!(pool.image().unwrap(), expected);
            measure.reopens += 1;
        }
        if measure.closure >= 1e-9 || measure.drift >= 0.005 {
            journal.oracle(&graph_at(&sim)?)?;
            panic!(
                "physical bound seed={} tick={} payload_delta_p_error={} total_delta_p_error={}",
                config.seed,
                tick + 1,
                measure.closure,
                measure.drift
            );
        }
        if tick >= config.open_at {
            measure.free += usize::from(
                pool.payload_reactions()
                    .iter()
                    .all(|r| !r.active || r.normal_impulse == 0.),
            );
            measure.travel = measure.travel.max((relative - at_load).length());
        }
    }
    measure.transferred = transferred.z;
    measure.digest = format!("{:x}", hash.finalize());
    let end = config.end;
    eprintln!(
        "grip={grip:?} mass={mass} load_N={load} exact_frames={end} load_window_ms={} both_warm={} loaded_transfers_kills_reopens={} post_open_free={} peak_hold_slip_m={} peak_post_open_displacement_m={} transferred_z_Ns={} external_z_Ns={} payload_closure_Ns={} total_drift_Ns={} sha256={}",
        config.open_at - config.load_at,
        measure.both,
        measure.warm_transfers,
        measure.free,
        measure.slip,
        measure.travel,
        measure.transferred,
        load_requested,
        measure.closure,
        measure.drift,
        measure.digest
    );
    assert!(measure.closure < 1e-9 && measure.drift < 0.005);
    if config.mixed_lock {
        pool.reset()?;
        assert_eq!(pool.observation().tick, 0);
        assert!(
            pool.observation()
                .joints
                .iter()
                .all(|j| j.q == 0. && j.qdot == 0. && pool.velocity_motor(j.joint_id).is_none())
        );
    }
    journal.success = true;
    Ok(measure)
}

#[test]
fn capped_native_motors_grasp_a_loaded_part_then_release_it() -> Result<()> {
    let strong = trial(&Program::baseline(Grip::Strong))?;
    let weak = trial(&Program::baseline(Grip::Weak))?;
    let preloaded = trial(&Program::baseline(Grip::PreloadedWeak))?;
    assert!(preloaded.slip > strong.slip * 10.);
    let open = trial(&Program::baseline(Grip::Open))?;
    assert!(strong.both > 200 && strong.warm_transfers >= 3 && strong.free > 50);
    assert!(strong.slip < 0.35 * open.slip && strong.slip < 0.6 * weak.slip);
    assert!(strong.transferred > 0.2 && strong.transferred > weak.transferred * 1.5);
    Ok(())
}

fn graph_at(sim: &gz_sim_core::Simulation) -> Result<Vec<u8>> {
    Ok(sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .checkpoint_grasp_revolute_tree()?)
}
#[test]
fn mixed_exact_lock_and_finite_motor_survive_loaded_reopen_and_reset() -> Result<()> {
    let mut cfg = Program::baseline(Grip::Strong);
    cfg.seed = 1;
    cfg.mixed_lock = true;
    let result = trial(&cfg)?;
    assert_eq!(result.extra_deaths, 2);
    assert_eq!(result.reopens, 2);
    Ok(())
}
