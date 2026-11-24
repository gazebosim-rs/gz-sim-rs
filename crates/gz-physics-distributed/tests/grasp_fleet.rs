// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_physics_distributed::{
    articulation::{ArticulationError, ArticulationLinkSpec},
    floating_tree::{
        FloatingTreeJointLimit, FloatingTreeJointSpec, FloatingTreeSpec, GraspFleetPlacement,
        GraspFleetPool, GraspFleetSpec, GraspTreeSpec, PayloadTreeSpec,
    },
};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::entity_by_id;
use gz_sim_scene::{BodyHandle, JointDefinition, JointHandle, PhysicsEngine};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_grasp_fleet.sdf");
const ARCHIVE: &str = "barrier.gzgraspfleet";
#[path = "support/tree_payload_fixture.rs"]
mod fixture;
fn recipe(xml: &str) -> Result<GraspFleetSpec> {
    Ok(GraspFleetSpec {
        assemblies: gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
            xml,
            "fleet.sdf",
            "default",
        )?,
    })
}
fn exe() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
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
fn two_authored_grippers_match_one_ordinary_world_with_shared_and_parallel_workers() -> Result<()> {
    let spec = recipe(XML)?;
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut packed = GraspFleetPool::create(
        exe(),
        a.path(),
        3,
        spec.clone(),
        GraspFleetPlacement::Packed,
    )?;
    let mut striped = GraspFleetPool::create(
        exe(),
        b.path(),
        3,
        spec.clone(),
        GraspFleetPlacement::RoundRobin,
    )?;
    assert_eq!(packed.owners(), [0, 0]);
    assert_eq!(striped.owners(), [0, 1]);
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        XML,
        "fleet.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    for s in &spec.assemblies {
        gz_sim_scene::set_model_fidelity(
            &mut sim,
            s.payload.tree.model_id,
            ArticulationTier::Dynamic,
        )?;
    }
    let w = sim.world();
    let body = |id| {
        w.get::<BodyHandle>(entity_by_id(w, id).unwrap())
            .unwrap()
            .id()
    };
    let mut bodies = Vec::new();
    let mut joints = Vec::new();
    for s in &spec.assemblies {
        bodies.push((
            s.payload
                .tree
                .links
                .iter()
                .map(|l| body(l.id))
                .collect::<Vec<_>>(),
            body(s.payload.payload.id),
        ));
        joints.push(
            s.payload
                .tree
                .joints
                .iter()
                .map(|j| {
                    let entity = entity_by_id(w, j.joint_id).unwrap();
                    (
                        j.joint_id,
                        w.get::<JointHandle>(entity).unwrap().id(),
                        if w.get::<JointDefinition>(entity).unwrap().0.name == "right_close" {
                            1.
                        } else {
                            -1.
                        },
                    )
                })
                .collect::<Vec<_>>(),
        );
    }
    let mut both = 0;
    let mut warm = [0; 2];
    let mut releases = [0; 2];
    let mut previous = [false; 2];
    let mut moves = 0;
    let mut graph = Sha256::new();
    let mut momentum = [glam::DVec3::ZERO; 2];
    let mut contact = [glam::DVec3::ZERO; 2];
    let mut max_residual = 0f64;
    for t in 0..1650 {
        let mut loads = Vec::new();
        let mut torques = Vec::new();
        for (i, s) in spec.assemblies.iter().enumerate() {
            let open = if i == 0 { 830 } else { 1120 };
            let close = if i == 0 { 1260 } else { 1350 };
            if t == 0 || t == open || t == close {
                for &(id, handle, sign) in &joints[i] {
                    let m = Some(JointVelocityMotor {
                        target_velocity: sign * if t == open { -6. } else { 6. },
                        max_force: 4.,
                    });
                    packed.set_velocity_motor(id, m)?;
                    striped.set_velocity_motor(id, m)?;
                    sim.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .set_joint_velocity_motor(handle, m)?;
                }
            }
            let z = if (600..1400).contains(&t) {
                if i == 0 { 1.2 } else { -0.82 }
            } else {
                0.
            };
            if z != 0. {
                loads.push((s.payload.payload.id, [0., 0., z]));
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(bodies[i].1, [0., 0., z], [0.; 3])?;
                momentum[i].z += z * 0.001;
            }
            if t % 67 == 0 {
                let id = joints[i][(t as usize / 67) % 2].0;
                let effort = if i == 0 { 0.001 } else { -0.002 };
                torques.push((id, effort));
                gz_sim_scene::queue_joint_effort(&mut sim, id, effort)?;
            }
        }
        if t == 510 {
            let id = joints[0][0].0;
            packed.queue_effort(id, -0.017)?;
            striped.queue_effort(id, -0.017)?;
            gz_sim_scene::queue_joint_effort(&mut sim, id, -0.017)?;
            packed.drop_next_acknowledgement(packed.owners()[0])?;
            striped.drop_next_acknowledgement(striped.owners()[1])?;
        }
        sim.step_exact(1)?;
        packed.advance_with_inputs_at(0, t, &torques, &loads)?;
        striped.advance_with_inputs_at(0, t, &torques, &loads)?;
        let backend = sim.world().resource::<PhysicsBackend>();
        let oracle = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
        let mut active = [false; 2];
        for (i, s) in spec.assemblies.iter().enumerate() {
            let model = s.payload.tree.model_id;
            assert_eq!(packed.image(model), striped.image(model));
            graph.update(packed.image(model).expect("advanced graph"));
            let tree = packed.observation(model).unwrap();
            let part = packed.payload_state(model).unwrap();
            assert_eq!(Some(tree), striped.observation(model));
            assert_eq!(Some(part), striped.payload_state(model));
            for (state, &handle) in tree
                .links
                .iter()
                .chain([part])
                .zip(bodies[i].0.iter().chain([&bodies[i].1]))
            {
                let expected = backend.state(handle).unwrap();
                assert_eq!(
                    state.position.map(f64::to_bits),
                    expected.position.map(f64::to_bits),
                    "world position model {model} body {} tick {}",
                    state.id,
                    t + 1
                );
                assert_eq!(
                    state.orientation.map(f64::to_bits),
                    expected.orientation.map(f64::to_bits)
                );
                assert_eq!(
                    state.velocity.map(f64::to_bits),
                    expected.velocity.map(f64::to_bits),
                    "world velocity model {model} body {} tick {}",
                    state.id,
                    t + 1
                );
                assert_eq!(
                    state.angular_velocity.map(f64::to_bits),
                    expected.angular_velocity.map(f64::to_bits)
                );
            }
            for &(id, h, _) in &joints[i] {
                let j = tree.joints.iter().find(|j| j.joint_id == id).unwrap();
                let expected = backend.joint_state(h).unwrap();
                assert_eq!(j.q.to_bits(), expected.position.to_bits());
                assert_eq!(j.qdot.to_bits(), expected.velocity.to_bits());
                assert_eq!(
                    packed.velocity_motor(id),
                    oracle.motor_revolute_velocity(h)?
                );
            }
            let mut loaded = 0;
            for (&link, r) in bodies[i]
                .0
                .iter()
                .zip(packed.payload_reactions(model).unwrap())
            {
                assert_eq!(
                    *r,
                    oracle.payload_tree_reaction(link, bodies[i].1)?,
                    "ordinary world contact model {model} tick {}",
                    t + 1
                );
                loaded += usize::from(
                    r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.,
                );
                contact[i] += glam::DVec3::from_array(r.normal) * r.normal_impulse
                    + glam::DVec3::from_array(r.tangent_impulse);
            }
            max_residual = max_residual.max(
                (glam::DVec3::from_array(part.velocity) * s.payload.payload.mass + contact[i]
                    - momentum[i])
                    .length(),
            );
            active[i] = loaded >= 2;
            warm[i] += usize::from(active[i]);
            releases[i] += usize::from(previous[i] && !active[i]);
            previous[i] = active[i];
        }
        if active.iter().all(|v| *v) {
            both += 1;
            if (moves == 0 || t % 61 == 0) && moves < 4 {
                let i = moves % 2;
                let model = spec.assemblies[i].payload.tree.model_id;
                let dest = (packed.owners()[i] + 1) % packed.width();
                let initial = packed.observation(model).unwrap().clone();
                kill(
                    packed
                        .worker_pid(if moves % 2 == 0 {
                            packed.owners()[i]
                        } else {
                            dest
                        })
                        .unwrap(),
                )?;
                packed.transfer(model, dest)?;
                drop(packed);
                packed = GraspFleetPool::resume(exe(), a.path())?;
                assert_eq!(packed.observation(model), Some(&initial));
                moves += 1;
            }
        }
    }
    eprintln!(
        "ordinary_world_frames=1650 matching_native_member_receipts={} warm_bilateral_ticks={warm:?} simultaneous_bilateral_ticks={both} releases={releases:?} actual_worker_deaths_loaded_transfers_reopens={moves} max_payload_impulse_closure_Ns={max_residual} sha256={:x}",
        spec.assemblies.len() * 1650,
        graph.finalize()
    );
    assert!(
        both > 150
            && warm.iter().all(|w| *w > 350)
            && releases.iter().all(|n| *n > 0)
            && moves == 4
            && max_residual < 1e-9
    );
    Ok(())
}
fn rebase(s: &mut PayloadTreeSpec, offset: u64, y: f64) {
    s.tree.model_id += offset;
    for l in &mut s.tree.links {
        l.id += offset;
        l.position[1] += y;
    }
    for j in &mut s.tree.joints {
        j.joint_id += offset;
        j.parent_id += offset;
        j.child_id += offset;
        j.anchor[1] += y;
    }
    for l in &mut s.limits {
        l.joint_id += offset;
    }
    s.payload.id += offset;
    s.payload.position[1] += y;
}
#[test]
fn eight_maximal_warm_native_graphs_share_one_worker_with_bounded_paged_receipts() -> Result<()> {
    let assemblies = (0..8)
        .map(|i| {
            let mut s = fixture::cage(8, false, 0.24, [0.26, 0.074, -0.021]);
            s.exact_revolute_locks = true;
            s.payload_can_sleep = false;
            rebase(&mut s, i * 2000, (i * 4) as f64);
            GraspTreeSpec { payload: s }
        })
        .collect();
    let spec = GraspFleetSpec { assemblies };
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut p = GraspFleetPool::create(
        exe(),
        a.path(),
        3,
        spec.clone(),
        GraspFleetPlacement::Packed,
    )?;
    let mut q = GraspFleetPool::create(
        exe(),
        b.path(),
        3,
        spec.clone(),
        GraspFleetPlacement::RoundRobin,
    )?;
    let mut size = 0;
    let mut warm = 0;
    for t in 0..16 {
        let inputs = spec
            .assemblies
            .iter()
            .flat_map(|g| g.payload.tree.joints.iter().map(|j| (j.joint_id, 0.001)))
            .collect::<Vec<_>>();
        let loads = spec
            .assemblies
            .iter()
            .map(|g| (g.payload.payload.id, [0.01, -0.003, 0.004]))
            .collect::<Vec<_>>();
        if t == 8 {
            kill(p.worker_pid(0).unwrap())?;
            p.drop_next_acknowledgement(0)?;
        }
        p.advance_with_inputs_at(0, t, &inputs, &loads)?;
        q.advance_with_inputs_at(0, t, &inputs, &loads)?;
        let mut bytes = 0;
        let mut loaded = 0;
        for s in &spec.assemblies {
            let m = s.payload.tree.model_id;
            assert_eq!(p.image(m), q.image(m));
            bytes += p.image(m).unwrap().len();
            loaded += p
                .payload_reactions(m)
                .unwrap()
                .iter()
                .filter(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                .count();
        }
        size = size.max(bytes);
        warm = warm.max(loaded);
    }
    let archive = fs::metadata(a.path().join(ARCHIVE))?.len();
    assert!(size > 192 * 1024 && archive > 192 * 1024 && warm > 40);
    eprintln!(
        "assemblies=8 tree_links=64 independent_payloads=8 cap_per_transport_frame=196608 bytes_all_native={size} bytes_archive={archive} simultaneous_warm_contacts={warm} packed_steps=16"
    );
    Ok(())
}

#[test]
fn impending_cross_assembly_contact_preserves_old_tick_and_every_pending_effort() -> Result<()> {
    let xml = XML
        .replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58");
    let spec = recipe(&xml)?;
    let a = tempfile::tempdir()?;
    let mut p = GraspFleetPool::create(
        exe(),
        a.path(),
        2,
        spec.clone(),
        GraspFleetPlacement::RoundRobin,
    )?;
    let ysign = |s: &GraspTreeSpec| -s.payload.payload.position[1].signum();
    let loads = spec
        .assemblies
        .iter()
        .map(|s| (s.payload.payload.id, [0., ysign(s) * 4., 0.]))
        .collect::<Vec<_>>();
    let mut witness = None;
    for _ in 0..800 {
        let tick = p.tick();
        let joint = spec.assemblies[0].payload.tree.joints[0].joint_id;
        p.queue_effort(joint, 0.0002)?;
        let prior = fs::read(a.path().join(ARCHIVE))?;
        let old = p
            .payload_state(spec.assemblies[0].payload.tree.model_id)
            .unwrap()
            .to_owned();
        let outcome = p.advance_with_inputs_at(0, tick, &[], &loads);
        if let Err(ArticulationError::PossibleCrossAssemblyContact {
            first_model,
            first_body,
            second_model,
            second_body,
            epoch,
            tick: input,
        }) = outcome
        {
            assert_eq!((epoch, input), (0, tick));
            assert_eq!(first_body, spec.assemblies[0].payload.payload.id);
            assert_eq!(second_body, spec.assemblies[1].payload.payload.id);
            assert_eq!(p.tick(), tick);
            assert_eq!(p.pending_joint_effort(joint), Some(0.0002));
            assert_eq!(p.payload_state(first_model), Some(&old));
            assert_eq!(prior, fs::read(a.path().join(ARCHIVE))?);
            drop(p);
            p = GraspFleetPool::resume(exe(), a.path())?;
            assert!(matches!(
                p.advance_with_inputs_at(0, tick, &[], &loads),
                Err(ArticulationError::PossibleCrossAssemblyContact { .. })
            ));
            if let Some(root) = std::env::var_os("GZ_GRASP_FLEET_WITNESS") {
                let d = Path::new(&root);
                fs::create_dir_all(d)?;
                fs::write(d.join(ARCHIVE), prior)?;
                fs::write(d.join("approach.sdf"), &xml)?;
                fs::write(
                    d.join("cut.json"),
                    serde_json::to_vec_pretty(
                        &serde_json::json!({"epoch":epoch,"tick":tick,"first_model":first_model,"second_model":second_model,"first_body":first_body,"second_body":second_body,"requested_forces":loads,"queued_joint":joint,"queued_torque_Nm":0.0002}),
                    )?,
                )?;
            }
            witness = Some(tick);
            break;
        }
        outcome?;
    }
    eprintln!(
        "cross_assembly_witness={witness:?} old_world_and_pending_effort_preserved=true rejected_again_after_reopen=true"
    );
    assert!(witness.is_some());
    Ok(())
}

#[test]
#[ignore = "matched native and storage timing on two through eight independently loaded grippers"]
fn benchmark_grasp_fleet_packed_and_round_robin() -> Result<()> {
    use std::{io::Write, time::Instant};
    let root = std::env::var_os("GZ_FLEET_BENCH_DIR").ok_or("supply GZ_FLEET_BENCH_DIR")?;
    let output = Path::new(&root);
    fs::create_dir_all(output)?;
    let mut table = std::io::BufWriter::new(fs::File::create(output.join("matched-latency.csv"))?);
    writeln!(
        table,
        "storage,groups,width,tick,packed_ns,round_robin_ns,simultaneous_bilateral_groups,native_bytes"
    )?;
    for n in [2, 8] {
        let recipe = recipe(XML)?;
        let mut assemblies = Vec::new();
        for i in 0..n {
            let mut s = recipe.assemblies[i % 2].payload.clone();
            rebase(&mut s, (i * 2000) as u64, 4. * i as f64);
            assemblies.push(GraspTreeSpec { payload: s });
        }
        let spec = GraspFleetSpec { assemblies };
        for (storage, path, samples) in [("tmpfs", "/dev/shm", 500), ("filesystem", "/var/tmp", 80)]
        {
            let seed = tempfile::tempdir_in("/dev/shm")?;
            let seed_shadow = tempfile::tempdir_in("/dev/shm")?;
            let width = if n == 2 { 2 } else { 4 };
            let mut p = GraspFleetPool::create(
                exe(),
                seed.path(),
                width,
                spec.clone(),
                GraspFleetPlacement::Packed,
            )?;
            let mut q = GraspFleetPool::create(
                exe(),
                seed_shadow.path(),
                width,
                spec.clone(),
                GraspFleetPlacement::RoundRobin,
            )?;
            for (i, s) in spec.assemblies.iter().enumerate() {
                // Initially leave odd groups untouched so contact work changes during the measured window.
                if i % 2 != 0 {
                    continue;
                }
                let palm = s.payload.tree.links[0].position[1];
                for j in &s.payload.tree.joints {
                    let finger = s
                        .payload
                        .tree
                        .links
                        .iter()
                        .find(|l| l.id == j.child_id)
                        .unwrap()
                        .position[1];
                    let m = Some(JointVelocityMotor {
                        target_velocity: -(finger - palm).signum() * 6.,
                        max_force: 4.,
                    });
                    p.set_velocity_motor(j.joint_id, m)?;
                    q.set_velocity_motor(j.joint_id, m)?;
                }
            }
            for _ in 0..650 {
                p.advance()?;
                q.advance()?;
            }
            drop(p);
            drop(q);
            let a = tempfile::tempdir_in(path)?;
            let b = tempfile::tempdir_in(path)?;
            fs::copy(seed.path().join(ARCHIVE), a.path().join(ARCHIVE))?;
            fs::copy(seed_shadow.path().join(ARCHIVE), b.path().join(ARCHIVE))?;
            let mut p = GraspFleetPool::resume(exe(), a.path())?;
            let mut q = GraspFleetPool::resume(exe(), b.path())?;
            let mut packed_times = Vec::new();
            let mut striped_times = Vec::new();
            let mut peak = 0;
            let mut native = 0;
            let mut sha = Sha256::new();
            for nstep in 0..samples {
                if nstep == samples / 4 {
                    for (i, s) in spec.assemblies.iter().enumerate() {
                        for j in &s.payload.tree.joints {
                            let palm = s.payload.tree.links[0].position[1];
                            let finger = s
                                .payload
                                .tree
                                .links
                                .iter()
                                .find(|l| l.id == j.child_id)
                                .unwrap()
                                .position[1];
                            let m = Some(JointVelocityMotor {
                                target_velocity: (finger - palm).signum()
                                    * if i % 2 == 0 { 6. } else { -6. },
                                max_force: 4.,
                            });
                            p.set_velocity_motor(j.joint_id, m)?;
                            q.set_velocity_motor(j.joint_id, m)?;
                        }
                    }
                }
                let loads = spec
                    .assemblies
                    .iter()
                    .enumerate()
                    .map(|(i, s)| {
                        (
                            s.payload.payload.id,
                            [0., 0., if i % 2 == 0 { 1.2 } else { -0.82 }],
                        )
                    })
                    .collect::<Vec<_>>();
                let timed = |q: &mut GraspFleetPool| -> Result<u128> {
                    let start = Instant::now();
                    q.advance_with_inputs(&[], &loads)?;
                    Ok(start.elapsed().as_nanos())
                };
                let (pn, qn) = if nstep % 2 == 0 {
                    let pns = timed(&mut p)?;
                    (pns, timed(&mut q)?)
                } else {
                    let qns = timed(&mut q)?;
                    (timed(&mut p)?, qns)
                };
                let mut active = 0;
                let mut nbytes = 0;
                for s in &spec.assemblies {
                    let model = s.payload.tree.model_id;
                    assert_eq!(p.image(model), q.image(model));
                    assert_eq!(p.observation(model), q.observation(model));
                    assert_eq!(p.payload_reactions(model), q.payload_reactions(model));
                    let bytes = p.image(model).unwrap();
                    sha.update(bytes);
                    nbytes += bytes.len();
                    active += usize::from(
                        p.payload_reactions(model)
                            .unwrap()
                            .iter()
                            .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                            .count()
                            >= 2,
                    );
                }
                peak = peak.max(active);
                native = native.max(nbytes);
                packed_times.push(pn);
                striped_times.push(qn);
                writeln!(
                    table,
                    "{storage},{n},{width},{},{pn},{qn},{active},{nbytes}",
                    p.tick()
                )?;
            }
            let quantiles = |mut v: Vec<u128>| {
                v.sort_unstable();
                [50, 95, 99].map(|k| v[(k * v.len()).div_ceil(100) - 1] as f64 / 1000.)
            };
            eprintln!(
                "storage={storage} groups={n} occupied_round_robin_workers={width} samples={samples} packed_p50_p95_p99_us={:?} round_robin_p50_p95_p99_us={:?} matched_peak_warm_groups={peak} peak_native_bytes={native} trajectory_sha256={:x}",
                quantiles(packed_times),
                quantiles(striped_times),
                sha.finalize()
            );
            table.flush()?;
        }
    }
    Ok(())
}
