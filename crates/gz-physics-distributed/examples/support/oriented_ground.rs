// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same SDF construction and native friction constraints before and after a live transfer.
use gz_physics::{BodySpec, DynamicBackend, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{
    SceneBackend,
    coordinator::{Coordinator, Frame, NativeRecoveryPlan, RecoveryPlan},
    physics::SimulationStep,
    transport::WorkerProcess,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world,
};
use std::{error::Error, path::Path};

const XML: &str = include_str!("../../../../examples/distributed_oriented_plane.sdf");
fn scene_frame(sim: &Simulation) -> &Frame {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .frame()
        .unwrap()
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}
fn support(f: &Frame, id: u64) -> Option<f64> {
    f.workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .find(|s| s.body == id)
        .map(|s| s.normal_impulse)
}
pub fn run(worker: &Path) -> Result<(), Box<dyn Error>> {
    let mut sim = load_world_configured(
        XML,
        "oriented_ground.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let seed = scene_frame(&sim).clone();
    let ground = seed.ground().unwrap();
    let dt = SimulationStep::Ms5;
    let h = dt.duration();
    let gravity = [0., 0., -9.81];
    let (s, c) = 0.25_f64.sin_cos();
    let (sy, cy) = 0.38_f64.sin_cos();
    let n = [s * cy, s * sy, c];
    let u = [c * cy, c * sy, -s];
    let anchor = [0.25 * cy, 0.25 * sy, 0.72];
    for i in 0..3 {
        assert!((ground.point()[i] - anchor[i]).abs() < 1e-14);
        assert!((ground.normal()[i] - n[i]).abs() < 1e-14);
    }
    let terrain = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::slope::terrain")
        })
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id();
    let backend = sim.world().resource::<PhysicsBackend>();
    assert_eq!(backend.state(terrain).unwrap().position, [0., 0., 0.6]);
    // The ray is 20 m from the reference point, outside the finite visual.
    let hit: [f64; 3] = std::array::from_fn(|i| anchor[i] + 20. * u[i]);
    let origin = std::array::from_fn(|i| hit[i] + 1.73 * n[i]);
    assert!((backend.cast_ray(origin, n.map(|v| -v), 4., None)?.unwrap() - 1.73).abs() < 1e-12);
    assert_eq!(
        backend.cast_ray(origin, n.map(|v| -v), 4., Some(terrain))?,
        None
    );
    assert_eq!(
        backend.cast_ray(std::array::from_fn(|i| hit[i] - 0.2 * n[i]), n, 3., None)?,
        Some(0.)
    );
    let mut native = MultiFidelityBackend::new(gravity)?;
    native.set_partition_step(h)?;
    native.set_partition_plane(ground.point(), ground.normal(), ground.friction())?;
    let bodies: Vec<_> = (0..2).map(|id| seed.body(id).unwrap().1.clone()).collect();
    let mut handles = Vec::new();
    for b in &bodies {
        handles.push(native.spawn_partition_box(
            BodySpec {
                position: b.position,
                velocity: b.linear,
                mass: Some(b.mass),
                shape: Shape::Box {
                    half_extents: b.size.map(|x| x / 2.),
                },
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            false,
            b.friction,
        )?);
    }
    let run = u64::from(std::process::id());
    let peers = [
        WorkerProcess::spawn_with_ground(worker, run, 0, &bodies[..1], dt, Some(ground))?,
        WorkerProcess::spawn_with_ground(worker, run, 1, &bodies[1..], dt, Some(ground))?,
    ];
    let mut migrating = Coordinator::new_recorded(run, peers, 1400)?;
    let dir = tempfile::tempdir()?;
    let (mut seed_speed, mut anchor_pos, mut drift) = (0., 0., 0.0f64);
    let mut impulses = [0.; 2];
    let mut first = [None; 2];
    for tick in 1..=500 {
        if tick == 151 {
            let old = migrating.frame().clone();
            assert!(support(&old, 0).is_some_and(|j| j > 0.));
            migrating.transfer_component(0)?;
            assert_ne!(old.body(0).unwrap().0, migrating.frame().body(0).unwrap().0);
            assert_eq!(support(&old, 0), support(migrating.frame(), 0));
        }
        if tick == 201 {
            let before = migrating.frame().clone();
            let journal = dir.path().join("actions");
            let image = dir.path().join("native");
            migrating.save_recovery(&journal)?;
            migrating.save_native_recovery(&image)?;
            for mut w in migrating.into_peers() {
                w.shutdown()?;
            }
            let plan = RecoveryPlan::open(&journal)?;
            assert_eq!(plan.ground(), Some(ground));
            let replay = plan.resume_with_ground(|run, id, bodies, step, g| {
                WorkerProcess::spawn_with_ground(worker, run, id, bodies, step, g)
            })?;
            assert_eq!(replay.frame(), &before);
            for mut w in replay.into_peers() {
                w.shutdown()?;
            }
            migrating = NativeRecoveryPlan::open(image)?.resume(worker)?;
            assert_eq!(migrating.frame(), &before);
        }
        let remote = migrating.advance_in_field_overlapped(gravity, &[])?;
        sim.step_exact(1)?;
        native.step(h)?;
        let scene = scene_frame(&sim);
        let contacts = native.partition_ground_contacts();
        for id in 0..2 {
            let body = scene.body(id as u64).unwrap().1;
            assert_eq!(
                body,
                remote.body(id as u64).unwrap().1,
                "transfer/recovery at {tick}"
            );
            let actual = native.state(handles[id]).unwrap();
            assert_eq!(
                body.position.map(f64::to_bits),
                actual.position.map(f64::to_bits)
            );
            assert_eq!(
                body.orientation.map(f64::to_bits),
                actual.orientation.map(f64::to_bits)
            );
            assert_eq!(
                body.angular.map(f64::to_bits),
                actual.angular_velocity.map(f64::to_bits)
            );
            assert_eq!(
                body.linear.map(f64::to_bits),
                actual.velocity.map(f64::to_bits)
            );
            let native_j = contacts
                .iter()
                .find(|p| p.active && p.body == handles[id])
                .map(|p| p.normal_impulse);
            let j = support(scene, id as u64);
            assert_eq!(j.map(f64::to_bits), native_j.map(f64::to_bits));
            assert_eq!(
                j.map(f64::to_bits),
                support(remote, id as u64).map(f64::to_bits)
            );
            if let Some(value) = j.filter(|j| *j > 0.) {
                first[id].get_or_insert(tick);
                if tick > 100 {
                    impulses[id] += value;
                }
            }
        }
        if tick == 100 {
            seed_speed = dot(scene.body(0).unwrap().1.linear, u);
            anchor_pos = dot(scene.body(1).unwrap().1.position, u);
        }
        if tick > 100 {
            drift = drift.max((dot(scene.body(1).unwrap().1.position, u) - anchor_pos).abs());
        }
    }
    let a = (dot(scene_frame(&sim).body(0).unwrap().1.linear, u) - seed_speed) / 2.;
    let expected = 9.81 * (s - 0.12 * c);
    let loads = impulses.map(|x| x / 2.);
    println!(
        "500 steps; first support {first:?}, acceleration {a:.10} vs {expected:.10} m/s²; holding drift {drift:.3e} m; loads {loads:?} vs {:.10} N; ray 1.73 m, 20 m beyond displayed center; all link and normal impulse bits matched after loaded transfer, actions replay and native restart.",
        2. * 9.81 * c
    );
    assert!(first.into_iter().all(|t| t.is_some_and(|v| v < 100)));
    assert!((a - expected).abs() < 0.02 && drift < 0.002);
    assert!(loads.into_iter().all(|f| (f - 2. * 9.81 * c).abs() < 0.02));
    for mut w in migrating.into_peers() {
        w.shutdown()?;
    }
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(scene_frame(&sim), &seed);
    Ok(())
}
