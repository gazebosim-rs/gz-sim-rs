// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Runtime Gazebo model operations against both real processes and a monolithic oracle.
use gz_physics::{BodySpec, DynamicBackend, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, STEP, Tier},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, ModelState, Name, PhysicsEngine, WorldLoadOptions, WorldPose,
    load_world_configured, model_state, prepare_model_template, remove_model, reset_loaded_world,
    set_model_state, spawn_model_template,
};
use sdformat::NoResources;
use std::path::Path;

const XML: &str = include_str!("../../../examples/distributed_stack.sdf");
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn spawn_native(native: &mut MultiFidelityBackend, b: &Body) {
    let id = native
        .spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: b.size.map(|v| v * 0.5),
                },
                position: b.position,
                velocity: b.linear,
                mass: (b.tier != Tier::Fixed).then_some(b.mass),
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            b.tier == Tier::Prescribed,
        )
        .unwrap();
    assert_eq!(id.raw(), b.id);
}
fn check(sim: &Simulation, native: &MultiFidelityBackend) {
    let frame = backend(sim).frame().unwrap();
    for b in frame.workers.iter().flat_map(|w| &w.snapshot.bodies) {
        let n = native.state(gz_physics::BodyId::from_raw(b.id)).unwrap();
        assert_eq!(
            b.position.map(f64::to_bits),
            n.position.map(f64::to_bits),
            "position {} at {}",
            b.id,
            frame.tick
        );
        assert_eq!(b.linear.map(f64::to_bits), n.velocity.map(f64::to_bits));
        assert_eq!(
            b.orientation.map(f64::to_bits),
            n.orientation.map(f64::to_bits)
        );
        assert_eq!(
            b.angular.map(f64::to_bits),
            n.angular_velocity.map(f64::to_bits)
        );
    }
    let mut distributed: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .map(|p| (p.a, p.b, p.normal_impulse.to_bits()))
        .collect();
    distributed.sort_unstable();
    let native: Vec<_> = native
        .rigid_contact_observations()
        .into_iter()
        .filter(|p| p.active)
        .map(|p| (p.a.raw(), p.b.raw(), p.normal_impulse.to_bits()))
        .collect();
    assert_eq!(distributed, native, "contacts at {}", frame.tick);
}
#[test]
fn loaded_scene_runtime_clone_reposition_retire_and_reset_match_native() {
    let mut sim = load_world_configured(
        XML,
        "stack.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )
    .unwrap();
    let parsed = sdformat::load_str(XML, "stack.sdf", &NoResources).unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[1]).unwrap();
    let base = backend(&sim).frame().unwrap().clone();
    let mut native = MultiFidelityBackend::new([0., 0., -9.81]).unwrap();
    let mut original: Vec<_> = base
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .collect();
    original.sort_by_key(|b| b.id);
    for b in original {
        spawn_native(&mut native, b);
    }
    let first_model = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::first"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let mut runtime = None;
    let mut hits = Vec::new();
    for tick in 1..=265 {
        if tick == 159 {
            let previous = backend(&sim).frame().unwrap().clone();
            let next = next_entity_id(sim.world()).unwrap();
            let pose = WorldPose {
                translation: previous.body(1).unwrap().1.position,
                rotation: [0., 0., 0., 1.],
            };
            assert!(spawn_model_template(&mut sim, &template, pose).is_err());
            assert_eq!(next_entity_id(sim.world()).unwrap(), next);
            assert_eq!(backend(&sim).frame().unwrap(), &previous);
            let id = spawn_model_template(
                &mut sim,
                &template,
                WorldPose {
                    translation: [0., 1.25, 2.5],
                    ..pose
                },
            )
            .unwrap();
            assert_eq!(id, next);
            let handle = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|n| n.0 == format!("default::runtime_{id}::body"))
                })
                .unwrap()
                .get::<BodyHandle>()
                .unwrap()
                .id();
            assert_eq!(handle.raw(), 3);
            spawn_native(
                &mut native,
                backend(&sim).frame().unwrap().body(3).unwrap().1,
            );
            runtime = Some((id, handle));
            check(&sim, &native);
            let state = model_state(&sim, id).unwrap();
            let overlap = ModelState {
                translation: model_state(&sim, first_model).unwrap().translation,
                ..state
            };
            assert!(set_model_state(&mut sim, id, overlap).is_err());
            assert_eq!(model_state(&sim, id).unwrap(), state);
            let changed = ModelState {
                translation: [0., 1.22, 1.75],
                linear_velocity: [0., -0.1, -0.6],
                angular_velocity: [0.5, 0., 0.],
                ..state
            };
            set_model_state(&mut sim, id, changed).unwrap();
            native
                .set_body_state(
                    handle,
                    gz_physics::BodyState {
                        position: changed.translation,
                        orientation: changed.rotation,
                        velocity: changed.linear_velocity,
                        angular_velocity: changed.angular_velocity,
                    },
                )
                .unwrap();
            check(&sim, &native);
        }
        if tick == 231 {
            let (id, body) = runtime.unwrap();
            let state = model_state(&sim, id).unwrap();
            assert!(
                backend(&sim).frame().unwrap().workers.iter().any(|w| w
                    .snapshot
                    .impulses
                    .iter()
                    .any(|p| p.a == 3 || p.b == 3))
            );
            let altered = ModelState {
                translation: [0., -1.2, 2.],
                linear_velocity: [0.; 3],
                angular_velocity: [0.; 3],
                ..state
            };
            set_model_state(&mut sim, id, altered).unwrap();
            native
                .set_body_state(
                    body,
                    gz_physics::BodyState {
                        position: altered.translation,
                        orientation: altered.rotation,
                        velocity: [0.; 3],
                        angular_velocity: [0.; 3],
                    },
                )
                .unwrap();
            check(&sim, &native);
        }
        if tick == 249 {
            let (id, body) = runtime.unwrap();
            assert_eq!(remove_model(&mut sim, id).unwrap().len(), 4);
            assert!(native.remove(body));
            assert!(entity_by_id(sim.world(), id).is_none());
            check(&sim, &native);
        }
        sim.step_exact(1).unwrap();
        native.step(STEP).unwrap();
        check(&sim, &native);
        if backend(&sim).frame().unwrap().workers.iter().any(|w| {
            w.snapshot
                .impulses
                .iter()
                .any(|p| p.b == 3 && p.normal_impulse > 0.)
        }) {
            hits.push(tick);
        }
    }
    assert!(!hits.is_empty());
    let pose = WorldPose {
        translation: [0., 1., 3.],
        rotation: [0., 0., 0., 1.],
    };
    let extra = spawn_model_template(&mut sim, &template, pose).unwrap();
    assert!(backend(&sim).frame().unwrap().body(4).is_some());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(backend(&sim).frame().unwrap(), &base);
    assert!(entity_by_id(sim.world(), extra).is_none());
    let after = spawn_model_template(&mut sim, &template, pose).unwrap();
    assert!(after > extra);
    assert!(backend(&sim).frame().unwrap().body(5).is_some());
    println!(
        "265 native-equivalent advancing barriers, clone with stable ID 3 supported on ticks {:?}–{:?}; warm edit, removal and loaded reset passed",
        hits.first(),
        hits.last()
    );
}
