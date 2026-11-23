// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec};
use gz_physics_distributed::{SceneBackend, physics::Tier};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{collections::BTreeSet, error::Error, path::Path, time::Duration};
fn backend(s: &gz_sim_core::Simulation) -> &SceneBackend {
    s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
}

#[test]
fn ordinary_four_worker_scene_routes_all_lanes_and_survives_a_dead_peer()
-> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(5);
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_four_lanes.sdf"),
        "four.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedPool(4),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )?;
    let initial = backend(&sim).pool_frame().unwrap().clone();
    let mut native = MultiFidelityBackend::new([0.; 3])?;
    native.set_partition_step(dt)?;
    native.set_partition_ground_material(0., 1.)?;
    let pids = backend(&sim).local_worker_pids().unwrap();
    assert_eq!(pids.iter().collect::<BTreeSet<_>>().len(), 4);
    assert!(initial.workers.iter().all(|w| w.snapshot.bodies.len() == 2));
    for raw in 0..8 {
        let b = initial.body(raw).unwrap().1;
        let id = native.spawn_partition_primitive(
            BodySpec {
                shape: b.shape()?,
                position: b.position,
                velocity: b.linear,
                mass: Some(b.mass),
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            b.tier == Tier::Prescribed,
            b.friction,
        )?;
        assert_eq!(id.raw(), raw);
    }
    let mut impact = [None; 4];
    let mut moved = BTreeSet::new();
    for tick in 1..=1100 {
        #[cfg(unix)]
        if tick == 429 {
            let pid = backend(&sim).local_worker_pids().unwrap()[2];
            assert!(
                std::process::Command::new("kill")
                    .args(["-KILL", &pid.to_string()])
                    .status()?
                    .success()
            );
        }
        sim.step_exact(1)?;
        native.step(dt)?;
        let engine = backend(&sim);
        let f = engine.pool_frame().unwrap();
        assert_eq!(f.tick, tick);
        for raw in 0..8 {
            let (owner, b) = f.body(raw).unwrap();
            let id = BodyId::from_raw(raw);
            let n = native.state(id).unwrap();
            assert_eq!(
                (b.position, b.orientation, b.linear, b.angular),
                (n.position, n.orientation, n.velocity, n.angular_velocity),
                "body {raw}, tick {tick}"
            );
            if owner != initial.body(raw).unwrap().0 {
                moved.insert(raw);
            }
            assert_eq!(engine.state(id).unwrap(), n);
        }
        let mut contacts = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| (p.a, p.b, p.normal_impulse.to_bits()))
            .collect::<Vec<_>>();
        contacts.sort();
        let mut control = native
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| (p.a.raw(), p.b.raw(), p.normal_impulse.to_bits()))
            .collect::<Vec<_>>();
        control.sort();
        assert_eq!(contacts, control, "impulses at {tick}");
        for (a, b, j) in &contacts {
            if f64::from_bits(*j) > 0. {
                assert_eq!(a / 2, b / 2);
                impact[(a / 2) as usize].get_or_insert(tick);
            }
        }
        let body = f.body(4).unwrap().1;
        let origin = [
            body.position[0],
            body.position[1] - 2.,
            body.position[2] + 0.1,
        ];
        assert_eq!(
            engine.cast_ray(origin, [0., 1., 0.], 8., None)?,
            native.cast_ray(origin, [0., 1., 0.], 8., None)?
        );
    }
    println!(
        "four real SDF workers: 1100 exact barriers and rays, first impulses per lane {impact:?}, migrated IDs {moved:?}, live process replacement before tick 429"
    );
    assert!(impact.iter().all(Option::is_some));
    assert!(moved.len() >= 4);
    #[cfg(unix)]
    assert_ne!(pids[2], backend(&sim).local_worker_pids().unwrap()[2]);
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(backend(&sim).pool_frame(), Some(&initial));
    Ok(())
}

#[test]
fn seed_and_live_pool_inputs_are_deferred_once_and_reset_discards_the_queue()
-> Result<(), Box<dyn Error>> {
    use gz_physics::{BodyState, PlaneSpec, Shape};
    use gz_physics_distributed::physics::{GroundPlane, Tier};
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for count in [3, 5, 8] {
        let dt = Duration::from_millis(2);
        let mut engine = SceneBackend::new(worker, [0.; 3], dt)?.with_worker_count(count)?;
        let b = engine.spawn_primitive_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.; 3],
                mass: Some(2.),
                velocity: [0.; 3],
            },
            RotationSpec::default(),
            None,
            Tier::Dynamic,
            0.2,
        )?;
        let seed = engine.initial_state();
        engine.queue_linear_impulse(b, [0.5, -0., 0.])?;
        engine.initialize()?;
        assert_eq!(engine.pool_frame().unwrap().workers.len(), count as usize);
        assert_eq!(engine.state(b).unwrap().velocity, [0.; 3]);
        let spec = BodySpec {
            shape: Shape::Sphere { radius: 0.5 },
            position: [20.; 3],
            mass: Some(1.),
            velocity: [0.; 3],
        };
        let pose = BodyState {
            position: [4.; 3],
            ..engine.state(b).unwrap()
        };
        let plane = PlaneSpec {
            position: [0., 0., -1.],
            orientation: [0., 0., 0., 1.],
            size: [2., 2.],
        };
        let added = engine.spawn_link(spec, RotationSpec::default(), None)?;
        assert_ne!(added, b);
        assert!(engine.remove(added));
        assert!(!engine.remove(added));
        assert!(
            engine
                .spawn_ground_geometry(plane, GroundPlane::new(-1.)?)
                .is_err()
        );
        engine.set_body_state(b, pose)?;
        assert_eq!(engine.state(b).unwrap().position, pose.position);
        engine.set_world_velocity(b, [0.; 3], [0.5; 3])?;
        assert_eq!(engine.state(b).unwrap().angular_velocity, [0.5; 3]);
        engine.set_world_velocity(b, [0.; 3], [0.; 3])?;
        assert!(engine.add_world_wrench(b, [f64::NAN; 3], [0.; 3]).is_err());
        engine.set_packed_observations(false);
        engine.queue_linear_impulse(b, [0., 1., 0.])?;
        assert_eq!(engine.state(b).unwrap().velocity, [0.; 3]);
        engine.step(dt)?;
        assert_eq!(engine.state(b).unwrap().velocity, [0.25, 0.5, 0.]);
        engine.set_packed_observations(true);
        engine.step(dt)?;
        assert_eq!(engine.state(b).unwrap().velocity, [0.25, 0.5, 0.]);
        assert!(
            engine
                .local_worker_traffic()
                .unwrap()
                .iter()
                .all(|t| t.replies >= 3)
        );
        engine.add_world_wrench(b, [10., 0., 0.], [0., 0., 0.32])?;
        engine.queue_linear_impulse(b, [1., 0., 0.])?;
        engine.step(Duration::ZERO)?;
        assert_eq!(engine.state(b).unwrap().velocity, [0.25, 0.5, 0.]);
        engine.step(dt)?;
        let affected = engine.state(b).unwrap();
        assert!((affected.velocity[0] - 0.76).abs() < 1e-12);
        assert!((affected.angular_velocity[2] - 0.02).abs() < 1e-12);
        engine.step(dt)?;
        assert_eq!(engine.state(b).unwrap().velocity, affected.velocity);
        engine.add_world_wrench(b, [0., 0., 1.], [1., 0., 0.])?;
        engine.queue_linear_impulse(b, [1., 0., 0.])?;
        let restore = engine.prepare_initial(&seed)?;
        engine.apply_initial(restore)?;
        engine.step(dt)?;
        assert_eq!(engine.state(b).unwrap().velocity, [0.; 3]);
    }
    Ok(())
}

#[test]
fn ordinary_four_worker_sdf_exposes_immediate_rotated_inertia_impulses_and_reset()
-> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(5);
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_four_lanes.sdf"),
        "four.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedPool(4),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )?;
    let initial = backend(&sim).pool_frame().unwrap().clone();
    let mut native = MultiFidelityBackend::new([0.; 3])?;
    native.set_partition_step(dt)?;
    native.set_partition_ground_material(0., 1.)?;
    let mut rotating = None;
    for raw in 0..8 {
        let b = initial.body(raw).unwrap().1;
        let h = native.spawn_partition_primitive(
            BodySpec {
                shape: b.shape()?,
                position: b.position,
                mass: Some(b.mass),
                velocity: b.linear,
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            b.tier == Tier::Prescribed,
            b.friction,
        )?;
        assert_eq!(h.raw(), raw);
        if b.tier == Tier::Dynamic
            && b.primitive != gz_physics_distributed::physics::Primitive::Sphere
        {
            rotating = Some(h);
        }
    }
    let h = rotating.expect("anisotropic dynamic receiver");
    for tick in 0..60 {
        if tick % 7 == 0 {
            let (j, l) = ([0.019, -0.027, 0.011], [0.057, 0.008, -0.035]);
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .apply_impulse(h, j, l)?;
            native.apply_impulse(h, j, l)?;
            assert_eq!(backend(&sim).state(h), native.state(h), "edit tick {tick}");
            assert_eq!(backend(&sim).pool_frame().unwrap().tick, tick);
        }
        native.step(dt)?;
        sim.step_exact(1)?;
        assert_eq!(
            backend(&sim).state(h),
            native.state(h),
            "advance tick {tick}"
        );
    }
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(backend(&sim).pool_frame().unwrap(), &initial);
    Ok(())
}

#[test]
fn unsupported_joint_and_collision_group_edits_leave_independent_shard_authority_untouched()
-> Result<(), Box<dyn Error>> {
    use gz_physics::{JointId, PhysicsError, Shape};
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for width in [2, 4] {
        let mut scene =
            SceneBackend::new(exe, [0.; 3], Duration::from_millis(2))?.with_worker_count(width)?;
        assert!(scene.local_worker_pids().is_none());
        assert!(scene.local_worker_traffic().is_none());
        let mut ids = Vec::new();
        for x in [0., 8.] {
            ids.push(scene.spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [x, 0., 0.],
                    mass: Some(1.),
                    velocity: [0.25, 0., 0.],
                },
                RotationSpec::default(),
                None,
            )?);
        }
        scene.initialize()?;
        assert_eq!(scene.local_worker_pids().unwrap().len(), width as usize);
        let requests = || {
            scene
                .local_worker_traffic()
                .unwrap()
                .into_iter()
                .map(|p| p.requests)
                .collect::<Vec<_>>()
        };
        let previous = requests();
        let before = ids
            .iter()
            .map(|id| scene.state(*id).unwrap())
            .collect::<Vec<_>>();
        let axis = [0., 0., 1.];
        let anchor = [4., 0., 0.];
        for result in [
            scene.add_world_revolute(ids[0], anchor, axis),
            scene.add_revolute(ids[0], ids[1], anchor, axis),
            scene.add_reduced_revolute(Some(ids[0]), ids[1], anchor, axis),
        ] {
            assert!(matches!(
                result,
                Err(PhysicsError::Unsupported {
                    capability: "distributed joints",
                    ..
                })
            ));
        }
        assert!(scene.set_self_collision_group(ids[0], Some(33)).is_err());
        assert!(matches!(
            scene.set_self_collision_group(BodyId::from_raw(44), None),
            Err(PhysicsError::MissingBody)
        ));
        scene.set_self_collision_group(ids[0], None)?;
        let missing = JointId::from_raw(0);
        assert!(scene.add_joint_effort(missing, 1.).is_err());
        assert!(scene.joint_child(missing).is_none());
        assert!(scene.joint_state(missing).is_none());
        assert!(!scene.remove_joint(missing));
        assert_eq!(
            scene
                .local_worker_traffic()
                .unwrap()
                .into_iter()
                .map(|p| p.requests)
                .collect::<Vec<_>>(),
            previous
        );
        for (id, old) in ids.iter().zip(&before) {
            assert_eq!(&scene.state(*id).unwrap(), old);
        }
        scene.step(Duration::from_millis(2))?;
        for (id, old) in ids.iter().zip(before) {
            assert!(
                (scene.state(*id).unwrap().position[0] - old.position[0] - 0.0005).abs() < 1e-12
            );
        }
    }
    Ok(())
}

#[test]
fn four_worker_sdf_model_state_edit_updates_native_link_and_visual_then_restores_the_loaded_seed()
-> Result<(), Box<dyn Error>> {
    use gz_physics::BodyState;
    use gz_sim_core::{EntityId, entity_by_id};
    use gz_sim_scene::{BodyHandle, ModelState, Name, WorldPose, model_state, set_model_state};
    let dt = Duration::from_millis(5);
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_four_lanes.sdf"),
        "four.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedPool(4),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )?;
    let named = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
    };
    let model = named("default::cargo_0").get::<EntityId>().unwrap().get();
    let link = named("default::cargo_0::body")
        .get::<EntityId>()
        .unwrap()
        .get();
    let handle = named("default::cargo_0::body")
        .get::<BodyHandle>()
        .unwrap()
        .id();
    let visual = named("default::cargo_0::body::paint")
        .get::<EntityId>()
        .unwrap()
        .get();
    let initial = backend(&sim).pool_frame().unwrap().clone();
    let (owner, body) = initial.body(handle.raw()).unwrap();
    let foreign = initial
        .workers
        .iter()
        .enumerate()
        .find(|(w, _)| *w != usize::from(owner))
        .unwrap()
        .1
        .snapshot
        .bodies[0]
        .position;
    let old = model_state(&sim, model)?;
    let parsed = sdformat::load_str(
        include_str!("../../../examples/distributed_four_lanes.sdf"),
        "four.sdf",
        &NoResources,
    )?;
    let cargo = parsed.worlds[0]
        .models
        .iter()
        .find(|m| m.name == "cargo_0")
        .unwrap();
    gz_sim_scene::prepare_model_template(&sim, &parsed, cargo)?;
    let rejected = ModelState {
        translation: foreign,
        ..old
    };
    assert!(set_model_state(&mut sim, model, rejected).is_err());
    assert_eq!(model_state(&sim, model)?, old);
    assert_eq!(backend(&sim).pool_frame(), Some(&initial));
    let native_pose = |s: ModelState| BodyState {
        position: s.translation,
        orientation: s.rotation,
        velocity: s.linear_velocity,
        angular_velocity: s.angular_velocity,
    };
    let mut native = MultiFidelityBackend::new([0.; 3])?;
    native.set_partition_step(dt)?;
    let native_id = native.spawn_partition_primitive(
        BodySpec {
            shape: body.shape()?,
            position: body.position,
            velocity: body.linear,
            mass: Some(body.mass),
        },
        RotationSpec {
            orientation: body.orientation,
            angular_velocity: body.angular,
        },
        false,
        body.friction,
    )?;
    let tilt = 0.16_f64;
    let edited = ModelState {
        translation: [1., 8., 3.25],
        rotation: [tilt.sin(), 0., 0., tilt.cos()],
        linear_velocity: [0.17, -0.21, 0.24],
        angular_velocity: [-0.3, 0.24, 0.18],
    };
    let range = |sim: &gz_sim_core::Simulation, x: f64, y: f64| {
        backend(sim)
            .cast_ray([x, y, 10.], [0., 0., -1.], 12., None)
            .unwrap()
            .unwrap()
    };
    assert_eq!(
        range(&sim, edited.translation[0], edited.translation[1]),
        10.
    );
    let old_range = range(&sim, old.translation[0], old.translation[1]);
    assert!(old_range < 10.);
    set_model_state(&mut sim, model, edited)?;
    native.set_body_state(native_id, native_pose(edited))?;
    let top = (body.size[2] * 0.5 / (2. * tilt).cos()).min(body.size[1] * 0.5 / (2. * tilt).sin());
    assert!(
        (range(&sim, edited.translation[0], edited.translation[1])
            - (10. - edited.translation[2] - top))
            .abs()
            < 1e-12
    );
    assert_eq!(range(&sim, old.translation[0], old.translation[1]), 10.);
    for tick in 0..=60 {
        if tick > 0 {
            sim.step_exact(1)?;
            native.step(dt)?;
        }
        let actual = model_state(&sim, model)?;
        let expected = native.state(native_id).unwrap();
        assert_eq!(native_pose(actual), expected);
        let w = sim.world();
        for logical in [model, link, visual] {
            let e = entity_by_id(w, logical).unwrap();
            let pose = w.get::<WorldPose>(e).unwrap();
            assert_eq!(pose.translation, actual.translation);
            assert!(
                pose.rotation
                    .into_iter()
                    .zip(actual.rotation)
                    .all(|(a, b)| (a - b).abs() < 2e-15)
            );
        }
        let f = backend(&sim).pool_frame().unwrap();
        assert_eq!(f.tick, tick);
        assert_eq!(f.body(handle.raw()).unwrap().0, owner);
    }
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(model_state(&sim, model)?, old);
    assert_eq!(backend(&sim).pool_frame(), Some(&initial));
    assert_eq!(
        range(&sim, edited.translation[0], edited.translation[1]),
        10.
    );
    assert_eq!(
        range(&sim, old.translation[0], old.translation[1]),
        old_range
    );
    Ok(())
}

#[test]
fn immutable_shared_plane_rejects_normal_removal_while_free_pair_members_can_retire()
-> Result<(), Box<dyn Error>> {
    use gz_sim_core::EntityId;
    use gz_sim_scene::{Name, RemoveError, remove_model};
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_four_lanes.sdf"),
        "four.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )?;
    let lookup = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let floor = lookup("default::floor");
    let cargo = lookup("default::cargo_0");
    let before = backend(&sim).frame().unwrap().clone();
    assert_eq!(
        remove_model(&mut sim, floor),
        Err(RemoveError::ImmutableModel)
    );
    assert_eq!(backend(&sim).frame(), Some(&before));
    let retired = remove_model(&mut sim, cargo)?;
    assert!(retired.contains(&cargo));
    sim.step_exact(1)?;
    let f = backend(&sim).frame().unwrap();
    assert!(f.ground().is_some());
    assert_eq!(
        f.workers
            .iter()
            .map(|w| w.snapshot.bodies.len())
            .sum::<usize>(),
        7
    );
    Ok(())
}
