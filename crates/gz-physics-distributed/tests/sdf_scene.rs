// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{SceneBackend, coordinator::Frame, physics::Tier};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, LoadError, Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose,
    load_world_configured, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;
use std::{path::Path, time::Duration};

const EXAMPLE: &str = include_str!("../../../examples/distributed_boxes.sdf");
const MISSING: &str = "/missing/gz-distributed-no-such-worker";
fn load(xml: &str, worker: &str) -> Result<Simulation, LoadError> {
    load_world_configured(
        xml,
        "distributed.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(worker)),
        },
    )
}

fn frame(sim: &Simulation) -> &Frame {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .frame()
        .unwrap()
}
fn link(sim: &Simulation, name: &str) -> (u64, BodyId) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (
        entity.get::<EntityId>().unwrap().get(),
        entity.get::<BodyHandle>().unwrap().id(),
    )
}
fn synchronized(sim: &Simulation, model: &str, body: BodyId) {
    let state = frame(sim).body(body.raw()).unwrap().1;
    let model = format!("default::{model}");
    for name in [
        &model,
        &format!("{model}::body"),
        &format!("{model}::body::box"),
        &format!("{model}::body::paint"),
    ] {
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == *name))
            .unwrap();
        let pose = e.get::<WorldPose>().unwrap();
        for (a, b) in pose.translation.into_iter().zip(state.position) {
            assert!((a - b).abs() < 1e-12);
        }
        for (a, b) in pose.rotation.into_iter().zip(state.orientation) {
            assert!((a - b).abs() < 1e-12);
        }
        if let Some(v) = e.get::<Velocity>() {
            assert_eq!(v.linear, state.linear);
            assert_eq!(v.angular, state.angular);
        }
    }
}

#[test]
fn sdf_steps_two_local_workers_routes_before_contact_and_replays_after_reset() {
    let mut sim = load(EXAMPLE, env!("CARGO_BIN_EXE_physics_worker")).unwrap();
    let initial = frame(&sim).clone();
    let pusher = link(&sim, "default::pusher::body");
    let cargo = link(&sim, "default::cargo::body");
    let (pusher_owner, p) = initial.body(pusher.1.raw()).unwrap();
    let (cargo_owner, c) = initial.body(cargo.1.raw()).unwrap();
    assert_ne!(pusher_owner, cargo_owner);
    assert_eq!((p.tier, c.tier), (Tier::Prescribed, Tier::Dynamic));
    let trace = |sim: &mut Simulation| {
        let mut frames = Vec::new();
        let mut first_contact = None;
        let mut first_route = None;
        for tick in 1..=132 {
            sim.step_exact(1).unwrap();
            let f = frame(sim);
            assert_eq!(f.tick, tick);
            let (a, _) = f.body(pusher.1.raw()).unwrap();
            let (b, _) = f.body(cargo.1.raw()).unwrap();
            if a == b {
                first_route.get_or_insert(tick);
            }
            for impulse in f
                .workers
                .iter()
                .flat_map(|worker| &worker.snapshot.impulses)
            {
                assert_eq!([impulse.a, impulse.b], [pusher.1.raw(), cargo.1.raw()]);
                if impulse.normal_impulse > 0. {
                    assert_eq!(a, b);
                    first_contact.get_or_insert(tick);
                }
            }
            synchronized(sim, "cargo", cargo.1);
            synchronized(sim, "pusher", pusher.1);
            frames.push(f.clone());
        }
        assert!(first_route.unwrap() < first_contact.unwrap());
        assert!(
            (98..=102).contains(&first_contact.unwrap()),
            "{first_contact:?}"
        );
        let cargo_x = frame(sim).body(cargo.1.raw()).unwrap().1.position[0];
        assert!((cargo_x - 0.78).abs() < 1e-8);
        println!("route={first_route:?} first_contact={first_contact:?} cargo_x={cargo_x}");
        frames
    };
    let original = trace(&mut sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(frame(&sim), &initial);
    assert_eq!(link(&sim, "default::pusher::body"), pusher);
    assert_eq!(link(&sim, "default::cargo::body"), cargo);
    sim.set_paused(true);
    assert!(queue_link_world_wrench(&mut sim, pusher.0, [0., 10., 0.], [0.; 3]).is_err());
    assert!(queue_link_world_wrench(&mut sim, cargo.0, [0.; 3], [0., f64::NAN, 0.]).is_err());
    let force = [0., 10., 0.];
    let torque = [0., 0., 0.06];
    let impulse = [0.3, 0., 0.];
    let mut oracle = MultiFidelityBackend::new([0.; 3]).unwrap();
    let body = oracle
        .spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.3; 3],
                },
                position: c.position,
                velocity: c.linear,
                mass: Some(c.mass),
            },
            RotationSpec::default(),
            false,
        )
        .unwrap();
    oracle.apply_impulse(body, impulse, [0.; 3]).unwrap();
    oracle.add_world_wrench(body, force, torque).unwrap();
    let queue_impulse = |sim: &mut Simulation| {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<SceneBackend>()
            .unwrap()
            .queue_linear_impulse(cargo.1, impulse)
            .unwrap();
    };
    queue_impulse(&mut sim);
    queue_link_world_wrench(&mut sim, cargo.0, force, torque).unwrap();
    assert!(sim.tick().unwrap().dt.is_zero());
    assert_eq!(frame(&sim), &initial);
    for tick in 1..=2 {
        sim.step_exact(1).unwrap();
        oracle.step(Duration::from_millis(10)).unwrap();
        let measured = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(cargo.1)
            .unwrap();
        assert_eq!(measured, oracle.state(body).unwrap());
        println!("loaded wrench + impulse tick {tick}: {measured:?}");
        assert!((measured.velocity[0] - 0.3).abs() < 1e-14);
        assert!((measured.velocity[1] - 0.1).abs() < 1e-14);
        assert!((measured.angular_velocity[2] - 0.01).abs() < 1e-14);
        let expected_y = if tick == 1 { 0.000625 } else { 0.001625 };
        assert!(
            (measured.position[1] - expected_y).abs() < 1e-14,
            "{measured:?}"
        );
    }
    queue_impulse(&mut sim);
    queue_link_world_wrench(&mut sim, cargo.0, [0., 20., 0.], torque).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(trace(&mut sim), original);
}

#[test]
fn rotated_link_offset_preserves_model_and_attached_visual_frames() {
    let source = EXAMPLE
        .replacen(
            "<link name=\"body\">",
            "<link name=\"body\"><pose>0.1 -0.2 0.3 0.21 -0.17 0.42</pose>",
            1,
        )
        .replace(
            "<friction>",
            "<contact><category_bitmask>255</category_bitmask></contact><friction>",
        );
    let mut sim = load(&source, env!("CARGO_BIN_EXE_physics_worker")).unwrap();
    let pose = |sim: &Simulation, name: &str| {
        *sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<WorldPose>()
            .unwrap()
    };
    let names = [
        "default::pusher",
        "default::pusher::body",
        "default::pusher::body::paint",
    ];
    let initial = names.map(|name| pose(&sim, name));
    assert_ne!(initial[0].translation, initial[1].translation);
    assert_ne!(initial[0].rotation, initial[1].rotation);
    assert_eq!(initial[1].translation, initial[2].translation);
    sim.step_exact(10).unwrap();
    for (name, start) in names.into_iter().zip(initial) {
        let end = pose(&sim, name);
        let mut expected = start.translation;
        expected[0] += 0.15;
        for (a, b) in end.translation.into_iter().zip(expected) {
            assert!((a - b).abs() < 1e-12);
        }
        for (a, b) in end.rotation.into_iter().zip(start.rotation) {
            assert!((a - b).abs() < 1e-12);
        }
    }
}

#[test]
fn unsupported_sdf_fails_before_starting_any_worker() {
    for (from, to, detail) in [
        (">0.01<", ">0.003<", "10 ms"),
        (
            "<model name=\"cargo\">",
            "<model name=\"cargo\"><enable_wind>true</enable_wind>",
            "unsupported model wind",
        ),
        ("<mu2>0</mu2>", "", "equal nonnegative ODE"),
        ("<mu>0</mu>", "<mu>0.1</mu>", "equal nonnegative ODE"),
        ("</ode>", "<slip1>1</slip1></ode>", "default contact"),
        (
            "<friction>",
            "<contact><collide_bitmask>1</collide_bitmask></contact><friction>",
            "default contact",
        ),
        (
            "<collision name=\"box\">",
            "<collision name=\"box\"><pose>0.1 0 0 0 0 0</pose>",
            "centered",
        ),
        (
            "<inertial>",
            "<inertial><pose>0.1 0 0 0 0 0</pose>",
            "inertial axes",
        ),
        ("<ixx>0.06</ixx>", "<ixx>0.07</ixx>", "homogeneous inertia"),
        (
            "<box><size>0.6 0.4 0.5</size></box>",
            "<cylinder><radius>.3</radius><length>.4</length></cylinder>",
            "homogeneous inertia",
        ),
    ] {
        let xml = EXAMPLE.replacen(from, to, 1);
        assert_ne!(xml, EXAMPLE);
        let error = load(&xml, MISSING)
            .err()
            .expect("unsupported profile")
            .to_string();
        assert!(error.contains(detail), "{detail}: {error}");
    }
}

#[test]
fn two_drops_in_native_gravity_route_into_a_loaded_platform_and_repeat_after_reset() {
    let xml = include_str!("../../../examples/distributed_drop.sdf");
    let mut sim = load(xml, env!("CARGO_BIN_EXE_physics_worker")).unwrap();
    let initial = frame(&sim).clone();
    let ids = [
        "default::platform::body",
        "default::first::body",
        "default::second::body",
    ]
    .map(|n| link(&sim, n).1);
    assert_ne!(
        initial.body(ids[0].raw()).unwrap().0,
        initial.body(ids[2].raw()).unwrap().0
    );
    let mut reference = MultiFidelityBackend::new([0., 0., -9.81]).unwrap();
    let mut handles = Vec::new();
    for id in ids {
        let b = initial.body(id.raw()).unwrap().1;
        let handle = reference
            .spawn_awake_frictionless_box(
                BodySpec {
                    position: b.position,
                    velocity: b.linear,
                    mass: Some(b.mass),
                    shape: Shape::Box {
                        half_extents: b.size.map(|s| s * 0.5),
                    },
                },
                RotationSpec {
                    orientation: b.orientation,
                    angular_velocity: b.angular,
                },
                b.tier == Tier::Prescribed,
            )
            .unwrap();
        handles.push(handle);
    }
    let mut impacts = [None; 2];
    let mut route = None;
    let mut trace = Vec::new();
    for tick in 1..=205 {
        reference.step(Duration::from_millis(10)).unwrap();
        sim.step_exact(1).unwrap();
        let f = frame(&sim);
        let reference_pairs = reference.rigid_contact_observations();
        let native: Vec<_> = reference_pairs
            .iter()
            .filter(|p| p.active)
            .map(|p| (p.a.raw(), p.b.raw(), p.normal_impulse))
            .collect();
        let reported: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| (p.a, p.b, p.normal_impulse))
            .collect();
        assert_eq!(native, reported, "contact at {tick}");
        for (&id, handle) in ids.iter().zip(&handles) {
            let (owner, b) = f.body(id.raw()).unwrap();
            let oracle = reference.state(*handle).unwrap();
            assert_eq!(b.position, oracle.position, "position at {tick}");
            assert_eq!(b.orientation, oracle.orientation);
            assert_eq!(b.linear, oracle.velocity);
            assert_eq!(b.angular, oracle.angular_velocity);
            if id == ids[2] && owner == f.body(ids[0].raw()).unwrap().0 {
                route.get_or_insert(tick);
            }
        }
        for (_, id, impulse) in reported {
            if impulse > 0. {
                let index = ids[1..].iter().position(|b| b.raw() == id).unwrap();
                impacts[index].get_or_insert(tick);
            }
        }
        trace.push(f.clone());
    }
    assert!(impacts[0].unwrap() < route.unwrap() && route.unwrap() < impacts[1].unwrap());
    println!(
        "first impact {:?}, second load routed {:?}, second impact {:?}, last normal impulses {:?}",
        impacts[0],
        route,
        impacts[1],
        frame(&sim)
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| p.normal_impulse)
            .collect::<Vec<_>>()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(frame(&sim), &initial);
    for expected in trace {
        sim.step_exact(1).unwrap();
        assert_eq!(frame(&sim), &expected);
    }
}

#[test]
fn static_sdf_floor_carries_the_weight_of_a_two_box_stack() {
    let mut sim = load(
        include_str!("../../../examples/distributed_stack.sdf"),
        env!("CARGO_BIN_EXE_physics_worker"),
    )
    .unwrap();
    let (floor_link, floor) = link(&sim, "default::platform::body");
    let lower = link(&sim, "default::first::body").1;
    let upper = link(&sim, "default::second::body").1;
    let mut native = MultiFidelityBackend::new([0., 0., -9.81]).unwrap();
    for id in [floor, lower, upper] {
        let (_, b) = frame(&sim).body(id.raw()).unwrap();
        assert_eq!(
            native
                .spawn_awake_frictionless_box(
                    BodySpec {
                        position: b.position,
                        velocity: b.linear,
                        mass: (id != floor).then_some(b.mass),
                        shape: Shape::Box {
                            half_extents: b.size.map(|s| s * 0.5)
                        },
                    },
                    RotationSpec {
                        orientation: b.orientation,
                        angular_velocity: b.angular
                    },
                    false
                )
                .unwrap(),
            id
        );
    }
    assert_eq!(frame(&sim).body(floor.raw()).unwrap().1.tier, Tier::Fixed);
    assert_eq!(native.is_dynamic(floor), Some(false));
    assert_eq!(native.is_kinematic(floor), Some(false));
    assert!(queue_link_world_wrench(&mut sim, floor_link, [1., 0., 0.], [0.; 3]).is_err());
    let mut first = None;
    let mut second = None;
    for tick in 1..=360 {
        sim.step_exact(1).unwrap();
        native.step(Duration::from_millis(10)).unwrap();
        let backend = sim.world().resource::<PhysicsBackend>();
        for id in [floor, lower, upper] {
            assert_eq!(backend.state(id), native.state(id), "state at tick {tick}");
        }
        let actual: Vec<_> = frame(&sim)
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter())
            .map(|p| (p.a, p.b, p.normal_impulse))
            .collect();
        let oracle: Vec<_> = native
            .rigid_contact_observations()
            .iter()
            .filter(|p| p.active)
            .map(|p| (p.a.raw(), p.b.raw(), p.normal_impulse))
            .collect();
        assert_eq!(actual, oracle);
        for &(a, b, j) in &actual {
            if j > 0. && (a, b) == (floor.raw(), lower.raw()) {
                first.get_or_insert(tick);
            }
            if j > 0. && (a, b) == (lower.raw(), upper.raw()) {
                second.get_or_insert(tick);
            }
        }
        if tick == 360 {
            assert_eq!(actual.len(), 2);
            assert!(
                (actual[0].2 / 0.01 - 3. * 9.81).abs() < 0.005,
                "floor reaction {:?}",
                actual
            );
            assert!(
                (actual[1].2 / 0.01 - 2. * 9.81).abs() < 0.005,
                "stack reaction {:?}",
                actual
            );
            println!(
                "fixed floor: impacts {first:?}, {second:?}; floor {:.6} N, interface {:.6} N",
                actual[0].2 / 0.01,
                actual[1].2 / 0.01
            );
        }
    }
    assert!(first.unwrap() < second.unwrap());
}

#[test]
fn sdf_step_sets_native_and_bevy_intervals_including_after_reset() {
    use gz_physics_distributed::physics::SimulationStep;
    for (seconds, profile) in [
        ("0.001", SimulationStep::Ms1),
        ("0.002", SimulationStep::Ms2),
        ("0.005", SimulationStep::Ms5),
        ("0.010", SimulationStep::Ms10),
    ] {
        let source = EXAMPLE.replace(">0.01<", &format!(">{seconds}<"));
        let mut sim = load(&source, env!("CARGO_BIN_EXE_physics_worker")).unwrap();
        let h = profile.duration();
        let seed = frame(&sim).clone();
        assert_eq!(seed.step(), profile);
        assert_eq!(sim.step_duration(), h);
        let cargo = link(&sim, "default::cargo::body");
        queue_link_world_wrench(&mut sim, cargo.0, [0., 8., 0.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(sim.elapsed(), h);
        let moved = frame(&sim).body(cargo.1.raw()).unwrap().1;
        let dt = h.as_secs_f64();
        assert!((moved.linear[1] - 8. * dt).abs() < 1e-14);
        assert!((moved.position[1] - 5. * dt * dt).abs() < 1e-14);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(frame(&sim), &seed);
    }
}

#[test]
fn shared_sdf_plane_preserves_handles_rays_support_and_reset() {
    let xml = include_str!("../../../examples/distributed_ground.sdf");
    let mut sim = load(xml, env!("CARGO_BIN_EXE_physics_worker")).unwrap();
    let model = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::ground"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    assert!(matches!(
        gz_sim_scene::model_state(&sim, model),
        Err(gz_sim_scene::WorldEditError::UnsupportedModel)
    ));
    let floor = link(&sim, "default::ground::floor");
    let blue = link(&sim, "default::blue::body");
    let seed = frame(&sim).clone();
    assert_eq!(seed.ground().unwrap().height(), 0.25);
    {
        let p = sim.world().resource::<PhysicsBackend>();
        assert_eq!(p.is_dynamic(floor.1), Some(false));
        assert_eq!(p.state(floor.1).unwrap().position, [0., 0., 0.25]);
        // Collision extends beyond the rendered 18 m wide SDF visual.
        assert_eq!(
            p.cast_ray([14., 1., 4.], [0., 0., -1.], 6., None).unwrap(),
            Some(3.75)
        );
        assert_eq!(
            p.cast_ray([14., 1., 4.], [0., 0., -1.], 6., Some(floor.1))
                .unwrap(),
            None
        );
        assert_eq!(
            p.cast_ray([14., 1., 0.], [0., 0., 1.], 6., None).unwrap(),
            Some(0.)
        );
    }
    sim.step_exact(160).unwrap();
    let loaded = frame(&sim).clone();
    assert!(
        loaded
            .workers
            .iter()
            .all(|w| !w.snapshot.supports.is_empty())
    );
    assert_eq!(
        loaded
            .workers
            .iter()
            .map(|w| w.snapshot.supports.len())
            .sum::<usize>(),
        3
    );
    assert_eq!(
        loaded
            .workers
            .iter()
            .map(|w| w.snapshot.contacts)
            .sum::<usize>(),
        0
    );
    let pose = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(blue.1)
        .unwrap();
    let below = gz_physics::BodyState {
        position: [0., 0., 0.1],
        ..pose
    };
    let mut p = sim.world_mut().resource_mut::<PhysicsBackend>();
    assert!(p.set_body_state(blue.1, below).is_err());
    assert!(p.set_body_state(floor.1, pose).is_err());
    assert!(!p.remove(floor.1));
    assert_eq!(frame(&sim), &loaded);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(frame(&sim), &seed);
    assert_eq!(link(&sim, "default::ground::floor"), floor);
    sim.step_exact(160).unwrap();
    assert_eq!(frame(&sim), &loaded);
}

#[test]
fn invalid_shared_ground_never_starts_workers() {
    let source = include_str!("../../../examples/distributed_ground.sdf");
    for (old, new) in [
        ("<static>true</static>", "<static>false</static>"),
        ("<normal>0 0 1</normal>", "<normal>0 1 0</normal>"),
        (
            "<pose>0 0 0.25 0 0 0</pose>",
            "<pose>0 0 0.25 0 3.141592653589793 0</pose>",
        ),
        ("<mu>0</mu>", "<mu>0.1</mu>"),
        (
            "<pose>-6 0 1.05 0 0 0</pose>",
            "<pose>-6 0 0.35 0 0 0</pose>",
        ),
    ] {
        let xml = source.replacen(old, new, 1);
        let error = match load(&xml, MISSING) {
            Ok(_) => panic!("admitted {new}"),
            Err(e) => e,
        };
        assert!(
            !error.to_string().contains("No such file"),
            "{new}: {error}"
        );
    }
    let start = source.find("<model name=\"ground\">").unwrap();
    let end = start + source[start..].find("</model>").unwrap() + "</model>".len();
    let duplicate = source[start..end].replace("name=\"ground\"", "name=\"another_floor\"");
    let xml = source.replace("</world>", &(duplicate + "</world>"));
    assert!(matches!(
        load(&xml, MISSING),
        Err(LoadError::Unsupported { .. })
    ));
}

#[test]
fn two_rough_ground_contacts_join_before_a_shared_impact_without_losing_static_or_sliding_friction()
{
    let xml = include_str!("../../../examples/distributed_rough_ground.sdf");
    let mut sim = load(xml, env!("CARGO_BIN_EXE_physics_worker")).unwrap();
    let seed = frame(&sim).clone();
    assert_eq!(seed.ground().unwrap().friction(), 0.25);
    let gravity = [2.1, 0., -9.81];
    let step = sim.step_duration();
    let mut native = MultiFidelityBackend::new(gravity).unwrap();
    native.set_partition_step(step).unwrap();
    native.set_partition_ground_material(0.25, 0.25).unwrap();
    let mut ids = Vec::new();
    for (id, mu) in [(0, 0.06), (1, 0.8)] {
        let b = seed.body(id).unwrap().1;
        assert_eq!(b.friction, mu);
        ids.push(
            native
                .spawn_partition_box(
                    BodySpec {
                        shape: Shape::Box {
                            half_extents: b.size.map(|v| v * 0.5),
                        },
                        mass: Some(b.mass),
                        position: b.position,
                        velocity: b.linear,
                    },
                    RotationSpec::default(),
                    false,
                    mu,
                )
                .unwrap(),
        );
    }
    let mut joined = None;
    let mut impact = None;
    let mut v_at_one = 0.;
    let mut v_at_two = 0.;
    let mut held_x = 0.;
    for tick in 1..=740 {
        let was_separate = frame(&sim).body(0).unwrap().0 != frame(&sim).body(1).unwrap().0;
        let supports_before = frame(&sim)
            .workers
            .iter()
            .map(|w| w.snapshot.supports.len())
            .sum::<usize>();
        sim.step_exact(1).unwrap();
        native.step(step).unwrap();
        let f = frame(&sim);
        if was_separate && f.body(0).unwrap().0 == f.body(1).unwrap().0 {
            assert_eq!(supports_before, 2);
            joined = Some(tick);
        }
        for (i, &id) in ids.iter().enumerate() {
            let b = f.body(i as u64).unwrap().1;
            let n = native.state(id).unwrap();
            assert_eq!(
                (
                    b.position.map(f64::to_bits),
                    b.linear.map(f64::to_bits),
                    b.orientation.map(f64::to_bits),
                    b.angular.map(f64::to_bits)
                ),
                (
                    n.position.map(f64::to_bits),
                    n.velocity.map(f64::to_bits),
                    n.orientation.map(f64::to_bits),
                    n.angular_velocity.map(f64::to_bits)
                ),
                "body {i} tick {tick}"
            );
        }
        let supports = native
            .partition_ground_contacts()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| (p.body.raw(), p.normal_impulse.to_bits()))
            .collect::<Vec<_>>();
        let mut actual = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .map(|p| (p.body, p.normal_impulse.to_bits()))
            .collect::<Vec<_>>();
        actual.sort();
        assert_eq!(actual, supports, "ground reaction at {tick}");
        let observed = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| p.normal_impulse.to_bits())
            .collect::<Vec<_>>();
        let contacts = native
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| p.normal_impulse.to_bits())
            .collect::<Vec<_>>();
        assert_eq!(observed, contacts, "mobile contact {tick}");
        if contacts.iter().any(|&j| f64::from_bits(j) > 0.) {
            impact.get_or_insert(tick);
        }
        if tick == 200 {
            v_at_one = f.body(0).unwrap().1.linear[0];
            held_x = f.body(1).unwrap().1.position[0];
        }
        if tick == 400 {
            v_at_two = f.body(0).unwrap().1.linear[0];
            assert!((f.body(1).unwrap().1.position[0] - held_x).abs() < 1e-6);
        }
    }
    let acceleration = v_at_two - v_at_one;
    assert!((acceleration - (2.1 - 0.06 * 9.81)).abs() < 1e-6);
    assert!(joined.is_some() && impact.is_some() && joined < impact);
    println!(
        "rough ground mu 0.25, body mu 0.06 and 0.80: first shared owner {joined:?}, first positive mutual contact {impact:?}, sliding acceleration {acceleration:.9} m/s²; 740/740 complete native state and pair/ground impulses bit-exact"
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(frame(&sim), &seed);
}
