// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, PhysicsBackend, PhysicsError, RotationSpec, Shape,
};
use gz_physics_distributed::{
    SceneBackend,
    physics::{STEP, Tier},
};
use gz_physics_multifidelity::{MultiFidelityBackend, SolidRay};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, LidarSample, Name, PhysicsEngine, WorldLoadOptions, load_world_configured,
    reset_loaded_world, set_lidar_interest,
};
use sdformat::NoResources;
use std::{path::Path, time::Duration};

fn distributed(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}

#[test]
fn rotated_box_casts_match_native_validation_endpoints_and_exclusion_before_initialization() {
    let mut scene = SceneBackend::new(Path::new("/unused"), [0.; 3], STEP).unwrap();
    let mut native = MultiFidelityBackend::new([0.; 3]).unwrap();
    let mut ids = vec![];
    for (x, angle, sides) in [(0., 0., [0.3, 0.4, 0.2]), (2., 0.4_f64, [0.5, 0.7, 0.3])] {
        let spec = BodySpec {
            shape: Shape::Box {
                half_extents: sides,
            },
            position: [x, 0., 0.],
            velocity: [0.; 3],
            mass: Some(1.),
        };
        let r = RotationSpec {
            orientation: [0., 0., (angle / 2.).sin(), (angle / 2.).cos()],
            ..Default::default()
        };
        ids.push((
            scene.spawn_link(spec, r, None).unwrap(),
            native.spawn_awake_frictionless_box(spec, r, false).unwrap(),
        ));
    }
    let cast = |o, d, limit, exclude: Option<usize>| {
        let result = scene.cast_ray(o, d, limit, exclude.map(|i| ids[i].0));
        assert_eq!(
            result,
            native.cast_ray(o, d, limit, exclude.map(|i| ids[i].1))
        );
        result.unwrap()
    };
    assert_eq!(cast([0.; 3], [1., 0., 0.], 10., None), Some(0.));
    assert_eq!(cast([-1.3, 0.0, 0.0], [1., 0., 0.], 1., None), Some(1.));
    assert_eq!(cast([-1.3, 0.4, 0.2], [1., 0., 0.], 1., None), Some(1.));
    assert_eq!(
        cast(
            [-1.3, 0.4, 0.2],
            [1., 0., 0.],
            f64::from_bits(1f64.to_bits() - 1),
            None
        ),
        None
    );
    assert_eq!(cast([0.; 3], [0., 1., 0.], 10., Some(0)), None);
    let x = cast([0.; 3], [1., 0., 0.], 3., Some(0)).unwrap();
    assert!((x - (2. - 0.5 / 0.4_f64.cos())).abs() < 1e-14);
    let missing = Some(BodyId::from_raw(882));
    for (o, d, limit) in [
        ([f64::NAN, 0., 0.], [1., 0., 0.], 1.),
        ([0.; 3], [1.01, 0., 0.], 1.),
        ([0.; 3], [1., 0., 0.], 0.),
        ([f64::MAX, 0., 0.], [1., 0., 0.], f64::MAX),
    ] {
        assert_eq!(
            scene.cast_ray(o, d, limit, missing),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(
        scene.cast_ray([0.; 3], [1., 0., 0.], 1., missing),
        Err(PhysicsError::MissingBody)
    );
    assert!(scene.remove(ids[1].0));
    assert_eq!(
        scene.cast_ray([0.; 3], [1., 0., 0.], 3., Some(ids[0].0)),
        Ok(None)
    );
}

#[test]
fn solid_query_rejects_invalid_box_states_without_losing_a_previous_equal_distance_hit() {
    let mut query = SolidRay::new([0.; 3], [1., 0., 0.], 3.).unwrap();
    let identity = [0., 0., 0., 1.];
    for y in [-0.5, 0.5] {
        query.include_box([1.; 3], [2., y, 0.], identity).unwrap();
    }
    assert_eq!(query.distance(), Some(1.5));
    for (size, position, q) in [
        ([0.; 3], [0.; 3], identity),
        ([f64::INFINITY; 3], [0.; 3], identity),
        ([1.; 3], [f64::NAN; 3], identity),
        ([1.; 3], [0.; 3], [0.; 4]),
    ] {
        assert_eq!(
            query.include_box(size, position, q),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(query.distance(), Some(1.5));
    }
}

#[test]
fn moving_sdf_lidar_matches_uninterrupted_native_world_before_and_after_route_and_contact() {
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_lidar.sdf"),
        "distributed_lidar.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            step: None,
            engine: PhysicsEngine::Distributed,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )
    .unwrap();
    let sensor_name = "default::scanner::body::laser";
    let sensor = entity(&sim, sensor_name);
    let sensor_id = sim.world().get::<EntityId>(sensor).unwrap().get();
    let scanner = sim
        .world()
        .get::<BodyHandle>(entity(&sim, "default::scanner::body"))
        .unwrap()
        .id();
    let initial = distributed(&sim).frame().unwrap().clone();
    let mut native = MultiFidelityBackend::new([0.; 3]).unwrap();
    let mut native_scanner = None;
    let mut bodies: Vec<_> = initial
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .collect();
    bodies.sort_by_key(|b| b.id);
    for b in bodies {
        let h = native
            .spawn_awake_frictionless_box(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: b.size.map(|v| v / 2.),
                    },
                    position: b.position,
                    velocity: b.linear,
                    mass: Some(b.mass),
                },
                RotationSpec {
                    orientation: b.orientation,
                    angular_velocity: b.angular,
                },
                b.tier == Tier::Prescribed,
            )
            .unwrap();
        if b.id == scanner.raw() {
            native_scanner = Some(h);
        }
    }
    let mut route = None;
    let mut contact = None;
    let mut max_error = 0.0f64;
    let mut hits = 0;
    let mut misses = 0;
    let mut trace = Vec::new();
    for tick in 1..=130 {
        sim.step_exact(1).unwrap();
        native.step(STEP).unwrap();
        let f = distributed(&sim).frame().unwrap();
        if f.workers
            .iter()
            .any(|w| w.generations.values().any(|g| *g > 0))
        {
            route.get_or_insert(tick);
        }
        if f.workers
            .iter()
            .any(|w| w.snapshot.impulses.iter().any(|i| i.normal_impulse > 0.))
        {
            contact.get_or_insert(tick);
        }
        let sample = sim.world().get::<LidarSample>(sensor).unwrap();
        assert_eq!(sample.epoch, 0);
        assert_eq!(sample.time, Duration::from_millis(tick * 10));
        assert_eq!(sample.ranges.len(), 31);
        let position = native.state(native_scanner.unwrap()).unwrap().position;
        let origin = [position[0] + 0.02, position[1], position[2] + 0.02];
        assert_eq!(sample.world_pose.translation, origin);
        for (i, &measured) in sample.ranges.iter().enumerate() {
            let angle = -0.75 + 0.05 * i as f64;
            let dir = [angle.cos(), angle.sin(), 0.];
            let expected = native.cast_ray(origin, dir, 6., native_scanner).unwrap();
            if let Some(range) = expected {
                assert!(measured.is_finite());
                max_error = max_error.max((range - measured).abs());
                hits += 1;
            } else {
                assert_eq!(measured, f64::INFINITY);
                misses += 1;
            }
        }
        trace.push(sample.ranges.clone());
    }
    assert!(route.unwrap() < contact.unwrap());
    assert!(
        max_error < 1e-11 && hits > 500 && misses > 100,
        "error={max_error} hits={hits} misses={misses}"
    );
    println!(
        "31 rays x 130 ticks, hits={hits} misses={misses} max_range_error={max_error} m route={route:?} contact={contact:?}"
    );
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(
        sim.world().get::<LidarSample>(sensor).unwrap().ranges,
        *trace.last().unwrap()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let sensor = entity(&sim, sensor_name);
    assert_eq!(
        sim.world().get::<EntityId>(sensor).unwrap().get(),
        sensor_id
    );
    set_lidar_interest(&mut sim, []);
    sim.step_exact(1).unwrap();
    assert!(sim.world().get::<LidarSample>(sensor).is_none());
    set_lidar_interest(&mut sim, [sensor]);
    for expected in trace.into_iter().skip(1) {
        sim.step_exact(1).unwrap();
        let s = sim.world().get::<LidarSample>(sensor).unwrap();
        assert_eq!(s.epoch, 1);
        assert_eq!(s.ranges, expected);
    }
}
