// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
use gz_physics_gpu::{
    BodySlot, EnvironmentState, GpuError, MixedBatch,
    simulation::{batch_boundary, checkpoint_mixed, reset_environments},
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    EntityKind, LocalPose, Name, PhysicsEngine, SceneResetError, Velocity, WorldLoadOptions,
    WorldPose,
    gpu_scene::{GpuSceneBindings, GpuSceneObservations, queue_link_impulse},
    load_world_on_adapter, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const WORLD: &str = include_str!("../../../examples/gpu_mixed_world.sdf");
fn load(
    xml: &str,
    engine: PhysicsEngine,
    adapter: Option<&str>,
) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_on_adapter(
        xml,
        "test.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        adapter,
    )
}
fn refused(xml: &str, engine: PhysicsEngine, adapter: Option<&str>, expected: &str) {
    let err = load(xml, engine, adapter)
        .err()
        .expect("scene must be refused");
    assert!(
        err.to_string().contains(expected),
        "{err:?}, expected {expected}"
    );
}
#[test]
fn gpu_configuration_is_explicit_and_scoped_before_device_open() {
    refused(WORLD, PhysicsEngine::GpuBatch, None, "gpu-adapter");
    refused(
        WORLD,
        PhysicsEngine::Rapier,
        None,
        "GpuBatch plugin requires",
    );
    refused(
        WORLD,
        PhysicsEngine::Rapier,
        Some("absent"),
        "GPU adapter requires",
    );
    let absent = Some("unavailable test device");
    let blank = r#"<sdf version="1.12"><world name="default"><physics name="gpu" type="ignored"><max_step_size>0.001</max_step_size></physics><plugin name="gz::sim::systems::GpuBatch" filename="gz-sim-gpu-batch-system"/></world></sdf>"#;
    refused(
        blank,
        PhysicsEngine::GpuBatch,
        absent,
        "empty GPU environment set",
    );
    let doc = roxmltree::Document::parse(WORLD).unwrap();
    let pair = doc
        .descendants()
        .find(|n| n.attribute("profile") == Some("sphere-pair"))
        .unwrap();
    let mut without_owner = WORLD.to_owned();
    without_owner.replace_range(pair.range(), "");
    refused(
        &without_owner,
        PhysicsEngine::GpuBatch,
        absent,
        "no GPU environment owner",
    );
    for (from, to, needle) in [
        (
            " profile=\"sphere-pair\"",
            " execution=\"auto\" profile=\"sphere-pair\"",
            "execution must be cpu or gpu",
        ),
        (
            "<a link=\"rolling::body\"",
            "<b link=\"rolling::body\"",
            "missing a sphere",
        ),
        (
            "<a link=\"rolling::body\"",
            "<a unused=\"rolling::body\"",
            "missing scoped model::link",
        ),
        (
            "linear=\"0 -0.73 0\"",
            "linear=\"bad 0 0\"",
            "linear velocity triple",
        ),
        (
            "linear=\"0 -0.73 0\"",
            "linear=\"0 bad 0\"",
            "linear velocity triple",
        ),
        (
            "linear=\"0 -0.73 0\"",
            "linear=\"0 0 bad\"",
            "linear velocity triple",
        ),
        (
            "<link name=\"body\">",
            "<link name=\"other\"/><link name=\"body\">",
            "one ordinary link",
        ),
        ("<max_step_size>0.001", "<max_step_size>0.002", "1 ms"),
        ("id=\"31\"", "id=\"9\"", "duplicate environment id"),
        (
            "<observe every=\"10\"/>",
            "<observe every=\"0\"/>",
            "observation cadence",
        ),
        (
            "<observe every=\"10\"/>",
            "<observe every=\"10\"/><observe every=\"20\"/>",
            "one <observe",
        ),
        (
            "<observe every=\"10\"/>",
            "<observe every=\"10\">x</observe>",
            "one <observe",
        ),
        ("id=\"31\"", "stray=\"q\" id=\"31\"", "unknown fields"),
        ("id=\"31\"", "id=\"invalid\"", "integer environment id"),
        (" profile=\"sphere-pair\"", "", "missing profile"),
        (" friction=\"0.20\"", "", "missing or invalid friction"),
        (
            "restitution=\"0.85\"",
            "restitution=\"oops\"",
            "invalid restitution",
        ),
        (
            "profile=\"sphere-pair\"",
            "profile=\"other\"",
            "unsupported profile",
        ),
        (
            "profile=\"sphere-plane\"",
            "profile=\"sphere-plane\" restitution=\"0.5\"",
            "unsupported profile",
        ),
        (
            "<b link=\"free_b::body\"",
            "<a link=\"free_b::body\"",
            "duplicate body role",
        ),
        (
            "link=\"free_b::body\"",
            "link=\"free_a::body\"",
            "multiply-owned",
        ),
        (
            "link=\"free_b::body\"",
            "link=\"missing::body\"",
            "unresolved",
        ),
        (
            "linear=\"0 -0.73 0\"",
            "linear=\"0 -0.73\"",
            "linear velocity triple",
        ),
        (
            "angular=\"0.4 1.8 1\"",
            "angular=\"NaN 0 0\"",
            "angular velocity triple",
        ),
        (
            "<plane link=\"incline::base\"/>",
            "<b link=\"incline::base\"/>",
            "role, physical geometry",
        ),
        (
            "<ixx>0.0086240000000000032",
            "<ixx>0.009",
            "solid-sphere inertia",
        ),
        (
            "<static>true</static>",
            "<static>false</static>",
            "dynamic sphere or a static plane",
        ),
        (
            "<collision name=\"sphere\">",
            "<collision name=\"sphere\"><pose>0 1 0 0 0 0</pose>",
            "coincide",
        ),
        (
            "filename=\"gz-sim-gpu-batch-system\">",
            "filename=\"gz-sim-gpu-batch-system\" junk=\"x\">",
            "invalid GpuBatch",
        ),
        (
            "<observe every=\"10\"/>",
            "<unexpected/>",
            "integer environment id",
        ),
    ] {
        assert!(WORLD.contains(from));
        refused(
            &WORLD.replace(from, to),
            PhysicsEngine::GpuBatch,
            Some("unavailable test device"),
            needle,
        );
    }
}

fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn gpu_world() -> Simulation {
    let selector = std::env::var("GZ_GPU_ADAPTER")
        .expect("set GZ_GPU_ADAPTER to a physical f64 Vulkan adapter");
    load(WORLD, PhysicsEngine::GpuBatch, Some(&selector)).unwrap()
}
fn compare_projection(sim: &Simulation) {
    let world = sim.world();
    let bindings = world.resource::<GpuSceneBindings>();
    let batch = world.resource::<MixedBatch>();
    for (name, id, slot) in [
        ("rolling", 9, BodySlot::A),
        ("free_a", 31, BodySlot::A),
        ("free_b", 31, BodySlot::B),
        ("supported_a", 80, BodySlot::A),
        ("supported_b", 80, BodySlot::B),
    ] {
        let model = entity(sim, &format!("default::{name}"));
        let link = entity(sim, &format!("default::{name}::body"));
        let marker = entity(sim, &format!("default::{name}::body::spin_marker"));
        assert_eq!(world.get::<EntityKind>(link), Some(&EntityKind::Link));
        assert_eq!(
            bindings.resolve(world.get::<EntityId>(link).unwrap().get()),
            Some((id, slot))
        );
        let (p, q, v, w) = match batch.get(id).unwrap() {
            EnvironmentState::SpherePlane(s) => {
                (s.position, s.orientation, s.velocity, s.angular_velocity)
            }
            state => {
                let pair = match state {
                    EnvironmentState::SpherePair(pair) => pair,
                    EnvironmentState::SupportedSpherePair(s) => &s.pair,
                    _ => unreachable!(),
                };
                let b = if slot == BodySlot::A { pair.a } else { pair.b };
                (b.position, b.orientation, b.velocity, b.angular_velocity)
            }
        };
        for node in [model, link] {
            let pose = world.get::<WorldPose>(node).unwrap();
            assert_eq!(pose.translation, p, "model/link {name}");
            assert_eq!(pose.rotation, q);
        }
        let velocity = world.get::<Velocity>(link).unwrap();
        assert_eq!(velocity.linear, v);
        assert_eq!(velocity.angular, w);
        let marker_local = world.get::<LocalPose>(marker).unwrap().0;
        let marker_pose = world.get::<WorldPose>(marker).unwrap();
        let d = glam::DQuat::from_array(q) * glam::DVec3::from_array(marker_local.translation)
            + glam::DVec3::from_array(p);
        assert!(
            d.distance(glam::DVec3::from_array(marker_pose.translation)) < 1e-13,
            "stale marker {name}"
        );
    }
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn world_clock_hierarchy_controls_and_reset_are_coherent() {
    let mut sim = gpu_world();
    assert!(
        !sim.world()
            .contains_resource::<gz_physics_rapier::PhysicsBackend>()
    );
    let initial = sim.world().resource::<MixedBatch>().environments().to_vec();
    compare_projection(&sim);
    let link = entity(&sim, "default::free_a::body");
    let link_id = sim.world().get::<EntityId>(link).unwrap().get();
    let plane = entity(&sim, "default::incline::base");
    let plane_id = sim.world().get::<EntityId>(plane).unwrap().get();
    let at = batch_boundary(&sim);
    assert!(matches!(
        queue_link_impulse(&mut sim, at, plane_id, [0.; 3]),
        Err(GpuError::UnknownSceneLink(_))
    ));
    assert!(matches!(
        reset_environments(&mut sim, at, &initial),
        Err(GpuError::FixedScene)
    ));
    assert!(matches!(checkpoint_mixed(&sim), Err(GpuError::FixedScene)));
    queue_link_impulse(&mut sim, at, link_id, [0.085, 0., 0.]).unwrap();
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .resource::<MixedBatch>()
            .pending_impulses()
            .len(),
        1
    );
    assert_eq!(sim.world().resource::<MixedBatch>().environments(), initial);
    sim.step_exact(9).unwrap();
    compare_projection(&sim);
    assert_eq!(
        sim.world()
            .resource::<MixedBatch>()
            .pending_impulses()
            .len(),
        0
    );
    let EnvironmentState::SpherePair(pair) = sim.world().resource::<MixedBatch>().get(31).unwrap()
    else {
        panic!("free pair")
    };
    assert!((pair.a.velocity[0] - 0.1).abs() < 2e-15);
    let o = sim.world().resource::<GpuSceneObservations>();
    assert_eq!(o.tick(), 0);
    assert_eq!(o.environments(), initial);
    assert!(matches!(
        queue_link_impulse(&mut sim, at, link_id, [0.; 3]),
        Err(GpuError::StaleBoundary)
    ));
    sim.step_exact(1).unwrap();
    let o = sim.world().resource::<GpuSceneObservations>();
    assert_eq!(
        (o.every(), o.epoch(), o.tick(), o.sim_time()),
        (10, 0, 10, Duration::from_millis(10))
    );
    assert_eq!(
        o.environments(),
        sim.world().resource::<MixedBatch>().environments()
    );
    sim.step_exact(3).unwrap();
    compare_projection(&sim);
    assert_eq!(sim.world().resource::<GpuSceneObservations>().tick(), 10);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    compare_projection(&sim);
    assert_eq!(sim.world().resource::<MixedBatch>().environments(), initial);
    let o = sim.world().resource::<GpuSceneObservations>();
    assert_eq!((o.epoch(), o.tick(), o.sim_time()), (1, 0, Duration::ZERO));
    assert_eq!(o.environments(), initial);
    assert_eq!(
        sim.world()
            .get::<EntityId>(entity(&sim, "default::free_a::body"))
            .unwrap()
            .get(),
        link_id
    );
    sim.step_exact(1).unwrap();
    sim.reset().unwrap();
    compare_projection(&sim);
    assert_eq!(sim.world().resource::<GpuSceneObservations>().epoch(), 2);
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn missing_support_or_replaced_owner_refuses_before_physics_advance() {
    let mut sim = gpu_world();
    let moving = entity(&sim, "default::free_a::body");
    let id = sim.world().get::<EntityId>(moving).unwrap().get();
    let mut another = gpu_world();
    let other = another.world_mut().remove_resource::<MixedBatch>().unwrap();
    sim.world_mut().insert_resource(other);
    let at = batch_boundary(&sim);
    assert!(matches!(
        queue_link_impulse(&mut sim, at, id, [0.; 3]),
        Err(GpuError::StaleBoundary)
    ));
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::ForeignPhysics)
    ));
    assert!(gz_sim_scene::gpu_scene::execution_counts(&sim).is_none());
    assert!(matches!(
        gz_sim_scene::gpu_scene::environment_placement(&sim, 31),
        Err(GpuError::StaleBoundary)
    ));
    assert!(matches!(
        gz_sim_scene::gpu_scene::set_environment_placement(
            &mut sim,
            at,
            &[31],
            gz_physics_gpu::ExecutionPlacement::Cpu
        ),
        Err(GpuError::StaleBoundary)
    ));
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.world().resource::<MixedBatch>().tick(), 0);
    let mut sim = gpu_world();
    let plane = entity(&sim, "default::incline::base");
    sim.world_mut()
        .get_mut::<WorldPose>(plane)
        .unwrap()
        .translation[2] += 0.01;
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.world().resource::<MixedBatch>().tick(), 0);
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn dead_bound_link_refuses_impulse_synchronously() {
    let mut sim = gpu_world();
    let moving = entity(&sim, "default::free_a::body");
    let id = sim.world().get::<EntityId>(moving).unwrap().get();
    sim.world_mut().entity_mut(moving).remove::<Velocity>();
    let at = batch_boundary(&sim);
    assert!(
        matches!(queue_link_impulse(&mut sim, at, id, [0.02, 0., 0.]), Err(GpuError::UnknownSceneLink(link)) if link == id)
    );
    assert_eq!(
        sim.world()
            .resource::<MixedBatch>()
            .pending_impulses()
            .count(),
        0
    );
}
