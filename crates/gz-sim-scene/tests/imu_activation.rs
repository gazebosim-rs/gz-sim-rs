// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, StepFault, entity_by_id};
use gz_sim_scene::{
    ImuActivationError, ImuPublication, ImuSample, Name, PhysicsEngine, imu_active,
    load_world_seeded, queue_link_world_velocity, remove_model, reset_loaded_world, set_imu_active,
    set_imu_interest, set_imu_rate,
};
use sdformat::NoResources;
use std::time::Duration;

fn fixture(rate: u32) -> String {
    include_str!("fixtures/fall.sdf").replace(
        "<update_rate>1000</update_rate>",
        &format!("<update_rate>{rate}</update_rate>"),
    )
}

fn load(xml: &str, engine: PhysicsEngine) -> (Simulation, u64) {
    let sim = load_world_seeded(
        xml,
        "activation.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &NoResources,
        engine,
        [41; 32],
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<ImuPublication>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    (sim, id)
}

fn sample(sim: &Simulation, id: u64) -> (Duration, gz_sensors::ImuReading) {
    let entity = entity_by_id(sim.world(), id).unwrap();
    let sample = sim.world().get::<ImuSample>(entity).unwrap();
    (sample.time, *sample.reading.as_ref().unwrap())
}

fn named_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn topic_id(sim: &Simulation, topic: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<ImuPublication>()
                .is_some_and(|value| value.topic == topic)
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn native_default_and_mutation_getters_preserve_phase_across_inactive_rate_changes() {
    let native: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/imu_activation.json")).unwrap();
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let (mut sim, id) = load(&fixture(0), engine);
        assert_eq!(
            imu_active(&sim, id).unwrap(),
            native["observations"]["groups"][0]["initial"]["active"]
                .as_bool()
                .unwrap()
        );
        set_imu_rate(&mut sim, id, 20.0).unwrap();
        sim.step_exact(1).unwrap();
        let cached = sample(&sim, id);
        set_imu_active(&mut sim, id, false).unwrap();
        assert_eq!(
            imu_active(&sim, id).unwrap(),
            native["observations"]["groups"][0]["rows"][1]["active"]
                .as_bool()
                .unwrap()
        );
        set_imu_rate(&mut sim, id, 10.0).unwrap();
        sim.step_exact(249).unwrap();
        assert_eq!(sample(&sim, id), cached);
        set_imu_active(&mut sim, id, true).unwrap();
        set_imu_active(&mut sim, id, true).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(sample(&sim, id).0, Duration::from_millis(251));
        sim.step_exact(98).unwrap();
        assert_eq!(sample(&sim, id).0, Duration::from_millis(251));
        sim.step_exact(1).unwrap();
        assert_eq!(sample(&sim, id).0, Duration::from_millis(350));
    }
}

#[test]
fn inactive_due_connected_imu_still_admits_its_continuous_peer() {
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let (mut sim, _) = load(include_str!("fixtures/runtime_imu_future_gate.sdf"), engine);
        let peer = topic_id(&sim, "/p2/other");
        let target = topic_id(&sim, "/p2/imu");
        let entities = [target, peer].map(|id| entity_by_id(sim.world(), id).unwrap());
        set_imu_interest(&mut sim, entities);
        set_imu_rate(&mut sim, target, 20.0).unwrap();
        sim.step_exact(1).unwrap();
        set_imu_rate(&mut sim, target, 0.0).unwrap();
        set_imu_active(&mut sim, peer, false).unwrap();
        sim.step_exact(10).unwrap();
        assert_eq!(sample(&sim, target).0, Duration::from_millis(11));
        assert_eq!(sample(&sim, peer).0, Duration::from_millis(1));
        set_imu_interest(&mut sim, [entities[0]]);
        sim.step_exact(38).unwrap();
        assert_eq!(sample(&sim, target).0, Duration::from_millis(11));
        sim.step_exact(1).unwrap();
        assert_eq!(sample(&sim, target).0, Duration::from_millis(50));
    }
}

#[test]
fn inactive_acquisition_keeps_physics_velocity_history_current() {
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let (mut sim, id) = load(&fixture(1000), engine);
        sim.step_exact(1).unwrap();
        set_imu_active(&mut sim, id, false).unwrap();
        let link = named_id(&sim, "default::ball::body");
        queue_link_world_velocity(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
        sim.step_exact(20).unwrap();
        assert_eq!(sample(&sim, id).0, Duration::from_millis(1));
        set_imu_active(&mut sim, id, true).unwrap();
        sim.step_exact(1).unwrap();
        assert!(sample(&sim, id).1.specific_force[0].abs() < 1e-9);
    }
}

#[test]
fn inactive_acquisition_tracks_integrated_gravity_between_samples() {
    let (mut sim, id) = load(&fixture(1000), PhysicsEngine::Rapier);
    sim.step_exact(1).unwrap();
    let first = sample(&sim, id);
    assert!(first.1.specific_force[2].abs() < 1e-9);
    set_imu_active(&mut sim, id, false).unwrap();
    // Gravity changes velocity without the command path's explicit history update.
    sim.step_exact(20).unwrap();
    assert_eq!(sample(&sim, id), first);
    set_imu_active(&mut sim, id, true).unwrap();
    sim.step_exact(1).unwrap();
    let resumed = sample(&sim, id);
    assert_eq!(resumed.0, Duration::from_millis(22));
    assert!(resumed.1.specific_force[2].abs() < 1e-9);
}

#[test]
fn inactive_sensor_consumes_no_white_noise_and_keeps_its_stream() {
    let xml = fixture(1000).replace("<model name=\"ball\">", "<model name=\"ball\"><static>true</static>")
        .replace("</sensor>", "<imu><angular_velocity><x><noise type='gaussian'><stddev>0.2</stddev></noise></x></angular_velocity></imu></sensor>");
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let (mut a, id) = load(&xml, engine);
        let (mut b, other) = load(&xml, engine);
        a.step_exact(1).unwrap();
        b.step_exact(1).unwrap();
        let first = sample(&a, id).1;
        assert_eq!(first, sample(&b, other).1);
        set_imu_active(&mut a, id, false).unwrap();
        a.step_exact(99).unwrap();
        assert_eq!(sample(&a, id).1, first);
        set_imu_active(&mut a, id, true).unwrap();
        a.step_exact(1).unwrap();
        b.step_exact(1).unwrap();
        assert_eq!(sample(&a, id).1, sample(&b, other).1);
        assert_ne!(
            first.angular_velocity[0],
            sample(&b, other).1.angular_velocity[0]
        );
    }
}

#[test]
fn pause_removal_and_loaded_reconstruction_have_explicit_activation_state() {
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let (mut sim, id) = load(&fixture(1000), engine);
        set_imu_active(&mut sim, id, false).unwrap();
        sim.set_paused(true);
        sim.tick().unwrap();
        assert!(!imu_active(&sim, id).unwrap());
        assert_eq!(sim.elapsed(), Duration::ZERO);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(imu_active(&sim, id).unwrap());
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        assert_eq!(sample(&sim, id).0, Duration::from_millis(1));
        let model = named_id(&sim, "default::ball");
        remove_model(&mut sim, model).unwrap();
        assert!(
            matches!(set_imu_active(&mut sim, id, false), Err(ImuActivationError::MissingImu(value)) if value == id)
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(imu_active(&sim, id).unwrap());
    }
}

#[test]
fn invalid_identity_and_terminal_fault_leave_activation_unchanged() {
    let (mut sim, id) = load(&fixture(1000), PhysicsEngine::Tpe);
    let model = named_id(&sim, "default::ball");
    for missing in [0, u64::MAX, model] {
        assert!(
            matches!(imu_active(&sim, missing), Err(ImuActivationError::MissingImu(value)) if value == missing)
        );
        assert!(
            matches!(set_imu_active(&mut sim, missing, false), Err(ImuActivationError::MissingImu(value)) if value == missing)
        );
        assert!(imu_active(&sim, id).unwrap());
    }
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("activation fault"));
    assert!(matches!(
        set_imu_active(&mut sim, id, false),
        Err(ImuActivationError::Clock(_))
    ));
    assert!(imu_active(&sim, id).unwrap());
}
