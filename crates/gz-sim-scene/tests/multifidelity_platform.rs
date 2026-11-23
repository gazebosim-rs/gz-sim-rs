// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, LoadError, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const EXAMPLE: &str = include_str!("../../../examples/multifidelity_platform.sdf");
const PLATFORM: &str = "default::platform::body";
const CARGO: &str = "default::cargo::body";

fn load(source: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        source,
        "multifidelity_platform.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
}

fn committed(sim: &Simulation, name: &str) -> (u64, BodyId, BodyState) {
    let link = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .expect("loaded link");
    let pose = link.get::<WorldPose>().unwrap();
    let velocity = link.get::<Velocity>().unwrap();
    let handle = link.get::<BodyHandle>().unwrap().id();
    let state = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    // Both links have their center of mass at the body origin in this fixture.
    assert_eq!(state.position, pose.translation);
    assert_eq!(state.orientation, pose.rotation);
    assert_eq!(state.velocity, velocity.linear);
    assert_eq!(state.angular_velocity, velocity.angular);
    (link.get::<EntityId>().unwrap().get(), handle, state)
}

fn ray_above_deck(sim: &Simulation, x: f64) -> Option<f64> {
    // This line is clear of the cargo and intersects the deck only after it moves.
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray([x, 0.35, 1.0], [0.0, 0.0, -1.0], 2.0, None)
        .unwrap()
}

#[test]
fn moving_deck_transmits_friction_to_falling_cargo_and_updates_rays() {
    let mut simulation = load(EXAMPLE).unwrap();
    assert_eq!(ray_above_deck(&simulation, 1.25), None);
    simulation.step_exact(50).unwrap();
    let cargo = committed(&simulation, CARGO).2;
    assert!(cargo.position[0].abs() < 1e-12);
    assert!((0.32..0.34).contains(&cargo.position[2]));
    assert!(cargo.velocity[2] < -0.4);

    simulation.step_exact(450).unwrap();
    assert_eq!(simulation.elapsed(), Duration::from_millis(500));
    let platform = committed(&simulation, PLATFORM).2;
    assert!((platform.position[0] - 0.5).abs() < 1e-10);
    assert!((platform.position[2] - 0.1).abs() < 1e-12);
    assert!((platform.velocity[0] - 1.0).abs() < 1e-10);
    let cargo = committed(&simulation, CARGO).2;
    assert!((0.15..0.5).contains(&cargo.position[0]), "{cargo:?}");
    assert!((0.29..0.31).contains(&cargo.position[2]), "{cargo:?}");
    assert!((0.8..1.2).contains(&cargo.velocity[0]), "{cargo:?}");
    assert!((ray_above_deck(&simulation, 1.25).unwrap() - 0.8).abs() < 1e-10);
}

fn state_bits(state: BodyState) -> Vec<u64> {
    state
        .position
        .into_iter()
        .chain(state.orientation)
        .chain(state.velocity)
        .chain(state.angular_velocity)
        .map(f64::to_bits)
        .collect()
}

// Sample before, during, and after contact so stale contact caches also affect replay.
fn run_trace(simulation: &mut Simulation) -> Vec<[Vec<u64>; 2]> {
    [50, 45, 15, 390]
        .into_iter()
        .map(|steps| {
            simulation.step_exact(steps).unwrap();
            [PLATFORM, CARGO].map(|name| state_bits(committed(simulation, name).2))
        })
        .collect()
}

#[test]
fn pause_and_reset_preserve_initial_ids_and_replay_committed_motion() {
    let mut simulation = load(EXAMPLE).unwrap();
    let initial = [PLATFORM, CARGO].map(|name| committed(&simulation, name));
    let trace = run_trace(&mut simulation);
    simulation.set_paused(true);
    assert!(simulation.tick().unwrap().dt.is_zero());
    assert_eq!(simulation.elapsed(), Duration::from_millis(500));
    assert_eq!(
        *trace.last().unwrap(),
        [PLATFORM, CARGO].map(|name| state_bits(committed(&simulation, name).2)),
    );
    reset_loaded_world(&mut simulation, |_| ()).unwrap();
    assert_eq!(simulation.epoch(), 1);
    assert_eq!(simulation.elapsed(), Duration::ZERO);
    assert_eq!(ray_above_deck(&simulation, 1.25), None);
    assert_eq!(
        initial,
        [PLATFORM, CARGO].map(|name| committed(&simulation, name))
    );
    assert_eq!(run_trace(&mut simulation), trace);
    assert_eq!(run_trace(&mut load(EXAMPLE).unwrap()), trace);
}

#[test]
fn malformed_prescribed_motion_is_refused_during_load() {
    for (original, invalid) in [
        ("<angular_velocity>0 0 0</angular_velocity>", ""),
        (
            "<linear_velocity>1 0 0</linear_velocity>",
            "<linear_velocity>NaN 0 0</linear_velocity>",
        ),
        (
            "<model name=\"platform\">",
            "<model name=\"platform\"><static>true</static>",
        ),
    ] {
        let source = EXAMPLE.replace(original, invalid);
        assert_ne!(source, EXAMPLE);
        assert!(
            matches!(load(&source), Err(LoadError::Unsupported { .. })),
            "{invalid}"
        );
    }
}
