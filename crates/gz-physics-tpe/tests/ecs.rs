// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::BodyState;
use gz_physics_rapier::{PhysicsError, RapierBackend};
use gz_physics_tpe::{InstallError, Motion, MotionError, MotionReset, StepFailure, install};
use gz_sim_core::{Phase, Simulation, StepError};
use std::time::Duration;

fn state() -> BodyState {
    BodyState {
        position: [1.0, 2.0, 3.0],
        velocity: [2.0, -3.0, 4.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.0, 0.0, 2.0],
    }
}

#[test]
fn bevy_bodies_step_pause_reset_and_retire() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    install(&mut sim).unwrap();
    #[derive(Resource, Default)]
    struct ResetObservation(Option<BodyState>);
    sim.world_mut().init_resource::<ResetObservation>();
    sim.add_systems(
        Phase::Reset,
        (|bodies: Query<&Motion>, mut observed: ResMut<ResetObservation>| {
            observed.0 = Some(*bodies.single().unwrap().state());
        })
        .after(MotionReset),
    );
    let initial = state();
    let entity = sim.world_mut().spawn(Motion::new(initial).unwrap()).id();
    let removed = sim.world_mut().spawn(Motion::new(initial).unwrap()).id();
    sim.tick().unwrap();
    assert_eq!(*sim.world().get::<Motion>(entity).unwrap().state(), initial);
    sim.step_exact(50).unwrap();
    let motion = sim.world().get::<Motion>(entity).unwrap().state();
    for (actual, expected) in motion.position.iter().zip([2.0, 0.5, 5.0]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    let expected_rotation = glam::DQuat::from_rotation_z(1.0).to_array();
    for (actual, expected) in motion.orientation.iter().zip(expected_rotation) {
        assert!((actual - expected).abs() < 1e-12);
    }
    assert_eq!(motion.velocity, initial.velocity);
    assert_eq!(motion.angular_velocity, initial.angular_velocity);
    let before_pause = *motion;
    sim.tick().unwrap();
    assert_eq!(
        *sim.world().get::<Motion>(entity).unwrap().state(),
        before_pause
    );
    sim.world_mut().entity_mut(removed).despawn();
    sim.reset().unwrap();
    assert_eq!(*sim.world().get::<Motion>(entity).unwrap().state(), initial);
    assert!(sim.world().get_entity(removed).is_err());
    assert_eq!(sim.world().resource::<ResetObservation>().0, Some(initial));
    sim.step_exact(1).unwrap();
    assert!((sim.world().get::<Motion>(entity).unwrap().state().position[0] - 1.02).abs() < 1e-12);
    sim.world_mut().entity_mut(entity).remove::<Motion>();
    sim.step_exact(1).unwrap();
    assert!(sim.world().get::<Motion>(entity).is_none());
}

#[test]
fn installed_engines_are_exclusive_even_after_resource_removal() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim).unwrap();
    assert_eq!(install(&mut sim), Err(InstallError::AlreadyInstalled));
    assert_eq!(
        gz_physics_rapier::install(&mut sim, RapierBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
    assert!(!sim.world().contains_resource::<RapierBackend>());
    let mut other = Simulation::new(Duration::from_millis(1)).unwrap();
    gz_physics_rapier::install(&mut other, RapierBackend::new([0.0; 3]).unwrap()).unwrap();
    assert_eq!(install(&mut other), Err(InstallError::AlreadyInstalled));
    other.world_mut().remove_resource::<RapierBackend>();
    assert_eq!(install(&mut other), Err(InstallError::AlreadyInstalled));
    assert_eq!(
        gz_physics_rapier::install(&mut other, RapierBackend::new([0.0; 3]).unwrap()),
        Err(PhysicsError::AlreadyInstalled)
    );
}

#[test]
fn invalid_construction_and_step_overflow_keep_typed_diagnostics() {
    let mut invalid = state();
    invalid.orientation = [0.0; 4];
    assert!(matches!(
        Motion::new(invalid),
        Err(MotionError::InvalidState)
    ));
    let mut sim = Simulation::new(Duration::from_secs(1)).unwrap();
    install(&mut sim).unwrap();
    let mut large = state();
    large.position[0] = f64::MAX;
    large.velocity[0] = f64::MAX;
    let entity = sim.world_mut().spawn(Motion::new(large).unwrap()).id();
    #[derive(Resource, Default)]
    struct Published(usize);
    sim.world_mut().init_resource::<Published>();
    sim.add_systems(Phase::Publish, |mut count: ResMut<Published>| count.0 += 1);
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.world().resource::<Published>().0, 0);
    assert_eq!(*sim.world().get::<Motion>(entity).unwrap().state(), large);
    let failure = sim.fault().unwrap().downcast_ref::<StepFailure>().unwrap();
    assert_eq!(failure.entity, entity);
    assert_eq!(failure.source, MotionError::Overflow);
    assert!(failure.to_string().contains("TPE motion failed"));
    assert!(std::error::Error::source(failure).is_some());
    assert_eq!(sim.reset(), Err(StepError::Faulted));
}

#[test]
fn checked_world_velocity_updates_current_motion_and_preserves_initial_reset() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    install(&mut sim).unwrap();
    let initial = state();
    let entity = sim.world_mut().spawn(Motion::new(initial).unwrap()).id();
    {
        let mut motion = sim.world_mut().get_mut::<Motion>(entity).unwrap();
        motion
            .set_world_velocity([1.0, 0.0, 0.0], [0.0; 3])
            .unwrap();
        let changed = *motion.state();
        for (linear, angular) in [([f64::NAN; 3], [0.0; 3]), ([9.0; 3], [f64::INFINITY; 3])] {
            assert_eq!(
                motion.set_world_velocity(linear, angular),
                Err(MotionError::InvalidState)
            );
            assert_eq!(*motion.state(), changed);
        }
    }
    sim.tick().unwrap();
    assert_eq!(
        sim.world().get::<Motion>(entity).unwrap().state().position,
        initial.position
    );
    sim.step_exact(1).unwrap();
    let changed = sim.world().get::<Motion>(entity).unwrap().state();
    assert!((changed.position[0] - 1.01).abs() < 1e-12);
    assert_eq!(changed.position[1..], initial.position[1..]);
    assert_eq!(changed.orientation, initial.orientation);
    sim.reset().unwrap();
    assert_eq!(*sim.world().get::<Motion>(entity).unwrap().state(), initial);
}
