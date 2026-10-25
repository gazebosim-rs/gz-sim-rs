// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::{
    prelude::*,
    schedule::{MultiThreadedExecutor, SingleThreadedExecutor},
};
use gz_sim_core::{Phase, Simulation, StepError, StepFault, UpdateInfo};
use std::time::Duration;

#[derive(Resource, Default)]
struct Observed(Vec<u64>);
#[derive(Component)]
struct Created;

#[test]
fn changing_executor_preserves_local_state_deferred_visibility_and_time() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    sim.world_mut().init_resource::<Observed>();
    sim.add_systems(Phase::PreStep, |mut commands: Commands| {
        commands.spawn(Created);
    });
    sim.add_systems(
        Phase::PhysicsStep,
        |created: Query<&Created>,
         time: Res<UpdateInfo>,
         mut local: Local<u64>,
         mut observed: ResMut<Observed>| {
            *local += 1;
            assert_eq!(created.iter().count(), *local as usize);
            assert_eq!(time.iteration, *local);
            observed.0.push(*local);
        },
    );
    sim.step_exact(1).unwrap();
    sim.set_step_executor(MultiThreadedExecutor::new());
    sim.step_exact(1).unwrap();
    sim.set_step_executor(SingleThreadedExecutor::new());
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Observed>().0, [1, 2, 3]);
    assert_eq!(sim.elapsed(), Duration::from_millis(3));
}

#[test]
fn both_executors_stop_before_physics_and_publication_on_a_prepare_fault() {
    for parallel in [false, true] {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        if parallel {
            sim.set_step_executor(MultiThreadedExecutor::new());
        }
        sim.world_mut().init_resource::<Observed>();
        sim.add_systems(Phase::PhysicsPrepare, |fault: Res<StepFault>| {
            fault.report(std::io::Error::other("prepare failed"));
        });
        sim.add_systems(Phase::PhysicsStep, |mut observed: ResMut<Observed>| {
            observed.0.push(1);
        });
        sim.add_systems(Phase::Publish, |mut observed: ResMut<Observed>| {
            observed.0.push(2);
        });
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
        assert!(sim.world().resource::<Observed>().0.is_empty());
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}
