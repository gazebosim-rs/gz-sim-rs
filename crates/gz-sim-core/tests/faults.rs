// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, StepError, StepFault, UpdateInfo};
use std::time::Duration;

#[derive(Debug, thiserror::Error)]
#[error("fixture failure in {0:?}")]
struct FixtureFault(Phase);

#[derive(Resource, Default)]
struct Log(Vec<Phase>);

#[derive(Resource, Default)]
struct Trigger(Option<Phase>);

#[test]
fn every_normal_phase_stops_later_systems_and_keeps_last_committed_time() {
    let phases = [
        Phase::Ingress,
        Phase::Lifecycle,
        Phase::PreStep,
        Phase::PhysicsPrepare,
        Phase::PhysicsStep,
        Phase::PhysicsCommit,
        Phase::PostStep,
        Phase::Publish,
        Phase::Finalize,
    ];
    for (failed, target) in phases.into_iter().enumerate() {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        sim.world_mut().init_resource::<Log>();
        sim.world_mut().init_resource::<Trigger>();
        for phase in phases {
            sim.add_systems(
                phase,
                (
                    move |mut log: ResMut<Log>| log.0.push(phase),
                    move |trigger: Res<Trigger>, fault: Res<StepFault>| {
                        if trigger.0 == Some(phase) {
                            fault.report(FixtureFault(phase));
                            fault.report(std::io::Error::other("later diagnostic"));
                        }
                    },
                )
                    .chain(),
            );
        }
        sim.set_paused(false);
        sim.tick().unwrap();
        assert_eq!(sim.world().resource::<Log>().0, phases);
        let committed = *sim.world().resource::<UpdateInfo>();
        sim.world_mut().resource_mut::<Log>().0.clear();
        sim.world_mut().resource_mut::<Trigger>().0 = Some(target);
        assert_eq!(sim.tick(), Err(StepError::Faulted));
        assert_eq!(sim.world().resource::<Log>().0, phases[..=failed]);
        assert_eq!(sim.elapsed(), Duration::from_millis(1));
        assert_eq!(*sim.world().resource::<UpdateInfo>(), committed);
        assert_eq!(
            sim.fault()
                .unwrap()
                .downcast_ref::<FixtureFault>()
                .unwrap()
                .0,
            target
        );
        assert!(sim.fault().unwrap().to_string().contains("fixture failure"));
        sim.world_mut().insert_resource(StepFault::default());
        assert!(sim.fault().is_none());
        assert_eq!(sim.tick(), Err(StepError::Faulted));
        assert_eq!(sim.step_exact(0), Err(StepError::Faulted));
        assert_eq!(sim.reset(), Err(StepError::Faulted));
        assert_eq!(sim.validate_steps(0), Err(StepError::Faulted));
        assert_eq!(sim.validate_reset(), Err(StepError::Faulted));
        assert_eq!(*sim.world().resource::<UpdateInfo>(), committed);
        assert_eq!(sim.world().resource::<Log>().0, phases[..=failed]);
    }
    assert!(StepError::Faulted.to_string().contains("failed"));
}

#[test]
fn a_pre_reported_fault_rejects_ticks_zero_steps_and_reset_before_systems() {
    for operation in 0..3 {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        sim.add_systems(Phase::Ingress, || panic!("no step system may run"));
        sim.add_systems(Phase::Reset, || panic!("no reset system may run"));
        assert!(sim.fault().is_none());
        let committed = *sim.world().resource::<UpdateInfo>();
        sim.world()
            .resource::<StepFault>()
            .report(FixtureFault(Phase::Ingress));
        assert_eq!(sim.validate_steps(0), Err(StepError::Faulted));
        assert_eq!(sim.validate_reset(), Err(StepError::Faulted));
        let result = match operation {
            0 => sim.tick().map(|_| ()),
            1 => sim.step_exact(0),
            _ => sim.reset(),
        };
        assert_eq!(result, Err(StepError::Faulted));
        sim.world_mut().insert_resource(StepFault::default());
        assert_eq!(sim.tick(), Err(StepError::Faulted));
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(*sim.world().resource::<UpdateInfo>(), committed);
    }
}

#[test]
fn reset_callback_failure_is_terminal_after_reset_time_and_mutations_commit() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    sim.world_mut().init_resource::<Log>();
    sim.step_exact(2).unwrap();
    sim.add_systems(
        Phase::Reset,
        (
            |mut log: ResMut<Log>, fault: Res<StepFault>| {
                log.0.push(Phase::Reset);
                fault.report(FixtureFault(Phase::Reset));
            },
            || panic!("later reset callback must be skipped"),
        )
            .chain(),
    );
    assert_eq!(sim.reset(), Err(StepError::Faulted));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    let committed = *sim.world().resource::<UpdateInfo>();
    assert_eq!(
        (committed.epoch, committed.iteration, committed.sim_time),
        (1, 0, Duration::ZERO)
    );
    assert_eq!(sim.world().resource::<Log>().0, [Phase::Reset]);
    assert_eq!(
        sim.fault()
            .unwrap()
            .downcast_ref::<FixtureFault>()
            .unwrap()
            .0,
        Phase::Reset
    );
    assert_eq!(sim.reset(), Err(StepError::Faulted));
    assert_eq!(*sim.world().resource::<UpdateInfo>(), committed);
    assert_eq!(sim.tick(), Err(StepError::Faulted));
}
