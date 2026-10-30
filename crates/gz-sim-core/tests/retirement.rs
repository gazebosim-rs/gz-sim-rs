// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation};
use std::{
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

#[derive(Resource, Default)]
struct Counts {
    created: Arc<AtomicUsize>,
    dropped: Arc<AtomicUsize>,
    calls: Vec<usize>,
}
struct OwnedState {
    dropped: Arc<AtomicUsize>,
    scratch: Vec<u8>,
    calls: usize,
}
impl FromWorld for OwnedState {
    fn from_world(world: &mut World) -> Self {
        let counts = world.resource::<Counts>();
        counts.created.fetch_add(1, Ordering::SeqCst);
        Self {
            dropped: Arc::clone(&counts.dropped),
            scratch: vec![0; 4096],
            calls: 0,
        }
    }
}
impl Drop for OwnedState {
    fn drop(&mut self) {
        self.dropped.fetch_add(1, Ordering::SeqCst);
    }
}
fn controller(mut state: Local<OwnedState>, mut counts: ResMut<Counts>) {
    state.calls += 1;
    state.scratch[0] = 1;
    counts.calls.push(state.calls);
}
fn simulation() -> Simulation {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    sim.world_mut().init_resource::<Counts>();
    sim
}
#[test]
fn retirement_drops_owned_state_stops_calls_and_reactivation_starts_fresh() {
    let mut sim = simulation();
    sim.step_exact(3).unwrap();
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .created
            .load(Ordering::SeqCst),
        0
    );
    sim.add_systems(Phase::PreStep, controller);
    sim.step_exact(2).unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1, 2]);
    assert_eq!(
        sim.remove_systems_in_set(Phase::PreStep, controller)
            .unwrap(),
        1
    );
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .dropped
            .load(Ordering::SeqCst),
        1
    );
    sim.step_exact(3).unwrap();
    let counts = sim.world().resource::<Counts>();
    assert_eq!(counts.created.load(Ordering::SeqCst), 1);
    assert_eq!(counts.dropped.load(Ordering::SeqCst), 1);
    assert_eq!(counts.calls, [1, 2]);
    sim.add_systems(Phase::PreStep, controller);
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1, 2, 1]);
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .created
            .load(Ordering::SeqCst),
        2
    );
}
#[test]
fn tick_and_reset_retirement_are_independent_and_unknown_sets_report_errors() {
    #[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
    struct Unknown;
    let mut sim = simulation();
    sim.add_systems(Phase::PreStep, controller);
    sim.add_systems(Phase::Reset, controller);
    sim.step_exact(1).unwrap();
    sim.reset().unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1, 1]);
    assert_eq!(
        sim.remove_systems_in_set(Phase::Reset, controller).unwrap(),
        1
    );
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1, 1, 2]);
    assert!(sim.remove_systems_in_set(Phase::PreStep, Unknown).is_err());
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1, 1, 2, 3]);
    assert_eq!(
        sim.remove_systems_in_set(Phase::PreStep, controller)
            .unwrap(),
        1
    );
    assert_eq!(
        sim.remove_systems_in_set(Phase::PreStep, controller)
            .unwrap(),
        0
    );
}
#[test]
fn retiring_one_world_does_not_change_another() {
    let mut a = simulation();
    let mut b = simulation();
    for sim in [&mut a, &mut b] {
        sim.add_systems(Phase::PreStep, controller);
    }
    a.step_exact(1).unwrap();
    b.step_exact(1).unwrap();
    a.remove_systems_in_set(Phase::PreStep, controller).unwrap();
    a.step_exact(1).unwrap();
    b.step_exact(1).unwrap();
    assert_eq!(a.world().resource::<Counts>().calls, [1]);
    assert_eq!(b.world().resource::<Counts>().calls, [1, 2]);
}

#[test]
fn reactivation_preserves_phase_order_and_unrelated_local_state() {
    #[derive(Resource, Default)]
    struct Trace(Vec<(&'static str, u64)>);
    fn before(mut trace: ResMut<Trace>, mut count: Local<u64>) {
        *count += 1;
        trace.0.push(("before", *count));
    }
    fn middle(mut trace: ResMut<Trace>, mut count: Local<u64>) {
        *count += 1;
        trace.0.push(("middle", *count));
    }
    fn after(mut trace: ResMut<Trace>, mut count: Local<u64>) {
        *count += 1;
        trace.0.push(("after", *count));
    }
    let mut sim = simulation();
    sim.world_mut().init_resource::<Trace>();
    sim.add_systems(Phase::PhysicsPrepare, before.before(middle));
    sim.add_systems(Phase::PhysicsPrepare, middle);
    sim.add_systems(Phase::PostStep, after);
    sim.step_exact(1).unwrap();
    sim.remove_systems_in_set(Phase::PhysicsPrepare, middle)
        .unwrap();
    sim.step_exact(1).unwrap();
    sim.add_systems(Phase::PhysicsPrepare, middle);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<Trace>().0,
        [
            ("before", 1),
            ("middle", 1),
            ("after", 1),
            ("before", 2),
            ("after", 2),
            ("before", 3),
            ("middle", 1),
            ("after", 3),
        ]
    );
}

#[test]
fn retirement_does_not_clear_a_terminal_runtime_fault() {
    use gz_sim_core::{StepError, StepFault};
    fn fail(fault: Res<StepFault>) {
        fault.report(std::io::Error::other("runtime"));
    }
    let mut sim = simulation();
    sim.add_systems(Phase::PreStep, fail);
    sim.add_systems(Phase::PostStep, controller);
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    sim.remove_systems_in_set(Phase::PostStep, controller)
        .unwrap();
    sim.remove_systems_in_set(Phase::PreStep, fail).unwrap();
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    assert!(sim.world().resource::<Counts>().calls.is_empty());
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .dropped
            .load(Ordering::SeqCst),
        1
    );
}

#[test]
fn retiring_systems_also_releases_their_conditions_without_idle_checks() {
    #[derive(Resource, Default)]
    struct ConditionCalls(AtomicUsize);
    fn ready(mut state: Local<OwnedState>, calls: Res<ConditionCalls>) -> bool {
        state.calls += 1;
        calls.0.fetch_add(1, Ordering::SeqCst);
        true
    }
    let mut sim = simulation();
    sim.world_mut().init_resource::<ConditionCalls>();
    sim.add_systems(Phase::PreStep, controller.run_if(ready));
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .created
            .load(Ordering::SeqCst),
        2
    );
    sim.remove_systems_in_set(Phase::PreStep, controller)
        .unwrap();
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .dropped
            .load(Ordering::SeqCst),
        2
    );
    sim.step_exact(10).unwrap();
    assert_eq!(
        sim.world()
            .resource::<ConditionCalls>()
            .0
            .load(Ordering::SeqCst),
        1
    );
    assert_eq!(sim.world().resource::<Counts>().calls, [1]);
}

#[test]
fn retirement_before_first_tick_initializes_and_releases_without_callbacks() {
    let mut sim = simulation();
    sim.add_systems(Phase::PreStep, controller);
    assert_eq!(
        sim.remove_systems_in_set(Phase::PreStep, controller)
            .unwrap(),
        1
    );
    let counts = sim.world().resource::<Counts>();
    assert_eq!(counts.created.load(Ordering::SeqCst), 1);
    assert_eq!(counts.dropped.load(Ordering::SeqCst), 1);
    assert!(counts.calls.is_empty());
    sim.step_exact(3).unwrap();
    assert!(sim.world().resource::<Counts>().calls.is_empty());
}

#[test]
fn invalid_pending_schedule_reports_a_build_error_without_running_callbacks() {
    fn other() {}
    let mut sim = simulation();
    sim.add_systems(
        Phase::PreStep,
        (controller.before(other), other.before(controller)),
    );
    assert!(
        sim.remove_systems_in_set(Phase::PreStep, controller)
            .is_err()
    );
    assert!(sim.world().resource::<Counts>().calls.is_empty());
}

#[test]
fn collective_condition_state_belongs_to_retained_ordering_sets() {
    fn other() {}
    fn ready(_state: Local<OwnedState>) -> bool {
        true
    }
    let mut sim = simulation();
    let dropped = Arc::clone(&sim.world().resource::<Counts>().dropped);
    sim.add_systems(Phase::PreStep, (controller, other).run_if(ready));
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .created
            .load(Ordering::SeqCst),
        2
    );
    sim.remove_systems_in_set(Phase::PreStep, controller)
        .unwrap();
    sim.remove_systems_in_set(Phase::PreStep, other).unwrap();
    assert_eq!(dropped.load(Ordering::SeqCst), 1);
    sim.step_exact(3).unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1]);
    drop(sim);
    assert_eq!(dropped.load(Ordering::SeqCst), 2);
}

#[test]
fn initialization_validates_both_schedules_without_running_callbacks() {
    fn extra(mut counts: ResMut<Counts>) {
        counts.calls.push(99);
    }
    let mut sim = simulation();
    assert_eq!(sim.scheduled_system_counts(), (0, 0));
    sim.add_systems(Phase::PreStep, controller);
    sim.add_systems(Phase::Reset, controller);
    assert_eq!(sim.scheduled_system_counts(), (1, 1));
    sim.initialize_schedules().unwrap();
    assert!(sim.world().resource::<Counts>().calls.is_empty());
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .created
            .load(Ordering::SeqCst),
        2
    );
    sim.add_systems(Phase::Publish, extra);
    assert_eq!(sim.scheduled_system_counts(), (2, 1));
    sim.initialize_schedules().unwrap();
    sim.step_exact(1).unwrap();
    sim.reset().unwrap();
    assert_eq!(sim.world().resource::<Counts>().calls, [1, 99, 1]);
    assert_eq!(
        sim.world()
            .resource::<Counts>()
            .created
            .load(Ordering::SeqCst),
        2
    );
}
