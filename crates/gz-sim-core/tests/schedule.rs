use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, StepError, UpdateInfo};
use std::time::Duration;

#[derive(Resource, Default)]
struct Observations(Vec<(Phase, UpdateInfo)>);

#[derive(Component)]
struct Created;

#[test]
fn step_time_is_visible_in_every_ordered_phase() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    sim.world_mut().init_resource::<Observations>();
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
    for phase in phases.into_iter().rev() {
        sim.add_systems(
            phase,
            move |time: Res<UpdateInfo>, mut seen: ResMut<Observations>| {
                seen.0.push((phase, *time));
            },
        );
    }
    sim.step_exact(2).unwrap();
    let seen = &sim.world().resource::<Observations>().0;
    assert_eq!(seen.len(), 18);
    for (index, (phase, time)) in seen.iter().enumerate() {
        assert_eq!(*phase, phases[index % 9]);
        assert_eq!(time.iteration, (index / 9 + 1) as u64);
        assert_eq!(time.sim_time, Duration::from_millis(time.iteration));
        assert_eq!(time.start_time + time.dt, time.sim_time);
        assert!(!time.paused);
    }
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert!(sim.is_paused());
}

#[test]
fn deferred_lifecycle_changes_are_visible_to_controllers() {
    let mut sim = Simulation::new(Duration::from_secs(1)).unwrap();
    sim.add_systems(Phase::Lifecycle, |mut commands: Commands| {
        commands.spawn(Created);
    });
    sim.add_systems(Phase::PreStep, |entities: Query<&Created>| {
        assert_eq!(entities.iter().count(), 1);
    });
    sim.tick().unwrap();
}

#[test]
fn pause_exact_steps_and_reset_keep_distinct_clock_contracts() {
    let mut sim = Simulation::new(Duration::from_nanos(7)).unwrap();
    let paused = sim.tick().unwrap();
    assert_eq!(paused.sim_time, Duration::ZERO);
    assert_eq!(paused.iteration, 0);
    assert_eq!(paused.dt, Duration::ZERO);
    assert!(paused.paused);
    sim.step_exact(3).unwrap();
    sim.step_exact(0).unwrap();
    assert_eq!(sim.elapsed(), Duration::from_nanos(21));
    sim.set_paused(false);
    assert!(!sim.is_paused());
    assert_eq!(sim.tick().unwrap().sim_time, Duration::from_nanos(28));
    sim.add_systems(
        Phase::Reset,
        |time: Res<UpdateInfo>, mut commands: Commands| {
            assert_eq!(time.sim_time, Duration::ZERO);
            assert_eq!(time.iteration, 0);
            assert_eq!(time.epoch, 1);
            commands.spawn(Created);
        },
    );
    sim.reset().unwrap();
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(
        sim.world_mut()
            .query::<&Created>()
            .iter(sim.world())
            .count(),
        1
    );
    assert_eq!(sim.tick().unwrap().epoch, 1);
}

#[test]
fn worlds_advance_independently() {
    let mut left = Simulation::new(Duration::from_secs(1)).unwrap();
    let right = Simulation::new(Duration::from_secs(2)).unwrap();
    left.step_exact(4).unwrap();
    assert_eq!(right.elapsed(), Duration::ZERO);
    assert_eq!(left.elapsed(), Duration::from_secs(4));
}

#[test]
fn invalid_duration_and_overflow_fail_before_running_systems() {
    assert!(matches!(
        Simulation::new(Duration::ZERO),
        Err(StepError::ZeroDuration)
    ));
    assert_eq!(
        StepError::ZeroDuration.to_string(),
        "step duration must be positive"
    );
    assert!(StepError::Overflow.to_string().contains("overflow"));
    let mut sim = Simulation::new(Duration::MAX).unwrap();
    sim.add_systems(Phase::PreStep, || {
        panic!("overflow must be caught before scheduling")
    });
    assert_eq!(sim.step_exact(2), Err(StepError::Overflow));
    assert_eq!(sim.step_exact(u64::MAX), Err(StepError::Overflow));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    let mut sim = Simulation::new(Duration::MAX).unwrap();
    sim.step_exact(1).unwrap();
    sim.set_paused(false);
    assert_eq!(sim.tick(), Err(StepError::Overflow));
}
