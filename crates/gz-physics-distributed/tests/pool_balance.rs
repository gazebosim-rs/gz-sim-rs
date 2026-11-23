// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Error,
    physics::{SimulationStep, fixture},
    pool::{PoolBalancePolicy, PoolLoadMode, WorkerPool},
};
use std::{num::NonZeroU16, path::Path, time::Duration};
type Test = Result<(), Box<dyn std::error::Error>>;
fn setup(limit: usize) -> WorkerPool {
    let mut group = vec![];
    for i in 0..8 {
        let mut b = fixture()[1].clone();
        b.id = 100 + i;
        b.position = [5. * i as f64, 0., 2.];
        b.linear = [0.; 3];
        b.angular = [0.; 3];
        group.push(b);
    }
    WorkerPool::spawn(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        7676,
        vec![group, vec![], vec![]],
        SimulationStep::Ms10,
        None,
        limit,
    )
    .unwrap()
}
fn policy(cost_ms: u64) -> PoolBalancePolicy {
    PoolBalancePolicy {
        horizon: NonZeroU16::new(100).unwrap(),
        handoff_cost: Duration::from_millis(cost_ms),
    }
}
#[test]
fn measured_decision_uses_budget_capacity_and_the_committed_window() -> Test {
    for limit in [2, 16] {
        let mut pool = setup(limit);
        pool.set_local_helper_limit(Some(0));
        let sample = pool.start_load_sample(NonZeroU16::new(4).unwrap())?;
        assert!(pool.finish_load_sample(sample.clone()).is_err());
        for _ in 0..4 {
            pool.advance()?;
        }
        let measured = pool.finish_load_sample(sample)?;
        assert_eq!(measured.measured_steps().get(), 4);
        assert_eq!(measured.workers().len(), 3);
        assert_eq!(measured.workers()[0].len(), 8);
        assert!(measured.workers()[0].iter().all(|r| r.elapsed_ns > 0));
        assert!(
            pool.plan_rebalance(&measured, policy(60000), [0.; 3], &[])?
                .is_none()
        );
        let decision = pool.plan_rebalance(&measured, policy(0), [0.; 3], &[])?;
        if limit >= 3 && std::thread::available_parallelism()?.get() > 1 {
            let d = decision.ok_or("expected a load-reducing transfer")?;
            assert_eq!(
                (d.source, d.destination, d.estimated_payback_ticks),
                (0, 1, 0)
            );
            assert!(d.before_lower_ns > d.after_serial_peak_ns);
            let id = d.members[0];
            pool.transfer_component(id, d.destination)?;
            assert_eq!(pool.frame().body(id).unwrap().0, 1);
            assert!(
                pool.plan_rebalance(&measured, policy(0), [0.; 3], &[])
                    .is_err()
            );
            pool.recover(1)?;
            assert_eq!(pool.frame().workers[1].generations[&id], 1);
        } else {
            assert!(decision.is_none());
        }
    }
    Ok(())
}
#[test]
fn scheduled_probe_keeps_membership_and_survives_a_retried_final_reply() -> Test {
    let mut pool = setup(16);
    let mut control = setup(16);
    let span = NonZeroU16::new(4).unwrap();
    let window = pool.start_load_sample_with_mode(span, PoolLoadMode::Scheduled)?;
    assert_eq!(window.scheduled_worker_mask(), 0b111);
    assert!(pool.start_load_sample(span).is_err());
    for tick in 1..=4 {
        if tick == 3 {
            pool.drop_next_acknowledgement(0)?;
            assert!(pool.advance().is_err());
        }
        pool.advance()?;
        control.advance()?;
        assert_eq!(pool.frame(), control.frame());
    }
    let sample = pool.finish_load_sample(window)?;
    assert_eq!(sample.scheduled_worker_mask(), 0b111);
    assert!(sample.workers()[0].iter().all(|row| row.elapsed_ns > 0));
    assert!(sample.workers()[1..].iter().all(Vec::is_empty));
    Ok(())
}

#[test]
fn superseding_a_window_cannot_label_scheduled_cost_as_an_older_serial_sample() -> Test {
    let mut pool = setup(16);
    let span = NonZeroU16::new(4).unwrap();
    let serial = pool.start_load_sample(span)?;
    pool.start_load_sample(NonZeroU16::new(2).unwrap())?;
    let concurrent = pool.start_load_sample_with_mode(span, PoolLoadMode::Scheduled)?;
    for _ in 0..4 {
        pool.advance()?;
    }
    assert!(pool.finish_load_sample(serial).is_err());
    assert_eq!(
        pool.finish_load_sample(concurrent)?.scheduled_worker_mask(),
        7
    );
    Ok(())
}

#[test]
fn stale_probe_cannot_authorize_a_handoff_after_replacement_input_restore_or_tick() -> Test {
    for action in 0..4 {
        let mut pool = setup(64);
        let sample = pool.start_load_sample(NonZeroU16::new(2).unwrap())?;
        pool.advance()?;
        match action {
            0 => pool.recover(2)?,
            1 => {
                pool.queue_impulse(101, [0.1, 0., 0.])?;
            }
            2 => {
                let dir = tempfile::tempdir()?;
                let file = dir.path().join("window.gzpool");
                pool.save_native_recovery(&file)?;
                drop(pool);
                pool = gz_physics_distributed::pool::PoolNativeRecovery::open(file)?
                    .resume(Path::new(env!("CARGO_BIN_EXE_physics_worker")))?;
            }
            _ => {}
        }
        pool.advance()?;
        if action == 3 {
            let measured = pool.finish_load_sample(sample)?;
            pool.advance()?;
            assert!(
                pool.plan_rebalance(&measured, policy(0), [0.; 3], &[])
                    .is_err()
            );
        } else {
            assert!(matches!(
                pool.finish_load_sample(sample),
                Err(Error::Invalid(_))
            ));
        }
    }
    Ok(())
}
#[test]
fn interrupted_step_cannot_start_a_probe_but_confirming_its_sequence_preserves_measurement() -> Test
{
    let mut p = setup(8);
    let window = p.start_load_sample(NonZeroU16::new(2).unwrap())?;
    p.advance()?;
    p.drop_next_acknowledgement(1)?;
    assert!(p.advance().is_err());
    assert!(matches!(
        p.start_load_sample(NonZeroU16::new(1).unwrap()),
        Err(Error::Pending)
    ));
    assert!(p.finish_load_sample(window.clone()).is_err());
    p.advance()?;
    assert_eq!(p.finish_load_sample(window)?.workers()[0].len(), 8);
    Ok(())
}

#[test]
fn bevy_scene_can_apply_a_measured_handoff_at_a_fixed_tick_and_reset_its_budget() -> Test {
    use gz_physics::PhysicsBackend;
    use gz_physics_distributed::SceneBackend;
    use gz_sim_core::Simulation;
    use gz_sim_scene::{
        PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
        reset_loaded_world,
    };
    use sdformat::NoResources;
    let make = || {
        load_world_with_runtime(
            include_str!("../../../examples/distributed_cargo_pads.sdf"),
            "cargo.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::DistributedIslands(4),
                step: None,
                noise_seed: [0; 32],
                distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
            },
            WorldRuntimeOptions {
                distributed_local_helper_limit: Some(0),
                ..Default::default()
            },
        )
    };
    fn states(sim: &Simulation) -> (u64, Vec<u8>) {
        let frame = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap()
            .pool_frame()
            .unwrap();
        let mut bodies: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.bodies)
            .collect();
        let mut pairs: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .collect();
        let mut floors: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .collect();
        bodies.sort_by_key(|b| b.id);
        pairs.sort_by_key(|p| (p.a, p.b));
        floors.sort_by_key(|p| p.body);
        (
            frame.tick,
            bincode::serialize(&(bodies, pairs, floors)).unwrap(),
        )
    }
    let mut actual = make()?;
    let mut control = make()?;
    actual.step_exact(70)?;
    let window = actual
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .start_pool_load_sample(NonZeroU16::new(8).unwrap())?;
    actual.step_exact(8)?;
    let before = states(&actual);
    let decision = actual
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .rebalance_sampled_island(window, policy(0))?;
    assert_eq!(states(&actual), before);
    assert_eq!(actual.elapsed(), Duration::from_millis(780));
    if std::thread::available_parallelism()?.get() >= 4 {
        let d = decision.ok_or("imbalanced scene produced no candidate")?;
        let backend = actual
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap();
        assert!(
            d.members
                .iter()
                .all(|id| backend.pool_frame().unwrap().body(*id).unwrap().0 == d.destination)
        );
    }
    control.step_exact(78)?;
    for _ in 0..10 {
        actual.step_exact(1)?;
        control.step_exact(1)?;
        assert_eq!(states(&actual), states(&control));
    }
    reset_loaded_world(&mut actual, |_| ())?;
    actual.step_exact(3)?;
    let backend = actual
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    assert_eq!(backend.last_local_helper_mask(), Some(0));
    Ok(())
}

#[test]
fn periodic_move_and_final_step_commit_once_after_lost_ack_and_recipient_death() -> Test {
    let mut pool = setup(32);
    let mut control = setup(32);
    pool.set_local_helper_limit(Some(0));
    pool.set_handoff_confirmation(true);
    pool.set_periodic_placement(Some("4,8,24,0".parse()?))?;
    for _ in 0..4 {
        pool.advance()?;
        control.advance()?;
    }
    let before = pool.frame().clone();
    pool.drop_next_acknowledgement(2)?;
    assert!(pool.advance().is_err());
    assert_eq!(pool.frame(), &before);
    assert_eq!(pool.placement_status().unwrap().committed_moves, 0);
    assert!(pool.set_periodic_placement(None).is_err());
    let receiver = pool.worker_pid(1).unwrap();
    pool.terminate_worker(1)?;
    pool.advance()?;
    assert_ne!(pool.worker_pid(1), Some(receiver));
    control.advance()?;
    let stats = pool.placement_status().unwrap();
    if std::thread::available_parallelism()?.get() > 1 {
        assert_eq!(stats.committed_moves, 1);
        let (tick, decision) = stats.last_move.as_ref().unwrap();
        assert_eq!((*tick, decision.source, decision.destination), (5, 0, 1));
        for id in &decision.members {
            assert_eq!(pool.frame().workers[1].generations[id], 1);
        }
    }
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("committed.gzpool");
    pool.save_native_recovery(&file)?;
    let mut restored = gz_physics_distributed::pool::PoolNativeRecovery::open(&file)?
        .resume(Path::new(env!("CARGO_BIN_EXE_physics_worker")))?;
    assert_eq!(restored.frame(), pool.frame());
    assert!(restored.placement_status().is_none());
    for _ in 0..10 {
        restored.advance()?;
        control.advance()?;
        for id in 100..108 {
            assert_eq!(
                restored.frame().body(id).unwrap().1,
                control.frame().body(id).unwrap().1
            );
        }
    }
    Ok(())
}

#[test]
fn invalidated_timing_never_moves_an_owner_and_the_next_sample_recovers() -> Test {
    let mut p = setup(16);
    p.set_local_helper_limit(Some(0));
    p.set_periodic_placement(Some("2,4,24,0".parse()?))?;
    p.advance()?;
    p.recover(0)?;
    p.advance()?;
    p.advance()?;
    let status = p.placement_status().unwrap();
    let parallel = std::thread::available_parallelism()?.get() > 1;
    assert_eq!(
        (status.windows_interrupted, status.committed_moves),
        (u64::from(parallel), 0)
    );
    for _ in 0..4 {
        p.advance()?;
    }
    assert_eq!(
        p.placement_status().unwrap().windows_completed,
        u64::from(parallel)
    );
    if std::thread::available_parallelism()?.get() > 1 {
        assert_eq!(p.placement_status().unwrap().committed_moves, 1);
    }
    for config in [
        "0,4,1,1",
        "4,2,1,1",
        "2,4,0,1",
        "2,4,2",
        "2,4,2,10,0",
        "2,4,1,-1",
    ] {
        assert!(
            config
                .parse::<gz_physics_distributed::pool::PeriodicPlacement>()
                .is_err()
        );
    }
    // A three-entry source journal can fit a separate explicit transfer but
    // cannot stage its three migration commands plus this step.
    let mut limited = setup(3);
    limited.set_periodic_placement(Some("2,4,24,0".parse()?))?;
    for _ in 0..12 {
        limited.advance()?;
    }
    assert!(
        limited.placement_status().unwrap().windows_completed
            + limited.placement_status().unwrap().windows_ineligible
            >= 2
    );
    assert_eq!(limited.placement_status().unwrap().committed_moves, 0);
    Ok(())
}

#[test]
fn background_sampling_defers_to_explicit_probe_across_periods() -> Test {
    let mut pool = setup(16);
    pool.set_periodic_placement(Some("2,2,24,0".parse()?))?;
    let manual = pool.start_load_sample(NonZeroU16::new(4).unwrap())?;
    for _ in 0..4 {
        pool.advance()?;
    }
    assert_eq!(pool.placement_status().unwrap().windows_started, 0);
    assert_eq!(pool.finish_load_sample(manual)?.measured_steps().get(), 4);
    pool.advance()?;
    assert_eq!(
        pool.placement_status().unwrap().windows_started,
        u64::from(std::thread::available_parallelism()?.get() > 1)
    );
    Ok(())
}

#[test]
fn indivisible_loads_skip_sampling_and_reconsider_after_manual_colocation() -> Test {
    let mut workers = Vec::new();
    for owner in 0..3 {
        let mut b = fixture()[1].clone();
        b.id = 121 + owner;
        b.position = [20. * owner as f64, 0., 2.];
        b.linear = [0.; 3];
        b.angular = [0.; 3];
        workers.push(vec![b]);
    }
    let mut pool = WorkerPool::spawn(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        1471,
        workers,
        SimulationStep::Ms10,
        None,
        32,
    )?;
    pool.set_periodic_placement(Some("2,4,24,0".parse()?))?;
    for _ in 0..4 {
        pool.advance()?;
    }
    assert_eq!(pool.placement_status().unwrap().windows_ineligible, 1);
    assert_eq!(pool.placement_status().unwrap().windows_started, 0);
    pool.transfer_component(122, 0)?;
    for _ in 0..4 {
        pool.advance()?;
    }
    let status = pool.placement_status().unwrap();
    if std::thread::available_parallelism()?.get() > 1 {
        assert_eq!((status.windows_completed, status.committed_moves), (1, 1));
    } else {
        assert_eq!(status.windows_ineligible, 2);
    }
    Ok(())
}

#[test]
fn unprofitable_identical_work_is_rechecked_on_deadline_or_session_change() -> Test {
    let parallel = std::thread::available_parallelism()?.get() > 1;
    let mut pool = setup(128);
    pool.set_local_helper_limit(Some(0));
    pool.set_periodic_placement(Some("2,4,1,4294967295,4".parse()?))?;
    for _ in 0..16 {
        pool.advance()?;
    }
    let status = pool.placement_status().unwrap();
    if parallel {
        // Starts 0 and 8, skips 4 and 12. Next unchanged sample is due at 24.
        assert_eq!(
            (
                status.windows_started,
                status.windows_completed,
                status.windows_deferred
            ),
            (2, 2, 2)
        );
    }
    pool.recover(2)?;
    for _ in 0..3 {
        pool.advance()?;
    }
    if parallel {
        assert_eq!(pool.placement_status().unwrap().windows_completed, 3);
        assert_eq!(pool.placement_status().unwrap().windows_woken, 1);
    }
    for _ in 0..4 {
        pool.advance()?;
    }
    if parallel {
        assert_eq!(pool.placement_status().unwrap().windows_deferred, 3);
    }
    assert_eq!(pool.placement_status().unwrap().committed_moves, 0);
    Ok(())
}

#[test]
fn dead_sampler_peer_is_recovered_by_the_physics_barrier() -> Test {
    for target in [0, 2] {
        let mut pool = setup(32);
        let mut control = setup(32);
        pool.set_local_helper_limit(Some(0));
        pool.set_periodic_placement(Some("2,4,1,4294967295".parse()?))?;
        // Owner 0 has every native world. Worker 2 still participates in
        // each authority barrier and its incarnation fences an open window.
        for tick in 0..9 {
            if tick == 0 || tick == 6 {
                let original = pool.frame().clone();
                let pids: Vec<_> = (0..3).map(|i| pool.worker_pid(i)).collect();
                pool.terminate_worker(target)?;
                assert!(pool.advance().is_err());
                assert_eq!(pool.frame(), &original);
                pool.advance()?;
                for (i, old) in pids.into_iter().enumerate() {
                    assert_eq!(pool.worker_pid(i as u8) != old, i == usize::from(target));
                }
            } else {
                pool.advance()?;
            }
            control.advance()?;
            assert_eq!(pool.frame(), control.frame());
        }
        let status = pool.placement_status().unwrap();
        if std::thread::available_parallelism()?.get() > 1 {
            assert_eq!(status.unavailable_probes, if target == 0 { 2 } else { 0 });
            assert_eq!(status.windows_started, if target == 0 { 2 } else { 3 });
            if target == 2 {
                assert_eq!(status.windows_interrupted, 1);
            }
        }
        assert_eq!(status.committed_moves, 0);
    }
    Ok(())
}

#[test]
fn mandatory_impact_routing_wins_at_the_end_of_a_profitable_timing_window() -> Test {
    let make = || -> Result<WorkerPool, Error> {
        let bodies: Vec<_> = (0..8)
            .map(|i| {
                let mut b = fixture()[1].clone();
                b.id = 51 + i;
                b.primitive = gz_physics_distributed::physics::Primitive::Sphere;
                b.size = [0.6; 3];
                b.mass = 1.;
                b.position = [if i == 1 { 1.6 } else { i as f64 * 8. }, 0., 2.];
                b.linear = [
                    match i {
                        0 => 5.,
                        1 => -5.,
                        _ => 0.,
                    },
                    0.,
                    0.,
                ];
                b.angular = [0.; 3];
                b
            })
            .collect();
        let mut workers = vec![bodies, vec![], vec![]];
        let incoming = workers[0].remove(1);
        workers[1].push(incoming);
        WorkerPool::spawn(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            6549,
            workers,
            SimulationStep::Ms10,
            None,
            96,
        )
    };
    let mut fixed = make()?;
    let mut frames = vec![];
    for _ in 0..23 {
        fixed.advance()?;
        frames.push(fixed.frame().clone());
    }
    let i = frames
        .iter()
        .position(|f| f.body(51).unwrap().0 == f.body(52).unwrap().0)
        .ok_or("no predicted impact colocation")?;
    assert!(i > 1 && i < 20);
    assert!(
        frames
            .iter()
            .flat_map(|f| &f.workers)
            .flat_map(|w| &w.snapshot.impulses)
            .any(|r| r.a == 51 && r.b == 52 && r.normal_impulse > 0.)
    );
    for kill in [false, true] {
        let mut actual = make()?;
        actual.set_local_helper_limit(Some(0));
        actual.set_handoff_confirmation(true);
        actual.set_periodic_placement(Some(format!("{i},32,24,0").parse()?))?;
        let before = actual.worker_pid(1);
        for (j, expected) in frames.iter().enumerate() {
            if j == i && kill {
                actual.terminate_worker(1)?;
            }
            actual.advance()?;
            assert_eq!(actual.frame(), expected);
        }
        assert_eq!(actual.worker_pid(1) != before, kill);
        let status = actual.placement_status().unwrap();
        assert_eq!(status.committed_moves, 0);
        if std::thread::available_parallelism()?.get() > 1 {
            assert_eq!(status.windows_completed, u64::from(!kill));
            assert_eq!(status.unavailable_probes, u64::from(kill));
        }
    }
    Ok(())
}

#[allow(dead_code)]
#[path = "../examples/support/changing_worlds.rs"]
mod changing_contacts;

#[test]
fn a_landing_inside_a_timing_window_cannot_extend_the_next_sampling_delay() -> Test {
    use changing_contacts as scene;
    use gz_physics_distributed::physics::GroundPlane;
    for phase in [3, 8] {
        let groups = scene::seeds(scene::Layout::Adjacent);
        let pads: Vec<_> = groups
            .iter()
            .filter(|g| g.len() == 25)
            .map(|g| g[0].id)
            .collect();
        let launch = std::array::from_fn(|i| scene::LAUNCHES[i] + (phase + 3 * i as u64) % 10);
        let mut pool = WorkerPool::spawn(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            27819,
            scene::Placement::Split.worker_seeds(&groups),
            SimulationStep::Ms10,
            Some(GroundPlane::new(0.)?),
            96,
        )?;
        pool.set_local_helper_limit(Some(0));
        pool.set_periodic_placement(Some("8,10,1,4294967295,4".parse()?))?;
        let mut starts = vec![];
        let mut events = vec![];
        let mut prior = [false; 2];
        for tick in 1..=351 {
            let before = pool.placement_status().unwrap().windows_started;
            pool.advance_islands_in_field(
                scene::GRAVITY,
                &scene::wrenches_at(tick, &groups, &launch),
            )?;
            if pool.placement_status().unwrap().windows_started != before {
                starts.push(tick - 1);
            }
            let loaded = std::array::from_fn(|i| {
                pool.frame()
                    .workers
                    .iter()
                    .flat_map(|w| &w.snapshot.impulses)
                    .any(|p| p.a == pads[i] && p.normal_impulse > 0.)
            });
            if tick > scene::SETTLED && prior != loaded {
                events.push(tick);
            }
            prior = loaded;
        }
        assert_eq!(events.len(), 12);
        if std::thread::available_parallelism()?.get() > 1 {
            assert!(pool.placement_status().unwrap().windows_deferred > 0);
            for tick in events {
                let next = starts
                    .iter()
                    .copied()
                    .find(|&s| s >= tick)
                    .ok_or("missing resample")?;
                assert!(
                    next - tick < 10,
                    "contact transition at {tick} waited until {next}"
                );
            }
        }
    }
    Ok(())
}

#[test]
fn automatic_probes_query_residents_and_explicit_diagnostics_query_every_process() -> Test {
    if std::thread::available_parallelism()?.get() == 1 {
        return Ok(());
    }
    let mut pool = setup(32);
    pool.set_local_helper_limit(Some(0));
    pool.set_periodic_placement(Some("2,8,1,4294967295".parse()?))?;
    let requests = |p: &WorkerPool| {
        p.traffic()
            .into_iter()
            .map(|t| t.requests)
            .collect::<Vec<_>>()
    };
    let initial = requests(&pool);
    for _ in 0..3 {
        pool.advance()?;
    }
    let after = requests(&pool);
    // Three authoritative solves for everybody plus start and read on the owner.
    assert_eq!(
        after
            .iter()
            .zip(&initial)
            .map(|(a, b)| a - b)
            .collect::<Vec<_>>(),
        [5, 3, 3]
    );
    assert_eq!(pool.placement_status().unwrap().windows_completed, 1);
    let token = pool.start_load_sample_with_mode(2.try_into()?, PoolLoadMode::Scheduled)?;
    assert_eq!(token.scheduled_worker_mask(), 7);
    for _ in 0..2 {
        pool.advance()?;
    }
    let rows = pool.finish_load_sample(token)?;
    assert_eq!(
        rows.workers().iter().map(Vec::len).collect::<Vec<_>>(),
        [8, 0, 0]
    );
    assert_eq!(
        requests(&pool)
            .iter()
            .zip(&after)
            .map(|(a, b)| a - b)
            .collect::<Vec<_>>(),
        [4, 4, 4]
    );
    Ok(())
}
