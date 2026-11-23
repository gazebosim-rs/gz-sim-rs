// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{PhysicsBackend, PhysicsError};
use gz_physics_distributed::{
    SceneBackend,
    pool::{PlacementMember, PlacementMove, PlacementRecord},
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    passive_checkpoint::PassivePoolScene,
};
use std::{error::Error, path::Path, time::Duration};
#[path = "private/passive_pose_proof.rs"]
mod ecs;
type Test = Result<(), Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_ground.sdf");
fn options(engine: PhysicsEngine) -> WorldLoadOptions<'static> {
    WorldLoadOptions {
        engine,
        step: Some(Duration::from_millis(10)),
        noise_seed: [0; 32],
        distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
    }
}
fn runtime() -> WorldRuntimeOptions<'static> {
    WorldRuntimeOptions {
        distributed_periodic_placement: Some("2,2,64,0".parse().unwrap()),
        ..Default::default()
    }
}
fn load() -> Result<PassivePoolScene, Box<dyn Error>> {
    Ok(PassivePoolScene::load(
        SDF,
        "ground.sdf",
        "default",
        options(PhysicsEngine::DistributedIslands(4)),
        runtime(),
    )?)
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn stay(t: u64) -> PlacementRecord {
    PlacementRecord {
        target_tick: t,
        optional_move: None,
    }
}
fn first_component(sim: &Simulation, t: u64) -> PlacementRecord {
    let f = backend(sim).pool_frame().unwrap();
    let (source, _) = f.body(0).unwrap();
    let old = &f.workers[usize::from(source)];
    let ids = &old
        .components
        .iter()
        .find(|c| c.ids.contains(&0))
        .unwrap()
        .ids;
    PlacementRecord {
        target_tick: t,
        optional_move: Some(PlacementMove {
            source,
            destination: (source + 1) % 4,
            members: ids
                .iter()
                .map(|&id| PlacementMember {
                    id,
                    generation: old.generations[&id],
                })
                .collect(),
        }),
    }
}
fn public_link(s: &PassivePoolScene) -> u64 {
    s.simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::red::body"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
#[test]
fn standard_sdf_physics_phase_commits_one_record_across_final_reply_loss_and_pause() -> Test {
    let mut a = load()?;
    let mut b = load()?;
    for s in [&mut a, &mut b] {
        let id = public_link(s);
        s.set_paused(true);
        s.queue_wrench(id, [0.3, -0., 0.], [0.; 3])?;
        s.queue_impulse(id, [0.001, 0., 0.])?;
        s.tick()?;
        assert_eq!(s.simulation().elapsed(), Duration::ZERO);
    }
    let mut a = a.into_simulation();
    let mut b = b.into_simulation();
    let choice = first_component(&a, 1);
    let m = choice.optional_move.as_ref().unwrap();
    let lost = (0..4)
        .find(|&w| w != m.source && w != m.destination)
        .unwrap();
    a.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .drop_next_acknowledgement(lost)?;
    let initial = ecs::proof(&a);
    let mut wrong = choice.clone();
    wrong.optional_move.as_mut().unwrap().members[0].generation += 1;
    assert!(gz_sim_scene::step_with_placement(&mut a, &wrong).is_err());
    assert!(!a.is_faulted());
    assert_eq!(ecs::proof(&a), initial);
    let pids = backend(&a).local_worker_pids().unwrap();
    for tick in 1..=100 {
        let r = if tick == 1 {
            choice.clone()
        } else {
            stay(tick)
        };
        for s in [&mut a, &mut b] {
            gz_sim_scene::step_with_placement(s, &r)?;
            assert_eq!(backend(s).last_island_placement(), Some(&r));
            assert_eq!(backend(s).placement_status().unwrap().windows_started, 0);
            assert!(s.is_paused());
        }
        assert_eq!(ecs::proof(&a), ecs::proof(&b));
    }
    assert_eq!(pids, backend(&a).local_worker_pids().unwrap());
    assert_eq!(a.elapsed(), Duration::from_secs(1));
    a.step_exact(1)?;
    assert_eq!(backend(&a).last_island_placement(), Some(&stay(101)));
    Ok(())
}
#[test]
fn passive_deferred_ecs_velocity_is_preserved_and_wrong_engine_or_clock_never_steps() -> Test {
    let mut s = load()?;
    let id = public_link(&s);
    s.queue_velocity(id, [0.02, 0., 0.], [0.; 3])?;
    let before = ecs::proof(s.simulation());
    assert!(matches!(
        s.step_with_placement(&stay(1)),
        Err(PhysicsError::TransitionDeferred { .. })
    ));
    assert_eq!(ecs::proof(s.simulation()), before);
    assert!(!s.simulation().is_faulted());
    s.step_exact(1)?;
    assert_eq!(
        backend(s.simulation())
            .pool_frame()
            .unwrap()
            .body(0)
            .unwrap()
            .1
            .linear[0],
        0.02
    );
    s.step_with_placement(&stay(2))?;
    let file = tempfile::tempdir()?;
    let archive = file.path().join("passive.gzscn");
    s.save(&archive)?;
    drop(s);
    let mut restored = PassivePoolScene::resume(
        SDF,
        "restored.sdf",
        "default",
        options(PhysicsEngine::DistributedIslands(4)),
        runtime(),
        &archive,
    )?;
    assert!(
        backend(restored.simulation())
            .last_island_placement()
            .is_none()
    );
    let before = ecs::proof(restored.simulation());
    assert!(restored.step_with_placement(&stay(2)).is_err());
    assert_eq!(ecs::proof(restored.simulation()), before);
    restored.step_with_placement(&stay(3))?;
    restored.reset_loaded()?;
    assert!(
        backend(restored.simulation())
            .last_island_placement()
            .is_none()
    );
    restored.step_with_placement(&stay(1))?;
    for engine in [
        PhysicsEngine::DistributedPool(4),
        PhysicsEngine::DistributedAdaptive(4),
    ] {
        let mut unsupported = PassivePoolScene::load(
            SDF,
            "ground.sdf",
            "default",
            options(engine),
            Default::default(),
        )?;
        let before = ecs::proof(unsupported.simulation());
        assert!(unsupported.step_with_placement(&stay(1)).is_err());
        assert_eq!(ecs::proof(unsupported.simulation()), before);
        unsupported.step_exact(1)?;
        assert!(
            backend(unsupported.simulation())
                .last_island_placement()
                .is_none()
        );
    }
    Ok(())
}

#[test]
fn a_checkpoint_requested_inside_the_scheduled_call_cannot_omit_its_placement() -> Test {
    use bevy_ecs::prelude::{Res, ResMut, Resource};
    use gz_sim_core::{Phase, UpdateInfo};
    #[derive(Resource)]
    struct Attempt {
        path: std::path::PathBuf,
        observed: Option<String>,
    }
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("during-step.gzscn");
    let mut sim = load()?.into_simulation();
    sim.world_mut().insert_resource(Attempt {
        path: file.clone(),
        observed: None,
    });
    sim.add_systems(
        Phase::PhysicsPrepare,
        |mut attempt: ResMut<Attempt>,
         time: Res<UpdateInfo>,
         mut engine: ResMut<PhysicsBackend>| {
            let result = engine
                .downcast_mut::<SceneBackend>()
                .unwrap()
                .save_loaded_pool_checkpoint(&attempt.path, time.iteration - 1, b"scope-witness");
            attempt.observed = result.err().map(|e| e.to_string());
        },
    );
    let r = first_component(&sim, 1);
    gz_sim_scene::step_with_placement(&mut sim, &r)?;
    assert_eq!(backend(&sim).last_island_placement(), Some(&r));
    assert!(
        sim.world()
            .resource::<Attempt>()
            .observed
            .as_ref()
            .is_some_and(|e| e.contains("placement replay is running"))
    );
    assert!(!file.exists());
    // After the call, the committed native barrier can be captured normally.
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .save_loaded_pool_checkpoint(&file, 1, b"scope-witness")?;
    assert!(file.exists());
    Ok(())
}
