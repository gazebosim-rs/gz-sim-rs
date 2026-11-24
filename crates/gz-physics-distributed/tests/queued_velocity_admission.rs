// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stable versus transient restrictions on a deferred public link twist.
#[path = "private/passive_pose_proof.rs"]
mod passive_pose_proof;
use gz_physics::{PhysicsBackend, PhysicsError};
use gz_physics_distributed::SceneBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, InputError, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    load_world_with_runtime, queue_link_world_velocity, queue_link_world_wrench, remove_model,
    reset_loaded_world,
};
use std::path::Path;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const SDF: &str = include_str!("../../../examples/distributed_stack.sdf");
fn options(engine: PhysicsEngine) -> WorldLoadOptions<'static> {
    WorldLoadOptions {
        engine,
        step: None,
        noise_seed: [0; 32],
        distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
    }
}
fn load(engine: PhysicsEngine, runtime: WorldRuntimeOptions<'_>) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        SDF,
        "stack.sdf",
        "default",
        &sdformat::NoResources,
        options(engine),
        runtime,
    )?)
}
fn component(sim: &Simulation, link: u64) -> &gz_physics_distributed::physics::ComponentInfo {
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    let id = sim
        .world()
        .get::<BodyHandle>(entity_by_id(sim.world(), link).unwrap())
        .unwrap()
        .id()
        .raw();
    let frame = backend.pool_frame().unwrap();
    let owner = frame.body(id).unwrap().0;
    frame.workers[usize::from(owner)]
        .components
        .iter()
        .find(|c| c.ids.contains(&id))
        .unwrap()
}

#[test]
fn retained_contact_history_refuses_before_queue_and_the_live_scene_keeps_advancing() -> Result<()>
{
    for width in [3, 4, 8] {
        let engine = PhysicsEngine::DistributedIslands(width);
        let mut tested = load(engine, Default::default())?;
        let mut reference = load(engine, Default::default())?;
        for sim in [&mut tested, &mut reference] {
            sim.step_exact(110)?;
            queue_link_world_wrench(sim, 7, [0.12, 0., 0.03], [0.; 3])?;
        }
        assert!(component(&tested, 7).contact_history);
        let before = passive_pose_proof::proof(&tested);
        let refused = queue_link_world_velocity(&mut tested, 7, [0.25, 0., 0.15], [0.; 3]);
        assert!(
            matches!(
                refused,
                Err(InputError::Physics(PhysicsError::TransitionDeferred { .. }))
            ),
            "loaded velocity admission was {refused:?}"
        );
        assert_eq!(passive_pose_proof::proof(&tested), before);
        for _ in 0..48 {
            tested.step_exact(1)?;
            reference.step_exact(1)?;
            assert_eq!(
                passive_pose_proof::proof(&tested),
                passive_pose_proof::proof(&reference)
            );
        }
        assert!(!tested.is_faulted());
        // The reset cancels old inputs and installs a genuinely cold native world.
        reset_loaded_world(&mut tested, |_| ())?;
        assert!(!component(&tested, 7).contact_history);
        queue_link_world_velocity(&mut tested, 7, [0.2, 0., 0.1], [0.; 3])?;
        queue_link_world_velocity(&mut tested, 7, [0.3, 0., 0.1], [0.; 3])?;
        let rejected = queue_link_world_velocity(&mut tested, 7, [f64::NAN, 0., 0.], [0.; 3]);
        assert!(matches!(rejected, Err(InputError::InvalidValue)));
        tested.tick()?; // paused: retain the second valid public request
        tested.step_exact(1)?;
        let state = tested
            .world()
            .get::<gz_sim_scene::Velocity>(entity_by_id(tested.world(), 7).unwrap())
            .unwrap();
        assert_eq!(state.linear[0], 0.3);
        println!(
            "width {width}: sticky history refused without mutation; 48 following native/public frames exact with retained wrench, loaded reset and replacement order preserved"
        );
    }
    Ok(())
}

#[test]
fn cold_group_can_become_editable_by_retirement_at_the_same_paused_tick() -> Result<()> {
    let mut sim = load(
        PhysicsEngine::DistributedPool(4),
        WorldRuntimeOptions {
            distributed_native_64: true,
            distributed_canonical_reference: true,
            ..Default::default()
        },
    )?;
    assert_eq!(component(&sim, 7).ids.len(), 3);
    assert!(!component(&sim, 7).contact_history);
    queue_link_world_velocity(&mut sim, 7, [0.25, 0., 0.], [0.; 3])?;
    remove_model(&mut sim, 2)?;
    remove_model(&mut sim, 10)?;
    assert_eq!(component(&sim, 7).ids.len(), 1);
    sim.step_exact(1)?;
    let v = sim
        .world()
        .get::<gz_sim_scene::Velocity>(entity_by_id(sim.world(), 7).unwrap())
        .unwrap();
    assert_eq!(v.linear[0], 0.25);
    Ok(())
}

#[test]
fn two_worker_native_solver_keeps_its_existing_contact_velocity_semantics() -> Result<()> {
    let mut sim = load(PhysicsEngine::Distributed, Default::default())?;
    sim.step_exact(110)?;
    queue_link_world_velocity(&mut sim, 7, [0.1, 0., 0.], [0.; 3])?;
    sim.step_exact(2)?;
    assert!(!sim.is_faulted());
    Ok(())
}

#[test]
fn released_singleton_preserves_permanent_velocity_admission_across_disk_recovery() -> Result<()> {
    use gz_sim_scene::passive_checkpoint::PassivePoolScene;
    let worker_options = options(PhysicsEngine::DistributedIslands(4));
    let scratch = tempfile::tempdir()?;
    let cut = scratch.path().join("released.gzscene");
    let mut source = PassivePoolScene::load(
        SDF,
        "stack.sdf",
        "default",
        worker_options,
        Default::default(),
    )?;
    let mut control = load(PhysicsEngine::DistributedIslands(4), Default::default())?;
    source.step_exact(110)?;
    control.step_exact(110)?;
    for neighbor in [2, 10] {
        source.retire(neighbor)?;
        remove_model(&mut control, neighbor)?;
    }
    assert_eq!(component(source.simulation(), 7).ids.len(), 1);
    assert!(component(source.simulation(), 7).contact_history);
    source.save(&cut)?;
    drop(source);
    let mut restored = PassivePoolScene::resume(
        SDF,
        "stack.sdf",
        "default",
        worker_options,
        Default::default(),
        &cut,
    )?;
    assert!(matches!(
        restored.queue_velocity(7, [0.3, 0., 0.], [0.; 3]),
        Err(InputError::Physics(PhysicsError::TransitionDeferred { .. }))
    ));
    for _ in 0..32 {
        control.step_exact(1)?;
        restored.step_exact(1)?;
        assert_eq!(
            passive_pose_proof::proof(restored.simulation()),
            passive_pose_proof::proof(&control)
        );
    }
    Ok(())
}
