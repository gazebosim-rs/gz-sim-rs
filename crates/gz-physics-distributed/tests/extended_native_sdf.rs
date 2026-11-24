// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/native_contact_control.rs"]
mod native;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{SceneBackend, pool::PoolFrame};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::{error::Error, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_native64_floor.sdf");
fn load(width: u8, extended: bool) -> Result<Simulation> {
    load_configured(
        SDF,
        PhysicsEngine::DistributedAdaptive(width),
        WorldRuntimeOptions {
            distributed_native_64: extended,
            distributed_contact_placement: true,
            ..Default::default()
        },
    )
}
fn load_configured(
    sdf: &str,
    engine: PhysicsEngine,
    runtime: WorldRuntimeOptions<'_>,
) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        sdf,
        "placement.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
        runtime,
    )?)
}
fn frame(sim: &Simulation) -> &PoolFrame {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .pool_frame()
        .unwrap()
}
#[test]
fn native64_sdf_exposes_the_same_physical_floor_for_three_through_eight_workers_and_reset()
-> Result<()> {
    for width in 3..=8 {
        let mut sim = load(width, true)?;
        let mut seeds = frame(&sim)
            .workers
            .iter()
            .flat_map(|w| w.snapshot.bodies.iter().cloned())
            .collect::<Vec<_>>();
        seeds.sort_by_key(|b| b.id);
        let mut control = native::Control::new(&seeds, frame(&sim).ground(), [0., 0., -9.81])?;
        let seed = frame(&sim).clone();
        for _ in 0..6 {
            sim.step_exact(1)?;
            let (pairs, floors) = control.advance(10, &[], &[])?;
            let obs = frame(&sim)
                .workers
                .iter()
                .find(|w| w.snapshot.bodies.len() == 64)
                .expect("all imminent floor contacts require one forecast group");
            assert_eq!(obs.components.len(), 1);
            assert_eq!(obs.snapshot.bodies, control.bodies);
            assert_eq!(obs.snapshot.impulses, pairs);
            assert_eq!(obs.snapshot.supports, floors);
        }
        assert_eq!(
            frame(&sim)
                .workers
                .iter()
                .map(|w| w.snapshot.supports.len())
                .sum::<usize>(),
            64
        );
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(frame(&sim), &seed);
        sim.step_exact(1)?;
    }
    Ok(())
}
#[test]
fn two_process_engine_refuses_extended_mode_and_standard_pool_retains_previous_limit() {
    assert!(load(2, true).is_err());
    match load(4, false) {
        Ok(mut standard) => {
            let old = frame(&standard).clone();
            assert!(standard.step_exact(1).is_err());
            assert_eq!(frame(&standard), &old);
        }
        Err(error) => assert!(
            error.to_string().contains("require a common native solver"),
            "{error}"
        ),
    }
}

#[test]
fn canonical_reference_seeds_and_resets_one_id_ordered_world_for_uniform_steps() -> Result<()> {
    let cargo_pads = include_str!("../../../examples/distributed_cargo_pads.sdf");
    let runtime = WorldRuntimeOptions {
        distributed_native_64: true,
        distributed_canonical_reference: true,
        ..Default::default()
    };
    for (engine, fixed_substeps) in [
        (PhysicsEngine::DistributedPool(3), Some(1)),
        (PhysicsEngine::DistributedAdaptive(8), None),
    ] {
        let mut simulation = load_configured(cargo_pads, engine, runtime)?;
        let initial = frame(&simulation).clone();
        let first = &initial.workers[0];
        assert_eq!(first.components.len(), 1);
        assert_eq!(first.snapshot.bodies.len(), 62);
        assert!(
            initial.workers[1..]
                .iter()
                .all(|worker| worker.components.is_empty())
        );
        assert!(first.snapshot.bodies.windows(2).all(|b| b[0].id < b[1].id));
        let mut reference =
            native::Control::new(&first.snapshot.bodies, initial.ground(), [0., 0., -9.81])?;
        for _ in 0..20 {
            simulation.step_exact(1)?;
            let backend = simulation
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<SceneBackend>()
                .unwrap();
            let substeps = fixed_substeps.or(backend.last_substeps()).unwrap();
            let (pairs, floors) = reference.advance(substeps, &[], &[])?;
            let obs = &frame(&simulation).workers[0];
            assert_eq!(obs.components.len(), 1);
            assert_eq!(obs.snapshot.bodies, reference.bodies);
            assert_eq!(obs.snapshot.impulses, pairs);
            assert_eq!(obs.snapshot.supports, floors);
        }
        reset_loaded_world(&mut simulation, |_| ())?;
        assert_eq!(frame(&simulation), &initial);
    }
    assert!(load_configured(cargo_pads, PhysicsEngine::DistributedIslands(3), runtime).is_err());
    let first_model = SDF
        .lines()
        .find(|line| line.starts_with("<model name=\"ball_01\">"))
        .unwrap();
    let extra = first_model
        .replace("name=\"ball_01\"", "name=\"ball_65\"")
        .replace("<pose>-1.54 -1.54 ", "<pose>25.0 25.0 ");
    let too_many = SDF.replace("</world>", &format!("{extra}</world>"));
    let err = load_configured(&too_many, PhysicsEngine::DistributedAdaptive(3), runtime)
        .err()
        .expect("a single reference graph is bounded to 64 finite bodies");
    assert!(
        err.to_string().contains("distributed scene body limit"),
        "{err}"
    );
    assert!(
        load_configured(
            cargo_pads,
            PhysicsEngine::DistributedAdaptive(3),
            WorldRuntimeOptions {
                distributed_native_64: false,
                ..runtime
            }
        )
        .is_err()
    );
    assert!(
        load_configured(
            cargo_pads,
            PhysicsEngine::DistributedAdaptive(3),
            WorldRuntimeOptions {
                distributed_contact_placement: true,
                ..runtime
            }
        )
        .is_err()
    );
    Ok(())
}

#[test]
fn canonical_per_point_sdf_reset_replays_slide_and_yaw_with_the_original_law() -> Result<()> {
    use gz_sim_scene::{BodyHandle, Name};
    let world = include_str!("../../../examples/distributed_torsional_contact.sdf");
    let exercise = |sim: &mut Simulation| -> Result<Vec<gz_physics_distributed::physics::Body>> {
        let id = sim
            .world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<Name>()?.0 == "default::yaw_block::body")
                    .then(|| e.get::<BodyHandle>().unwrap().id())
            })
            .unwrap();
        sim.step_exact(30)?;
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .apply_impulse(id, [4.8, 0., 0.], [0., 0., 0.96])?;
        let mut states = Vec::new();
        for _ in 0..90 {
            sim.step_exact(1)?;
            let (owner, body) = frame(sim).body(id.raw()).unwrap();
            assert_eq!(owner, 0);
            let mut body = body.clone();
            // A loaded reset may issue fresh native IDs for the same authored link.
            body.id = 0;
            states.push(body);
        }
        Ok(states)
    };
    let mut ends = Vec::new();
    for point in [false, true] {
        let runtime = WorldRuntimeOptions {
            distributed_native_64: true,
            distributed_canonical_reference: true,
            rapier_per_point_friction: point,
            ..Default::default()
        };
        let mut sim = load_configured(world, PhysicsEngine::DistributedAdaptive(3), runtime)?;
        let first = exercise(&mut sim)?;
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(exercise(&mut sim)?, first);
        ends.push(first.last().unwrap().position[0]);
    }
    assert!((ends[0] - ends[1]).abs() > 0.1, "{ends:?}");
    Ok(())
}
