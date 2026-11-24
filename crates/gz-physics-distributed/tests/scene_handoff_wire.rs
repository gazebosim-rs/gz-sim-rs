// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::{error::Error, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn load(worker: &Path, engine: PhysicsEngine, packed: bool) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        include_str!("../../../examples/distributed_stack.sdf"),
        "stack.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
        WorldRuntimeOptions {
            distributed_packed_handoffs: packed,
            ..Default::default()
        },
    )?)
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
}

#[test]
fn real_sdf_selects_negotiated_handoffs_and_keeps_them_after_worker_and_world_restarts()
-> Result<()> {
    let modern = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for width in [2, 3, 8] {
        let engine = PhysicsEngine::DistributedIslands(width);
        let mut packed = load(modern, engine, true)?;
        let mut json = load(modern, engine, false)?;
        assert_eq!(
            backend(&packed).packed_handoff_workers(),
            Some(usize::from(width))
        );
        assert_eq!(backend(&json).packed_handoff_workers(), Some(0));
        let pids = backend(&packed).local_worker_pids().unwrap();
        let mut amount = (0, 0);
        for tick in 1..=160 {
            if tick == 130 {
                for (sim, out) in [(&packed, &mut amount.0), (&json, &mut amount.1)] {
                    *out = backend(sim)
                        .local_worker_traffic()
                        .unwrap()
                        .iter()
                        .map(|w| w.request_bytes + w.reply_bytes)
                        .sum::<u64>();
                }
                #[cfg(unix)]
                assert!(
                    std::process::Command::new("kill")
                        .args(["-KILL", &pids[0].to_string()])
                        .status()?
                        .success()
                );
            }
            packed.step_exact(1)?;
            json.step_exact(1)?;
            assert_eq!(backend(&packed).frame(), backend(&json).frame());
            assert_eq!(backend(&packed).pool_frame(), backend(&json).pool_frame());
        }
        #[cfg(unix)]
        assert_ne!(backend(&packed).local_worker_pids().unwrap()[0], pids[0]);
        assert_eq!(
            backend(&packed).packed_handoff_workers(),
            Some(usize::from(width))
        );
        if width > 2 {
            assert!(
                amount.0 < amount.1,
                "packed pool payloads should be smaller: {amount:?}"
            );
        }
        reset_loaded_world(&mut packed, |_| ())?;
        assert_eq!(
            backend(&packed).packed_handoff_workers(),
            Some(usize::from(width))
        );
        packed
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<SceneBackend>()
            .unwrap()
            .set_packed_handoffs(false);
        reset_loaded_world(&mut packed, |_| ())?;
        assert_eq!(backend(&packed).packed_handoff_workers(), Some(0));
        println!(
            "width {width}, pre-fault application bytes packed/JSON {}/{}, complete physics matched",
            amount.0, amount.1
        );
    }
    if let Some(old) = std::env::var_os("GZ_TEST_LEGACY_HANDOFF_WORKER") {
        for width in [2, 4] {
            let mut sim = load(
                Path::new(&old),
                PhysicsEngine::DistributedIslands(width),
                true,
            )?;
            assert_eq!(backend(&sim).packed_handoff_workers(), Some(0));
            sim.step_exact(160)?;
            reset_loaded_world(&mut sim, |_| ())?;
            assert_eq!(backend(&sim).packed_handoff_workers(), Some(0));
            println!("historical width {width} workers completed 160 SDF steps using JSON");
        }
    }
    Ok(())
}

#[test]
fn unrelated_physics_engine_refuses_packed_worker_option_before_loading() {
    let error = load(Path::new("/unused"), PhysicsEngine::Rapier, true)
        .err()
        .expect("accepted unrelated option");
    assert!(
        error
            .to_string()
            .contains("packed handoffs require the distributed engine")
    );
}
