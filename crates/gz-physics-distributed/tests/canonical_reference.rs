// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/native_profile_grid.rs"]
mod grid;
#[path = "../examples/support/native_contact_control.rs"]
mod native;
use gz_physics_distributed::{
    physics::{GroundPlane, PendingImpulse, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
use std::{error::Error, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[test]
fn canonical_birth_order_survives_warm_moves_journal_replacement_native_reload_and_input_order()
-> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let sphere = grid::grid(64, true);
    let ground = Some(GroundPlane::new(0.)?);
    let g = [0., 0., -9.81];
    let mut native = native::Control::new(&sphere, ground, g)?;
    let mut shuffled = sphere.clone();
    shuffled.reverse();
    let mut pool = WorkerPool::spawn_canonical_scene(
        worker,
        8833,
        shuffled,
        4,
        SimulationStep::Ms10,
        ground,
        128,
    )?;
    assert_eq!(pool.frame().workers[0].components.len(), 1);
    assert_eq!(pool.frame().workers[0].snapshot.bodies, sphere);
    let directory = tempfile::tempdir()?;
    let manifest = directory.path().join("initial");
    pool.save_native_recovery(&manifest)?;
    let bytes = fs::read(&manifest)?;
    let json: serde_json::Value = serde_json::from_slice(&bytes[40..])?;
    let bindings: Vec<u64> =
        serde_json::from_value(json["images"][0]["partition"]["solvers"][0]["bindings"].clone())?;
    assert_eq!(bindings, (0..64).collect::<Vec<_>>());
    let mut owner = 0;
    for tick in 1..=12 {
        let pending = (tick == 4 || tick == 7).then_some(PendingImpulse {
            id: 1,
            impulse: [0.001, 0., 0.],
        });
        if let Some(impulse) = &pending {
            pool.queue_impulse(impulse.id, impulse.impulse)?;
        }
        if tick == 2 {
            pool.terminate_worker(0)?;
            pool.recover(0)?;
        }
        if tick == 4 {
            pool.transfer_component(1, 3)?;
            owner = 3;
        }
        if tick == 5 {
            pool.terminate_worker(owner)?;
            pool.recover(owner)?;
        }
        if tick == 7 {
            let file = directory.path().join("warm");
            pool.save_native_recovery(&file)?;
            drop(pool);
            pool = PoolNativeRecovery::open(file)?.resume(worker)?;
        }
        if tick == 9 {
            pool.transfer_component(64, 2)?;
            owner = 2;
        }
        pool.advance_adaptive_in_field(g, &[])?;
        let (pairs, floors) =
            native.advance(pool.last_substeps().unwrap(), &[], pending.as_slice())?;
        let obs = &pool.frame().workers[owner as usize];
        assert_eq!(obs.components.len(), 1);
        assert_eq!(obs.snapshot.bodies, native.bodies);
        assert_eq!(obs.snapshot.impulses, pairs);
        assert_eq!(obs.snapshot.supports, floors);
    }
    assert!(matches!(
        WorkerPool::spawn_canonical_scene(
            Path::new("/absent/worker"),
            9,
            grid::grid(65, false),
            4,
            SimulationStep::Ms10,
            ground,
            128
        ),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
    Ok(())
}
