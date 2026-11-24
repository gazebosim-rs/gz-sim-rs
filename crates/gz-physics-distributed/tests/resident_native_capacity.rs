// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/resident_capacity.rs"]
mod capacity;
#[path = "../examples/support/native_contact_control.rs"]
mod native;
use gz_physics_distributed::{
    physics::{Body, GroundPlane, NativeSolverProfile, SimulationStep},
    pool::WorkerPool,
};
use std::{error::Error, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[test]
fn four_full_native_graphs_share_a_256_body_worker_and_migrate_one_at_a_time() -> Result<()> {
    let sites: Vec<_> = (0..4).map(capacity::lattice).collect();
    let ground = Some(GroundPlane::new(0.)?);
    let mut oversized = sites.concat();
    oversized.push(Body {
        id: 257,
        position: [100., 0., 5.],
        ..oversized[0].clone()
    });
    assert!(matches!(
        WorkerPool::spawn_with_profile(
            Path::new("/absent/worker"),
            7080,
            vec![oversized, vec![], vec![]],
            SimulationStep::Ms10,
            ground,
            128,
            NativeSolverProfile::Extended64
        ),
        Err(gz_physics_distributed::coordinator::Error::Capacity)
    ));
    for packed in [false, true] {
        let scratch = tempfile::tempdir()?;
        let result = capacity::exercise(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            packed,
            scratch.path(),
            12,
            |_| Ok(()),
        )?;
        assert!(
            result.peak_pairs > 496,
            "expected more pairs than one standard solver could produce"
        );
        assert!(result.peak_floor > 0);
        assert_eq!(result.events.len(), 3);
        println!("packed={packed} {}", serde_json::to_string(&result)?);
    }
    Ok(())
}
