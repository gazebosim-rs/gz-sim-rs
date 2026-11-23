// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::{Body, GroundPlane, Primitive, SimulationStep, fixture},
    pool::{PoolNativeRecovery, WorkerPool},
    transport::WorkerProcess,
};
use std::{error::Error, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const ZERO: [f64; 3] = [0.; 3];

fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}

fn sphere(id: u64, position: [f64; 3]) -> Body {
    Body {
        id,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        linear: ZERO,
        position,
        ..fixture()[0].promoted()
    }
}

// Exercise semantic checks independently of the outer file checksum.
fn altered_ids(path: &Path, ids: serde_json::Value) -> Result<std::path::PathBuf> {
    use sha2::{Digest, Sha256};
    let bytes = std::fs::read(path)?;
    let mut content: serde_json::Value = serde_json::from_slice(&bytes[40..])?;
    content["last_island_ids"] = ids;
    let payload = serde_json::to_vec(&content)?;
    let mut rehashed = bytes[..8].to_vec();
    rehashed.extend(Sha256::digest(&payload));
    rehashed.extend(payload);
    let output = path.with_extension("altered");
    std::fs::write(&output, rehashed)?;
    Ok(output)
}

#[test]
fn staged_impulse_selects_a_cross_owner_chain_and_exactly_replays_its_mask() -> Result<()> {
    let left = [sphere(1, ZERO), sphere(4, [30., 0., 0.])];
    let right = [sphere(2, [0.32, 0., 0.]), sphere(3, [2.3, 0., 0.])];
    let peers = [
        WorkerProcess::spawn(worker(), 0x7310, 0, &left)?,
        WorkerProcess::spawn(worker(), 0x7310, 1, &right)?,
    ];
    let mut sim = Coordinator::new_recorded(0x7310, peers, 128)?;
    assert_eq!(sim.last_island_refined_bodies(), None);
    assert!(sim.refined_island_bodies(ZERO, &[])?.is_empty());
    sim.advance_islands_in_field(ZERO, &[])?;
    assert_eq!(sim.last_island_refined_bodies(), Some([].as_slice()));
    sim.queue_impulse_routed(1, [20., 0., 0.])?;
    // Only body 1 can reach body 2 in straight free flight. Body 3 belongs to
    // the same reachable island because a native contact could redirect 2.
    let expected = [1, 2, 3];
    assert_eq!(sim.refined_island_bodies(ZERO, &[])?, expected);
    sim.advance_islands_in_field(ZERO, &[])?;
    assert_eq!(sim.last_island_refined_bodies(), Some(expected.as_slice()));
    assert_eq!(sim.last_substeps(), Some(10));
    let dir = tempfile::tempdir()?;
    let journal = dir.path().join("journal");
    let native = dir.path().join("native");
    sim.save_recovery(&journal)?;
    sim.save_native_recovery(&native)?;
    for ids in [
        serde_json::json!([1, 1]),
        serde_json::json!([999]),
        serde_json::json!([]),
    ] {
        assert!(NativeRecoveryPlan::open(altered_ids(&native, ids)?).is_err());
    }
    let at = sim.frame().clone();
    let next = sim.advance_islands_in_field(ZERO, &[])?.clone();
    drop(sim);
    let mut replay = RecoveryPlan::open(journal)?
        .resume(|run, id, bodies| WorkerProcess::spawn(worker(), run, id, bodies))?;
    assert_eq!(replay.frame(), &at);
    assert_eq!(
        replay.last_island_refined_bodies(),
        Some(expected.as_slice())
    );
    assert_eq!(replay.advance_islands_in_field(ZERO, &[])?, &next);
    drop(replay);
    let mut native = NativeRecoveryPlan::open(native)?.resume(worker())?;
    assert_eq!(
        native.last_island_refined_bodies(),
        Some(expected.as_slice())
    );
    assert_eq!(native.advance_islands_in_field(ZERO, &[])?, &next);
    native.advance_adaptive_in_field(ZERO, &[])?;
    assert_eq!(native.last_island_refined_bodies(), None);
    Ok(())
}

#[test]
fn same_owner_uses_both_resolutions_and_preserves_last_receipt_through_a_failed_step() -> Result<()>
{
    let ground = Some(GroundPlane::new(0.)?);
    let seeds = vec![
        vec![sphere(1, [0., 0., 0.18]), sphere(2, [20., 0., 8.])],
        vec![],
        vec![],
        vec![],
    ];
    let mut p = WorkerPool::spawn(worker(), 0x7311, seeds, SimulationStep::Ms10, ground, 128)?;
    let g = [0., 0., -9.81];
    p.advance_islands_in_field(g, &[])?;
    assert_eq!(p.last_island_refined_bodies(), Some([].as_slice()));
    p.queue_impulse_in_gravity(1, [0., 0., -10.], g)?;
    assert_eq!(p.refined_island_bodies(g, &[])?, [1]);
    let start = p.frame().clone();
    p.drop_next_acknowledgement(0)?;
    assert!(p.advance_islands_in_field(g, &[]).is_err());
    assert_eq!(p.frame(), &start);
    assert_eq!(p.last_island_refined_bodies(), Some([].as_slice()));
    p.advance_islands_in_field(g, &[])?;
    assert_eq!(p.last_island_refined_bodies(), Some([1].as_slice()));
    assert_eq!(p.last_substeps(), Some(10));
    p.transfer_component(1, 3)?;
    let dir = tempfile::tempdir()?;
    let saved = dir.path().join("native");
    p.save_native_recovery(&saved)?;
    for ids in [
        serde_json::json!([1, 1]),
        serde_json::json!([999]),
        serde_json::json!([]),
    ] {
        assert!(PoolNativeRecovery::open(altered_ids(&saved, ids)?).is_err());
    }
    let expected = p.advance_islands_in_field(g, &[])?.clone();
    drop(p);
    let mut resumed = PoolNativeRecovery::open(saved)?.resume(worker())?;
    assert_eq!(resumed.last_island_refined_bodies(), Some([1].as_slice()));
    assert_eq!(resumed.advance_islands_in_field(g, &[])?, &expected);
    resumed.advance_in_field(g, &[])?;
    assert_eq!(resumed.last_island_refined_bodies(), None);
    Ok(())
}
