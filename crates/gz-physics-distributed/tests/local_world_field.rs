// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/native_contact_control.rs"]
mod native;
use gz_physics_distributed::{
    coordinator::Error as CoordinatorError,
    physics::{BodyWrench, GroundPlane, PendingImpulse, SimulationStep},
    pool::WorkerPool,
};
use std::{error::Error, path::Path};
#[path = "../examples/support/world_profile_scene.rs"]
mod scene;

#[test]
fn mixed_rates_keep_ordered_loads_local_across_a_retried_worker_barrier()
-> Result<(), Box<dyn Error>> {
    let gravity = [0., 0., -9.81];
    let ground = Some(GroundPlane::new(0.)?);
    let mut groups = scene::stacks();
    // Twelve additional floor contacts and one airborne coarse sphere share the owner.
    for id in 9..=21 {
        let z = if id == 21 { 80. } else { 0.455 };
        groups.push(vec![scene::sphere(
            id,
            [12. * id as f64, 0., z],
            0.9,
            [0.; 3],
        )]);
    }
    let mut controls = groups
        .iter()
        .map(|g| native::Control::new(g, ground, gravity))
        .collect::<Result<Vec<_>, _>>()?;
    let seed = groups.into_iter().flatten().collect();
    let mut pool = WorkerPool::spawn(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        0x1a441,
        vec![seed, vec![], vec![], vec![]],
        SimulationStep::Ms10,
        ground,
        256,
    )?;
    // Interleave inputs for separate targets and repeat a target in submission order.
    let loads = [(21, 1.), (6, -3.), (19, -5.), (6, 1.)].map(|(id, f)| BodyWrench {
        id,
        force: [0., 0., f],
        torque: [0., 0., f * 0.01],
    });
    let fine: Vec<u64> = (1..=20).collect();
    for tick in 1..=120 {
        let mut pending = Vec::new();
        if tick == 110 {
            for (id, impulse) in [
                (19, [0., 0., -0.4]),
                (21, [0.2, 0., 0.]),
                (19, [0., 0., 0.1]),
            ] {
                pool.queue_impulse_in_field(id, impulse, gravity, &loads)?;
                pending.push(PendingImpulse { id, impulse });
            }
            let committed = pool.frame().clone();
            let prior_rate = pool.last_island_refined_bodies().unwrap().to_vec();
            pool.drop_next_acknowledgement(0)?;
            assert!(matches!(
                pool.advance_islands_in_field(gravity, &loads),
                Err(CoordinatorError::Transport(_))
            ));
            assert_eq!(pool.frame(), &committed);
            assert_eq!(
                pool.last_island_refined_bodies(),
                Some(prior_rate.as_slice())
            );
        }
        pool.advance_islands_in_field(gravity, &loads)?;
        assert_eq!(pool.last_island_refined_bodies(), Some(fine.as_slice()));
        let obs = &pool.frame().workers[0];
        assert_eq!(obs.snapshot.bodies.len(), 21);
        let mut all_pairs = Vec::new();
        let mut all_supports = Vec::new();
        for c in &mut controls {
            let owns = |id| c.bodies.iter().any(|b| b.id == id);
            let local = loads
                .iter()
                .filter(|w| owns(w.id))
                .cloned()
                .collect::<Vec<_>>();
            let inputs = pending
                .iter()
                .filter(|p| owns(p.id))
                .cloned()
                .collect::<Vec<_>>();
            let (pairs, supports) = c.advance(if owns(21) { 1 } else { 10 }, &local, &inputs)?;
            all_pairs.extend(pairs);
            all_supports.extend(supports);
            for b in &c.bodies {
                assert_eq!(pool.frame().body(b.id).unwrap(), (0, b), "tick {tick}");
            }
        }
        all_pairs.sort_by_key(|p| (p.a, p.b));
        all_supports.sort_by_key(|p| p.body);
        assert_eq!(obs.snapshot.impulses, all_pairs, "tick {tick}");
        assert_eq!(obs.snapshot.supports, all_supports, "tick {tick}");
    }
    Ok(())
}

#[test]
fn helper_budget_caps_actual_grants_across_replacement_and_pool_restart()
-> Result<(), Box<dyn Error>> {
    use gz_physics_distributed::pool::PoolNativeRecovery;
    let cpus = std::thread::available_parallelism()?.get();
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let temp = tempfile::tempdir()?;
    for owners in [1, 2] {
        let checkpoint = temp.path().join(format!("helpers-{owners}.gzpool"));
        let mut seeds = vec![vec![]; 4];
        for i in 0..4 {
            seeds[i % owners].push(scene::sphere(
                i as u64 + 1,
                [i as f64 * 12., 0., 0.45],
                0.9,
                [0.; 3],
            ));
        }
        let mut pool = WorkerPool::spawn(
            worker,
            849,
            seeds,
            SimulationStep::Ms10,
            Some(GroundPlane::new(0.)?),
            64,
        )?;
        let gravity = [0., 0., -9.81];
        let granted = cpus.saturating_sub(owners).min(owners);
        assert_eq!(pool.last_local_helper_mask(), None);
        pool.set_local_helper_limit(Some(1));
        pool.advance_islands_in_field(gravity, &[])?;
        let original = pool.last_local_helper_mask().unwrap();
        assert_eq!(original.count_ones() as usize, granted.min(1));
        assert_eq!(original >> owners, 0);

        pool.set_local_helper_limit(Some(0));
        pool.drop_next_acknowledgement(0)?;
        assert!(matches!(
            pool.advance_islands_in_field(gravity, &[]),
            Err(CoordinatorError::Transport(_))
        ));
        assert_eq!(pool.last_local_helper_mask(), Some(original));
        pool.advance_islands_in_field(gravity, &[])?;
        assert_eq!(pool.last_local_helper_mask(), Some(0));
        pool.compact()?;
        pool.terminate_worker(0)?;
        pool.recover(0)?;
        pool.advance_islands_in_field(gravity, &[])?;
        assert_eq!(pool.last_local_helper_mask(), Some(0));

        pool.save_native_recovery(&checkpoint)?;
        drop(pool);
        let mut restored = PoolNativeRecovery::open(&checkpoint)?.resume(worker)?;
        assert_eq!(restored.last_local_helper_mask(), None);
        restored.advance_islands_in_field(gravity, &[])?;
        let automatic = restored.last_local_helper_mask().unwrap();
        assert_eq!(automatic.count_ones() as usize, granted);
        assert_eq!(automatic >> owners, 0);
        restored.set_local_helper_limit(Some(0));
        restored.advance_islands_in_field(gravity, &[])?;
        assert_eq!(restored.last_local_helper_mask(), Some(0));
        restored.set_local_helper_limit(None);
        restored.advance_islands_in_field(gravity, &[])?;
        assert_eq!(
            restored.last_local_helper_mask().unwrap().count_ones() as usize,
            granted
        );
    }
    Ok(())
}
