// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::BodyState;
use gz_physics_distributed::{
    physics::{Body, NativeFrictionModel, NativeSolverProfile, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
use std::{error::Error, path::Path};
#[path = "../examples/support/coupled_box_pair.rs"]
mod box_pair;
#[path = "../examples/support/coupled_point_control.rs"]
mod direct;
const G: [f64; 3] = [0., 0., -9.81];
#[test]
fn a_loaded_per_corner_patch_retains_its_native_dynamics_after_transfer_and_disk_restart()
-> Result<(), Box<dyn Error>> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let profile = NativeSolverProfile::Standard32.with_friction(NativeFrictionModel::PerPoint);
    let seeds = box_pair::seeds(false, true, false);
    let mut reference = direct::Direct::new(&seeds, true);
    let mut grouped = direct::Direct::new(&seeds, false);
    let mut pool = WorkerPool::spawn_with_profile(
        worker,
        0x7653,
        vec![seeds.to_vec(), vec![], vec![]],
        SimulationStep::Ms10,
        None,
        100,
        profile,
    )?;
    let mut divergence = 0.0_f64;
    let mut native_loaded = false;
    let tmp = tempfile::tempdir()?;
    for tick in 1..=60 {
        if tick == 31 {
            let (l, a) = ([4.8, 0., 0.], [0., 0., 0.96]);
            pool.apply_impulse_in_field(2, l, a, G, &[])?;
            reference.kick(l, a);
            grouped.kick(l, a);
        }
        pool.advance_adaptive_in_field(G, &[])?;
        let steps = pool.last_substeps().unwrap();
        let mut sum = None;
        for _ in 0..steps {
            let dt = 0.01 / f64::from(steps);
            reference.step(dt);
            grouped.step(dt);
            if let Some(j) = reference.normal_receipt() {
                *sum.get_or_insert(0.) += j;
            }
            for p in reference.points() {
                let j = p.tangent.unwrap();
                assert!(j.length() <= 0.35 * p.impulse + 1e-12);
                assert!(p.lever.is_finite());
                assert_eq!(p.normal.z, 1.);
            }
        }
        let (owner, b) = pool.frame().body(2).unwrap();
        let s = reference.state(1);
        assert_eq!(
            (b.position, b.orientation, b.linear, b.angular),
            (s.position, s.orientation, s.velocity, s.angular_velocity)
        );
        let pairs = &pool.frame().workers[owner as usize].snapshot.impulses;
        assert_eq!(pairs.first().map(|p| p.normal_impulse), sum);
        divergence = divergence.max((b.angular[2] - grouped.state(1).angular_velocity[2]).abs());
        match tick {
            36 => {
                assert!(sum.unwrap() > 0.);
                pool.transfer_component(2, 1)?;
                assert_eq!(pool.frame().body(2).unwrap().0, 1);
                assert_eq!(pool.frame().workers[1].generations[&2], 1);
            }
            37 => {
                let old = pool.frame().clone();
                let pid = pool.worker_pid(owner);
                pool.terminate_worker(owner)?;
                pool.recover(owner)?;
                assert_ne!(pool.worker_pid(owner), pid);
                assert_eq!(pool.frame(), &old);
            }
            38 => {
                let old = pool.frame().clone();
                let path = tmp.path().join("loaded");
                pool.save_native_recovery(&path)?;
                drop(pool);
                let image = PoolNativeRecovery::open(path)?;
                assert_eq!(image.native_solver_profile(), profile);
                let by_owner = image.finite_contact_patches()?;
                let patch = &by_owner[1][0];
                assert_eq!((patch.a, patch.b), (1, 2));
                assert_eq!(patch.points.len(), 4);
                let points = reference.points();
                for stored in &patch.points {
                    let native = points
                        .iter()
                        .find(|p| p.lever.to_array() == stored.lever_b)
                        .ok_or("missing saved corner")?;
                    let table = stored
                        .tangent_impulse_on_a
                        .ok_or("per-point profile lost signed force")?;
                    assert_eq!(table, native.tangent.unwrap().to_array().map(|v| -v));
                    assert_eq!(stored.normal_impulse, native.impulse);
                    assert!(
                        table[0].hypot(table[1]).hypot(table[2])
                            <= 0.35 * stored.normal_impulse + 1e-12
                    );
                }
                pool = image.resume(worker)?;
                assert_eq!(pool.frame(), &old);
                native_loaded = true;
            }
            _ => {}
        }
    }
    assert!(native_loaded && divergence > 1.);
    Ok(())
}
