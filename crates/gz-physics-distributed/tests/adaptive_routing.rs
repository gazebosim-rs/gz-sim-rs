// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    adaptive_substeps,
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::{Body, BodyWrench, PendingImpulse, Primitive, SimulationStep, Tier, fixture},
    pool::{PoolNativeRecovery, WorkerPool},
    transport::WorkerProcess,
};
use std::{error::Error, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn boxes() -> [Body; 2] {
    [1, 2].map(|id| Body {
        id,
        position: [(id - 1) as f64 * 0.6, 0., 0.],
        linear: [0.; 3],
        size: [0.5; 3],
        mass: 1.,
        ..fixture()[0].promoted()
    })
}
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}

#[test]
fn selection_accounts_for_accepted_inputs_and_relative_acceleration() -> Result<()> {
    let [a, b] = boxes();
    let step = SimulationStep::Ms10;
    let inputs = [PendingImpulse {
        id: 1,
        impulse: [20., 0., 0.],
    }];
    let held = [BodyWrench {
        id: 1,
        force: [2000., 0., 0.],
        torque: [0.; 3],
    }];
    let both = [a.clone(), b.clone()];
    assert_eq!(adaptive_substeps(&both, step, &[], [0.; 3], &[])?, 1);
    assert_eq!(
        adaptive_substeps(&both, step, &[], [2000., 0., 0.], &[])?,
        1
    );
    assert_eq!(adaptive_substeps(&both, step, &inputs, [0.; 3], &[])?, 10);
    assert_eq!(
        adaptive_substeps(&[b.clone(), a.clone()], step, &inputs, [0.; 3], &[])?,
        10
    );
    assert_eq!(adaptive_substeps(&both, step, &[], [0.; 3], &held)?, 10);
    assert!(adaptive_substeps(&both, SimulationStep::Ms5, &[], [0.; 3], &[]).is_err());
    assert!(adaptive_substeps(&[a.clone(), a.clone()], step, &[], [0.; 3], &[]).is_err());
    let sphere = Body {
        primitive: Primitive::Sphere,
        ..b.clone()
    };
    assert_eq!(
        adaptive_substeps(&[a.clone(), sphere], step, &inputs, [0.; 3], &[])?,
        10
    );
    for bad in [
        Body {
            mass: 0.,
            ..a.clone()
        },
        Body {
            linear: [f64::NAN; 3],
            ..a.clone()
        },
        Body {
            orientation: [0.; 4],
            ..a.clone()
        },
        Body {
            size: [-0.5; 3],
            ..a.clone()
        },
    ] {
        assert!(adaptive_substeps(&[bad], step, &[], [0.; 3], &[]).is_err());
    }
    let unrealizable = [PendingImpulse {
        id: 1,
        impulse: [401., 0., 0.],
    }];
    assert!(adaptive_substeps(&both, step, &unrealizable, [0.; 3], &[]).is_err());
    let fixed = Body {
        tier: Tier::Fixed,
        mass: 0.,
        ..a.clone()
    };
    let prescribed = Body {
        tier: Tier::Prescribed,
        linear: [-20., 0., 0.],
        ..b
    };
    assert_eq!(
        adaptive_substeps(&[fixed.clone(), prescribed], step, &[], [0.; 3], &[])?,
        1
    );
    assert!(adaptive_substeps(&[fixed], step, &inputs, [0.; 3], &[]).is_err());
    Ok(())
}

#[test]
fn lost_refined_ack_preserves_barrier_and_native_resume_width() -> Result<()> {
    let stars = boxes().to_vec();
    let seeds = vec![stars, vec![], vec![]];
    let start = |run| {
        WorkerPool::spawn(
            worker(),
            run,
            seeds.clone(),
            SimulationStep::Ms10,
            None,
            128,
        )
    };
    let mut pool = start(0xa320)?;
    let mut control = start(0xa321)?;
    let loads = [BodyWrench {
        id: 1,
        force: [0.1, 0., 0.],
        torque: [0.; 3],
    }];
    for p in [&mut pool, &mut control] {
        p.queue_impulse(1, [20., 0., 0.])?;
        assert_eq!(p.adaptive_substeps([0.; 3], &loads)?, 10);
    }
    let before = pool.frame().clone();
    pool.drop_next_acknowledgement(0)?;
    assert!(pool.advance_adaptive_in_field([0.; 3], &loads).is_err());
    assert_eq!(pool.frame(), &before);
    assert_eq!(pool.last_substeps(), None);
    assert_eq!(
        pool.advance_adaptive_in_field([0.; 3], &loads)?,
        control.advance_adaptive_in_field([0.; 3], &loads)?
    );
    assert_eq!(pool.last_substeps(), Some(10));
    assert!(
        pool.frame().workers[0]
            .snapshot
            .impulses
            .iter()
            .any(|p| p.normal_impulse > 1.)
    );
    let files = tempfile::tempdir()?;
    let checkpoint = files.path().join("pool");
    pool.save_native_recovery(&checkpoint)?;
    drop(pool);
    let mut restored = PoolNativeRecovery::open(&checkpoint)?.resume(worker())?;
    assert_eq!(restored.last_substeps(), Some(10));
    assert_eq!(
        restored.advance_in_field([0.; 3], &[])?,
        control.advance_in_field([0.; 3], &[])?
    );
    assert_eq!(restored.last_substeps(), Some(1));
    Ok(())
}

#[test]
fn recorded_and_native_recovery_keep_the_two_worker_refinement_decision() -> Result<()> {
    let [a, b] = boxes();
    let peers = [
        WorkerProcess::spawn(worker(), 0xa322, 0, &[a])?,
        WorkerProcess::spawn(worker(), 0xa322, 1, &[b])?,
    ];
    let mut world = Coordinator::new_recorded(0xa322, peers, 128)?;
    world.queue_impulse_routed(1, [20., 0., 0.])?;
    world.advance_adaptive_in_field([0.; 3], &[])?;
    assert_eq!(world.last_substeps(), Some(10));
    assert_eq!(
        world.frame().body(1).unwrap().0,
        world.frame().body(2).unwrap().0
    );
    let before = world.frame().clone();
    let files = tempfile::tempdir()?;
    let journal = files.path().join("journal");
    let native = files.path().join("native");
    world.save_recovery(&journal)?;
    world.save_native_recovery(&native)?;
    drop(world);
    let mut replay = RecoveryPlan::open(&journal)?
        .resume(|run, i, b| WorkerProcess::spawn(worker(), run, i, b))?;
    assert_eq!(replay.frame(), &before);
    assert_eq!(replay.last_substeps(), Some(10));
    let next = replay.advance_adaptive_in_field([0.; 3], &[])?.clone();
    drop(replay);
    let mut resumed = NativeRecoveryPlan::open(&native)?.resume(worker())?;
    assert_eq!(resumed.frame(), &before);
    assert_eq!(resumed.last_substeps(), Some(10));
    assert_eq!(resumed.advance_adaptive_in_field([0.; 3], &[])?, &next);
    Ok(())
}

#[test]
fn swept_curved_and_mixed_pairs_catch_between_barrier_hits_and_clear_near_misses() -> Result<()> {
    let kinds = [
        Primitive::Box,
        Primitive::Sphere,
        Primitive::Capsule,
        Primitive::Cylinder,
    ];
    let select = |b: &[Body]| adaptive_substeps(b, SimulationStep::Ms10, &[], [0.; 3], &[]);
    for (i, left) in kinds.into_iter().enumerate() {
        for &right in &kinds[i..] {
            if left == Primitive::Box && right == Primitive::Box {
                continue;
            }
            let [mut a, mut b] = boxes();
            (a.primitive, b.primitive) = (left, right);
            for body in [&mut a, &mut b] {
                body.size = match body.primitive {
                    Primitive::Sphere => [0.2; 3],
                    _ => [0.2, 0.2, 2.],
                };
            }
            // Even a contact-free pair at both public endpoints can collide midway.
            b.position = [-0.5, 0.19, 0.];
            b.linear = [100., 0., 0.];
            assert_eq!(select(&[a.clone(), b.clone()])?, 10, "{left:?} / {right:?}");
            assert_eq!(select(&[b.clone(), a.clone()])?, 10);
            // Parallel lanes with clearance, despite overlapping circumspheres.
            b.position[1] = 0.24;
            assert_eq!(select(&[a.clone(), b.clone()])?, 1, "{left:?} / {right:?}");
            assert_eq!(select(&[b, a])?, 1);
        }
    }
    Ok(())
}

#[test]
fn curved_axial_extents_follow_orientation_spin_and_pending_loads() -> Result<()> {
    let [mut target, mut axial] = boxes();
    target.primitive = Primitive::Sphere;
    target.size = [0.10; 3];
    target.position = [0., 0.92, 0.];
    axial.position = [0.; 3];
    axial.size = [0.2, 0.2, 2.];
    let step = SimulationStep::Ms10;
    for shape in [Primitive::Cylinder, Primitive::Capsule] {
        axial.primitive = shape;
        let select = |target: &Body, axial: &Body| {
            adaptive_substeps(&[target.clone(), axial.clone()], step, &[], [0.; 3], &[])
        };
        assert_eq!(select(&target, &axial)?, 1);
        let mut rotated = axial.clone();
        let h = std::f64::consts::FRAC_1_SQRT_2;
        rotated.orientation = [h, 0., 0., h];
        assert_eq!(select(&target, &rotated)?, 10);
        for tier in [Tier::Dynamic, Tier::Prescribed] {
            let spin = Body {
                angular: [157., 0., 0.],
                tier,
                ..axial.clone()
            };
            assert_eq!(select(&target, &spin)?, 10);
        }
        let pair = [target.clone(), axial.clone()];
        assert_eq!(
            adaptive_substeps(&pair, step, &[], [0., -10000., 0.], &[])?,
            1
        );
        let kick = [PendingImpulse {
            id: target.id,
            impulse: [0., -100., 0.],
        }];
        assert_eq!(adaptive_substeps(&pair, step, &kick, [0.; 3], &[])?, 10);
        let wrench = [BodyWrench {
            id: axial.id,
            force: [0.; 3],
            torque: [100., 0., 0.],
        }];
        assert_eq!(adaptive_substeps(&pair, step, &[], [0.; 3], &wrench)?, 10);
    }
    Ok(())
}

#[test]
fn sphere_spin_leaves_close_diagonal_clearances_unchanged() -> Result<()> {
    let [mut a, mut b] = boxes();
    for body in [&mut a, &mut b] {
        body.primitive = Primitive::Sphere;
        body.angular = [2., -5., 1.];
    }
    b.position = [0.38, 0.38, 0.];
    let wrench = [BodyWrench {
        id: a.id,
        force: [0.; 3],
        torque: [2.; 3],
    }];
    assert_eq!(
        adaptive_substeps(&[a, b], SimulationStep::Ms10, &[], [0.; 3], &wrench)?,
        1
    );
    Ok(())
}
