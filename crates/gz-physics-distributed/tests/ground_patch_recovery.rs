// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{
        Body, GroundContactPatch, GroundPlane, NativeSolverProfile, Primitive, SimulationStep, Tier,
    },
    pool::{PoolNativeRecovery, WorkerPool},
};
use std::path::Path;

fn block(id: u64, x: f64, z: f64) -> Body {
    Body {
        id,
        position: [x, 0., z],
        size: [0.6, 0.4, 0.2],
        mass: 2.,
        friction: 0.35,
        primitive: Primitive::Box,
        tier: Tier::Dynamic,
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    }
}
fn ground(image: &PoolNativeRecovery) -> Vec<(u8, GroundContactPatch)> {
    let mut result = vec![];
    for (i, rows) in image
        .ground_contact_patches()
        .unwrap()
        .into_iter()
        .enumerate()
    {
        assert!(rows.windows(2).all(|w| w[0].body <= w[1].body));
        for r in rows {
            assert_eq!(image.frame().body(r.body).unwrap().0, i as u8);
            assert_eq!(r.normal_on_body, [0., 0., 1.]);
            assert_eq!(r.points.len(), 4);
            assert!(
                r.points
                    .iter()
                    .all(|p| p.plane_witness[2].abs() < 1e-12 && p.normal_impulse > 0.)
            );
            let j = r.points.iter().map(|p| p.normal_impulse).sum::<f64>();
            let w = &image.frame().workers[i];
            let public = w
                .snapshot
                .supports
                .iter()
                .find(|s| s.body == r.body)
                .unwrap()
                .normal_impulse;
            // Each resting assembly ran ten one-millisecond solves this public interval.
            assert!(public > 9. * j && public < 11. * j);
            let mass = if r.body == 101 { 4. } else { 2. };
            assert!((public / (mass * 9.81 * 0.01) - 1.).abs() < 0.01);
            result.push((i as u8, r));
        }
    }
    result.sort_by_key(|(_, r)| r.body);
    assert_eq!(
        result.iter().map(|(_, r)| r.body).collect::<Vec<_>>(),
        [34, 77, 101]
    );
    let pairs = image.finite_contact_patches().unwrap();
    let finite = pairs.iter().flatten().collect::<Vec<_>>();
    assert_eq!(finite.len(), 1);
    assert_eq!((finite[0].a, finite[0].b), (99, 101));
    assert_eq!(finite[0].points.len(), 4);
    result
}

#[test]
fn archived_shared_plane_and_finite_manifolds_remap_across_resident_worlds() {
    check_archived_patches(NativeSolverProfile::Standard32);
}

#[test]
fn per_point_shared_plane_and_finite_manifolds_survive_transfer_death_and_reload() {
    check_archived_patches(NativeSolverProfile::Standard32PerPoint);
}

fn check_archived_patches(profile: NativeSolverProfile) {
    let temp = tempfile::tempdir().unwrap();
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let plane = GroundPlane::oriented([0.; 3], [0., 0., 1.], 0.65).unwrap();
    let mut p = WorkerPool::spawn_with_profile(
        worker,
        0x4719,
        vec![
            vec![
                block(101, 0., 0.105),
                block(99, 0., 1.305),
                block(77, -12., 0.105),
            ],
            vec![],
            vec![block(34, 18., 0.105)],
        ],
        SimulationStep::Ms10,
        Some(plane),
        128,
        profile,
    )
    .unwrap();
    let save = |pool: &mut WorkerPool, name: &str| {
        let file = temp.path().join(name);
        pool.save_native_recovery(&file).unwrap();
        let image = PoolNativeRecovery::open(&file).unwrap();
        assert_eq!(image.native_solver_profile(), profile);
        image
    };
    let seed = save(&mut p, "at-seed");
    assert!(
        seed.ground_contact_patches()
            .unwrap()
            .iter()
            .all(Vec::is_empty)
    );
    for _ in 0..100 {
        p.advance_adaptive_in_field([0., 0., -9.81], &[]).unwrap();
    }
    assert_eq!(p.last_substeps(), Some(10));
    let before = ground(&save(&mut p, "loaded"));
    assert_eq!(
        before.iter().map(|(i, _)| *i).collect::<Vec<_>>(),
        [2, 0, 0]
    );
    p.transfer_component(101, 2).unwrap();
    let moved = save(&mut p, "after-move");
    let after = ground(&moved);
    assert_eq!(after.iter().map(|(i, _)| *i).collect::<Vec<_>>(), [2, 0, 2]);
    assert_eq!(
        before.iter().map(|(_, r)| r).collect::<Vec<_>>(),
        after.iter().map(|(_, r)| r).collect::<Vec<_>>()
    );
    p.terminate_worker(2).unwrap();
    p.recover(2).unwrap();
    assert_eq!(ground(&save(&mut p, "replaced")), after);
    drop(p);
    let mut resumed = moved.resume(worker).unwrap();
    assert_eq!(ground(&save(&mut resumed, "fresh")), after);
    // A released world can retain broad-phase contact candidates. Once the
    // body is airborne, only its two stationary neighbors may report a patch.
    let gravity = [0., 0., -9.81];
    resumed
        .apply_impulse_in_field(34, [2., 0., 8.], [0.; 3], gravity, &[])
        .unwrap();
    for _ in 0..10 {
        resumed.advance_adaptive_in_field(gravity, &[]).unwrap();
    }
    let flight = save(&mut resumed, "airborne");
    assert!(flight.frame().body(34).unwrap().1.position[2] > 0.4);
    let airborne_rows = flight.ground_contact_patches().unwrap();
    let mut others = airborne_rows
        .iter()
        .flatten()
        .map(|r| r.body)
        .collect::<Vec<_>>();
    others.sort();
    assert_eq!(others, [77, 101]);
    for _ in 0..2 {
        resumed.compact().unwrap();
        for _ in 0..80 {
            resumed.advance_adaptive_in_field(gravity, &[]).unwrap();
        }
    }
    let landed = ground(&save(&mut resumed, "landed"));
    let xs = |patch: &GroundContactPatch| {
        patch.points.iter().map(|p| p.plane_witness[0]).sum::<f64>() / patch.points.len() as f64
    };
    assert!(
        xs(&landed[0].1) - xs(&before[0].1) > 0.5,
        "fresh ground footprint after flight"
    );
}
