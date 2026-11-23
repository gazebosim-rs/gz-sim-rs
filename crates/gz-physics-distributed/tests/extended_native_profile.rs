// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{Body, GroundPlane, NativeSolverProfile, SimulationStep, fixture},
    pool::{PoolNativeRecovery, WorkerPool},
};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::Path};
#[path = "../examples/support/native_profile_grid.rs"]
mod grid;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: [f64; 3] = [0., 0., -9.81];

fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}

fn spawn(
    path: &Path,
    run: u64,
    bodies: Vec<Body>,
    profile: NativeSolverProfile,
) -> Result<WorkerPool> {
    Ok(WorkerPool::spawn_with_profile(
        path,
        run,
        vec![bodies, vec![], vec![]],
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        128,
        profile,
    )?)
}

#[test]
fn warm_64_migrates_with_real_contacts_in_both_encodings_and_restores_an_edited_allocator()
-> Result<()> {
    for packed in [false, true] {
        let mut a = spawn(
            worker(),
            7700,
            grid::grid(64, true),
            NativeSolverProfile::Extended64,
        )?;
        let mut control = spawn(
            worker(),
            7701,
            grid::grid(64, true),
            NativeSolverProfile::Extended64,
        )?;
        a.set_packed_observations(packed);
        assert_eq!(a.set_packed_handoffs(packed), if packed { 3 } else { 0 });
        for _ in 0..3 {
            a.advance_adaptive_in_field(G, &[])?;
            control.advance_adaptive_in_field(G, &[])?;
        }
        let source = &a.frame().workers[0];
        assert_eq!(source.components[0].ids.len(), 64);
        assert!(source.snapshot.impulses.len() >= 64);
        assert!(
            source
                .snapshot
                .impulses
                .iter()
                .any(|p| p.normal_impulse > 0.)
        );
        assert_eq!(source.snapshot.supports.len(), 64);
        let before = source.snapshot.clone();
        a.queue_impulse(1, [0.001, 0., 0.])?;
        control.queue_impulse(1, [0.001, 0., 0.])?;
        let traffic = a.traffic();
        a.transfer_component(1, 1)?;
        let transfer = a.traffic();
        assert_eq!(a.frame().workers[1].snapshot, before);
        assert!(a.frame().workers[1].generations.values().all(|&v| v == 1));
        let extra = Body {
            id: 300,
            position: [100., 0., 10.],
            ..fixture()[1].clone()
        };
        a.insert_body(extra.clone())?;
        a.retire(extra.id)?;
        let dir = tempfile::tempdir()?;
        let file = dir.path().join("native64");
        a.save_native_recovery(&file)?;
        let bytes = fs::read(&file)?;
        assert_eq!(&bytes[..8], b"GZPOOL03");
        let payload: serde_json::Value = serde_json::from_slice(&bytes[40..])?;
        let image = &payload["images"][1]["partition"]["solvers"][0];
        assert_eq!(payload["population_high_water"], extra.id);
        assert_eq!(image["schema"], 3);
        let native = image["native"].as_array().unwrap().len();
        assert!(native < gz_physics_multifidelity::PARTITION_CHECKPOINT_LIMIT);
        println!(
            "packed={packed} bodies=64 pair_receipts={} ground_receipts={} native_bytes={native} archive_bytes={} export_reply_bytes={} prepare_request_bytes={}",
            before.impulses.len(),
            before.supports.len(),
            bytes.len(),
            transfer[0].reply_bytes - traffic[0].reply_bytes,
            transfer[1].request_bytes - traffic[1].request_bytes
        );
        let native_bytes = serde_json::from_value::<Vec<u8>>(image["native"].clone())?;
        assert!(
            gz_physics_multifidelity::MultiFidelityBackend::restore_trusted_solver(&native_bytes)
                .is_err()
        );
        assert!(
            gz_physics_multifidelity::MultiFidelityBackend::restore_solver_with_profile(
                &native_bytes,
                NativeSolverProfile::Extended64
            )
            .is_ok()
        );
        let mut missing_profile = payload.clone();
        missing_profile.as_object_mut().unwrap().remove("profile");
        let mut mixed_profile = payload.clone();
        mixed_profile["images"][1]["partition"]
            .as_object_mut()
            .unwrap()
            .remove("profile");
        for (name, magic, forged) in [
            ("wrong-tag", b"GZPOOL02", payload.clone()),
            ("missing-profile", b"GZPOOL02", missing_profile),
            ("mixed-profile", b"GZPOOL03", mixed_profile),
        ] {
            let contents = serde_json::to_vec(&forged)?;
            let mut replacement = magic.to_vec();
            replacement.extend_from_slice(&Sha256::digest(&contents));
            replacement.extend_from_slice(&contents);
            let path = dir.path().join(name);
            fs::write(&path, replacement)?;
            assert!(PoolNativeRecovery::open(path).is_err());
        }
        let frame = a.frame().clone();
        drop(a);
        let recovery = PoolNativeRecovery::open(file)?;
        assert_eq!(
            recovery.native_solver_profile(),
            NativeSolverProfile::Extended64
        );
        let mut a = recovery.resume(worker())?;
        a.set_packed_observations(packed);
        a.set_packed_handoffs(packed);
        assert_eq!(a.frame(), &frame);
        assert!(a.insert_body(extra).is_err());
        a.drop_next_acknowledgement(1)?;
        assert!(a.advance_adaptive_in_field(G, &[]).is_err());
        assert_eq!(a.frame(), &frame);
        a.advance_adaptive_in_field(G, &[])?;
        control.advance_adaptive_in_field(G, &[])?;
        assert_eq!(
            a.frame().workers[1].snapshot,
            control.frame().workers[0].snapshot
        );
        a.terminate_worker(1)?;
        a.recover(1)?;
        a.advance_adaptive_in_field(G, &[])?;
        control.advance_adaptive_in_field(G, &[])?;
        assert_eq!(
            a.frame().workers[1].snapshot,
            control.frame().workers[0].snapshot
        );
    }
    Ok(())
}

#[test]
fn first_oversize_field_is_refused_without_publishing_or_advancing_workers() -> Result<()> {
    for (count, profile) in [
        (33, NativeSolverProfile::Standard32),
        (65, NativeSolverProfile::Extended64),
    ] {
        let mut pool = spawn(worker(), 7799, grid::grid(count, false), profile)?;
        let frame = pool.frame().clone();
        let traffic = pool.traffic();
        assert!(
            pool.advance_adaptive_in_field([0., 0., -1500.], &[])
                .is_err()
        );
        assert_eq!(pool.frame(), &frame);
        assert!(
            pool.traffic()
                .iter()
                .zip(&traffic)
                .all(|(a, b)| a.requests == b.requests)
        );
        assert_eq!(pool.advance_in_gravity([0.; 3])?.tick, 1);
    }
    Ok(())
}
