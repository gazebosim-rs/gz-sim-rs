// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{
        Body, GroundPlane, NativeFrictionModel, NativeSolverProfile, SimulationStep, fixture,
    },
    pool::{NativeContactSnapshot, PoolNativeRecovery, WorkerPool},
};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn write_envelope(path: &Path, magic: &[u8; 8], payload: &Value) -> Result<()> {
    let bytes = serde_json::to_vec(payload)?;
    let mut envelope = magic.to_vec();
    envelope.extend(Sha256::digest(&bytes));
    envelope.extend(bytes);
    fs::write(path, envelope)?;
    Ok(())
}
fn save_cut(
    dir: &Path,
    profile: NativeSolverProfile,
    packed: bool,
) -> Result<(std::path::PathBuf, Value)> {
    let b = Body {
        friction: 0.6,
        position: [0., 0., 0.32],
        linear: [0.; 3],
        ..fixture()[0].promoted()
    };
    let mut p = WorkerPool::spawn_with_profile(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        7113,
        vec![vec![b.clone()], vec![], vec![]],
        SimulationStep::Ms10,
        Some(GroundPlane::with_friction(0., 0.6)?),
        64,
        profile,
    )?;
    p.set_packed_handoffs(packed);
    p.set_packed_observations(packed);
    for _ in 0..40 {
        p.advance_adaptive_in_field([0., 0., -9.81], &[])?;
    }
    assert!(p.frame().workers[0].snapshot.supports[0].normal_impulse > 0.);
    p.transfer_component(b.id, 1)?;
    p.terminate_worker(1)?;
    p.recover(1)?;
    let extra = Body {
        id: 12,
        position: [10., 0., 1.],
        ..b
    };
    p.insert_body(extra)?;
    p.retire(12)?;
    let file = dir.join(format!("{profile:?}-{packed}.gzpool"));
    p.save_native_recovery(&file)?;
    let bytes = fs::read(&file)?;
    let magic = if profile.friction() == NativeFrictionModel::PerPoint {
        b"GZPOOL04"
    } else if profile == NativeSolverProfile::Standard32 {
        b"GZPOOL02"
    } else {
        b"GZPOOL03"
    };
    assert_eq!(&bytes[..8], magic);
    let plan = PoolNativeRecovery::open(&file)?;
    assert_eq!(plan.population_high_water(), Some(12));
    assert_eq!(plan.native_solver_profile(), profile);
    assert_eq!(plan.frame(), p.frame());
    assert_eq!(plan.ground_contact_patches()?[1][0].points.len(), 4);
    assert_eq!(NativeContactSnapshot::open(&file)?.frame, *p.frame());
    let mut fresh = plan.resume(Path::new(env!("CARGO_BIN_EXE_physics_worker")))?;
    for _ in 0..4 {
        assert_eq!(
            fresh.advance_adaptive_in_field([0., 0., -9.81], &[])?,
            p.advance_adaptive_in_field([0., 0., -9.81], &[])?
        );
    }
    Ok((file, serde_json::from_slice(&bytes[40..])?))
}
#[test]
fn point_envelope_rejects_retagged_profiles_and_genuine_mixed_worker_images() -> Result<()> {
    let dir = tempfile::tempdir()?;
    for capacity in [
        NativeSolverProfile::Standard32,
        NativeSolverProfile::Extended64,
    ] {
        let point = capacity.with_friction(NativeFrictionModel::PerPoint);
        for packed in [false, true] {
            let (_, group) = save_cut(dir.path(), capacity, packed)?;
            let (_, data) = save_cut(dir.path(), point, packed)?;
            let wrong = dir.path().join("wrong");
            for tag in [b"GZPOOL01", b"GZPOOL02", b"GZPOOL03"] {
                write_envelope(&wrong, tag, &data)?;
                assert!(PoolNativeRecovery::open(&wrong).is_err());
                assert!(NativeContactSnapshot::open(&wrong).is_err());
            }
            // Both per-point capacities share the new envelope. The typed
            // barrier must also agree with every sealed worker image.
            let other = if capacity == NativeSolverProfile::Standard32 {
                NativeSolverProfile::Extended64
            } else {
                NativeSolverProfile::Standard32
            }
            .with_friction(NativeFrictionModel::PerPoint);
            let mut wrong_capacity = data.clone();
            wrong_capacity["profile"] = serde_json::to_value(other)?;
            write_envelope(&wrong, b"GZPOOL04", &wrong_capacity)?;
            assert!(PoolNativeRecovery::open(&wrong).is_err());
            write_envelope(&wrong, b"GZPOOL04", &group)?;
            assert!(PoolNativeRecovery::open(&wrong).is_err());
            for owner in 0..3 {
                // The forged outer archive is checksummed correctly. Each
                // substituted inner image is genuine and matches this run,
                // worker slot and clock, even for the empty nonparticipant.
                let mut forged = data.clone();
                forged["images"][owner] = group["images"][owner].clone();
                write_envelope(&wrong, b"GZPOOL04", &forged)?;
                assert!(PoolNativeRecovery::open(&wrong).is_err());
            }
        }
    }
    Ok(())
}
