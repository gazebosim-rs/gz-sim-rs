// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const PROFILES: [NativeSolverProfile; 4] = [
    NativeSolverProfile::Standard32,
    NativeSolverProfile::Extended64,
    NativeSolverProfile::Standard32PerPoint,
    NativeSolverProfile::Extended64PerPoint,
];

fn block(x: f64) -> BodySpec {
    BodySpec {
        position: [x, 0., 0.14],
        velocity: [0.; 3],
        mass: Some(2.),
        shape: Shape::Box {
            half_extents: [0.4, 0.2, 0.1],
        },
    }
}
fn advanced(backend: &mut RapierBackend) -> Result<(), PhysicsError> {
    backend.set_partition_gravity([0., 0., -9.81])?;
    backend.step(std::time::Duration::from_millis(1))?;
    backend.set_partition_gravity([0.; 3])
}

#[test]
fn capacity_and_friction_are_independent_and_legacy_discriminants_stay_stable() {
    for (i, profile) in PROFILES.into_iter().enumerate() {
        assert_eq!(
            bincode::serialize(&profile).unwrap(),
            (i as u32).to_le_bytes()
        );
        assert_eq!(profile.max_bodies(), if i % 2 == 0 { 32 } else { 64 });
        assert_eq!(profile.capacity_profile(), PROFILES[i % 2]);
        assert_eq!(
            profile.with_friction(NativeFrictionModel::Grouped),
            PROFILES[i % 2]
        );
        assert_eq!(
            profile.with_friction(NativeFrictionModel::PerPoint),
            PROFILES[i % 2 + 2]
        );
        assert_eq!(profile, profile.with_friction(profile.friction()));
    }
}
#[test]
fn loaded_archive_requires_matching_header_and_solver_then_tracks_slide_and_yaw_exactly()
-> Result<(), PhysicsError> {
    let mut final_x = Vec::new();
    for profile in PROFILES {
        let mut source = RapierBackend::new_multifidelity_with_profile([0.; 3], profile)?;
        source.set_partition_ground_material(0., 0.65)?;
        let id = source.spawn_partition_box(block(0.), Default::default(), false, 0.35)?;
        for _ in 0..230 {
            advanced(&mut source)?;
        }
        assert!(
            source
                .partition_ground_contacts()
                .iter()
                .any(|c| c.normal_impulse > 0.)
        );
        source.apply_impulse(id, [4.8, 0., 0.], [0., 0., 0.96])?;
        for _ in 0..23 {
            advanced(&mut source)?;
        }
        let image = source.checkpoint_solver_with_profile(profile)?;
        for other in PROFILES.into_iter().filter(|other| *other != profile) {
            assert!(source.checkpoint_solver_with_profile(other).is_err());
            assert!(RapierBackend::restore_solver_with_profile(&image, other).is_err());
        }
        // Changing just the format bit can never relabel an existing world.
        let mut altered = image.clone();
        let tag = u32::from_le_bytes(image[..4].try_into().unwrap()) ^ (1 << 29);
        altered[..4].copy_from_slice(&tag.to_le_bytes());
        let other = if profile.friction() == NativeFrictionModel::Grouped {
            NativeFrictionModel::PerPoint
        } else {
            NativeFrictionModel::Grouped
        };
        assert!(
            RapierBackend::restore_solver_with_profile(&altered, profile.with_friction(other))
                .is_err()
        );
        let mut resume = RapierBackend::restore_solver_with_profile(&image, profile)?;
        for wrong in PROFILES.into_iter().filter(|p| *p != profile) {
            assert!(resume.checkpoint_solver_with_profile(wrong).is_err());
        }
        for _ in 0..256 {
            advanced(&mut source)?;
            advanced(&mut resume)?;
            let observed = |world: &RapierBackend| {
                let s = world.state(id).unwrap();
                s.position
                    .into_iter()
                    .chain(s.orientation)
                    .chain(s.velocity)
                    .chain(s.angular_velocity)
                    .chain(
                        world
                            .partition_ground_contacts()
                            .into_iter()
                            .map(|c| c.normal_impulse),
                    )
                    .map(f64::to_bits)
                    .collect::<Vec<_>>()
            };
            assert_eq!(observed(&source), observed(&resume));
        }
        final_x.push(source.state(id).unwrap().position[0]);
    }
    assert_eq!(final_x[0].to_bits(), final_x[1].to_bits());
    assert_eq!(final_x[2].to_bits(), final_x[3].to_bits());
    assert!((final_x[2] - final_x[0]).abs() > 0.01);
    Ok(())
}

#[test]
fn disjoint_join_accepts_same_friction_and_refuses_each_mixed_placement() -> Result<(), PhysicsError>
{
    for profile in PROFILES {
        let source =
            |offset: f64, mode: NativeFrictionModel| -> Result<RapierBackend, PhysicsError> {
                let mut b = RapierBackend::new_multifidelity_with_profile(
                    [0.; 3],
                    profile.with_friction(mode),
                )?;
                b.spawn_partition_box(block(offset), Default::default(), false, 0.35)?;
                advanced(&mut b)?;
                Ok(b)
            };
        let a = source(0., profile.friction())?;
        let b = source(3., profile.friction())?;
        let other = if profile.friction() == NativeFrictionModel::Grouped {
            NativeFrictionModel::PerPoint
        } else {
            NativeFrictionModel::Grouped
        };
        let foreign = source(6., other)?;
        let (mut joined, maps) = a.join_box_solvers_with_profile(&[&b], profile)?;
        assert_eq!(maps[0].len(), 1);
        advanced(&mut joined)?;
        joined.checkpoint_solver_with_profile(profile)?;
        assert!(
            a.join_box_solvers_with_profile(&[&foreign], profile)
                .is_err()
        );
        assert!(
            foreign
                .join_box_solvers_with_profile(&[&a], profile)
                .is_err()
        );
    }
    Ok(())
}

/// Qualify files produced by earlier checkouts without making external campaign data
/// a dependency of the normal test suite. The input is a list of trusted local archives.
#[test]
#[ignore = "set GZ_LEGACY_POOL_ARCHIVES to paths of pre-policy GZPOOL01/02/03 archives"]
fn old_grouped_native_payloads_reseal_byte_for_byte() -> Result<(), Box<dyn std::error::Error>> {
    use sha2::{Digest, Sha256};
    let paths = std::env::var_os("GZ_LEGACY_POOL_ARCHIVES").ok_or("missing legacy archives")?;
    let mut capacities = [false; 2];
    for path in std::env::split_paths(&paths) {
        let file = std::fs::read(&path)?;
        let (profile, capacity) = match file.get(..8) {
            Some(b"GZPOOL01" | b"GZPOOL02") => (NativeSolverProfile::Standard32, 0),
            Some(b"GZPOOL03") => (NativeSolverProfile::Extended64, 1),
            _ => return Err("expected a historical grouped pool file".into()),
        };
        let payload = file.get(40..).ok_or("truncated pool file")?;
        let metadata: serde_json::Value = serde_json::from_slice(payload)?;
        let mut count = 0;
        for worker in metadata["images"]
            .as_array()
            .ok_or("missing worker images")?
        {
            for island in worker["partition"]["solvers"]
                .as_array()
                .ok_or("missing solvers")?
            {
                let bytes: Vec<u8> = serde_json::from_value(island["native"].clone())?;
                let world = RapierBackend::restore_solver_with_profile(&bytes, profile)?;
                assert_eq!(world.checkpoint_solver_with_profile(profile)?, bytes);
                assert!(
                    RapierBackend::restore_solver_with_profile(
                        &bytes,
                        profile.with_friction(NativeFrictionModel::PerPoint)
                    )
                    .is_err()
                );
                count += 1;
            }
        }
        assert!(count > 0);
        capacities[capacity] = true;
        println!(
            "archive={} sha256={:x} profile={profile:?} exact_payloads={count}",
            path.display(),
            Sha256::digest(&file)
        );
    }
    assert_eq!(capacities, [true, true]);
    Ok(())
}
