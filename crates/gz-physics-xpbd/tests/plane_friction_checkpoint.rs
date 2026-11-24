// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "one-way-checkpoint")]

use gz_physics_xpbd::{Boundary, SoftBody};
use sha2::{Digest, Sha256};
use std::time::Duration;

// Recorded on 5f06f6f. Each digest includes both particles and every signed
// boundary reaction at each of the 25 endpoints, including breakaway.
const SEED: &[u8] = include_bytes!("fixtures/plane_friction_v1.bin");
const ORIGINAL_ID: [u8; 32] = [
    157, 103, 247, 208, 220, 244, 179, 171, 254, 157, 57, 106, 78, 202, 43, 34, 140, 118, 117, 37,
    178, 194, 157, 104, 13, 103, 40, 183, 18, 175, 96, 118,
];

#[test]
fn restores_the_original_contact_equations_and_preserves_the_cold_seed() {
    let original =
        SoftBody::from_one_way_construction(&bincode::deserialize(SEED).unwrap()).unwrap();
    assert_eq!(original.one_way_identity().unwrap(), ORIGINAL_ID);
    assert_eq!(
        bincode::serialize(&original.one_way_construction().unwrap()).unwrap(),
        SEED
    );
    let current = original.clone().with_static_plane_friction(0.45).unwrap();
    assert_ne!(current.one_way_identity().unwrap(), ORIGINAL_ID);
    let current_seed = current.one_way_construction().unwrap();
    let current_bytes = bincode::serialize(&current_seed).unwrap();
    assert_ne!(current_bytes, SEED);
    let reopened = SoftBody::from_one_way_construction(&current_seed).unwrap();
    assert_eq!(
        current.one_way_identity().unwrap(),
        reopened.one_way_identity().unwrap()
    );
    let floor = Boundary::Plane {
        point: [0.; 3],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    };
    let board = Boundary::StaticCuboid {
        center: [0., 0., -0.05],
        half_extents: [2., 0.4, 0.05],
        orientation: [0., 0., 0., 1.],
    };
    for (drive, support, golden) in [
        (
            6.,
            floor,
            "8023afae1592b9287eb3669b90ade220f7edb4e4e0059193c3e83e8b4f3d43e8",
        ),
        (
            6.,
            board,
            "f029925f463b1bbdedce48b3b74742424085a10f7f980bf0e1283570376c8421",
        ),
        (
            2.,
            floor,
            "c437e58dcd1c51dc6d9a1fa78b4b751f6fc14316664c9782ded5a3f50fefadf3",
        ),
        (
            2.,
            board,
            "e38f8fda104ba2935abda7f09af8496d457d490674306bdc1b7917227204ec8c",
        ),
    ] {
        let trace = |mut body: SoftBody| {
            body.set_gravity([drive, 0., -10.]).unwrap();
            let mut hash = Sha256::new();
            for _ in 0..25 {
                body.step(Duration::from_millis(5), &[support]).unwrap();
                hash.update(bincode::serialize(&body.one_way_state().unwrap()).unwrap());
            }
            format!("{:x}", hash.finalize())
        };
        assert_eq!(trace(original.clone()), golden);
        let projected = trace(current.clone());
        assert_ne!(projected, golden);
        assert_eq!(projected, trace(reopened.clone()));
    }
}

#[test]
fn resumed_material_retains_its_trajectory_and_signed_moving_deck_reaction() {
    let mut continuous = SoftBody::from_one_way_construction(&bincode::deserialize(SEED).unwrap())
        .unwrap()
        .with_static_plane_friction(0.45)
        .unwrap();
    let dt = Duration::from_millis(5);
    let deck = |tick: u32| Boundary::TranslatingCuboid {
        center: [0.25 * dt.as_secs_f64() * f64::from(tick), 0., -0.05],
        half_extents: [2., 0.4, 0.05],
        orientation: [0., 0., 0., 1.],
        velocity: [0.25, 0., 0.],
    };
    let construction = bincode::serialize(&continuous.one_way_construction().unwrap()).unwrap();
    for tick in 0..10 {
        continuous.step(dt, &[deck(tick)]).unwrap();
    }
    let state = bincode::serialize(&continuous.one_way_state().unwrap()).unwrap();
    let mut resumed =
        SoftBody::from_one_way_construction(&bincode::deserialize(&construction).unwrap()).unwrap();
    resumed
        .install_one_way_state(bincode::deserialize(&state).unwrap())
        .unwrap();
    assert_eq!(resumed.one_way_identity(), continuous.one_way_identity());
    for tick in 10..35 {
        continuous.step(dt, &[deck(tick)]).unwrap();
        resumed.step(dt, &[deck(tick)]).unwrap();
        assert_eq!(resumed.particles(), continuous.particles());
        assert_eq!(
            resumed.last_boundary_impulses(),
            continuous.last_boundary_impulses()
        );
        assert!(continuous.last_boundary_impulses()[0].linear[2] < 0.);
    }
}

#[test]
fn loaded_rotating_deck_restarts_from_only_material_construction_state_and_external_twist() {
    use glam::{DQuat, DVec3};
    let mut continuous = SoftBody::from_one_way_construction(&bincode::deserialize(SEED).unwrap())
        .unwrap()
        .with_static_plane_friction(0.45)
        .unwrap();
    let construction = continuous.one_way_construction().unwrap();
    let id = continuous.one_way_identity().unwrap();
    let dt = Duration::from_millis(1);
    let deck = |tick: u32| {
        let time = f64::from(tick) * 0.001;
        Boundary::Turntable {
            center: [0.13 * time, 0., -0.05],
            half_extents: [0.5, 0.3, 0.05],
            orientation: DQuat::from_rotation_z(-3.2 * time).to_array(),
            velocity: [0.13, 0., 0.],
            angular_speed: -3.2,
        }
    };
    for t in 0..70 {
        continuous.step(dt, &[deck(t)]).unwrap();
    }
    let bytes = bincode::serialize(&continuous.one_way_state().unwrap()).unwrap();
    let mut resumed = SoftBody::from_one_way_construction(&construction).unwrap();
    resumed
        .install_one_way_state(bincode::deserialize(&bytes).unwrap())
        .unwrap();
    for t in 70..100 {
        continuous.step(dt, &[deck(t)]).unwrap();
        resumed.step(dt, &[deck(t)]).unwrap();
        let j = continuous.last_boundary_impulses()[0];
        assert!(j.linear[2] < 0. && DVec3::from(j.linear).is_finite());
        assert_eq!(resumed.one_way_identity().unwrap(), id);
        assert_eq!(resumed.particles(), continuous.particles());
        assert_eq!(
            resumed.last_boundary_impulses(),
            continuous.last_boundary_impulses()
        );
    }
}

/// Seed and endpoint digests recorded by the projected-friction solver at d0dafcb.
#[test]
fn previous_projected_law_resumes_its_original_stop_and_reversal() {
    let seed = include_bytes!("fixtures/plane_friction_v2.bin");
    let original =
        SoftBody::from_one_way_construction(&bincode::deserialize(seed).unwrap()).unwrap();
    assert_eq!(
        original.one_way_identity().unwrap(),
        [
            237, 2, 221, 122, 11, 223, 145, 72, 18, 205, 164, 209, 33, 166, 19, 24, 237, 181, 149,
            98, 32, 27, 92, 229, 255, 247, 169, 217, 161, 114, 100, 172,
        ]
    );
    assert_eq!(
        bincode::serialize(&original.one_way_construction().unwrap()).unwrap(),
        seed
    );
    let authored = original.clone().with_static_plane_friction(0.6).unwrap();
    assert_ne!(original.one_way_identity(), authored.one_way_identity());
    let reopened =
        SoftBody::from_one_way_construction(&authored.one_way_construction().unwrap()).unwrap();
    for (speed, golden) in [
        (
            0.,
            "c5a13f4b09ceaa72615fa0defe315079521425927c00779a5c194d006df48606",
        ),
        (
            0.02,
            "047c57ee0d54f6ed50b2d14ac5f7cc282591ece0ce5b8efc82491884d0a9795c",
        ),
    ] {
        let trace = |mut body: SoftBody| {
            let floor = Boundary::Plane {
                point: [0.; 3],
                normal: [0., 0., 1.],
                velocity: [speed, 0., 0.],
            };
            let mut hash = Sha256::new();
            for _ in 0..10 {
                body.step(Duration::from_millis(10), &[floor]).unwrap();
                hash.update(bincode::serialize(&body.one_way_state().unwrap()).unwrap());
            }
            format!("{:x}", hash.finalize())
        };
        assert_eq!(trace(original.clone()), golden);
        assert_ne!(trace(authored.clone()), golden);
        assert_eq!(trace(authored.clone()), trace(reopened.clone()));
    }
}
