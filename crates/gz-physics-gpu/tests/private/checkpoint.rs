// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::simulation::{
    batch_boundary, checkpoint_mixed, mixed_batch_simulation, queue_environment_impulses,
    reset_environments, restore_mixed_simulation,
};
use gz_sim_core::Simulation;
#[test]
fn checkpoint_and_edits_require_a_mixed_physics_world() {
    let mut sim = Simulation::new(STEP).unwrap();
    let at = batch_boundary(&sim);
    assert!(checkpoint_mixed(&sim).is_err());
    assert!(reset_environments(&mut sim, at, &[]).is_err());
    assert!(queue_environment_impulses(&mut sim, at, &[]).is_err());
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn malformed_replay_data_refuses_topology_commands_clock_and_incompatible_device() {
    let name = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut sim = mixed_batch_simulation(&name, crate::mixed::tests::initial()).unwrap();
    sim.step_exact(2).unwrap();
    let at = batch_boundary(&sim);
    let impulse = CentralImpulse {
        id: 0,
        body: BodySlot::A,
        impulse: [0.1, 0., 0.],
    };
    queue_environment_impulses(&mut sim, at, &[impulse]).unwrap();
    let snap = checkpoint_mixed(&sim).unwrap();
    // The v1 wire layout is exactly the original five fields. A valid old
    // record has implicit all-GPU placement. Instrument compatibility remains
    // checked separately on restore by the device and kernel signature.
    let mut legacy = V1.to_vec();
    codec()
        .serialize_into(
            &mut legacy,
            &(
                &snap.initial,
                &snap.committed,
                &snap.pending,
                &snap.clock,
                &snap.target,
            ),
        )
        .unwrap();
    legacy.extend_from_slice(&Sha256::digest(&legacy));
    let original = MixedCheckpoint::from_bytes(&legacy).unwrap();
    assert!(original.initial_cpu.is_empty() && original.cpu.is_empty());
    assert_eq!(original.committed, snap.committed);
    for owners in [vec![0, 0], vec![9, 0], vec![u64::MAX]] {
        let mut damaged = snap.clone();
        damaged.cpu = owners.clone();
        assert!(MixedCheckpoint::from_bytes(&damaged.to_bytes().unwrap()).is_err());
        damaged.cpu.clear();
        damaged.initial_cpu = owners;
        assert!(MixedCheckpoint::from_bytes(&damaged.to_bytes().unwrap()).is_err());
    }
    let b = sim.world().resource::<MixedBatch>();
    assert_eq!(
        b.get_many(&[70, 9, 70])
            .unwrap()
            .into_iter()
            .map(|e| e.id)
            .collect::<Vec<_>>(),
        [70, 9, 70]
    );
    assert!(matches!(
        b.get_many(&[70, 71]),
        Err(GpuError::UnknownEnvironment(71))
    ));
    let mut damaged = snap.clone();
    damaged.initial.clear();
    assert!(MixedCheckpoint::from_bytes(&damaged.to_bytes().unwrap()).is_err());
    damaged = snap.clone();
    damaged.initial.swap(0, 1);
    assert!(damaged.validate().is_err());
    damaged = snap.clone();
    damaged.committed.swap(0, 1);
    assert!(damaged.validate().is_err());
    damaged = snap.clone();
    damaged.committed[0].state.bytes_mut()[..8].copy_from_slice(&f64::NAN.to_le_bytes());
    assert!(damaged.validate().is_err());
    damaged = snap.clone();
    damaged.pending.push(impulse);
    assert!(damaged.validate().is_err());
    damaged = snap.clone();
    damaged.pending[0].id = u64::MAX;
    assert!(damaged.validate().is_err());
    damaged = snap.clone();
    damaged.pending[0].impulse[1] = f64::INFINITY;
    assert!(damaged.validate().is_err());
    damaged = snap.clone();
    damaged.clock.tick += 1;
    assert!(restore_mixed_simulation(&name, &damaged).is_err());
    damaged = snap.clone();
    damaged.target.driver_info.push('x');
    assert!(restore_mixed_simulation(&name, &damaged).is_err());
    let mut malformed = MAGIC.to_vec();
    malformed.extend_from_slice(&[0xff; 8]);
    let digest = Sha256::digest(&malformed);
    malformed.extend_from_slice(&digest);
    assert!(MixedCheckpoint::from_bytes(&malformed).is_err());
    // Direct low-level advancement is observable if someone bypasses the clock owner.
    sim.world_mut().resource_mut::<MixedBatch>().step().unwrap();
    assert!(matches!(
        checkpoint_mixed(&sim),
        Err(GpuError::StaleBoundary)
    ));
}
