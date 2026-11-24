// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::simulation::{
    batch_boundary, checkpoint_mixed, install_mixed, queue_environment_impulses,
    restore_mixed_simulation, set_environment_placement,
};
use gz_sim_core::Simulation;
fn adapter() -> String {
    std::env::var("GZ_GPU_ADAPTER").unwrap()
}
fn batch() -> MixedBatch {
    MixedBatch::new(&adapter(), crate::mixed::tests::initial()).unwrap()
}
fn comparable(a: &[Environment], b: &[Environment]) {
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(b) {
        assert_eq!(a.id, b.id);
        for (&av, &bv) in a.state.values().iter().zip(b.state.values()) {
            assert!((av - bv).abs() < 2e-11, "id {}, {av} vs {bv}", a.id);
        }
    }
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn stable_subset_repacking_zero_transfer_cpu_and_pending_checkpoint() {
    let mut b = batch().with_initial_cpu(&[0, 9, 70]).unwrap();
    assert!(b.groups().is_empty());
    let mut oracle = batch();
    for _ in 0..12 {
        let t = b.step().unwrap();
        assert_eq!((t.uploaded_bytes, t.readback_bytes), (0, 0));
        oracle.step().unwrap();
    }
    comparable(b.environments(), oracle.environments());
    let original = b.environments().to_vec();
    for (ids, err) in [(vec![0, 8], false), (vec![9, 9], true)] {
        let failure = b
            .set_placement(12, &ids, ExecutionPlacement::Gpu)
            .unwrap_err();
        assert!(if err {
            matches!(failure, GpuError::DuplicateEnvironment(_))
        } else {
            matches!(failure, GpuError::UnknownEnvironment(_))
        });
        assert!(b.groups().is_empty());
        assert_eq!(b.environments(), original);
    }
    assert!(matches!(
        b.set_placement(11, &[0], ExecutionPlacement::Gpu),
        Err(GpuError::StaleBoundary)
    ));
    let change = b.set_placement(12, &[70], ExecutionPlacement::Gpu).unwrap();
    assert_eq!(
        change,
        PlacementChange {
            moved: 1,
            reseeded_gpu_rows: 1
        }
    );
    assert_eq!(b.groups()[0].offset, 0);
    assert_eq!(b.environments(), original);
    assert_eq!(b.cpu_environments(), 2);
    let bytes = b.active_gpu_bytes();
    assert_eq!(bytes, size_of::<SupportedSpherePairLane>() as u64);
    let metrics = b.step().unwrap();
    assert_eq!(metrics.uploaded_bytes, bytes);
    assert_eq!(
        metrics.readback_bytes,
        bytes + 32 * u64::from(b.device_info().timestamp_queries)
    );
    oracle.step().unwrap();
    comparable(b.environments(), oracle.environments());
    let impulse = CentralImpulse {
        id: 0,
        body: BodySlot::A,
        impulse: [0.1, 0.2, 0.],
    };
    let at = b.tick();
    b.queue_impulses(at, &[impulse]).unwrap();
    oracle.queue_impulses(at, &[impulse]).unwrap();
    b.set_placement(at, &[0], ExecutionPlacement::Gpu).unwrap();
    assert_eq!(b.pending_impulses().len(), 1);
    let before = b.environments().to_vec();
    let m = b.step().unwrap();
    assert!(m.uploaded_bytes >= size_of::<SpherePairLane>() as u64);
    assert_ne!(b.environments(), before);
    oracle.step().unwrap();
    comparable(b.environments(), oracle.environments());
    let at = b.tick();
    b.set_placement(at, &[0, 70], ExecutionPlacement::Cpu)
        .unwrap();
    assert_eq!(b.step().unwrap().readback_bytes, 0);
    b.reset();
    assert_eq!(b.tick(), 0);
    assert_eq!(b.cpu_environments(), 3);
    assert!(b.pending_impulses().next().is_none());
    let mut sim = Simulation::new(STEP).unwrap();
    install_mixed(&mut sim, b).unwrap();
    sim.step_exact(2).unwrap();
    let at = batch_boundary(&sim);
    set_environment_placement(&mut sim, at, &[0, 9], ExecutionPlacement::Gpu).unwrap();
    queue_environment_impulses(&mut sim, at, &[impulse]).unwrap();
    let cp =
        MixedCheckpoint::from_bytes(&checkpoint_mixed(&sim).unwrap().to_bytes().unwrap()).unwrap();
    let mut restored = restore_mixed_simulation(&adapter(), &cp).unwrap();
    assert_eq!(
        restored.world().resource::<MixedBatch>().cpu_environments(),
        1
    );
    assert_eq!(
        restored
            .world()
            .resource::<MixedBatch>()
            .pending_impulses()
            .len(),
        1
    );
    for _ in 0..28 {
        sim.step_exact(1).unwrap();
        restored.step_exact(1).unwrap();
        assert_eq!(
            sim.world().resource::<MixedBatch>().environments(),
            restored.world().resource::<MixedBatch>().environments()
        );
    }
    restored.reset().unwrap();
    assert_eq!(
        restored.world().resource::<MixedBatch>().cpu_environments(),
        3
    );
    assert_eq!(
        restored
            .world()
            .resource::<MixedBatch>()
            .execution_placement(70)
            .unwrap(),
        ExecutionPlacement::Cpu
    );
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn gpu_failure_after_cpu_prediction_keeps_committed_barrier_and_input() {
    let mut b = batch().with_initial_cpu(&[0, 9]).unwrap();
    b.step().unwrap();
    let mut control = batch().with_initial_cpu(&[0, 9]).unwrap();
    control.step().unwrap();
    let input = CentralImpulse {
        id: 0,
        body: BodySlot::A,
        impulse: [1., 0., 0.],
    };
    for owner in [&mut b, &mut control] {
        owner.queue_impulses(1, &[input]).unwrap();
    }
    let previous = b.environments().to_vec();
    b.gpu
        .queue
        .write_buffer(&b.gpu.state, 0, &f64::NAN.to_le_bytes());
    assert!(matches!(
        b.step(),
        Err(GpuError::NonfiniteEnvironment { id: 70, .. })
    ));
    assert_eq!(b.tick(), 1);
    assert_eq!(b.environments(), previous);
    assert_eq!(b.pending_impulses().next(), Some(&input));
    let retry = b.step().unwrap();
    assert_eq!(retry.uploaded_bytes, b.active_gpu_bytes());
    control.step().unwrap();
    assert_eq!(b.environments(), control.environments());
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn replacement_initial_set_drives_both_first_advance_and_full_reset() {
    let mut b = batch()
        .with_initial_cpu(&[0, 9])
        .unwrap()
        .with_initial_cpu(&[70])
        .unwrap();
    let mut control = batch().with_initial_cpu(&[70]).unwrap();
    assert_eq!(b.initial_cpu_environments(), 1);
    assert_eq!(b.execution_placement(9).unwrap(), ExecutionPlacement::Gpu);
    assert_eq!(b.execution_placement(70).unwrap(), ExecutionPlacement::Cpu);
    b.step().unwrap();
    control.step().unwrap();
    assert_eq!(b.environments(), control.environments());
    b.reset();
    assert_eq!(b.execution_placement(70).unwrap(), ExecutionPlacement::Cpu);
}
