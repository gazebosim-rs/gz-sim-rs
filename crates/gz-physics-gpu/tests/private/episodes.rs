// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::simulation::{
    BatchTiming, batch_boundary, checkpoint_mixed, mixed_batch_simulation,
    queue_environment_impulses, reset_environments, restore_mixed_simulation,
};
use gz_sim_core::{Simulation, UpdateInfo};
fn impulse(id: u64, x: f64) -> CentralImpulse {
    CentralImpulse {
        id,
        body: BodySlot::A,
        impulse: [x, 0., 0.],
    }
}
fn batch(sim: &Simulation) -> &MixedBatch {
    sim.world().resource()
}
#[test]
fn stateless_draws_are_domain_separated_repeatable_and_in_range() {
    use crate::sampling::{counter, uniform};
    let value = counter(17, 42, 9, 3, 6);
    assert_eq!(value, counter(17, 42, 9, 3, 6));
    for x in [
        counter(18, 42, 9, 3, 6),
        counter(17, 43, 9, 3, 6),
        counter(17, 42, 10, 3, 6),
        counter(17, 42, 9, 4, 6),
        counter(17, 42, 9, 3, 7),
    ] {
        assert_ne!(x, value);
    }
    for k in 0..100 {
        assert!((0.0..1.).contains(&uniform(8, 4, 6, 5, k)));
    }
    assert_eq!(counter(0, 0, 0, 0, 0), 0x2130748aaac80268);
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn sparse_resets_validate_all_ids_and_preserve_unrelated_physics_and_pending_inputs() {
    let seed = initial();
    let mut sim = mixed_batch_simulation(&selector(), seed.clone()).unwrap();
    let mut baseline = mixed_batch_simulation(&selector(), seed.clone()).unwrap();
    sim.step_exact(4).unwrap();
    baseline.step_exact(4).unwrap();
    let at = batch_boundary(&sim);
    let pending = [impulse(0, 0.3), impulse(9, -0.1)];
    queue_environment_impulses(&mut sim, at, &pending).unwrap();
    queue_environment_impulses(&mut baseline, at, &pending[..1]).unwrap();
    queue_environment_impulses(&mut sim, at, &[]).unwrap();
    assert!(matches!(
        queue_environment_impulses(&mut sim, at, &[impulse(70, 0.1), impulse(0, 0.1)]),
        Err(GpuError::DuplicateEnvironment(0))
    ));
    let before = checkpoint_mixed(&sim).unwrap().to_bytes().unwrap();
    let mut duplicate = seed[0];
    duplicate.id = 100;
    assert!(matches!(
        reset_environments(&mut sim, at, &[seed[2], duplicate]),
        Err(GpuError::UnknownEnvironment(100))
    ));
    assert!(matches!(
        reset_environments(&mut sim, at, &[seed[2], seed[2]]),
        Err(GpuError::DuplicateEnvironment(9))
    ));
    duplicate.id = 9;
    assert!(matches!(
        reset_environments(&mut sim, at, &[duplicate]),
        Err(GpuError::ProfileChange(9))
    ));
    let mut invalid = seed[2];
    let State::SpherePlane(p) = &mut invalid.state else {
        unreachable!()
    };
    p.radius = 0.;
    assert!(matches!(
        reset_environments(&mut sim, at, &[invalid]),
        Err(GpuError::InvalidEnvironment { id: 9, .. })
    ));
    for bad in [
        CentralImpulse {
            body: BodySlot::B,
            ..impulse(9, 0.3)
        },
        CentralImpulse {
            impulse: [0., 0., 1.],
            ..impulse(70, 0.)
        },
        impulse(70, f64::NAN),
        impulse(70, f64::MAX),
    ] {
        // The existing plane action is reserved, so test body B on a fresh batch below.
        assert!(queue_environment_impulses(&mut sim, at, &[bad]).is_err());
    }
    assert_eq!(checkpoint_mixed(&sim).unwrap().to_bytes().unwrap(), before);
    let mut reset = seed[2];
    let State::SpherePlane(p) = &mut reset.state else {
        unreachable!()
    };
    p.velocity[0] = 2.1;
    reset_environments(&mut sim, at, &[reset]).unwrap();
    assert_eq!(
        batch(&sim).pending_impulses().copied().collect::<Vec<_>>(),
        pending[..1]
    );
    assert!(matches!(
        queue_environment_impulses(
            &mut sim,
            at,
            &[CentralImpulse {
                body: BodySlot::B,
                ..impulse(9, 0.3)
            }]
        ),
        Err(GpuError::InvalidAction(9))
    ));
    sim.tick().unwrap();
    assert_eq!(batch(&sim).tick(), 4);
    assert_eq!(batch(&sim).get(9), Some(&reset.state));
    sim.step_exact(1).unwrap();
    baseline.step_exact(1).unwrap();
    assert_eq!(batch(&sim).get(70), batch(&baseline).get(70));
    assert_eq!(batch(&sim).get(0), batch(&baseline).get(0));
    assert_ne!(batch(&sim).get(9), batch(&baseline).get(9));
    assert_eq!(
        sim.world()
            .resource::<BatchTiming>()
            .last
            .unwrap()
            .uploaded_bytes,
        (size_of::<SpherePlaneLane>() + size_of::<SpherePairLane>()) as u64
    );
    assert_eq!(batch(&sim).pending_impulses().len(), 0);
    assert!(matches!(
        reset_environments(&mut sim, at, &[]),
        Err(GpuError::StaleBoundary)
    ));
    sim.reset().unwrap();
    sim.step_exact(4).unwrap();
    assert!(matches!(
        reset_environments(&mut sim, at, &[]),
        Err(GpuError::StaleBoundary)
    ));
    let changed = batch_boundary(&sim);
    queue_environment_impulses(
        &mut sim,
        changed,
        &[CentralImpulse {
            body: BodySlot::B,
            ..impulse(0, 0.55)
        }],
    )
    .unwrap();
    sim.reset().unwrap();
    assert_eq!(batch(&sim).pending_impulses().len(), 0);
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn checkpoint_after_selected_reset_recovers_terminal_dispatch_with_exact_queued_action() {
    let seed = initial();
    let mut sim = mixed_batch_simulation(&selector(), seed.clone()).unwrap();
    sim.reset().unwrap();
    sim.step_exact(7).unwrap();
    let at = batch_boundary(&sim);
    reset_environments(&mut sim, at, &seed[0..1]).unwrap();
    queue_environment_impulses(&mut sim, at, &[impulse(70, 0.3), impulse(9, -0.15)]).unwrap();
    let bytes = checkpoint_mixed(&sim).unwrap().to_bytes().unwrap();
    let cp = MixedCheckpoint::from_bytes(&bytes).unwrap();
    assert_eq!((cp.epoch(), cp.tick()), (1, 7));
    let mut restored = restore_mixed_simulation(&selector(), &cp).unwrap();
    assert_eq!(batch(&restored).environments(), batch(&sim).environments());
    assert_eq!(restored.is_paused(), sim.is_paused());
    assert_eq!(
        restored.world().resource::<UpdateInfo>(),
        sim.world().resource::<UpdateInfo>()
    );
    let mut damaged = bytes.clone();
    damaged[20] ^= 0x80;
    for b in [&damaged[..], &bytes[..17], &bytes[..bytes.len() - 1]] {
        assert!(MixedCheckpoint::from_bytes(b).is_err());
    }
    // Damage an unrelated clean, resident record. Reset/actions are still only
    // staged for the other two rows and must survive the failed world exactly.
    let m = batch(&sim);
    m.gpu
        .queue
        .write_buffer(&m.gpu.state, m.groups()[1].offset, &f64::NAN.to_le_bytes());
    assert!(sim.step_exact(1).is_err());
    assert_eq!(checkpoint_mixed(&sim).unwrap().to_bytes().unwrap(), bytes);
    assert!(reset_environments(&mut sim, at, &[]).is_err());
    let mut recovered =
        restore_mixed_simulation(&selector(), &checkpoint_mixed(&sim).unwrap()).unwrap();
    for _ in 0..16 {
        recovered.step_exact(1).unwrap();
        restored.step_exact(1).unwrap();
        assert_eq!(
            batch(&recovered).environments(),
            batch(&restored).environments()
        );
    }
    let mut fresh = mixed_batch_simulation(&selector(), seed).unwrap();
    fresh.reset().unwrap();
    fresh.step_exact(7).unwrap();
    reset_environments(&mut fresh, at, &initial()[0..1]).unwrap();
    queue_environment_impulses(&mut fresh, at, &[impulse(70, 0.3), impulse(9, -0.15)]).unwrap();
    fresh.step_exact(16).unwrap();
    assert_eq!(
        batch(&fresh).environments(),
        batch(&recovered).environments()
    );
    recovered.reset().unwrap();
    assert_eq!(batch_boundary(&recovered).epoch, 2);
    assert_eq!(batch(&recovered).tick(), 0);
}
use EnvironmentState as State;
