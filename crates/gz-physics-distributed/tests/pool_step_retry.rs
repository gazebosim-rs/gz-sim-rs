// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{Body, BodyWrench, SimulationStep, fixture},
    pool::{PoolNativeRecovery, WorkerPool},
};
use std::{error::Error, path::Path};
type Test = Result<(), Box<dyn Error>>;
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn pool(approaching: bool) -> WorkerPool {
    let make = |i: usize| {
        let mut b: Body = fixture()[1].clone();
        b.id = 100 + i as u64;
        b.size = [0.5; 3];
        b.position = [i as f64 * if approaching { 0.54 } else { 5.0 }, 0., 0.];
        b.linear = [
            if approaching {
                if i == 0 { 3.0 } else { -3.0 }
            } else {
                0.
            },
            0.,
            0.,
        ];
        b.angular = [0.; 3];
        b
    };
    WorkerPool::spawn(
        worker(),
        4219,
        vec![vec![make(0)], vec![make(1)], vec![]],
        SimulationStep::Ms10,
        None,
        48,
    )
    .unwrap()
}
fn pids(pool: &WorkerPool) -> Vec<u32> {
    (0..3).map(|i| pool.worker_pid(i).unwrap()).collect()
}
fn force(x: f64) -> BodyWrench {
    BodyWrench {
        id: 100,
        force: [x, 0., 3.],
        torque: [0., 0., 0.2],
    }
}
#[test]
fn repeated_final_replies_commit_routing_and_inputs_exactly_once() -> Test {
    for packed in [false, true] {
        let mut actual = pool(true);
        let mut reference = pool(true);
        actual.set_packed_observations(packed);
        let prior = actual.frame().clone();
        let before = pids(&actual);
        // The third, idle endpoint receives only the final solve. The other two
        // have staged a prospective contact handoff before this reply is lost.
        actual.drop_next_acknowledgement(2)?;
        let loads = [force(1.25)];
        assert!(actual.advance_islands_in_field([0.; 3], &loads).is_err());
        assert_eq!(actual.frame(), &prior);
        assert_eq!(actual.last_local_helper_mask(), None);
        assert_eq!(actual.last_island_refined_bodies(), None);
        assert_eq!(
            actual.advance_islands_in_field([0.; 3], &loads)?,
            reference.advance_islands_in_field([0.; 3], &loads)?
        );
        assert_eq!(pids(&actual), before);
        let (owner, _) = actual.frame().body(100).unwrap();
        assert_eq!(actual.frame().body(101).unwrap().0, owner);
        assert_eq!(
            actual.frame().workers[usize::from(owner)]
                .generations
                .values()
                .copied()
                .sum::<u64>(),
            1
        );
        assert_eq!(
            actual.last_island_refined_bodies(),
            reference.last_island_refined_bodies()
        );
        for _ in 0..3 {
            assert_eq!(actual.advance()?, reference.advance()?);
        }
        actual.compact()?;
        actual.recover(owner)?;
        assert_eq!(actual.advance()?, reference.advance()?);
    }
    Ok(())
}

#[test]
fn changed_input_bits_or_fidelity_fence_the_speculative_tick() -> Test {
    for mode_change in [false, true] {
        let mut actual = pool(false);
        let mut reference = pool(false);
        let before = pids(&actual);
        actual.drop_next_acknowledgement(0)?;
        let loads = [force(0.)];
        assert!(actual.advance_islands_in_field([0.; 3], &loads).is_err());
        if mode_change {
            assert_eq!(
                actual.advance_in_field([0.; 3], &loads)?,
                reference.advance_in_field([0.; 3], &loads)?
            );
        } else {
            let different = [force(-0.)];
            assert_eq!(
                actual.advance_islands_in_field([0.; 3], &different)?,
                reference.advance_islands_in_field([0.; 3], &different)?
            );
        }
        assert!(pids(&actual).iter().zip(before).all(|(a, b)| *a != b));
    }
    Ok(())
}

#[test]
fn lost_cache_and_repeated_loss_fall_back_to_the_committed_barrier() -> Test {
    for kill in [true, false] {
        let mut actual = pool(false);
        let mut reference = pool(false);
        let before = pids(&actual);
        actual.queue_impulse(100, [1., 0., 0.])?;
        reference.queue_impulse(100, [1., 0., 0.])?;
        actual.drop_next_acknowledgement(0)?;
        assert!(actual.advance().is_err());
        if kill {
            actual.terminate_worker(0)?;
        } else {
            actual.drop_next_acknowledgement(0)?;
        }
        assert_eq!(actual.advance()?, reference.advance()?);
        let now = pids(&actual);
        assert_ne!(now[0], before[0]);
        assert_eq!(&now[1..], &before[1..]);
        assert_eq!(actual.advance()?, reference.advance()?);
    }
    Ok(())
}

#[test]
fn disk_recovery_excludes_an_unpublished_solve_and_its_staged_handoff() -> Test {
    let mut actual = pool(true);
    let prior = actual.frame().clone();
    actual.drop_next_acknowledgement(2)?;
    assert!(actual.advance().is_err());
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("confirmed.gzpool");
    actual.save_native_recovery(&path)?;
    let image = PoolNativeRecovery::open(path)?;
    assert_eq!(image.frame(), &prior);
    let mut restored = image.resume(worker())?;
    assert_eq!(restored.advance()?, actual.advance()?);
    Ok(())
}

#[test]
fn only_missing_replies_are_retransmitted_and_a_confirmed_peer_death_still_fences() -> Test {
    for killed in [false, true] {
        let mut actual = pool(false);
        let mut reference = pool(false);
        actual.drop_next_acknowledgement(0)?;
        actual.drop_next_acknowledgement(2)?;
        assert!(actual.advance().is_err());
        if killed {
            actual.terminate_worker(1)?;
        }
        let before = actual.traffic();
        let ids = pids(&actual);
        assert_eq!(actual.advance()?, reference.advance()?);
        if killed {
            let now = pids(&actual);
            assert_ne!(now[1], ids[1]);
            assert_eq!(now[0], ids[0]);
            assert_eq!(now[2], ids[2]);
        } else {
            let delta: Vec<_> = actual
                .traffic()
                .iter()
                .zip(before)
                .map(|(a, b)| a.requests - b.requests)
                .collect();
            assert_eq!(delta, [1, 0, 1]);
            assert_eq!(pids(&actual), ids);
        }
    }
    Ok(())
}

#[test]
fn reconstructs_only_the_dead_peer_including_its_staged_contact_transfer() -> Test {
    for dead in 0..3 {
        let mut actual = pool(true);
        let mut control = pool(true);
        let before = pids(&actual);
        actual.drop_next_acknowledgement(2)?;
        let field = [force(2.5)];
        assert!(actual.advance_islands_in_field([0.; 3], &field).is_err());
        assert_eq!(actual.frame().tick, 0);
        actual.terminate_worker(dead)?;
        assert_eq!(
            actual.advance_islands_in_field([0.; 3], &field)?,
            control.advance_islands_in_field([0.; 3], &field)?
        );
        for (w, (old, new)) in before.into_iter().zip(pids(&actual)).enumerate() {
            assert_eq!(old != new, w == usize::from(dead));
        }
        let dir = tempfile::tempdir()?;
        let actual_path = dir.path().join("actual.gzpool");
        let control_path = dir.path().join("control.gzpool");
        actual.save_native_recovery(&actual_path)?;
        control.save_native_recovery(&control_path)?;
        assert_eq!(std::fs::read(actual_path)?, std::fs::read(control_path)?);
        assert_eq!(actual.advance()?, control.advance()?);
    }
    Ok(())
}

#[test]
fn cached_confirmation_grace_accepts_only_positive_bounded_durations() -> Test {
    use std::time::Duration;
    let mut subject = pool(false);
    let mut reference = pool(false);
    for bad in [Duration::ZERO, Duration::from_millis(1001)] {
        assert!(subject.set_step_confirmation_window(bad).is_err());
    }
    for grace in [Duration::from_millis(150), Duration::from_secs(1)] {
        subject.set_step_confirmation_window(grace)?;
        let before = pids(&subject);
        subject.drop_next_acknowledgement(0)?;
        assert!(subject.advance().is_err());
        assert_eq!(subject.advance()?, reference.advance()?);
        assert_eq!(pids(&subject), before);
    }
    Ok(())
}
