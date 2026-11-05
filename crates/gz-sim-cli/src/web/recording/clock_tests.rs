// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn stamp(epoch: u64, sequence: u64, seconds: u64) -> Stamp {
    Stamp {
        epoch,
        sequence,
        seconds,
        nanoseconds: 0,
    }
}

#[test]
fn timestamps_credit_same_time_order_and_reset_generation() {
    let now = Instant::now();
    let mut clock = Clock::new(
        vec![
            stamp(0, 2, 0),
            stamp(0, 5, 1),
            stamp(0, 6, 1),
            stamp(1, 8, 0),
        ],
        now,
    )
    .unwrap();
    assert_eq!(clock.frames(), 4);
    assert!(!clock.playing());
    let old = clock.credit();
    clock.play(now).unwrap();
    assert!(!clock.advance(now + Duration::from_secs(10)).unwrap());
    clock.accept(old);
    assert!(!clock.advance(now + Duration::from_millis(999)).unwrap());
    assert!(clock.advance(now + Duration::from_secs(1)).unwrap());
    assert_eq!(clock.frame(), 1);
    clock.accept(old); // Duplicate cannot accept the next ordinal.
    assert!(!clock.advance(now + Duration::from_secs(1)).unwrap());
    clock.accept(clock.credit());
    assert!(clock.advance(now + Duration::from_secs(1)).unwrap());
    assert_eq!(clock.frame(), 2); // Equal time never skips an outcome.
    clock.accept(clock.credit());
    assert!(clock.advance(now + Duration::from_secs(1)).unwrap());
    assert_eq!(clock.epoch(), 1);
    assert!(clock.finished());
    assert!(!clock.playing());
    assert!(clock.play(now).is_err());
    let stale = clock.credit();
    clock.restart(now).unwrap();
    assert_eq!((clock.frame(), clock.epoch()), (0, 2));
    clock.play(now).unwrap();
    clock.accept(stale);
    clock.accept(old); // Even the old initial ordinal has a stale generation.
    assert!(!clock.advance(now + Duration::from_secs(100)).unwrap());
    clock.accept(clock.credit());
    assert!(clock.advance(now + Duration::from_secs(100)).unwrap());
    assert_eq!(clock.frame(), 1);
}

#[test]
fn pause_retains_only_active_elapsed_time_and_long_gap_heartbeats() {
    let now = Instant::now();
    let mut clock = Clock::new(vec![stamp(0, 0, 0), stamp(0, 1, 30)], now).unwrap();
    clock.accept(clock.credit());
    clock.play(now).unwrap();
    clock.pause(now + Duration::from_secs(7));
    clock.pause(now + Duration::from_secs(80));
    assert!(!clock.advance(now + Duration::from_secs(100)).unwrap());
    clock.play(now + Duration::from_secs(100)).unwrap();
    clock.play(now + Duration::from_secs(110)).unwrap(); // No clock reset.
    for seconds in [105, 110, 115, 120, 122] {
        clock.accept(clock.credit()); // Periodic coherent same-frame tuples.
        assert!(!clock.advance(now + Duration::from_secs(seconds)).unwrap());
    }
    assert!(clock.advance(now + Duration::from_secs(123)).unwrap());
    assert!(clock.finished());
}

#[test]
fn malformed_clock_and_exhausted_restart_are_atomic() {
    let now = Instant::now();
    for stamps in [
        vec![],
        vec![stamp(1, 0, 0)],
        vec![stamp(0, 0, 1)],
        vec![stamp(0, 1, 0), stamp(0, 1, 1)],
        vec![stamp(0, 0, 0), stamp(1, 1, 1)],
        vec![stamp(0, 0, 0), stamp(0, 2, 2), stamp(0, 3, 1)],
        vec![stamp(0, 0, 0), stamp(1, 1, 0), stamp(0, 2, 0)],
        vec![Stamp {
            nanoseconds: 1_000_000_000,
            ..stamp(0, 0, 0)
        }],
        vec![stamp(0, 0, 0); 4097],
    ] {
        assert!(Clock::new(stamps, now).is_err());
    }
    let mut clock = Clock::new(vec![stamp(0, 0, 0), stamp(1, 1, 0)], now).unwrap();
    clock.generation = u64::MAX;
    assert!(clock.restart(now).is_err());
    assert_eq!(clock.frame(), 0);
    assert_eq!(clock.epoch(), 0);
    clock.generation = 1;
    clock.epoch = u64::MAX;
    assert!(clock.restart(now).is_err());
    assert_eq!(clock.generation, 1);
    clock.play(now).unwrap();
    clock.accept(clock.credit());
    assert!(clock.advance(now).is_err());
    assert_eq!(clock.frame(), 0);
    assert_eq!(clock.epoch(), u64::MAX);
    let one = Clock::new(vec![stamp(0, 0, 0)], now).unwrap();
    assert!(one.finished());
    assert!(!one.playing());
}
