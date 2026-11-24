// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::gpu_scene::ExecutionPlacement::{Cpu, Gpu};
fn config() -> AutoPlacementConfig {
    AutoPlacementConfig {
        window_ticks: 8,
        min_residency_ticks: 64,
        reprobe_ticks: 512,
        hysteresis_percent: 10,
    }
}
fn run(mut p: Policy, cost: impl Fn(SamplingPhase, u64) -> u64) -> Vec<Decision> {
    let mut events = vec![];
    for tick in 1..240 {
        let dt = Duration::from_nanos(cost(p.phase, tick));
        if let Some(d) = p.sample(tick, dt) {
            p.management_cost(Duration::from_nanos(7));
            events.push(d);
        }
    }
    events
}
#[test]
fn switch_needs_bracketed_advantage_hysteresis_and_measured_payback() {
    let make = || Policy::new(config(), Cpu, 0);
    let r = run(make(), |p, _| {
        if p == SamplingPhase::Alternative {
            500
        } else {
            1000
        }
    });
    assert_eq!((r[0].reason, r[0].target), (PlacementReason::Probe, Gpu));
    assert_eq!(
        (r[1].reason, r[1].target),
        (PlacementReason::ConfirmBaseline, Cpu)
    );
    assert_eq!((r[2].reason, r[2].target), (PlacementReason::Commit, Gpu));
    assert_eq!(r[2].evidence.conservative_saving_ns, 500);
    assert_eq!(r[2].evidence.break_even_ticks, Some(7));
    for alternative in [950, 1000, 1300] {
        let r = run(make(), |p, _| {
            if p == SamplingPhase::Alternative {
                alternative
            } else {
                1000
            }
        });
        assert_eq!(r[2].reason, PlacementReason::InsufficientGain);
        assert_eq!(r[2].target, Cpu);
    }
    let r = run(make(), |p, t| {
        if p == SamplingPhase::Alternative {
            if t == 72 { 55_000 } else { 500 }
        } else {
            1000
        }
    });
    assert_eq!(r[2].reason, PlacementReason::InsufficientResidence);
    let r = run(make(), |p, _| match p {
        SamplingPhase::Alternative => 300,
        SamplingPhase::Confirmation => 550,
        _ => 1000,
    });
    assert_eq!(r[2].reason, PlacementReason::UnstableWindow);
}
#[test]
fn unstable_current_window_and_short_lived_loads_do_not_move() {
    let r = run(Policy::new(config(), Gpu, 0), |_, t| {
        if t % 2 == 0 { 950 } else { 370 }
    });
    assert_eq!(r.len(), 1);
    assert_eq!(r[0].reason, PlacementReason::UnstableWindow);
    let mut p = Policy::new(config(), Cpu, 0);
    for t in 1..150 {
        if t % 7 == 0 {
            p.rearm(t, Cpu);
        }
        assert!(p.sample(t, Duration::from_millis(1)).is_none());
    }
    p.phase = SamplingPhase::Manual;
    assert!(p.sample(u64::MAX, Duration::from_millis(1)).is_none());
}
