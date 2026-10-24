// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn counter_overflows_do_not_mutate_the_clock() {
    let mut sim = Simulation::new(Duration::from_nanos(1)).unwrap();
    sim.iteration = u64::MAX;
    assert_eq!(sim.step_exact(1), Err(StepError::Overflow));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    sim.set_paused(false);
    assert_eq!(sim.tick(), Err(StepError::Overflow));
    sim.epoch = u64::MAX;
    assert_eq!(sim.reset(), Err(StepError::Overflow));
    assert_eq!(sim.epoch, u64::MAX);
}
