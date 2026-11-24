// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Frozen sweep regressions for the boundary between accepted inputs and native contact ownership.
#[path = "../examples/support/field_trial.rs"]
mod field_trial;
use field_trial::{Trial, trial};
use std::path::Path;

#[test]
fn a_free_arrival_keeps_its_queued_input_when_a_later_wrench_requires_a_warm_owner() {
    for (seed, impacts) in [
        (20261514821, [Some(48), Some(123)]),
        (20261521923, [Some(57), Some(171)]),
    ] {
        let mut result = Trial {
            seed,
            bitwise: true,
            exact_contact_sets: true,
            ..Default::default()
        };
        trial(&mut result, Path::new(env!("CARGO_BIN_EXE_physics_worker"))).unwrap();
        assert_eq!(result.ticks, 216);
        assert!(result.bitwise && result.exact_contact_sets && result.replacement);
        assert_eq!(result.impacts, impacts);
        assert_eq!(result.transfers, 1);
    }
}
