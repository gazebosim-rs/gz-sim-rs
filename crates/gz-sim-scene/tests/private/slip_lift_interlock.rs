// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::*;
use SlipCatchState::{AtLimit, Caught};

#[test]
fn only_contiguous_simulated_time_at_limit_latches_the_upward_cap() {
    let mut lift = SlipLiftInterlock::new(Duration::from_millis(3));
    for (state, saturated, ms, expected) in [
        (AtLimit, true, 2, 10.0),
        (AtLimit, false, 25, 10.0),
        (AtLimit, true, 2, 10.0),
        (Caught, true, 1, 10.0),
        (AtLimit, true, 2, 10.0),
        (Caught, false, 0, 10.0),
        (AtLimit, true, 1, 7.0),
        (Caught, false, 40, 7.0),
    ] {
        assert_eq!(
            lift.command(state, saturated, Duration::from_millis(ms), 10., 7.),
            expected
        );
    }
    assert_eq!(lift.command(Caught, false, Duration::ZERO, -1., 7.), -1.);
}

#[test]
fn immediate_interlock_still_requires_a_committed_limit_observation() {
    let mut lift = SlipLiftInterlock::new(Duration::ZERO);
    assert_eq!(lift.command(AtLimit, true, Duration::ZERO, 3., 2.), 3.);
    assert_eq!(
        lift.command(Caught, false, Duration::from_nanos(1), 3., 2.),
        3.
    );
    assert_eq!(
        lift.command(AtLimit, true, Duration::from_nanos(1), 3., 2.),
        2.
    );
}
