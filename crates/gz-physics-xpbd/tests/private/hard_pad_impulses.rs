// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{RigidMobility, coupled::Receiver, pad_loads::PadLoads};

#[test]
fn hard_pad_rejects_overflow_in_either_the_global_or_the_individual_load() {
    let mut pad = PreparedBoundary::new(Boundary::Cuboid {
        center: [0.; 3],
        half_extents: [1.; 3],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    })
    .unwrap();
    let mobility = RigidMobility::PrescribedCuboid {
        normal_compliance: 0.,
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    };
    pad.receiver = Some(Receiver::new(mobility, DVec3::ZERO, 0.001, true, DVec3::ZERO).unwrap());
    let impulse = [f64::MAX * DVec3::X, DVec3::ZERO];
    let mut loads = Some(Box::<PadLoads>::default());
    let mut total = BoundaryImpulse::default();
    pad.react_at_particle(&mut total, &mut loads, (0, 0), DVec3::ZERO, impulse)
        .unwrap();
    assert_eq!(
        pad.react_at_particle(&mut total, &mut loads, (0, 1), DVec3::ZERO, impulse),
        Err(XpbdError::Numerical)
    );
    // Opposing particles can leave a finite net wrench while a single measured
    // particle overflows. Both diagnostics must propagate to the caller.
    pad.react_at_particle(
        &mut total,
        &mut loads,
        (0, 1),
        DVec3::ZERO,
        [-impulse[0], DVec3::ZERO],
    )
    .unwrap();
    assert_eq!(
        pad.react_at_particle(&mut total, &mut loads, (0, 0), DVec3::ZERO, impulse),
        Err(XpbdError::Numerical)
    );
    pad.receiver = None;
    assert!(!pad.prescribed_pad());
}
