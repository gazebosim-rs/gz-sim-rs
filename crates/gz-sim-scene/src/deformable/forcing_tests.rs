// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn a_native_impulse_or_unavailable_input_keeps_the_native_prediction() {
    let mass = SoftRigidInertia {
        inverse_mass: 2.,
        inverse_principal_inertia: [3., 4., 5.],
        principal_orientation: DQuat::IDENTITY.to_array(),
    };
    let start = BodyState {
        position: [0.; 3],
        velocity: [2., -1., 0.5],
        orientation: DQuat::IDENTITY.to_array(),
        angular_velocity: [0.; 3],
    };
    let a = DVec3::new(0., 2., -9.81);
    let dt = Duration::from_millis(10);
    let mut end = BodyState {
        velocity: (DVec3::from(start.velocity) + dt.as_secs_f64() * a).to_array(),
        ..start
    };
    assert!(matches!(
        ReceiverMass::ForceAware(mass).trajectory(start, end, Some(a.to_array()), dt),
        RigidMobility::ForceAware { .. }
    ));
    for (m, acceleration) in [
        (ReceiverMass::Principal(mass), Some(a.to_array())),
        (ReceiverMass::ForceAware(mass), None),
    ] {
        assert!(matches!(
            m.trajectory(start, end, acceleration, dt),
            RigidMobility::Principal(_)
        ));
    }
    end.velocity[0] += 1e-8;
    assert!(matches!(
        ReceiverMass::ForceAware(mass).trajectory(start, end, Some(a.to_array()), dt),
        RigidMobility::Principal(_)
    ));
}
