// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn unloading_and_overflow_do_not_publish_spurious_loads() {
    let mut loads = PadLoads::default();
    let force = DVec3::new(0.1, 0.2, 0.3);
    loads.react(2, 1, force, -force).unwrap();
    loads.publish();
    let first = loads.committed.clone();
    for (normal, tangent) in [
        (DVec3::splat(f64::MAX), DVec3::ZERO),
        (DVec3::ZERO, DVec3::splat(f64::MAX)),
    ] {
        loads.begin();
        loads.react(2, 1, normal, tangent).unwrap();
        assert_eq!(
            loads.react(2, 1, normal, tangent),
            Err(XpbdError::Numerical)
        );
        assert_eq!(loads.committed, first);
    }
    loads.begin();
    loads.react(2, 1, force, -force).unwrap();
    loads.react(2, 1, -force, force).unwrap();
    loads.publish();
    assert!(loads.committed.is_empty());
}

#[test]
fn an_out_of_range_plane_reading_expires_with_its_successful_interval() {
    let mut probe = super::PlaneImpulseProbe::default();
    let loads = probe.loads.get_or_insert_with(Default::default);
    loads.observe_plane(1, 2, DVec3::X * f64::MAX, DVec3::ZERO);
    loads.observe_plane(1, 2, DVec3::X * f64::MAX, DVec3::ZERO);
    loads.publish();
    assert_eq!(probe.readings(), Err(XpbdError::Numerical));
    let loads = probe.loads.as_mut().unwrap();
    loads.begin();
    loads.observe_plane(1, 2, DVec3::Z, DVec3::ZERO);
    loads.publish();
    assert_eq!(probe.readings().unwrap()[0].normal, [0., 0., -1.]);
}
