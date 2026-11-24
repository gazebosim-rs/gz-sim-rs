// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn incomplete_and_overflowed_measurements_do_not_publish_partial_rows_and_recover() {
    let mut sample = DeformableSample::default();
    assert!(sample.last_plane_impulses().is_none());
    let mut integral = PlaneIntegral::default();
    let row = ParticleBoundaryImpulse {
        boundary: 0,
        particle: 2,
        normal: [f64::MAX; 3],
        tangent: [0.; 3],
    };
    let mut compare = |total: &PlaneIntegral, reset| {
        total.publish(sample.plane.get_or_insert_with(Default::default), reset);
        sample.last_plane_impulses().unwrap().map(<[_]>::to_vec)
    };
    for row in [
        row,
        ParticleBoundaryImpulse {
            normal: [0.; 3],
            tangent: [f64::MAX; 3],
            ..row
        },
    ] {
        integral.clear();
        integral.add(Ok(&[row]), &[4]);
        assert_eq!(compare(&integral, false).unwrap()[0].boundary, 4);
        integral.add(Ok(&[row]), &[4]);
        assert_eq!(compare(&integral, false), Err(PlaneImpulseOverflow));
        assert!(compare(&integral, true).unwrap().is_empty());
        integral.clear();
        integral.add(Err(XpbdError::Numerical), &[]);
        integral.add(Ok(&[row]), &[4]);
        assert_eq!(compare(&integral, false), Err(PlaneImpulseOverflow));
        integral.clear();
        integral.add(Ok(&[row]), &[4]);
        integral.add(
            Ok(&[ParticleBoundaryImpulse {
                normal: row.normal.map(|v| -v),
                tangent: row.tangent.map(|v| -v),
                ..row
            }]),
            &[4],
        );
        assert!(compare(&integral, false).unwrap().is_empty());
    }
}

#[cfg(feature = "distributed-physics")]
#[test]
fn worker_cannot_accept_local_only_sensor_and_publish_zero() {
    let xml = include_str!("../../../../examples/worlds/local_floor_impulses.sdf");
    let sim = crate::load_world_with_sdf_step(
        xml,
        "floor.sdf",
        "default",
        &sdformat::NoResources,
        crate::PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let material = sim.world().resource::<crate::deformable::Deformables>();
    assert!(
        material
            .managed_recipe()
            .err()
            .unwrap()
            .to_string()
            .contains("per-particle plane impulses require local material ownership")
    );
}
