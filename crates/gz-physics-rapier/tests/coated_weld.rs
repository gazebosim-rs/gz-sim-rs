// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodyId, BodySpec, DynamicBackend, RapierBackend, Shape, SoleFoot};
fn make(b: &mut RapierBackend, z: f64, mass: Option<f64>) -> BodyId {
    b.spawn(BodySpec {
        shape: Shape::Box {
            half_extents: [0.1; 3],
        },
        position: [0., 0., z],
        velocity: [0.; 3],
        mass,
    })
    .unwrap()
}
#[test]
fn invalid_companion_domain_or_endpoint_leaves_the_weld_supports_and_wrench_intact() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let lower = make(&mut b, 0.4, Some(1.));
    let upper = make(&mut b, 0.8, Some(2.));
    b.add_fixed_joint(Some(lower), upper, [0., 0., 0.55])
        .unwrap();
    let plates = [make(&mut b, -0.3, None), make(&mut b, -0.7, None)];
    let binding = SoleFoot::Weld { lower, upper };
    b.add_world_wrench(upper, [2., -3., 4.], [0.03, -0.04, 0.2])
        .unwrap();
    let held = b.sole_wrench(binding, plates.into_iter()).unwrap();
    assert_eq!(held.0, [2., -3., 4.]);
    for invalid in [
        SoleFoot::Box(lower),
        SoleFoot::Compound(lower),
        SoleFoot::Weld {
            lower,
            upper: lower,
        },
        SoleFoot::Weld {
            lower,
            upper: plates[0],
        },
    ] {
        assert!(b.sole_wrench(invalid, plates.into_iter()).is_err());
    }
    assert!(
        b.sole_wrench(binding, [plates[0], plates[0]].into_iter())
            .is_err()
    );
    assert!(
        b.sole_wrench(binding, [plates[0], upper].into_iter())
            .is_err()
    );
    let initial = [lower, upper, plates[0], plates[1]].map(|id| b.state(id).unwrap());
    let rows = plates.map(|p| (p, b.state(p).unwrap()));
    let mut invalid = rows;
    invalid[1].1.angular_velocity = [1., 0., 0.];
    assert!(
        b.commit_sole_panels((binding, initial[0]), &invalid)
            .is_err()
    );
    for bad in [f64::NAN, f64::INFINITY] {
        let mut malformed = initial[0];
        malformed.velocity = [bad, 0., 0.];
        assert!(b.commit_sole_panels((binding, malformed), &rows).is_err());
    }
    assert_eq!(
        initial,
        [lower, upper, plates[0], plates[1]].map(|id| b.state(id).unwrap())
    );
    assert_eq!(held, b.sole_wrench(binding, plates.into_iter()).unwrap());
    let a = b.welded_assembly_state(lower).unwrap();
    let mut end = initial[0];
    end.position[0] += 0.2;
    end.velocity = [1., 1., 0.];
    end.angular_velocity = [1., 0., 0.];
    b.commit_sole_panels((binding, end), &rows).unwrap();
    let moved = b.welded_assembly_state(upper).unwrap();
    assert!((moved.center[0] - a.center[0] - 0.2).abs() < 1e-13);
    assert_eq!(moved.spin, [1., 0., 0.]);
    assert!(
        moved
            .velocity
            .iter()
            .zip(end.velocity)
            .all(|(a, b)| (a - b).abs() < 1e-12)
    );
    assert_eq!(moved.force, [0.; 3]);
    assert_eq!(moved.torque, [0.; 3]);
}
#[test]
fn detached_bodies_and_unregistered_geometry_cannot_impersonate_a_coated_weld() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let lower = make(&mut b, 0.4, Some(1.));
    let upper = make(&mut b, 0.8, Some(1.));
    let support = make(&mut b, 0., None);
    let read =
        |b: &RapierBackend| b.sole_wrench(SoleFoot::Weld { lower, upper }, [support].into_iter());
    assert!(read(&b).is_err());
    b.add_fixed_joint(Some(lower), upper, [0., 0., 0.6])
        .unwrap();
    assert!(read(&b).is_ok());
    let other = b
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.1 },
            position: [0.3, 0., 0.],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    assert!(read(&b).is_err());
    assert!(
        b.sole_wrench(
            SoleFoot::Weld { lower, upper },
            [support, other].into_iter()
        )
        .is_err()
    );
}
#[test]
fn independent_box_and_fixed_or_jointed_foot_follow_separate_admission_rules() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let foot = make(&mut b, 0.4, Some(1.));
    let plate = make(&mut b, 0., None);
    assert!(b.sole_wrench(foot, [plate].into_iter()).is_ok());
    assert!(b.sole_wrench(plate, [foot].into_iter()).is_err());
    let mut bad = b.state(foot).unwrap();
    bad.orientation = [0.; 4];
    assert!(
        b.commit_sole_panels((foot, bad), &[(plate, b.state(plate).unwrap())])
            .is_err()
    );
    let mut fixed = RapierBackend::new([0.; 3]).unwrap();
    let a = make(&mut fixed, 0.4, None);
    let c = make(&mut fixed, -0.4, None);
    assert!(fixed.sole_wrench(a, [c].into_iter()).is_err());
}
