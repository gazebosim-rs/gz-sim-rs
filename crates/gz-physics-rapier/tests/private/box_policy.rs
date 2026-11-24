// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_rapier::BoxContactActions;

#[test]
fn rejected_feature_policies_leave_queued_forces_and_the_next_native_solve_intact() {
    let build = || fixture(0.12, 0., [0.1; 3]);
    let (mut actual, foot, pad) = build();
    let (mut reference, other_foot, _) = build();
    let pair = [(foot, pad)];
    let dup = [pair[0]; 2];
    let same = [(foot, foot)];
    let missing = [(BodyId::from_raw(u64::MAX), pad)];
    let triple = [(foot, pad, BodyId::from_raw(u64::MAX))];
    for (w, f) in [(&mut actual, foot), (&mut reference, other_foot)] {
        w.add_world_wrench(f, [200., 50., -7.], [1., -2., 0.])
            .unwrap();
    }
    let pre = actual.state(foot);
    let options = [
        BoxContactActions {
            fresh_features: &pair,
            returning_features: &pair,
            ..Default::default()
        },
        BoxContactActions {
            returning_features: &dup,
            ..Default::default()
        },
        BoxContactActions {
            fresh_features: &missing,
            ..Default::default()
        },
        BoxContactActions {
            returning_features: &[(pad, foot)],
            ..Default::default()
        },
        BoxContactActions {
            sustained_edges: &same,
            ..Default::default()
        },
        BoxContactActions {
            face_bearings: &missing,
            ..Default::default()
        },
        BoxContactActions {
            face_bearings: &dup,
            ..Default::default()
        },
        BoxContactActions {
            sustained_edges: &pair,
            face_bearings: &pair,
            ..Default::default()
        },
        BoxContactActions {
            deck_edge_bearings: &pair,
            ..Default::default()
        },
        BoxContactActions {
            rocking_limits: &pair,
            ..Default::default()
        },
        BoxContactActions {
            two_bearing_edges: &pair,
            ..Default::default()
        },
        BoxContactActions {
            target_point_bounds: &pair,
            ..Default::default()
        },
        BoxContactActions {
            compact_point_tails: &pair,
            ..Default::default()
        },
        BoxContactActions {
            subdivide_point_bounds: &pair,
            ..Default::default()
        },
        BoxContactActions {
            sliding_vertices: &pair,
            ..Default::default()
        },
        BoxContactActions {
            finite_point_impacts: &pair,
            point_transfers: &dup,
            ..Default::default()
        },
        BoxContactActions {
            integrate_ballistic_translation: &pair,
            ..Default::default()
        },
        BoxContactActions {
            project_opposing_corners: &triple,
            ..Default::default()
        },
        BoxContactActions {
            project_opposing_returns: &triple,
            ..Default::default()
        },
        BoxContactActions {
            project_opposing_normals: &triple,
            ..Default::default()
        },
        BoxContactActions {
            opposing_box_guards: &triple,
            guard_separated_flight: &pair,
            ..Default::default()
        },
    ];
    for policy in options {
        assert!(
            actual
                .advance_box_contacts(DT, 4, 4.into(), policy, None)
                .is_err(),
            "{policy:?}"
        );
        assert_eq!(actual.state(foot), pre);
    }
    let a = actual.step_contact_interval(DT, 4, 4).unwrap();
    let b = reference.step_contact_interval(DT, 4, 4).unwrap();
    assert_eq!(a.native_steps, b.native_steps);
    assert_eq!(actual.state(foot), reference.state(other_foot));
    assert!(actual.state(foot).unwrap().velocity[0] > 0.);
}

#[cfg(feature = "landing-audit")]
#[test]
fn travel_policy_preserves_loaded_contact_and_paused_ticks_leave_no_observations() {
    use gz_physics_rapier::ReturnFeatureMode;
    let (mut w, f, p) = fixture(0.05019, 0., [0.1; 3]);
    let iterations = w.ordinary_contact_iterations();
    let pair = (f, p);
    assert_eq!(w.step_box_features(Duration::ZERO, &[pair]).unwrap(), 0);
    assert_eq!(
        w.step_contact_interval_with_box_features(Duration::ZERO, 4, iterations, &[pair])
            .unwrap()
            .1,
        0
    );
    w.set_contact_pgs_passes(3).unwrap();
    assert_eq!(w.contact_pgs_passes(), 3);
    assert!(w.set_contact_pgs_passes(0).is_err());
    assert!(w.set_contact_pgs_passes(33).is_err());
    assert_eq!(w.contact_pgs_passes(), 3);
    let mut times = vec![];
    let (paused, counters) = w
        .step_returning_box_features(
            pair,
            Duration::ZERO,
            4,
            iterations,
            ReturnFeatureMode::FirstTravel,
            Some(&mut |_| panic!("paused observer")),
        )
        .unwrap();
    assert_eq!(paused.native_steps, 0);
    assert_eq!(counters, Default::default());
    let (landing, _) = w
        .step_returning_box_features(
            pair,
            Duration::from_millis(2),
            4,
            iterations,
            ReturnFeatureMode::FirstTravel,
            Some(&mut |s| times.push(s.elapsed)),
        )
        .unwrap();
    assert_eq!(times.len(), 4);
    assert!(times.windows(2).all(|ts| ts[0] < ts[1]));
    assert!(landing.pairs.iter().any(|p| p.normal_impulse > 0.8));
    assert!(w.has_box_contact(f, p));
    assert!(!w.has_box_contact(f, BodyId::from_raw(u64::MAX)));
    assert!(w.step_box_features(DT, &[pair]).is_ok());
    assert_eq!(w.step_box_features(DT, &[]).unwrap(), 0);
}

#[cfg(feature = "landing-audit")]
#[test]
fn welded_foot_corner_mobility_matches_parallel_axis_inertia_after_native_step() {
    let mut w = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let links = [0., 0.2].map(|z| {
        w.spawn_articulation_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.03 },
                position: [0., 0., z],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [0.02, 0.02, 0.02, 0., 0., 0.],
            }),
        )
        .unwrap()
    });
    w.add_fixed_joint(Some(links[0]), links[1], [0., 0., 0.1])
        .unwrap();
    let arms = [
        [-0.18, -0.1, -0.06],
        [-0.18, 0.1, -0.06],
        [0.18, -0.1, -0.06],
        [0.18, 0.1, -0.06],
    ];
    assert!(w.landing_normal_mobility(links[0], arms).is_err());
    w.step(Duration::from_millis(1)).unwrap();
    assert!(w.landing_normal_mobility(links[1], arms).is_err());
    assert!(
        w.landing_normal_mobility(links[0], [[f64::NAN; 3]; 4])
            .is_err()
    );
    let matrix = w.landing_normal_mobility(links[0], arms).unwrap();
    for (i, a) in arms.iter().enumerate() {
        for (j, b) in arms.iter().enumerate() {
            // The two mass centers are 0.1 m on either side of the welded COM.
            let inertia = 2. * (0.02 + 0.1_f64.powi(2));
            let response = 0.5 + (a[0] * b[0] + a[1] * b[1]) / inertia;
            assert!((matrix[i][j] - response).abs() < 1e-11);
        }
    }
}
