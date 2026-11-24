// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn incremental_support_ignores_departed_nodes_and_obeys_its_own_friction_budget() {
    let plane = |normal: DVec3| {
        PreparedBoundary::new(Boundary::Plane {
            point: [0.; 3],
            normal: normal.to_array(),
            velocity: [0.5, 0., 0.1],
        })
        .unwrap()
    };
    let boundaries = [plane(DVec3::Z), plane(DVec3::X)];
    let mut contacts = Contacts::default();
    let projection = Projection {
        event: None,
        normal: DVec3::Z,
        depth: 0.,
        initial_overlap: 0.,
    };
    assert!(!contacts.on_single_plane(&boundaries));
    for id in 0..3 {
        contacts.record(id, 0, &projection, 0., None);
    }
    assert!(contacts.on_single_plane(&boundaries));
    let x = [
        DVec3::new(1., 2., 0.11),
        DVec3::new(1., 2., 0.12),
        DVec3::new(1., 2., 0.11),
    ];
    let mut p = x.map(|pos| Particle {
        position: pos.to_array(),
        velocity: [2., 0., -0.9],
        inverse_mass: 0.5,
    });
    p[2].velocity[2] = 0.6;
    let before = p;
    let mut reactions = [BoundaryImpulse::default(); 2];
    contacts
        .finish_plane_increment(
            &x,
            &mut p,
            &boundaries,
            &mut reactions,
            0.25,
            Interval {
                t0: 0.9,
                t1: 1.,
                h: 0.1,
                radius: 0.01,
            },
        )
        .unwrap();
    assert_eq!(p[1..], before[1..]);
    let dv = DVec3::from(p[0].velocity) - DVec3::from(before[0].velocity);
    assert!((dv - DVec3::new(-0.25, 0., 1.)).length() < 1e-14);
    let impulse = dv / p[0].inverse_mass;
    assert!((DVec3::from(reactions[0].linear) + impulse).length() < 1e-14);
    assert!((DVec3::from(reactions[0].angular) + x[0].cross(impulse)).length() < 1e-14);
    contacts.record(0, 1, &projection, 0., None);
    assert!(!contacts.on_single_plane(&boundaries));
}

#[test]
fn spring_pads_allow_at_most_one_distinct_rigid_plane_at_the_endpoint() {
    let plane = PreparedBoundary::new(Boundary::Plane {
        point: [0.; 3],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    })
    .unwrap();
    let hard = PreparedBoundary::new(Boundary::Cuboid {
        center: [0.; 3],
        half_extents: [0.2; 3],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    })
    .unwrap();
    let mut pad = hard.clone();
    pad.normal_compliance = 1e-4;
    let boundaries = [pad, plane.clone(), plane, hard];
    let projection = Projection {
        event: None,
        normal: DVec3::Z,
        depth: 0.,
        initial_overlap: 0.,
    };
    for (ids, eligible) in [
        (&[][..], false),
        (&[0][..], true),
        (&[1][..], false),
        (&[0, 1, 1][..], true),
        (&[1, 0, 1][..], true),
        (&[0, 1, 2][..], false),
        (&[0, 3][..], false),
        (&[3, 0][..], false),
    ] {
        let mut contacts = Contacts::default();
        for (particle, &boundary) in ids.iter().enumerate() {
            contacts.record(particle, boundary, &projection, 0., None);
        }
        assert_eq!(
            contacts.spring_pads_with_one_plane(&boundaries),
            eligible,
            "{ids:?}"
        );
    }
}

#[test]
fn a_static_plane_row_rejects_an_unrepresentable_force_before_moving_the_node() {
    let boundary = PreparedBoundary::new(Boundary::Plane {
        point: [0.; 3],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    })
    .unwrap();
    let mut p = Particle {
        position: [0., 0., 0.1],
        velocity: [0.; 3],
        inverse_mass: f64::from_bits(1),
    };
    let mut x = DVec3::new(0.001, 0., 0.1);
    let mut contacts = Contacts::default();
    contacts.record(
        0,
        0,
        &Projection {
            event: None,
            normal: DVec3::Z,
            depth: 0.01,
            initial_overlap: 0.,
        },
        0.,
        None,
    );
    let mut reaction = BoundaryImpulse::default();
    let time = Interval {
        h: 0.001,
        t0: 0.,
        t1: 0.001,
        radius: 0.1,
    };
    assert_eq!(
        contacts.items[0].stick_on_plane(
            &mut x,
            (&mut p, DVec3::ZERO),
            &boundary,
            time,
            0.5,
            &mut reaction
        ),
        Err(XpbdError::Numerical)
    );
    assert_eq!(x, DVec3::new(0.001, 0., 0.1));
    assert_eq!(p.velocity, [0.; 3]);
    assert_eq!(reaction.linear, [0.; 3]);
}
