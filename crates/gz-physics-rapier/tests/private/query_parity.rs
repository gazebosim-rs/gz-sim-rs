// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::contact_dispatcher::{NativeContacts, Prepare};
use rapier3d_f64::{
    geometry::ContactManifold,
    math::{Pose, Rotation, Vector},
};

pub(super) fn assert_native_queries<T: Prepare>(saved: &NativeContacts<T>) {
    use rapier3d_f64::parry::{
        query::{self as q, PersistentQueryDispatcher, QueryDispatcher},
        shape::{Capsule, Cuboid},
    };
    let direct = q::DefaultQueryDispatcher;
    let (a, b) = (
        Capsule::new_z(0.35, 0.2),
        Cuboid::new(Vector::new(0.4, 0.3, 0.7)),
    );
    let v = Vector::new(-1., 0.01, 0.);
    for x in [0.3, 0.69, 1.35, 2.8] {
        let pose = Pose::from_parts(Vector::new(x, -0.08, 0.02), Rotation::from_rotation_y(0.17));
        assert_eq!(
            saved.intersection_test(&pose, &a, &b).unwrap(),
            direct.intersection_test(&pose, &a, &b).unwrap()
        );
        assert_eq!(
            saved.distance(&pose, &a, &b).unwrap(),
            direct.distance(&pose, &a, &b).unwrap()
        );
        let contact =
            |c: Option<q::Contact>| c.map(|c| (c.point1, c.point2, c.normal1, c.normal2, c.dist));
        assert_eq!(
            contact(saved.contact(&pose, &a, &b, 0.4).unwrap()),
            contact(direct.contact(&pose, &a, &b, 0.4).unwrap())
        );
        assert_eq!(
            saved.closest_points(&pose, &a, &b, 0.4).unwrap(),
            direct.closest_points(&pose, &a, &b, 0.4).unwrap()
        );
        let hit = |h: Option<q::ShapeCastHit>| {
            h.map(|h| {
                (
                    h.time_of_impact,
                    h.witness1,
                    h.witness2,
                    h.normal1,
                    h.normal2,
                    h.status,
                )
            })
        };
        let opts = q::ShapeCastOptions::with_max_time_of_impact(1.);
        assert_eq!(
            hit(saved.cast_shapes(&pose, v, &a, &b, opts).unwrap()),
            hit(direct.cast_shapes(&pose, v, &a, &b, opts).unwrap())
        );
        let stationary = q::NonlinearRigidMotion::constant_position(Pose::IDENTITY);
        let movement =
            q::NonlinearRigidMotion::new(pose, Vector::ZERO, v, Vector::new(0., 0.12, -0.03));
        assert_eq!(
            hit(saved
                .cast_shapes_nonlinear(&stationary, &a, &movement, &b, 0., 1., true)
                .unwrap()),
            hit(direct
                .cast_shapes_nonlinear(&stationary, &a, &movement, &b, 0., 1., true)
                .unwrap())
        );
        let mut m = ContactManifold::new();
        let mut n = ContactManifold::new();
        saved
            .contact_manifold_convex_convex(&pose, &a, &b, None, None, 0.4, &mut m)
            .unwrap();
        direct
            .contact_manifold_convex_convex(&pose, &a, &b, None, None, 0.4, &mut n)
            .unwrap();
        assert_eq!(
            (m.local_n1, m.local_n2, m.points.len()),
            (n.local_n1, n.local_n2, n.points.len())
        );
        for (p, q) in m.points.iter().zip(n.points.iter()) {
            assert_eq!(
                (p.local_p1, p.local_p2, p.fid1, p.fid2),
                (q.local_p1, q.local_p2, q.fid1, q.fid2)
            );
        }
    }
}
