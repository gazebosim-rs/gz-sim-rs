// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use rapier3d_f64::prelude::*;

#[test]
fn native_cut_checks_current_shape_and_pose_before_retiring_a_cached_prediction() {
    for edit in [
        "larger shape",
        "moved parent",
        "moved child",
        "increased skin",
    ] {
        let mut w = PhysicsWorld::default();
        w.gravity = Vector::ZERO;
        let robot = w.insert_body(RigidBodyBuilder::dynamic());
        let neighbor = w.insert_body(RigidBodyBuilder::dynamic().translation(Vector::X * 0.1005));
        let a = w.insert_collider(ColliderBuilder::ball(0.05), Some(robot));
        let b = w.insert_collider(ColliderBuilder::ball(0.05), Some(neighbor));
        w.step();
        let eligible = |w: &PhysicsWorld| {
            w.islands.can_sleep_with_separated_predictions(
                &w.bodies,
                &[robot],
                &w.colliders,
                &w.narrow_phase,
                &[(a, b)],
            )
        };
        assert!(eligible(&w), "initial 0.5 mm gap for {edit}");

        match edit {
            "larger shape" => w.colliders[b].set_shape(SharedShape::ball(0.052)),
            "moved parent" => w.bodies[neighbor].set_translation(Vector::X * 0.099, true),
            "moved child" => w.colliders[b].set_translation_wrt_parent(Vector::X * -0.002),
            "increased skin" => w.colliders[b].set_contact_skin(0.002),
            _ => unreachable!(),
        }

        // No intervening solve: the old graph edge is still a separated prediction.
        let pair = w.narrow_phase.contact_pair(a, b).unwrap();
        assert!(pair.has_any_active_contact());
        assert!(pair.solver_manifolds()[0].data.solver_contacts[0].dist > 0.);
        assert!(!eligible(&w), "stale native admission after {edit}");
        assert!(!w.islands.sleep_with_separated_predictions(
            &mut w.bodies,
            &[robot],
            &w.colliders,
            &mut w.narrow_phase,
            &[(a, b)],
        ));
        assert!(!w.bodies[robot].is_sleeping());
        assert!(
            w.narrow_phase
                .contact_pair(a, b)
                .unwrap()
                .has_any_active_contact()
        );
    }
}
