// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn pair() -> (
    RapierBackend,
    BodyId,
    BodyId,
    ColliderHandle,
    ColliderHandle,
) {
    let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let sphere = |x, y| BodySpec {
        position: [x, y, 0.],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: gz_physics::Shape::Sphere { radius: 0.1 },
    };
    let [root, tip] = [0., 1.].map(|y| {
        backend
            .spawn_articulation_link(sphere(0., y), RotationSpec::default(), None)
            .unwrap()
    });
    backend
        .add_limited_revolute(Some(root), tip, [0., 0.5, 0.], [0., 0., 1.], [-1., 1.])
        .unwrap();
    let payload = backend.spawn(sphere(0.204, 1.)).unwrap();
    backend
        .switch_articulation(root, ArticulationTier::Prescribed, Duration::from_millis(1))
        .unwrap();
    let [a, b] =
        [tip, payload].map(|id| backend.world.bodies[backend.bodies[&id].0].colliders()[0]);
    (backend, root, payload, a, b)
}

#[test]
fn promotion_uses_both_skins_across_a_gap() {
    let (mut backend, root, payload, a, b) = pair();
    let predicts = |w: &RapierBackend| {
        w.predicts_articulation_contact(root, payload, Duration::from_millis(1))
            .unwrap()
    };
    // Default prediction distance is resolved at runtime, independently of scene units.
    let margin = backend.world.integration_parameters.prediction_distance();
    let h = backend.bodies[&payload].0;
    backend.world.bodies[h].set_translation(Vector::new(0.2 + margin + 0.003, 1., 0.), true);
    assert!(!predicts(&backend));
    backend.world.colliders[a].set_contact_skin(0.002);
    assert!(!predicts(&backend));
    backend.world.colliders[b].set_contact_skin(0.002);
    assert!(predicts(&backend));
}

#[test]
fn sensor_disabled_and_masked_shapes_cannot_request_a_rigid_solve() {
    let (mut backend, root, payload, a, b) = pair();
    backend.world.colliders[a].set_contact_skin(0.01);
    let predicts = |w: &RapierBackend| {
        w.predicts_articulation_contact(root, payload, Duration::from_millis(1))
            .unwrap()
    };
    assert!(predicts(&backend));
    let active = backend.world.colliders[a].active_collision_types();
    backend.world.colliders[a].set_active_collision_types(ActiveCollisionTypes::empty());
    assert!(predicts(&backend)); // Rapier accepts the union of the two masks.
    backend.world.colliders[b].set_active_collision_types(ActiveCollisionTypes::empty());
    assert!(!predicts(&backend));
    backend.world.colliders[a].set_active_collision_types(active);
    backend.world.colliders[b].set_active_collision_types(active);
    for handle in [a, b] {
        backend.world.colliders[handle].set_sensor(true);
        assert!(!predicts(&backend));
        backend.world.colliders[handle].set_sensor(false);
        backend.world.colliders[handle].set_enabled(false);
        assert!(!predicts(&backend));
        backend.world.colliders[handle].set_enabled(true);
        let original = backend.world.colliders[handle].collision_groups();
        backend.world.colliders[handle].set_collision_groups(InteractionGroups::none());
        assert!(!predicts(&backend));
        backend.world.colliders[handle].set_collision_groups(original);
        let original = backend.world.colliders[handle].solver_groups();
        backend.world.colliders[handle].set_solver_groups(InteractionGroups::none());
        assert!(!predicts(&backend));
        backend.world.colliders[handle].set_solver_groups(original);
        assert!(predicts(&backend));
    }
}

#[test]
fn broad_phase_matches_unpruned_native_casts_for_skew_box_paths() {
    let (mut backend, root, payload, _, selected) = pair();
    let h = backend.bodies[&payload].0;
    let shape = SharedShape::cuboid(0.23, 0.08, 0.12);
    backend.world.colliders[selected].set_shape(shape.clone());
    backend.world.colliders[selected].set_contact_skin(0.003);
    let dt = Duration::from_millis(10);
    let index = backend.articulation_index(root).unwrap();
    let mut hits = 0;
    let mut total = 0;
    for rotation in [0., 0.37, 1.2] {
        backend.world.bodies[h].set_rotation(Rotation::from_rotation_z(rotation), true);
        for x in [-0.72, -0.38, -0.32, 0.04, 0.45] {
            for y in [0.02, 0.63, 0.79, 0.89, 1.04, 1.15, 1.22, 1.4] {
                for v in [
                    Vector::ZERO,
                    Vector::new(38., 6., 0.),
                    Vector::new(-17., -7., 4.),
                ] {
                    backend.world.bodies[h].set_translation(Vector::new(x, y, 0.03), true);
                    backend.world.bodies[h].set_linvel(v, true);
                    let pose = *backend.world.bodies[h].position();
                    let pred = backend.world.integration_parameters.prediction_distance() + 0.003;
                    let options = ShapeCastOptions {
                        max_time_of_impact: dt.as_secs_f64(),
                        target_distance: pred,
                        ..Default::default()
                    };
                    let tree = backend.world.multibody_joints.get_multibody(index).unwrap();
                    let expected = tree.links().any(|l| {
                        let body = &backend.world.bodies[l.rigid_body_handle()];
                        let c = &backend.world.colliders[body.colliders()[0]];
                        let source = *body.position() * c.position_wrt_parent().unwrap();
                        if v == Vector::ZERO {
                            distance(&source, c.shape(), &pose, shape.as_ref())
                                .unwrap()
                                .distance
                                <= pred
                        } else {
                            cast_shapes(
                                &source,
                                Vector::ZERO,
                                c.shape(),
                                &pose,
                                v,
                                shape.as_ref(),
                                options,
                            )
                            .unwrap()
                            .is_some()
                        }
                    });
                    assert_eq!(
                        backend
                            .predicts_articulation_contact(root, payload, dt)
                            .unwrap(),
                        expected,
                        "x={x} y={y} v={v:?} angle={rotation}"
                    );
                    hits += usize::from(expected);
                    total += 1;
                }
            }
        }
    }
    assert!(hits > 0 && hits < total);
    eprintln!("unpruned direct casts: {total} comparisons, {hits} hits");
}
