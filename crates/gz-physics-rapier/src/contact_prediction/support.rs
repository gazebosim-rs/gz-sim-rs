// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Eligibility for a single native solve with a persistent planar support.
use super::*;
use rapier3d_f64::parry::bounding_volume::Aabb;

impl RapierBackend {
    /// Rapier 0.36.1 builds rigid solver contacts before velocity iterations.
    /// In a single collision/solve epoch with soft CCD off, new constraints
    /// start within prediction_distance + both skins (narrow_phase/pair_update).
    /// The primary shape cast already covers this band and free motion. A load
    /// from the same face can change the payload velocity, but cannot create
    /// another solver constraint halfway through those iterations. The next
    /// Bevy PhysicsPrepare reads the solved pose and twist. Recycled arm contacts,
    /// multiple collision epochs, and anything that could redirect the payload
    /// from another face or body retain the full native speed-cap fallback.
    pub(super) fn single_planar_support(
        &self,
        tree: MultibodyIndex,
        payload: ColliderHandle,
        support: ColliderHandle,
        pose: &Pose,
        reachable: &Aabb,
        dt: f64,
    ) -> bool {
        let world = &self.world;
        let payload_shape = &world.colliders[payload];
        let body = &world.bodies[payload_shape.parent().expect("free payload")];
        let native_discrete = |b: &RigidBody| {
            !b.is_ccd_enabled() && b.soft_ccd_prediction() == 0. && b.soft_body().is_none()
        };
        if world.integration_parameters.max_ccd_substeps > 1
            || self.curves.active()
            || body.colliders().len() != 1
            || !native_discrete(body)
            || body.user_force() != Vector::ZERO
            || body.user_torque() != Vector::ZERO
        {
            return false;
        }
        let Some(pair) = world.narrow_phase.contact_pair(payload, support) else {
            return false;
        };
        let impulse = pair.total_impulse_magnitude();
        if !impulse.is_finite() || impulse <= 0. {
            return false;
        }
        let mut active = pair
            .solver_manifolds()
            .iter()
            .filter(|m| !m.data.solver_contacts.is_empty());
        let Some(manifold) = active.next() else {
            return false;
        };
        if active.next().is_some() {
            return false;
        }
        let n = if pair.collider1 == support {
            manifold.data.normal
        } else {
            -manifold.data.normal
        };
        if world.gravity.dot(n) >= 0.
            || body.linvel().dot(n).abs() * dt > world.integration_parameters.prediction_distance()
        {
            return false;
        }
        let shape = world.colliders[support].shape();
        let face_offset = if let Some(plane) = shape.as_halfspace() {
            if (n - pose.rotation * plane.normal).length_squared() > 1e-12 {
                return false;
            }
            0.
        } else if let Some(b) = shape.as_cuboid() {
            let local_normal = (pose.rotation.inverse() * n).abs();
            let axis = local_normal.max_position();
            if 1. - local_normal[axis] > 1e-12 {
                return false;
            }
            let local_reach = reachable.transform_by(&pose.inverse());
            for i in 0..3 {
                if i != axis
                    && (local_reach.mins[i]
                        <= -b.half_extents[i] + world.colliders[support].contact_skin()
                        || local_reach.maxs[i]
                            >= b.half_extents[i] - world.colliders[support].contact_skin())
                {
                    return false;
                }
            }
            b.half_extents[axis]
        } else {
            return false;
        };
        let Some(payload_map) = payload_shape.shape().as_support_map() else {
            return false;
        };
        let payload_pose = body.position()
            * payload_shape
                .position_wrt_parent()
                .expect("attached payload");
        let lowest = payload_map.support_point(&payload_pose, -n);
        let gap = n.dot(lowest - pose.translation)
            - face_offset
            - world.colliders[support].contact_skin()
            - payload_shape.contact_skin();
        if gap.abs() > world.integration_parameters.prediction_distance() {
            // The cached contact predates this tick's geometry (e.g. a pose edit
            // or deep penetration). A previous load alone cannot certify the face.
            return false;
        }
        world
            .multibody_joints
            .get_multibody(tree)
            .expect("registered tree")
            .links()
            .all(|link| {
                let source = &world.bodies[link.rigid_body_handle()];
                native_discrete(source)
                    && source.colliders().iter().all(|h| {
                        !collision::can_exchange_impulses(
                            &world.colliders[*h],
                            payload_shape,
                            RigidBodyType::Dynamic,
                        ) || !world
                            .narrow_phase
                            .contact_pair(*h, payload)
                            .is_some_and(|p| p.has_any_active_contact())
                    })
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const DT: Duration = Duration::from_millis(1);
    fn ball(position: [f64; 3]) -> BodySpec {
        BodySpec {
            shape: gz_physics::Shape::Sphere { radius: 0.1 },
            position,
            velocity: [0.; 3],
            mass: Some(1.),
        }
    }

    fn fixture(half_extent: Option<f64>) -> (RapierBackend, BodyId, BodyId, BodyId) {
        let mut backend = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
        let [root, tip] = [0., 1.].map(|y| {
            backend
                .spawn_articulation_link(
                    BodySpec {
                        shape: gz_physics::Shape::Box {
                            half_extents: [0.1; 3],
                        },
                        ..ball([0., y, 0.1])
                    },
                    RotationSpec::default(),
                    None,
                )
                .unwrap()
        });
        backend
            .add_reduced_revolute(Some(root), tip, [0., 0.5, 0.1], [0., 0., 1.])
            .unwrap();
        let floor = match half_extent {
            Some(x) => backend
                .spawn(BodySpec {
                    shape: gz_physics::Shape::Box {
                        half_extents: [x, 5., 0.1],
                    },
                    position: [0., 0., -0.1],
                    mass: None,
                    velocity: [0.; 3],
                })
                .unwrap(),
            None => backend
                .spawn_horizontal_halfspace(gz_physics::PlaneSpec {
                    position: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    size: [10.; 2],
                })
                .unwrap(),
        };
        for _ in 0..2000 {
            backend.step(DT).unwrap();
        }
        backend
            .switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        let payload = backend.spawn(ball([-0.47, 1., 0.1])).unwrap();
        (backend, root, payload, floor)
    }
    fn query(b: &RapierBackend, root: BodyId, payload: BodyId) -> bool {
        b.predicts_articulation_contact(root, payload, DT).unwrap()
    }
    fn settle(b: &mut RapierBackend) {
        for _ in 0..25 {
            b.step(DT).unwrap();
        }
    }

    #[test]
    fn loaded_halfspace_and_cuboid_interior_require_current_supported_geometry() {
        for extent in [None, Some(5.)] {
            let (mut backend, root, payload, _) = fixture(extent);
            assert!(
                query(&backend, root, payload),
                "new contact before native load"
            );
            settle(&mut backend);
            assert!(!query(&backend, root, payload), "loaded interior");
            let handle = backend.bodies[&payload].0;
            let mut position = backend.world.bodies[handle].translation();
            position.z -= 0.04;
            backend.world.bodies[handle].set_translation(position, true);
            assert!(
                query(&backend, root, payload),
                "old load after deeper pose edit"
            );
        }
    }

    #[test]
    fn extent_boundary_and_second_reachable_obstacle_remain_conservative() {
        let (mut b, root, payload, _) = fixture(Some(0.65));
        settle(&mut b);
        assert!(
            query(&b, root, payload),
            "finite support edge lies within one tick's cap"
        );

        let (mut b, root, payload, _) = fixture(Some(5.));
        settle(&mut b);
        assert!(!query(&b, root, payload));
        // The post-deflection envelope reaches this fixed obstacle, but the stationary
        // payload's free chord cannot. The floor and wall may act in the same step.
        b.spawn(BodySpec {
            mass: None,
            ..ball([-0.9, 1., 0.1])
        })
        .unwrap();
        assert!(query(&b, root, payload));
    }

    #[test]
    fn a_second_surface_skin_changes_reach_for_attached_and_world_colliders() {
        for attached in [true, false] {
            let (mut b, root, payload, _) = fixture(None);
            settle(&mut b);
            let center = b.world.bodies[b.bodies[&payload].0].translation();
            let reach = b.world.integration_parameters.max_linear_velocity() * DT.as_secs_f64()
                + b.world.integration_parameters.prediction_distance()
                + 0.2;
            let position = center - Vector::X * (reach + 0.005);
            let handle = if attached {
                let fixed = b
                    .spawn(BodySpec {
                        mass: None,
                        ..ball(position.to_array())
                    })
                    .unwrap();
                b.world.bodies[b.bodies[&fixed].0].colliders()[0]
            } else {
                b.world
                    .colliders
                    .insert(ColliderBuilder::ball(0.1).translation(position))
            };
            assert!(!query(&b, root, payload));
            b.world.colliders[handle].set_contact_skin(0.01);
            assert!(query(&b, root, payload));
            b.world.colliders[handle].set_sensor(true);
            assert!(
                !query(&b, root, payload),
                "sensor cannot deflect the supported payload"
            );
        }
    }

    #[test]
    fn a_loaded_curved_or_moving_support_retains_deflection_reach() {
        let (mut b, root, payload, floor) = fixture(None);
        let support = b.bodies[&floor].0;
        let slab = b.world.bodies[support].colliders()[0];
        let h = b.bodies[&payload].0;
        let ball = b.world.bodies[h].colliders()[0];
        settle(&mut b);
        assert!(!query(&b, root, payload));
        b.world.bodies[support].set_body_type(RigidBodyType::KinematicVelocityBased, true);
        assert!(query(&b, root, payload));

        b.world.bodies[support].set_body_type(RigidBodyType::Fixed, true);
        b.world.colliders[slab].set_shape(SharedShape::ball(5.));
        b.world.bodies[support].set_translation(Vector::new(-0.47, 1., -5.), true);
        settle(&mut b);
        assert!(
            b.world
                .narrow_phase
                .contact_pair(ball, slab)
                .unwrap()
                .total_impulse_magnitude()
                > 0.
        );
        assert!(query(&b, root, payload));
    }

    #[test]
    fn extra_solve_epochs_soft_ccd_and_new_external_load_keep_full_reach() {
        let (mut b, root, payload, _) = fixture(None);
        settle(&mut b);
        let h = b.bodies[&payload].0;
        assert!(!query(&b, root, payload));
        b.world.integration_parameters.max_ccd_substeps = 2;
        assert!(query(&b, root, payload));
        b.world.integration_parameters.max_ccd_substeps = 1;
        b.world.bodies[h].set_soft_ccd_prediction(0.2);
        assert!(query(&b, root, payload));
        b.world.bodies[h].set_soft_ccd_prediction(0.);
        b.world.bodies[h].add_force(Vector::new(10., 0., 0.), true);
        assert!(query(&b, root, payload));
    }
}
