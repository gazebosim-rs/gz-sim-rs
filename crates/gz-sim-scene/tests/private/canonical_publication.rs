// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::{reserve_id, restore_id};

#[test]
fn dense_model_publication_follows_the_live_link_across_reordering_and_replacement() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let offset = WorldPose {
        translation: [0.25, -0.1, 0.35],
        rotation: DQuat::from_rotation_y(-0.75).to_array(),
    };
    let mut owners = Vec::new();
    // An additional table grows in descending public ID order after replacement.
    #[derive(Component)]
    struct RestoredLink;
    for i in 0..9000 {
        let pose = WorldPose {
            translation: [f64::from(i), 2., -3.],
            rotation: DQuat::from_rotation_z(0.4).to_array(),
        };
        let link = sim.world_mut().spawn(pose).id();
        let id = assign_id(sim.world_mut(), link).unwrap();
        let model = sim
            .world_mut()
            .spawn((
                ModelAnchor {
                    link: id,
                    link_to_model: offset,
                },
                pose,
            ))
            .id();
        owners.push((model, link));
    }
    sim.add_systems(Phase::PhysicsCommit, sync_models);
    sim.add_systems(Phase::Reset, sync_models);
    sim.step_exact(1).unwrap();

    for (model, link) in owners.iter_mut().rev().step_by(3) {
        let reservation = reserve_id(sim.world(), *link).unwrap();
        let mut pose = *sim.world().get::<WorldPose>(*link).unwrap();
        pose.translation[1] += 5.;
        pose.rotation = DQuat::from_rotation_x(-0.8).to_array();
        sim.world_mut().despawn(*link);
        *link = sim.world_mut().spawn((pose, RestoredLink)).id();
        restore_id(sim.world_mut(), *link, &reservation).unwrap();
        assert_eq!(
            sim.world().get::<ModelAnchor>(*model).unwrap().link,
            reservation.get()
        );
    }
    sim.step_exact(1).unwrap();
    sim.reset().unwrap();
    for (model, link) in owners {
        let source = sim.world().get::<WorldPose>(link).unwrap();
        let published = sim.world().get::<WorldPose>(model).unwrap();
        let rotation = DQuat::from_array(source.rotation);
        let translation = DVec3::from_array(source.translation)
            + rotation * DVec3::from_array(offset.translation);
        assert_eq!(published.translation, translation.to_array());
        assert_eq!(
            published.rotation,
            (rotation * DQuat::from_array(offset.rotation)).to_array()
        );
    }
}

#[test]
fn dense_body_publication_preserves_each_native_pose_and_center_velocity() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend: PhysicsBackend = RapierBackend::new([0.; 3]).unwrap().into();
    let mut cases = Vec::new();
    let rotation = RotationSpec {
        orientation: DQuat::from_rotation_z(0.5).to_array(),
        angular_velocity: [0., 0., 2.],
    };
    // Two distinct Bevy tables have full batches and unequal final batches.
    for i in 0..9001 {
        let spec = BodySpec {
            shape: Shape::Sphere { radius: 0.1 },
            position: [f64::from(i), 0., 3.],
            velocity: [0.25, f64::from(i), -0.5],
            mass: Some(1.),
        };
        let id = backend.spawn_link(spec, rotation, None).unwrap();
        let mut body = sim.world_mut().spawn((
            BodyHandle(id),
            WorldPose {
                translation: [0.; 3],
                rotation: [0., 0., 0., 1.],
            },
            Velocity {
                linear: [0.; 3],
                angular: [0.; 3],
            },
        ));
        let center = [0.001 * f64::from(i), 0.1, 0.];
        if i % 2 == 0 {
            body.insert(LinkCenterOfMass(center));
        }
        cases.push((body.id(), id, (i % 2 == 0).then_some(center)));
    }
    sim.world_mut().insert_resource(backend);
    sim.add_systems(Phase::PhysicsCommit, sync_bodies);
    sim.step_exact(1).unwrap();
    let backend = sim.world().resource::<PhysicsBackend>();
    for (entity, id, center) in cases {
        let native = backend.state(id).unwrap();
        let published = sim.world().get::<WorldPose>(entity).unwrap();
        let velocity = sim.world().get::<Velocity>(entity).unwrap();
        assert_eq!(published.translation, native.position);
        assert_eq!(published.rotation, native.orientation);
        assert_eq!(velocity.angular, native.angular_velocity);
        let mut expected = DVec3::from_array(native.velocity);
        if let Some(center) = center {
            let arm = DQuat::from_array(native.orientation) * DVec3::from_array(center);
            expected -= DVec3::from_array(native.angular_velocity).cross(arm);
        }
        assert_eq!(velocity.linear, expected.to_array());
    }
}
