// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Source controls for resolved link inertia; qualification waits for the parent.
use gz_physics::{
    BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia, PhysicsBackend,
    RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, LocalPose, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, time::Duration};

fn quaternion([roll, pitch, yaw]: [f64; 3]) -> [f64; 4] {
    (glam::DQuat::from_rotation_z(yaw)
        * glam::DQuat::from_rotation_y(pitch)
        * glam::DQuat::from_rotation_x(roll))
    .to_array()
}
fn center(count: usize) -> [f64; 3] {
    match count {
        1 => [0.03, -0.02, 0.05],
        2 => [0.1, -0.05, 0.2],
        3 => [0., 0., 0.5],
        _ => unreachable!(),
    }
}
fn fixtures(count: usize) -> (String, Vec<CollisionSpec>) {
    let collision = |index: usize, pose: &str, geometry: &str| {
        format!(
            "<collision name='c{index}'><pose>{pose}</pose><geometry>{geometry}</geometry></collision>"
        )
    };
    let identity = [0., 0., 0., 1.];
    if count == 1 {
        return (
            collision(0, "0 0 0 0 0 0", "<sphere><radius>0.2</radius></sphere>"),
            vec![CollisionSpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.; 3],
                orientation: identity,
            }],
        );
    }
    let cylinder_x = if count == 2 { 0.4 } else { 0. };
    let mut xml = collision(0, "-0.4 0 0 0 0 0", "<box><size>0.6 0.4 0.8</size></box>")
        + &collision(
            1,
            &format!("{cylinder_x} 0 0 0 1.5708 0.3"),
            "<cylinder><radius>0.2</radius><length>1</length></cylinder>",
        );
    let mut specs = vec![
        CollisionSpec {
            shape: Shape::Box {
                half_extents: [0.3, 0.2, 0.4],
            },
            position: [-0.4, 0., 0.],
            orientation: identity,
        },
        CollisionSpec {
            shape: Shape::Cylinder {
                radius: 0.2,
                half_height: 0.5,
            },
            position: [cylinder_x, 0., 0.],
            orientation: quaternion([0., "1.5708".parse().unwrap(), 0.3]),
        },
    ];
    if count == 3 {
        xml += &collision(
            2,
            "0.4 0 0 0.2 0.4 -0.1",
            "<capsule><radius>0.2</radius><length>0.8</length></capsule>",
        );
        specs.push(CollisionSpec {
            shape: Shape::Capsule {
                radius: 0.2,
                half_length: 0.4,
            },
            position: [0.4, 0., 0.],
            orientation: quaternion([0.2, 0.4, -0.1]),
        });
    }
    (xml, specs)
}
fn xml(count: usize) -> String {
    let [x, y, z] = center(count);
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='compound'><link name='body'><pose>0 0 2 0.2 0.1 -0.3</pose><inertial><pose>{x} {y} {z} 0.1 0.2 0.3</pose><mass>2</mass><inertia><ixx>1</ixx><iyy>1.5</iyy><izz>2</izz></inertia></inertial>{}</link></model></world></sdf>",
        fixtures(count).0
    )
}
fn ids(sim: &Simulation) -> BTreeMap<String, u64> {
    sim.world()
        .iter_entities()
        .filter_map(|entity| {
            Some((
                entity.get::<Name>()?.0.clone(),
                entity.get::<EntityId>()?.get(),
            ))
        })
        .collect()
}
fn close(a: impl IntoIterator<Item = f64>, b: impl IntoIterator<Item = f64>) {
    let a: Vec<_> = a.into_iter().collect();
    let b: Vec<_> = b.into_iter().collect();
    assert_eq!(a.len(), b.len());
    for (a, b) in a.into_iter().zip(b) {
        assert!((a - b).abs() < 1e-12, "{a} != {b}");
    }
}
fn fields(state: BodyState) -> impl Iterator<Item = f64> {
    state
        .position
        .into_iter()
        .chain(state.orientation)
        .chain(state.velocity)
        .chain(state.angular_velocity)
}

#[test]
fn resolved_offset_inertia_matches_direct_api_and_loaded_reset() {
    for count in 1..=3 {
        let mut sim = load_world_with_sdf_step(
            &xml(count),
            "offset-com.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Dart,
        )
        .unwrap();
        let original = ids(&sim);
        assert_eq!(original.len(), 3 + count);
        let link_id = original["default::compound::body"];
        let specs = fixtures(count).1;
        for epoch in 0..2 {
            assert_eq!(ids(&sim), original);
            let mut control = DartBackend::new([0.; 3]).unwrap();
            let body = control
                .spawn_compound_link(
                    CompoundBodySpec {
                        collisions: &specs,
                        position: [0., 0., 2.],
                        velocity: [0.; 3],
                        mass: Some(2.),
                    },
                    RotationSpec {
                        orientation: quaternion([0.2, 0.1, -0.3]),
                        ..Default::default()
                    },
                    Inertia {
                        center: center(count),
                        orientation: quaternion([0.1, 0.2, 0.3]),
                        tensor: [1., 1.5, 2., 0., 0., 0.],
                    },
                )
                .unwrap();
            for _ in 0..5 {
                queue_link_world_wrench(&mut sim, link_id, [1., 2., 3.], [0., 0.4, 0.2]).unwrap();
                control
                    .add_world_wrench(body, [1., 2., 3.], [0., 0.4, 0.2])
                    .unwrap();
                control.step(Duration::from_millis(1)).unwrap();
                sim.step_exact(1).unwrap();
                let expected = control.state(body).unwrap();
                let link = entity_by_id(sim.world(), link_id).unwrap();
                let handle = sim.world().get::<BodyHandle>(link).unwrap();
                close(
                    fields(
                        sim.world()
                            .resource::<PhysicsBackend>()
                            .state(handle.id())
                            .unwrap(),
                    ),
                    fields(expected),
                );
                let pose = sim.world().get::<WorldPose>(link).unwrap();
                close(pose.translation, expected.position);
                close(pose.rotation, expected.orientation);
                let velocity = sim.world().get::<Velocity>(link).unwrap();
                let arm = glam::DQuat::from_array(expected.orientation)
                    * glam::DVec3::from_array(center(count));
                let origin_velocity = glam::DVec3::from_array(expected.velocity)
                    - glam::DVec3::from_array(expected.angular_velocity).cross(arm);
                close(velocity.linear, origin_velocity.to_array());
                close(velocity.angular, expected.angular_velocity);
                let rotation = glam::DQuat::from_array(pose.rotation);
                for (index, spec) in specs.iter().enumerate() {
                    let entity = entity_by_id(
                        sim.world(),
                        original[&format!("default::compound::body::c{index}")],
                    )
                    .unwrap();
                    let local = sim.world().get::<LocalPose>(entity).unwrap().0;
                    close(local.translation, spec.position);
                    close(local.rotation, spec.orientation);
                    let attached = sim.world().get::<WorldPose>(entity).unwrap();
                    close(
                        attached.translation,
                        (glam::DVec3::from_array(pose.translation)
                            + rotation * glam::DVec3::from_array(spec.position))
                        .to_array(),
                    );
                    close(
                        attached.rotation,
                        (rotation * glam::DQuat::from_array(spec.orientation)).to_array(),
                    );
                }
            }
            let link = entity_by_id(sim.world(), link_id).unwrap();
            let velocity = sim.world().get::<Velocity>(link).unwrap();
            let pose = sim.world().get::<WorldPose>(link).unwrap();
            let arm =
                glam::DQuat::from_array(pose.rotation) * glam::DVec3::from_array(center(count));
            let origin_difference = glam::DVec3::from_array(velocity.angular).cross(arm);
            assert!(
                origin_difference.length() > 1e-9,
                "fixture must distinguish COM and origin velocity"
            );
            if epoch == 0 {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                assert_eq!(sim.elapsed(), Duration::ZERO);
            }
        }
        remove_model(&mut sim, original["default::compound"]).unwrap();
        assert_eq!(
            ids(&sim).keys().map(String::as_str).collect::<Vec<_>>(),
            ["default"]
        );
        sim.step_exact(1).unwrap();
    }
}

#[test]
fn offset_com_stays_outside_other_engines_and_planes() {
    for engine in [
        PhysicsEngine::Tpe,
        PhysicsEngine::Rapier,
        PhysicsEngine::Mujoco,
    ] {
        assert!(matches!(
            load_world_with_sdf_step(&xml(1), "offset-com.sdf", "default", &NoResources, engine),
            Err(gz_sim_scene::LoadError::Unsupported {
                feature: "offset center of mass",
                ..
            })
        ));
    }
    let plane = xml(1)
        .replace(
            "<model name='compound'>",
            "<model name='compound'><static>true</static>",
        )
        .replace(
            "<sphere><radius>0.2</radius></sphere>",
            "<plane><normal>0 0 1</normal><size>1 1</size></plane>",
        );
    assert!(matches!(
        load_world_with_sdf_step(
            &plane,
            "offset-com-plane.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Dart
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "offset center of mass",
            ..
        })
    ));
}

#[test]
fn resolved_inertial_frame_and_model_pose_preserve_link_origin_semantics() {
    let source = xml(2)
        .replace("<model name='compound'>", "<model name='compound'><pose>1 -2 3 0.4 -0.2 0.1</pose><frame name='inertial_frame' attached_to='body'><pose relative_to='body'>0.1 -0.05 0.2 0.1 0.2 0.3</pose></frame>")
        .replace("<pose>0.1 -0.05 0.2 0.1 0.2 0.3</pose>", "<pose relative_to='inertial_frame'>0 0 0 0 0 0</pose>");
    let mut sim = load_world_with_sdf_step(
        &source,
        "offset-frame.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let original = ids(&sim);
    let model_id = original["default::compound"];
    let link_id = original["default::compound::body"];
    let model_rotation = glam::DQuat::from_array(quaternion([0.4, -0.2, 0.1]));
    let link_rotation = glam::DQuat::from_array(quaternion([0.2, 0.1, -0.3]));
    let initial_position =
        glam::DVec3::new(1., -2., 3.) + model_rotation * glam::DVec3::new(0., 0., 2.);
    let specs = fixtures(2).1;
    let mut control = DartBackend::new([0.; 3]).unwrap();
    let body = control
        .spawn_compound_link(
            CompoundBodySpec {
                collisions: &specs,
                position: initial_position.to_array(),
                velocity: [0.; 3],
                mass: Some(2.),
            },
            RotationSpec {
                orientation: (model_rotation * link_rotation).to_array(),
                ..Default::default()
            },
            Inertia {
                center: center(2),
                orientation: quaternion([0.1, 0.2, 0.3]),
                tensor: [1., 1.5, 2., 0., 0., 0.],
            },
        )
        .unwrap();
    for _ in 0..3 {
        queue_link_world_wrench(&mut sim, link_id, [1., 0., 0.], [0., 0.2, 0.]).unwrap();
        control
            .add_world_wrench(body, [1., 0., 0.], [0., 0.2, 0.])
            .unwrap();
        control.step(Duration::from_millis(1)).unwrap();
        sim.step_exact(1).unwrap();
        let expected = control.state(body).unwrap();
        let link = entity_by_id(sim.world(), link_id).unwrap();
        let handle = sim.world().get::<BodyHandle>(link).unwrap();
        close(
            fields(
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(handle.id())
                    .unwrap(),
            ),
            fields(expected),
        );
        let local = sim.world().get::<LocalPose>(link).unwrap().0;
        close(local.translation, [0., 0., 2.]);
        close(local.rotation, link_rotation.to_array());
        let expected_model_rotation =
            glam::DQuat::from_array(expected.orientation) * link_rotation.conjugate();
        let expected_model_position = glam::DVec3::from_array(expected.position)
            - expected_model_rotation * glam::DVec3::new(0., 0., 2.);
        let model = entity_by_id(sim.world(), model_id).unwrap();
        let model_pose = sim.world().get::<WorldPose>(model).unwrap();
        close(model_pose.translation, expected_model_position.to_array());
        close(model_pose.rotation, expected_model_rotation.to_array());
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(ids(&sim), original);
    let link = entity_by_id(sim.world(), link_id).unwrap();
    close(
        sim.world().get::<WorldPose>(link).unwrap().translation,
        initial_position.to_array(),
    );
}

#[test]
fn static_offset_com_compound_keeps_physical_origin_and_reset_ownership() {
    let source = xml(3).replace(
        "<model name='compound'>",
        "<model name='compound'><static>true</static>",
    );
    let mut sim = load_world_with_sdf_step(
        &source,
        "offset-static.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let original = ids(&sim);
    let link_id = original["default::compound::body"];
    let link = entity_by_id(sim.world(), link_id).unwrap();
    let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
    let initial = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    close(initial.position, [0., 0., 2.]);
    close(initial.orientation, quaternion([0.2, 0.1, -0.3]));
    assert_eq!(
        sim.world().resource::<PhysicsBackend>().is_dynamic(handle),
        Some(false)
    );
    for _ in 0..2 {
        assert!(matches!(
            queue_link_world_wrench(&mut sim, link_id, [1., 0., 0.], [0.; 3]),
            Err(gz_sim_scene::InputError::Unsupported("fixed-body input"))
        ));
        sim.step_exact(3).unwrap();
        assert_eq!(
            sim.world().resource::<PhysicsBackend>().state(handle),
            Some(initial)
        );
        let link = entity_by_id(sim.world(), link_id).unwrap();
        let pose = sim.world().get::<WorldPose>(link).unwrap();
        close(pose.translation, initial.position);
        close(pose.rotation, initial.orientation);
        let rotation = glam::DQuat::from_array(pose.rotation);
        for (index, spec) in fixtures(3).1.into_iter().enumerate() {
            let child = entity_by_id(
                sim.world(),
                original[&format!("default::compound::body::c{index}")],
            )
            .unwrap();
            let actual = sim.world().get::<WorldPose>(child).unwrap();
            close(
                actual.translation,
                (glam::DVec3::from_array(initial.position)
                    + rotation * glam::DVec3::from_array(spec.position))
                .to_array(),
            );
            close(
                actual.rotation,
                (rotation * glam::DQuat::from_array(spec.orientation)).to_array(),
            );
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(ids(&sim), original);
    }
    remove_model(&mut sim, original["default::compound"]).unwrap();
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .is_none()
    );
    sim.step_exact(1).unwrap();
}

#[test]
fn origin_velocity_configuration_and_imu_follow_the_link_point_after_reset() {
    let source = xml(1).replace("</link>", "<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate><imu/></sensor></link>");
    let mut sim = load_world_with_sdf_step(
        &source,
        "offset-imu.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let original = ids(&sim);
    let link_id = original["default::compound::body"];
    let sensor_id = original["default::compound::body::imu"];
    let origin_command = [0.2, -0.1, 0.3];
    let angular_command = [0.1, 0.2, -0.3];
    let orientation = quaternion([0.2, 0.1, -0.3]);
    let center = center(1);
    let initial_arm = glam::DQuat::from_array(orientation) * glam::DVec3::from_array(center);
    let com_command = glam::DVec3::from_array(origin_command)
        + glam::DVec3::from_array(angular_command).cross(initial_arm);
    let specs = fixtures(1).1;
    for epoch in 0..2 {
        assert_eq!(ids(&sim), original);
        let mut control = DartBackend::new([0.; 3]).unwrap();
        let body = control
            .spawn_compound_link(
                CompoundBodySpec {
                    collisions: &specs,
                    position: [0., 0., 2.],
                    velocity: [0.; 3],
                    mass: Some(2.),
                },
                RotationSpec {
                    orientation,
                    ..Default::default()
                },
                Inertia {
                    center,
                    orientation: quaternion([0.1, 0.2, 0.3]),
                    tensor: [1., 1.5, 2., 0., 0., 0.],
                },
            )
            .unwrap();
        control
            .set_world_velocity(body, com_command.to_array(), angular_command)
            .unwrap();
        queue_link_world_velocity(&mut sim, link_id, origin_command, angular_command).unwrap();
        // Configuration replaces the previous origin velocity without a spurious impulse.
        let mut previous = glam::DVec3::from_array(origin_command);
        for tick in 1..=5 {
            control.step(Duration::from_millis(1)).unwrap();
            sim.step_exact(1).unwrap();
            let expected = control.state(body).unwrap();
            let rotation = glam::DQuat::from_array(expected.orientation);
            let origin_velocity = glam::DVec3::from_array(expected.velocity)
                - glam::DVec3::from_array(expected.angular_velocity)
                    .cross(rotation * glam::DVec3::from_array(center));
            let acceleration = (origin_velocity - previous) / 0.001;
            previous = origin_velocity;
            let link = entity_by_id(sim.world(), link_id).unwrap();
            let handle = sim.world().get::<BodyHandle>(link).unwrap();
            close(
                fields(
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .state(handle.id())
                        .unwrap(),
                ),
                fields(expected),
            );
            close(
                sim.world().get::<Velocity>(link).unwrap().linear,
                origin_velocity.to_array(),
            );
            let sensor = entity_by_id(sim.world(), sensor_id).unwrap();
            let sample = sim.world().get::<gz_sim_scene::ImuSample>(sensor).unwrap();
            assert_eq!(sample.time, Duration::from_millis(tick));
            assert_eq!(sample.epoch, epoch);
            let reading = sample.reading.as_ref().unwrap();
            close(
                reading.specific_force,
                (rotation.inverse() * acceleration).to_array(),
            );
            close(
                reading.angular_velocity,
                (rotation.inverse() * glam::DVec3::from_array(expected.angular_velocity))
                    .to_array(),
            );
            assert!(
                acceleration.length() > 1e-6,
                "rotating offset link must exercise origin acceleration"
            );
        }
        if epoch == 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
    }
}
