// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodyState, CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia, PhysicsBackend,
    RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LoadError, LocalPose, Name, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, time::Duration};

const FIRST: &str = "<collision name='first'><pose>0.5 0 0 0 0 0</pose><density>7</density><geometry><sphere><radius>0.1</radius></sphere></geometry></collision>";
const SECOND: &str = "<collision name='second'><pose>-0.5 0 0 0 0 1.5707963267948966</pose><density>11</density><geometry><box><size>0.2 0.4 0.6</size></box></geometry></collision>";
const THIRD: &str = "<collision name='third'><pose>0 0.5 0 0 0 0</pose><geometry><cylinder><radius>0.1</radius><length>0.4</length></cylinder></geometry></collision>";

fn xml(collisions: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='body'><link name='link'><pose>0 0 10 0 0 0</pose><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>{collisions}</link></model></world></sdf>"
    )
}
fn load(source: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(source, "compound.sdf", "default", &NoResources, engine)
}
fn ids(sim: &Simulation) -> BTreeMap<String, u64> {
    sim.world()
        .iter_entities()
        .filter_map(|e| Some((e.get::<Name>()?.0.clone(), e.get::<EntityId>()?.get())))
        .collect()
}
fn close(actual: impl IntoIterator<Item = f64>, expected: impl IntoIterator<Item = f64>) {
    let actual: Vec<_> = actual.into_iter().collect();
    let expected: Vec<_> = expected.into_iter().collect();
    assert_eq!(actual.len(), expected.len());
    for (a, b) in actual.into_iter().zip(expected) {
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
fn compound_link_preserves_body_identity_collision_metadata_reset_and_removal() {
    let mut sim = load(
        &xml(&format!("{FIRST}{SECOND}{THIRD}")),
        PhysicsEngine::Dart,
    )
    .unwrap();
    let original_ids = ids(&sim);
    assert_eq!(original_ids.len(), 6);
    let link_id = original_ids["default::body::link"];
    let half = std::f64::consts::FRAC_PI_4;
    let specs = [
        CollisionSpec {
            shape: Shape::Sphere { radius: 0.1 },
            position: [0.5, 0., 0.],
            orientation: [0., 0., 0., 1.],
        },
        CollisionSpec {
            shape: Shape::Box {
                half_extents: [0.1, 0.2, 0.3],
            },
            position: [-0.5, 0., 0.],
            orientation: [0., 0., half.sin(), half.cos()],
        },
        CollisionSpec {
            shape: Shape::Cylinder {
                radius: 0.1,
                half_height: 0.2,
            },
            position: [0., 0.5, 0.],
            orientation: [0., 0., 0., 1.],
        },
    ];
    for epoch in 0..2 {
        assert_eq!(ids(&sim), original_ids);
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<BodyHandle>())
                .count(),
            1
        );
        for (index, (name, density)) in
            [("first", Some(7.)), ("second", Some(11.)), ("third", None)]
                .into_iter()
                .enumerate()
        {
            let entity = entity_by_id(
                sim.world(),
                original_ids[&format!("default::body::link::{name}")],
            )
            .unwrap();
            assert_eq!(
                sim.world().get::<EntityKind>(entity),
                Some(&EntityKind::Collision)
            );
            assert_eq!(
                sim.world().get::<CollisionDensity>(entity).map(|d| d.0),
                density
            );
            let local = sim.world().get::<LocalPose>(entity).unwrap();
            close(local.0.translation, specs[index].position);
            close(local.0.rotation, specs[index].orientation);
        }
        let mut reference = DartBackend::new([0.; 3]).unwrap();
        let body = reference
            .spawn_compound_link(
                CompoundBodySpec {
                    collisions: &specs,
                    position: [0., 0., 10.],
                    velocity: [0.; 3],
                    mass: Some(2.),
                },
                RotationSpec::default(),
                Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [1., 1., 1., 0., 0., 0.],
                },
            )
            .unwrap();
        for _ in 0..5 {
            queue_link_world_wrench(&mut sim, link_id, [1., 2., 3.], [0., 0., 0.4]).unwrap();
            reference
                .add_world_wrench(body, [1., 2., 3.], [0., 0., 0.4])
                .unwrap();
            reference.step(Duration::from_millis(1)).unwrap();
            sim.step_exact(1).unwrap();
            let link = entity_by_id(sim.world(), link_id).unwrap();
            let handle = sim.world().get::<BodyHandle>(link).unwrap();
            close(
                fields(
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .state(handle.id())
                        .unwrap(),
                ),
                fields(reference.state(body).unwrap()),
            );
            let pose = sim.world().get::<WorldPose>(link).unwrap();
            for (index, name) in ["first", "second", "third"].into_iter().enumerate() {
                let child = entity_by_id(
                    sim.world(),
                    original_ids[&format!("default::body::link::{name}")],
                )
                .unwrap();
                let attached = sim.world().get::<WorldPose>(child).unwrap();
                let rotation = glam::DQuat::from_array(pose.rotation);
                let expected = glam::DVec3::from_array(pose.translation)
                    + rotation * glam::DVec3::from_array(specs[index].position);
                close(attached.translation, expected.to_array());
                close(
                    attached.rotation,
                    (rotation * glam::DQuat::from_array(specs[index].orientation)).to_array(),
                );
            }
        }
        if epoch == 0 {
            let first =
                entity_by_id(sim.world(), original_ids["default::body::link::first"]).unwrap();
            sim.world_mut()
                .entity_mut(first)
                .remove::<CollisionDensity>();
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
    }
    remove_model(&mut sim, original_ids["default::body"]).unwrap();
    assert_eq!(
        ids(&sim).keys().map(String::as_str).collect::<Vec<_>>(),
        ["default"]
    );
    sim.step_exact(1).unwrap();
}

#[test]
fn other_engines_keep_single_centered_collision_domain() {
    for engine in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Mujoco,
        PhysicsEngine::Tpe,
    ] {
        for source in [xml(FIRST), xml(&format!("{FIRST}{SECOND}"))] {
            assert!(matches!(
                load(&source, engine),
                Err(LoadError::Unsupported { .. })
            ));
        }
    }
}

#[test]
fn every_compound_collision_is_validated_and_offset_com_is_preserved() {
    let bad = [
        (
            format!(
                "{FIRST}{SECOND}{}",
                THIRD.replace(
                    "<cylinder><radius>0.1</radius><length>0.4</length></cylinder>",
                    "<ellipsoid><radii>1 1 1</radii></ellipsoid>"
                )
            ),
            "collision geometry",
        ),
        (
            format!(
                "{FIRST}{SECOND}{}",
                THIRD.replace(
                    "</collision>",
                    "<surface><friction><ode><mu>0.3</mu></ode></friction></surface></collision>"
                )
            ),
            "collision surface parameters",
        ),
        (
            format!(
                "{FIRST}{SECOND}{}",
                THIRD.replace("<pose>0 0.5 0 0 0 0</pose>", "").replace(
                    "<cylinder><radius>0.1</radius><length>0.4</length></cylinder>",
                    "<plane><normal>0 0 1</normal><size>1 1</size></plane>"
                )
            ),
            "compound plane collision geometry",
        ),
        (
            format!("{FIRST}{SECOND}{THIRD}{}", THIRD.replace("third", "fourth")),
            "collision count (expected one)",
        ),
        (String::new(), "collision count (expected one)"),
    ];
    for (collisions, expected) in bad {
        assert!(
            matches!(load(&xml(&collisions), PhysicsEngine::Dart), Err(LoadError::Unsupported { feature, .. }) if feature == expected),
            "{expected}"
        );
    }
    let offset_com = xml(&format!("{FIRST}{SECOND}"))
        .replace("<mass>2</mass>", "<mass>2</mass><pose>0 0 0.5 0 0 0</pose>");
    let mut sim = load(&offset_com, PhysicsEngine::Dart).unwrap();
    let original_ids = ids(&sim);
    assert_eq!(original_ids.len(), 5);
    let link_id = original_ids["default::body::link"];
    let half = std::f64::consts::FRAC_PI_4;
    let specs = [
        CollisionSpec {
            shape: Shape::Sphere { radius: 0.1 },
            position: [0.5, 0., 0.],
            orientation: [0., 0., 0., 1.],
        },
        CollisionSpec {
            shape: Shape::Box {
                half_extents: [0.1, 0.2, 0.3],
            },
            position: [-0.5, 0., 0.],
            orientation: [0., 0., half.sin(), half.cos()],
        },
    ];
    for epoch in 0..2 {
        assert_eq!(ids(&sim), original_ids);
        for (name, density) in [("first", 7.), ("second", 11.)] {
            let entity = entity_by_id(
                sim.world(),
                original_ids[&format!("default::body::link::{name}")],
            )
            .unwrap();
            assert_eq!(
                sim.world().get::<CollisionDensity>(entity),
                Some(&CollisionDensity(density))
            );
        }
        let mut control = DartBackend::new([0.; 3]).unwrap();
        let body = control
            .spawn_compound_link(
                CompoundBodySpec {
                    collisions: &specs,
                    position: [0., 0., 10.],
                    velocity: [0.; 3],
                    mass: Some(2.),
                },
                RotationSpec::default(),
                Inertia {
                    center: [0., 0., 0.5],
                    orientation: [0., 0., 0., 1.],
                    tensor: [1., 1., 1., 0., 0., 0.],
                },
            )
            .unwrap();
        for _ in 0..3 {
            queue_link_world_wrench(&mut sim, link_id, [1., 0., 0.], [0., 0.4, 0.2]).unwrap();
            control
                .add_world_wrench(body, [1., 0., 0.], [0., 0.4, 0.2])
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
        }
        if epoch == 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
    }
    let link = entity_by_id(sim.world(), link_id).unwrap();
    let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
    remove_model(&mut sim, original_ids["default::body"]).unwrap();
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .is_none()
    );
}

#[test]
fn one_and_two_offset_collisions_keep_one_link_body() {
    for collisions in [FIRST.to_string(), format!("{FIRST}{SECOND}")] {
        let source = xml(&collisions)
            .replace("<model name='body'>", "<model name='body'><frame name='offset' attached_to='link'><pose relative_to='link'>0.5 0 0 0 0 0</pose></frame>")
            .replace("<pose>0.5 0 0 0 0 0</pose>", "<pose relative_to='offset'>0 0 0 0 0 0</pose>");
        let mut sim = load(&source, PhysicsEngine::Dart).unwrap();
        let first = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::body::link::first")
            })
            .unwrap();
        close(
            first.get::<LocalPose>().unwrap().0.translation,
            [0.5, 0., 0.],
        );
        let original = ids(&sim);
        let count = if collisions == FIRST { 1 } else { 2 };
        assert_eq!(original.len(), 3 + count);
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<BodyHandle>())
                .count(),
            1
        );
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(ids(&sim), original);
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Collision))
                .count(),
            count
        );
    }
}

#[test]
fn static_compound_reset_retains_each_attached_collision() {
    let source = xml(&format!("{FIRST}{SECOND}{THIRD}")).replace(
        "<model name='body'>",
        "<model name='body'><static>true</static>",
    );
    let mut sim = load(&source, PhysicsEngine::Dart).unwrap();
    let original = ids(&sim);
    let link = entity_by_id(sim.world(), original["default::body::link"]).unwrap();
    let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
    assert_eq!(
        sim.world().resource::<PhysicsBackend>().is_dynamic(handle),
        Some(false)
    );
    for _ in 0..2 {
        sim.step_exact(3).unwrap();
        let link = entity_by_id(sim.world(), original["default::body::link"]).unwrap();
        assert_eq!(
            sim.world().get::<WorldPose>(link).unwrap().translation,
            [0., 0., 10.]
        );
        for (name, offset) in [
            ("first", [0.5, 0., 0.]),
            ("second", [-0.5, 0., 0.]),
            ("third", [0., 0.5, 0.]),
        ] {
            let child = entity_by_id(
                sim.world(),
                original[&format!("default::body::link::{name}")],
            )
            .unwrap();
            close(
                sim.world().get::<LocalPose>(child).unwrap().0.translation,
                offset,
            );
            close(
                sim.world().get::<WorldPose>(child).unwrap().translation,
                [offset[0], offset[1], 10.],
            );
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(ids(&sim), original);
    }
    remove_model(&mut sim, original["default::body"]).unwrap();
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .is_none()
    );
}

#[test]
fn singleton_offset_plane_keeps_its_previous_refusal() {
    let plane = FIRST.replace(
        "<sphere><radius>0.1</radius></sphere>",
        "<plane><normal>0 0 1</normal><size>1 1</size></plane>",
    );
    let source = xml(&plane).replace(
        "<model name='body'>",
        "<model name='body'><static>true</static>",
    );
    assert!(matches!(
        load(&source, PhysicsEngine::Dart),
        Err(LoadError::Unsupported {
            feature: "offset collision pose",
            ..
        })
    ));
}
