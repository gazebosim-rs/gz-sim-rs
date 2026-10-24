// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::{
    entity::Entity,
    hierarchy::{ChildOf, Children},
};
use gz_physics_rapier::RapierBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{BodyHandle, EntityKind, LocalName, LocalPose, Name, WorldPose, load_world};
use sdformat::NoResources;
use std::time::Duration;

fn load(xml: &str) -> Simulation {
    load_world(
        xml,
        "hierarchy.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn local(sim: &Simulation, name: &str) -> WorldPose {
    sim.world().get::<LocalPose>(entity(sim, name)).unwrap().0
}
fn near(actual: [f64; 3], expected: [f64; 3]) {
    for (a, b) in actual.into_iter().zip(expected) {
        assert!((a - b).abs() < 1e-12, "{actual:?} != {expected:?}");
    }
}

#[test]
fn ownership_and_local_poses_preserve_canonical_model_motion_and_ids() {
    let xml = include_str!("fixtures/fall.sdf").replace("<sensor name=", "<visual name='view'><pose>0 0 1 0 0 0</pose><geometry><sphere/></geometry></visual><sensor name=");
    let mut sim = load(&xml);
    let world = entity(&sim, "default");
    let ball = entity(&sim, "default::ball");
    let body = entity(&sim, "default::ball::body");
    let sensor = entity(&sim, "default::ball::body::imu");
    let visual = entity(&sim, "default::ball::body::view");
    assert_eq!(
        sim.world().get::<EntityKind>(world),
        Some(&EntityKind::World)
    );
    assert_eq!(
        sim.world().get::<EntityKind>(ball),
        Some(&EntityKind::Model)
    );
    assert_eq!(sim.world().get::<EntityKind>(body), Some(&EntityKind::Link));
    assert_eq!(
        sim.world().get::<EntityKind>(sensor),
        Some(&EntityKind::Sensor)
    );
    assert_eq!(
        sim.world().get::<EntityKind>(visual),
        Some(&EntityKind::Visual)
    );
    assert_eq!(sim.world().get::<ChildOf>(ball).unwrap().parent(), world);
    assert_eq!(sim.world().get::<ChildOf>(body).unwrap().parent(), ball);
    assert_eq!(sim.world().get::<ChildOf>(sensor).unwrap().parent(), body);
    assert_eq!(sim.world().get::<Children>(world).unwrap().len(), 2);
    assert_eq!(sim.world().get::<Children>(body).unwrap().len(), 3);
    assert_eq!(sim.world().get::<LocalName>(sensor).unwrap().0, "imu");
    let ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<Name>())
        .map(|e| (e.id(), e.get::<EntityId>().unwrap().get()))
        .collect();
    assert_eq!(ids.len(), 9);
    assert_eq!(
        ids.iter()
            .map(|(_, id)| *id)
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        9
    );
    sim.step_exact(3).unwrap();
    let world_body = sim.world().get::<WorldPose>(body).unwrap();
    assert!(world_body.translation[2] < 5.0);
    near(
        local(&sim, "default::ball").translation,
        world_body.translation,
    );
    near(local(&sim, "default::ball::body").translation, [0.0; 3]);
    near(
        local(&sim, "default::ground::base").translation,
        [0.0, 0.0, -0.5],
    );
    near(
        local(&sim, "default::ball::body::view").translation,
        [0.0, 0.0, 1.0],
    );
    let world_visual = sim.world().get::<WorldPose>(visual).unwrap();
    near(
        world_visual.translation,
        [0.0, 0.0, world_body.translation[2] + 1.0],
    );
    for (entity, id) in &ids {
        assert_eq!(entity_by_id(sim.world(), *id), Some(*entity));
    }
    sim.reset().unwrap();
    near(local(&sim, "default::ball").translation, [0.0, 0.0, 5.0]);
    near(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        [0.0, 0.0, 6.0],
    );
    for (entity, id) in &ids {
        assert_eq!(entity_by_id(sim.world(), *id), Some(*entity));
    }
}

#[test]
fn nested_canonical_links_and_relative_frames_keep_ownership_distinct() {
    let xml = include_str!("fixtures/nested.sdf");
    let mut sim = load(xml);
    sim.step_exact(3).unwrap();
    // Native pinned Physics::UpdateModelPose keeps these parent-relative values.
    near(
        local(&sim, "default::outer::inner").translation,
        [2.0, 0.0, 1.0],
    );
    near(
        local(&sim, "default::outer::inner::a").translation,
        [0.0, 1.0, 2.0],
    );
    near(
        local(&sim, "default::outer::inner::b").translation,
        [0.0, 3.0, 0.0],
    );
    near(
        local(&sim, "default::outer::other").translation,
        [5.0, 0.0, 1.0],
    );
    near(
        local(&sim, "default::outer::inner::a::v").translation,
        [0.0, 0.0, 1.0],
    );
    let outer = local(&sim, "default::outer");
    assert!(outer.translation[2] < 3.0);
    assert!((outer.rotation[2] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
    let b = entity(&sim, "default::outer::inner::b");
    let id = sim.world().get::<BodyHandle>(b).unwrap().id();
    sim.world_mut()
        .resource_mut::<RapierBackend>()
        .apply_impulse(id, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    sim.step_exact(1).unwrap();
    // A secondary link moves relative to its model, along the rotated local -Y.
    assert!(local(&sim, "default::outer::inner::b").translation[1] < 3.0);
    near(
        local(&sim, "default::outer::inner::a").translation,
        [0.0, 1.0, 2.0],
    );
    sim.reset().unwrap();
    near(local(&sim, "default::outer").translation, [1.0, 2.0, 3.0]);
    near(
        local(&sim, "default::outer::inner::b").translation,
        [0.0, 3.0, 0.0],
    );

    let static_xml = xml.replace(
        "<model name=\"outer\">",
        "<model name=\"outer\"><static>true</static>",
    );
    let mut fixed = load(&static_xml);
    fixed.step_exact(5).unwrap();
    near(local(&fixed, "default::outer").translation, [1.0, 2.0, 3.0]);
}

#[test]
fn explicit_canonical_link_drives_the_model_and_resolved_visual_frames() {
    let xml = include_str!("fixtures/nested.sdf")
        .replace(
            "<model name=\"inner\">",
            "<model name=\"inner\" canonical_link=\"b\">",
        )
        .replace(
            "<pose>0 0 1 0 0 0</pose>",
            "<pose relative_to='b'>0 0 1 0 0 0</pose>",
        );
    let mut sim = load(&xml);
    let b = entity(&sim, "default::outer::inner::b");
    let id = sim.world().get::<BodyHandle>(b).unwrap().id();
    sim.world_mut()
        .resource_mut::<RapierBackend>()
        .apply_impulse(id, [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    sim.step_exact(1).unwrap();
    assert!(local(&sim, "default::outer").translation[0] > 1.0);
    near(
        local(&sim, "default::outer::inner::b").translation,
        [0.0, 3.0, 0.0],
    );
    assert!(local(&sim, "default::outer::inner::a").translation[1] > 1.0);
    near(
        local(&sim, "default::outer::inner::a::v").translation,
        [0.0, 2.0, -1.0],
    );
}

#[test]
fn static_nested_canonical_link_is_not_the_world_attachment_frame() {
    let xml = include_str!("fixtures/nested.sdf").replace(
        "<model name=\"inner\">",
        "<model name=\"inner\"><static>true</static>",
    );
    let mut sim = load(&xml);
    sim.step_exact(3).unwrap();
    near(local(&sim, "default::outer").translation, [1.0, 2.0, 3.0]);
    near(
        local(&sim, "default::outer::inner").translation,
        [2.0, 0.0, 1.0],
    );
    near(
        local(&sim, "default::outer::inner::a").translation,
        [0.0, 1.0, 2.0],
    );
    // The other nested model still falls relative to the anchored outer model.
    assert!(local(&sim, "default::outer::other").translation[2] < 1.0);
}

#[test]
fn dynamic_models_without_a_canonical_link_report_the_unsupported_case() {
    let xml = "<sdf version='1.12'><world name='default'><model name='outer'><model name='empty'><static>true</static></model></model></world></sdf>";
    let error = load_world(
        xml,
        "empty-nested.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .err()
    .unwrap();
    assert!(matches!(
        error,
        gz_sim_scene::LoadError::Unsupported {
            feature: "dynamic model without a canonical link",
            ..
        }
    ));
}

#[test]
fn canonical_links_cannot_bind_to_another_top_level_model() {
    let xml = include_str!("fixtures/fall.sdf").replace(
        "<model name=\"ball\">",
        "<model name=\"ball\" canonical_link=\"::default::ground::base\">",
    );
    let error = load_world(
        &xml,
        "foreign-canonical.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .err()
    .unwrap();
    assert!(
        matches!(error, gz_sim_scene::LoadError::Sdf(diagnostic) if diagnostic.code == sdformat::ErrorCode::UnknownReference)
    );
}
