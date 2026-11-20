// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded posed-box Source regressions; all execution waits for whole review.
use bevy_ecs::entity::Entity;
use gz_physics::{BodyId, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, LoadError, LocalPose, Name, PhysicsEngine, WorldPose, load_world_with_engine,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const FIXTURE: &str = include_str!("fixtures/m78_posed_box.sdf");
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_engine(
        xml,
        "posed-box.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
        engine,
    )
}

fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("named entity")
        .id()
}

fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}

fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(named(sim, name))
        .unwrap()
        .id()
}

fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *sim.world().get::<WorldPose>(named(sim, name)).unwrap()
}

fn near(actual: [f64; 3], expected: [f64; 3]) {
    for (a, b) in actual.into_iter().zip(expected) {
        assert!((a - b).abs() < 1e-11, "{actual:?} != {expected:?}");
    }
}

fn ray(
    sim: &Simulation,
    origin: [f64; 3],
    direction: [f64; 3],
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, direction, 20., exclude)
        .unwrap()
}

fn world(models: &str, gravity: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>{gravity}</gravity>{models}</world></sdf>"
    )
}

fn dynamic(collision_pose: &str, inertia: &str) -> String {
    world(
        &format!(
            "<model name='posed'><pose>0 0 5 0 0 0</pose><link name='body'>{inertia}<collision name='shape'><pose>{collision_pose}</pose><geometry><box><size>1 2 3</size></box></geometry></collision></link></model>"
        ),
        "0 0 0",
    )
}

#[test]
fn resolved_box_faces_preserve_link_visual_sensor_frames_and_ray_exclusion() {
    let mut sim = load(FIXTURE, PhysicsEngine::Rapier).unwrap();
    let handle = body(&sim, "default::posed::body");
    // Independent quarter-turn matrices give center(.75,5.75,4), with
    // local box X/Y/Z mapped to world -Y/+Z/-X respectively.
    for (origin, direction, distance) in [
        ([0.75, 5.75, 8.], [0., 0., -1.], 3.),
        ([5., 5.75, 4.], [-1., 0., 0.], 2.75),
        ([0.75, 9., 4.], [0., -1., 0.], 2.75),
    ] {
        assert!((ray(&sim, origin, direction, None).unwrap() - distance).abs() < 1e-11);
        assert_eq!(ray(&sim, origin, direction, Some(handle)), None);
    }
    // The old centered collider would hit here and miss the first ray above.
    assert_eq!(ray(&sim, [1., 4., 8.], [0., 0., -1.], None), None);
    assert_eq!(ray(&sim, [2.3, 5.75, 8.], [0., 0., -1.], None), None);
    near(pose(&sim, "default::posed::body").translation, [1., 4., 4.]);
    near(
        pose(&sim, "default::posed::body::view").translation,
        [1., 4., 6.],
    );
    near(
        pose(&sim, "default::posed::body::imu").translation,
        [1., 4., 4.],
    );
    let local = sim
        .world()
        .get::<LocalPose>(named(&sim, "default::posed::body::shape"))
        .unwrap()
        .0;
    near(local.translation, [0.25, -1.75, 0.]);
    assert!((local.rotation.into_iter().sum::<f64>().abs() - 2.).abs() < 1e-11);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<BodyHandle>())
            .count(),
        2
    );
    let initial = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    sim.step_exact(10).unwrap();
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap(),
        initial
    );
}

#[test]
fn posed_box_reset_and_removal_restore_geometry_and_public_identities() {
    let mut sim = load(FIXTURE, PhysicsEngine::Rapier).unwrap();
    let model = id(&sim, "default::posed");
    let link = id(&sim, "default::posed::body");
    let survivor = body(&sim, "default::survivor::body");
    let handle = body(&sim, "default::posed::body");
    sim.step_exact(5).unwrap();
    sim.reset().unwrap();
    assert!((ray(&sim, [0.75, 5.75, 8.], [0., 0., -1.], None).unwrap() - 3.).abs() < 1e-11);
    assert_eq!(body(&sim, "default::posed::body"), handle);
    remove_model(&mut sim, model).unwrap();
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .is_none()
    );
    assert_eq!(ray(&sim, [0.75, 5.75, 8.], [0., 0., -1.], None), None);
    assert!((ray(&sim, [20., 0., 3.], [0., 0., -1.], None).unwrap() - 2.5).abs() < 1e-12);
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(survivor)
            .is_some()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "default::posed"), model);
    assert_eq!(id(&sim, "default::posed::body"), link);
    assert_eq!(body(&sim, "default::posed::body"), handle);
    assert_eq!(body(&sim, "default::survivor::body"), survivor);
    assert!((ray(&sim, [0.75, 5.75, 8.], [0., 0., -1.], None).unwrap() - 3.).abs() < 1e-11);
    near(
        pose(&sim, "default::posed::body::imu").translation,
        [1., 4., 4.],
    );
}

#[test]
fn displaced_box_uses_link_centered_mass_and_inertia_for_world_impulses() {
    let mut sim = load(
        &dynamic("1 -0.5 0.25 0.2 -0.3 0.4", INERTIA),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let handle = body(&sim, "default::posed::body");
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_impulse(handle, [2., 0., 0.], [0.; 3])
        .unwrap();
    let linear = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    near(linear.position, [0., 0., 5.]);
    near(linear.velocity, [1., 0., 0.]);
    near(linear.angular_velocity, [0.; 3]);
    sim.step_exact(100).unwrap();
    near(
        pose(&sim, "default::posed::body").translation,
        [0.1, 0., 5.],
    );
    sim.reset().unwrap();
    let initial = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    near(initial.position, [0., 0., 5.]);
    near(initial.velocity, [0.; 3]);
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_impulse(handle, [0.; 3], [0., 0., 4.])
        .unwrap();
    let angular = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    near(angular.velocity, [0.; 3]);
    near(angular.angular_velocity, [0., 0., 1.]);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap(),
        initial
    );
}

#[test]
fn omitted_xml_inertia_retains_sdformat_defaults_instead_of_collider_mass() {
    // Pinned SDFormat defaults are mass1 and the identity inertia tensor;
    // Scene supplies those resolved values explicitly to the backend API.
    let mut sim = load(&dynamic("1 0 0 0 0 0.7", ""), PhysicsEngine::Rapier).unwrap();
    let handle = body(&sim, "default::posed::body");
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_impulse(handle, [1., 0., 0.], [0., 0., 1.])
        .unwrap();
    let state = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap();
    near(state.position, [0., 0., 5.]);
    near(state.velocity, [1., 0., 0.]);
    near(state.angular_velocity, [0., 0., 1.]);
}

#[test]
fn offset_contact_surface_supports_a_ball_that_the_centered_control_misses() {
    let models = "<model name='ground'><static>true</static><link name='body'><collision name='shape'><pose>1.5 0 1 0 0 1.5707963267948966</pose><geometry><box><size>2 0.4 0.4</size></box></geometry></collision></link></model><model name='ball'><pose>1.5 0 2 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link></model>";
    let mut posed = load(&world(models, "0 0 -9.81"), PhysicsEngine::Rapier).unwrap();
    let mut centered = load(
        &world(
            &models.replace("1.5 0 1 0 0 1.5707963267948966", "0 0 0 0 0 0"),
            "0 0 -9.81",
        ),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    posed.step_exact(1000).unwrap();
    centered.step_exact(1000).unwrap();
    assert!((pose(&posed, "default::ball::body").translation[2] - 1.4).abs() < 0.01);
    assert!(pose(&centered, "default::ball::body").translation[2] < 0.);
    near(pose(&posed, "default::ground::body").translation, [0.; 3]);
    reset_loaded_world(&mut posed, |_| ()).unwrap();
    posed.step_exact(1000).unwrap();
    assert!((pose(&posed, "default::ball::body").translation[2] - 1.4).abs() < 0.01);
}

#[test]
fn posed_box_links_retain_same_model_contact_exclusion_and_distinct_model_contacts() {
    let first = format!(
        "<link name='a'>{INERTIA}<collision name='shape'><pose>1 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision></link>"
    );
    let second = first.replace("name='a'", "name='b'").replace(
        "<link name='b'>",
        "<link name='b'><pose>0.75 0 0 0 0 0</pose>",
    );
    let same = format!("<model name='same'>{first}{second}</model>");
    let different =
        format!("<model name='first'>{first}</model><model name='second'>{second}</model>");
    let mut excluded = load(&world(&same, "0 0 0"), PhysicsEngine::Rapier).unwrap();
    let mut contacts = load(&world(&different, "0 0 0"), PhysicsEngine::Rapier).unwrap();
    excluded.step_exact(100).unwrap();
    contacts.step_exact(100).unwrap();
    near(pose(&excluded, "default::same::a").translation, [0.; 3]);
    near(
        pose(&excluded, "default::same::b").translation,
        [0.75, 0., 0.],
    );
    assert!(pose(&contacts, "default::first::a").translation[0] < -1e-7);
    assert!(pose(&contacts, "default::second::b").translation[0] > 0.7500001);
}

#[test]
fn identity_and_old_refusal_domains_remain_separate_from_posed_boxes() {
    let offset_imu = FIXTURE.replace(
        "<pose relative_to=\"mount\">0.5 2 -1 0 0 0</pose>",
        "<pose>0 0 0.2 0 0 0</pose>",
    );
    assert_ne!(offset_imu, FIXTURE);
    assert!(matches!(
        load(&offset_imu, PhysicsEngine::Rapier),
        Err(LoadError::Unsupported {
            feature: "offset IMU position",
            ..
        })
    ));
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(load(&dynamic("0 0 0 0 0 0", INERTIA), engine).is_ok());
    }
    let identity = load(&dynamic("0 0 0 0 0 0", INERTIA), PhysicsEngine::Rapier).unwrap();
    let rotated = load(
        &dynamic("0 0 0 0 0 1.5707963267948966", INERTIA),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert_eq!(ray(&identity, [0.75, 0., 9.], [0., 0., -1.], None), None);
    assert!((ray(&rotated, [0.75, 0., 9.], [0., 0., -1.], None).unwrap() - 2.5).abs() < 1e-11);
    assert!(matches!(
        load(&dynamic("1 0 0 0 0 0", INERTIA), PhysicsEngine::Tpe),
        Err(LoadError::Unsupported {
            feature: "offset collision pose",
            ..
        })
    ));
    let box_xml = dynamic("1 0 0 0 0 0.2", INERTIA);
    // SDFormat normalizes finite nonzero XML quaternions before Scene receives
    // them; the direct backend API independently requires unit input.
    let normalized_xml = box_xml.replace(
        "<pose>1 0 0 0 0 0.2</pose>",
        "<pose rotation_format='quat_xyzw'>1 0 0 0 0 0 2</pose>",
    );
    let normalized = load(&normalized_xml, PhysicsEngine::Rapier).unwrap();
    let canonical = load(&dynamic("1 0 0 0 0 0", INERTIA), PhysicsEngine::Rapier).unwrap();
    assert_eq!(
        normalized
            .world()
            .resource::<PhysicsBackend>()
            .state(body(&normalized, "default::posed::body")),
        canonical
            .world()
            .resource::<PhysicsBackend>()
            .state(body(&canonical, "default::posed::body"))
    );
    assert!((ray(&normalized, [1., 0., 9.], [0., 0., -1.], None).unwrap() - 2.5).abs() < 1e-11);
    for origin in [[1., 0., 9.], [1.4, 0.9, 9.], [1.6, 0., 9.], [0., 0., 9.]] {
        assert_eq!(
            ray(&normalized, origin, [0., 0., -1.], None),
            ray(&canonical, origin, [0., 0., -1.], None)
        );
    }
    // Pinned SDFormat defaults a box with any nonpositive dimension to a unit cube.
    let zero_size_xml = box_xml.replace("<size>1 2 3</size>", "<size>0 2 3</size>");
    let unit_size_xml = box_xml.replace("<size>1 2 3</size>", "<size>1 1 1</size>");
    assert_ne!(zero_size_xml, box_xml);
    assert_ne!(unit_size_xml, box_xml);
    let zero_size = load(&zero_size_xml, PhysicsEngine::Rapier).unwrap();
    let unit_size = load(&unit_size_xml, PhysicsEngine::Rapier).unwrap();
    for origin in [[1., 0., 9.], [1.2, 0.2, 9.], [3., 0., 9.]] {
        assert_eq!(
            ray(&zero_size, origin, [0., 0., -1.], None),
            ray(&unit_size, origin, [0., 0., -1.], None)
        );
    }
    assert!((ray(&zero_size, [1., 0., 9.], [0., 0., -1.], None).unwrap() - 3.5).abs() < 1e-11);
    assert!((ray(&canonical, [1., 0., 9.], [0., 0., -1.], None).unwrap() - 2.5).abs() < 1e-11);
    for geometry in [
        "<sphere><radius>0.5</radius></sphere>",
        "<cylinder><radius>0.5</radius><length>1</length></cylinder>",
        "<capsule><radius>0.5</radius><length>1</length></capsule>",
    ] {
        let other = box_xml.replace("<box><size>1 2 3</size></box>", geometry);
        assert!(matches!(
            load(&other, PhysicsEngine::Rapier),
            Err(LoadError::Unsupported {
                feature: "offset collision pose",
                ..
            })
        ));
    }
    let compound = box_xml.replace(
        "</link>",
        "<collision name='extra'><geometry><box/></geometry></collision></link>",
    );
    assert!(matches!(
        load(&compound, PhysicsEngine::Rapier),
        Err(LoadError::Unsupported {
            feature: "collision count (expected one)",
            ..
        })
    ));
    let offset_com = box_xml.replace("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>");
    assert!(matches!(
        load(&offset_com, PhysicsEngine::Rapier),
        Err(LoadError::Unsupported {
            feature: "offset center of mass",
            ..
        })
    ));
    let surface = box_xml.replace(
        "</collision>",
        "<surface><friction><ode><mu>0.3</mu></ode></friction></surface></collision>",
    );
    assert!(matches!(
        load(&surface, PhysicsEngine::Rapier),
        Err(LoadError::Unsupported {
            feature: "collision surface parameters",
            ..
        })
    ));
    let zero_quaternion = box_xml.replace(
        "<pose>1 0 0 0 0 0.2</pose>",
        "<pose rotation_format='quat_xyzw'>1 0 0 0 0 0 0</pose>",
    );
    for (case, invalid) in [
        zero_quaternion,
        box_xml.replace("<size>1 2 3</size>", "<size>1e308 2 3</size>"),
        box_xml.replace("<mass>2</mass>", "<mass>-2</mass>"),
        box_xml.replace("<ixx>2</ixx>", "<ixx>-2</ixx>"),
        box_xml.replace("1 0 0 0 0 0.2", "nan 0 0 0 0 0"),
        box_xml
            .replace("1 0 0 0 0 0.2", "1e308 0 0 0 0 0.2")
            .replace("<pose>0 0 5 0 0 0</pose>", "<pose>1e308 0 5 0 0 0</pose>"),
    ]
    .into_iter()
    .enumerate()
    {
        assert_ne!(
            invalid, box_xml,
            "invalid fixture mutation {case} was a no-op"
        );
        assert!(
            load(&invalid, PhysicsEngine::Rapier).is_err(),
            "invalid fixture case {case}"
        );
    }
}
