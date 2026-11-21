// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolved cylinder geometry with unchanged link, sensor and controller frames.
use bevy_ecs::entity::Entity;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, LoadError, LocalPose, Name, PhysicsEngine, WorldPose, load_world_with_engine,
    remove_model, reset_loaded_world, set_velocity_control_command,
    velocity_control_binding_matches,
};
use sdformat::NoResources;
use std::time::Duration;

const FIXTURE: &str = include_str!("fixtures/m80_posed_cylinder.sdf");
const GEOMETRY: &str = "<cylinder><radius>0.5</radius><length>3</length></cylinder>";
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const PLUGIN: &str = "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><initial_linear>1 0 0</initial_linear><initial_angular>0 0 0</initial_angular></plugin>";

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_engine(
        xml,
        "posed-cylinder.sdf",
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

fn state(sim: &Simulation, name: &str) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body(sim, name))
        .unwrap()
}

fn near(actual: [f64; 3], expected: [f64; 3]) {
    for (a, b) in actual.into_iter().zip(expected) {
        assert!(
            a.is_finite() && (a - b).abs() < 1e-11,
            "{actual:?} != {expected:?}"
        );
    }
}

fn same_rotation(actual: [f64; 4], expected: [f64; 4]) {
    let dot: f64 = actual.into_iter().zip(expected).map(|(a, b)| a * b).sum();
    assert!((dot.abs() - 1.).abs() < 1e-11, "{actual:?} != {expected:?}");
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

fn distance(sim: &Simulation, origin: [f64; 3], direction: [f64; 3], expected: f64) {
    let actual = ray(sim, origin, direction, None).expect("analytic cylinder hit");
    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
}

fn world(models: &str, gravity: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>{gravity}</gravity>{models}</world></sdf>"
    )
}

fn dynamic(collision_pose: &str, inertia: &str) -> String {
    world(
        &format!(
            "<model name='posed'><pose>0 0 5 0 0 0</pose><link name='body'>{inertia}<collision name='shape'><pose>{collision_pose}</pose><geometry>{GEOMETRY}</geometry></collision></link></model>"
        ),
        "0 0 0",
    )
}

#[test]
fn resolved_cylinder_axis_and_radial_faces_preserve_all_reported_frames() {
    let mut sim = load(FIXTURE, PhysicsEngine::Rapier).unwrap();
    let handle = body(&sim, "default::posed::body");
    // Independent quarter-turn composition gives center(.75,5.75,4) and
    // cylinder axis -X. The radius is .5 and the axial half-length is 1.5.
    for (origin, direction, expected) in [
        ([5., 5.75, 4.], [-1., 0., 0.], 2.75),
        ([-4., 5.75, 4.], [1., 0., 0.], 3.25),
        ([0.75, 5.75, 8.], [0., 0., -1.], 3.5),
        ([0.75, 9., 4.], [0., -1., 0.], 2.75),
        ([0.75, 6.05, 8.], [0., 0., -1.], 3.6),
    ] {
        distance(&sim, origin, direction, expected);
        assert_eq!(ray(&sim, origin, direction, Some(handle)), None);
    }
    assert_eq!(ray(&sim, [0.75, 6.251, 8.], [0., 0., -1.], None), None);
    assert_eq!(ray(&sim, [1., 4., 8.], [0., 0., -1.], None), None);
    near(pose(&sim, "default::posed::body").translation, [1., 4., 4.]);
    same_rotation(
        pose(&sim, "default::posed::body").rotation,
        [0., 0., 1., 0.],
    );
    near(
        pose(&sim, "default::posed::body::shape").translation,
        [0.75, 5.75, 4.],
    );
    same_rotation(
        pose(&sim, "default::posed::body::shape").rotation,
        [-0.5, 0.5, 0.5, -0.5],
    );
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
    same_rotation(local.rotation, [0.5; 4]);
    let initial = state(&sim, "default::posed::body");
    sim.step_exact(10).unwrap();
    assert_eq!(state(&sim, "default::posed::body"), initial);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<BodyHandle>())
            .count(),
        2
    );
}

#[test]
fn cylinder_tangent_identity_and_sdformat_normalizations_keep_distinct_geometry() {
    let posed_xml = dynamic("1 -2 0.5 0 0 0", INERTIA);
    let posed = load(&posed_xml, PhysicsEngine::Rapier).unwrap();
    // Parallel to the Gazebo Z axis: exactly tangent, interior and exterior.
    distance(&posed, [1.5, -2., 9.], [0., 0., -1.], 2.);
    distance(&posed, [1.49, -2., 9.], [0., 0., -1.], 2.);
    assert_eq!(ray(&posed, [1.51, -2., 9.], [0., 0., -1.], None), None);
    let origin_xml = dynamic("0 0 0 0 0 0", INERTIA);
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(load(&origin_xml, engine).is_ok());
    }
    let origin = load(&origin_xml, PhysicsEngine::Rapier).unwrap();
    distance(&origin, [0., 0., 9.], [0., 0., -1.], 2.5);
    assert_eq!(ray(&origin, [1., -2., 9.], [0., 0., -1.], None), None);
    let quaternion_xml = posed_xml.replace(
        "<pose>1 -2 0.5 0 0 0</pose>",
        "<pose rotation_format='quat_xyzw'>1 -2 0.5 0 0 0 2</pose>",
    );
    assert_ne!(quaternion_xml, posed_xml);
    let normalized = load(&quaternion_xml, PhysicsEngine::Rapier).unwrap();
    assert_eq!(
        state(&normalized, "default::posed::body"),
        state(&posed, "default::posed::body")
    );
    for x in [1., 1.3, 1.49, 1.51] {
        assert_eq!(
            ray(&normalized, [x, -2., 9.], [0., 0., -1.], None),
            ray(&posed, [x, -2., 9.], [0., 0., -1.], None)
        );
    }
    // Pinned SDFormat replaces a nonpositive cylinder radius/length with .5/1.
    let defaults_xml = posed_xml.replace(
        "<radius>0.5</radius><length>3</length>",
        "<radius>-1</radius><length>0</length>",
    );
    let unit_xml = posed_xml.replace("<length>3</length>", "<length>1</length>");
    assert_ne!(defaults_xml, posed_xml);
    let defaults = load(&defaults_xml, PhysicsEngine::Rapier).unwrap();
    let unit = load(&unit_xml, PhysicsEngine::Rapier).unwrap();
    for x in [1., 1.3, 1.49, 1.51] {
        assert_eq!(
            ray(&defaults, [x, -2., 9.], [0., 0., -1.], None),
            ray(&unit, [x, -2., 9.], [0., 0., -1.], None)
        );
    }
    distance(&defaults, [1., -2., 9.], [0., 0., -1.], 3.);
}

#[test]
fn posed_cylinder_ordinary_prepared_and_loaded_resets_preserve_geometry_and_ids() {
    let mut sim = load(FIXTURE, PhysicsEngine::Rapier).unwrap();
    let model = id(&sim, "default::posed");
    let link = id(&sim, "default::posed::body");
    let handle = body(&sim, "default::posed::body");
    let survivor = body(&sim, "default::survivor::body");
    sim.step_exact(5).unwrap();
    sim.reset().unwrap();
    distance(&sim, [0.75, 5.75, 8.], [0., 0., -1.], 3.5);
    assert_eq!(body(&sim, "default::posed::body"), handle);
    let initial = sim.world().resource::<PhysicsBackend>().initial_state();
    assert!(
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .remove(handle)
    );
    let prepared = sim
        .world()
        .resource::<PhysicsBackend>()
        .prepare_initial(&initial)
        .unwrap();
    // Preparation is read-only; the missing collider returns only on apply.
    assert_eq!(ray(&sim, [0.75, 5.75, 8.], [0., 0., -1.], None), None);
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_initial(prepared)
        .unwrap();
    distance(&sim, [0.75, 5.75, 8.], [0., 0., -1.], 3.5);
    assert_eq!(body(&sim, "default::posed::body"), handle);
    let removed = remove_model(&mut sim, model).unwrap();
    assert!(removed.contains(&model) && removed.contains(&link));
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .is_none()
    );
    assert_eq!(ray(&sim, [0.75, 5.75, 8.], [0., 0., -1.], None), None);
    distance(&sim, [20., 0., 3.], [0., 0., -1.], 2.5);
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
    distance(&sim, [0.75, 5.75, 8.], [0., 0., -1.], 3.5);
    near(
        pose(&sim, "default::posed::body::view").translation,
        [1., 4., 6.],
    );
    near(
        pose(&sim, "default::posed::body::imu").translation,
        [1., 4., 4.],
    );
}

#[test]
fn displaced_cylinder_uses_explicit_link_centered_mass_and_world_impulses() {
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
    near(state(&sim, "default::posed::body").position, [0., 0., 5.]);
    near(state(&sim, "default::posed::body").velocity, [1., 0., 0.]);
    near(
        state(&sim, "default::posed::body").angular_velocity,
        [0.; 3],
    );
    sim.step_exact(100).unwrap();
    near(
        pose(&sim, "default::posed::body").translation,
        [0.1, 0., 5.],
    );
    sim.reset().unwrap();
    let initial = state(&sim, "default::posed::body");
    near(initial.velocity, [0.; 3]);
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_impulse(handle, [0.; 3], [0., 0., 4.])
        .unwrap();
    near(state(&sim, "default::posed::body").velocity, [0.; 3]);
    near(
        state(&sim, "default::posed::body").angular_velocity,
        [0., 0., 1.],
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(state(&sim, "default::posed::body"), initial);
}

#[test]
fn cylinder_resolved_default_and_centered_upstream_inertia_are_not_collider_offsets() {
    let mut defaults = load(&dynamic("1 0 0 0 0 0.7", ""), PhysicsEngine::Rapier).unwrap();
    let handle = body(&defaults, "default::posed::body");
    defaults
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_impulse(handle, [1., 0., 0.], [0., 0., 1.])
        .unwrap();
    near(
        state(&defaults, "default::posed::body").velocity,
        [1., 0., 0.],
    );
    near(
        state(&defaults, "default::posed::body").angular_velocity,
        [0., 0., 1.],
    );
    // The pinned parser, not Scene, resolves this centered automatic tensor.
    // Gazebo Z rotated about Y becomes X: Ixx=M*r²/2 and mass=750*pi.
    let mut resolved = load(
        &dynamic("0 0 0 0 1.5707963267948966 0", "<inertial auto='true'/>"),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let mass = 750. * std::f64::consts::PI;
    let handle = body(&resolved, "default::posed::body");
    resolved
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_impulse(handle, [mass, 0., 0.], [mass * 0.125, 0., 0.])
        .unwrap();
    near(
        state(&resolved, "default::posed::body").position,
        [0., 0., 5.],
    );
    near(
        state(&resolved, "default::posed::body").velocity,
        [1., 0., 0.],
    );
    near(
        state(&resolved, "default::posed::body").angular_velocity,
        [1., 0., 0.],
    );
    assert!(matches!(
        load(
            &dynamic("1 0 0 0 0 0", "<inertial auto='true'/>"),
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "offset center of mass",
            ..
        })
    ));
}

#[test]
fn offset_cylinder_cap_supports_a_ball_that_the_centered_control_misses() {
    let models = "<model name='ground'><static>true</static><link name='body'><collision name='shape'><pose>1.5 0 1 0 0 0.7</pose><geometry><cylinder><radius>0.4</radius><length>2</length></cylinder></geometry></collision></link></model><model name='ball'><pose>1.5 0 2.6 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link></model>";
    let mut posed = load(&world(models, "0 0 -9.81"), PhysicsEngine::Rapier).unwrap();
    let control = models.replace("1.5 0 1 0 0 0.7", "0 0 0 0 0 0.7");
    assert_ne!(control, models);
    let mut centered = load(&world(&control, "0 0 -9.81"), PhysicsEngine::Rapier).unwrap();
    let support = ray(
        &posed,
        [1.5, 0., 4.],
        [0., 0., -1.],
        Some(body(&posed, "default::ball::body")),
    )
    .unwrap();
    assert!((support - 2.).abs() < 1e-11);
    assert_eq!(
        ray(
            &centered,
            [1.5, 0., 4.],
            [0., 0., -1.],
            Some(body(&centered, "default::ball::body"))
        ),
        None
    );
    posed.step_exact(1000).unwrap();
    centered.step_exact(1000).unwrap();
    assert!((pose(&posed, "default::ball::body").translation[2] - 2.2).abs() < 0.01);
    assert!(pose(&centered, "default::ball::body").translation[2] < 0.);
    near(pose(&posed, "default::ground::body").translation, [0.; 3]);
    reset_loaded_world(&mut posed, |_| ()).unwrap();
    posed.step_exact(1000).unwrap();
    assert!((pose(&posed, "default::ball::body").translation[2] - 2.2).abs() < 0.01);
}

#[test]
fn posed_cylinder_links_preserve_same_model_exclusion_and_distinct_model_contacts() {
    let first = format!(
        "<link name='a'>{INERTIA}<collision name='shape'><pose>1 0 0 0 0 0</pose><geometry>{GEOMETRY}</geometry></collision></link>"
    );
    let second = first.replace("name='a'", "name='b'").replace(
        "<link name='b'>",
        "<link name='b'><pose>0.75 0 0 0 0 0</pose>",
    );
    let mut same = load(
        &world(
            &format!("<model name='same'>{first}{second}</model>"),
            "0 0 0",
        ),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let mut different = load(
        &world(
            &format!("<model name='first'>{first}</model><model name='second'>{second}</model>"),
            "0 0 0",
        ),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    same.step_exact(100).unwrap();
    different.step_exact(100).unwrap();
    near(pose(&same, "default::same::a").translation, [0.; 3]);
    near(pose(&same, "default::same::b").translation, [0.75, 0., 0.]);
    assert!(pose(&different, "default::first::a").translation[0] < -1e-7);
    assert!(pose(&different, "default::second::b").translation[0] > 0.7500001);
}

#[test]
fn posed_cylinder_refusals_keep_tpe_compounds_com_and_other_pose_domains_closed() {
    let xml = dynamic("1 0 0 0 0 0.2", INERTIA);
    assert!(matches!(
        load(&xml, PhysicsEngine::Tpe),
        Err(LoadError::Unsupported {
            feature: "offset collision pose",
            ..
        })
    ));
    for (source, feature) in [
        (xml.replace("</link>", "<collision name='extra'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>"), "collision count (expected one)"),
        (xml.replace("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>"), "offset center of mass"),
        (xml.replace("</collision>", "<surface><friction><ode><mu>0.3</mu></ode></friction></surface></collision>"), "collision surface parameters"),
    ] {
        assert_ne!(source, xml);
        assert!(matches!(load(&source, PhysicsEngine::Rapier), Err(LoadError::Unsupported { feature: actual, .. }) if actual==feature));
    }
    // M81 explicitly replaces only the old posed-capsule refusal. Use a
    // zero-rotation offset fixture and an independent hemispherical chord.
    let capsule_xml = xml
        .replace(
            GEOMETRY,
            "<capsule><radius>0.5</radius><length>3</length></capsule>",
        )
        .replace("1 0 0 0 0 0.2", "1 0 0 0 0 0");
    let capsule = load(&capsule_xml, PhysicsEngine::Rapier).unwrap();
    distance(&capsule, [1.3, 0., 9.], [0., 0., -1.], 2.1);
    // M82 replaces only the Sphere refusal, retaining the original tolerance.
    // Center(1,0,5) and radius .5 give axial3.5 and transverse.3 chord3.6.
    let sphere = load(
        &xml.replace(GEOMETRY, "<sphere><radius>0.5</radius></sphere>"),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    distance(&sphere, [1., 0., 9.], [0., 0., -1.], 3.5);
    distance(&sphere, [1.3, 0., 9.], [0., 0., -1.], 3.6);
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
    for (case, invalid) in [
        xml.replace(
            "<pose>1 0 0 0 0 0.2</pose>",
            "<pose rotation_format='quat_xyzw'>1 0 0 0 0 0 0</pose>",
        ),
        xml.replace("<radius>0.5</radius>", "<radius>1e308</radius>"),
        xml.replace("<length>3</length>", "<length>1e308</length>"),
        xml.replace("<mass>2</mass>", "<mass>-2</mass>"),
        xml.replace("<ixx>2</ixx>", "<ixx>-2</ixx>"),
        xml.replace("1 0 0 0 0 0.2", "nan 0 0 0 0 0"),
        xml.replace("1 0 0 0 0 0.2", "1e308 0 0 0 0 0.2")
            .replace("<pose>0 0 5 0 0 0</pose>", "<pose>1e308 0 5 0 0 0</pose>"),
    ]
    .into_iter()
    .enumerate()
    {
        assert_ne!(invalid, xml, "invalid fixture mutation {case} was a no-op");
        assert!(
            load(&invalid, PhysicsEngine::Rapier).is_err(),
            "invalid cylinder case {case}"
        );
    }
}

#[test]
fn posed_cylinder_coexists_with_an_unchanged_centered_velocity_controller_only() {
    let rig = format!(
        "<model name='rig'><pose>30 0 10 0 0 0</pose>{PLUGIN}<link name='body'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>"
    );
    let coexist = FIXTURE.replace("</world>", &format!("{rig}</world>"));
    let mut sim = load(&coexist, PhysicsEngine::Rapier).unwrap();
    let mut control = load(&world(&rig, "0 0 0"), PhysicsEngine::Rapier).unwrap();
    let owner = id(&sim, "default::rig");
    let control_owner = id(&control, "default::rig");
    let posed = id(&sim, "default::posed");
    assert!(velocity_control_binding_matches(&sim, owner));
    assert!(!velocity_control_binding_matches(&sim, posed));
    set_velocity_control_command(&mut sim, owner, [0., 2., 0.], [0., 0., 0.5]).unwrap();
    set_velocity_control_command(&mut control, control_owner, [0., 2., 0.], [0., 0., 0.5]).unwrap();
    for _ in 0..20 {
        sim.step_exact(1).unwrap();
        control.step_exact(1).unwrap();
        let actual = state(&sim, "default::rig::body");
        let expected = state(&control, "default::rig::body");
        near(actual.position, expected.position);
        near(actual.velocity, expected.velocity);
        near(actual.angular_velocity, expected.angular_velocity);
        same_rotation(actual.orientation, expected.orientation);
        distance(&sim, [0.75, 5.75, 8.], [0., 0., -1.], 3.5);
    }
    remove_model(&mut sim, posed).unwrap();
    sim.step_exact(3).unwrap();
    control.step_exact(3).unwrap();
    near(
        state(&sim, "default::rig::body").position,
        state(&control, "default::rig::body").position,
    );
    assert!(velocity_control_binding_matches(&sim, owner));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "default::rig"), owner);
    sim.step_exact(1).unwrap();
    near(state(&sim, "default::rig::body").velocity, [1., 0., 0.]);
    distance(&sim, [0.75, 5.75, 8.], [0., 0., -1.], 3.5);
    let controlled_pose = dynamic("1 0 0 0 0 0", INERTIA)
        .replace("<link name='body'>", &format!("{PLUGIN}<link name='body'>"));
    assert!(matches!(
        load(&controlled_pose, PhysicsEngine::Rapier),
        Err(LoadError::Unsupported {
            feature: "VelocityControl centered primitive",
            ..
        })
    ));
}
