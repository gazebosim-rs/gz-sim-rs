// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolved sphere geometry with unchanged link, sensor and controller frames.
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

const FIXTURE: &str = include_str!("fixtures/m82_posed_sphere.sdf");
const GEOMETRY: &str = "<sphere><radius>0.5</radius></sphere>";
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const PLUGIN: &str = "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><initial_linear>1 0 0</initial_linear><initial_angular>0 0 0</initial_angular></plugin>";

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_engine(
        xml,
        "posed-sphere.sdf",
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
    let actual = ray(sim, origin, direction, None).expect("analytic sphere hit");
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
fn resolved_sphere_center_and_chords_preserve_all_reported_frames() {
    let mut sim = load(FIXTURE, PhysicsEngine::Rapier).unwrap();
    let handle = body(&sim, "default::posed::body");
    // Independent quarter-turn frame composition gives center(.75,5.75,4).
    // Local rotation preserves the sphere but must not rotate its offset again.
    for (origin, direction, expected) in [
        ([5., 5.75, 4.], [-1., 0., 0.], 3.75),
        ([-4., 5.75, 4.], [1., 0., 0.], 4.25),
        ([0.75, 5.75, 8.], [0., 0., -1.], 3.5),
        ([0.75, 9., 4.], [0., -1., 0.], 2.75),
        // Radius .5 and transverse .3 give the independent .4 chord root.
        ([0.75, 6.05, 8.], [0., 0., -1.], 3.6),
        ([5., 6.05, 4.], [-1., 0., 0.], 3.85),
        ([-4., 6.05, 4.], [1., 0., 0.], 4.35),
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
    let nearer = "<model name='nearer'><static>true</static><pose>0.75 5.75 6 0 0 0</pose><link name='body'><collision name='shape'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    let mixed = load(
        &FIXTURE.replace("</world>", &format!("{nearer}</world>")),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    distance(&mixed, [0.75, 5.75, 8.], [0., 0., -1.], 1.75);
    let remaining = ray(
        &mixed,
        [0.75, 5.75, 8.],
        [0., 0., -1.],
        Some(body(&mixed, "default::nearer::body")),
    )
    .unwrap();
    assert!((remaining - 3.5).abs() < 1e-11);
    assert_eq!(
        ray(
            &mixed,
            [0.75, 5.75, 8.],
            [0., 0., -1.],
            Some(body(&mixed, "default::posed::body"))
        ),
        Some(1.75)
    );
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
fn sphere_local_rotation_tangency_identity_and_sdformat_defaults_keep_exact_geometry() {
    let xml = dynamic("1 -2 0.5 0 0 0", INERTIA);
    let posed = load(&xml, PhysicsEngine::Rapier).unwrap();
    distance(&posed, [1.5, -2., 9.], [0., 0., -1.], 3.5);
    distance(&posed, [1.3, -2., 9.], [0., 0., -1.], 3.1);
    distance(&posed, [2.5, -2., 7.5], [-0.6, 0., -0.8], 2.);
    let origin = [2.5, -2., 7.5];
    let direction = [-0.6, 0., -0.8];
    let boundary = ray(&posed, origin, direction, None).unwrap();
    let backend = posed.world().resource::<PhysicsBackend>();
    assert_eq!(
        backend.cast_ray(origin, direction, boundary - 1e-6, None),
        Ok(None)
    );
    assert_eq!(
        backend.cast_ray(origin, direction, boundary, None),
        Ok(Some(boundary))
    );
    assert_eq!(
        backend.cast_ray(origin, direction, boundary + 1e-6, None),
        Ok(Some(boundary))
    );
    assert_eq!(ray(&posed, [1., -2., 5.5], [0., 0., 1.], None), Some(0.));
    assert_eq!(ray(&posed, [1., -2., 6.], [0., 0., 1.], None), Some(0.));
    assert_eq!(ray(&posed, [1.501, -2., 9.], [0., 0., -1.], None), None);
    let rotated_xml = xml.replace("1 -2 0.5 0 0 0", "1 -2 0.5 0 0 1.5707963267948966");
    assert_ne!(rotated_xml, xml);
    let rotated = load(&rotated_xml, PhysicsEngine::Rapier).unwrap();
    near(
        pose(&rotated, "default::posed::body::shape").translation,
        [1., -2., 5.5],
    );
    same_rotation(
        pose(&rotated, "default::posed::body::shape").rotation,
        [
            0.,
            0.,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        ],
    );
    assert_eq!(
        state(&rotated, "default::posed::body"),
        state(&posed, "default::posed::body")
    );
    let quaternion_xml = xml.replace(
        "<pose>1 -2 0.5 0 0 0</pose>",
        "<pose rotation_format='quat_xyzw'>1 -2 0.5 0 0 0 2</pose>",
    );
    assert_ne!(quaternion_xml, xml);
    let normalized = load(&quaternion_xml, PhysicsEngine::Rapier).unwrap();
    for origin in [
        [1., -2., 9.],
        [1.3, -2., 9.],
        [1.49, -2., 9.],
        [1.501, -2., 9.],
    ] {
        assert_eq!(
            ray(&rotated, origin, [0., 0., -1.], None),
            ray(&posed, origin, [0., 0., -1.], None)
        );
        assert_eq!(
            ray(&normalized, origin, [0., 0., -1.], None),
            ray(&posed, origin, [0., 0., -1.], None)
        );
    }
    let centered_xml = dynamic("0 0 0 0 0 0", INERTIA);
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(load(&centered_xml, engine).is_ok());
    }
    let centered = load(&centered_xml, PhysicsEngine::Rapier).unwrap();
    distance(&centered, [0., 0., 9.], [0., 0., -1.], 3.5);
    assert_eq!(ray(&centered, [1., -2., 9.], [0., 0., -1.], None), None);
    // Pinned SDFormat maps missing and nonpositive sphere radius to 1.
    let unit_xml = xml.replace("<radius>0.5</radius>", "<radius>1</radius>");
    let unit = load(&unit_xml, PhysicsEngine::Rapier).unwrap();
    for geometry in [
        "<sphere/>",
        "<sphere><radius>0</radius></sphere>",
        "<sphere><radius>-1</radius></sphere>",
    ] {
        let defaults_xml = xml.replace(GEOMETRY, geometry);
        assert_ne!(defaults_xml, xml);
        let defaults = load(&defaults_xml, PhysicsEngine::Rapier).unwrap();
        distance(&defaults, [1., -2., 9.], [0., 0., -1.], 2.5);
        for x in [1., 1.3, 1.9, 2.01] {
            assert_eq!(
                ray(&defaults, [x, -2., 9.], [0., 0., -1.], None),
                ray(&unit, [x, -2., 9.], [0., 0., -1.], None)
            );
        }
    }
}

#[test]
fn posed_sphere_ordinary_prepared_and_loaded_resets_preserve_geometry_and_ids() {
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
fn displaced_sphere_uses_explicit_link_centered_mass_and_world_impulses() {
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
fn sphere_resolved_defaults_and_rotated_centered_auto_inertia_keep_link_mass_frame() {
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
    // Density1000, radius.5: M=500*pi/3 and I=2*M*r²/5=50*pi/3.
    // A sphere's isotropic tensor permits plain rotation-only upstream auto.
    let mass = (500. / 3.) * std::f64::consts::PI;
    let moment = (50. / 3.) * std::f64::consts::PI;
    for collision_pose in ["0 0 0 0 0 0", "0 0 0 0.2 -0.3 0.4"] {
        let mut resolved = load(
            &dynamic(collision_pose, "<inertial auto='true'/>"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let handle = body(&resolved, "default::posed::body");
        resolved
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .apply_impulse(handle, [mass, 0., 0.], [moment, 2. * moment, 3. * moment])
            .unwrap();
        near(
            state(&resolved, "default::posed::body").velocity,
            [1., 0., 0.],
        );
        near(
            state(&resolved, "default::posed::body").angular_velocity,
            [1., 2., 3.],
        );
    }
    let automatic = dynamic("1 0 0 0 0 0.7", "<inertial auto='true'/>");
    m104_mass_response(
        load(&automatic, PhysicsEngine::Rapier).unwrap(),
        &automatic,
        "default::posed::body",
    );
    for collision_pose in [
        "<pose rotation_format='quat_xyzw'>0 0 0 0 0 0 1</pose>",
        "<pose relative_to='body'>0 0 0 0 0 0</pose>",
    ] {
        let xml = dynamic("0 0 0 0 0 0", "<inertial auto='true'/>")
            .replace("<pose>0 0 0 0 0 0</pose>", collision_pose);
        assert!(load(&xml, PhysicsEngine::Rapier).is_err());
    }
}

#[test]
fn displaced_sphere_supports_a_ball_that_the_centered_control_misses() {
    let models = "<model name='ground'><static>true</static><link name='body'><collision name='shape'><pose>1.5 0 1 0 0 0.7</pose><geometry><sphere><radius>0.4</radius></sphere></geometry></collision></link></model><model name='ball'><pose>1.5 0 3 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link></model>";
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
    assert!((support - 2.6).abs() < 1e-11);
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
    assert!((pose(&posed, "default::ball::body").translation[2] - 1.6).abs() < 0.01);
    assert!(pose(&centered, "default::ball::body").translation[2] < 0.);
    near(pose(&posed, "default::ground::body").translation, [0.; 3]);
    reset_loaded_world(&mut posed, |_| ()).unwrap();
    posed.step_exact(1000).unwrap();
    assert!((pose(&posed, "default::ball::body").translation[2] - 1.6).abs() < 0.01);
}

#[test]
fn posed_sphere_links_preserve_same_model_exclusion_and_distinct_model_contacts() {
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
fn posed_sphere_refusals_keep_tpe_compounds_com_and_nonprimitive_pose_domains_closed() {
    let xml = dynamic("1 0 0 0 0 0.2", INERTIA);
    m107_bounds_response(
        load(&xml, PhysicsEngine::Tpe).unwrap(),
        &xml,
        "default::posed::body",
    );
    // M84 replaces only the old two-primitive row, at the same link origin.
    let compound_xml = xml.replace("</link>", "<collision name='extra'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>");
    let compound = load(&compound_xml, PhysicsEngine::Rapier).unwrap();
    distance(&compound, [1., 0., 9.], [0., 0., -1.], 3.5);
    distance(&compound, [0., 0., 9.], [0., 0., -1.], 3.5);
    near(
        pose(&compound, "default::posed::body").translation,
        [0., 0., 5.],
    );
    near(
        pose(&compound, "default::posed::body::shape").translation,
        [1., 0., 5.],
    );
    near(
        pose(&compound, "default::posed::body::extra").translation,
        [0., 0., 5.],
    );
    for (name, expected) in [
        ("default::posed::body", [0., 0., 0., 1.]),
        (
            "default::posed::body::shape",
            [0., 0., (0.1f64).sin(), (0.1f64).cos()],
        ),
        ("default::posed::body::extra", [0., 0., 0., 1.]),
    ] {
        for (actual, expected) in pose(&compound, name).rotation.into_iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-11);
        }
    }
    distance(&compound, [1.3, 0., 9.], [0., 0., -1.], 3.6);
    distance(&compound, [-0.3, 0., 9.], [0., 0., -1.], 3.6);
    let offset_com = xml.replace("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>");
    m104_mass_response(
        load(&offset_com, PhysicsEngine::Rapier).unwrap(),
        &offset_com,
        "default::posed::body",
    );
    for (source, feature) in [
        (
            xml.replace(
                "</collision>",
                "<surface><friction><ode><mu>0.3</mu></ode></friction></surface></collision>",
            ),
            "collision surface parameters",
        ),
        (
            xml.replace(
                GEOMETRY,
                "<plane><normal>0 0 1</normal><size>2 2</size></plane>",
            ),
            "offset collision pose",
        ),
    ] {
        assert_ne!(source, xml);
        assert!(
            matches!(load(&source, PhysicsEngine::Rapier), Err(LoadError::Unsupported { feature:actual, .. }) if actual==feature)
        );
    }
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
        xml.replace("<radius>0.5</radius>", "<radius>1e-308</radius>"),
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
            "invalid sphere case {case}"
        );
    }
}

#[test]
fn posed_sphere_coexists_with_an_unchanged_centered_velocity_controller_only() {
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
    // M83 changes only this Rapier posed-owner disposition; TPE remains closed.
    assert!(matches!(
        load(&controlled_pose, PhysicsEngine::Tpe),
        Err(LoadError::Unsupported {
            feature: "VelocityControl centered primitive",
            ..
        })
    ));
    let mut controlled = load(&controlled_pose, PhysicsEngine::Rapier).unwrap();
    let owner = id(&controlled, "default::posed");
    assert!(velocity_control_binding_matches(&controlled, owner));
    near(
        pose(&controlled, "default::posed::body").translation,
        [0., 0., 5.],
    );
    near(
        pose(&controlled, "default::posed::body::shape").translation,
        [1., 0., 5.],
    );
    distance(&controlled, [1., 0., 9.], [0., 0., -1.], 3.5);
    // Radius .5, transverse .3 and chord root .4 give a side hit at1.6.
    distance(&controlled, [1.3, 2., 5.], [0., -1., 0.], 1.6);
    set_velocity_control_command(&mut controlled, owner, [0., 2., 0.], [0.; 3]).unwrap();
    controlled.step_exact(3).unwrap();
    assert!(velocity_control_binding_matches(&controlled, owner));
    near(
        state(&controlled, "default::posed::body").position,
        [0.001, 0.004, 5.],
    );
    near(
        state(&controlled, "default::posed::body").velocity,
        [0., 2., 0.],
    );
    near(
        pose(&controlled, "default::posed::body::shape").translation,
        [1.001, 0.004, 5.],
    );
    distance(&controlled, [1.001, 0.004, 9.], [0., 0., -1.], 3.5);
    distance(&controlled, [1.301, 2.004, 5.], [0., -1., 0.], 1.6);
}

// M104: the exact formerly refused XML must produce a real mass-frame response.
fn m104_mass_response(mut sim: Simulation, text: &str, link_name: &str) {
    sim.initialize_schedules().unwrap();
    let dom = sdformat::load_str(text, "retained-resolved-com", &sdformat::NoResources).unwrap();
    let link = dom
        .worlds
        .iter()
        .flat_map(|w| &w.models)
        .flat_map(|m| &m.links)
        .find(|l| l.scoped_name == link_name)
        .unwrap();
    let mass = link.inertial.mass;
    assert!(mass.is_finite() && mass > 0.);
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == link_name)
        })
        .unwrap();
    let public = entity.get::<gz_sim_core::EntityId>().unwrap().get();
    let native = entity.get::<gz_sim_scene::BodyHandle>().unwrap().id();
    assert_eq!(
        sim.world()
            .resource::<gz_physics::PhysicsBackend>()
            .is_dynamic(native),
        Some(true)
    );
    let before = sim
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .state(native)
        .unwrap();
    gz_sim_scene::queue_link_world_wrench(&mut sim, public, [mass, 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    let after = sim
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .state(native)
        .unwrap();
    assert!(
        (after.velocity[0] - before.velocity[0] - sim.step_duration().as_secs_f64()).abs() < 1e-11
    );
    assert!(
        after
            .position
            .into_iter()
            .chain(after.velocity)
            .chain(after.angular_velocity)
            .all(f64::is_finite)
    );
    for (a, b) in after
        .angular_velocity
        .into_iter()
        .zip(before.angular_velocity)
    {
        assert!((a - b).abs() < 1e-11);
    }
}

// M107: an owned probe contacts the actual posed primitive envelope, then leaves it.
fn m107_bounds_response(mut sim: Simulation, text: &str, link_name: &str) {
    use glam::{DQuat, DVec3};
    use gz_physics_tpe::{Contacts, ModelBounds, Motion};
    sim.initialize_schedules().unwrap();
    let dom = sdformat::load_str(text, "retained-tpe-bounds", &sdformat::NoResources).unwrap();
    let link = dom
        .worlds
        .iter()
        .flat_map(|w| &w.models)
        .flat_map(|m| &m.links)
        .find(|l| l.scoped_name == link_name)
        .unwrap();
    let collision = &link.collisions[0];
    let collision_name = format!("{}::{}", link_name, collision.name);
    let pose = *sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == collision_name)
        })
        .unwrap()
        .get::<gz_sim_scene::WorldPose>()
        .unwrap();
    let q = DQuat::from_array(pose.rotation);
    let x = q * DVec3::X;
    let y = q * DVec3::Y;
    let z = q * DVec3::Z;
    let extent = match &collision.geometry {
        sdformat::Geometry::Box { size } => {
            (x.x.abs() * size[0] + y.x.abs() * size[1] + z.x.abs() * size[2]) * 0.5
        }
        sdformat::Geometry::Sphere { radius } => *radius,
        sdformat::Geometry::Cylinder { radius, length } => {
            radius * (x.x * x.x + y.x * y.x).sqrt() + 0.5 * length * z.x.abs()
        }
        sdformat::Geometry::Capsule { radius, length } => radius + 0.5 * length * z.x.abs(),
        _ => panic!("retained primitive"),
    };
    assert!(extent.is_finite() && extent > 0.01);
    let center = DVec3::from_array(pose.translation) + DVec3::X * (extent - 0.01);
    let subject_name = link_name.rsplit_once("::").unwrap().0;
    let subject = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == subject_name)
        })
        .unwrap()
        .id();
    let state = gz_physics::BodyState {
        position: center.to_array(),
        velocity: [0.; 3],
        orientation: [0., 0., 0., 1.],
        angular_velocity: [0.; 3],
    };
    let probe = sim
        .world_mut()
        .spawn((
            Motion::new_fixed(state).unwrap(),
            ModelBounds::new([-0.001; 3], [0.001; 3], true, u16::MAX).unwrap(),
        ))
        .id();
    sim.step_exact(1).unwrap();
    let points = sim
        .world()
        .resource::<Contacts>()
        .as_slice()
        .iter()
        .filter(|c| c.models.contains(&subject) && c.models.contains(&probe))
        .map(|c| c.point)
        .collect::<Vec<_>>();
    assert_eq!(points.len(), 1);
    for (a, b) in points[0].into_iter().zip(center.to_array()) {
        assert!((a - b).abs() < 1e-11);
    }
    let far = gz_physics::BodyState {
        position: (center + DVec3::splat(1000.)).to_array(),
        ..state
    };
    sim.world_mut()
        .entity_mut(probe)
        .insert(Motion::new_fixed(far).unwrap());
    sim.step_exact(1).unwrap();
    assert!(
        !sim.world()
            .resource::<Contacts>()
            .as_slice()
            .iter()
            .any(|c| c.models.contains(&subject) && c.models.contains(&probe))
    );
    sim.world_mut().despawn(probe);
}
