// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pin-readiness regressions; these do not add a Scene URDF entry point.
use gz_sim_core::Simulation;
use gz_sim_scene::{
    ImuSample, JointDefinition, LoadError, Name, PhysicsEngine, Velocity, WorldPhysics, WorldPose,
    load_world_with_sdf_step, reset_loaded_world,
};
use sdformat::{ErrorCode, NoResources, Resource, ResourceResolver};
use std::time::Duration;

fn legacy(version: &str, field: &str) -> String {
    include_str!("fixtures/headless_imu.sdf")
        .replace("version=\"1.12\"", &format!("version=\"{version}\""))
        .replace("<gravity>0 0 -9.81</gravity>", "")
        .replace(
            "<physics name=\"exact\" type=\"ode\">",
            &format!("<physics name=\"exact\" type=\"ode\">{field}"),
        )
}

fn assert_loaded_gravity(sim: &Simulation, engine: PhysicsEngine) {
    let link = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::ball::body")
        })
        .unwrap();
    let velocity = link.get::<Velocity>().unwrap().linear;
    let expected = if engine == PhysicsEngine::Tpe {
        [0.; 3]
    } else {
        [0.01, -0.02, -0.03]
    };
    for (actual, expected) in velocity.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "{engine:?}: {actual} != {expected}"
        );
    }
    let position = link.get::<WorldPose>().unwrap().translation;
    if engine == PhysicsEngine::Tpe {
        assert_eq!(position, [0., 0., 5.]);
    } else {
        assert!(position[0] > 0. && position[1] < 0. && position[2] < 5.);
    }
    let sample = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<ImuSample>())
        .unwrap();
    assert_eq!(sample.time, Duration::from_millis(10));
    let reading = sample.reading.as_ref().unwrap();
    let force = if engine == PhysicsEngine::Tpe {
        [-1., 2., 3.]
    } else {
        [0.; 3]
    };
    for (actual, expected) in reading.specific_force.into_iter().zip(force) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "{engine:?}: {actual} != {expected}"
        );
    }
}

#[test]
fn legacy_gravity_reaches_physics_imu_and_loaded_reset_on_all_backends() {
    for version in ["1.4", "1.5"] {
        let xml = legacy(version, "<gravity>1 -2 -3</gravity>");
        let parsed = sdformat::load_str(&xml, "legacy-gravity.sdf", &NoResources).unwrap();
        assert_eq!(parsed.version, version);
        assert_eq!(parsed.worlds[0].gravity, [1., -2., -3.]);
        for engine in [
            PhysicsEngine::Tpe,
            PhysicsEngine::Rapier,
            PhysicsEngine::Dart,
            PhysicsEngine::Mujoco,
        ] {
            let mut sim = load_world_with_sdf_step(
                &xml,
                "legacy-gravity.sdf",
                "default",
                &NoResources,
                engine,
            )
            .unwrap();
            let physics = sim.world().resource::<WorldPhysics>().clone();
            assert!(
                physics
                    .selected()
                    .unwrap()
                    .xml
                    .as_ref()
                    .unwrap()
                    .contains("<gravity>1 -2 -3</gravity>")
            );
            assert_eq!(
                physics.selected().unwrap().source.as_ref().unwrap().uri,
                "legacy-gravity.sdf"
            );
            assert_eq!(sim.step_duration(), Duration::from_millis(1));
            sim.step_exact(10).unwrap();
            assert_loaded_gravity(&sim, engine);
            sim.world_mut()
                .resource_mut::<WorldPhysics>()
                .profiles
                .clear();
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.elapsed(), Duration::ZERO);
            assert_eq!(
                sim.world().resource::<WorldPhysics>().profiles,
                physics.profiles
            );
            sim.step_exact(10).unwrap();
            assert_loaded_gravity(&sim, engine);
        }
    }
}

#[test]
fn legacy_nondefault_magnetic_field_reaches_existing_consumer_rejection() {
    for version in ["1.4", "1.5"] {
        let xml = legacy(version, "<magnetic_field>1 2 3</magnetic_field>");
        assert_eq!(
            sdformat::load_str(&xml, "magnetic.sdf", &NoResources)
                .unwrap()
                .worlds[0]
                .magnetic_field,
            [1., 2., 3.]
        );
        for engine in [
            PhysicsEngine::Tpe,
            PhysicsEngine::Rapier,
            PhysicsEngine::Dart,
            PhysicsEngine::Mujoco,
        ] {
            let result =
                load_world_with_sdf_step(&xml, "magnetic.sdf", "default", &NoResources, engine);
            assert!(matches!(
                result,
                Err(LoadError::Unsupported {
                    feature: "world magnetic field",
                    ..
                })
            ));
        }
    }
}

#[test]
fn legacy_default_magnetic_metadata_keeps_original_profile_snapshot() {
    for version in ["1.4", "1.5"] {
        let field = "<magnetic_field>5.5645e-6 22.8758e-6 -42.3884e-6</magnetic_field>";
        let xml = legacy(version, field);
        let mut sim = load_world_with_sdf_step(
            &xml,
            "default-magnetic.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Tpe,
        )
        .unwrap();
        let physics = sim.world().resource::<WorldPhysics>().clone();
        assert!(
            physics
                .selected()
                .unwrap()
                .xml
                .as_ref()
                .unwrap()
                .contains(field)
        );
        sim.step_exact(2).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            sim.world().resource::<WorldPhysics>().profiles,
            physics.profiles
        );
        sim.step_exact(1).unwrap();
        assert_eq!(sim.elapsed(), Duration::from_millis(1));
    }
}

#[test]
fn legacy_strict_boundaries_still_fail_before_scene_creation() {
    for version in ["1.4", "1.5"] {
        for (physics, world, code) in [
            (
                "<gravity>1 2 3</gravity><gravity>4 5 6</gravity>",
                "",
                ErrorCode::InvalidValue,
            ),
            (
                "<gravity>1 2 3</gravity>",
                "<gravity>4 5 6</gravity>",
                ErrorCode::Unsupported,
            ),
            (
                "<gravity unknown='x'>1 2 3</gravity>",
                "",
                ErrorCode::Unsupported,
            ),
            ("<gravity>nan 2 3</gravity>", "", ErrorCode::InvalidValue),
        ] {
            let xml = format!(
                "<sdf version='{version}'><world name='default'>{world}<physics type='ode'>{physics}</physics></world></sdf>"
            );
            assert!(
                matches!(load_world_with_sdf_step(&xml, "strict.sdf", "default", &NoResources, PhysicsEngine::Tpe), Err(LoadError::Sdf(d)) if d.code == code && d.source.uri == "strict.sdf")
            );
        }
        let xml = format!(
            "<sdf version='{version}'><world name='default'><physics name='a' type='ode'/><physics name='b' type='ode'><gravity>1 2 3</gravity></physics></world></sdf>"
        );
        assert!(
            matches!(load_world_with_sdf_step(&xml, "later.sdf", "default", &NoResources, PhysicsEngine::Tpe), Err(LoadError::Sdf(d)) if d.code == ErrorCode::Unsupported)
        );
    }
    for version in ["1.6", "1.12"] {
        let xml = format!(
            "<sdf version='{version}'><world name='default'><physics type='ode'><gravity>1 2 3</gravity></physics></world></sdf>"
        );
        assert!(
            matches!(load_world_with_sdf_step(&xml, "modern.sdf", "default", &NoResources, PhysicsEngine::Tpe), Err(LoadError::Sdf(d)) if d.code == ErrorCode::Unsupported)
        );
    }
}

struct UrdfResource;
impl ResourceResolver for UrdfResource {
    fn resolve(&self, uri: &str, parent: &str) -> Result<Resource, String> {
        assert_eq!((uri, parent), ("model://urdf", "included.sdf"));
        Ok(Resource {
            canonical_uri: "fixture:robot.urdf".into(),
            xml: include_str!("fixtures/sdf486_continuous_original.urdf").into(),
        })
    }
}

#[test]
fn urdf_import_capability_does_not_activate_scene_or_include_conversion() {
    let xml = include_str!("fixtures/sdf486_continuous_original.urdf");
    let imported = sdformat::load_urdf_str(xml, "robot.urdf").unwrap();
    assert_eq!(imported.sdf.models[0].links.len(), 2);
    assert_eq!(imported.sdf.models[0].joints.len(), 1);
    assert_eq!(imported.original_xml(), xml);
    assert_eq!(
        imported.converted_source().uri,
        "generated:urdf-to-sdf:robot.urdf"
    );
    assert!(
        matches!(load_world_with_sdf_step(xml, "robot.urdf", "default", &NoResources, PhysicsEngine::Tpe), Err(LoadError::Sdf(d)) if d.code == ErrorCode::Xml)
    );
    let included = "<sdf version='1.12'><world name='default'><include><uri>model://urdf</uri></include></world></sdf>";
    assert!(
        matches!(load_world_with_sdf_step(included, "included.sdf", "default", &UrdfResource, PhysicsEngine::Tpe), Err(LoadError::Sdf(d)) if d.code == ErrorCode::Xml && d.source.uri == "fixture:robot.urdf")
    );
}

fn articulated_urdf(extra: &str) -> String {
    let link = |name: &str| {
        format!(
            "<link name='{name}'><inertial><mass value='1'/><inertia ixx='0.02' ixy='0' ixz='0' iyy='0.02' iyz='0' izz='0.02'/></inertial><collision><geometry><sphere radius='0.2'/></geometry></collision></link>"
        )
    };
    format!(
        "<robot name='robot'>{}{}<joint name='joint' type='continuous'><parent link='parent'/><child link='child'/><origin xyz='0 0 2'/>{extra}</joint></robot>",
        link("child"),
        link("parent")
    )
}
fn explicit_sdf_projection(urdf: &str) -> String {
    let imported = sdformat::load_urdf_str(urdf, "explicit.urdf").unwrap();
    let model = imported
        .converted_xml()
        .strip_prefix("<sdf version=\"1.12\">")
        .unwrap()
        .strip_suffix("</sdf>")
        .unwrap();
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>{model}</world></sdf>"
    )
}

#[test]
fn explicit_converted_sdf_retains_existing_joint_metadata_guards() {
    // Explicit diagnostic bridge only; Scene itself still has no URDF entry point.
    let xml = explicit_sdf_projection(&articulated_urdf(""));
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Dart] {
        let mut sim =
            load_world_with_sdf_step(&xml, "explicit.sdf", "default", &NoResources, engine)
                .unwrap();
        let joint = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<JointDefinition>())
            .unwrap();
        assert_eq!(joint.0.parent, "parent");
        assert_eq!(joint.0.child, "child");
        assert_eq!(joint.0.kind, sdformat::JointKind::Revolute);
        assert_eq!(joint.0.pose.relative_to, "parent");
        assert_eq!(joint.0.axis.as_ref().unwrap().xyz, [1., 0., 0.]);
        for (name, expected) in [("parent", [0., 0., 0.]), ("child", [0., 0., 2.])] {
            let body = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|n| n.0 == format!("default::robot::{name}"))
                })
                .unwrap();
            assert_eq!(body.get::<WorldPose>().unwrap().translation, expected);
        }
        sim.step_exact(2).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
    }
    assert!(matches!(
        load_world_with_sdf_step(
            &xml,
            "explicit.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "TPE joints",
            ..
        })
    ));
    assert!(matches!(
        load_world_with_sdf_step(
            &xml,
            "explicit.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Mujoco
        ),
        Err(LoadError::Physics(gz_physics::PhysicsError::Unsupported {
            engine: "mujoco",
            capability: "body-to-body revolute joint"
        }))
    ));
    for (extra, expected) in [
        ("<limit effort='0' velocity='0'/>", "joint limits"),
        ("<dynamics damping='0.1' friction='0.2'/>", "joint dynamics"),
    ] {
        let xml = explicit_sdf_projection(&articulated_urdf(extra));
        for engine in [
            PhysicsEngine::Rapier,
            PhysicsEngine::Dart,
            PhysicsEngine::Mujoco,
        ] {
            assert!(
                matches!(load_world_with_sdf_step(&xml, "explicit.sdf", "default", &NoResources, engine), Err(LoadError::Unsupported { feature, .. }) if feature == expected)
            );
        }
    }
}
