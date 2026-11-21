// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    InputError, Name, PhysicsEngine, Velocity, VelocityControlCommandError,
    VelocityControlConfiguration, WorldPose, initial_velocity_control_configurations,
    initial_velocity_control_configurations_with_identity, load_world_with_sdf_step, model_state,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_model_state, set_velocity_control_command, velocity_control_binding_matches,
    velocity_control_binding_matches_configuration,
};
use sdformat::NoResources;
const XML: &str = include_str!("fixtures/m79_velocity_control.sdf");
fn load_xml(xml: &str, engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step(xml, "velocity.sdf", "default", &NoResources, engine).unwrap()
}
fn load(engine: PhysicsEngine) -> Simulation {
    load_xml(XML, engine)
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn pose(sim: &Simulation, id: u64) -> WorldPose {
    *sim.world()
        .get::<WorldPose>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
}
fn velocity(sim: &Simulation, id: u64) -> Velocity {
    *sim.world()
        .get::<Velocity>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
}
fn near(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        assert!(a.is_finite() && (a - b).abs() < 1e-11, "{a} != {b}");
    }
}
fn rotate([x, y, z, w]: [f64; 4], [a, b, c]: [f64; 3]) -> [f64; 3] {
    // Explicit independent rotation matrix, not the production quaternion helper.
    [
        (1. - 2. * (y * y + z * z)) * a + 2. * (x * y - z * w) * b + 2. * (x * z + y * w) * c,
        2. * (x * y + z * w) * a + (1. - 2. * (x * x + z * z)) * b + 2. * (y * z - x * w) * c,
        2. * (x * z - y * w) * a + 2. * (y * z + x * w) * b + (1. - 2. * (x * x + y * y)) * c,
    ]
}

#[test]
fn velocity_control_initial_six_axes_use_model_frame_and_real_rapier_and_tpe_steps() {
    let xml = XML
        .replace(
            "<pose>0 0 10 0 0 0</pose>",
            "<pose>0 0 10 0 0 1.5707963267948966</pose>",
        )
        .replace(
            "<initial_linear>1 0 0</initial_linear>",
            "<initial_linear>1 2 3</initial_linear>",
        )
        .replace(
            "<initial_angular>0 0 0</initial_angular>",
            "<initial_angular>0.1 0.2 0.3</initial_angular>",
        );
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load_xml(&xml, engine);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        near(velocity(&sim, link).linear, [0.; 3]);
        let before = pose(&sim, model);
        sim.tick().unwrap();
        assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
        near(pose(&sim, model).translation, before.translation);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [-2., 1., 3.]);
        near(velocity(&sim, link).angular, [-0.2, 0.1, 0.3]);
        near(pose(&sim, model).translation, [-0.002, 0.001, 10.003]);
        for _ in 0..20 {
            let before = pose(&sim, model);
            let expected = rotate(before.rotation, [1., 2., 3.]);
            let angular = rotate(before.rotation, [0.1, 0.2, 0.3]);
            sim.step_exact(1).unwrap();
            near(velocity(&sim, link).linear, expected);
            near(velocity(&sim, link).angular, angular);
            near(
                pose(&sim, model).translation,
                std::array::from_fn(|i| before.translation[i] + expected[i] * 0.001),
            );
        }
    }
}

#[test]
fn velocity_control_paused_pending_first_resume_old_target_and_next_step_latch_persist() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(engine);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        set_velocity_control_command(&mut sim, model, [9.; 3], [0.; 3]).unwrap();
        set_velocity_control_command(&mut sim, model, [0., 2., 0.], [0.; 3]).unwrap();
        for _ in 0..3 {
            sim.tick().unwrap();
            near(velocity(&sim, link).linear, [0.; 3]);
        }
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [1., 0., 0.]);
        near(pose(&sim, model).translation, [0.001, 0., 10.]);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [0., 2., 0.]);
        near(pose(&sim, model).translation, [0.001, 0.002, 10.]);
        sim.step_exact(3).unwrap();
        near(pose(&sim, model).translation, [0.001, 0.008, 10.]);
        set_velocity_control_command(&mut sim, model, [0.; 3], [0.; 3]).unwrap();
        sim.tick().unwrap();
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [0., 2., 0.]);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [0.; 3]);
        let stopped = pose(&sim, model);
        sim.step_exact(3).unwrap();
        near(pose(&sim, model).translation, stopped.translation);
    }
}

#[test]
fn velocity_control_latest_six_axis_command_rotates_without_pose_or_initial_state_edits() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let xml = XML.replace(
            "<pose>0 0 10 0 0 0</pose>",
            "<pose>0 0 10 0 0 1.5707963267948966</pose>",
        );
        let mut sim = load_xml(&xml, engine);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        let initial = pose(&sim, model);
        set_velocity_control_command(&mut sim, model, [1., -2., 3.], [0.2, 0.3, -0.1]).unwrap();
        let after = pose(&sim, model);
        assert_eq!(
            (after.translation, after.rotation),
            (initial.translation, initial.rotation)
        );
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [0., 1., 0.]);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [2., 1., 3.]);
        near(velocity(&sim, link).angular, [-0.3, 0.2, -0.1]);
        sim.reset().unwrap();
        near(pose(&sim, model).translation, initial.translation);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [0., 1., 0.]);
        near(velocity(&sim, link).angular, [0.; 3]);
    }
}

#[test]
fn velocity_control_invalid_or_unrepresentable_commands_preserve_prior_pending_target() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let xml = XML.replace(
            "<pose>0 0 10 0 0 0</pose>",
            "<pose>0 0 10 0 0 0.7853981633974483</pose>",
        );
        let mut sim = load_xml(&xml, engine);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        set_velocity_control_command(&mut sim, model, [0., 2., 0.], [0.; 3]).unwrap();
        for (linear, angular) in [
            ([f64::NAN; 3], [0.; 3]),
            ([0.; 3], [f64::INFINITY; 3]),
            ([f64::MAX, f64::MAX, 0.], [0.; 3]),
        ] {
            assert!(matches!(
                set_velocity_control_command(&mut sim, model, linear, angular),
                Err(VelocityControlCommandError::InvalidCommand)
            ));
        }
        assert!(matches!(
            set_velocity_control_command(&mut sim, u64::MAX, [0.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        assert!(matches!(
            set_velocity_control_command(&mut sim, link, [0.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        sim.step_exact(2).unwrap();
        near(velocity(&sim, link).linear, [-2f64.sqrt(), 2f64.sqrt(), 0.]);
    }
}

#[test]
fn velocity_control_loaded_reset_removal_and_survivor_keep_original_metadata_and_public_ids() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(engine);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        let other = id(&sim, "default::survivor::body");
        let config = initial_velocity_control_configurations(&sim)
            .unwrap()
            .next()
            .unwrap()
            .1
            .clone();
        assert_eq!(config.model_name, "default::rig");
        assert_eq!(config.topic, "/model/rig/cmd_vel");
        assert_eq!(config.link, link);
        assert!(velocity_control_binding_matches_configuration(
            &sim, model, &config
        ));
        queue_link_world_velocity(&mut sim, other, [0., 1., 0.], [0.; 3]).unwrap();
        set_velocity_control_command(&mut sim, model, [8.; 3], [0.; 3]).unwrap();
        let epoch = sim.epoch();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(sim.epoch(), epoch + 1);
        assert_eq!(id(&sim, "default::rig"), model);
        assert_eq!(id(&sim, "default::rig::body"), link);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [1., 0., 0.]);
        near(velocity(&sim, other).linear, [0.; 3]);
        let before = pose(&sim, other);
        remove_model(&mut sim, model).unwrap();
        assert!(!velocity_control_binding_matches(&sim, model));
        assert!(matches!(
            set_velocity_control_command(&mut sim, model, [1.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        assert_eq!(
            initial_velocity_control_configurations(&sim)
                .unwrap()
                .count(),
            1
        );
        let row = initial_velocity_control_configurations_with_identity(&sim)
            .unwrap()
            .next()
            .unwrap();
        assert_eq!(row.id, model);
        assert_eq!(row.configuration, &config);
        assert_eq!(row.name, "default::rig");
        assert!(row.parent.is_some());
        sim.step_exact(3).unwrap();
        near(pose(&sim, other).translation, before.translation);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(velocity_control_binding_matches_configuration(
            &sim, model, &config
        ));
        assert_eq!(id(&sim, "default::rig::body"), link);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [1., 0., 0.]);
    }
}

#[test]
fn velocity_control_exclusive_velocity_and_model_mutation_leave_uncontrolled_inputs_and_wrenches_real()
 {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(engine);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        let other = id(&sim, "default::survivor::body");
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [9.; 3], [0.; 3]),
            Err(InputError::Unsupported(
                "VelocityControl owns link velocity"
            ))
        ));
        queue_link_world_velocity(&mut sim, other, [0., 3., 0.], [0.; 3]).unwrap();
        if engine == PhysicsEngine::Rapier {
            let state = model_state(&sim, model).unwrap();
            assert!(set_model_state(&mut sim, model, state).is_err());
            queue_link_world_wrench(&mut sim, link, [2., 0., 0.], [0.; 3]).unwrap();
        }
        sim.step_exact(1).unwrap();
        near(velocity(&sim, other).linear, [0., 3., 0.]);
        near(
            velocity(&sim, link).linear,
            if engine == PhysicsEngine::Rapier {
                [1.001, 0., 0.]
            } else {
                [1., 0., 0.]
            },
        );
        sim.step_exact(1).unwrap();
        near(velocity(&sim, link).linear, [1., 0., 0.]);
    }
    let xml = XML.replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.8</gravity>");
    let mut sim = load_xml(&xml, PhysicsEngine::Rapier);
    let model = id(&sim, "default::rig");
    let link = id(&sim, "default::rig::body");
    set_velocity_control_command(&mut sim, model, [0.; 3], [0.; 3]).unwrap();
    sim.step_exact(2).unwrap();
    near(velocity(&sim, link).linear, [0., 0., -0.0098]);
    assert!(pose(&sim, model).translation[2] < 10.);
}

#[test]
fn velocity_control_metadata_drift_cannot_release_exclusive_ownership_or_advance_physics() {
    for fault in 0..3 {
        let mut sim = load(PhysicsEngine::Rapier);
        let model = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        let entity = entity_by_id(sim.world(), model).unwrap();
        let original = sim
            .world()
            .get::<VelocityControlConfiguration>(entity)
            .unwrap()
            .clone();
        match fault {
            0 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<VelocityControlConfiguration>();
            }
            1 => {
                let mut changed = original.clone();
                changed.topic.push_str("/foreign");
                sim.world_mut().entity_mut(entity).insert(changed);
            }
            2 => {
                let mut changed = original.clone();
                changed.link = id(&sim, "default::survivor::body");
                sim.world_mut().entity_mut(entity).insert(changed);
            }
            _ => unreachable!(),
        }
        assert!(!velocity_control_binding_matches(&sim, model));
        assert!(set_velocity_control_command(&mut sim, model, [0.; 3], [0.; 3]).is_err());
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [9.; 3], [0.; 3]),
            Err(InputError::Unsupported(_))
        ));
        let before = pose(&sim, model);
        assert!(sim.step_exact(1).is_err());
        let after = pose(&sim, model);
        assert_eq!(
            (after.translation, after.rotation),
            (before.translation, before.rotation)
        );
    }
}

#[test]
fn velocity_control_default_custom_topics_four_centered_shapes_and_closed_refusal_domain() {
    let marker = "<plugin filename=\"gz-sim-velocity-control-system\" name=\"gz::sim::systems::VelocityControl\">";
    for shape in [
        "<sphere><radius>0.5</radius></sphere>",
        "<box><size>1 2 3</size></box>",
        "<cylinder><radius>0.5</radius><length>1</length></cylinder>",
        "<capsule><radius>0.5</radius><length>1</length></capsule>",
    ] {
        let xml = XML
            .replace("<sphere><radius>0.5</radius></sphere>", shape)
            .replace(marker, &format!("{marker}<topic>/custom/velocity</topic>"));
        for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
            let sim = load_xml(&xml, engine);
            let c = initial_velocity_control_configurations(&sim)
                .unwrap()
                .next()
                .unwrap()
                .1;
            assert_eq!(c.topic, "/custom/velocity");
        }
    }
    // M83 admits the rotation-only sphere on Rapier; TPE keeps its old refusal.
    let rotated_sphere = XML.replacen(
        "<collision name=\"shape\">",
        "<collision name=\"shape\"><pose>0 0 0 0 0 1</pose>",
        1,
    );
    assert_ne!(rotated_sphere, XML);
    assert!(
        load_world_with_sdf_step(
            &rotated_sphere,
            "bad",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
    let mut posed = load_xml(&rotated_sphere, PhysicsEngine::Rapier);
    let mut centered = load(PhysicsEngine::Rapier);
    let owner = id(&posed, "default::rig");
    let control_owner = id(&centered, "default::rig");
    assert!(velocity_control_binding_matches(&posed, owner));
    let local_rotation = pose(&posed, id(&posed, "default::rig::body::shape")).rotation;
    for (actual, expected) in local_rotation
        .into_iter()
        .zip([0., 0., 0.5_f64.sin(), 0.5_f64.cos()])
    {
        assert!((actual - expected).abs() < 1e-11);
    }
    for sim in [&posed, &centered] {
        let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
        for (x, expected) in [(0., 1.5), (0.3, 1.6)] {
            let actual = backend
                .cast_ray([x, 0., 12.], [0., 0., -1.], 3., None)
                .unwrap()
                .unwrap();
            assert!((actual - expected).abs() < 1e-11);
        }
    }
    set_velocity_control_command(&mut posed, owner, [0., 2., 0.], [0.; 3]).unwrap();
    set_velocity_control_command(&mut centered, control_owner, [0., 2., 0.], [0.; 3]).unwrap();
    posed.step_exact(3).unwrap();
    centered.step_exact(3).unwrap();
    // Native-compatible old target on first step, then the latched new target.
    near(pose(&posed, owner).translation, [0.001, 0.004, 10.]);
    near(
        pose(&posed, owner).translation,
        pose(&centered, control_owner).translation,
    );
    assert!(velocity_control_binding_matches(&posed, owner));
    near(
        pose(&posed, id(&posed, "default::rig::body")).translation,
        [0.001, 0.004, 10.],
    );
    near(
        pose(&posed, id(&posed, "default::rig::body::shape")).translation,
        [0.001, 0.004, 10.],
    );
    let actual = posed
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .cast_ray([0.301, 0.004, 12.], [0., 0., -1.], 3., None)
        .unwrap()
        .unwrap();
    assert!((actual - 1.6).abs() < 1e-11);
    let mut invalid = vec![
        XML.replace(
            "<link name=\"body\">",
            "<link name=\"body\"><pose>1 0 0 0 0 0</pose>",
        ),
        XML.replace(
            "<model name=\"rig\">",
            "<model name=\"rig\"><static>true</static>",
        ),
        XML.replacen("</link>", "</link><link name='second'/>", 1),
        XML.replacen("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>", 1),
        XML.replacen(
            "<sphere><radius>0.5</radius></sphere>",
            "<plane><normal>0 0 1</normal><size>2 2</size></plane>",
            1,
        ),
    ];
    let start = XML.find("<model name=\"rig\">").unwrap();
    let end = XML.find("</model>").unwrap() + "</model>".len();
    invalid.push(format!(
        "{}<model name='outer'>{}</model>{}",
        &XML[..start],
        &XML[start..end],
        &XML[end..]
    ));
    invalid.push(XML.replacen(
        "</model>",
        "<joint name='held' type='fixed'><parent>world</parent><child>body</child></joint></model>",
        1,
    ));
    invalid.push(XML.replacen("</plugin>","</plugin><plugin filename='gz-sim-joint-controller-system' name='gz::sim::systems::JointController'><joint_name>missing</joint_name></plugin>",1));
    for fields in [
        "<link_name>body</link_name>",
        "<initial_linear>1 2</initial_linear>",
        "<initial_linear>nan 0 0</initial_linear>",
        "<topic>bad space</topic>",
        "<topic>/a/*</topic>",
        "<extra/>",
    ] {
        invalid.push(XML.replace(marker, &format!("{marker}{fields}")));
    }
    let plugin = XML
        .split(marker)
        .nth(1)
        .unwrap()
        .split("</plugin>")
        .next()
        .unwrap();
    invalid.push(XML.replace("</plugin>", &format!("</plugin>{marker}{plugin}</plugin>")));
    invalid.push(XML.replace(
        "<model name=\"survivor\">",
        &format!("<model name=\"survivor\">{marker}</plugin>"),
    ));
    for xml in invalid {
        for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
            assert!(
                load_world_with_sdf_step(&xml, "bad", "default", &NoResources, engine).is_err()
            );
        }
    }
}
