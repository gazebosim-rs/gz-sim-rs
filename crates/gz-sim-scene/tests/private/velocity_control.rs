// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const XML: &str = include_str!("../fixtures/m79_velocity_control.sdf");
fn load(engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step(XML, "m79", "default", &sdformat::NoResources, engine).unwrap()
}
fn controller(sim: &Simulation) -> (Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<Controller>())
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn model() -> Model {
    sdformat::load_str(XML, "config", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .remove(0)
}
fn payload(model: &mut Model, text: &str) {
    let p = &mut model.plugins[0];
    p.xml = format!(
        "<plugin filename='{}' name='{}'>{text}</plugin>",
        p.filename, p.name
    );
}

#[test]
fn velocity_control_absence_and_last_removal_retire_all_owned_callbacks_and_restore_once() {
    let begin = XML.find("      <plugin").unwrap();
    let end = XML.find("</plugin>").unwrap() + "</plugin>".len();
    let plain = format!("{}{}", &XML[..begin], &XML[end..]);
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut empty =
            load_world_with_sdf_step(&plain, "plain", "default", &sdformat::NoResources, engine)
                .unwrap();
        let mut active = load(engine);
        empty.initialize_schedules().unwrap();
        active.initialize_schedules().unwrap();
        assert!(!empty.world().contains_resource::<Installed>());
        assert_eq!(
            initial_velocity_control_configurations(&empty)
                .unwrap()
                .count(),
            0
        );
        let baseline = empty.scheduled_system_counts();
        assert_eq!(
            active.scheduled_system_counts(),
            (baseline.0 + 2, baseline.1 + 1)
        );
        install(&mut active);
        assert_eq!(
            active.scheduled_system_counts(),
            (baseline.0 + 2, baseline.1 + 1)
        );
        let (_, id) = controller(&active);
        remove_model(&mut active, id).unwrap();
        assert!(!active.world().contains_resource::<Installed>());
        assert_eq!(active.scheduled_system_counts(), baseline);
        reset_loaded_world(&mut active, |_| ()).unwrap();
        active.initialize_schedules().unwrap();
        assert!(active.world().contains_resource::<Installed>());
        assert_eq!(
            active.scheduled_system_counts(),
            (baseline.0 + 2, baseline.1 + 1)
        );
    }
}

#[test]
fn velocity_control_closed_xml_parser_preserves_defaults_and_refuses_ambiguous_lexical_fields() {
    let mut m = model();
    payload(&mut m, "<!-- ordinary comment -->");
    let p = parse(&m).unwrap().unwrap();
    assert_eq!(p.topic, "/model/rig/cmd_vel");
    assert_eq!(p.linear, [0.; 3]);
    assert_eq!(p.angular, [0.; 3]);
    payload(
        &mut m,
        "<initial_linear>1e100 -2 0</initial_linear><initial_angular>0 0.3 -0.4</initial_angular><topic>/chosen/path</topic>",
    );
    let p = parse(&m).unwrap().unwrap();
    assert_eq!(p.linear, [1e100, -2., 0.]);
    assert_eq!(p.angular, [0., 0.3, -0.4]);
    assert_eq!(p.topic, "/chosen/path");
    let original = m.plugins[0].clone();
    for text in [
        "<initial_linear/>",
        "<initial_linear>1 2 3 4</initial_linear>",
        "<initial_linear>1 wrong 3</initial_linear>",
        "<initial_angular>0 inf 0</initial_angular>",
        "<initial_linear>0 0 0</initial_linear><initial_linear>1 2 3</initial_linear>",
        "<initial_angular attr='x'>0 0 0</initial_angular>",
        "<initial_angular><x>0</x></initial_angular>",
        "<initial_angular><![CDATA[0 0 0]]></initial_angular>",
        "<topic>/a<!--split-->/b</topic>",
        "plain",
        "<link_name>body</link_name>",
        "<topic>/</topic>",
        "<topic>/bad//path</topic>",
        "<topic>/bad:=path</topic>",
        "<topic>/bad~path</topic>",
    ] {
        payload(&mut m, text);
        assert!(parse(&m).is_err(), "{text}");
    }
    for xml in [
        "<not-plugin/>",
        "<plugin xmlns='urn:x'/>",
        "<plugin filename='wrong' name='wrong'/>",
        "<plugin extra='x'/>",
        "<plugin>",
        "garbage",
    ] {
        m.plugins[0] = original.clone();
        m.plugins[0].xml = xml.to_owned();
        assert!(parse(&m).is_err());
    }
    m.plugins[0] = original.clone();
    m.plugins[0].xml = " ".repeat(65537);
    assert!(parse(&m).is_err());
    m.plugins.clear();
    assert!(parse(&m).unwrap().is_none());
}

#[test]
fn velocity_control_checked_rotation_refuses_bad_quaternions_and_coupled_overflow_before_engine_mutation()
 {
    assert!(world_vectors([0.; 4], [1.; 3], [0.; 3]).is_none());
    assert!(world_vectors([f64::NAN; 4], [1.; 3], [0.; 3]).is_none());
    let half = std::f64::consts::FRAC_PI_4 / 2.;
    let rotation = [0., 0., half.sin(), half.cos()];
    assert!(world_vectors(rotation, [f64::MAX, f64::MAX, 0.], [0.; 3]).is_none());
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let xml = XML.replace(
            "<pose>0 0 10 0 0 0</pose>",
            "<pose>0 0 10 0 0 0.7853981633974483</pose>",
        );
        let mut sim =
            load_world_with_sdf_step(&xml, "overflow", "default", &sdformat::NoResources, engine)
                .unwrap();
        let (e, id) = controller(&sim);
        let before = *sim.world().get::<WorldPose>(e).unwrap();
        sim.world_mut().get_mut::<Controller>(e).unwrap().committed =
            ([f64::MAX, f64::MAX, 0.], [0.; 3]);
        assert!(sim.step_exact(1).is_err());
        let after = *sim.world().get::<WorldPose>(e).unwrap();
        assert_eq!(
            (after.translation, after.rotation),
            (before.translation, before.rotation)
        );
        assert!(velocity_control_binding_matches(&sim, id));
    }
}

#[test]
fn velocity_control_live_private_or_public_binding_drift_never_redirects_owned_body_or_releases_velocity()
 {
    for fault in 0..9 {
        let mut sim = load(PhysicsEngine::Rapier);
        let (e, id) = controller(&sim);
        let link = sim.world().get::<Controller>(e).unwrap().configuration.link;
        let le = gz_sim_core::entity_by_id(sim.world(), link).unwrap();
        match fault {
            0 => {
                sim.world_mut().entity_mut(e).remove::<Controller>();
            }
            1 => {
                assert!(sim.world().get::<LinkCenterOfMass>(le).is_none());
                sim.world_mut()
                    .entity_mut(le)
                    .insert(LinkCenterOfMass([0.1, 0., 0.]));
            }
            2 => {
                sim.world_mut().get_mut::<Controller>(e).unwrap().owner = u64::MAX;
            }
            3 => {
                sim.world_mut().get_mut::<WorldPose>(e).unwrap().translation[0] = 1.;
            }
            4 => {
                let other = sim
                    .world()
                    .iter_entities()
                    .find(|e| {
                        e.get::<Name>()
                            .is_some_and(|n| n.0 == "default::survivor::body")
                    })
                    .unwrap()
                    .id();
                let foreign = sim
                    .world_mut()
                    .entity_mut(other)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(le).insert(foreign);
            }
            5 => {
                let other = sim
                    .world_mut()
                    .spawn((EntityKind::World, Name("foreign".to_owned())))
                    .id();
                gz_sim_core::assign_id(sim.world_mut(), other).unwrap();
                sim.world_mut().entity_mut(e).insert(ChildOf(other));
            }
            6 => {
                sim.world_mut()
                    .insert_resource(tpe::TpeWorld(std::sync::Arc::new(())));
            }
            7 => {
                let mut changed = sim
                    .world()
                    .get::<Controller>(e)
                    .unwrap()
                    .configuration
                    .clone();
                changed.initial_linear = [2.; 3];
                sim.world_mut()
                    .get_mut::<Controller>(e)
                    .unwrap()
                    .configuration = changed.clone();
                sim.world_mut().entity_mut(e).insert(changed);
            }
            8 => {
                sim.world_mut().get_mut::<WorldPose>(e).unwrap().rotation = [f64::NAN; 4];
            }
            _ => unreachable!(),
        }
        assert!(!velocity_control_binding_matches(&sim, id), "fault {fault}");
        assert!(
            set_velocity_control_command(&mut sim, id, [0.; 3], [0.; 3]).is_err(),
            "fault {fault}"
        );
        assert!(
            queue_link_world_velocity(&mut sim, link, [0.; 3], [0.; 3]).is_err(),
            "fault {fault}"
        );
        assert!(sim.step_exact(1).is_err(), "fault {fault}");
    }
    let mut sim = load(PhysicsEngine::Rapier);
    let (_, id) = controller(&sim);
    sim.world_mut().remove_resource::<PhysicsBackend>();
    assert!(matches!(
        set_velocity_control_command(&mut sim, id, [0.; 3], [0.; 3]),
        Err(VelocityControlCommandError::ForeignPhysics)
    ));
    let unloaded = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(initial_velocity_control_configurations(&unloaded).is_err());
}

#[test]
fn velocity_control_configuration_limits_nested_jointed_mesh_and_initial_overflow_without_new_scope()
 {
    let mut m = model();
    let plan = parse(&m).unwrap().unwrap();
    let mut sim = load(PhysicsEngine::Rapier);
    sim.initialize_schedules().unwrap();
    let sdf = sdformat::load_str(XML, "configuration", &sdformat::NoResources).unwrap();
    let pose = sdf.pose_of(&m.scoped_name, "default").unwrap();
    // M105 admits another eligible metadata plan below the eight-owner limit.
    // This preflight neither publishes a duplicate model nor changes this owner.
    let (existing_entity, existing_owner) = controller(&sim);
    let existing = sim
        .world()
        .get::<Controller>(existing_entity)
        .unwrap()
        .clone();
    let existing_link =
        gz_sim_core::entity_by_id(sim.world(), existing.configuration.link).unwrap();
    let existing_body = sim.world().get::<BodyHandle>(existing_link).unwrap().id();
    let before_entities = sim
        .world()
        .iter_entities()
        .map(|e| (e.id(), e.get::<EntityId>().map(EntityId::get)))
        .collect::<Vec<_>>();
    let before_native = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(existing_body);
    let before_pose = *sim.world().get::<WorldPose>(existing_entity).unwrap();
    let before_link_pose = *sim.world().get::<WorldPose>(existing_link).unwrap();
    let before_clock = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
    plan.validate(
        &sdf,
        &m,
        &sim,
        Some(sim.world().resource::<PhysicsBackend>().engine()),
        (true, false),
        pose,
    )
    .unwrap();
    assert_eq!(
        sim.world()
            .iter_entities()
            .map(|e| (e.id(), e.get::<EntityId>().map(EntityId::get)))
            .collect::<Vec<_>>(),
        before_entities
    );
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(existing_body),
        before_native
    );
    let current_pose = sim.world().get::<WorldPose>(existing_entity).unwrap();
    assert_eq!(current_pose.translation, before_pose.translation);
    assert_eq!(current_pose.rotation, before_pose.rotation);
    let current_link_pose = sim.world().get::<WorldPose>(existing_link).unwrap();
    assert_eq!(current_link_pose.translation, before_link_pose.translation);
    assert_eq!(current_link_pose.rotation, before_link_pose.rotation);
    let current = sim.world().get::<Controller>(existing_entity).unwrap();
    assert_eq!(current.owner, existing.owner);
    assert_eq!(current.configuration, existing.configuration);
    assert_eq!(current.committed, existing.committed);
    assert_eq!(current.pending, existing.pending);
    assert_eq!(current.center, existing.center);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
        before_clock
    );
    assert!(velocity_control_binding_matches(&sim, existing_owner));
    // A separate measured witness uses the original fixture's initial command;
    // it does not install the metadata plan a second time.
    let mut moving = load(PhysicsEngine::Rapier);
    let (moving_entity, moving_owner) = controller(&moving);
    let link_id = moving
        .world()
        .get::<Controller>(moving_entity)
        .unwrap()
        .configuration
        .link;
    let moving_link = gz_sim_core::entity_by_id(moving.world(), link_id).unwrap();
    let moving_body = moving.world().get::<BodyHandle>(moving_link).unwrap().id();
    let initial_model = *moving.world().get::<WorldPose>(moving_entity).unwrap();
    let initial_link = *moving.world().get::<WorldPose>(moving_link).unwrap();
    moving.step_exact(1).unwrap();
    let native = moving
        .world()
        .resource::<PhysicsBackend>()
        .state(moving_body)
        .unwrap();
    for (axis, speed) in [1.0, 0.0, 0.0].into_iter().enumerate() {
        for actual in [
            native.velocity[axis],
            moving.world().get::<Velocity>(moving_link).unwrap().linear[axis],
        ] {
            assert!((actual - speed).abs() < 1e-11);
        }
        assert!(native.angular_velocity[axis].abs() < 1e-11);
        assert!(moving.world().get::<Velocity>(moving_link).unwrap().angular[axis].abs() < 1e-11);
        assert!(
            (native.position[axis] - initial_link.translation[axis] - speed * 0.001).abs() < 1e-11
        );
        assert!(
            (moving
                .world()
                .get::<WorldPose>(moving_entity)
                .unwrap()
                .translation[axis]
                - initial_model.translation[axis]
                - speed * 0.001)
                .abs()
                < 1e-11
        );
        assert!(
            (moving
                .world()
                .get::<WorldPose>(moving_link)
                .unwrap()
                .translation[axis]
                - initial_link.translation[axis]
                - speed * 0.001)
                .abs()
                < 1e-11
        );
    }
    assert_eq!(native.orientation, initial_link.rotation);
    assert_eq!(
        moving
            .world()
            .get::<WorldPose>(moving_entity)
            .unwrap()
            .rotation,
        initial_model.rotation
    );
    assert_eq!(
        moving
            .world()
            .get::<WorldPose>(moving_link)
            .unwrap()
            .rotation,
        initial_link.rotation
    );
    assert!(velocity_control_binding_matches(&moving, moving_owner));
    let empty = load_world_with_sdf_step(
        "<sdf version='1.12'><world name='default'/></sdf>",
        "empty",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert!(
        plan.validate(
            &sdf,
            &m,
            &empty,
            Some(empty.world().resource::<PhysicsBackend>().engine()),
            (false, false),
            pose
        )
        .is_err()
    );
    assert!(
        plan.validate(
            &sdf,
            &m,
            &empty,
            Some(empty.world().resource::<PhysicsBackend>().engine()),
            (true, true),
            pose
        )
        .is_err()
    );
    let mesh_xml = XML.replacen(
        "<sphere><radius>0.5</radius></sphere>",
        "<mesh><uri>not-loaded.obj</uri></mesh>",
        1,
    );
    m.links[0].collisions[0].geometry =
        sdformat::load_str(&mesh_xml, "metadata-only", &sdformat::NoResources)
            .unwrap()
            .worlds
            .remove(0)
            .models
            .remove(0)
            .links
            .remove(0)
            .collisions
            .remove(0)
            .geometry;
    assert!(
        plan.validate(
            &sdf,
            &m,
            &empty,
            Some(empty.world().resource::<PhysicsBackend>().engine()),
            (true, false),
            pose
        )
        .is_err()
    );
    let mut m = model();
    payload(
        &mut m,
        "<initial_linear>1.7976931348623157e308 1.7976931348623157e308 0</initial_linear>",
    );
    let plan = parse(&m).unwrap().unwrap();
    let mut rotated = pose;
    let a = std::f64::consts::FRAC_PI_4 / 2.;
    rotated.rotation = [0., 0., a.sin(), a.cos()];
    assert!(
        plan.validate(
            &sdf,
            &m,
            &empty,
            Some(empty.world().resource::<PhysicsBackend>().engine()),
            (true, false),
            rotated
        )
        .is_err()
    );
    // Public XML is checked eagerly; exercise the parsed-DOM consumer boundary
    // with a valid first child and an independently valid second child first.
    let mut compound_sdf =
        sdformat::load_str(XML, "late-child-pose", &sdformat::NoResources).unwrap();
    let compound_model = &mut compound_sdf.worlds[0].models[0];
    assert_eq!(compound_model.links[0].collisions.len(), 1);
    let mut late = compound_model.links[0].collisions[0].clone();
    late.name = "late_valid_child".to_owned();
    late.pose.transform.translation = [2., 0., 0.];
    compound_model.links[0].collisions.push(late);
    assert_eq!(compound_model.links[0].collisions.len(), 2);
    let compound_plan = parse(compound_model).unwrap().unwrap();
    let compound_pose = compound_sdf
        .pose_of(&compound_sdf.worlds[0].models[0].scoped_name, "default")
        .unwrap();
    let identities = |sim: &Simulation| {
        sim.world()
            .iter_entities()
            .map(|e| (e.id(), e.get::<EntityId>().map(EntityId::get)))
            .collect::<Vec<_>>()
    };
    let empty_ids = identities(&empty);
    assert!(
        !empty
            .world()
            .iter_entities()
            .any(|e| e.contains::<Controller>() || e.contains::<BodyHandle>())
    );
    let mut preserved = load(PhysicsEngine::Rapier);
    let (preserved_entity, preserved_owner) = controller(&preserved);
    set_velocity_control_command(
        &mut preserved,
        preserved_owner,
        [2., 3., 4.],
        [0.1, 0.2, 0.3],
    )
    .unwrap();
    let preserved_ids = identities(&preserved);
    let saved_controller = preserved
        .world()
        .get::<Controller>(preserved_entity)
        .unwrap()
        .clone();
    assert!(saved_controller.pending.is_some());
    let preserved_link =
        gz_sim_core::entity_by_id(preserved.world(), saved_controller.configuration.link).unwrap();
    let preserved_body = preserved
        .world()
        .get::<BodyHandle>(preserved_link)
        .unwrap()
        .id();
    let saved_native = preserved
        .world()
        .resource::<PhysicsBackend>()
        .state(preserved_body);
    let saved_time = (preserved.elapsed(), preserved.epoch());
    compound_plan
        .validate(
            &compound_sdf,
            &compound_sdf.worlds[0].models[0],
            &empty,
            Some(empty.world().resource::<PhysicsBackend>().engine()),
            (true, false),
            compound_pose,
        )
        .unwrap();
    assert_eq!(identities(&empty), empty_ids);
    // Alter only the last child's reference, after proving both children valid.
    let missing = "missing_m85_last_collision_frame";
    let first_pose = compound_sdf.worlds[0].models[0].links[0].collisions[0]
        .pose
        .clone();
    compound_sdf.worlds[0].models[0].links[0].collisions[1]
        .pose
        .relative_to = missing.to_owned();
    assert!(matches!(
        compound_plan.validate(
            &compound_sdf, &compound_sdf.worlds[0].models[0], &empty,
            Some(empty.world().resource::<PhysicsBackend>().engine()), (true, false), compound_pose,
        ),
        Err(LoadError::Sdf(diagnostic)) if diagnostic.code == sdformat::ErrorCode::UnknownReference
            && diagnostic.message.contains(missing)
    ));
    assert_eq!(
        compound_sdf.worlds[0].models[0].links[0].collisions[0].pose,
        first_pose
    );
    assert_eq!(identities(&empty), empty_ids);
    assert!(
        !empty
            .world()
            .iter_entities()
            .any(|e| e.contains::<Controller>() || e.contains::<BodyHandle>())
    );
    assert_eq!(empty.elapsed(), Duration::ZERO);
    assert_eq!(identities(&preserved), preserved_ids);
    assert_eq!(
        preserved
            .world()
            .resource::<PhysicsBackend>()
            .state(preserved_body),
        saved_native
    );
    let current_controller = preserved
        .world()
        .get::<Controller>(preserved_entity)
        .unwrap();
    assert_eq!(current_controller.owner, saved_controller.owner);
    assert_eq!(
        current_controller.configuration,
        saved_controller.configuration
    );
    assert_eq!(current_controller.committed, saved_controller.committed);
    assert_eq!(current_controller.pending, saved_controller.pending);
    assert_eq!(current_controller.center, saved_controller.center);
    assert_eq!((preserved.elapsed(), preserved.epoch()), saved_time);
    assert!(velocity_control_binding_matches(
        &preserved,
        preserved_owner
    ));
}

#[derive(Resource, Default)]
struct PreTargets(Vec<([f64; 3], [f64; 3])>);
fn capture_pre_target(world: &mut World) {
    let target = world
        .iter_entities()
        .find_map(|e| e.get::<Controller>().map(|c| c.committed))
        .unwrap();
    world.resource_mut::<PreTargets>().0.push(target);
}
fn pre_target(sim: &mut Simulation, advancing: bool) -> ([f64; 3], [f64; 3]) {
    let before = sim.world().resource::<PreTargets>().0.len();
    if advancing {
        sim.step_exact(1).unwrap();
    } else {
        sim.tick().unwrap();
    }
    let rows = &sim.world().resource::<PreTargets>().0;
    assert_eq!(rows.len(), before + 1);
    rows[before]
}

#[test]
fn velocity_control_model_targets_match_complete_fresh_original_native_component_reference() {
    // Root must supply actual independently audited native DATA at qualification;
    // this fixture is deliberately absent during Source preparation.
    let native: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/m79_velocity_control_native.json")).unwrap();
    assert_eq!(native["schema_version"], 1);
    assert_eq!(
        native["state"],
        "M79_ORIGINAL_VELOCITY_CONTROL_COMPONENT_REFERENCE"
    );
    let modes = native["modes"].as_array().unwrap();
    assert_eq!(modes.len(), 2);
    let names = [
        "initial_pre",
        "a_first_pre",
        "a_effect",
        "paused_held",
        "first_resume_pre",
        "zero_effect",
        "partial_before_post",
        "partial_effect",
        "persistence",
    ];
    for (row, mode) in modes.iter().zip(["defaults", "explicit"]) {
        assert_eq!(row["mode"], mode);
        let (linear, angular) = if mode == "defaults" {
            ([0.; 3], [0.; 3])
        } else {
            ([0.25, -0.5, 0.75], [-0.125, 0.375, -0.625])
        };
        assert_eq!(row["initial_linear"], serde_json::json!(linear));
        assert_eq!(row["initial_angular"], serde_json::json!(angular));
        let fields = if mode == "defaults" {
            String::new()
        } else {
            "<initial_linear>0.25 -0.5 0.75</initial_linear><initial_angular>-0.125 0.375 -0.625</initial_angular><topic>/explicit/velocity</topic>".to_owned()
        };
        let begin = XML.find("<initial_linear>").unwrap();
        let end = XML.find("</plugin>").unwrap();
        let xml = format!("{}{}{}", &XML[..begin], fields, &XML[end..]);
        let observations = row["observations"].as_array().unwrap();
        assert_eq!(observations.len(), names.len());
        for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
            let mut sim = load_world_with_sdf_step(
                &xml,
                "native-model-targets",
                "default",
                &sdformat::NoResources,
                engine,
            )
            .unwrap();
            let (_, id) = controller(&sim);
            sim.world_mut().init_resource::<PreTargets>();
            sim.add_systems(Phase::PreStep, capture_pre_target.after(apply));
            let mut actual = vec![pre_target(&mut sim, true)];
            set_velocity_control_command(&mut sim, id, [1.25, -2.5, 3.75], [-0.5, 1., -1.5])
                .unwrap();
            actual.push(pre_target(&mut sim, true));
            actual.push(pre_target(&mut sim, true));
            set_velocity_control_command(&mut sim, id, [0.; 3], [0.; 3]).unwrap();
            actual.push(pre_target(&mut sim, false));
            actual.push(pre_target(&mut sim, true));
            actual.push(pre_target(&mut sim, true));
            set_velocity_control_command(&mut sim, id, [0.; 3], [0.25, -0.75, 1.5]).unwrap();
            actual.push(pre_target(&mut sim, true));
            actual.push(pre_target(&mut sim, true));
            actual.push(pre_target(&mut sim, true));
            assert_eq!(actual.len(), observations.len());
            for ((target, observation), name) in actual.into_iter().zip(observations).zip(names) {
                assert_eq!(observation["milestone"], name);
                assert_eq!(
                    observation["linear"],
                    serde_json::json!(target.0),
                    "{mode}/{name}"
                );
                assert_eq!(
                    observation["angular"],
                    serde_json::json!(target.1),
                    "{mode}/{name}"
                );
            }
        }
    }
}
