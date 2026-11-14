// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const ORIGINAL: &str = include_str!("../../../../examples/m9_joint_controllers.sdf");
const POSITION_PLUGIN: &str = "<plugin filename=\"gz-sim-joint-position-controller-system\"";
fn velocity_only(model: &mut String) {
    let start = model.find(POSITION_PLUGIN).unwrap();
    let finish = start + model[start..].find("</plugin>").unwrap() + "</plugin>".len();
    model.replace_range(start..finish, "");
    // An unconfigured base joint retains the accepted infinite effort/velocity
    // domain; removing only its plugin would leave an unsupported graph.
    *model = model.replacen(
        "<limit><effort>3</effort><velocity>4</velocity></limit>",
        "",
        1,
    );
}
fn source(count: usize, single: bool) -> String {
    let begin = ORIGINAL.find("  <model name=\"arm\"").unwrap();
    let end = ORIGINAL.rfind("</model>").unwrap() + "</model>".len();
    let mut result = ORIGINAL[..begin].to_owned();
    for index in 0..count {
        let mut model = ORIGINAL[begin..end]
            .replacen("name=\"arm\"", &format!("name=\"arm{index}\""), 1)
            .replacen(
                "<pose>0 0 1 0 0 0</pose>",
                &format!("<pose>0 {} 1 0 0 0</pose>", index * 4),
                1,
            );
        if single {
            velocity_only(&mut model);
        }
        result.push_str(&model);
    }
    result.push_str(&ORIGINAL[end..]);
    result
}
fn try_load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "m58-private.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
}
fn load(count: usize, single: bool) -> Simulation {
    try_load(&source(count, single)).unwrap()
}
fn rows(sim: &Simulation) -> Vec<(Entity, u64)> {
    let mut result: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.contains::<Controller>())
        .map(|entity| (entity.id(), entity.get::<EntityId>().unwrap().get()))
        .collect();
    result.sort_by_key(|(entity, _)| sim.world().get::<Name>(*entity).unwrap().0.clone());
    result
}
fn native(sim: &Simulation, values: &[(Entity, u64)]) -> Vec<gz_physics::JointState> {
    values
        .iter()
        .map(|(entity, _)| {
            let joint = sim.world().get::<Controller>(*entity).unwrap().joint;
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap()
        })
        .collect()
}
fn targets(sim: &Simulation, values: &[(Entity, u64)]) -> Vec<u64> {
    values
        .iter()
        .map(|(entity, _)| {
            sim.world()
                .get::<Controller>(*entity)
                .unwrap()
                .target
                .to_bits()
        })
        .collect()
}

#[test]
fn zero_one_two_loaded_owners_reserve_only_the_original_four_slots() {
    let absent = Simulation::new(Duration::from_millis(1)).unwrap();
    assert_eq!(admitted_slots(absent.world()), None);
    assert!(validate_loaded_controllers(absent.world()).is_ok());
    assert!(matches!(
        initial_joint_controller_configurations_with_identity(&absent),
        Err(SceneResetError::MissingInitialState)
    ));
    for count in 0..=2 {
        for single in [false, true] {
            let sim = load(count, single);
            let values = rows(&sim);
            let expected = count * if single { 1 } else { 2 };
            assert_eq!(values.len(), expected);
            assert_eq!(admitted_slots(sim.world()), Some(expected));
            assert_eq!(
                initial_joint_controller_configurations(&sim)
                    .unwrap()
                    .count(),
                expected
            );
            assert_eq!(sim.world().contains_resource::<Installed>(), expected != 0);
            assert!(validate_loaded_controllers(sim.world()).is_ok());
            let originals: Vec<_> = initial_joint_controller_configurations_with_identity(&sim)
                .unwrap()
                .collect();
            assert_eq!(originals.len(), expected);
            for original in originals {
                assert_eq!(original.kind, EntityKind::Joint);
                let entity = gz_sim_core::entity_by_id(sim.world(), original.id).unwrap();
                let parent = sim.world().get::<ChildOf>(entity).unwrap().parent();
                assert_eq!(
                    original.parent,
                    Some(sim.world().get::<EntityId>(parent).unwrap().get())
                );
                assert_eq!(sim.world().get::<Name>(entity).unwrap().0, original.name);
            }
        }
    }
    for single in [false, true] {
        assert!(matches!(
            try_load(&source(3, single)),
            Err(LoadError::Unsupported {
                feature: "multiple joint-controller chains",
                ..
            })
        ));
    }
    let mut mixed = source(2, false);
    velocity_only(&mut mixed);
    let mixed = try_load(&mixed).unwrap();
    assert_eq!(rows(&mixed).len(), 3);
    assert_eq!(admitted_slots(mixed.world()), Some(3));
    let unsupported = source(2, true).replacen(
        "<axis><xyz>0 0 1</xyz></axis>",
        "<axis><xyz>0 0 1</xyz><limit><effort>3</effort><velocity>4</velocity></limit></axis>",
        1,
    );
    assert!(matches!(
        try_load(&unsupported),
        Err(LoadError::Unsupported {
            feature: "joint limits",
            ..
        })
    ));

    let xml = source(2, false);
    let second = xml.find("name=\"arm1\"").unwrap();
    let mut invalid = xml[..second].to_owned();
    invalid.push_str(&xml[second..].replacen("<cmd_max>2</cmd_max>", "<cmd_max>0</cmd_max>", 1));
    assert!(matches!(
        try_load(&invalid),
        Err(LoadError::Unsupported {
            feature: "joint controller limits",
            ..
        })
    ));
}

#[test]
fn either_live_owner_drift_refuses_peer_commands_and_all_actuation_before_mutation() {
    for owner in 0..2 {
        for mutation in 0..6 {
            let mut sim = load(2, false);
            let values = rows(&sim);
            let entity = values[owner * 2].0;
            let peer = values[(1 - owner) * 2].0;
            let before = native(&sim, &values);
            let pending = targets(&sim, &values);
            match mutation {
                0 => {
                    sim.world_mut()
                        .entity_mut(entity)
                        .insert(Name("default::forged::base_hinge".into()));
                }
                1 => {
                    let parent = sim.world().get::<ChildOf>(peer).unwrap().parent();
                    sim.world_mut().entity_mut(entity).insert(ChildOf(parent));
                }
                2 => {
                    sim.world_mut().entity_mut(entity).insert(EntityKind::Model);
                }
                3 => {
                    let mut configuration = sim
                        .world()
                        .get::<JointControllerConfiguration>(entity)
                        .unwrap()
                        .clone();
                    configuration.topic.push_str("/drift");
                    sim.world_mut().entity_mut(entity).insert(configuration);
                }
                4 => {
                    let handle = *sim.world().get::<JointHandle>(peer).unwrap();
                    sim.world_mut().entity_mut(entity).insert(handle);
                }
                _ => {
                    let parent = sim.world().get::<ChildOf>(entity).unwrap().parent();
                    sim.world_mut()
                        .entity_mut(parent)
                        .insert(Name("default::renamed".into()));
                }
            }
            if mutation == 3 || mutation == 4 {
                assert!(!joint_controller_binding_matches(&sim, values[owner * 2].1));
            }
            assert!(matches!(
                set_joint_controller_command(&mut sim, values[(1 - owner) * 2].1, 0.8),
                Err(JointControllerCommandError::InvalidPhysics)
            ));
            assert_eq!(targets(&sim, &values), pending);
            sim.world_mut().resource_mut::<UpdateInfo>().dt = Duration::from_millis(1);
            apply(sim.world_mut());
            assert!(
                sim.fault()
                    .unwrap()
                    .downcast_ref::<ControllerFault>()
                    .is_some()
            );
            assert_eq!(native(&sim, &values), before);
            assert_eq!(targets(&sim, &values), pending);
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
    }
    let absent = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(!joint_controller_binding_matches(&absent, 1));
    for owner in 0..2 {
        let mut sim = load(2, false);
        let values = rows(&sim);
        assert!(
            values
                .iter()
                .all(|(_, id)| joint_controller_binding_matches(&sim, *id))
        );
        assert!(!joint_controller_binding_matches(&sim, 0));
        assert!(!joint_controller_binding_matches(&sim, u64::MAX));
        let model = sim.world().get::<ChildOf>(values[0].0).unwrap().parent();
        assert!(!joint_controller_binding_matches(
            &sim,
            sim.world().get::<EntityId>(model).unwrap().get()
        ));
        let first = values[owner * 2].0;
        let second = values[owner * 2 + 1].0;
        let before = native(&sim, &values);
        let pending = targets(&sim, &values);
        let first_handle = *sim.world().get::<JointHandle>(first).unwrap();
        let second_handle = *sim.world().get::<JointHandle>(second).unwrap();
        let mut first_definition = sim.world().get::<JointDefinition>(first).unwrap().clone();
        let mut second_definition = sim.world().get::<JointDefinition>(second).unwrap().clone();
        std::mem::swap(
            &mut first_definition.0.child,
            &mut second_definition.0.child,
        );
        sim.world_mut()
            .entity_mut(first)
            .insert((second_handle, first_definition));
        sim.world_mut()
            .entity_mut(second)
            .insert((first_handle, second_definition));
        // Both public definitions still name actual native child bodies. The
        // private owner/native witness rejects this coordinated redirection.
        for entity in [first, second] {
            let handle = sim.world().get::<JointHandle>(entity).unwrap();
            let child = gz_sim_core::entity_by_id(sim.world(), handle.child()).unwrap();
            let definition = sim.world().get::<JointDefinition>(entity).unwrap();
            let expected_child_suffix = format!("::{}", definition.0.child);
            assert!(
                sim.world()
                    .get::<Name>(child)
                    .unwrap()
                    .0
                    .ends_with(expected_child_suffix.as_str())
            );
            let body = sim.world().get::<BodyHandle>(child).unwrap();
            assert_eq!(
                sim.world()
                    .resource::<PhysicsBackend>()
                    .joint_child(handle.id()),
                Some(body.id())
            );
            assert!(!joint_controller_binding_matches(
                &sim,
                sim.world().get::<EntityId>(entity).unwrap().get()
            ));
        }
        assert!(
            joint_controller_binding_matches(&sim, values[(1 - owner) * 2].1),
            "the witness is per active ID"
        );
        assert_eq!(native(&sim, &values), before);
        assert_eq!(targets(&sim, &values), pending);
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    for damage in 0..4 {
        let mut sim = load(2, false);
        let values = rows(&sim);
        let native_ids: Vec<_> = values
            .iter()
            .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
            .collect();
        let before: Vec<_> = native_ids
            .iter()
            .map(|joint| sim.world().resource::<PhysicsBackend>().joint_state(*joint))
            .collect();
        match damage {
            0 => {
                sim.world_mut()
                    .entity_mut(values[0].0)
                    .remove::<Controller>();
            }
            1 => {
                sim.world_mut()
                    .entity_mut(values[0].0)
                    .remove::<JointHandle>();
            }
            2 => {
                sim.world_mut()
                    .entity_mut(values[0].0)
                    .remove::<JointControllerConfiguration>();
            }
            _ => {
                sim.world_mut()
                    .get_mut::<Controller>(values[0].0)
                    .unwrap()
                    .owner = values[2].1;
            }
        }
        assert!(!joint_controller_binding_matches(&sim, values[0].1));
        assert!(joint_controller_binding_matches(&sim, values[2].1));
        assert_eq!(
            native_ids
                .iter()
                .map(|joint| sim.world().resource::<PhysicsBackend>().joint_state(*joint))
                .collect::<Vec<_>>(),
            before
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    let mut sim = load(2, false);
    let values = rows(&sim);
    let native_ids: Vec<_> = values
        .iter()
        .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
        .collect();
    let before = native(&sim, &values);
    let original_backend = sim.world_mut().remove_resource::<PhysicsBackend>().unwrap();
    let mut foreign = load(1, false);
    let foreign_backend = foreign
        .world_mut()
        .remove_resource::<PhysicsBackend>()
        .unwrap();
    sim.world_mut().insert_resource(foreign_backend);
    assert!(
        values
            .iter()
            .all(|(_, id)| !joint_controller_binding_matches(&sim, *id))
    );
    assert_eq!(
        native_ids
            .iter()
            .map(|joint| original_backend.joint_state(*joint).unwrap())
            .collect::<Vec<_>>(),
        before
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

#[test]
fn reset_preserves_query_order_counterfeit_overflow_for_one_and_two_real_owners() {
    for count in [1, 2] {
        let mut sim = load(count, false);
        let values = rows(&sim);
        for (_, id) in &values {
            set_joint_controller_command(&mut sim, *id, 0.4).unwrap();
        }
        let pending = targets(&sim, &values);
        let before = native(&sim, &values);
        let copy = sim.world().get::<Controller>(values[0].0).unwrap().clone();
        sim.world_mut().spawn(copy);
        let expected = {
            let world = sim.world_mut();
            let mut query = world.query::<(Entity, &mut Controller)>();
            query.iter_mut(world).nth(count * 2).unwrap().0
        };
        reset(sim.world_mut());
        let fault = sim
            .fault()
            .unwrap()
            .downcast_ref::<ControllerFault>()
            .unwrap();
        assert_eq!(fault.entity, expected);
        assert!(fault.message.contains("too many loaded controllers"));
        assert_eq!(targets(&sim, &values), pending);
        assert_eq!(native(&sim, &values), before);
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    // Combine both faults using the actual current query order: an earlier
    // missing native joint must win over the later counterfeit overflow.
    for count in [1, 2] {
        let mut sim = load(count, false);
        let values = rows(&sim);
        for (_, id) in &values {
            set_joint_controller_command(&mut sim, *id, 0.4).unwrap();
        }
        let pending = targets(&sim, &values);
        let copy = sim.world().get::<Controller>(values[0].0).unwrap().clone();
        sim.world_mut().spawn(copy);
        let ordered: Vec<_> = {
            let world = sim.world_mut();
            let mut query = world.query::<(Entity, &mut Controller)>();
            query
                .iter_mut(world)
                .map(|(entity, controller)| (entity, controller.joint))
                .collect()
        };
        assert_eq!(ordered.len(), count * 2 + 1);
        let (earlier, missing) = ordered[0];
        assert!(
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .remove_joint(missing)
        );
        let native_ids: Vec<_> = values
            .iter()
            .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
            .collect();
        let before: Vec<_> = native_ids
            .iter()
            .map(|joint| sim.world().resource::<PhysicsBackend>().joint_state(*joint))
            .collect();
        reset(sim.world_mut());
        let fault = sim
            .fault()
            .unwrap()
            .downcast_ref::<ControllerFault>()
            .unwrap();
        assert_eq!(fault.entity, earlier);
        assert!(fault.message.contains("joint identifier"));
        assert_eq!(targets(&sim, &values), pending);
        assert_eq!(
            native_ids
                .iter()
                .map(|joint| sim.world().resource::<PhysicsBackend>().joint_state(*joint))
                .collect::<Vec<_>>(),
            before
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}

#[test]
fn two_owner_reset_native_preflight_and_identity_preflight_both_preserve_every_target() {
    for missing_native in [false, true] {
        let mut sim = load(2, false);
        let values = rows(&sim);
        for (_, id) in &values {
            set_joint_controller_command(&mut sim, *id, 0.7).unwrap();
        }
        let pending = targets(&sim, &values);
        let damaged = values.last().unwrap().0;
        if missing_native {
            let joint = sim.world().get::<Controller>(damaged).unwrap().joint;
            assert!(
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove_joint(joint)
            );
        } else {
            sim.world_mut()
                .entity_mut(damaged)
                .insert(Name("default::forged::tip_hinge".into()));
        }
        reset(sim.world_mut());
        let fault = sim
            .fault()
            .unwrap()
            .downcast_ref::<ControllerFault>()
            .unwrap();
        assert_eq!(fault.entity, damaged);
        assert!(fault.message.contains(if missing_native {
            "joint identifier"
        } else {
            "invalid controller ownership"
        }));
        assert_eq!(targets(&sim, &values), pending);
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}

#[test]
fn late_nonrepresentable_peer_target_prevents_all_native_actuation() {
    let mut sim = load(2, false);
    let values = rows(&sim);
    let before = native(&sim, &values);
    sim.world_mut()
        .get_mut::<Controller>(values[0].0)
        .unwrap()
        .target = 0.6;
    sim.world_mut()
        .get_mut::<Controller>(values[3].0)
        .unwrap()
        .target = f64::NAN;
    let pending = targets(&sim, &values);
    sim.world_mut().resource_mut::<UpdateInfo>().dt = Duration::from_millis(1);
    apply(sim.world_mut());
    let fault = sim
        .fault()
        .unwrap()
        .downcast_ref::<ControllerFault>()
        .unwrap();
    assert_eq!(fault.entity, values[3].0);
    assert_eq!(native(&sim, &values), before);
    assert_eq!(targets(&sim, &values), pending);
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

#[test]
fn removing_one_real_graph_keeps_peer_motors_then_last_removal_retires_callbacks() {
    let mut sim = load(2, false);
    let original: Vec<_> = rows(&sim).iter().map(|(_, id)| *id).collect();
    let model = |sim: &Simulation, index: usize| {
        sim.world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Name>()
                    .is_some_and(|name| name.0 == format!("default::arm{index}"))
            })
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let first = model(&sim, 0);
    remove_model(&mut sim, first).unwrap();
    assert!(sim.world().contains_resource::<Installed>());
    let peer = rows(&sim);
    assert_eq!(peer.len(), 2);
    set_joint_controller_command(&mut sim, peer[1].1, -0.5).unwrap();
    sim.step_exact(1000).unwrap();
    assert!((native(&sim, &peer)[1].velocity + 0.5).abs() < 0.01);
    let last = model(&sim, 1);
    remove_model(&mut sim, last).unwrap();
    assert!(!sim.world().contains_resource::<Installed>());
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        4
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(sim.world().contains_resource::<Installed>());
    assert_eq!(
        rows(&sim).iter().map(|(_, id)| *id).collect::<Vec<_>>(),
        original
    );
    assert!(
        native(&sim, &rows(&sim))
            .iter()
            .all(|state| state.position == 0.0 && state.velocity == 0.0)
    );
}

#[test]
fn trusted_configuration_witness_separates_public_drift_from_private_binding() {
    for fault in [
        "public-only",
        "public-missing",
        "owner",
        "private-publication",
        "handle",
        "private",
        "physics",
    ] {
        let mut sim = load(2, false);
        let values = rows(&sim);
        let (entity, id) = values[0];
        let original = sim
            .world()
            .get::<Controller>(entity)
            .unwrap()
            .publication
            .clone();
        let joints: Vec<_> = values
            .iter()
            .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
            .collect();
        let before = native(&sim, &values);
        assert!(joint_controller_binding_matches(&sim, id));
        assert!(!joint_controller_binding_matches(&sim, 0));
        assert!(joint_controller_binding_matches_configuration(
            &sim, id, &original
        ));
        assert!(!joint_controller_binding_matches_configuration(
            &sim, 0, &original
        ));
        let mut changed = original.clone();
        changed.topic.push_str("/reset-drift");
        sim.world_mut().entity_mut(entity).insert(changed.clone());
        assert!(!joint_controller_binding_matches(&sim, id));
        assert!(!joint_controller_binding_matches_configuration(
            &sim, id, &changed
        ));
        match fault {
            "public-only" => {}
            "public-missing" => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<JointControllerConfiguration>();
                assert!(!joint_controller_binding_matches(&sim, id));
            }
            "owner" => sim.world_mut().get_mut::<Controller>(entity).unwrap().owner = 0,
            "private-publication" => {
                sim.world_mut()
                    .get_mut::<Controller>(entity)
                    .unwrap()
                    .publication = changed;
            }
            "handle" => {
                sim.world_mut().entity_mut(entity).remove::<JointHandle>();
            }
            "private" => {
                sim.world_mut().entity_mut(entity).remove::<Controller>();
            }
            "physics" => {}
            _ => unreachable!(),
        }
        let backend = if fault == "physics" {
            sim.world_mut().remove_resource::<PhysicsBackend>()
        } else {
            None
        };
        assert_eq!(
            joint_controller_binding_matches_configuration(&sim, id, &original),
            matches!(fault, "public-only" | "public-missing")
        );
        if let Some(backend) = backend {
            sim.world_mut().insert_resource(backend);
        }
        assert_eq!(
            joints
                .iter()
                .map(|joint| sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .joint_state(*joint)
                    .unwrap())
                .collect::<Vec<_>>(),
            before
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}
