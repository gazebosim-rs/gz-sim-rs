// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const XML: &str = include_str!("../../../../examples/m9_joint_controllers.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m9.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn controllers(sim: &Simulation) -> Vec<(Entity, u64)> {
    let mut values: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.contains::<Controller>())
        .map(|entity| (entity.id(), entity.get::<EntityId>().unwrap().get()))
        .collect();
    values.sort_by_key(|(entity, _)| {
        sim.world()
            .get::<JointControllerConfiguration>(*entity)
            .unwrap()
            .joint_name
            .clone()
    });
    values
}
fn states(sim: &Simulation) -> Vec<gz_physics::JointState> {
    controllers(sim)
        .into_iter()
        .map(|(entity, _)| {
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(sim.world().get::<Controller>(entity).unwrap().joint)
                .unwrap()
        })
        .collect()
}
#[test]
fn real_chain_tracks_continuous_positions_and_measured_velocity_without_pose_edits() {
    let mut sim = load(XML);
    let ids = controllers(&sim);
    assert_eq!(ids.len(), 2);
    let before = states(&sim);
    set_joint_controller_command(&mut sim, ids[0].1, 0.7).unwrap();
    set_joint_controller_command(&mut sim, ids[1].1, 0.5).unwrap();
    assert_eq!(
        states(&sim),
        before,
        "admission must not teleport positions or velocities"
    );
    for target in [
        0.7,
        4.0 * std::f64::consts::PI + 0.3,
        -2.0 * std::f64::consts::PI - 0.2,
    ] {
        set_joint_controller_command(&mut sim, ids[0].1, target).unwrap();
        sim.step_exact(30_000).unwrap();
        let actual = states(&sim);
        assert!(
            (actual[0].position - target).abs() < 0.01,
            "position target must converge in continuous coordinates: {actual:?}, target {target}"
        );
        assert!(
            (actual[1].velocity - 0.5).abs() < 0.01,
            "velocity controller must track measured joint velocity: {actual:?}"
        );
        let base = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Name>()
                    .is_some_and(|name| name.0 == "default::arm::base")
            })
            .unwrap();
        let forward = DQuat::from_array(base.get::<WorldPose>().unwrap().rotation) * DVec3::X;
        assert!(
            (forward.x - target.cos()).abs() < 0.02 && (forward.y - target.sin()).abs() < 0.02,
            "displayed body orientation must follow actual joint motion"
        );
        for (entity, id) in &ids {
            assert_eq!(gz_sim_core::entity_by_id(sim.world(), *id), Some(*entity));
            let sample = sim.world().get::<JointSample>(*entity).unwrap();
            assert_eq!(sample.time, sim.elapsed());
            assert_eq!(sample.epoch, 0);
        }
    }
}
#[test]
fn finite_target_limits_bad_commands_and_paused_receipt_preserve_physics() {
    let mut sim = load(XML);
    let ids = controllers(&sim);
    let before = states(&sim);
    set_joint_controller_command(&mut sim, ids[0].1, -0.7).unwrap();
    set_joint_controller_command(&mut sim, ids[1].1, f64::MAX).unwrap();
    assert_eq!(sim.world().get::<Controller>(ids[1].0).unwrap().target, 4.0);
    for (_, id) in &ids {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                set_joint_controller_command(&mut sim, *id, value),
                Err(JointControllerCommandError::InvalidCommand)
            ));
        }
    }
    assert_eq!(
        sim.world().get::<Controller>(ids[0].0).unwrap().target,
        -0.7
    );
    assert_eq!(sim.world().get::<Controller>(ids[1].0).unwrap().target, 4.0);
    assert_eq!(states(&sim), before);
    sim.set_paused(true);
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(states(&sim), before);
    assert!(matches!(
        set_joint_controller_command(&mut sim, u64::MAX, 0.0),
        Err(JointControllerCommandError::MissingController(_))
    ));
    let model = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::arm")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    assert!(matches!(
        set_joint_controller_command(&mut sim, model, 0.0),
        Err(JointControllerCommandError::MissingController(_))
    ));
}
#[test]
fn both_resets_restore_initial_targets_and_model_removal_retires_the_family() {
    let xml = XML
        .replace(
            "<initial_position>0</initial_position>",
            "<initial_position>0.25</initial_position>",
        )
        .replace(
            "<initial_velocity>0</initial_velocity>",
            "<initial_velocity>0.1</initial_velocity>",
        );
    let mut sim = load(&xml);
    let original = controllers(&sim);
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        2
    );
    set_joint_controller_command(&mut sim, original[0].1, -1.0).unwrap();
    set_joint_controller_command(&mut sim, original[1].1, -0.5).unwrap();
    sim.step_exact(1000).unwrap();
    sim.set_paused(false);
    sim.reset().unwrap();
    assert!(!sim.is_paused(), "ordinary reset preserves pause state");
    for ((entity, _), initial) in original.iter().zip([0.25, 0.1]) {
        assert_eq!(
            sim.world().get::<Controller>(*entity).unwrap().target,
            initial
        );
    }
    assert!(
        states(&sim)
            .iter()
            .all(|state| state.position == 0.0 && state.velocity == 0.0)
    );
    let model = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::arm")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let removed = remove_model(&mut sim, model).unwrap();
    assert!(original.iter().all(|(_, id)| removed.contains(id)));
    assert!(!sim.world().contains_resource::<Installed>());
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        2
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(sim.world().contains_resource::<Installed>());
    let restored = controllers(&sim);
    assert_eq!(
        restored.iter().map(|(_, id)| *id).collect::<Vec<_>>(),
        original.iter().map(|(_, id)| *id).collect::<Vec<_>>()
    );
    for ((entity, _), initial) in restored.iter().zip([0.25, 0.1]) {
        assert_eq!(
            sim.world().get::<Controller>(*entity).unwrap().target,
            initial
        );
    }
    assert!(
        states(&sim)
            .iter()
            .all(|state| state.position == 0.0 && state.velocity == 0.0)
    );
}
#[test]
fn absent_configuration_owns_no_controller_callbacks_and_other_backends_refuse() {
    let begin = XML.find("<plugin").unwrap();
    let end = XML.rfind("</plugin>").unwrap() + "</plugin>".len();
    let mut xml = format!("{}{}", &XML[..begin], &XML[end..]);
    let finite = "<limit><effort>3</effort><velocity>4</velocity></limit>";
    assert_eq!(xml.matches(finite).count(), 2);
    xml = xml.replace(finite, "<limit/>");
    let sim = load(&xml);
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(controllers(&sim).is_empty());
    assert_eq!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count(),
        0
    );
    let mut configured = load(XML);
    configured.initialize_schedules().unwrap();
    let mut absent = sim;
    absent.initialize_schedules().unwrap();
    let present_count = configured.scheduled_system_counts();
    let absent_count = absent.scheduled_system_counts();
    assert_eq!(present_count.0, absent_count.0 + 1);
    assert_eq!(present_count.1, absent_count.1 + 1);
    for engine in [
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
        PhysicsEngine::Tpe,
    ] {
        assert!(
            load_world_with_sdf_step(XML, "m9.sdf", "default", &sdformat::NoResources, engine)
                .is_err()
        );
    }
}
#[test]
fn typed_owner_and_native_binding_drift_fault_before_any_controller_actuation() {
    for damage in 0..5 {
        let mut sim = load(XML);
        let ids = controllers(&sim);
        let before = states(&sim);
        let native_ids: Vec<_> = ids
            .iter()
            .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
            .collect();
        match damage {
            0 => {
                let mut configuration = sim
                    .world()
                    .get::<JointControllerConfiguration>(ids[0].0)
                    .unwrap()
                    .clone();
                configuration.topic.push_str("/changed");
                sim.world_mut().entity_mut(ids[0].0).insert(configuration);
            }
            1 => {
                sim.world_mut().entity_mut(ids[0].0).remove::<JointHandle>();
            }
            2 => {
                sim.world_mut()
                    .get_mut::<Controller>(ids[1].0)
                    .unwrap()
                    .max_force = f64::NAN;
            }
            3 => {
                sim.world_mut()
                    .get_mut::<Controller>(ids[1].0)
                    .unwrap()
                    .max_velocity = 0.0;
            }
            _ => {
                let duplicate = sim.world().get::<Controller>(ids[0].0).unwrap().clone();
                sim.world_mut().spawn(duplicate);
            }
        }
        if damage != 4 {
            assert!(
                set_joint_controller_command(&mut sim, ids[if damage < 2 { 0 } else { 1 }].1, 0.5)
                    .is_err()
            );
        }
        assert!(sim.step_exact(1).is_err());
        let fault = sim
            .fault()
            .unwrap()
            .downcast_ref::<ControllerFault>()
            .unwrap();
        assert!(ids.iter().any(|(entity, _)| *entity == fault.entity) || damage == 4);
        assert_eq!(
            native_ids
                .into_iter()
                .map(|joint| sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .joint_state(joint)
                    .unwrap())
                .collect::<Vec<_>>(),
            before,
            "a failed chain preflight cannot advance native state"
        );
    }
}

#[test]
fn reversed_joint_declarations_keep_public_order_and_physical_chain_through_reset() {
    let root_start = XML.find("<joint name=\"base_hinge\"").unwrap();
    let root_end = root_start + XML[root_start..].find("</joint>").unwrap() + "</joint>".len();
    let tip_start = XML.find("<joint name=\"tip_hinge\"").unwrap();
    let tip_end = tip_start + XML[tip_start..].find("</joint>").unwrap() + "</joint>".len();
    assert!(root_end <= tip_start);
    let reversed = format!(
        "{}{}{}{}{}",
        &XML[..root_start],
        &XML[tip_start..tip_end],
        &XML[root_end..tip_start],
        &XML[root_start..root_end],
        &XML[tip_end..],
    );
    let ordinary = load(XML);
    let declared = controllers(&ordinary);
    let mut sim = load(&reversed);
    let ids = controllers(&sim);
    // Native construction must start at the world root even though public IDs
    // still follow the original XML declarations (tip first in this input).
    assert_eq!(ids.len(), 2);
    assert_eq!(ids[0].1, declared[1].1);
    assert_eq!(ids[1].1, declared[0].1);
    let loaded = states(&sim);
    assert!(
        loaded
            .iter()
            .all(|state| state.position == 0.0 && state.velocity == 0.0)
    );
    for (pass, target, velocity) in [(0_u64, 0.6_f64, 0.4_f64), (1, -0.6, -0.4)] {
        set_joint_controller_command(&mut sim, ids[0].1, target).unwrap();
        set_joint_controller_command(&mut sim, ids[1].1, velocity).unwrap();
        assert_eq!(
            states(&sim),
            loaded,
            "commands cannot edit native coordinates"
        );
        sim.step_exact(30_000).unwrap();
        let actual = states(&sim);
        assert!((actual[0].position - target).abs() < 0.01, "{actual:?}");
        assert!((actual[1].velocity - velocity).abs() < 0.01, "{actual:?}");
        assert!(
            actual[1].position * velocity > 0.1,
            "child joint must physically advance: {actual:?}"
        );
        let base = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Name>()
                    .is_some_and(|name| name.0 == "default::arm::base")
            })
            .unwrap();
        let forward = DQuat::from_array(base.get::<WorldPose>().unwrap().rotation) * DVec3::X;
        assert!((forward.x - target.cos()).abs() < 0.02 && (forward.y - target.sin()).abs() < 0.02);
        for (entity, id) in &ids {
            assert_eq!(gz_sim_core::entity_by_id(sim.world(), *id), Some(*entity));
            let sample = sim.world().get::<JointSample>(*entity).unwrap();
            assert_eq!(sample.epoch, pass);
            assert_eq!(sample.time, sim.elapsed());
        }
        sim.reset().unwrap();
        assert_eq!(sim.epoch(), pass + 1);
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(controllers(&sim), ids);
        assert_eq!(states(&sim), loaded);
        for (entity, id) in &ids {
            assert_eq!(gz_sim_core::entity_by_id(sim.world(), *id), Some(*entity));
            assert_eq!(sim.world().get::<Controller>(*entity).unwrap().target, 0.0);
        }
    }
}

#[path = "joint_controller_build.rs"]
mod construction;

#[test]
fn reset_preflight_refuses_corrupt_ownership_before_restoring_any_pending_target() {
    use bevy_ecs::system::RunSystemOnce;
    for extra_controller in [false, true] {
        let mut sim = load(XML);
        let ids = controllers(&sim);
        set_joint_controller_command(&mut sim, ids[0].1, 0.4).unwrap();
        set_joint_controller_command(&mut sim, ids[1].1, 1.0).unwrap();
        let native: Vec<_> = ids
            .iter()
            .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
            .collect();
        let extra = if extra_controller {
            let copy = sim.world().get::<Controller>(ids[0].0).unwrap().clone();
            Some(sim.world_mut().spawn(copy).id())
        } else {
            assert!(
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove_joint(native[1])
            );
            None
        };
        let before: Vec<_> = native
            .iter()
            .map(|joint| sim.world().resource::<PhysicsBackend>().joint_state(*joint))
            .collect();
        let targets: Vec<_> = ids
            .iter()
            .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().target)
            .collect();
        // ECS archetype transitions may place the extra entity anywhere in the
        // query. Bind the diagnostic to the actual third match, not spawn order.
        let overflow_owner = extra.map(|_| {
            let world = sim.world_mut();
            let mut query = world.query::<(Entity, &mut Controller)>();
            query.iter_mut(world).nth(2).unwrap().0
        });
        // Invoke the controller reset handler on the damaged state directly:
        // the ordinary scene reset would repair native state first and hide it.
        sim.world_mut().run_system_once(reset).unwrap();
        let fault = sim
            .fault()
            .unwrap()
            .downcast_ref::<ControllerFault>()
            .unwrap();
        if let Some(expected) = overflow_owner {
            assert_eq!(fault.entity, expected);
            assert!(fault.message.contains("too many loaded controllers"));
        } else {
            assert_eq!(fault.entity, ids[1].0);
            assert!(fault.message.contains("joint identifier"));
        }
        assert_eq!(
            ids.iter()
                .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().target)
                .collect::<Vec<_>>(),
            targets
        );
        assert_eq!(
            native
                .iter()
                .map(|joint| sim.world().resource::<PhysicsBackend>().joint_state(*joint))
                .collect::<Vec<_>>(),
            before
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(sim.epoch(), 0);
    }
}

#[test]
fn last_controller_disappearance_is_inert_before_callbacks_are_physically_retired() {
    let mut sim = load(XML);
    let ids = controllers(&sim);
    let before = states(&sim);
    let native: Vec<_> = ids
        .iter()
        .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
        .collect();
    sim.initialize_schedules().unwrap();
    let counts = sim.scheduled_system_counts();
    for (entity, _) in ids {
        sim.world_mut().entity_mut(entity).remove::<Controller>();
    }
    sim.world_mut().resource_mut::<UpdateInfo>().dt = Duration::from_millis(1);
    apply(sim.world_mut());
    assert!(sim.fault().is_none());
    assert_eq!(
        native
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
    retire_if_unused(&mut sim).unwrap();
    assert!(!sim.world().contains_resource::<Installed>());
    let retired = sim.scheduled_system_counts();
    assert_eq!(counts.0, retired.0 + 1);
    assert_eq!(counts.1, retired.1 + 1);
    assert!(
        initial_joint_controller_configurations(&sim)
            .unwrap()
            .count()
            == 2,
        "retirement must preserve the loaded reset blueprint"
    );
}

#[test]
fn loaded_controller_admission_rejects_bad_configuration_unrepresentable_rate_and_force_conflict() {
    let plugin = "<plugin filename=\"gz-sim-joint-position-controller-system\"";
    let force = "<plugin name='gz::sim::systems::ApplyJointForce' filename='gz-sim-apply-joint-force-system'><joint_name>base_hinge</joint_name></plugin>";
    for (xml, expected) in [
        (
            XML.replace("<cmd_max>2</cmd_max>", "<cmd_max>0</cmd_max>"),
            "joint controller limits",
        ),
        (
            XML.replace("<cmd_max>2</cmd_max>", "<cmd_max>5e-324</cmd_max>"),
            "unrepresentable controller initialization",
        ),
        (
            XML.replacen(plugin, &format!("{force}{plugin}"), 1),
            "conflicting joint actuator ownership",
        ),
    ] {
        let error = match load_world_with_sdf_step(
            &xml,
            "m9-invalid.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        ) {
            Ok(_) => panic!("invalid loaded controller was admitted: {expected}"),
            Err(error) => error,
        };
        assert!(
            matches!(error, LoadError::Unsupported { feature, .. } if feature == expected),
            "expected {expected}"
        );
    }
    let sim = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_joint_controller_configurations(&sim),
        Err(SceneResetError::MissingInitialState)
    ));
}

#[test]
fn loaded_reduced_controller_chain_refuses_offset_centers_and_unrepresentable_torque_impulses() {
    for inertial in ["<inertial><mass>1</mass>", "<inertial><mass>0.5</mass>"] {
        for offset in ["0.01", "0.0000000000001"] {
            let moved = inertial.replacen(
                "<inertial>",
                &format!("<inertial><pose>{offset} 0 0 0 0 0</pose>"),
                1,
            );
            let xml = XML.replacen(inertial, &moved, 1);
            let error = match load_world_with_sdf_step(
                &xml,
                "m9-offset.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier,
            ) {
                Ok(_) => panic!("reduced controller chain admitted an unsupported inertial center"),
                Err(error) => error,
            };
            if offset == "0.01" {
                // Preserve the Scene's existing center-of-mass admission domain.
                assert!(
                    matches!(
                        &error,
                        LoadError::Unsupported {
                            feature: "offset center of mass",
                            ..
                        }
                    ),
                    "{inertial}/{offset}: {error:?}"
                );
            } else {
                // A nonzero center inside Scene's unchanged 1e-12 envelope must
                // still be refused by the reduced backend's exact-zero contract.
                assert!(
                    matches!(
                        &error,
                        LoadError::Physics(PhysicsError::Unsupported {
                            engine: "rapier",
                            capability: "offset articulation centers of mass"
                        })
                    ),
                    "{inertial}/{offset}: {error:?}"
                );
            }
        }
    }
    let xml = XML
        .replace(
            "<max_step_size>0.001</max_step_size>",
            "<max_step_size>2</max_step_size>",
        )
        .replacen(
            "<effort>3</effort>",
            "<effort>1.7976931348623157e308</effort>",
            1,
        );
    let error = match load_world_with_sdf_step(
        &xml,
        "m9-torque-overflow.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    ) {
        Ok(_) => panic!("finite torque whose next impulse overflows was admitted"),
        Err(error) => error,
    };
    assert!(matches!(
        error,
        LoadError::Physics(PhysicsError::InvalidValue)
    ));
}

#[test]
fn controller_retirement_propagates_schedule_failure_and_rejects_late_consumer_reentry() {
    use bevy_ecs::schedule::ScheduleBuilt;
    fn first() {}
    fn second() {}
    let mut invalid = load(XML);
    let ids = controllers(&invalid);
    let native: Vec<_> = ids
        .iter()
        .map(|(entity, _)| invalid.world().get::<Controller>(*entity).unwrap().joint)
        .collect();
    let before = states(&invalid);
    for (entity, _) in ids {
        invalid
            .world_mut()
            .entity_mut(entity)
            .remove::<Controller>();
    }
    invalid.add_systems(Phase::Reset, (first.before(second), second.before(first)));
    assert!(matches!(
        lifecycle::retire_unused_systems(&mut invalid),
        Err(lifecycle::SystemRetirementError::Schedule(_))
    ));
    assert!(invalid.world().contains_resource::<Installed>());
    assert_eq!(
        native
            .iter()
            .map(|joint| invalid
                .world()
                .resource::<PhysicsBackend>()
                .joint_state(*joint)
                .unwrap())
            .collect::<Vec<_>>(),
        before
    );
    assert_eq!(invalid.elapsed(), Duration::ZERO);

    let mut sim = load(XML);
    sim.initialize_schedules().unwrap();
    let ids = controllers(&sim);
    let copy = sim.world().get::<Controller>(ids[0].0).unwrap().clone();
    sim.world_mut()
        .add_observer(move |_: On<ScheduleBuilt>, mut commands: Commands| {
            commands.spawn(copy.clone());
        });
    let model = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::arm")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    assert!(matches!(
        remove_model(&mut sim, model),
        Err(RemoveError::CommittedSchedule(_))
    ));
    assert!(gz_sim_core::entity_by_id(sim.world(), model).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(
        sim.fault()
            .unwrap()
            .to_string()
            .contains("schedule observer added consumers")
    );
    assert_eq!(sim.step_exact(1), Err(gz_sim_core::StepError::Faulted));
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

#[test]
fn individually_finite_targets_with_overflowing_continuous_error_are_command_refusals() {
    // Server keeps its previous target only for InvalidCommand. A finite scalar
    // whose continuous subtraction overflows belongs to that same refusal class,
    // rather than being reported as a terminal physics ownership failure.
    let error = request(
        JointControllerMode::Position,
        f64::MAX,
        -f64::MAX,
        2.0,
        3.0,
        Duration::from_millis(1),
    )
    .map_err(command_error);
    assert!(matches!(
        error,
        Err(JointControllerCommandError::InvalidCommand)
    ));
}

#[test]
fn controller_child_corruption_refuses_removal_order_and_public_removal_without_native_changes() {
    for despawn_child in [false, true] {
        let mut sim = load(XML);
        let ids = controllers(&sim);
        set_joint_controller_command(&mut sim, ids[0].1, 0.4).unwrap();
        set_joint_controller_command(&mut sim, ids[1].1, 0.5).unwrap();
        let before = states(&sim);
        let bodies: std::collections::BTreeSet<_> = ids
            .iter()
            .map(|(entity, _)| {
                let native = sim.world().get::<Controller>(*entity).unwrap().joint;
                sim.world()
                    .resource::<PhysicsBackend>()
                    .joint_child(native)
                    .unwrap()
            })
            .collect();
        let body_states: Vec<_> = bodies
            .iter()
            .map(|body| sim.world().resource::<PhysicsBackend>().state(*body))
            .collect();
        let child_id = sim.world().get::<JointHandle>(ids[1].0).unwrap().child();
        let child = gz_sim_core::entity_by_id(sim.world(), child_id).unwrap();
        let model = sim.world().get::<ChildOf>(ids[0].0).unwrap().parent();
        let model_id = sim.world().get::<EntityId>(model).unwrap().get();
        if despawn_child {
            sim.world_mut().entity_mut(child).despawn();
        } else {
            sim.world_mut().entity_mut(child).remove::<BodyHandle>();
        }
        assert!(joints::removal_order(sim.world(), &bodies).is_none());
        assert_eq!(
            remove_model(&mut sim, model_id),
            Err(RemoveError::InvalidPhysics)
        );
        assert_eq!(
            gz_sim_core::entity_by_id(sim.world(), model_id),
            Some(model)
        );
        assert_eq!(states(&sim), before);
        assert_eq!(
            bodies
                .iter()
                .map(|body| sim.world().resource::<PhysicsBackend>().state(*body))
                .collect::<Vec<_>>(),
            body_states
        );
        assert_eq!(sim.world().get::<Controller>(ids[0].0).unwrap().target, 0.4);
        assert_eq!(sim.world().get::<Controller>(ids[1].0).unwrap().target, 0.5);
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}

#[test]
fn retired_controller_joint_callbacks_are_inert_and_stale_endpoint_lookup_refuses() {
    use bevy_ecs::system::RunSystemOnce;
    let mut sim = load(XML);
    let ids = controllers(&sim);
    let before = states(&sim);
    let native: Vec<_> = ids
        .iter()
        .map(|(entity, _)| sim.world().get::<Controller>(*entity).unwrap().joint)
        .collect();
    // Exercise the lifecycle gap before the installed callbacks retire, using
    // real loaded native joints rather than replacing the backend or its owner.
    for (entity, _) in &ids {
        sim.world_mut()
            .entity_mut(*entity)
            .remove::<(Controller, JointHandle, joints::JointNode)>();
    }
    sim.world_mut().run_system_once(joints::preflight).unwrap();
    assert!(sim.fault().is_none());
    let retired = ids[0].0;
    sim.world_mut().entity_mut(retired).despawn();
    assert!(joints::input_binding(sim.world(), retired).is_none());
    assert_eq!(
        native
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

#[test]
fn shared_joint_admission_refuses_an_axis_removed_after_controller_configuration_parse() {
    let sdf = sdformat::load_str(XML, "m9-axis.sdf", &sdformat::NoResources).unwrap();
    let source = sdf.worlds[0]
        .models
        .iter()
        .find(|model| model.name == "arm")
        .unwrap();
    assert!(parse(source).unwrap().is_some());
    let mut damaged = source.clone();
    damaged
        .joints
        .iter_mut()
        .find(|joint| joint.name == "base_hinge")
        .unwrap()
        .axis = None;
    let error = joints::validate_model(
        &sdf,
        &damaged,
        "default",
        true,
        Some("rapier"),
        false,
        &["base_hinge", "tip_hinge"],
    )
    .unwrap_err();
    assert!(
        matches!(error, LoadError::Unsupported { entity, feature: "joint without axis" } if entity == "default::arm::base_hinge")
    );
    assert!(
        source.joints.iter().all(|joint| joint.axis.is_some()),
        "admission must leave the original parsed graph intact"
    );
}
