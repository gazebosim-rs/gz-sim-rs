// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::system::RunSystemOnce;
use std::collections::BTreeMap;
const XML: &str = include_str!("../../../../examples/m8_diff_drive.sdf");

fn fleet_xml(count: usize) -> String {
    let begin = XML.find("<model name=\"robot\"").unwrap();
    let end = begin + XML[begin..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[begin..end];
    let models = (0..count)
        .map(|index| {
            let x = if index.is_multiple_of(2) { -1.25 } else { 1.25 };
            let y = -2.25 + (index / 2) as f64 * 1.5;
            robot
                .replace("name=\"robot\"", &format!("name=\"fleet_{index}\""))
                .replace(
                    "<pose>0 0 0.2 0 0 0</pose>",
                    &format!("<pose>{x} {y} 0.2 0 0 0</pose>"),
                )
                .replace("/model/robot/cmd_vel", &format!("/fleet/{index}/cmd_vel"))
                .replace("/model/robot/odometry", &format!("/fleet/{index}/odom"))
                .replace("robot/odom", &format!("fleet_{index}_odom"))
                .replace("robot/base_link", &format!("fleet_{index}_base"))
        })
        .collect::<Vec<_>>()
        .join("");
    XML.replacen(robot, &models, 1)
}

fn pair_xml() -> String {
    let begin = XML.find("<model name=\"robot\"").unwrap();
    let end = begin + XML[begin..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[begin..end];
    let owner = |side: &str, y: i32| {
        robot
            .replace("name=\"robot\"", &format!("name=\"robot_{side}\""))
            .replace(
                "<pose>0 0 0.2 0 0 0</pose>",
                &format!("<pose>0 {y} 0.2 0 0 0</pose>"),
            )
            .replace("/model/robot/cmd_vel", &format!("/m59/{side}/cmd_vel"))
            .replace("/model/robot/odometry", &format!("/m59/{side}/odom"))
            .replace("robot/odom", &format!("m59_{side}_odom"))
            .replace("robot/base_link", &format!("m59_{side}_base"))
    };
    XML.replacen(robot, &(owner("left", -2) + &owner("right", 2)), 1)
}
fn load_xml(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m59-two-drives.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn load_pair() -> Simulation {
    load_xml(&pair_xml())
}
fn owners(sim: &Simulation) -> BTreeMap<String, (Entity, u64)> {
    sim.world()
        .iter_entities()
        .filter_map(|row| {
            let drive = row.get::<Drive>()?;
            Some((
                drive.publication.model_name.clone(),
                (row.id(), row.get::<EntityId>().unwrap().get()),
            ))
        })
        .collect()
}
type NativeStates = Vec<(JointId, Option<[u64; 2]>)>;
type SampleCache = (Duration, u64, [u64; 3], [u64; 2]);
type Cache = (String, String, Option<SampleCache>);
fn native(sim: &Simulation) -> NativeStates {
    let backend = sim.world().resource::<PhysicsBackend>();
    let mut result: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|row| row.get::<JointHandle>())
        .map(|handle| {
            (
                handle.id(),
                backend
                    .joint_state(handle.id())
                    .map(|state| [state.position.to_bits(), state.velocity.to_bits()]),
            )
        })
        .collect();
    result.sort_by_key(|row| row.0);
    result
}
fn cache(sim: &Simulation, entity: Entity) -> Cache {
    let drive = sim.world().get::<Drive>(entity).unwrap();
    (
        format!("{:?}", drive.odometry),
        format!("{:?}", drive.schedule),
        sim.world().get::<DiffDriveSample>(entity).map(|row| {
            (
                row.time,
                row.epoch,
                row.pose.map(f64::to_bits),
                row.twist.map(f64::to_bits),
            )
        }),
    )
}
fn advancing(sim: &mut Simulation, now: Duration) {
    let mut time = *sim.world().resource::<UpdateInfo>();
    time.sim_time = now;
    time.dt = Duration::from_millis(1);
    time.paused = false;
    sim.world_mut().insert_resource(time);
}

#[test]
fn two_loaded_owners_keep_commands_motor_motion_and_encoder_origins_independent() {
    let mut sim = load_pair();
    let values = owners(&sim);
    assert_eq!(values.len(), 2);
    let (left, left_id) = values["robot_left"];
    let (right, right_id) = values["robot_right"];
    assert_ne!(left_id, right_id);
    assert_eq!(
        initial_diff_drive_configurations_with_identity(&sim)
            .unwrap()
            .count(),
        2
    );
    let before = native(&sim);
    set_diff_drive_command(&mut sim, left_id, 0.4, 0.0).unwrap();
    assert_eq!(sim.world().get::<Drive>(left).unwrap().desired, [0.4, 0.0]);
    assert_eq!(sim.world().get::<Drive>(right).unwrap().desired, [0.0, 0.0]);
    set_diff_drive_command(&mut sim, right_id, -0.4, 0.0).unwrap();
    assert_eq!(native(&sim), before); // Pending commands never teleport wheel state.
    sim.step_exact(1000).unwrap();
    let l = sim.world().get::<DiffDriveSample>(left).unwrap();
    let r = sim.world().get::<DiffDriveSample>(right).unwrap();
    assert!(l.pose[0] > 0.01 && r.pose[0] < -0.01);
    assert!(sim.world().get::<WorldPose>(left).unwrap().translation[0] > 0.01);
    assert!(sim.world().get::<WorldPose>(right).unwrap().translation[0] < -0.01);
    assert_eq!(
        (l.time, r.time, l.epoch, r.epoch),
        (Duration::from_secs(1), Duration::from_secs(1), 0, 0)
    );
    for (entity, sign) in [(left, 1.0), (right, -1.0)] {
        for joint in sim.world().get::<Drive>(entity).unwrap().joints {
            let state = sim
                .world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap();
            assert!(state.position * sign > 0.01 && state.velocity * sign > 0.01);
        }
    }
}

#[test]
fn original_private_wheel_witness_is_local_and_refuses_coordinated_metadata_changes() {
    for corruption in 0..5 {
        let mut sim = load_pair();
        let values = owners(&sim);
        let (left, left_id) = values["robot_left"];
        let (_, right_id) = values["robot_right"];
        assert!(diff_drive_binding_matches(&sim, left_id));
        match corruption {
            0 => sim
                .world_mut()
                .get_mut::<Drive>(left)
                .unwrap()
                .joints
                .swap(0, 1),
            1 => sim.world_mut().get_mut::<Drive>(left).unwrap().owner = right_id,
            2 => {
                sim.world_mut().entity_mut(left).remove::<Drive>();
            }
            3 => {
                let mut changed = sim
                    .world()
                    .get::<DiffDriveConfiguration>(left)
                    .unwrap()
                    .clone();
                changed.frame_id.push_str("/counterfeit");
                sim.world_mut().get_mut::<Drive>(left).unwrap().publication = changed.clone();
                sim.world_mut().entity_mut(left).insert(changed);
            }
            _ => {
                let joint = sim.world().get::<Drive>(left).unwrap().joints[0];
                let node = sim
                    .world()
                    .iter_entities()
                    .find(|row| {
                        row.get::<JointHandle>()
                            .is_some_and(|handle| handle.id() == joint)
                    })
                    .unwrap()
                    .id();
                sim.world_mut()
                    .get_mut::<JointDefinition>(node)
                    .unwrap()
                    .0
                    .axis
                    .as_mut()
                    .unwrap()
                    .velocity = 9.0;
            }
        }
        assert!(!diff_drive_binding_matches(&sim, left_id));
        // This predicate does not scan or reject unrelated owners.
        assert!(diff_drive_binding_matches(&sim, right_id));
    }
    let mut sim = load_pair();
    let values = owners(&sim);
    let left_id = values["robot_left"].1;
    let mut foreign = load_pair();
    let backend = foreign
        .world_mut()
        .remove_resource::<PhysicsBackend>()
        .unwrap();
    sim.world_mut().insert_resource(backend);
    assert!(!diff_drive_binding_matches(&sim, left_id));
}

#[test]
fn every_original_live_owner_preflights_before_healthy_cache_or_physics_advances() {
    for corruption in 0..4 {
        let mut sim = load_pair();
        let values = owners(&sim);
        let (left, left_id) = values["robot_left"];
        let (right, right_id) = values["robot_right"];
        set_diff_drive_command(&mut sim, left_id, 0.4, 0.0).unwrap();
        set_diff_drive_command(&mut sim, right_id, -0.4, 0.0).unwrap();
        match corruption {
            0 => {
                sim.world_mut()
                    .entity_mut(right)
                    .remove::<Drive>()
                    .remove::<DiffDriveConfiguration>();
            }
            1 => sim.world_mut().get_mut::<Drive>(right).unwrap().owner = left_id,
            2 => sim.world_mut().get_mut::<Drive>(right).unwrap().desired[0] = f64::NAN,
            _ => {
                let duplicate = sim.world().get::<Drive>(right).unwrap().clone();
                sim.world_mut().spawn(duplicate); // Counterfeit, never a positive owner.
            }
        }
        assert!(sim.world().get_entity(right).is_ok());
        let before = native(&sim);
        let held = cache(&sim, left);
        advancing(&mut sim, Duration::from_millis(1));
        apply(sim.world_mut());
        assert!(sim.fault().is_some());
        assert_eq!(native(&sim), before);
        assert_eq!(cache(&sim, left), held);
        assert_eq!(sim.world().get::<Drive>(left).unwrap().desired, [0.4, 0.0]);
        observe(sim.world_mut());
        assert_eq!(cache(&sim, left), held);
        assert!(sim.step_exact(1).is_err());
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(native(&sim), before);
    }
}

#[test]
fn owner_cadence_and_paused_receipt_do_not_depend_on_publication_consumers() {
    let xml = pair_xml();
    let at = xml.find("<model name=\"robot_right\"").unwrap();
    let xml = xml[..at].to_owned()
        + &xml[at..].replace(
            "<odom_publish_frequency>50</odom_publish_frequency>",
            "<odom_publish_frequency>25</odom_publish_frequency>",
        );
    let mut sim = load_xml(&xml);
    let values = owners(&sim);
    let (left, left_id) = values["robot_left"];
    let (right, right_id) = values["robot_right"];
    let mut stamps = [Vec::new(), Vec::new()];
    for _ in 0..80 {
        sim.step_exact(1).unwrap();
        for (index, entity) in [left, right].into_iter().enumerate() {
            if let Some(sample) = sim.world().get::<DiffDriveSample>(entity)
                && stamps[index].last() != Some(&sample.time)
            {
                stamps[index].push(sample.time);
            }
        }
    }
    assert_eq!(
        stamps[0],
        (1..=4)
            .map(|n| Duration::from_millis(n * 20))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        stamps[1],
        (1..=2)
            .map(|n| Duration::from_millis(n * 40))
            .collect::<Vec<_>>()
    );
    sim.set_paused(true);
    let before = native(&sim);
    let held = [cache(&sim, left), cache(&sim, right)];
    let time = sim.elapsed();
    set_diff_drive_command(&mut sim, left_id, 0.3, 0.0).unwrap();
    set_diff_drive_command(&mut sim, right_id, -0.2, 0.0).unwrap();
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), time);
    assert_eq!(native(&sim), before);
    assert_eq!([cache(&sim, left), cache(&sim, right)], held);
}

#[test]
fn late_encoder_failure_cannot_commit_an_earlier_owners_history_or_sample() {
    let mut sim = load_pair();
    sim.step_exact(20).unwrap();
    // Witness the actual enumeration order used by observe, not model-name order.
    let ordered: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|row| row.contains::<Drive>())
        .map(|row| row.id())
        .collect();
    assert_eq!(ordered.len(), 2);
    let [early, late] = [ordered[0], ordered[1]];
    let held = cache(&sim, early);
    sim.world_mut().get_mut::<Drive>(late).unwrap().odometry = None;
    advancing(&mut sim, Duration::from_millis(21));
    observe(sim.world_mut());
    let fault = sim.fault().unwrap().downcast_ref::<DriveFault>().unwrap();
    assert_eq!(fault.entity, late);
    assert_eq!(fault.message, "missing initial encoder reference");
    assert_eq!(cache(&sim, early), held);
}

#[test]
fn failed_reset_discards_every_pending_command_and_sample_but_does_not_actuate() {
    for missing_reference in [false, true] {
        let mut sim = load_pair();
        let values = owners(&sim);
        let (left, left_id) = values["robot_left"];
        let (right, right_id) = values["robot_right"];
        set_diff_drive_command(&mut sim, left_id, 0.4, 0.0).unwrap();
        set_diff_drive_command(&mut sim, right_id, -0.4, 0.0).unwrap();
        sim.step_exact(20).unwrap();
        if missing_reference {
            sim.world_mut().get_mut::<Drive>(right).unwrap().odometry = None;
        } else {
            let joint = sim.world().get::<Drive>(right).unwrap().joints[1];
            assert!(
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove_joint(joint)
            );
        }
        let held = native(&sim);
        sim.world_mut().run_system_once(reset).unwrap();
        assert!(sim.fault().is_some());
        for entity in [left, right] {
            assert_eq!(sim.world().get::<Drive>(entity).unwrap().desired, [0.0; 2]);
            assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
        }
        assert_eq!(native(&sim), held);
        assert!(sim.step_exact(1).is_err());
    }
}

#[test]
fn whole_model_retirement_keeps_peer_live_and_loaded_reset_restores_original_pairs() {
    let mut sim = load_pair();
    let values = owners(&sim);
    let (left, left_id) = values["robot_left"];
    let (_, right_id) = values["robot_right"];
    let originals: Vec<_> = initial_diff_drive_configurations_with_identity(&sim)
        .unwrap()
        .map(|row| {
            (
                row.id,
                row.name.to_owned(),
                row.parent,
                row.configuration.clone(),
            )
        })
        .collect();
    assert!(remove_model(&mut sim, left_id).unwrap().contains(&left_id));
    assert!(sim.world().get_entity(left).is_err());
    assert!(sim.world().contains_resource::<Installed>());
    assert!(diff_drive_binding_matches(&sim, right_id));
    set_diff_drive_command(&mut sim, right_id, -0.4, 0.0).unwrap();
    sim.step_exact(1000).unwrap();
    let remaining = owners(&sim);
    assert_eq!(remaining.len(), 1);
    assert!(
        sim.world()
            .get::<DiffDriveSample>(remaining["robot_right"].0)
            .unwrap()
            .pose[0]
            < -0.01
    );
    assert_eq!(
        initial_diff_drive_configurations_with_identity(&sim)
            .unwrap()
            .count(),
        2
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored: Vec<_> = initial_diff_drive_configurations_with_identity(&sim)
        .unwrap()
        .map(|row| {
            (
                row.id,
                row.name.to_owned(),
                row.parent,
                row.configuration.clone(),
            )
        })
        .collect();
    assert_eq!(restored, originals);
    for (entity, id) in owners(&sim).values() {
        assert!(diff_drive_binding_matches(&sim, *id));
        assert_eq!(sim.world().get::<Drive>(*entity).unwrap().desired, [0.0; 2]);
        assert!(sim.world().get::<DiffDriveSample>(*entity).is_none());
    }
    remove_model(&mut sim, left_id).unwrap();
    remove_model(&mut sim, right_id).unwrap();
    assert!(!sim.world().contains_resource::<Installed>());
}

#[test]
fn zero_singleton_two_and_third_owner_domains_keep_the_original_numeric_leafs() {
    let singleton = load_xml(XML);
    assert_eq!(owners(&singleton).len(), 1);
    let none = XML[..XML.find("<model name=\"robot\"").unwrap()].to_owned() + "</world></sdf>";
    let absent = load_xml(&none);
    assert!(owners(&absent).is_empty());
    assert!(!absent.world().contains_resource::<Installed>());
    assert_eq!(
        initial_diff_drive_configurations_with_identity(&absent)
            .unwrap()
            .count(),
        0
    );
    let two = pair_xml();
    let begin = two.find("<model name=\"robot_right\"").unwrap();
    let end = begin + two[begin..].find("</model>").unwrap() + "</model>".len();
    let third = two[begin..end].replace("robot_right", "robot_third");
    let three = two.replace("</world>", &(third + "</world>"));
    assert_eq!(owners(&load_xml(&three)).len(), 3);
    assert!(matches!(
        load_world_with_sdf_step(
            &fleet_xml(9),
            "m61-ninth.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "multiple differential-drive systems",
            ..
        })
    ));
}

#[test]
fn restoration_witness_isolates_public_frame_drift_from_private_and_native_corruption() {
    for corruption in 0..6 {
        let mut sim = load_pair();
        let values = owners(&sim);
        let (left, left_id) = values["robot_left"];
        let (_, right_id) = values["robot_right"];
        let original = initial_diff_drive_configurations_with_identity(&sim)
            .unwrap()
            .find(|row| row.id == left_id)
            .unwrap()
            .configuration
            .clone();
        let mut drift = original.clone();
        drift.frame_id.push_str("/public-reset-drift");
        sim.world_mut().entity_mut(left).insert(drift.clone());
        assert!(!diff_drive_binding_matches(&sim, left_id));
        assert!(diff_drive_binding_matches_configuration(
            &sim, left_id, &original
        ));
        assert!(!diff_drive_binding_matches_configuration(
            &sim, left_id, &drift
        ));
        assert!(diff_drive_binding_matches(&sim, right_id));
        match corruption {
            0 => {} // Public-only drift is the sole relaxed equality.
            1 => sim.world_mut().get_mut::<Drive>(left).unwrap().owner = right_id,
            2 => sim
                .world_mut()
                .get_mut::<Drive>(left)
                .unwrap()
                .joints
                .swap(0, 1),
            3 => sim.world_mut().get_mut::<Drive>(left).unwrap().publication = drift.clone(),
            4 => {
                sim.world_mut().entity_mut(left).remove::<Drive>();
            }
            _ => {
                let joint = sim.world().get::<Drive>(left).unwrap().joints[0];
                assert!(
                    sim.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .remove_joint(joint)
                );
            }
        }
        let held = native(&sim);
        assert_eq!(
            diff_drive_binding_matches_configuration(&sim, left_id, &original),
            corruption == 0
        );
        assert!(!diff_drive_binding_matches(&sim, left_id));
        assert!(diff_drive_binding_matches(&sim, right_id));
        assert_eq!(native(&sim), held); // Both witnesses are pure, even on refusal.
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}

#[test]
fn incomplete_original_graph_refuses_commands_without_mutating_either_owner() {
    for corruption in 0..8 {
        let mut sim = load_pair();
        let values = owners(&sim);
        let (left, left_id) = values["robot_left"];
        let (right, right_id) = values["robot_right"];
        let original = sim
            .world()
            .get::<DiffDriveConfiguration>(left)
            .unwrap()
            .clone();
        let world = sim.world().get::<ChildOf>(left).unwrap().parent();
        let joint = sim.world().get::<Drive>(left).unwrap().joints[0];
        let wheel = sim
            .world()
            .iter_entities()
            .find(|row| {
                row.get::<JointHandle>()
                    .is_some_and(|handle| handle.id() == joint)
            })
            .unwrap()
            .id();
        let child_id = sim.world().get::<JointHandle>(wheel).unwrap().child();
        let child = gz_sim_core::entity_by_id(sim.world(), child_id).unwrap();
        let parent_name = sim
            .world()
            .get::<JointDefinition>(wheel)
            .unwrap()
            .0
            .parent
            .clone();
        let chassis = sim
            .world()
            .iter_entities()
            .find(|row| {
                row.get::<ChildOf>().map(ChildOf::parent) == Some(left)
                    && row
                        .get::<Name>()
                        .is_some_and(|name| name.0 == format!("default::robot_left::{parent_name}"))
            })
            .unwrap()
            .id();
        match corruption {
            0 => {
                sim.world_mut()
                    .get_mut::<JointDefinition>(wheel)
                    .unwrap()
                    .0
                    .axis = None
            }
            1 => {
                sim.world_mut().entity_mut(wheel).remove::<EntityId>();
            }
            2 => {
                sim.world_mut()
                    .entity_mut(wheel)
                    .remove::<JointDefinition>();
            }
            3 => {
                sim.world_mut().entity_mut(child).remove::<EntityId>();
            }
            4 => sim
                .world_mut()
                .get_mut::<Name>(chassis)
                .unwrap()
                .0
                .push_str("/counterfeit"),
            5 => {
                sim.world_mut().entity_mut(world).remove::<EntityId>();
            }
            6 => {
                sim.world_mut().entity_mut(left).remove::<EntityId>();
            }
            _ => {} // An unknown original ID cannot borrow another owner's configuration.
        }
        let id = if corruption == 7 { u64::MAX } else { left_id };
        let before = native(&sim);
        let held = [cache(&sim, left), cache(&sim, right)];
        let desired = [
            sim.world().get::<Drive>(left).unwrap().desired,
            sim.world().get::<Drive>(right).unwrap().desired,
        ];
        assert!(
            !diff_drive_binding_matches_configuration(&sim, id, &original),
            "case {corruption}"
        );
        assert!(!diff_drive_binding_matches(&sim, id), "case {corruption}");
        let command = set_diff_drive_command(&mut sim, id, 0.4, 0.0);
        if matches!(corruption, 1 | 3 | 6) {
            // Broken native identity bindings are rejected before local metadata.
            assert!(
                matches!(command, Err(DiffDriveCommandError::ForeignPhysics)),
                "case {corruption}"
            );
        } else {
            assert!(
                matches!(command, Err(DiffDriveCommandError::MissingDrive(missing)) if missing == id),
                "case {corruption}"
            );
        }
        if corruption != 5 {
            assert!(
                diff_drive_binding_matches(&sim, right_id),
                "case {corruption}"
            );
        }
        assert_eq!(native(&sim), before, "case {corruption}");
        assert_eq!(
            [cache(&sim, left), cache(&sim, right)],
            held,
            "case {corruption}"
        );
        assert_eq!(
            [
                sim.world().get::<Drive>(left).unwrap().desired,
                sim.world().get::<Drive>(right).unwrap().desired,
            ],
            desired,
            "case {corruption}"
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert!(sim.fault().is_none());
    }
}

#[test]
fn absent_original_snapshot_and_configuration_only_counterfeits_fail_closed() {
    let empty = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(active_drives(empty.world()).unwrap().is_empty());

    let mut loaded = load_pair();
    let values = owners(&loaded);
    let (left, left_id) = values["robot_left"];
    let (right, _) = values["robot_right"];
    let configuration = loaded
        .world()
        .get::<DiffDriveConfiguration>(left)
        .unwrap()
        .clone();
    loaded
        .world_mut()
        .remove_resource::<initial::InitialScene>()
        .unwrap();
    let before = native(&loaded);
    let held = [cache(&loaded, left), cache(&loaded, right)];
    assert!(!diff_drive_binding_matches_configuration(
        &loaded,
        left_id,
        &configuration
    ));
    assert_eq!(
        active_drives(loaded.world()).unwrap_err().message,
        "invalid loaded drive ownership"
    );
    advancing(&mut loaded, Duration::from_millis(1));
    apply(loaded.world_mut());
    assert!(loaded.fault().is_some());
    assert_eq!(native(&loaded), before);
    assert_eq!([cache(&loaded, left), cache(&loaded, right)], held);
    assert_eq!(loaded.elapsed(), Duration::ZERO);

    let no_drives = XML[..XML.find("<model name=\"robot\"").unwrap()].to_owned() + "</world></sdf>";
    let mut absent = load_xml(&no_drives);
    assert!(active_drives(absent.world()).unwrap().is_empty());
    let counterfeit = absent.world_mut().spawn(configuration).id();
    let fault = active_drives(absent.world()).unwrap_err();
    assert_eq!(fault.entity, counterfeit);
    assert_eq!(fault.message, "invalid loaded drive ownership");
    assert!(native(&absent).is_empty());
    assert_eq!(absent.elapsed(), Duration::ZERO);
}

#[test]
fn all_eight_bindings_are_preflighted_before_cache_updates() {
    for callback in [apply as fn(&mut World), observe] {
        for invalid in 0..8 {
            let mut sim = load_xml(&fleet_xml(8));
            let values = owners(&sim);
            for (_, id) in values.values() {
                set_diff_drive_command(&mut sim, *id, 0.4, 0.0).unwrap();
            }
            sim.step_exact(20).unwrap();
            let entity = values[&format!("fleet_{invalid}")].0;
            let mut configuration = sim
                .world()
                .get::<DiffDriveConfiguration>(entity)
                .unwrap()
                .clone();
            configuration.frame_id.push_str("_corrupt");
            sim.world_mut().entity_mut(entity).insert(configuration);
            let held: Vec<_> = values
                .values()
                .map(|(entity, _)| cache(&sim, *entity))
                .collect();
            let before = native(&sim);
            advancing(&mut sim, Duration::from_millis(40));
            callback(sim.world_mut());
            assert!(sim.fault().is_some());
            assert_eq!(native(&sim), before);
            let after: Vec<_> = values
                .values()
                .map(|(entity, _)| cache(&sim, *entity))
                .collect();
            assert_eq!(after, held, "invalid owner {invalid}");
        }
    }
}

#[test]
fn ninth_counterfeit_reset_clears_every_bounded_command_and_sample_before_refusal() {
    let mut sim = load_xml(&fleet_xml(8));
    let values = owners(&sim);
    for (_, id) in values.values() {
        set_diff_drive_command(&mut sim, *id, 0.4, 0.0).unwrap();
    }
    sim.step_exact(20).unwrap();
    let original = values["fleet_7"].0;
    let duplicate = sim.world().get::<Drive>(original).unwrap().clone();
    let source = sim.world().get::<DiffDriveSample>(original).unwrap();
    let sample = DiffDriveSample {
        time: source.time,
        epoch: source.epoch,
        pose: source.pose,
        twist: source.twist,
    };
    let counterfeit = sim.world_mut().spawn((duplicate, sample)).id();
    for entity in values.values().map(|value| value.0).chain([counterfeit]) {
        assert_eq!(
            sim.world().get::<Drive>(entity).unwrap().desired,
            [0.4, 0.0]
        );
        assert!(sim.world().get::<DiffDriveSample>(entity).is_some());
    }
    let before = native(&sim);
    reset(sim.world_mut());
    assert!(sim.fault().is_some());
    assert_eq!(native(&sim), before);
    for entity in values.values().map(|value| value.0).chain([counterfeit]) {
        assert_eq!(
            sim.world().get::<Drive>(entity).unwrap().desired,
            [0.0, 0.0]
        );
        assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
    }
}
