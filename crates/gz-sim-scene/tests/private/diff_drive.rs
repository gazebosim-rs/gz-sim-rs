// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const XML: &str = include_str!("../../../../examples/m8_diff_drive.sdf");
fn load() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m8.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn drive(sim: &Simulation) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<Drive>())
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
#[test]
fn command_limits_are_atomic_and_not_physical_velocity_teleports() {
    let mut sim = load();
    let (entity, id) = drive(&sim);
    let joints = sim.world().get::<Drive>(entity).unwrap().joints;
    let before = joints.map(|id| {
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(id)
            .unwrap()
    });
    set_diff_drive_command(&mut sim, id, 100.0, -100.0).unwrap();
    assert_eq!(
        sim.world().get::<Drive>(entity).unwrap().desired,
        [1.0, -2.0]
    );
    assert_eq!(
        before,
        joints.map(|id| sim
            .world()
            .resource::<PhysicsBackend>()
            .joint_state(id)
            .unwrap())
    );
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            set_diff_drive_command(&mut sim, id, value, 0.0),
            Err(DiffDriveCommandError::InvalidCommand)
        ));
        assert_eq!(
            sim.world().get::<Drive>(entity).unwrap().desired,
            [1.0, -2.0]
        );
    }
    assert!(matches!(
        set_diff_drive_command(&mut sim, u64::MAX, 0.0, 0.0),
        Err(DiffDriveCommandError::MissingDrive(_))
    ));
    sim.world_mut()
        .entity_mut(entity)
        .remove::<DiffDriveConfiguration>();
    assert!(matches!(
        set_diff_drive_command(&mut sim, id, 0.0, 0.0),
        Err(DiffDriveCommandError::MissingDrive(_))
    ));
}
#[test]
fn real_wheel_encoders_drive_fifty_hertz_odometry_and_reset_retires_state() {
    let mut sim = load();
    let (entity, id) = drive(&sim);
    set_diff_drive_command(&mut sim, id, 0.4, 0.0).unwrap();
    let mut stamps = Vec::new();
    for _ in 0..1000 {
        sim.step_exact(1).unwrap();
        if let Some(sample) = sim.world().get::<DiffDriveSample>(entity) {
            if stamps.last() != Some(&sample.time) {
                stamps.push(sample.time);
            }
            assert_eq!(sample.epoch, 0);
            assert!(
                sample
                    .pose
                    .into_iter()
                    .chain(sample.twist)
                    .all(f64::is_finite)
            );
        }
    }
    assert_eq!(
        stamps,
        (1..=50)
            .map(|index| Duration::from_millis(index * 20))
            .collect::<Vec<_>>()
    );
    let sample = sim.world().get::<DiffDriveSample>(entity).unwrap();
    assert!(
        sample.pose[0] > 0.01,
        "positive actual wheel rotation must advance encoder odometry"
    );
    let model_x = sim.world().get::<WorldPose>(entity).unwrap().translation[0];
    assert!(
        model_x > 0.01,
        "motor constraints and contact must move the actual robot"
    );
    let paused_time = sim.elapsed();
    let paused_sample = sample.time;
    sim.set_paused(true);
    set_diff_drive_command(&mut sim, id, 0.0, 1.0).unwrap();
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), paused_time);
    assert_eq!(
        sim.world().get::<DiffDriveSample>(entity).unwrap().time,
        paused_sample
    );
    sim.reset().unwrap();
    assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
    assert_eq!(sim.world().get::<Drive>(entity).unwrap().desired, [0.0; 2]);
    assert!(sim.world().get::<Drive>(entity).unwrap().odometry.is_some());
    set_diff_drive_command(&mut sim, id, 0.0, 1.0).unwrap();
    sim.step_exact(1000).unwrap();
    assert!(sim.world().get::<DiffDriveSample>(entity).unwrap().pose[2] > 0.01);
    let rotation = sim.world().get::<WorldPose>(entity).unwrap().rotation;
    let forward = DQuat::from_array(rotation) * DVec3::X;
    assert!(
        forward.x.hypot(forward.y) > 0.5,
        "robot forward axis retains a horizontal heading"
    );
    assert!(
        forward.y.atan2(forward.x) > 0.01,
        "differential wheel contact must turn the actual chassis heading"
    );
    let removed = remove_model(&mut sim, id).unwrap();
    assert!(removed.contains(&id));
    assert!(!sim.world().contains_resource::<Installed>());
    assert_eq!(initial_diff_drive_configurations(&sim).unwrap().count(), 1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let (restored, restored_id) = drive(&sim);
    assert_eq!(restored_id, id);
    assert_eq!(
        sim.world().get::<Drive>(restored).unwrap().desired,
        [0.0; 2]
    );
    assert!(sim.world().contains_resource::<Installed>());
    assert!(sim.world().get::<DiffDriveSample>(restored).is_none());
}
#[test]
fn missing_family_installs_no_drive_callbacks_or_buffers_and_other_engines_refuse() {
    let plain = "<sdf version='1.12'><world name='default'/></sdf>";
    let sim = load_world_with_sdf_step(
        plain,
        "empty.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(
        !sim.world()
            .iter_entities()
            .any(|entity| entity.contains::<Drive>() || entity.contains::<DiffDriveSample>())
    );
    assert_eq!(initial_diff_drive_configurations(&sim).unwrap().count(), 0);
    for engine in [
        PhysicsEngine::Tpe,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ] {
        assert!(
            load_world_with_sdf_step(XML, "m8.sdf", "default", &sdformat::NoResources, engine)
                .is_err()
        );
    }
}

#[test]
fn first_advancing_tick_is_integrated_and_missing_native_bindings_refuse_commands() {
    let mut sim = load();
    let (entity, id) = drive(&sim);
    let geometry = sim.world().get::<Drive>(entity).unwrap().geometry;
    let joints = sim.world().get::<Drive>(entity).unwrap().joints;
    let angles = joints.map(|joint| {
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(joint)
            .unwrap()
            .position
    });
    let mut reference = model::Odometry::new(geometry, angles, Duration::ZERO).unwrap();
    set_diff_drive_command(&mut sim, id, 0.4, 0.0).unwrap();
    for index in 1..=20 {
        sim.step_exact(1).unwrap();
        let angles = joints.map(|joint| {
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap()
                .position
        });
        let expected = reference
            .observe(angles, Duration::from_millis(index))
            .unwrap();
        if index == 20 {
            let sample = sim.world().get::<DiffDriveSample>(entity).unwrap();
            assert_eq!(sample.pose, expected.pose);
            assert_eq!(sample.twist, expected.twist);
        }
    }
    let wheel = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<JointHandle>()
                .is_some_and(|handle| handle.id() == joints[0])
        })
        .unwrap()
        .id();
    sim.world_mut().entity_mut(wheel).remove::<JointHandle>();
    assert!(matches!(
        set_diff_drive_command(&mut sim, id, 0.0, 0.0),
        Err(DiffDriveCommandError::ForeignPhysics)
    ));
    assert_eq!(
        sim.world().get::<Drive>(entity).unwrap().desired,
        [0.4, 0.0]
    );
    assert!(sim.step_exact(1).is_err());
}

fn advancing_time(sim: &mut Simulation, now: Duration) {
    let mut time = *sim.world().resource::<UpdateInfo>();
    time.dt = Duration::from_millis(1);
    time.sim_time = now;
    time.paused = false;
    sim.world_mut().insert_resource(time);
}

#[test]
fn invalid_controller_state_cannot_replace_commands_or_actuate_native_wheels() {
    for invalid in 0..4 {
        let mut sim = load();
        let (entity, id) = drive(&sim);
        set_diff_drive_command(&mut sim, id, 0.4, 0.0).unwrap();
        let joints = sim.world().get::<Drive>(entity).unwrap().joints;
        let before = joints.map(|joint| {
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap()
        });
        match invalid {
            0 => sim.world_mut().get_mut::<Drive>(entity).unwrap().speed[0] = 0.0,
            1 => sim.world_mut().get_mut::<Drive>(entity).unwrap().force[1] = f64::NAN,
            2 => {
                let mut changed = sim
                    .world()
                    .get::<DiffDriveConfiguration>(entity)
                    .unwrap()
                    .clone();
                changed.frame_id.push_str("/foreign");
                sim.world_mut().entity_mut(entity).insert(changed);
            }
            _ => {
                sim.world_mut().entity_mut(entity).remove::<Drive>();
            }
        }
        assert!(set_diff_drive_command(&mut sim, id, 0.0, 0.0).is_err());
        assert_eq!(
            before,
            joints.map(|joint| sim
                .world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap())
        );
        if let Some(driver) = sim.world().get::<Drive>(entity) {
            assert_eq!(driver.desired, [0.4, 0.0]);
        }
    }
    for invalid in 0..4 {
        let mut sim = load();
        let (entity, _) = drive(&sim);
        let joints = sim.world().get::<Drive>(entity).unwrap().joints;
        let before = joints.map(|joint| {
            sim.world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap()
        });
        match invalid {
            0 => {
                let duplicate = sim.world().get::<Drive>(entity).unwrap().clone();
                sim.world_mut().spawn(duplicate);
            }
            1 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<DiffDriveConfiguration>();
            }
            2 => sim.world_mut().get_mut::<Drive>(entity).unwrap().desired[0] = f64::NAN,
            _ => sim.world_mut().get_mut::<Drive>(entity).unwrap().force[1] = f64::NAN,
        }
        advancing_time(&mut sim, Duration::from_millis(1));
        apply(sim.world_mut());
        assert!(sim.fault().is_some());
        assert_eq!(
            before,
            joints.map(|joint| sim
                .world()
                .resource::<PhysicsBackend>()
                .joint_state(joint)
                .unwrap())
        );
        // A previously faulted run cannot install new targets or advance physics.
        apply(sim.world_mut());
        assert!(sim.step_exact(1).is_err());
    }
}

#[test]
fn encoder_and_clock_failures_are_terminal_and_reset_discards_stale_samples() {
    use bevy_ecs::system::RunSystemOnce;
    for failure in 0..4 {
        let mut sim = load();
        let (entity, _) = drive(&sim);
        match failure {
            0 => {
                let joint = sim.world().get::<Drive>(entity).unwrap().joints[0];
                assert!(
                    sim.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .remove_joint(joint)
                );
            }
            1 => sim.world_mut().get_mut::<Drive>(entity).unwrap().odometry = None,
            2 => {} // A repeated/nonadvancing acquisition is not an encoder sample.
            _ => {} // A due acquisition at the representable clock end cannot schedule a successor.
        }
        let now = if failure == 2 {
            Duration::ZERO
        } else if failure == 3 {
            Duration::MAX
        } else {
            Duration::from_millis(1)
        };
        advancing_time(&mut sim, now);
        sim.world_mut().run_system_once(observe).unwrap();
        let fault = sim.fault().unwrap().downcast_ref::<DriveFault>().unwrap();
        assert_eq!(fault.entity, entity);
        assert_eq!(
            fault.message,
            [
                "missing drive wheel state",
                "missing initial encoder reference",
                "encoder observation time must advance monotonically",
                "sensor sampling time overflow"
            ][failure]
        );
        assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
    }
    for missing_reference in [false, true] {
        let mut sim = load();
        let (entity, _) = drive(&sim);
        sim.world_mut().entity_mut(entity).insert(DiffDriveSample {
            time: Duration::from_millis(20),
            epoch: 0,
            pose: [1.0; 3],
            twist: [1.0; 2],
        });
        if missing_reference {
            sim.world_mut().get_mut::<Drive>(entity).unwrap().odometry = None;
        } else {
            let joint = sim.world().get::<Drive>(entity).unwrap().joints[1];
            assert!(
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove_joint(joint)
            );
        }
        sim.world_mut().run_system_once(reset).unwrap();
        let fault = sim.fault().unwrap().downcast_ref::<DriveFault>().unwrap();
        assert_eq!(fault.entity, entity);
        assert_eq!(
            fault.message,
            if missing_reference {
                "missing initial encoder reference"
            } else {
                "missing reset wheel state"
            }
        );
        assert!(sim.world().get::<DiffDriveSample>(entity).is_none());
        assert_eq!(sim.world().get::<Drive>(entity).unwrap().desired, [0.0; 2]);
    }
}

#[test]
fn loaded_driver_admission_requires_a_unique_planar_world_owned_rapier_model() {
    let load_xml = |xml: &str| {
        load_world_with_sdf_step(
            xml,
            "m8-domain.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
    };
    for pose in ["0 0 0.2 0.1 0 0", "0 0 0.2 0 0.1 0"] {
        let xml = XML.replace(
            "<pose>0 0 0.2 0 0 0</pose>",
            &format!("<pose>{pose}</pose>"),
        );
        assert!(matches!(
            load_xml(&xml),
            Err(LoadError::Unsupported {
                feature: "non-planar DiffDrive initial frame",
                ..
            })
        ));
    }
    let slow = XML.replace(
        "<odom_publish_frequency>50</odom_publish_frequency>",
        "<odom_publish_frequency>1e-320</odom_publish_frequency>",
    );
    assert!(matches!(
        load_xml(&slow),
        Err(LoadError::Unsupported {
            feature: "unrepresentable odometry rate",
            ..
        })
    ));
    let begin = XML.find("<model name=\"robot\"").unwrap();
    let end = begin + XML[begin..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[begin..end];
    let second = robot.replace("name=\"robot\"", "name=\"other\"");
    let duplicate = XML.replace("</world>", &format!("{second}</world>"));
    assert!(matches!(
        load_xml(&duplicate),
        Err(LoadError::Unsupported {
            feature: "multiple differential-drive systems",
            ..
        })
    ));
    let nested = XML.replace(robot, &format!("<model name='carrier'>{robot}</model>"));
    assert!(matches!(
        load_xml(&nested),
        Err(LoadError::Unsupported {
            feature: "DiffDrive requires a top-level dynamic Rapier model",
            ..
        })
    ));
    let conflict = XML.replace("</plugin>", "</plugin><plugin name='gz::sim::systems::ApplyJointForce' filename='gz-sim-apply-joint-force-system'><joint_name>left_wheel_joint</joint_name></plugin>");
    assert!(matches!(
        load_xml(&conflict),
        Err(LoadError::Unsupported {
            feature: "conflicting wheel actuator ownership",
            ..
        })
    ));
}
