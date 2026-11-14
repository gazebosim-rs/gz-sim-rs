// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    BodyHandle, GroundTruthOdometryDimensions as Dimensions,
    GroundTruthOdometryPublication as Publication, GroundTruthOdometrySample as Sample, InputError,
    JointHandle, LoadError, Name, PhysicsEngine, SceneResetError, WorldPose,
    initial_ground_truth_odometry_publications, load_world_seeded, queue_link_world_wrench,
    remove_model, reset_loaded_world,
};
use std::time::Duration;

const ARTICULATED: &str = include_str!("fixtures/m31_pose_static_publisher.sdf");
type SampleBits = ([u64; 3], [u64; 4], [u64; 3], [u64; 3]);
fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn plugin(index: usize, dimensions: usize, rate: &str, sigma: &str, offsets: bool) -> String {
    let offsets = if offsets {
        "<xyz_offset>0.4 -0.3 0.2</xyz_offset><xyz_global_offset>-0.7 0.5 -0.6</xyz_global_offset><rpy_offset>0.3 -0.4 0.5</rpy_offset><rpy_global_offset>-0.2 0.25 -0.35</rpy_global_offset>"
    } else {
        ""
    };
    format!(
        "<plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'><dimensions>{dimensions}</dimensions><odom_publish_frequency>{rate}</odom_publish_frequency><gaussian_noise>{sigma}</gaussian_noise><odom_frame>rig{index}/odom</odom_frame><robot_base_frame>rig{index}/base</robot_base_frame><odom_topic>/m56/{index}/odom</odom_topic><odom_covariance_topic>/m56/{index}/cov</odom_covariance_topic><tf_topic>/m56/{index}/tf</tf_topic>{offsets}</plugin>"
    )
}
fn free_model(index: usize, fixed: bool, same_pose: bool, config: &str) -> String {
    let y = if same_pose { 2 } else { 2 + index * 4 };
    format!(
        "<model name='rig{index}'><static>{fixed}</static><pose>1 {y} 3 0 0 0.3</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='shape'><geometry><box><size>.2 .2 .2</size></box></geometry></collision></link>{config}</model>"
    )
}
fn articulated(index: usize, dimensions: usize, rate: &str) -> String {
    let start = ARTICULATED.find("  <model ").unwrap();
    let end = ARTICULATED.rfind("</model>").unwrap() + "</model>".len();
    let source = &ARTICULATED[start..end];
    let plugin_start = source.find("    <plugin").unwrap();
    let plugin_end =
        source[plugin_start..].find("</plugin>").unwrap() + plugin_start + "</plugin>".len();
    let model = format!(
        "{}{}{}",
        &source[..plugin_start],
        plugin(index, dimensions, rate, "0", true),
        &source[plugin_end..]
    );
    // Rename only the model attribute; frame/topic strings are already owner-specific.
    model
        .replace("name=\"rig\"", &format!("name=\"rig{index}\""))
        .replace(
            "<pose>1 2 3 0 0 0.6</pose>",
            &format!("<pose>1 {} 3 0 0 0.6</pose>", 2 + index * 4),
        )
}
fn try_load(source: &str, seed: [u8; 32]) -> Result<Simulation, LoadError> {
    load_world_seeded(
        source,
        "m56-multiple-odometry-publishers.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        seed,
    )
}
fn load(source: &str) -> Simulation {
    try_load(source, [56; 32]).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let node = sim
        .world()
        .iter_entities()
        .find(|node| node.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (node.id(), node.get::<EntityId>().unwrap().get())
}
fn owner(sim: &Simulation, index: usize) -> (Entity, u64) {
    named(sim, &format!("default::rig{index}"))
}
fn sample(sim: &Simulation, index: usize) -> Option<Sample> {
    sim.world().get::<Sample>(owner(sim, index).0).copied()
}
fn bits(value: Sample) -> SampleBits {
    (
        value.pose.translation.map(f64::to_bits),
        value.pose.rotation.map(f64::to_bits),
        value.linear.map(f64::to_bits),
        value.angular.map(f64::to_bits),
    )
}
fn snapshot(value: Sample) -> (Duration, u64, u64, SampleBits) {
    (value.time, value.epoch, value.revision, bits(value))
}
fn rotation(rpy: [f64; 3]) -> DQuat {
    DQuat::from_rotation_z(rpy[2]) * DQuat::from_rotation_y(rpy[1]) * DQuat::from_rotation_x(rpy[0])
}
fn geometry(sim: &Simulation, index: usize) {
    let entity = owner(sim, index).0;
    let metadata = sim.world().get::<Publication>(entity).unwrap();
    let input = sim.world().get::<WorldPose>(entity).unwrap();
    let actual = sample(sim, index).unwrap();
    assert_eq!(actual.time, sim.elapsed());
    let offsets = metadata.offsets;
    // Independent point/basis geometry at the retained live bound. This is not
    // a filtered velocity estimator or a new native scalar-bit oracle.
    let inverse_global = rotation(offsets.global_rotation_rpy()).inverse();
    let point = inverse_global
        * (DVec3::from_array(input.translation)
            + DQuat::from_array(input.rotation) * DVec3::from_array(offsets.local_translation())
            - DVec3::from_array(offsets.global_translation()));
    let orientation =
        inverse_global * DQuat::from_array(input.rotation) * rotation(offsets.local_rotation_rpy());
    for (axis, expected) in point.to_array().into_iter().enumerate() {
        if axis == 2 && metadata.dimensions == Dimensions::Two {
            assert_eq!(actual.pose.translation[axis], 0.0);
        } else {
            assert!((actual.pose.translation[axis] - expected).abs() < 1e-12);
        }
    }
    for (actual, expected) in actual.pose.rotation.into_iter().zip(orientation.to_array()) {
        assert!((actual - expected).abs() < 1e-12);
    }
    assert!((DQuat::from_array(actual.pose.rotation).length_squared() - 1.0).abs() < 1e-12);
    if metadata.dimensions == Dimensions::Two {
        assert_eq!(
            (actual.linear[2], actual.angular[0], actual.angular[1]),
            (0.0, 0.0, 0.0)
        );
    }
}

#[test]
fn distinct_owners_real_force_torque_and_joint_motion_preserve_fresh_offset_geometry_and_reset() {
    let source = world(&(articulated(0, 2, "100") + &articulated(1, 3, "50")));
    let mut sim = load(&source);
    let owners = [owner(&sim, 0), owner(&sim, 1)];
    let tips = [
        named(&sim, "default::rig0::tip"),
        named(&sim, "default::rig1::tip"),
    ];
    let initial = tips.map(|(entity, _)| *sim.world().get::<WorldPose>(entity).unwrap());
    let mut joints = Vec::new();
    for (index, (entity, id)) in tips.iter().enumerate() {
        // Keep the narrow Scene input domain explicit for this two-link model.
        assert!(
            matches!(queue_link_world_wrench(&mut sim, *id, [2.0, 3.0, 0.0], [0.0, 0.0, 0.5]), Err(InputError::NotFreeLink(actual)) if actual == *id)
        );
        let body = sim.world().get::<BodyHandle>(*entity).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(body, [2.0, 3.0, 0.0], [0.0, 0.0, 0.5 + index as f64])
            .unwrap();
        let joint = named(&sim, &format!("default::rig{index}::tip_hinge")).0;
        let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_motor(
                handle,
                Some(JointVelocityMotor {
                    target_velocity: 0.5,
                    max_force: 0.5,
                }),
            )
            .unwrap();
        joints.push(handle);
    }
    let mut held = [None, None];
    let mut stamps = [Vec::new(), Vec::new()];
    for _ in 0..80 {
        sim.step_exact(1).unwrap();
        for (index, old) in held.iter_mut().enumerate() {
            let value = sample(&sim, index);
            if value.is_some_and(|reading| reading.time == sim.elapsed()) {
                geometry(&sim, index);
                stamps[index].push(sim.elapsed());
            } else {
                assert_eq!(value.map(snapshot), *old);
            }
            *old = value.map(snapshot);
        }
    }
    assert_eq!(
        stamps[0],
        (1..=8)
            .map(|n| Duration::from_millis(n * 10))
            .collect::<Vec<_>>()
    );
    assert_eq!(stamps[1], [20, 40, 60, 80].map(Duration::from_millis));
    for (index, joint) in joints.into_iter().enumerate() {
        let state = sim
            .world()
            .resource::<PhysicsBackend>()
            .joint_state(joint)
            .unwrap();
        assert!(state.position.abs() > 0.0 && state.velocity.abs() > 0.0);
        let moved = sim.world().get::<WorldPose>(tips[index].0).unwrap();
        assert_ne!(moved.translation, initial[index].translation);
        assert_ne!(moved.rotation, initial[index].rotation);
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let mut fresh = load(&source);
    for (old, id) in owners {
        let restored = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(restored, old);
        assert!(sim.world().get::<Sample>(restored).is_none());
    }
    sim.step_exact(40).unwrap();
    fresh.step_exact(40).unwrap();
    for index in 0..2 {
        let a = sample(&sim, index).unwrap();
        let b = sample(&fresh, index).unwrap();
        assert_eq!((a.time, a.revision, bits(a)), (b.time, b.revision, bits(b)));
        assert_eq!((a.epoch, b.epoch), (1, 0));
    }
}

#[test]
fn absent_single_pair_and_eight_keep_one_family_schedule_and_single_owner_values() {
    let mut absent = load(&world(""));
    absent.step_exact(2).unwrap();
    assert_eq!(
        initial_ground_truth_odometry_publications(&absent)
            .unwrap()
            .count(),
        0
    );
    assert!(
        !absent
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Publication>() || entity.contains::<Sample>())
    );
    for count in [1, 2, 8] {
        let plain = (0..count)
            .map(|index| free_model(index, true, false, ""))
            .collect::<String>();
        let baseline = load(&world(&plain)).scheduled_system_counts();
        let models = (0..count)
            .map(|index| {
                free_model(
                    index,
                    true,
                    false,
                    &plugin(
                        index,
                        if index.is_multiple_of(2) { 2 } else { 3 },
                        "0",
                        "0",
                        false,
                    ),
                )
            })
            .collect::<String>();
        let mut sim = load(&world(&models));
        assert_eq!(
            sim.scheduled_system_counts(),
            (baseline.0 + 1, baseline.1 + 1)
        );
        assert_eq!(
            initial_ground_truth_odometry_publications(&sim)
                .unwrap()
                .count(),
            count
        );
        sim.step_exact(1).unwrap();
        assert!((0..count).all(|index| sample(&sim, index).is_none()));
        sim.step_exact(1).unwrap();
        for index in 0..count {
            geometry(&sim, index);
        }
    }
    let first = free_model(0, false, false, &plugin(0, 3, "0", "0", false));
    let second = free_model(1, false, false, &plugin(1, 2, "50", "0", true));
    let mut single = load(&world(&first));
    let mut pair = load(&world(&(first + &second)));
    assert_eq!(owner(&single, 0).1, owner(&pair, 0).1);
    for sim in [&mut single, &mut pair] {
        let body = named(sim, "default::rig0::body").1;
        queue_link_world_wrench(sim, body, [3.0, 2.0, 1.0], [0.0, 0.0, 0.5]).unwrap();
        sim.step_exact(20).unwrap();
    }
    assert_eq!(
        sample(&single, 0).map(snapshot),
        sample(&pair, 0).map(snapshot)
    );
}

#[test]
fn no_listener_pause_and_loaded_reset_preserve_rates_while_core_reset_retains_history() {
    let source = world(
        &(free_model(0, true, false, &plugin(0, 2, "100", "0", false))
            + &free_model(1, true, false, &plugin(1, 3, "50", "0", false))),
    );
    let mut sim = load(&source);
    // No transport or subscriber interest exists in this Scene-only test.
    sim.step_exact(40).unwrap();
    let before = [sample(&sim, 0).map(snapshot), sample(&sim, 1).map(snapshot)];
    for _ in 0..3 {
        sim.tick().unwrap();
    }
    sim.step_exact(0).unwrap();
    assert_eq!(
        [sample(&sim, 0).map(snapshot), sample(&sim, 1).map(snapshot)],
        before
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(9).unwrap();
    assert!(sample(&sim, 0).is_none() && sample(&sim, 1).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim, 0).unwrap().time, Duration::from_millis(10));
    assert!(sample(&sim, 1).is_none());
    sim.step_exact(10).unwrap();
    assert_eq!(sample(&sim, 1).unwrap().time, Duration::from_millis(20));
    sim.reset().unwrap();
    assert!(sample(&sim, 0).is_none() && sample(&sim, 1).is_none());
    // Bare reset clears samples but does not replace previous_time=20ms.
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    let clock = (sim.elapsed(), sim.epoch());
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert_eq!((sim.elapsed(), sim.epoch()), clock);
}

#[test]
fn identical_inputs_noise_isolation_and_same_id_appended_peer_replay_survive_loaded_reset() {
    let first = free_model(0, true, true, &plugin(0, 3, "0", "0.05", false));
    let second = free_model(1, true, true, &plugin(1, 3, "0", "0.05", false));
    let source = world(&(first.clone() + &second));
    let mut single = load(&world(&first));
    let mut pair = load(&source);
    assert_eq!(owner(&single, 0).1, owner(&pair, 0).1);
    assert_ne!(owner(&pair, 0).1, owner(&pair, 1).1);
    for _ in 0..20 {
        single.step_exact(1).unwrap();
        pair.step_exact(1).unwrap();
        let a = pair.world().get::<WorldPose>(owner(&pair, 0).0).unwrap();
        let b = pair.world().get::<WorldPose>(owner(&pair, 1).0).unwrap();
        assert_eq!(
            a.translation.map(f64::to_bits),
            b.translation.map(f64::to_bits)
        );
        assert_eq!(a.rotation.map(f64::to_bits), b.rotation.map(f64::to_bits));
        assert_eq!(
            sample(&single, 0).map(snapshot),
            sample(&pair, 0).map(snapshot)
        );
    }
    let a = sample(&pair, 0).unwrap();
    let b = sample(&pair, 1).unwrap();
    assert_eq!(
        (a.time, a.pose.translation, a.pose.rotation),
        (b.time, b.pose.translation, b.pose.rotation)
    );
    assert_ne!(
        (a.linear.map(f64::to_bits), a.angular.map(f64::to_bits)),
        (b.linear.map(f64::to_bits), b.angular.map(f64::to_bits))
    );
    let ids = [owner(&pair, 0).1, owner(&pair, 1).1];
    reset_loaded_world(&mut pair, |_| ()).unwrap();
    let mut fresh = load(&source);
    pair.step_exact(20).unwrap();
    fresh.step_exact(20).unwrap();
    for (index, id) in ids.into_iter().enumerate() {
        assert_eq!(owner(&pair, index).1, id);
        let a = sample(&pair, index).unwrap();
        let b = sample(&fresh, index).unwrap();
        assert_eq!((a.time, a.revision, bits(a)), (b.time, b.revision, bits(b)));
        assert_eq!((a.epoch, b.epoch), (1, 0));
    }
}

#[test]
fn removal_retires_only_the_last_owner_and_metadata_fault_remains_global_terminal() {
    let source = world(
        &(free_model(0, true, false, &plugin(0, 2, "0", "0", false))
            + &free_model(1, true, false, &plugin(1, 3, "0", "0", true))),
    );
    let plain = world(&(free_model(0, true, false, "") + &free_model(1, true, false, "")));
    let baseline = load(&plain).scheduled_system_counts();
    let mut sim = load(&source);
    let initial = initial_ground_truth_odometry_publications(&sim)
        .unwrap()
        .map(|(id, config)| (id, config.clone()))
        .collect::<Vec<_>>();
    let originals = [owner(&sim, 0), owner(&sim, 1)];
    sim.step_exact(2).unwrap();
    remove_model(&mut sim, originals[0].1).unwrap();
    assert!(entity_by_id(sim.world(), originals[0].1).is_none());
    sim.step_exact(1).unwrap();
    geometry(&sim, 1);
    assert_eq!(
        initial_ground_truth_odometry_publications(&sim)
            .unwrap()
            .map(|(id, config)| (id, config.clone()))
            .collect::<Vec<_>>(),
        initial
    );
    remove_model(&mut sim, originals[1].1).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|entity| entity.contains::<Publication>() || entity.contains::<Sample>())
    );
    // The family's two callbacks retire; shared physics/input callbacks retain
    // the same baseline as the ordinary loaded world without this publication.
    assert_eq!(sim.scheduled_system_counts(), baseline);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        (baseline.0 + 1, baseline.1 + 1)
    );
    for (old, id) in originals {
        let entity = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(entity, old);
        assert!(sim.world().get::<Sample>(entity).is_none());
    }
    sim.step_exact(2).unwrap();
    let victim = owner(&sim, 1).0;
    sim.world_mut()
        .get_mut::<Publication>(victim)
        .unwrap()
        .robot_base_frame
        .push_str("changed");
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    assert!(sim.world().get::<Sample>(victim).is_none());
    assert!(sim.world().resource::<StepFault>().error().is_some());
    let clock = (sim.elapsed(), sim.epoch());
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert_eq!((sim.elapsed(), sim.epoch()), clock);
}

#[test]
fn ninth_owner_late_invalid_metadata_and_existing_unrelated_refusals_stay_closed() {
    let model = |index| free_model(index, true, false, &plugin(index, 2, "50", "0", false));
    let eight = (0..8).map(model).collect::<String>();
    let ninth = world(&(eight.clone() + &model(8)));
    assert!(matches!(
        try_load(&ninth, [56; 32]),
        Err(LoadError::Unsupported {
            feature: "multiple ground-truth odometry models",
            ..
        })
    ));
    for invalid in [
        model(7).replace("<dimensions>2</dimensions>", "<dimensions>4</dimensions>"),
        model(7).replace(
            "<gaussian_noise>0</gaussian_noise>",
            "<gaussian_noise>NaN</gaussian_noise>",
        ),
        model(7).replace(
            "<odom_frame>rig7/odom</odom_frame>",
            "<odom_frame> </odom_frame>",
        ),
        model(7).replace("</plugin>", "<xyz_offset>0 NaN 0</xyz_offset></plugin>"),
        model(7).replace(
            "</model>",
            &format!("{}</model>", plugin(7, 2, "50", "0", false)),
        ),
    ] {
        let prefix = (0..7).map(model).collect::<String>();
        assert!(matches!(
            try_load(&world(&(prefix + &invalid)), [56; 32]),
            Err(LoadError::Unsupported { .. })
        ));
    }
    let nested = world(&format!("<model name='parent'>{}</model>", model(0)));
    assert!(try_load(&nested, [56; 32]).is_err());
    assert!(
        load_world_seeded(
            &world(&model(0)),
            "m56-tpe.sdf",
            "default",
            Some(Duration::from_millis(1)),
            &sdformat::NoResources,
            PhysicsEngine::Tpe,
            [56; 32]
        )
        .is_err()
    );
}
