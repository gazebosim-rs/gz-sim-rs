// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::DQuat;
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    ImuPublication, ImuSample, LoadError, Name, PhysicsEngine, SceneResetError, WorldPose,
    load_world_seeded, queue_link_world_velocity, queue_link_world_wrench, remove_model,
    reset_loaded_world, set_imu_interest,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m45_imu_custom_world.sdf");
const REFERENCE: &str = "<orientation_reference_frame><localization>CUSTOM</localization><custom_rpy parent_frame=\"world\">0.3 -0.4 0.5</custom_rpy></orientation_reference_frame>";
fn configured(angles: &str, explicit: bool) -> String {
    let localization = if explicit {
        "<localization>CUSTOM</localization>"
    } else {
        ""
    };
    XML.replace(REFERENCE, &format!("<orientation_reference_frame>{localization}<custom_rpy parent_frame='world'>{angles}</custom_rpy></orientation_reference_frame>"))
}
fn try_load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        "m45.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [45; 32],
    )
}
fn load(xml: &str) -> Simulation {
    try_load(xml).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let row = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (row.id(), row.get::<EntityId>().unwrap().get())
}
fn sensor(sim: &Simulation) -> Entity {
    named(sim, "default::rig::body::imu").0
}
fn sample(sim: &Simulation) -> (Duration, u64, gz_sensors::ImuReading) {
    let row = sim.world().get::<ImuSample>(sensor(sim)).unwrap();
    (row.time, row.epoch, *row.reading.as_ref().unwrap())
}
fn drive(sim: &mut Simulation) {
    let body = named(sim, "default::rig::body").1;
    queue_link_world_velocity(sim, body, [1.5, -0.5, 0.25], [0.4, -0.2, 0.7]).unwrap();
    // Velocity replacement rebases IMU history. A real force supplies acceleration.
    queue_link_world_wrench(sim, body, [8.0, -4.0, 2.0], [0.0; 3]).unwrap();
}
fn same_motion(a: &gz_sensors::ImuReading, b: &gz_sensors::ImuReading) {
    assert_eq!(
        a.specific_force.map(f64::to_bits),
        b.specific_force.map(f64::to_bits)
    );
    assert_eq!(
        a.angular_velocity.map(f64::to_bits),
        b.angular_velocity.map(f64::to_bits)
    );
    assert_eq!(a.angular_velocity_covariance, b.angular_velocity_covariance);
    assert_eq!(
        a.linear_acceleration_covariance,
        b.linear_acceleration_covariance
    );
}
fn geometry(sim: &Simulation, angles: [f64; 3]) {
    let reference = DQuat::from_rotation_z(angles[2])
        * DQuat::from_rotation_y(angles[1])
        * DQuat::from_rotation_x(angles[0]);
    let world = DQuat::from_array(sim.world().get::<WorldPose>(sensor(sim)).unwrap().rotation);
    let expected = (reference.inverse() * world).to_array();
    for (actual, expected) in sample(sim).2.orientation.into_iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-9, "{actual} vs {expected}");
    }
}

#[test]
fn effective_custom_world_loads_moves_and_healthy_resets_actual_configuration() {
    for (text, angles) in [
        ("0 0 0", [0.0; 3]),
        ("0.3 -0.4 0.5", [0.3, -0.4, 0.5]),
        ("0.4 0 0", [0.4, 0.0, 0.0]),
        ("0 -0.3 0", [0.0, -0.3, 0.0]),
        ("0 0 0.5", [0.0, 0.0, 0.5]),
    ] {
        for explicit in [false, true] {
            let mut sim = load(&configured(text, explicit));
            let public = named(&sim, "default::rig::body::imu").1;
            let body = named(&sim, "default::rig::body").1;
            let first_pose = *sim
                .world()
                .get::<WorldPose>(named(&sim, "default::rig::body").0)
                .unwrap();
            assert!(sim.world().get::<ImuSample>(sensor(&sim)).is_none());
            drive(&mut sim);
            sim.step_exact(1).unwrap();
            assert!(sample(&sim).2.specific_force.iter().any(|v| v.abs() > 1.0));
            geometry(&sim, angles);
            sim.step_exact(19).unwrap();
            let first = sample(&sim);
            assert_eq!(first.0, Duration::from_millis(20));
            geometry(&sim, angles);
            let current = sim
                .world()
                .get::<WorldPose>(named(&sim, "default::rig::body").0)
                .unwrap();
            assert_ne!(current.rotation, first_pose.rotation);
            assert!(first.2.angular_velocity.iter().any(|v| v.abs() > 0.1));
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            let restored = entity_by_id(sim.world(), public).unwrap();
            assert_eq!(restored, sensor(&sim));
            assert_eq!(named(&sim, "default::rig::body").1, body);
            assert!(sim.world().get::<ImuSample>(restored).is_none());
            assert_eq!(sim.elapsed(), Duration::ZERO);
            let meta = sim.world().get::<ImuPublication>(restored).unwrap();
            assert_eq!(
                (meta.topic.as_str(), meta.frame_id.as_str()),
                ("/m45/imu", "rig/body/imu")
            );
            drive(&mut sim);
            sim.step_exact(20).unwrap();
            let replay = sample(&sim);
            assert_eq!((replay.0, replay.2), (first.0, first.2));
            assert_eq!(replay.1, first.1 + 1);
            geometry(&sim, angles);
        }
    }
}

#[test]
fn explicit_world_zero_is_not_legacy_initial_and_omitted_localization_matches() {
    let mut explicit = load(&configured("0 0 0", true));
    let mut omitted = load(&configured("0 0 0", false));
    let mut signed = load(&configured("-0 0 -0", false));
    let legacy_xml = XML.replace(REFERENCE, "");
    let mut legacy = load(&legacy_xml);
    let mut legacy_explicit = load(&XML.replace(REFERENCE, "<orientation_reference_frame><localization>CUSTOM</localization></orientation_reference_frame>"));
    for sim in [
        &mut explicit,
        &mut omitted,
        &mut signed,
        &mut legacy,
        &mut legacy_explicit,
    ] {
        sim.step_exact(1).unwrap();
    }
    assert_eq!(sample(&explicit), sample(&omitted));
    assert_eq!(sample(&legacy), sample(&legacy_explicit));
    assert_eq!(
        sample(&legacy).2.orientation.map(f64::to_bits),
        sample(&legacy_explicit).2.orientation.map(f64::to_bits)
    );
    assert_ne!(
        sample(&explicit).2.orientation,
        sample(&legacy).2.orientation
    );
    geometry(&explicit, [0.0; 3]);
    geometry(&signed, [-0.0, 0.0, -0.0]);
    same_motion(&sample(&explicit).2, &sample(&legacy).2);
}

#[test]
fn custom_world_preserves_same_seed_physics_motion_noise_covariances_and_disabled_reading() {
    let noise = "<angular_velocity><z><noise type='gaussian'><mean>-0.2</mean><stddev>0.03</stddev></noise></z></angular_velocity><linear_acceleration><x><noise type='gaussian'><mean>0.1</mean><stddev>0.02</stddev></noise></x></linear_acceleration>";
    for noisy in [false, true] {
        let mut world = configured("0.3 -0.4 0.5", true);
        if noisy {
            world = world.replace("</imu>", &format!("{noise}</imu>"));
        }
        let mut selected = load(&world);
        let mut omitted = load(&world.replace("<localization>CUSTOM</localization>", ""));
        let mut legacy = load(&world.replace(
            "<custom_rpy parent_frame='world'>0.3 -0.4 0.5</custom_rpy>",
            "",
        ));
        let mut disabled = load(&world.replace(
            "<imu>",
            "<imu><enable_orientation>false</enable_orientation>",
        ));
        assert!(
            !disabled
                .world()
                .get::<ImuPublication>(sensor(&disabled))
                .unwrap()
                .enable_orientation
        );
        for sim in [&mut selected, &mut omitted, &mut legacy, &mut disabled] {
            drive(sim);
        }
        for index in 1..=30 {
            for sim in [&mut selected, &mut omitted, &mut legacy, &mut disabled] {
                sim.step_exact(1).unwrap();
            }
            let a = sample(&selected);
            assert_eq!(a, sample(&omitted));
            assert_eq!(
                a,
                sample(&disabled),
                "encoder omission does not erase the Scene reading"
            );
            same_motion(&a.2, &sample(&legacy).2);
            let pose = *selected
                .world()
                .get::<WorldPose>(named(&selected, "default::rig::body").0)
                .unwrap();
            for peer in [&omitted, &legacy, &disabled] {
                let actual = peer
                    .world()
                    .get::<WorldPose>(named(peer, "default::rig::body").0)
                    .unwrap();
                assert_eq!(
                    (actual.translation, actual.rotation),
                    (pose.translation, pose.rotation)
                );
            }
            assert_eq!(
                a.2.angular_velocity_covariance,
                [0.0, 0.0, if noisy { 0.0009f32 as f64 } else { 0.0 }]
            );
            assert_eq!(
                a.2.linear_acceleration_covariance,
                [if noisy { 0.0004f32 as f64 } else { 0.0 }, 0.0, 0.0]
            );
            if index == 1 {
                assert!(a.2.specific_force.iter().any(|v| v.abs() > 1.0));
            }
            if index % 10 == 0 {
                geometry(&selected, [0.3, -0.4, 0.5]);
            }
        }
    }
}

#[test]
fn custom_world_demand_pause_cadence_removal_and_reset_preserve_acquisition() {
    let xml = configured("0.3 -0.4 0.5", false).replace("</imu>", "<angular_velocity><x><noise type='gaussian'><stddev>0.2</stddev></noise></x></angular_velocity></imu>");
    let mut sim = load(&xml);
    let mut peer = load(&xml);
    sim.step_exact(1).unwrap();
    peer.step_exact(1).unwrap();
    let first = sample(&sim);
    assert_eq!(first, sample(&peer));
    set_imu_interest(&mut sim, []);
    sim.step_exact(49).unwrap();
    assert_eq!(sample(&sim), first);
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(sim.elapsed(), Duration::from_millis(50));
    assert_eq!(sample(&sim), first);
    let target = sensor(&sim);
    set_imu_interest(&mut sim, [target]);
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    peer.step_exact(9).unwrap();
    assert_eq!(sample(&sim).0, Duration::from_millis(51));
    assert_eq!(
        sample(&sim).2,
        sample(&peer).2,
        "no demand consumes no noise draws"
    );
    let held = sample(&sim);
    sim.step_exact(8).unwrap();
    assert_eq!(sample(&sim), held);
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim).0, Duration::from_millis(60));
    let public = named(&sim, "default::rig::body::imu").1;
    let model = named(&sim, "default::rig").1;
    remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), public).is_none());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), public).unwrap();
    assert_eq!(restored, sensor(&sim));
    assert!(sim.world().get::<ImuSample>(restored).is_none());
    set_imu_interest(&mut sim, [restored]);
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim).2, first.2);
}

#[test]
fn custom_world_preserves_typed_parent_geodesy_and_sensor_domain_refusals() {
    let spherical = "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>";
    for explicit in [false, true] {
        let xml = configured("0.3 -0.4 0.5", explicit).replace(
            "<gravity>0 0 0</gravity>",
            &format!("<gravity>0 0 0</gravity>{spherical}"),
        );
        // This exact former heading-zero spherical refusal is now a physical
        // load/motion/healthy-reset positive, with every neighboring refusal kept.
        let mut sim = load(&xml);
        let public = named(&sim, "default::rig::body::imu").1;
        let body = named(&sim, "default::rig::body").1;
        let initial = *sim
            .world()
            .get::<WorldPose>(named(&sim, "default::rig::body").0)
            .unwrap();
        queue_link_world_wrench(&mut sim, body, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0]).unwrap();
        sim.step_exact(1).unwrap();
        assert!(sample(&sim).2.specific_force.iter().any(|v| v.abs() > 1.0));
        sim.step_exact(19).unwrap();
        let first = sample(&sim);
        assert!(first.2.angular_velocity.iter().any(|v| v.abs() > 0.001));
        let pose = sim
            .world()
            .get::<WorldPose>(named(&sim, "default::rig::body").0)
            .unwrap();
        assert_ne!(pose.rotation, initial.rotation);
        geometry(&sim, [0.3, -0.4, 0.5]);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(entity_by_id(sim.world(), public), Some(sensor(&sim)));
        assert_eq!(named(&sim, "default::rig::body").1, body);
        assert!(sim.world().get::<ImuSample>(sensor(&sim)).is_none());
        queue_link_world_wrench(&mut sim, body, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0]).unwrap();
        sim.step_exact(20).unwrap();
        assert_eq!((sample(&sim).0, sample(&sim).2), (first.0, first.2));
        assert_eq!(sample(&sim).1, first.1 + 1);
        geometry(&sim, [0.3, -0.4, 0.5]);
        let geodesy = xml.replace(
            "<latitude_deg>0</latitude_deg>",
            "<latitude_deg>1</latitude_deg>",
        );
        assert!(matches!(
            try_load(&geodesy),
            Err(LoadError::Unsupported {
                feature: "IMU orientation configuration",
                ..
            })
        ));
    }
    let legacy = XML.replace(REFERENCE, "").replace(
        "<gravity>0 0 0</gravity>",
        &format!("<gravity>0 0 0</gravity>{spherical}"),
    );
    let mut legacy = load(&legacy);
    legacy.step_exact(1).unwrap();
    let old = sample(&legacy);
    reset_loaded_world(&mut legacy, |_| ()).unwrap();
    legacy.step_exact(1).unwrap();
    assert_eq!(sample(&legacy).2, old.2);
    let base = configured("0.3 -0.4 0.5", true);
    for xml in [
        base.replace("parent_frame='world'", "parent_frame=''"),
        base.replace("parent_frame='world'", "parent_frame='body'"),
        base.replace("parent_frame='world'", "parent_frame='World'"),
        base.replace(
            "<localization>CUSTOM</localization>",
            "<localization>UNKNOWN</localization>",
        ),
        base.replace(
            "<localization>CUSTOM</localization>",
            "<localization>NED</localization>",
        ),
    ] {
        assert!(matches!(
            try_load(&xml),
            Err(LoadError::Unsupported {
                feature: "IMU orientation configuration",
                ..
            })
        ));
    }
    for angles in ["nan 0 0", "inf 0 0", "0 0", "0 bad 0"] {
        assert!(try_load(&configured(angles, true)).is_err());
    }
    let offset = base.replace(
        "<pose>0 0 0 0.1 -0.2 0.3</pose>",
        "<pose>0.1 0 0 0.1 -0.2 0.3</pose>",
    );
    assert!(matches!(
        try_load(&offset),
        Err(LoadError::Unsupported {
            feature: "offset IMU position",
            ..
        })
    ));
}

#[test]
fn terminal_fault_refuses_custom_world_reset_before_observers_or_scene_mutation() {
    let mut sim = load(XML);
    drive(&mut sim);
    sim.step_exact(20).unwrap();
    let public = named(&sim, "default::rig::body::imu").1;
    let before = sample(&sim);
    let clock = (sim.epoch(), sim.elapsed(), sim.is_paused());
    let pose = *sim.world().get::<WorldPose>(sensor(&sim)).unwrap();
    let publication = sim
        .world()
        .get::<ImuPublication>(sensor(&sim))
        .unwrap()
        .clone();
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other(
            "reference reset must preserve a terminal clock fault",
        ));
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    let mut observed = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| observed = true),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!observed);
    assert!(sim.is_faulted());
    assert_eq!((sim.epoch(), sim.elapsed(), sim.is_paused()), clock);
    assert_eq!(entity_by_id(sim.world(), public), Some(sensor(&sim)));
    assert_eq!(sample(&sim), before);
    let actual = sim.world().get::<WorldPose>(sensor(&sim)).unwrap();
    assert_eq!(
        (actual.translation, actual.rotation),
        (pose.translation, pose.rotation)
    );
    let actual = sim.world().get::<ImuPublication>(sensor(&sim)).unwrap();
    assert_eq!(
        (
            actual.enable_orientation,
            actual.entity_name.as_str(),
            actual.frame_id.as_str(),
            actual.topic.as_str()
        ),
        (
            publication.enable_orientation,
            publication.entity_name.as_str(),
            publication.frame_id.as_str(),
            publication.topic.as_str()
        )
    );
}
