// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::DQuat;
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    ImuPublication, ImuSample, LoadError, Name, PhysicsEngine, SceneResetError, WorldPose,
    load_world_seeded, queue_link_world_wrench, remove_model, reset_loaded_world, set_imu_interest,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m48_imu_world_heading.sdf");
const SPHERICAL: &str = "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>45</heading_deg></spherical_coordinates>";
const REFERENCE: &str =
    "<orientation_reference_frame><localization>NED</localization></orientation_reference_frame>";
fn configured(name: &str, heading: Option<&str>, explicit: bool) -> String {
    let frame = if name == "CUSTOM" {
        format!(
            "<orientation_reference_frame>{}<custom_rpy parent_frame='world'>0.3 -0.4 0.5</custom_rpy></orientation_reference_frame>",
            if explicit {
                "<localization>CUSTOM</localization>"
            } else {
                ""
            }
        )
    } else {
        format!(
            "<orientation_reference_frame><localization>{name}</localization></orientation_reference_frame>"
        )
    };
    let xml = XML.replace(REFERENCE, &frame);
    match heading {
        Some(text) => xml.replace(
            "<heading_deg>45</heading_deg>",
            &format!("<heading_deg>{text}</heading_deg>"),
        ),
        None => xml.replace(SPHERICAL, ""),
    }
}
fn try_load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        "m48.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [48; 32],
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
    queue_link_world_wrench(sim, body, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0]).unwrap();
}
fn body_pose(sim: &Simulation) -> ([f64; 3], [f64; 4]) {
    let pose = sim
        .world()
        .get::<WorldPose>(named(sim, "default::rig::body").0)
        .unwrap();
    (pose.translation, pose.rotation)
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
fn reference(name: &str, heading: f64) -> DQuat {
    let table = match name {
        "ENU" => DQuat::IDENTITY,
        "NED" => {
            DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2)
                * DQuat::from_rotation_x(std::f64::consts::PI)
        }
        "NWU" => DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        "CUSTOM" => {
            DQuat::from_rotation_z(0.5) * DQuat::from_rotation_y(-0.4) * DQuat::from_rotation_x(0.3)
        }
        _ => panic!("bounded reference matrix"),
    };
    DQuat::from_rotation_z(heading) * table
}
fn geometry(sim: &Simulation, name: &str, heading: f64) {
    let world = DQuat::from_array(sim.world().get::<WorldPose>(sensor(sim)).unwrap().rotation);
    let expected = (reference(name, heading).inverse() * world).to_array();
    for (actual, expected) in sample(sim).2.orientation.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "{name}/{heading}: {actual} vs {expected}"
        );
    }
}

#[test]
fn four_heading_references_move_under_force_and_torque_and_replay_healthy_reset() {
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        for (text, heading) in [
            ("0", 0.0),
            ("-0", -0.0),
            ("45", std::f64::consts::FRAC_PI_4),
            ("-60", -std::f64::consts::FRAC_PI_3),
            ("180", std::f64::consts::PI),
            ("-180", -std::f64::consts::PI),
        ] {
            let mut sim = load(&configured(name, Some(text), true));
            let public = named(&sim, "default::rig::body::imu").1;
            let body = named(&sim, "default::rig::body").1;
            let initial = body_pose(&sim);
            assert!(sim.world().get::<ImuSample>(sensor(&sim)).is_none());
            drive(&mut sim);
            sim.step_exact(1).unwrap();
            assert!(sample(&sim).2.specific_force.iter().any(|v| v.abs() > 1.0));
            geometry(&sim, name, heading);
            sim.step_exact(19).unwrap();
            let first = sample(&sim);
            assert_eq!(first.0, Duration::from_millis(20));
            assert!(first.2.angular_velocity.iter().any(|v| v.abs() > 0.001));
            assert_ne!(body_pose(&sim).0, initial.0);
            assert_ne!(body_pose(&sim).1, initial.1);
            geometry(&sim, name, heading);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            let restored = entity_by_id(sim.world(), public).unwrap();
            assert_eq!(restored, sensor(&sim));
            assert_eq!(named(&sim, "default::rig::body").1, body);
            assert!(sim.world().get::<ImuSample>(restored).is_none());
            assert_eq!(body_pose(&sim), initial);
            assert_eq!(sim.elapsed(), Duration::ZERO);
            let meta = sim.world().get::<ImuPublication>(restored).unwrap();
            assert_eq!(
                (meta.topic.as_str(), meta.frame_id.as_str()),
                ("/m48/imu", "rig/body/imu")
            );
            drive(&mut sim);
            sim.step_exact(20).unwrap();
            assert_eq!((sample(&sim).0, sample(&sim).2), (first.0, first.2));
            assert_eq!(sample(&sim).1, first.1 + 1);
            geometry(&sim, name, heading);
        }
    }
}

#[test]
fn heading_changes_only_orientation_through_seeded_noise_and_disabled_publication() {
    let noise = "<angular_velocity><z><noise type='gaussian'><mean>-0.2</mean><stddev>0.03</stddev></noise></z></angular_velocity><linear_acceleration><x><noise type='gaussian'><mean>0.1</mean><stddev>0.02</stddev></noise></x></linear_acceleration>";
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        for noisy in [false, true] {
            let mut xml = configured(name, Some("45"), true);
            if noisy {
                xml = xml.replace("</imu>", &format!("{noise}</imu>"));
            }
            let mut heading = load(&xml);
            let mut baseline = load(&xml.replace(SPHERICAL, ""));
            let mut disabled = load(&xml.replace(
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
            for sim in [&mut heading, &mut baseline, &mut disabled] {
                drive(sim);
            }
            let mut previous_sample = None;
            let mut acquisition_pose = None;
            let mut fresh_stamps = Vec::new();
            for _ in 0..21 {
                for sim in [&mut heading, &mut baseline, &mut disabled] {
                    sim.step_exact(1).unwrap();
                }
                let current = sample(&heading);
                if current.0 == heading.elapsed() {
                    fresh_stamps.push(current.0);
                    acquisition_pose = Some(
                        heading
                            .world()
                            .get::<WorldPose>(sensor(&heading))
                            .unwrap()
                            .rotation,
                    );
                } else {
                    assert_eq!(Some(current), previous_sample);
                }
                let a = current.2;
                let b = sample(&baseline).2;
                same_motion(&a, &b);
                assert_eq!(sample(&heading), sample(&disabled));
                assert_eq!(body_pose(&heading), body_pose(&baseline));
                assert_eq!(body_pose(&heading), body_pose(&disabled));
                assert_eq!(
                    a.angular_velocity_covariance,
                    [0.0, 0.0, if noisy { 0.0009f32 as f64 } else { 0.0 }]
                );
                assert_eq!(
                    a.linear_acceleration_covariance,
                    [if noisy { 0.0004f32 as f64 } else { 0.0 }, 0.0, 0.0]
                );
                assert_ne!(a.orientation, b.orientation);
                let world = DQuat::from_array(acquisition_pose.unwrap());
                let expected =
                    (reference(name, std::f64::consts::FRAC_PI_4).inverse() * world).to_array();
                for (actual, expected) in a.orientation.into_iter().zip(expected) {
                    assert!(
                        (actual - expected).abs() < 1e-9,
                        "{name}: {actual} vs {expected} at {:?}",
                        current.0
                    );
                }
                previous_sample = Some(current);
            }
            assert_eq!(fresh_stamps, [1, 10, 20].map(Duration::from_millis));
        }
    }
}

#[test]
fn effective_custom_omission_and_legacy_initial_reference_keep_exact_prior_paths() {
    for heading in ["45", "-180"] {
        let mut explicit = load(&configured("CUSTOM", Some(heading), true));
        let mut omitted = load(&configured("CUSTOM", Some(heading), false));
        for sim in [&mut explicit, &mut omitted] {
            drive(sim);
            sim.step_exact(20).unwrap();
        }
        assert_eq!(sample(&explicit), sample(&omitted));
        assert_eq!(body_pose(&explicit), body_pose(&omitted));
    }
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        let mut absent = load(&configured(name, None, true));
        let mut zero = load(&configured(name, Some("0"), true));
        for sim in [&mut absent, &mut zero] {
            drive(sim);
            sim.step_exact(20).unwrap();
        }
        assert_eq!(
            sample(&absent).2.orientation.map(f64::to_bits),
            sample(&zero).2.orientation.map(f64::to_bits)
        );
        same_motion(&sample(&absent).2, &sample(&zero).2);
    }
    let legacy = XML.replace(REFERENCE, "");
    let mut world = load(&legacy);
    let mut absent = load(&legacy.replace(SPHERICAL, ""));
    for sim in [&mut world, &mut absent] {
        drive(sim);
        sim.step_exact(20).unwrap();
    }
    assert_eq!(sample(&world), sample(&absent));
    reset_loaded_world(&mut world, |_| ()).unwrap();
    drive(&mut world);
    world.step_exact(20).unwrap();
    assert_eq!(
        sample(&world).2.orientation.map(f64::to_bits),
        sample(&absent).2.orientation.map(f64::to_bits)
    );
}

#[test]
fn heading_demand_pause_equal_time_rate_removal_and_reset_preserve_noise_sequence() {
    for name in ["NED", "CUSTOM"] {
        let xml=configured(name,Some("-60"),false).replace("</imu>","<angular_velocity><x><noise type='gaussian'><stddev>0.2</stddev></noise></x></angular_velocity></imu>");
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
        sim.tick().unwrap();
        assert_eq!(sim.elapsed(), Duration::from_millis(50));
        assert_eq!(sample(&sim), first);
        let restored_interest = sensor(&sim);
        set_imu_interest(&mut sim, [restored_interest]);
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        peer.step_exact(9).unwrap();
        assert_eq!(sample(&sim).0, Duration::from_millis(51));
        assert_eq!(sample(&sim).2, sample(&peer).2);
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
}

#[test]
fn nonheading_spherical_metadata_and_neighboring_imu_domains_remain_closed() {
    for name in ["ENU", "NED", "NWU", "CUSTOM"] {
        let base = configured(name, Some("45"), true);
        for xml in [
            base.replace("EARTH_WGS84", "MOON_SCS"),
            base.replace(
                "<latitude_deg>0</latitude_deg>",
                "<latitude_deg>1</latitude_deg>",
            ),
            base.replace(
                "<longitude_deg>0</longitude_deg>",
                "<longitude_deg>-1</longitude_deg>",
            ),
            base.replace("<elevation>0</elevation>", "<elevation>1</elevation>"),
            base.replace(
                "<heading_deg>45</heading_deg>",
                "<heading_deg>181</heading_deg>",
            ),
            base.replace(
                "<heading_deg>45</heading_deg>",
                "<heading_deg>-181</heading_deg>",
            ),
            base.replace(
                "<heading_deg>45</heading_deg>",
                "<heading_deg>NaN</heading_deg>",
            ),
            base.replace(
                "<heading_deg>45</heading_deg>",
                "<heading_deg>inf</heading_deg>",
            ),
            base.replace(
                "<world_frame_orientation>ENU</world_frame_orientation>",
                "<world_frame_orientation>NED</world_frame_orientation>",
            ),
            base.replace(
                "<pose>0 0 0 0.1 -0.2 0.3</pose>",
                "<pose>0.1 0 0 0.1 -0.2 0.3</pose>",
            ),
        ] {
            assert!(try_load(&xml).is_err());
        }
    }
    let base = configured("CUSTOM", Some("45"), true);
    for parent in ["body", "World", ""] {
        assert!(
            try_load(&base.replace("parent_frame='world'", &format!("parent_frame='{parent}'")))
                .is_err()
        );
    }
    // New restrictions are consumer-only: legacy initial reference keeps old admission.
    let legacy = XML.replace(REFERENCE, "").replace(
        "<latitude_deg>0</latitude_deg>",
        "<latitude_deg>1</latitude_deg>",
    );
    let mut sim = load(&legacy);
    sim.step_exact(1).unwrap();
    assert!(sample(&sim).2.orientation.iter().all(|v| v.is_finite()));
}

#[test]
fn terminal_fault_refuses_heading_reset_without_mutating_clock_pose_or_sample() {
    let mut sim = load(XML);
    drive(&mut sim);
    sim.step_exact(20).unwrap();
    let public = named(&sim, "default::rig::body::imu").1;
    let before = sample(&sim);
    let pose = body_pose(&sim);
    let clock = (sim.elapsed(), sim.epoch(), sim.is_paused());
    let meta = sim
        .world()
        .get::<ImuPublication>(sensor(&sim))
        .unwrap()
        .clone();
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("M48 terminal reset refusal"));
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    let mut observed = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| observed = true),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!observed);
    assert!(sim.is_faulted());
    assert_eq!((sim.elapsed(), sim.epoch(), sim.is_paused()), clock);
    assert_eq!(entity_by_id(sim.world(), public), Some(sensor(&sim)));
    assert_eq!(sample(&sim), before);
    assert_eq!(body_pose(&sim), pose);
    let actual = sim.world().get::<ImuPublication>(sensor(&sim)).unwrap();
    assert_eq!(
        (
            actual.enable_orientation,
            actual.entity_name.as_str(),
            actual.frame_id.as_str(),
            actual.topic.as_str()
        ),
        (
            meta.enable_orientation,
            meta.entity_name.as_str(),
            meta.frame_id.as_str(),
            meta.topic.as_str()
        )
    );
}
