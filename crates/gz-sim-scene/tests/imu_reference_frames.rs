// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::DQuat;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ImuPublication, ImuSample, LoadError, Name, PhysicsEngine, WorldPose, load_world_seeded,
    queue_link_world_velocity, remove_model, reset_loaded_world, set_imu_interest,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m43_imu_reference_frames.sdf");
const REFERENCES: [&str; 3] = ["ENU", "NED", "NWU"];
fn localized(name: &str) -> String {
    XML.replace(
        "<localization>NED</localization>",
        &format!("<localization>{name}</localization>"),
    )
}
fn try_load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        "m43.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [43; 32],
    )
}
fn load(xml: &str) -> Simulation {
    try_load(xml).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
fn sensor(sim: &Simulation) -> Entity {
    named(sim, "default::rig::body::imu").0
}
fn sample(sim: &Simulation) -> (Duration, u64, gz_sensors::ImuReading) {
    let sample = sim.world().get::<ImuSample>(sensor(sim)).unwrap();
    (sample.time, sample.epoch, *sample.reading.as_ref().unwrap())
}
fn drive(sim: &mut Simulation) {
    let id = named(sim, "default::rig::body").1;
    queue_link_world_velocity(sim, id, [1.0, -0.5, 0.2], [0.4, -0.3, 0.8]).unwrap();
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
fn expected_reference(name: &str) -> DQuat {
    // Independent axis-angle construction of the fixed ENU-source table.
    match name {
        "ENU" => DQuat::IDENTITY,
        "NED" => {
            DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2)
                * DQuat::from_rotation_x(std::f64::consts::PI)
        }
        "NWU" => DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        _ => panic!("test supplies only named frames"),
    }
}
fn named_geometry(sim: &Simulation, name: &str) {
    let rotation = DQuat::from_array(sim.world().get::<WorldPose>(sensor(sim)).unwrap().rotation);
    let expected = (expected_reference(name).inverse() * rotation).to_array();
    for (actual, expected) in sample(sim).2.orientation.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "{name}: {actual} vs {expected}"
        );
    }
}

#[test]
fn named_references_load_move_and_healthy_reset_with_stable_public_ids() {
    for name in REFERENCES {
        let mut sim = load(&localized(name));
        let old_sensor = sensor(&sim);
        let public = named(&sim, "default::rig::body::imu").1;
        let body = named(&sim, "default::rig::body").1;
        let initial_pose = sim
            .world()
            .get::<WorldPose>(named(&sim, "default::rig::body").0)
            .unwrap()
            .rotation;
        assert!(sim.world().get::<ImuSample>(old_sensor).is_none());
        drive(&mut sim);
        sim.step_exact(20).unwrap();
        named_geometry(&sim, name);
        let first = sample(&sim);
        assert_eq!(first.0, Duration::from_millis(20));
        assert_ne!(
            sim.world()
                .get::<WorldPose>(named(&sim, "default::rig::body").0)
                .unwrap()
                .rotation,
            initial_pose
        );
        assert!(first.2.angular_velocity.into_iter().any(|v| v.abs() > 0.1));
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let fresh_sensor = entity_by_id(sim.world(), public).unwrap();
        assert_eq!(fresh_sensor, sensor(&sim));
        assert_ne!(fresh_sensor, old_sensor);
        assert_eq!(named(&sim, "default::rig::body").1, body);
        assert!(sim.world().get::<ImuSample>(fresh_sensor).is_none());
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(sim.epoch(), first.1 + 1);
        drive(&mut sim);
        sim.step_exact(20).unwrap();
        named_geometry(&sim, name);
        let replay = sample(&sim);
        assert_eq!((replay.0, replay.2), (first.0, first.2));
        assert_eq!(replay.1, first.1 + 1);
        let meta = sim.world().get::<ImuPublication>(fresh_sensor).unwrap();
        assert_eq!(
            (meta.topic.as_str(), meta.frame_id.as_str()),
            ("/m43/imu", "rig/body/imu")
        );
    }
}

#[test]
fn custom_default_and_explicit_zero_reference_preserve_legacy_bits() {
    let explicit = localized("CUSTOM");
    let default = XML.replace("<imu><orientation_reference_frame><localization>NED</localization></orientation_reference_frame></imu>", "");
    let mut a = load(&explicit);
    let mut b = load(&default);
    // Private tests separately assert every supplied initial reference bit.
    for _ in 0..2 {
        drive(&mut a);
        drive(&mut b);
        a.step_exact(20).unwrap();
        b.step_exact(20).unwrap();
        let left = sample(&a);
        let right = sample(&b);
        assert_eq!(left, right);
        assert_eq!(
            left.2.orientation.map(f64::to_bits),
            right.2.orientation.map(f64::to_bits)
        );
        reset_loaded_world(&mut a, |_| ()).unwrap();
        reset_loaded_world(&mut b, |_| ()).unwrap();
    }
}

#[test]
fn named_references_change_only_orientation_and_preserve_sensor_axes_noise_and_covariances() {
    let noise = "<angular_velocity><x><noise type='gaussian'><mean>0.1</mean><stddev>0.02</stddev></noise></x></angular_velocity><linear_acceleration><z><noise type='gaussian'><mean>-0.2</mean><stddev>0.03</stddev></noise></z></linear_acceleration>";
    for name in REFERENCES {
        let mut named_sim = load(&localized(name).replace("</imu>", &format!("{noise}</imu>")));
        let mut custom = load(&localized("CUSTOM").replace("</imu>", &format!("{noise}</imu>")));
        drive(&mut named_sim);
        drive(&mut custom);
        for _ in 0..4 {
            named_sim.step_exact(10).unwrap();
            custom.step_exact(10).unwrap();
            let a = sample(&named_sim);
            let b = sample(&custom);
            assert_eq!((a.0, a.1), (b.0, b.1));
            same_motion(&a.2, &b.2);
            named_geometry(&named_sim, name);
            assert_ne!(a.2.orientation, b.2.orientation);
            let av = named_sim
                .world()
                .get::<WorldPose>(named(&named_sim, "default::rig::body").0)
                .unwrap();
            let bv = custom
                .world()
                .get::<WorldPose>(named(&custom, "default::rig::body").0)
                .unwrap();
            assert_eq!((av.translation, av.rotation), (bv.translation, bv.rotation));
        }
    }
}

#[test]
fn disabled_orientation_keeps_named_reference_gyro_and_specific_force() {
    for name in REFERENCES {
        let mut enabled = load(&localized(name));
        let mut disabled = load(&localized(name).replace(
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
        drive(&mut enabled);
        drive(&mut disabled);
        enabled.step_exact(20).unwrap();
        disabled.step_exact(20).unwrap();
        // Scene retains the measurement; the established encoder omits the field.
        assert_eq!(sample(&enabled), sample(&disabled));
        same_motion(&sample(&enabled).2, &sample(&disabled).2);
    }
}

#[test]
fn named_reference_demand_pause_rate_and_removal_preserve_acquisition_lifecycle() {
    let noise = "<angular_velocity><x><noise type='gaussian'><stddev>0.2</stddev></noise></x></angular_velocity>";
    for name in REFERENCES {
        let xml = localized(name).replace("</imu>", &format!("{noise}</imu>"));
        let mut sim = load(&xml);
        let mut peer = load(&xml);
        sim.step_exact(1).unwrap();
        peer.step_exact(1).unwrap();
        let first = sample(&sim);
        assert_eq!(first, sample(&peer));
        let target = sensor(&sim);
        set_imu_interest(&mut sim, []);
        sim.step_exact(49).unwrap();
        assert_eq!(sample(&sim), first);
        sim.set_paused(true);
        sim.tick().unwrap();
        assert_eq!(sim.elapsed(), Duration::from_millis(50));
        assert_eq!(sample(&sim), first);
        set_imu_interest(&mut sim, [target]);
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        peer.step_exact(9).unwrap();
        let resumed = sample(&sim);
        assert_eq!(resumed.0, Duration::from_millis(51));
        assert_eq!(
            resumed.2,
            sample(&peer).2,
            "no demand consumed no noise draws"
        );
        sim.step_exact(8).unwrap();
        assert_eq!(sample(&sim), resumed);
        sim.step_exact(1).unwrap();
        assert_eq!(sample(&sim).0, Duration::from_millis(60));
        let public = named(&sim, "default::rig::body::imu").1;
        let model = named(&sim, "default::rig").1;
        remove_model(&mut sim, model).unwrap();
        assert!(entity_by_id(sim.world(), public).is_none());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(entity_by_id(sim.world(), public).is_some());
        let restored = sensor(&sim);
        set_imu_interest(&mut sim, [restored]);
        sim.step_exact(1).unwrap();
        assert_eq!(sample(&sim).2, first.2);
    }
}

#[test]
fn named_spherical_and_custom_configuration_boundaries_remain_typed() {
    let spherical = "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>";
    for name in REFERENCES {
        let xml = localized(name).replace(
            "<gravity>0 0 0</gravity>",
            &format!("<gravity>0 0 0</gravity>{spherical}"),
        );
        assert!(matches!(
            try_load(&xml),
            Err(LoadError::Unsupported {
                feature: "IMU orientation configuration",
                ..
            })
        ));
    }
    let custom = localized("CUSTOM").replace(
        "<gravity>0 0 0</gravity>",
        &format!("<gravity>0 0 0</gravity>{spherical}"),
    );
    let mut legacy = load(&custom);
    legacy.step_exact(1).unwrap();
    let first = sample(&legacy);
    reset_loaded_world(&mut legacy, |_| ()).unwrap();
    legacy.step_exact(1).unwrap();
    assert_eq!(sample(&legacy).2, first.2);
    for xml in [
        localized("UNKNOWN"),
        localized("CUSTOM").replace(
            "</orientation_reference_frame>",
            "<custom_rpy>0 0 0.1</custom_rpy></orientation_reference_frame>",
        ),
        localized("CUSTOM").replace(
            "</orientation_reference_frame>",
            "<custom_rpy parent_frame='world'>0 0 0</custom_rpy></orientation_reference_frame>",
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
    let offset = localized("NED").replace(
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
