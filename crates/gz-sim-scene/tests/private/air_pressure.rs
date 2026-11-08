// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;

const XML: &str = include_str!("../fixtures/m15_air_pressure.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m15-air_pressure.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("admitted air_pressure fixture")
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .expect("fixture entity");
    (
        entity.id(),
        entity.get::<EntityId>().expect("public identity").get(),
    )
}
fn sensor(sim: &Simulation) -> Entity {
    named(sim, "default::rig::body::air_pressure").0
}
fn without_sensor() -> String {
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}

#[test]
fn pressure_noise_ownership_and_typed_parameter_boundaries_are_explicit() {
    let sim = load(XML);
    assert!(
        sim.world()
            .get::<AirPressure>(sensor(&sim))
            .unwrap()
            .noise
            .is_none()
    );
    for kind in [
        sdformat::NoiseKind::Gaussian,
        sdformat::NoiseKind::GaussianQuantized,
    ] {
        let config = sdformat::Noise {
            kind,
            ..Default::default()
        };
        assert!(PressureNoise::new(&config, [42; 32], 1).unwrap().is_some());
        for field in 0..7 {
            let mut invalid = config;
            match field {
                0 => invalid.mean = f64::NAN,
                1 => invalid.stddev = f64::INFINITY,
                2 => invalid.bias_mean = f64::NAN,
                3 => invalid.bias_stddev = f64::INFINITY,
                4 => invalid.dynamic_bias_stddev = f64::NAN,
                5 => invalid.dynamic_bias_correlation_time = f64::INFINITY,
                _ => invalid.precision = f64::NAN,
            }
            assert!(PressureNoise::new(&invalid, [42; 32], 1).is_err());
        }
        for field in 0..4 {
            let mut invalid = config;
            match field {
                0 => invalid.stddev = -1.0,
                1 => invalid.bias_stddev = -1.0,
                2 => invalid.dynamic_bias_stddev = -1.0,
                _ => invalid.precision = -1.0,
            }
            assert!(PressureNoise::new(&invalid, [42; 32], 1).is_err());
        }
    }
}

#[test]
fn invalid_ideal_acquisition_preserves_the_noise_stream_and_reports_the_ideal_cause() {
    fn invalid_pose(mut sensors: Query<&mut WorldPose, With<AirPressure>>) {
        for mut pose in &mut sensors {
            pose.translation[2] = f64::NAN;
        }
    }
    let configured = XML.replace("<air_pressure/>", "<air_pressure><pressure><noise type='gaussian'><stddev>2</stddev><bias_stddev>1</bias_stddev></noise></pressure></air_pressure>");
    let mut sim = load(&configured);
    let entity = sensor(&sim);
    let mut before = sim
        .world()
        .get::<AirPressure>(entity)
        .unwrap()
        .noise
        .clone()
        .unwrap();
    sim.add_systems(Phase::PostStep, invalid_pose.before(sample));
    sim.step_exact(1).unwrap();
    let acquired = sim.world().get::<AirPressureSample>(entity).unwrap();
    assert_eq!(acquired.time, Duration::from_millis(1));
    assert_eq!(
        acquired.reading,
        Err(AirPressureSampleError::Ideal(InvalidAirPressureInput))
    );
    let mut after = sim
        .world()
        .get::<AirPressure>(entity)
        .unwrap()
        .noise
        .clone()
        .unwrap();
    let ideal = gz_sensors::air_pressure::air_pressure(AirPressureInput { world_z: 3.5 }).unwrap();
    assert_eq!(before.apply(ideal), after.apply(ideal));
}

#[test]
fn absent_air_pressure_owns_no_callbacks_marker_interest_or_samples() {
    let mut absent = load(&without_sensor());
    let present = load(XML);
    let counts = absent.scheduled_system_counts();
    assert_eq!(
        present.scheduled_system_counts(),
        (counts.0 + 1, counts.1 + 1)
    );
    assert!(!absent.world().contains_resource::<Installed>());
    set_air_pressure_interest(&mut absent, []);
    assert!(!absent.world().contains_resource::<Interest>());
    absent.step_exact(3).expect("advancing absence");
    absent.reset().expect("reset absence");
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert!(!absent.world().contains_resource::<Installed>());
    assert!(!absent.world().contains_resource::<Interest>());
}

#[test]
fn absolute_rotated_sensor_world_height_drives_pressure_not_link_or_initial_height() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    let (link, link_id) = named(&sim, "default::rig::body");
    let metadata = sim
        .world()
        .get::<AirPressurePublication>(entity)
        .expect("metadata");
    assert_eq!(
        (
            metadata.entity_name.as_str(),
            metadata.frame_id.as_str(),
            metadata.topic.as_str()
        ),
        (
            "rig::body::air_pressure",
            "rig/body/air_pressure",
            "/m15/air_pressure"
        )
    );
    queue_link_world_velocity(&mut sim, link_id, [0.0, 0.0, 3.0], [0.0, 2.0, 0.0])
        .expect("finite offset motion");
    sim.step_exact(1).expect("real Rapier advance");
    let world = sim.world();
    let pose = world.get::<WorldPose>(link).expect("link pose");
    let [x, y, z, w] = pose.rotation;
    // Independent scalar R*[1,0,.5]; sensor orientation does not rotate gravity.
    let offset_z = 2.0 * (x * z - y * w) + 0.5 * (1.0 - 2.0 * (x * x + y * y));
    let expected_z = pose.translation[2] + offset_z;
    let sample = world
        .get::<AirPressureSample>(entity)
        .expect("fresh sample");
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    let reading = sample.reading.as_ref().expect("physical atmosphere");
    let at =
        |world_z| gz_sensors::air_pressure::air_pressure(AirPressureInput { world_z }).unwrap();
    assert!((reading.pressure - at(expected_z).pressure).abs() < 1e-9);
    assert!((reading.pressure - at(pose.translation[2]).pressure).abs() > 1.0);
    assert!((reading.pressure - at(expected_z - 3.5).pressure).abs() > 30.0);
    assert_eq!(reading.variance, 0.0);
}

#[test]
fn demand_fixed_cadence_and_pause_do_not_advance_unrequested_measurements() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    set_air_pressure_interest(&mut sim, []);
    sim.step_exact(20).expect("no demand advance");
    assert!(sim.world().get::<AirPressureSample>(entity).is_none());
    assert!(
        sim.world()
            .get::<AirPressure>(entity)
            .expect("sensor")
            .schedule
            .is_due(sim.elapsed())
    );
    set_air_pressure_interest(&mut sim, [entity, entity]);
    assert_eq!(sim.world().resource::<Interest>().0.len(), 1);
    sim.step_exact(1).expect("first demanded acquisition");
    let first = sim
        .world()
        .get::<AirPressureSample>(entity)
        .expect("due sample")
        .time;
    assert_eq!(first, Duration::from_millis(21));
    sim.tick().expect("paused poll");
    assert_eq!(
        sim.world()
            .get::<AirPressureSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(8).expect("before phase-aligned deadline");
    assert_eq!(
        sim.world()
            .get::<AirPressureSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(1).expect("phase-aligned acquisition");
    assert_eq!(
        sim.world()
            .get::<AirPressureSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(30)
    );
    set_air_pressure_interest(&mut sim, []);
    assert!(sim.world().get::<AirPressureSample>(entity).is_none());
    clear_air_pressure_interest(&mut sim);
    sim.step_exact(10).expect("local manual demand");
    assert_eq!(
        sim.world()
            .get::<AirPressureSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(40)
    );
}

#[test]
fn last_model_removal_retires_callbacks_and_loaded_reset_restores_absolute_height() {
    let mut sim = load(XML);
    let (entity, identity) = named(&sim, "default::rig::body::air_pressure");
    let model = named(&sim, "default::rig").1;
    let counts = sim.scheduled_system_counts();
    sim.step_exact(12).expect("sample before removal");
    remove_model(&mut sim, model).expect("remove owned model");
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_air_pressure_interest(&mut sim, [entity]);
    assert!(!sim.world().contains_resource::<Interest>());
    initial::reset_loaded_world(&mut sim, |sim| {
        let restored = entity_by_id(sim.world(), identity).expect("same public identity");
        assert!(sim.world().get::<AirPressureSample>(restored).is_none());
        assert_eq!(
            sim.world()
                .get::<WorldPose>(restored)
                .expect("reset pose")
                .translation[2],
            3.5
        );
    })
    .expect("loaded scene reconstruction");
    assert_eq!(sim.scheduled_system_counts(), counts);
    let restored = entity_by_id(sim.world(), identity).expect("restored sensor");
    sim.step_exact(1).expect("new epoch acquisition");
    let sample = sim
        .world()
        .get::<AirPressureSample>(restored)
        .expect("reset sample");
    assert_eq!((sample.epoch, sample.time), (1, Duration::from_millis(1)));
    assert_eq!(
        sample.reading.as_ref().expect("reading").pressure,
        gz_sensors::air_pressure::air_pressure(AirPressureInput { world_z: 3.5 })
            .unwrap()
            .pressure
    );
}

#[test]
fn admission_refuses_nondefault_noise_metadata_second_sensor_and_other_engines() {
    let original = "<air_pressure/>";
    for replacement in [
        "<air_pressure><pressure><noise type='gaussian'><precision>-0.1</precision></noise></pressure></air_pressure>",
        "<air_pressure><pressure><noise type='none'><mean>1</mean></noise></pressure></air_pressure>",
        "<air_pressure><reference_altitude>1</reference_altitude></air_pressure>",
        "<air_pressure/><visualize>true</visualize>",
        "<air_pressure/><enable_metrics>true</enable_metrics>",
        "<air_pressure/><alwaysOn>true</alwaysOn>",
        "<air_pressure/><plugin name='custom' filename='custom'/>",
    ] {
        let xml = XML.replace(original, replacement);
        sdformat::load_str(&xml, "air_pressure-parsed-negative", &NoResources)
            .expect("negative remains a parsed typed sensor, not an XML rejection");
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "air_pressure-refusal",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            ),
            Err(LoadError::Unsupported { .. })
        ));
    }
    let tiny = XML.replace(
        "<update_rate>100</update_rate>",
        "<update_rate>1e-300</update_rate>",
    );
    sdformat::load_str(&tiny, "air_pressure-tiny-rate", &NoResources)
        .expect("finite positive SDF rate");
    assert!(matches!(
        load_world_with_sdf_step(
            &tiny,
            "air_pressure-tiny-rate",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "unrepresentable sensor rate",
            ..
        })
    ));
    let twice = XML.replace(
        "</sensor>",
        "</sensor><sensor name='second' type='air_pressure'><air_pressure/></sensor>",
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &twice,
            "two-air_pressures",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "multiple air_pressures",
            ..
        })
    ));
    assert!(matches!(
        load_world_with_sdf_step(
            XML,
            "air_pressure-engine",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "Rapier ideal air_pressure",
            ..
        })
    ));
}

#[test]
fn omitted_topic_and_frame_use_native_names_and_canonical_frame_wins_over_legacy() {
    let plain = XML
        .replace("<topic>/m15/air_pressure</topic>", "")
        .replace("<frame_id>rig/body/air_pressure</frame_id>", "");
    let sim = load(&plain);
    let metadata = sim
        .world()
        .get::<AirPressurePublication>(sensor(&sim))
        .expect("default metadata");
    assert_eq!(
        metadata.topic,
        "/world/default/model/rig/link/body/sensor/air_pressure/air_pressure"
    );
    assert_eq!(metadata.frame_id, "rig::body::air_pressure");
    let legacy = plain.replace(
        "<air_pressure/>",
        "<gz_frame_id>legacy</gz_frame_id><air_pressure/>",
    );
    let sim = load(&legacy);
    assert_eq!(
        sim.world()
            .get::<AirPressurePublication>(sensor(&sim))
            .expect("legacy metadata")
            .frame_id,
        "legacy"
    );
    let both = XML.replace(
        "<air_pressure/>",
        "<gz_frame_id>legacy</gz_frame_id><air_pressure/>",
    );
    let sim = load(&both);
    assert_eq!(
        sim.world()
            .get::<AirPressurePublication>(sensor(&sim))
            .expect("canonical metadata")
            .frame_id,
        "rig/body/air_pressure"
    );
}

#[test]
fn omitted_config_explicit_zero_reference_and_continuous_rate_are_admitted_but_atmosphere_is_not() {
    for config in [
        "",
        "<air_pressure><reference_altitude>0</reference_altitude></air_pressure>",
    ] {
        let mut sim = load(&XML.replace("<air_pressure/>", config).replace(
            "<update_rate>100</update_rate>",
            "<update_rate>0</update_rate>",
        ));
        let entity = sensor(&sim);
        sim.step_exact(2).expect("continuous acquisition");
        assert_eq!(
            sim.world()
                .get::<AirPressureSample>(entity)
                .expect("sample")
                .time,
            Duration::from_millis(2)
        );
    }
    let explicit_atmosphere = XML.replace("<gravity>", "<atmosphere type='adiabatic'/><gravity>");
    assert!(matches!(
        load_world_with_sdf_step(
            &explicit_atmosphere,
            "explicit-atmosphere",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "world atmosphere",
            ..
        })
    ));
    let outside = XML.replace("<pose>0 0 3 0 0 0</pose>", "<pose>0 0 1000000 0 0 0</pose>");
    assert!(matches!(
        load_world_with_sdf_step(
            &outside,
            "atmosphere-domain",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "air_pressure atmosphere domain",
            ..
        })
    ));
}

#[test]
fn invalid_physical_measurement_remains_a_typed_acquisition_error_not_a_fabricated_pressure() {
    fn invalid_pose(mut sensors: Query<&mut WorldPose, With<AirPressure>>) {
        for mut pose in &mut sensors {
            pose.translation[2] = 1e6;
        }
    }
    let mut sim = load(XML);
    let entity = sensor(&sim);
    // Inject a nonphysical state at acquisition, after committed attached poses.
    sim.add_systems(Phase::PostStep, invalid_pose.before(sample));
    sim.step_exact(1)
        .expect("measurement error is observable, not a schedule failure");
    let sample = sim
        .world()
        .get::<AirPressureSample>(entity)
        .expect("typed acquisition outcome");
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    assert_eq!(
        sample.reading,
        Err(AirPressureSampleError::Ideal(InvalidAirPressureInput))
    );
    assert!(!sim.is_faulted());
    assert!(sim.fault().is_none());
    assert_eq!(sim.elapsed(), Duration::from_millis(1));
    sim.reset()
        .expect("invalid sample does not poison loaded reset");
    assert!(sim.world().get::<AirPressureSample>(entity).is_none());
}
