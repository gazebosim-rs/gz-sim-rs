// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;

const XML: &str = include_str!("../fixtures/m17_magnetometer.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m17-magnetometer.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("admitted magnetometer fixture")
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
    named(sim, "default::rig::body::magnetometer").0
}
fn without_sensor() -> String {
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}

#[test]
fn absent_magnetometer_owns_no_callbacks_marker_interest_or_samples() {
    let mut absent = load(&without_sensor().replace(
        "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
        "",
    ));
    let present = load(XML);
    let counts = absent.scheduled_system_counts();
    assert_eq!(
        present.scheduled_system_counts(),
        (counts.0 + 1, counts.1 + 1)
    );
    assert!(!absent.world().contains_resource::<Installed>());
    set_magnetometer_interest(&mut absent, []);
    assert!(!absent.world().contains_resource::<Interest>());
    absent.step_exact(3).expect("advancing absence");
    absent.reset().expect("reset absence");
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert!(!absent.world().contains_resource::<Installed>());
    assert!(!absent.world().contains_resource::<Interest>());
}

// Scalar transpose of the sensor-to-world rotation; independent of the model.
fn expected(q: [f64; 4], field: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let [a, b, c] = field;
    [
        (1.0 - 2.0 * (y * y + z * z)) * a + 2.0 * (x * y + z * w) * b + 2.0 * (x * z - y * w) * c,
        2.0 * (x * y - z * w) * a + (1.0 - 2.0 * (x * x + z * z)) * b + 2.0 * (y * z + x * w) * c,
        2.0 * (x * z + y * w) * a + 2.0 * (y * z - x * w) * b + (1.0 - 2.0 * (x * x + y * y)) * c,
    ]
}

#[test]
fn actual_resolved_link_and_sensor_rotation_drive_field_with_translation_invariance() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    let (link, link_id) = named(&sim, "default::rig::body");
    let metadata = sim.world().get::<MagnetometerPublication>(entity).unwrap();
    assert_eq!(
        (
            metadata.entity_name.as_str(),
            metadata.frame_id.as_str(),
            metadata.topic.as_str()
        ),
        (
            "rig::body::magnetometer",
            "rig/body/magnetometer",
            "/m17/magnetometer"
        )
    );
    let initial = sim.world().get::<WorldPose>(entity).unwrap().rotation;
    queue_link_world_velocity(&mut sim, link_id, [2.0, -3.0, 4.0], [0.0; 3]).unwrap();
    sim.step_exact(1).unwrap();
    let reading = sim
        .world()
        .get::<MagnetometerSample>(entity)
        .unwrap()
        .reading
        .as_ref()
        .unwrap()
        .field_tesla;
    let pose = sim.world().get::<WorldPose>(entity).unwrap();
    for (a, b) in pose.rotation.into_iter().zip(initial) {
        assert!((a - b).abs() < 1e-14);
    }
    let field = [2e-5, -3e-5, 4e-5];
    for (a, b) in reading.into_iter().zip(expected(pose.rotation, field)) {
        assert!((a - b).abs() < 1e-18);
    }
    let link_only = expected(sim.world().get::<WorldPose>(link).unwrap().rotation, field);
    assert!(
        reading
            .into_iter()
            .zip(link_only)
            .any(|(a, b)| (a - b).abs() > 1e-6)
    );
    sim.step_exact(9).unwrap();
    let later = sim.world().get::<MagnetometerSample>(entity).unwrap();
    assert_eq!(later.time, Duration::from_millis(10));
    for (a, b) in later
        .reading
        .as_ref()
        .unwrap()
        .field_tesla
        .into_iter()
        .zip(reading)
    {
        assert!((a - b).abs() < 1e-18);
    }
    queue_link_world_velocity(&mut sim, link_id, [0.0; 3], [0.0, 2.0, 0.0]).unwrap();
    sim.step_exact(10).unwrap();
    let moved = sim.world().get::<MagnetometerSample>(entity).unwrap();
    let actual = moved.reading.as_ref().unwrap().field_tesla;
    assert_eq!((moved.epoch, moved.time), (0, Duration::from_millis(20)));
    for (a, b) in actual.into_iter().zip(expected(
        sim.world().get::<WorldPose>(entity).unwrap().rotation,
        field,
    )) {
        assert!((a - b).abs() < 1e-18);
    }
    assert!(
        actual
            .into_iter()
            .zip(reading)
            .any(|(a, b)| (a - b).abs() > 1e-8)
    );
}

#[test]
fn demand_fixed_cadence_and_pause_do_not_advance_unrequested_measurements() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    set_magnetometer_interest(&mut sim, []);
    sim.step_exact(20).expect("no demand advance");
    assert!(sim.world().get::<MagnetometerSample>(entity).is_none());
    assert!(
        sim.world()
            .get::<Magnetometer>(entity)
            .expect("sensor")
            .schedule
            .is_due(sim.elapsed())
    );
    set_magnetometer_interest(&mut sim, [entity, entity]);
    assert_eq!(sim.world().resource::<Interest>().0.len(), 1);
    sim.step_exact(1).expect("first demanded acquisition");
    let first = sim
        .world()
        .get::<MagnetometerSample>(entity)
        .expect("due sample")
        .time;
    assert_eq!(first, Duration::from_millis(21));
    sim.tick().expect("paused poll");
    assert_eq!(
        sim.world()
            .get::<MagnetometerSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(8).expect("before phase-aligned deadline");
    assert_eq!(
        sim.world()
            .get::<MagnetometerSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(1).expect("phase-aligned acquisition");
    assert_eq!(
        sim.world()
            .get::<MagnetometerSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(30)
    );
    set_magnetometer_interest(&mut sim, []);
    assert!(sim.world().get::<MagnetometerSample>(entity).is_none());
    clear_magnetometer_interest(&mut sim);
    sim.step_exact(10).expect("local manual demand");
    assert_eq!(
        sim.world()
            .get::<MagnetometerSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(40)
    );
}

#[test]
fn last_model_removal_retires_callbacks_and_loaded_reset_restores_original_field() {
    let mut sim = load(XML);
    let (entity, identity) = named(&sim, "default::rig::body::magnetometer");
    let model = named(&sim, "default::rig").1;
    let original_rotation = sim.world().get::<WorldPose>(entity).unwrap().rotation;
    let original_position = sim.world().get::<WorldPose>(entity).unwrap().translation;
    let counts = sim.scheduled_system_counts();
    sim.step_exact(12).expect("sample before removal");
    remove_model(&mut sim, model).expect("remove owned model");
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_magnetometer_interest(&mut sim, [entity]);
    assert!(!sim.world().contains_resource::<Interest>());
    initial::reset_loaded_world(&mut sim, |sim| {
        let restored = entity_by_id(sim.world(), identity).expect("same public identity");
        assert!(sim.world().get::<MagnetometerSample>(restored).is_none());
        assert_eq!(
            sim.world()
                .get::<WorldPose>(restored)
                .expect("reset pose")
                .translation,
            original_position
        );
    })
    .expect("loaded scene reconstruction");
    assert_eq!(sim.scheduled_system_counts(), counts);
    let restored = entity_by_id(sim.world(), identity).expect("restored sensor");
    sim.step_exact(1).expect("new epoch acquisition");
    let sample = sim
        .world()
        .get::<MagnetometerSample>(restored)
        .expect("reset sample");
    assert_eq!((sample.epoch, sample.time), (1, Duration::from_millis(1)));
    assert_eq!(
        sample.reading.as_ref().expect("reading").field_tesla,
        gz_sensors::magnetometer::magnetometer(MagnetometerInput {
            world_orientation: original_rotation,
            field_world: [2e-5, -3e-5, 4e-5]
        })
        .unwrap()
        .field_tesla
    );
}

#[test]
fn admission_refuses_invalid_noise_metadata_second_sensor_and_other_engines() {
    let original = "<magnetometer/>";
    for replacement in [
        "<magnetometer><x><noise type='gaussian'><precision>-1</precision></noise></x></magnetometer>",
        "<magnetometer><y><noise type='none'><mean>1</mean></noise></y></magnetometer>",
        "<magnetometer><z><noise type='none'><bias_mean>1</bias_mean></noise></z></magnetometer>",
        "<magnetometer/><visualize>true</visualize>",
        "<magnetometer/><enable_metrics>true</enable_metrics>",
        "<magnetometer/><alwaysOn>true</alwaysOn>",
        "<magnetometer/><plugin name='custom' filename='custom'/>",
    ] {
        let xml = XML.replace(original, replacement);
        sdformat::load_str(&xml, "magnetometer-parsed-negative", &NoResources)
            .expect("negative remains a parsed typed sensor, not an XML rejection");
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "magnetometer-refusal",
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
    sdformat::load_str(&tiny, "magnetometer-tiny-rate", &NoResources)
        .expect("finite positive SDF rate");
    assert!(matches!(
        load_world_with_sdf_step(
            &tiny,
            "magnetometer-tiny-rate",
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
        "</sensor><sensor name='second' type='magnetometer'><magnetometer/></sensor>",
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &twice,
            "two-magnetometers",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "multiple magnetometers",
            ..
        })
    ));
    assert!(matches!(
        load_world_with_sdf_step(
            XML,
            "magnetometer-engine",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "Rapier ideal magnetometer",
            ..
        })
    ));
}

#[test]
fn omitted_topic_and_frame_use_native_names_and_canonical_frame_wins_over_legacy() {
    let plain = XML
        .replace("<topic>/m17/magnetometer</topic>", "")
        .replace("<frame_id>rig/body/magnetometer</frame_id>", "");
    let sim = load(&plain);
    let metadata = sim
        .world()
        .get::<MagnetometerPublication>(sensor(&sim))
        .expect("default metadata");
    assert_eq!(
        metadata.topic,
        "/world/default/model/rig/link/body/sensor/magnetometer/magnetometer"
    );
    assert_eq!(metadata.frame_id, "rig::body::magnetometer");
    let legacy = plain.replace(
        "<magnetometer/>",
        "<gz_frame_id>legacy</gz_frame_id><magnetometer/>",
    );
    let sim = load(&legacy);
    assert_eq!(
        sim.world()
            .get::<MagnetometerPublication>(sensor(&sim))
            .expect("legacy metadata")
            .frame_id,
        "legacy"
    );
    let both = XML.replace(
        "<magnetometer/>",
        "<gz_frame_id>legacy</gz_frame_id><magnetometer/>",
    );
    let sim = load(&both);
    assert_eq!(
        sim.world()
            .get::<MagnetometerPublication>(sensor(&sim))
            .expect("canonical metadata")
            .frame_id,
        "rig/body/magnetometer"
    );
}

#[test]
fn default_explicit_zero_and_continuous_field_controls_preserve_inert_world_refusals() {
    let field_xml = "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>";
    for (text, field) in [
        ("", [5.5645e-6, 22.8758e-6, -42.3884e-6]),
        ("<magnetic_field>0 0 0</magnetic_field>", [0.0; 3]),
    ] {
        let xml = XML
            .replace(field_xml, text)
            .replace("<magnetometer/>", "")
            .replace(
                "<update_rate>100</update_rate>",
                "<update_rate>0</update_rate>",
            );
        let mut sim = load(&xml);
        let entity = sensor(&sim);
        sim.step_exact(2).unwrap();
        let sample = sim.world().get::<MagnetometerSample>(entity).unwrap();
        assert_eq!(sample.time, Duration::from_millis(2));
        for (a, b) in sample
            .reading
            .as_ref()
            .unwrap()
            .field_tesla
            .into_iter()
            .zip(expected(
                sim.world().get::<WorldPose>(entity).unwrap().rotation,
                field,
            ))
        {
            assert!((a - b).abs() < 1e-18);
        }
    }
    let absent = without_sensor();
    assert!(matches!(
        load_world_with_sdf_step(
            &absent,
            "unconsumed-field",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "world magnetic field",
            ..
        })
    ));
    let spherical=XML.replace("<gravity>","<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates><gravity>");
    assert!(
        load_world_with_sdf_step(
            &spherical,
            "geographic-refused",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
}

#[test]
fn preflight_validates_all_consumer_properties_before_authorizing_world_field() {
    let sdf = sdformat::load_str(XML, "magnetometer-preflight", &NoResources).unwrap();
    assert!(preflight(&sdf, &sdf.worlds[0], PhysicsEngine::Rapier).unwrap());
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        // Frame graph and typed config are unchanged; mutate just the public field.
        let mut parsed = sdformat::load_str(XML, "bad-field", &NoResources).unwrap();
        parsed.worlds[0].magnetic_field[1] = bad;
        assert!(preflight(&parsed, &parsed.worlds[0], PhysicsEngine::Rapier).is_err());
    }
    let mut parsed = sdformat::load_str(XML, "wrong-type", &NoResources).unwrap();
    parsed.worlds[0].models[0].links[0].sensors[0].kind = "air_pressure".into();
    assert!(preflight(&parsed, &parsed.worlds[0], PhysicsEngine::Rapier).is_err());
    let mut parsed = sdformat::load_str(XML, "missing-config", &NoResources).unwrap();
    parsed.worlds[0].models[0].links[0].sensors[0].config = sdformat::SensorConfig::Unconfigured;
    assert!(preflight(&parsed, &parsed.worlds[0], PhysicsEngine::Rapier).is_err());
    for rate in [f64::NAN, f64::INFINITY, 1e-300] {
        let mut parsed = sdformat::load_str(XML, "bad-rate", &NoResources).unwrap();
        parsed.worlds[0].models[0].links[0].sensors[0].update_rate = rate;
        assert!(preflight(&parsed, &parsed.worlds[0], PhysicsEngine::Rapier).is_err());
    }
}

#[test]
fn invalid_measurement_remains_typed_and_reset_clears_it_without_faulting_clock() {
    fn invalid_pose(mut sensors: Query<&mut WorldPose, With<Magnetometer>>) {
        for mut pose in &mut sensors {
            pose.rotation = [0.0; 4];
        }
    }
    let mut sim = load(XML);
    let entity = sensor(&sim);
    sim.add_systems(Phase::PostStep, invalid_pose.before(sample));
    sim.step_exact(1)
        .expect("typed bad reading is not a deadline fault");
    let sample = sim.world().get::<MagnetometerSample>(entity).unwrap();
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    assert!(matches!(
        sample.reading,
        Err(MagnetometerSampleError::Ideal(InvalidMagnetometerInput))
    ));
    assert!(!sim.is_faulted());
    assert!(sim.fault().is_none());
    sim.reset().unwrap();
    assert!(sim.world().get::<MagnetometerSample>(entity).is_none());
}

#[test]
fn magnetic_noise_domain_is_validated_before_authorizing_a_nondefault_world_field() {
    assert!(
        MagnetometerNoise::new(&sdformat::Magnetometer::default(), [0; 32], 1)
            .unwrap()
            .is_none()
    );
    for axis in 0..3 {
        for field in 0..7 {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut parsed =
                    sdformat::load_str(XML, "invalid-magnetic-noise", &NoResources).unwrap();
                let sdformat::SensorConfig::Magnetometer(config) =
                    &mut parsed.worlds[0].models[0].links[0].sensors[0].config
                else {
                    panic!("typed fixture")
                };
                let noise = &mut config.noise[axis];
                noise.kind = sdformat::NoiseKind::Gaussian;
                match field {
                    0 => noise.mean = value,
                    1 => noise.stddev = value,
                    2 => noise.bias_mean = value,
                    3 => noise.bias_stddev = value,
                    4 => noise.dynamic_bias_stddev = value,
                    5 => noise.dynamic_bias_correlation_time = value,
                    _ => noise.precision = value,
                }
                assert!(MagnetometerNoise::new(config, [0; 32], 1).is_err());
                assert!(matches!(
                    preflight(&parsed, &parsed.worlds[0], PhysicsEngine::Rapier),
                    Err(LoadError::Unsupported {
                        feature: "unrepresentable magnetometer noise",
                        ..
                    })
                ));
            }
        }
        for kind in [
            sdformat::NoiseKind::Gaussian,
            sdformat::NoiseKind::GaussianQuantized,
        ] {
            let mut config = sdformat::Magnetometer::default();
            config.noise[axis].kind = kind;
            let state = MagnetometerNoise::new(&config, [0; 32], 1)
                .unwrap()
                .unwrap();
            assert!(state.channels[axis].is_some());
            assert_eq!(state.channels.iter().filter(|n| n.is_some()).count(), 1);
            for field in 0..4 {
                let mut invalid = config.clone();
                let noise = &mut invalid.noise[axis];
                match field {
                    0 => noise.stddev = -0.1,
                    1 => noise.bias_stddev = -0.1,
                    2 => noise.dynamic_bias_stddev = -0.1,
                    _ => noise.precision = -0.1,
                }
                assert!(MagnetometerNoise::new(&invalid, [0; 32], 1).is_err());
            }
        }
        for field in 0..7 {
            let mut hidden = sdformat::Magnetometer::default();
            let noise = &mut hidden.noise[axis];
            match field {
                0 => noise.mean = 1.0,
                1 => noise.stddev = 1.0,
                2 => noise.bias_mean = 1.0,
                3 => noise.bias_stddev = 1.0,
                4 => noise.dynamic_bias_stddev = 1.0,
                5 => noise.dynamic_bias_correlation_time = 1.0,
                _ => noise.precision = 1.0,
            }
            assert!(MagnetometerNoise::new(&hidden, [0; 32], 1).is_err());
        }
    }
    for field in ["stddev", "bias_stddev", "dynamic_bias_stddev"] {
        let xml=XML.replace("<magnetometer/>",&format!("<magnetometer><x><noise type='gaussian'><{field}>-0.1</{field}></noise></x></magnetometer>"));
        let error = sdformat::load_str(&xml, "magnetic-negative-deviation", &NoResources)
            .expect_err("parser rejects negative deviation");
        assert_eq!(error.code, sdformat::ErrorCode::InvalidValue);
        assert_eq!(
            error.message,
            "noise standard deviations must be nonnegative"
        );
    }
}

#[test]
fn finite_initial_bias_overflow_is_refused_and_finite_correlation_times_have_no_elapsed_evolution()
{
    let mut config = sdformat::Magnetometer::default();
    config.noise[0] = sdformat::Noise {
        kind: sdformat::NoiseKind::Gaussian,
        bias_mean: f64::MAX,
        bias_stddev: f64::MAX,
        ..Default::default()
    };
    assert!((0..64).any(|seed| MagnetometerNoise::new(&config, [seed; 32], 9).is_err()));
    for correlation in [-1.0, 0.0, 2.0] {
        config.noise[0] = sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            mean: 0.25,
            dynamic_bias_stddev: f64::MAX,
            dynamic_bias_correlation_time: correlation,
            ..Default::default()
        };
        let mut state = MagnetometerNoise::new(&config, [0; 32], 1)
            .unwrap()
            .unwrap();
        for _ in 0..3 {
            let mut reading = MagnetometerReading {
                field_tesla: [1.0, 2.0, 3.0],
            };
            state.apply(&mut reading).unwrap();
            assert_eq!(reading.field_tesla, [1.25, 2.0, 3.0]);
        }
    }
}

#[test]
fn a_nonfinite_noisy_vector_replaces_previous_valid_sample_and_is_distinct_from_ideal_faults() {
    let xml = XML
        .replace("0 0 3 0.2 -0.3 0.4", "0 0 3 0 0 0")
        .replace("1 0 0.5 0.3 0.4 -0.2", "1 0 0.5 0 0 0");
    let mut sim = load(&xml);
    let entity = sensor(&sim);
    let identity = sim.world().get::<EntityId>(entity).unwrap().get();
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<MagnetometerSample>(entity)
            .unwrap()
            .reading
            .is_ok()
    );
    let mut config = sdformat::Magnetometer::default();
    config.noise[0] = sdformat::Noise {
        kind: sdformat::NoiseKind::Gaussian,
        mean: f64::MAX,
        ..Default::default()
    };
    {
        let mut sensor = sim.world_mut().get_mut::<Magnetometer>(entity).unwrap();
        sensor.noise = MagnetometerNoise::new(&config, [0; 32], identity).unwrap();
        sensor.field_world = [f64::MAX, 0.0, 0.0];
    }
    sim.step_exact(9).unwrap();
    let sample = sim.world().get::<MagnetometerSample>(entity).unwrap();
    assert_eq!(sample.time, Duration::from_millis(10));
    assert!(matches!(
        sample.reading,
        Err(MagnetometerSampleError::Noise(_))
    ));
    assert!(!sim.is_faulted());
    sim.world_mut()
        .get_mut::<Magnetometer>(entity)
        .unwrap()
        .field_world[0] = f64::NAN;
    sim.step_exact(10).unwrap();
    assert!(matches!(
        sim.world()
            .get::<MagnetometerSample>(entity)
            .unwrap()
            .reading,
        Err(MagnetometerSampleError::Ideal(_))
    ));
}

#[test]
fn noisy_model_removal_retires_state_and_loaded_reconstruction_replays_original_public_identity() {
    let xml = XML.replace(
        "<magnetometer/>",
        "<magnetometer><z><noise type='gaussian'><stddev>0.2</stddev></noise></z></magnetometer>",
    );
    let mut sim = load(&xml);
    let (old, id) = named(&sim, "default::rig::body::magnetometer");
    let model = named(&sim, "default::rig").1;
    sim.step_exact(1).unwrap();
    let first = sim
        .world()
        .get::<MagnetometerSample>(old)
        .unwrap()
        .reading
        .as_ref()
        .unwrap()
        .field_tesla;
    remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), id).is_none());
    assert!(
        !sim.world().contains_resource::<Installed>()
            && !sim.world().contains_resource::<Interest>()
    );
    initial::reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), id).unwrap();
    assert_ne!(restored, old);
    assert!(
        sim.world()
            .get::<Magnetometer>(restored)
            .unwrap()
            .noise
            .is_some()
    );
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .get::<MagnetometerSample>(restored)
            .unwrap()
            .reading
            .as_ref()
            .unwrap()
            .field_tesla,
        first
    );
}
