// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;

const XML: &str = include_str!("../fixtures/m13_altimeter.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m13-altimeter.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("admitted altimeter fixture")
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
    named(sim, "default::rig::body::altimeter").0
}
fn without_sensor() -> String {
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}

#[test]
fn absent_altimeter_owns_no_callbacks_marker_interest_or_samples() {
    let mut absent = load(&without_sensor());
    let present = load(XML);
    let counts = absent.scheduled_system_counts();
    assert_eq!(
        present.scheduled_system_counts(),
        (counts.0 + 1, counts.1 + 1)
    );
    assert!(!absent.world().contains_resource::<Installed>());
    set_altimeter_interest(&mut absent, []);
    assert!(!absent.world().contains_resource::<Interest>());
    absent.step_exact(3).expect("advancing absence");
    absent.reset().expect("reset absence");
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert!(!absent.world().contains_resource::<Installed>());
    assert!(!absent.world().contains_resource::<Interest>());
}

#[test]
fn native_world_z_reference_and_actual_offset_point_velocity_are_preserved() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    let (link, link_id) = named(&sim, "default::rig::body");
    let metadata = sim
        .world()
        .get::<AltimeterPublication>(entity)
        .expect("metadata");
    assert_eq!(metadata.vertical_reference, 3.5);
    assert_eq!(
        (
            metadata.entity_name.as_str(),
            metadata.frame_id.as_str(),
            metadata.topic.as_str()
        ),
        (
            "rig::body::altimeter",
            "rig/body/altimeter",
            "/m13/altimeter"
        )
    );
    queue_link_world_velocity(&mut sim, link_id, [0.0, 0.0, 3.0], [0.0, 2.0, 0.0])
        .expect("finite offset-point motion");
    sim.step_exact(1).expect("real Rapier advance");
    let world = sim.world();
    let pose = world.get::<WorldPose>(link).expect("link pose");
    let velocity = world.get::<Velocity>(link).expect("link velocity");
    let [x, y, z, w] = pose.rotation;
    // Independent scalar expansion of R*[1,0,0.5], not the sensor's orientation.
    let offset_x = 1.0 - 2.0 * (y * y + z * z) + (x * z + y * w);
    let offset_y = 2.0 * (x * y + z * w) + (y * z - x * w);
    let offset_z = 2.0 * (x * z - y * w) + 0.5 * (1.0 - 2.0 * (x * x + y * y));
    let expected_vz =
        velocity.linear[2] + velocity.angular[0] * offset_y - velocity.angular[1] * offset_x;
    let sample = world.get::<AltimeterSample>(entity).expect("fresh sample");
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    let reading = sample.reading.as_ref().expect("valid point measurement");
    assert!((reading.vertical_velocity - expected_vz).abs() < 1e-12);
    assert!((reading.vertical_velocity - velocity.linear[2]).abs() > 1.9);
    assert!((reading.vertical_position - (pose.translation[2] + offset_z - 3.5)).abs() < 1e-12);
    assert_eq!(reading.vertical_reference, 3.5);
    assert_eq!(
        world
            .get::<AltimeterPublication>(entity)
            .expect("fixed metadata")
            .vertical_reference,
        3.5
    );
}

#[test]
fn demand_fixed_cadence_and_pause_do_not_advance_unrequested_measurements() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    set_altimeter_interest(&mut sim, []);
    sim.step_exact(20).expect("no demand advance");
    assert!(sim.world().get::<AltimeterSample>(entity).is_none());
    assert!(
        sim.world()
            .get::<Altimeter>(entity)
            .expect("sensor")
            .schedule
            .is_due(sim.elapsed())
    );
    set_altimeter_interest(&mut sim, [entity, entity]);
    assert_eq!(sim.world().resource::<Interest>().0.len(), 1);
    sim.step_exact(1).expect("first demanded acquisition");
    let first = sim
        .world()
        .get::<AltimeterSample>(entity)
        .expect("due sample")
        .time;
    assert_eq!(first, Duration::from_millis(21));
    sim.tick().expect("paused poll");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(8).expect("before phase-aligned deadline");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(1).expect("phase-aligned acquisition");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(30)
    );
    set_altimeter_interest(&mut sim, []);
    assert!(sim.world().get::<AltimeterSample>(entity).is_none());
    clear_altimeter_interest(&mut sim);
    sim.step_exact(10).expect("local manual demand");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(40)
    );
}

#[test]
fn last_model_removal_retires_callbacks_and_loaded_reset_restores_identity_reference() {
    let mut sim = load(XML);
    let (entity, identity) = named(&sim, "default::rig::body::altimeter");
    let model = named(&sim, "default::rig").1;
    let counts = sim.scheduled_system_counts();
    sim.step_exact(12).expect("sample before removal");
    remove_model(&mut sim, model).expect("remove owned model");
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_altimeter_interest(&mut sim, [entity]);
    assert!(!sim.world().contains_resource::<Interest>());
    initial::reset_loaded_world(&mut sim, |sim| {
        let restored = entity_by_id(sim.world(), identity).expect("same public identity");
        assert!(sim.world().get::<AltimeterSample>(restored).is_none());
        assert_eq!(
            sim.world()
                .get::<AltimeterPublication>(restored)
                .expect("metadata")
                .vertical_reference,
            3.5
        );
    })
    .expect("loaded scene reconstruction");
    assert_eq!(sim.scheduled_system_counts(), counts);
    let restored = entity_by_id(sim.world(), identity).expect("restored sensor");
    sim.step_exact(1).expect("new epoch acquisition");
    let sample = sim
        .world()
        .get::<AltimeterSample>(restored)
        .expect("reset sample");
    assert_eq!((sample.epoch, sample.time), (1, Duration::from_millis(1)));
    assert_eq!(
        sample.reading.as_ref().expect("reading").vertical_position,
        0.0
    );
}

#[test]
fn admission_refuses_invalid_noise_metadata_second_sensor_and_other_engines() {
    let original = "<altimeter/>";
    // Negative deviations are rejected by the maintained parser before Scene admission.
    for field in ["stddev", "bias_stddev", "dynamic_bias_stddev"] {
        let xml = XML.replace(original, &format!(
            "<altimeter><vertical_position><noise type='gaussian'><{field}>-0.1</{field}></noise></vertical_position></altimeter>"
        ));
        let error = sdformat::load_str(&xml, "altimeter-negative-deviation", &NoResources)
            .expect_err("negative deviation must not parse");
        assert_eq!(error.code, sdformat::ErrorCode::InvalidValue);
        assert_eq!(
            error.message,
            "noise standard deviations must be nonnegative"
        );
        assert!(matches!(
            load_world_with_sdf_step(&xml, "altimeter-negative-deviation", "default", &NoResources, PhysicsEngine::Rapier),
            Err(LoadError::Sdf(error)) if error.code == sdformat::ErrorCode::InvalidValue
        ));
    }
    for replacement in [
        "<altimeter><vertical_position><noise type='none'><mean>1</mean></noise></vertical_position></altimeter>",
        "<altimeter><vertical_velocity><noise type='gaussian'><precision>-1</precision></noise></vertical_velocity></altimeter>",
        "<altimeter/><visualize>true</visualize>",
        "<altimeter/><enable_metrics>true</enable_metrics>",
        "<altimeter/><alwaysOn>true</alwaysOn>",
        "<altimeter/><plugin name='custom' filename='custom'/>",
    ] {
        let xml = XML.replace(original, replacement);
        sdformat::load_str(&xml, "altimeter-parsed-negative", &NoResources)
            .expect("negative remains a parsed typed sensor, not an XML rejection");
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "altimeter-refusal",
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
    sdformat::load_str(&tiny, "altimeter-tiny-rate", &NoResources)
        .expect("finite positive SDF rate");
    assert!(matches!(
        load_world_with_sdf_step(
            &tiny,
            "altimeter-tiny-rate",
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
        "</sensor><sensor name='second' type='altimeter'><altimeter/></sensor>",
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &twice,
            "two-altimeters",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "multiple altimeters",
            ..
        })
    ));
    assert!(matches!(
        load_world_with_sdf_step(
            XML,
            "altimeter-engine",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "Rapier ideal altimeter",
            ..
        })
    ));
}

#[test]
fn omitted_topic_and_frame_use_native_names_and_canonical_frame_wins_over_legacy() {
    let plain = XML
        .replace("<topic>/m13/altimeter</topic>", "")
        .replace("<frame_id>rig/body/altimeter</frame_id>", "");
    let sim = load(&plain);
    let metadata = sim
        .world()
        .get::<AltimeterPublication>(sensor(&sim))
        .expect("default metadata");
    assert_eq!(
        metadata.topic,
        "/world/default/model/rig/link/body/sensor/altimeter/altimeter"
    );
    assert_eq!(metadata.frame_id, "rig::body::altimeter");
    let legacy = plain.replace(
        "<altimeter/>",
        "<gz_frame_id>legacy</gz_frame_id><altimeter/>",
    );
    let sim = load(&legacy);
    assert_eq!(
        sim.world()
            .get::<AltimeterPublication>(sensor(&sim))
            .expect("legacy metadata")
            .frame_id,
        "legacy"
    );
    let both = XML.replace(
        "<altimeter/>",
        "<gz_frame_id>legacy</gz_frame_id><altimeter/>",
    );
    let sim = load(&both);
    assert_eq!(
        sim.world()
            .get::<AltimeterPublication>(sensor(&sim))
            .expect("canonical metadata")
            .frame_id,
        "rig/body/altimeter"
    );
}

#[test]
fn noise_state_is_optional_and_rejects_every_nonfinite_parameter_and_bounded_subset_violation() {
    assert!(
        AltimeterNoise::new(&sdformat::Altimeter::default(), [0; 32], 1)
            .unwrap()
            .is_none()
    );
    for kind in [
        sdformat::NoiseKind::Gaussian,
        sdformat::NoiseKind::GaussianQuantized,
    ] {
        for channel in 0..2 {
            let mut config = sdformat::Altimeter::default();
            let noise = if channel == 0 {
                &mut config.vertical_position
            } else {
                &mut config.vertical_velocity
            };
            noise.kind = kind;
            let state = AltimeterNoise::new(&config, [0; 32], 1).unwrap().unwrap();
            assert!(state.channels[channel].is_some());
            assert!(state.channels[1 - channel].is_none());
            for field in 0..7 {
                for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                    let mut invalid = config.clone();
                    let noise = if channel == 0 {
                        &mut invalid.vertical_position
                    } else {
                        &mut invalid.vertical_velocity
                    };
                    match field {
                        0 => noise.mean = value,
                        1 => noise.stddev = value,
                        2 => noise.bias_mean = value,
                        3 => noise.bias_stddev = value,
                        4 => noise.dynamic_bias_stddev = value,
                        5 => noise.dynamic_bias_correlation_time = value,
                        _ => noise.precision = value,
                    }
                    assert!(AltimeterNoise::new(&invalid, [0; 32], 1).is_err());
                }
            }
            for field in 0..4 {
                let mut invalid = config.clone();
                let noise = if channel == 0 {
                    &mut invalid.vertical_position
                } else {
                    &mut invalid.vertical_velocity
                };
                match field {
                    0 => noise.stddev = -0.1,
                    1 => noise.bias_stddev = -0.1,
                    2 => noise.dynamic_bias_stddev = -0.1,
                    _ => noise.precision = -0.1,
                }
                assert!(AltimeterNoise::new(&invalid, [0; 32], 1).is_err());
            }
        }
    }
    for field in 0..7 {
        let mut hidden = sdformat::Altimeter::default();
        match field {
            0 => hidden.vertical_position.mean = 1.0,
            1 => hidden.vertical_position.stddev = 1.0,
            2 => hidden.vertical_position.bias_mean = 1.0,
            3 => hidden.vertical_position.bias_stddev = 1.0,
            4 => hidden.vertical_position.dynamic_bias_stddev = 1.0,
            5 => hidden.vertical_position.dynamic_bias_correlation_time = 1.0,
            _ => hidden.vertical_position.precision = 1.0,
        }
        assert!(AltimeterNoise::new(&hidden, [0; 32], 1).is_err());
    }
}

#[test]
fn noise_fault_is_typed_and_replaces_prior_valid_sample_without_changing_reference() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .unwrap()
            .reading
            .is_ok()
    );
    let config = sdformat::Altimeter {
        vertical_position: sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            mean: f64::MAX,
            ..Default::default()
        },
        ..Default::default()
    };
    let identity = sim.world().get::<EntityId>(entity).unwrap().get();
    {
        let mut owned = sim.world_mut().get_mut::<Altimeter>(entity).unwrap();
        owned.noise = AltimeterNoise::new(&config, [0; 32], identity).unwrap();
        // Finite model subtraction produces MAX; adding the finite mean overflows.
        owned.reference = -f64::MAX;
    }
    sim.step_exact(9).unwrap();
    let sample = sim.world().get::<AltimeterSample>(entity).unwrap();
    assert_eq!(sample.time, Duration::from_millis(10));
    assert!(matches!(
        sample.reading,
        Err(AltimeterSampleError::Noise(_))
    ));
    sim.world_mut()
        .get_mut::<Altimeter>(entity)
        .unwrap()
        .reference = f64::NAN;
    sim.step_exact(10).unwrap();
    assert!(matches!(
        sim.world().get::<AltimeterSample>(entity).unwrap().reading,
        Err(AltimeterSampleError::Ideal(_))
    ));
}

#[test]
fn noisy_loaded_removal_retires_state_and_restores_seeded_original_identity() {
    let configured = XML.replace("<altimeter/>", "<altimeter><vertical_velocity><noise type='gaussian'><stddev>0.3</stddev></noise></vertical_velocity></altimeter>");
    let mut sim = load(&configured);
    let (old_entity, id) = named(&sim, "default::rig::body::altimeter");
    let model = named(&sim, "default::rig").1;
    sim.step_exact(1).unwrap();
    let first = *sim
        .world()
        .get::<AltimeterSample>(old_entity)
        .unwrap()
        .reading
        .as_ref()
        .unwrap();
    remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), id).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    initial::reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = entity_by_id(sim.world(), id).unwrap();
    assert_ne!(restored, old_entity);
    assert!(
        sim.world()
            .get::<Altimeter>(restored)
            .unwrap()
            .noise
            .is_some()
    );
    sim.step_exact(1).unwrap();
    assert_eq!(
        *sim.world()
            .get::<AltimeterSample>(restored)
            .unwrap()
            .reading
            .as_ref()
            .unwrap(),
        first
    );
}

#[test]
fn finite_but_overflowing_initial_bias_is_refused_and_finite_correlation_time_never_evolves() {
    let overflowing = sdformat::Altimeter {
        vertical_position: sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            bias_mean: f64::MAX,
            bias_stddev: f64::MAX,
            ..Default::default()
        },
        ..Default::default()
    };
    // Fixed bounded seeds exercise finite inputs whose derived initialization is not finite.
    assert!((0..64).any(|seed| AltimeterNoise::new(&overflowing, [seed; 32], 9).is_err()));
    for correlation in [-1.0, 0.0, 2.0] {
        let config = sdformat::Altimeter {
            vertical_position: sdformat::Noise {
                kind: sdformat::NoiseKind::Gaussian,
                mean: 0.25,
                dynamic_bias_stddev: f64::MAX,
                dynamic_bias_correlation_time: correlation,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut state = AltimeterNoise::new(&config, [0; 32], 1).unwrap().unwrap();
        for _ in 0..3 {
            let mut reading = AltimeterReading {
                vertical_position: 0.0,
                vertical_velocity: 2.0,
                vertical_reference: 3.5,
            };
            state.apply(&mut reading).unwrap();
            assert_eq!(
                (
                    reading.vertical_position,
                    reading.vertical_velocity,
                    reading.vertical_reference
                ),
                (0.25, 2.0, 3.5)
            );
        }
    }
}
