// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;

const XML: &str = include_str!("../fixtures/m26_air_speed.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m26-air_speed.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("admitted air_speed fixture")
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
    named(sim, "default::rig::body::air_speed").0
}
fn without_sensor() -> String {
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}

#[test]
fn absent_air_speed_owns_no_callbacks_marker_interest_or_samples() {
    let mut absent = load(&without_sensor());
    let present = load(XML);
    let counts = absent.scheduled_system_counts();
    assert_eq!(
        present.scheduled_system_counts(),
        (counts.0 + 1, counts.1 + 1)
    );
    assert!(!absent.world().contains_resource::<Installed>());
    set_air_speed_interest(&mut absent, []);
    assert!(!absent.world().contains_resource::<Interest>());
    absent.step_exact(3).expect("advancing absence");
    absent.reset().expect("reset absence");
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert!(!absent.world().contains_resource::<Installed>());
    assert!(!absent.world().contains_resource::<Interest>());
}

#[test]
fn demand_fixed_cadence_and_pause_do_not_advance_unrequested_measurements() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    set_air_speed_interest(&mut sim, []);
    sim.step_exact(20).expect("no demand advance");
    assert!(sim.world().get::<AirSpeedSample>(entity).is_none());
    assert!(
        sim.world()
            .get::<AirSpeed>(entity)
            .expect("sensor")
            .schedule
            .is_due(sim.elapsed())
    );
    set_air_speed_interest(&mut sim, [entity, entity]);
    assert_eq!(sim.world().resource::<Interest>().0.len(), 1);
    sim.step_exact(1).expect("first demanded acquisition");
    let first = sim
        .world()
        .get::<AirSpeedSample>(entity)
        .expect("due sample")
        .time;
    assert_eq!(first, Duration::from_millis(21));
    sim.tick().expect("paused poll");
    assert_eq!(
        sim.world()
            .get::<AirSpeedSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(8).expect("before phase-aligned deadline");
    assert_eq!(
        sim.world()
            .get::<AirSpeedSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(1).expect("phase-aligned acquisition");
    assert_eq!(
        sim.world()
            .get::<AirSpeedSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(30)
    );
    set_air_speed_interest(&mut sim, []);
    assert!(sim.world().get::<AirSpeedSample>(entity).is_none());
    clear_air_speed_interest(&mut sim);
    sim.step_exact(10).expect("local manual demand");
    assert_eq!(
        sim.world()
            .get::<AirSpeedSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(40)
    );
}

#[test]
fn last_model_removal_retires_callbacks_and_loaded_reset_restores_absolute_height() {
    let mut sim = load(XML);
    let (entity, identity) = named(&sim, "default::rig::body::air_speed");
    let model = named(&sim, "default::rig").1;
    let counts = sim.scheduled_system_counts();
    sim.step_exact(12).expect("sample before removal");
    remove_model(&mut sim, model).expect("remove owned model");
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_air_speed_interest(&mut sim, [entity]);
    assert!(!sim.world().contains_resource::<Interest>());
    initial::reset_loaded_world(&mut sim, |sim| {
        let restored = entity_by_id(sim.world(), identity).expect("same public identity");
        assert!(sim.world().get::<AirSpeedSample>(restored).is_none());
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
        .get::<AirSpeedSample>(restored)
        .expect("reset sample");
    assert_eq!((sample.epoch, sample.time), (1, Duration::from_millis(1)));
    assert_eq!(
        sample.reading.as_ref().expect("reading").diff_pressure,
        gz_sensors::air_speed::air_speed(AirSpeedInput {
            world_z: 3.5,
            velocity_sensor: [0.0; 3]
        })
        .unwrap()
        .diff_pressure
    );
}

#[test]
fn omitted_topic_and_frame_use_native_names_and_canonical_frame_wins_over_legacy() {
    let plain = XML
        .replace("<topic>/m26/air_speed</topic>", "")
        .replace("<frame_id>rig/body/air_speed</frame_id>", "");
    let sim = load(&plain);
    let metadata = sim
        .world()
        .get::<AirSpeedPublication>(sensor(&sim))
        .expect("default metadata");
    assert_eq!(
        metadata.topic,
        "/world/default/model/rig/link/body/sensor/air_speed/air_speed"
    );
    assert_eq!(metadata.frame_id, "rig::body::air_speed");
    let legacy = plain.replace(
        "<air_speed/>",
        "<gz_frame_id>legacy</gz_frame_id><air_speed/>",
    );
    let sim = load(&legacy);
    assert_eq!(
        sim.world()
            .get::<AirSpeedPublication>(sensor(&sim))
            .expect("legacy metadata")
            .frame_id,
        "legacy"
    );
    let both = XML.replace(
        "<air_speed/>",
        "<gz_frame_id>legacy</gz_frame_id><air_speed/>",
    );
    let sim = load(&both);
    assert_eq!(
        sim.world()
            .get::<AirSpeedPublication>(sensor(&sim))
            .expect("canonical metadata")
            .frame_id,
        "rig/body/air_speed"
    );
}

#[test]
fn omitted_config_explicit_default_noise_and_continuous_rate_are_admitted_but_atmosphere_is_not() {
    for config in [
        "",
        "<air_speed><pressure><noise type='none'/></pressure></air_speed>",
    ] {
        let mut sim = load(&XML.replace("<air_speed/>", config).replace(
            "<update_rate>100</update_rate>",
            "<update_rate>0</update_rate>",
        ));
        let entity = sensor(&sim);
        sim.step_exact(2).expect("continuous acquisition");
        assert_eq!(
            sim.world()
                .get::<AirSpeedSample>(entity)
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
            feature: "air_speed default atmosphere",
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
            feature: "air_speed atmosphere domain",
            ..
        })
    ));
}

#[test]
fn invalid_physical_measurement_remains_a_typed_acquisition_error_not_a_fabricated_pressure() {
    fn invalid_pose(mut sensors: Query<&mut WorldPose, With<AirSpeed>>) {
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
        .get::<AirSpeedSample>(entity)
        .expect("typed acquisition outcome");
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    assert_eq!(
        sample.reading,
        Err(AirSpeedSampleError::Ideal(InvalidAirSpeedInput))
    );
    assert!(!sim.is_faulted());
    assert!(sim.fault().is_none());
    assert_eq!(sim.elapsed(), Duration::from_millis(1));
    sim.reset()
        .expect("invalid sample does not poison loaded reset");
    assert!(sim.world().get::<AirSpeedSample>(entity).is_none());
}

fn scalar_rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let cross = [
        y * v[2] - z * v[1],
        z * v[0] - x * v[2],
        x * v[1] - y * v[0],
    ];
    let again = [
        y * cross[2] - z * cross[1],
        z * cross[0] - x * cross[2],
        x * cross[1] - y * cross[0],
    ];
    std::array::from_fn(|i| v[i] + 2.0 * (w * cross[i] + again[i]))
}

#[test]
fn actual_rotating_offset_velocity_is_rotated_once_and_not_taken_at_link_center() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    let (link, id) = named(&sim, "default::rig::body");
    queue_link_world_velocity(&mut sim, id, [3.0, 0.0, 0.0], [0.0, 0.0, 2.0]).unwrap();
    sim.step_exact(1).unwrap();
    let world = sim.world();
    let pose = world.get::<WorldPose>(entity).unwrap();
    let lp = world.get::<WorldPose>(link).unwrap();
    let velocity = world.get::<Velocity>(link).unwrap();
    let r = scalar_rotate(lp.rotation, [0.0, 0.5, 0.5]);
    let w = velocity.angular;
    let cross = [
        w[1] * r[2] - w[2] * r[1],
        w[2] * r[0] - w[0] * r[2],
        w[0] * r[1] - w[1] * r[0],
    ];
    let point = std::array::from_fn(|i| velocity.linear[i] + cross[i]);
    let q = pose.rotation;
    let inverse = [-q[0], -q[1], -q[2], q[3]];
    let relative = scalar_rotate(inverse, point);
    let expected = gz_sensors::air_speed::air_speed(AirSpeedInput {
        world_z: pose.translation[2],
        velocity_sensor: relative,
    })
    .unwrap();
    let sample = world.get::<AirSpeedSample>(entity).unwrap();
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    assert_eq!(*sample.reading.as_ref().unwrap(), expected);
    let center = gz_sensors::air_speed::air_speed(AirSpeedInput {
        world_z: pose.translation[2],
        velocity_sensor: scalar_rotate(inverse, velocity.linear),
    })
    .unwrap();
    let twice = gz_sensors::air_speed::air_speed(AirSpeedInput {
        world_z: pose.translation[2],
        velocity_sensor: scalar_rotate(inverse, relative),
    })
    .unwrap();
    assert!((expected.diff_pressure - center.diff_pressure).abs() > 0.5);
    assert!((expected.diff_pressure - twice.diff_pressure).abs() > 0.5);
    assert!((pose.translation[2] - lp.translation[2]).abs() > 0.4);
}

#[test]
fn runtime_invalid_acquisition_replaces_a_valid_sample_with_the_exact_ideal_fault() {
    fn invalid(mut sensors: Query<&mut WorldPose, With<AirSpeed>>) {
        for mut p in &mut sensors {
            p.translation[2] = 50000.0;
        }
    }
    let mut sim = load(XML);
    let e = sensor(&sim);
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<AirSpeedSample>(e)
            .unwrap()
            .reading
            .is_ok()
    );
    sim.add_systems(Phase::PostStep, invalid.before(sample));
    sim.step_exact(9).unwrap();
    let value = sim.world().get::<AirSpeedSample>(e).unwrap();
    assert_eq!(value.time, Duration::from_millis(10));
    assert_eq!(
        value.reading,
        Err(AirSpeedSampleError::Ideal(InvalidAirSpeedInput))
    );
}

#[test]
fn strict_noise_metadata_domain_nested_duplicate_and_backend_admission_are_atomic() {
    for replacement in [
        "<air_speed><pressure><noise type='none'><mean>1</mean></noise></pressure></air_speed>",
        "<air_speed/><visualize>true</visualize>",
        "<air_speed/><enable_metrics>true</enable_metrics>",
        "<air_speed/><plugin name='custom' filename='custom'/>",
        "<air_speed/><alwaysOn>true</alwaysOn>",
    ] {
        let xml = XML.replace("<air_speed/>", replacement);
        sdformat::load_str(&xml, "parsed-air-speed-refusal", &NoResources).unwrap();
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "refuse",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            ),
            Err(LoadError::Unsupported { .. })
        ));
    }
    for xml in [
        XML.replace(
            "<update_rate>100</update_rate>",
            "<update_rate>1e-300</update_rate>",
        ),
        XML.replace("<pose>0 0 3 0 0 0</pose>", "<pose>0 0 50000 0 0 0</pose>"),
        XML.replace(
            "</sensor>",
            "</sensor><sensor name='second' type='air_speed'><air_speed/></sensor>",
        ),
        XML.replace(
            "<model name='rig'>",
            "<model name='outer'><model name='rig'>",
        )
        .replace("</link></model>", "</link></model></model>"),
        XML.replace(
            "<gravity>0 0 0</gravity>",
            "<gravity>0 0 0</gravity><wind><linear_velocity>NaN 0 0</linear_velocity></wind>",
        ),
        XML.replace(
            "<gravity>0 0 0</gravity>",
            "<gravity>0 0 0</gravity><atmosphere type='adiabatic'/>",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bounded-air-speed",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            )
            .is_err()
        );
    }
    assert!(matches!(
        load_world_with_sdf_step(XML, "backend", "default", &NoResources, PhysicsEngine::Tpe),
        Err(LoadError::Unsupported {
            feature: "Rapier ideal air_speed",
            ..
        })
    ));
    for field in 0..8 {
        let mut sdf = sdformat::load_str(XML, "typed-air-speed", &NoResources).unwrap();
        let mut world = sdf.worlds.remove(0);
        let sensor = &mut world.models[0].links[0].sensors[0];
        if field == 0 {
            sensor.config = sdformat::SensorConfig::Unconfigured;
        } else {
            let sdformat::SensorConfig::AirSpeed(config) = &mut sensor.config else {
                panic!("typed fixture")
            };
            match field {
                1 => config.pressure.mean = f64::NAN,
                2 => config.pressure.stddev = 1.0,
                3 => config.pressure.bias_mean = 1.0,
                4 => config.pressure.bias_stddev = 1.0,
                5 => config.pressure.dynamic_bias_stddev = 1.0,
                6 => config.pressure.dynamic_bias_correlation_time = 1.0,
                _ => config.pressure.precision = 1.0,
            }
        }
        assert!(preflight(&sdf, &world, PhysicsEngine::Rapier).is_err());
    }
}

#[test]
fn continuous_finite_rates_one_nanosecond_clamp_and_schedule_fault_are_explicit() {
    for rate in ["0", "-1", "1e30"] {
        let mut sim = load(&XML.replace(
            "<update_rate>100</update_rate>",
            &format!("<update_rate>{rate}</update_rate>"),
        ));
        let e = sensor(&sim);
        for tick in 1..=4 {
            sim.step_exact(1).unwrap();
            assert_eq!(
                sim.world().get::<AirSpeedSample>(e).unwrap().time,
                Duration::from_millis(tick)
            );
        }
    }
    fn overflow(mut time: ResMut<UpdateInfo>) {
        time.sim_time = Duration::MAX;
    }
    let mut sim = load(XML);
    let e = sensor(&sim);
    sim.add_systems(Phase::PostStep, overflow.before(sample));
    let error = sim.step_exact(1).unwrap_err();
    assert_eq!(error, gz_sim_core::StepError::Faulted);
    let fault = sim
        .world()
        .resource::<StepFault>()
        .error()
        .unwrap()
        .downcast_ref::<AirSpeedScheduleFault>()
        .expect("typed air-speed sampling deadline fault");
    assert_eq!(fault.entity, e);
    assert_eq!(fault.source, schedule::ScheduleError::TimeOverflow);
    assert_eq!(
        fault.to_string(),
        format!("air_speed {e:?} sampling deadline: sensor sampling time overflow")
    );
    assert!(sim.world().get::<AirSpeedSample>(e).is_none());
}

#[test]
fn noise_configuration_distinguishes_none_enabled_zero_and_typed_invalid_fields() {
    let default = sdformat::Noise::default();
    assert!(PressureNoise::new(&default, [42; 32], 1).unwrap().is_none());
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
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut bad = config;
                match field {
                    0 => bad.mean = invalid,
                    1 => bad.stddev = invalid,
                    2 => bad.bias_mean = invalid,
                    3 => bad.bias_stddev = invalid,
                    4 => bad.dynamic_bias_stddev = invalid,
                    5 => bad.dynamic_bias_correlation_time = invalid,
                    _ => bad.precision = invalid,
                }
                assert!(PressureNoise::validate(&bad).is_err());
            }
        }
        for field in 0..4 {
            let mut bad = config;
            match field {
                0 => bad.stddev = -1.0,
                1 => bad.bias_stddev = -1.0,
                2 => bad.dynamic_bias_stddev = -1.0,
                _ => bad.precision = -1.0,
            }
            assert!(PressureNoise::validate(&bad).is_err());
        }
        let signed = sdformat::Noise {
            mean: -1.0,
            bias_mean: -0.3,
            dynamic_bias_correlation_time: -1.0,
            ..config
        };
        assert!(PressureNoise::validate(&signed).is_ok());
    }
}

#[test]
fn noise_float_conversion_and_final_scale_errors_are_distinct_from_ideal_and_do_not_hide_cancellation()
 {
    let input = AirSpeedInput {
        world_z: 3.5,
        velocity_sensor: [0.0; 3],
    };
    for (mean, precision) in [(f64::MAX, 0.0), (f64::from(f32::MAX), 0.0), (f64::MAX, 0.1)] {
        let config = sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            mean,
            precision,
            ..Default::default()
        };
        let mut noise = PressureNoise::new(&config, [42; 32], 1).unwrap().unwrap();
        assert_eq!(
            noise.apply(input),
            Err(AirSpeedSampleError::Noise(InvalidNoise))
        );
    }
    let mut noise = PressureNoise::new(
        &sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            ..Default::default()
        },
        [42; 32],
        1,
    )
    .unwrap()
    .unwrap();
    assert_eq!(
        noise.apply(AirSpeedInput {
            world_z: f64::NAN,
            ..input
        }),
        Err(AirSpeedSampleError::Ideal(InvalidAirSpeedInput))
    );
    let large = AirSpeedInput {
        world_z: 0.0,
        velocity_sensor: [1e20, 0.0, 0.0],
    };
    let (precursor, _) = air_speed_unscaled(large).unwrap();
    let mut cancelling = PressureNoise::new(
        &sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            mean: -f64::from(precursor),
            ..Default::default()
        },
        [42; 32],
        1,
    )
    .unwrap()
    .unwrap();
    assert_eq!(cancelling.apply(large).unwrap().diff_pressure, 0.0);
    let mut a = PressureNoise::new(
        &sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            stddev: 0.2,
            ..Default::default()
        },
        [42; 32],
        1,
    )
    .unwrap()
    .unwrap();
    let mut b = PressureNoise::new(
        &sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            stddev: 0.2,
            ..Default::default()
        },
        [42; 32],
        2,
    )
    .unwrap()
    .unwrap();
    assert!((0..20).any(|_| a.apply(input).unwrap() != b.apply(input).unwrap()));
}

#[test]
fn noise_fault_replaces_previous_valid_sample_without_a_terminal_schedule_error() {
    let xml=XML.replace("<air_speed/>","<air_speed><pressure><noise type='gaussian'><mean>0.1</mean></noise></pressure></air_speed>");
    let mut sim = load(&xml);
    let entity = sensor(&sim);
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<AirSpeedSample>(entity)
            .unwrap()
            .reading
            .is_ok()
    );
    sim.world_mut().get_mut::<AirSpeed>(entity).unwrap().noise = PressureNoise::new(
        &sdformat::Noise {
            kind: sdformat::NoiseKind::Gaussian,
            mean: f64::MAX,
            ..Default::default()
        },
        [42; 32],
        1,
    )
    .unwrap();
    sim.step_exact(9).unwrap();
    assert_eq!(
        sim.world().get::<AirSpeedSample>(entity).unwrap().reading,
        Err(AirSpeedSampleError::Noise(InvalidNoise))
    );
    assert!(sim.world().resource::<StepFault>().error().is_none());
}
