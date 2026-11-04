// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Supplemental accepted-producer pin checks, not original-test completion.
#[path = "support/lidar_admission.rs"]
mod lidar_admission;
use gz_sensors::ImuReading;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ImuSample, InputError, LoadError, Name, PhysicsEngine, Velocity, load_world_seeded,
    load_world_with_sdf_step, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::{NoResources, NoiseKind, Resource, ResourceResolver};
use std::time::Duration;

const RESOURCE: &str = include_str!("fixtures/sdf1b_legacy_imu.sdf");
const ENGINES: [PhysicsEngine; 4] = [
    PhysicsEngine::Tpe,
    PhysicsEngine::Rapier,
    PhysicsEngine::Dart,
    PhysicsEngine::Mujoco,
];

fn model(xml: &str) -> &str {
    &xml[xml.find("<model").unwrap()..xml.find("</model>").unwrap() + "</model>".len()]
}
fn world(version: &str, model: &str) -> String {
    format!(
        "<sdf version='{version}'><world name='default'><gravity>0 0 0</gravity>{model}</world></sdf>"
    )
}
fn modern() -> String {
    let old = model(RESOURCE);
    let old_noise =
        &old[old.find("<noise>").unwrap()..old.find("</noise>").unwrap() + "</noise>".len()];
    let axes = |fields: &str| {
        ["x", "y", "z"]
            .map(|axis| format!("<{axis}><noise type='gaussian'>{fields}</noise></{axis}>"))
            .concat()
    };
    let rate = axes(
        "<mean>0</mean><stddev>0.0002</stddev><bias_mean>7.5e-06</bias_mean><bias_stddev>8e-07</bias_stddev>",
    );
    let accel = axes(
        "<mean>0</mean><stddev>0.017</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev>",
    );
    old.replace(old_noise, &format!("<angular_velocity>{rate}</angular_velocity><linear_acceleration>{accel}</linear_acceleration>"))
}
struct Resources;
impl ResourceResolver for Resources {
    fn resolve(&self, uri: &str, parent: &str) -> Result<Resource, String> {
        assert_eq!(uri, "legacy-robot");
        assert_eq!(parent, "include-world.sdf");
        Ok(Resource {
            canonical_uri: "resources/legacy-robot.sdf".into(),
            xml: RESOURCE.into(),
        })
    }
}
fn reading(sim: &Simulation) -> (Duration, ImuReading) {
    let sample = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<ImuSample>())
        .unwrap();
    (sample.time, *sample.reading.as_ref().unwrap())
}

#[test]
fn legacy_imu_noise_reaches_all_channels_and_reset_in_direct_and_resource_worlds() {
    let modern = world("1.12", &modern());
    let include = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><include><uri>legacy-robot</uri></include></world></sdf>";
    let imported = sdformat::load_str(include, "include-world.sdf", &Resources).unwrap();
    let link = &imported.worlds[0].models[0].links[0];
    assert_eq!(link.source.uri, "resources/legacy-robot.sdf");
    assert_eq!(link.sensors[0].source.uri, "resources/legacy-robot.sdf");
    let imu = link.sensors[0].imu().unwrap();
    for noise in &imu.angular_velocity {
        assert_eq!(noise.kind, NoiseKind::Gaussian);
        assert_eq!(
            (noise.mean, noise.stddev, noise.bias_mean, noise.bias_stddev),
            (0., 0.0002, 7.5e-06, 8e-07)
        );
        assert_eq!(
            (
                noise.dynamic_bias_stddev,
                noise.dynamic_bias_correlation_time,
                noise.precision
            ),
            (0., 0., 0.)
        );
    }
    for noise in &imu.linear_acceleration {
        assert_eq!(noise.kind, NoiseKind::Gaussian);
        assert_eq!(
            (noise.mean, noise.stddev, noise.bias_mean, noise.bias_stddev),
            (0., 0.017, 0.1, 0.001)
        );
        assert_eq!(
            (
                noise.dynamic_bias_stddev,
                noise.dynamic_bias_correlation_time,
                noise.precision
            ),
            (0., 0., 0.)
        );
    }
    for engine in ENGINES {
        for version in ["1.4", "1.5"] {
            let legacy = world(version, model(RESOURCE));
            let mut expected = load_world_seeded(
                &modern,
                "modern.sdf",
                "default",
                Some(Duration::from_millis(1)),
                &NoResources,
                engine,
                [42; 32],
            )
            .unwrap();
            let mut direct = load_world_seeded(
                &legacy,
                "legacy.sdf",
                "default",
                Some(Duration::from_millis(1)),
                &NoResources,
                engine,
                [42; 32],
            )
            .unwrap();
            let mut resource = load_world_seeded(
                include,
                "include-world.sdf",
                "default",
                Some(Duration::from_millis(1)),
                &Resources,
                engine,
                [42; 32],
            )
            .unwrap();
            let mut first = None;
            for step in 1..=3 {
                for sim in [&mut expected, &mut direct, &mut resource] {
                    sim.step_exact(1).unwrap();
                }
                let actual = reading(&direct);
                assert_eq!(actual, reading(&expected), "{engine:?}/{version}");
                assert_eq!(actual, reading(&resource), "{engine:?}/{version}/resource");
                assert_eq!(actual.0, Duration::from_millis(step));
                assert_eq!(
                    actual.1.angular_velocity_covariance,
                    [(0.0002_f64 * 0.0002) as f32 as f64; 3]
                );
                assert_eq!(
                    actual.1.linear_acceleration_covariance,
                    [(0.017_f64 * 0.017) as f32 as f64; 3]
                );
                assert!(actual.1.angular_velocity.iter().any(|v| *v != 0.));
                assert!(actual.1.specific_force.iter().any(|v| *v != 0.));
                first.get_or_insert(actual);
            }
            for sim in [&mut direct, &mut resource] {
                reset_loaded_world(sim, |_| ()).unwrap();
                assert_eq!(sim.elapsed(), Duration::ZERO);
                sim.step_exact(1).unwrap();
                assert_eq!(reading(sim), first.unwrap());
            }
        }
    }
}

#[test]
fn legacy_noise_that_runtime_cannot_represent_is_explicitly_refused() {
    let xml = world(
        "1.5",
        &model(RESOURCE).replace("<stddev>0.0002</stddev>", "<stddev>1e308</stddev>"),
    );
    let parsed = sdformat::load_str(&xml, "overflow-noise.sdf", &NoResources).unwrap();
    assert_eq!(
        parsed.worlds[0].models[0].links[0].sensors[0]
            .imu()
            .unwrap()
            .angular_velocity[0]
            .stddev,
        1e308
    );
    for engine in ENGINES {
        assert!(matches!(
            load_world_with_sdf_step(&xml, "overflow-noise.sdf", "default", &NoResources, engine),
            Err(LoadError::Unsupported {
                feature: "unrepresentable IMU noise",
                ..
            })
        ));
    }
}

#[test]
fn manual_auto_false_and_zero_retain_backend_mass_inertia_and_wrench_policy() {
    let base = world("1.12", &modern());
    for engine in ENGINES {
        for attribute in ["", " auto='false'", " auto='0'"] {
            let xml = base.replace(" auto='false'", attribute);
            let parsed = sdformat::load_str(&xml, "manual.sdf", &NoResources).unwrap();
            let inertia = &parsed.worlds[0].models[0].links[0].inertial;
            assert_eq!(inertia.mass, 2.);
            assert_eq!(inertia.inertia, [0.2, 0.2, 0.2, 0., 0., 0.]);
            let mut sim =
                load_world_with_sdf_step(&xml, "manual.sdf", "default", &NoResources, engine)
                    .unwrap();
            let id = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|n| n.0 == "default::robot::body")
                })
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            let result = queue_link_world_wrench(&mut sim, id, [2., 0., 0.], [0., 0., 0.2]);
            if engine == PhysicsEngine::Tpe {
                assert!(matches!(
                    result,
                    Err(InputError::Unsupported("TPE external wrench"))
                ));
            } else {
                result.unwrap();
            }
            sim.step_exact(1).unwrap();
            let velocity = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<EntityId>().is_some_and(|i| i.get() == id))
                .unwrap()
                .get::<Velocity>()
                .unwrap();
            let speed = if engine == PhysicsEngine::Tpe {
                0.
            } else {
                0.001
            };
            assert!(
                (velocity.linear[0] - speed).abs() < 1e-12,
                "{engine:?}: {:?}",
                velocity.linear
            );
            assert!(
                (velocity.angular[2] - speed).abs() < 1e-12,
                "{engine:?}: {:?}",
                velocity.angular
            );
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            sim.step_exact(1).unwrap();
            let velocity = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<EntityId>().is_some_and(|i| i.get() == id))
                .unwrap()
                .get::<Velocity>()
                .unwrap();
            assert_eq!(velocity.linear, [0.; 3]);
            assert_eq!(velocity.angular, [0.; 3]);
        }
    }
}

#[test]
fn standalone_lidar_aliases_do_not_enable_scene_sensor_execution() {
    let mut admitted_cpu_lidars = 0;
    let body = "<scan><horizontal><samples>2</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>1</max_angle></horizontal></scan><range><min>0.1</min><max>10</max><resolution>0.01</resolution></range>";
    for alias in ["lidar", "ray", "gpu_lidar", "gpu_ray"] {
        let element = format!("<{alias}>{body}</{alias}>");
        let metadata = sdformat::load_lidar_str(&element, "lidar.xml").unwrap();
        assert_eq!(metadata.horizontal.samples, 2);
        assert_eq!(metadata.range_max, 10.);
        assert!(metadata.to_xml().unwrap().starts_with("<lidar>"));
        let model = modern();
        let old_sensor = &model
            [model.find("<sensor").unwrap()..model.find("</sensor>").unwrap() + "</sensor>".len()];
        let standalone_wrapped = world(
            "1.12",
            &model.replace(
                old_sensor,
                &format!("<sensor name='scan' type='{alias}'>{element}</sensor>"),
            ),
        );
        if alias.starts_with("gpu_") {
            for engine in ENGINES {
                assert!(matches!(
                    load_world_with_sdf_step(&standalone_wrapped, "wrong-wrapper.sdf", "default", &NoResources, engine),
                    Err(LoadError::Sdf(error)) if error.code == sdformat::ErrorCode::Unsupported
                ));
            }
        }
        // Standalone aliases do not change Sensor's existing payload grammar.
        let child = alias.strip_prefix("gpu_").unwrap_or(alias);
        let payload = format!("<{child}>{body}</{child}>");
        let xml = world(
            "1.12",
            &model.replace(
                old_sensor,
                &format!("<sensor name='scan' type='{alias}'>{payload}</sensor>"),
            ),
        );
        let parsed = sdformat::load_str(&xml, "lidar-world.sdf", &NoResources).unwrap();
        assert_eq!(
            parsed.worlds[0].models[0].links[0].sensors[0]
                .lidar()
                .unwrap()
                .horizontal
                .samples,
            2
        );
        for engine in ENGINES {
            if matches!(alias, "lidar" | "ray") && matches!(engine, PhysicsEngine::Rapier) {
                let sim = load_world_with_sdf_step(
                    &xml,
                    "lidar-world.sdf",
                    "default",
                    &NoResources,
                    engine,
                )
                .unwrap();
                lidar_admission::assert_cpu_lidar(
                    sim,
                    &xml,
                    "lidar-world.sdf",
                    "default::robot::body::scan",
                    2,
                    [0.0; 3],
                );
                admitted_cpu_lidars += 1;
                continue;
            }
            assert!(matches!(
                load_world_with_sdf_step(&xml, "lidar-world.sdf", "default", &NoResources, engine),
                Err(LoadError::Unsupported {
                    feature: "non-IMU sensors",
                    ..
                })
            ));
        }
    }
    assert_eq!(admitted_cpu_lidars, 2);
}
