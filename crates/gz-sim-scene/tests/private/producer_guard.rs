// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::{
    AxisMimic, ExtensionMetadata, Light, ParticleEmitter, Plugin, Projector, Sensor, SensorConfig,
};

type Edit = (&'static str, fn(&mut sdformat::World));

fn world() -> sdformat::World {
    let xml = "<sdf version='1.12'><world name='default'><model name='robot'><link name='body'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><sensor name='imu' type='imu'><imu/></sensor></link><joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model></world></sdf>";
    sdformat::load_str(xml, "guard", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
}
fn extension(source: sdformat::Source) -> ExtensionMetadata {
    ExtensionMetadata {
        tag: "vendor".into(),
        attributes: vec![],
        xml: "<vendor/>".into(),
        source,
    }
}
fn failure(world: &sdformat::World, feature: &str) {
    let error = validate(world).unwrap_err();
    assert!(matches!(error, LoadError::Unsupported { feature: actual, .. } if actual == feature));
}
#[test]
fn world_admission_keeps_native_defaults_and_rejects_unconsumed_fields() {
    assert!(validate(&world()).is_ok());
    let cases: [Edit; 6] = [
        ("world lights", |w| w.lights.push(Light::default())),
        ("world joints", |w| {
            w.joints.push(w.models[0].joints[0].clone())
        }),
        ("world extensions", |w| {
            w.extensions.push(extension(w.source.clone()))
        }),
        ("world wind", |w| w.wind_linear_velocity[0] = 1.0),
        ("world magnetic field", |w| w.magnetic_field[0] = 0.0),
        ("world atmosphere", |w| {
            w.atmosphere.source = Some(w.source.clone())
        }),
    ];
    for (expected, edit) in cases {
        let mut value = world();
        edit(&mut value);
        failure(&value, expected);
    }
}
#[test]
fn model_link_and_sensor_metadata_are_checked_recursively() {
    let cases: [Edit; 12] = [
        ("model wind", |w| w.models[0].enable_wind = true),
        ("model namespace", |w| {
            w.models[0].raw_namespace = Some("robot".into())
        }),
        ("model namespace", |w| {
            w.models[0].raw_namespace = Some(String::new())
        }),
        ("model extensions", |w| {
            let m = &mut w.models[0];
            m.extensions.push(extension(m.source.clone()));
        }),
        ("link lights", |w| {
            w.models[0].links[0].lights.push(Light::default())
        }),
        ("particle emitters", |w| {
            w.models[0].links[0]
                .particle_emitters
                .push(ParticleEmitter::default())
        }),
        ("projectors", |w| {
            w.models[0].links[0].projectors.push(Projector::default())
        }),
        ("collision density", |w| {
            w.models[0].links[0].collisions[0].density = Some(1000.0)
        }),
        ("collision geometry", |w| {
            w.models[0].links[0].collisions[0].geometry = sdformat::Geometry::Empty
        }),
        ("sensor visualization", |w| {
            w.models[0].links[0].sensors[0].visualize = true
        }),
        ("sensor plugins", |w| {
            w.models[0].links[0].sensors[0].plugins.push(Plugin {
                name: "p".into(),
                filename: "p.so".into(),
                xml: "<plugin/>".into(),
            })
        }),
        ("sensor extensions", |w| {
            let s = &mut w.models[0].links[0].sensors[0];
            s.extensions.push(extension(s.source.clone()));
        }),
    ];
    for (expected, edit) in cases {
        let mut value = world();
        edit(&mut value);
        failure(&value, expected);
    }
    let mut value = world();
    value.models[0].links[0].sensors[0].config = SensorConfig::Camera(Box::default());
    failure(&value, "non-IMU sensors");
    let mut value = world();
    let child = value.models[0].clone();
    value.models[0].models.push(child);
    value.models[0].models[0].enable_wind = true;
    failure(&value, "model wind");
}
#[test]
fn joint_metadata_and_joint_sensor_envelopes_are_not_silently_dropped() {
    for kind in [
        sdformat::JointKind::Ball,
        sdformat::JointKind::Gearbox,
        sdformat::JointKind::Screw,
    ] {
        let mut value = world();
        value.models[0].joints[0].kind = kind;
        failure(&value, "joints (unsupported type)");
    }
    let mut value = world();
    value.models[0].joints[0].physics_xml = Some("<physics/>".into());
    failure(&value, "joint physics metadata");
    let mut value = world();
    value.models[0].joints[0].axis.as_mut().unwrap().mimic = Some(AxisMimic {
        joint: "leader".into(),
        axis: "axis".into(),
        multiplier: 1.0,
        offset: 0.0,
        reference: 0.0,
    });
    failure(&value, "joint mimic");
    let mut value = world();
    let sensor = Sensor {
        name: "s".into(),
        config: SensorConfig::Imu(Box::default()),
        ..Sensor::default()
    };
    value.models[0].joints[0].sensors.push(sensor);
    failure(&value, "joint sensors");
}
