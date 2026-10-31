// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Admission of newly retained SDF metadata before any simulation is created.

use crate::{LoadError, require};

pub(super) fn validate(world: &sdformat::World) -> Result<(), LoadError> {
    let name = &world.name;
    require(world.lights.is_empty(), name, "world lights")?;
    require(world.joints.is_empty(), name, "world joints")?;
    require(world.extensions.is_empty(), name, "world extensions")?;
    require(world.wind_linear_velocity == [0.0; 3], name, "world wind")?;
    // SDFormat's implicit Earth magnetic field is inert until a supported
    // magnetic sensor consumes it. A nondefault field is not yet represented.
    require(
        world.magnetic_field == [5.5645e-6, 22.8758e-6, -42.3884e-6],
        name,
        "world magnetic field",
    )?;
    require(world.atmosphere.source.is_none(), name, "world atmosphere")?;
    for model in &world.models {
        validate_model(model)?;
    }
    Ok(())
}

fn validate_model(model: &sdformat::Model) -> Result<(), LoadError> {
    let name = &model.scoped_name;
    require(!model.enable_wind, name, "model wind")?;
    require(model.raw_namespace.is_none(), name, "model namespace")?;
    require(model.extensions.is_empty(), name, "model extensions")?;
    for link in &model.links {
        let name = &link.scoped_name;
        require(link.lights.is_empty(), name, "link lights")?;
        require(link.particle_emitters.is_empty(), name, "particle emitters")?;
        require(link.projectors.is_empty(), name, "projectors")?;
        for collision in &link.collisions {
            require(collision.density.is_none(), name, "collision density")?;
            require(
                !matches!(collision.geometry, sdformat::Geometry::Empty),
                name,
                "collision geometry",
            )?;
        }
        for sensor in &link.sensors {
            validate_sensor(sensor, name)?;
        }
    }
    for joint in &model.joints {
        let name = format!("{}::{}", model.scoped_name, joint.name);
        require(
            !matches!(
                joint.kind,
                sdformat::JointKind::Ball
                    | sdformat::JointKind::Gearbox
                    | sdformat::JointKind::Screw
            ),
            &name,
            "joints (unsupported type)",
        )?;
        require(joint.physics_xml.is_none(), &name, "joint physics metadata")?;
        for axis in [joint.axis.as_ref(), joint.axis2.as_ref()]
            .into_iter()
            .flatten()
        {
            require(axis.mimic.is_none(), &name, "joint mimic")?;
        }
        require(joint.sensors.is_empty(), &name, "joint sensors")?;
    }
    for child in &model.models {
        validate_model(child)?;
    }
    Ok(())
}

fn validate_sensor(sensor: &sdformat::Sensor, parent: &str) -> Result<(), LoadError> {
    let name = format!("{parent}::{}", sensor.name);
    require(!sensor.visualize, &name, "sensor visualization")?;
    require(sensor.plugins.is_empty(), &name, "sensor plugins")?;
    require(sensor.extensions.is_empty(), &name, "sensor extensions")?;
    require(
        sensor.imu().is_some() || crate::supports_camera_sensor(sensor),
        &name,
        "non-IMU sensors",
    )
}

#[cfg(test)]
#[path = "../tests/private/producer_guard.rs"]
mod tests;
