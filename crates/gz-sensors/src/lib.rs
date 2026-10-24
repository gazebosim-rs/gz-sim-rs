//! Sensor measurement models. Scheduling and transport belong to the simulator.

use glam::{DQuat, DVec3};

/// SI inputs at one acquisition instant. Quaternions use [x, y, z, w].
#[derive(Debug, Clone, Copy)]
pub struct ImuInput {
    pub world_orientation: [f64; 4],
    pub reference_orientation: [f64; 4],
    pub acceleration_sensor: [f64; 3],
    pub angular_velocity_sensor: [f64; 3],
    pub gravity_world: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    pub orientation: [f64; 4],
    pub specific_force: [f64; 3],
    pub angular_velocity: [f64; 3],
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
#[error("IMU input must be finite, with unit quaternions and finite derived acceleration")]
pub struct InvalidImuInput;

fn unit(value: [f64; 4]) -> Result<DQuat, InvalidImuInput> {
    let q = DQuat::from_array(value);
    if !q.is_finite() || (q.length_squared() - 1.0).abs() > 1e-12 {
        return Err(InvalidImuInput);
    }
    Ok(q)
}

/// Computes the noise-free Gazebo IMU convention: acceleration minus gravity.
/// A stationary upright sensor reports positive Z specific force in Z-down gravity.
pub fn imu(input: ImuInput) -> Result<ImuReading, InvalidImuInput> {
    let world = unit(input.world_orientation)?;
    let reference = unit(input.reference_orientation)?;
    let acceleration = DVec3::from_array(input.acceleration_sensor);
    let angular_velocity = DVec3::from_array(input.angular_velocity_sensor);
    let gravity = DVec3::from_array(input.gravity_world);
    let specific_force = acceleration - world.inverse() * gravity;
    if !acceleration.is_finite()
        || !angular_velocity.is_finite()
        || !gravity.is_finite()
        || !specific_force.is_finite()
    {
        return Err(InvalidImuInput);
    }
    Ok(ImuReading {
        orientation: (reference.inverse() * world).to_array(),
        specific_force: specific_force.to_array(),
        angular_velocity: angular_velocity.to_array(),
    })
}
