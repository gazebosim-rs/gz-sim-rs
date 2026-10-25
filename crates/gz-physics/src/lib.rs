// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared f64 construction and state values for physics adapters.

#[derive(Debug, Clone, Copy)]
pub enum Shape {
    Sphere { radius: f64 },
    Box { half_extents: [f64; 3] },
}

/// Initial state, in meters, kilograms, and meters per second. `None` mass is fixed.
#[derive(Debug, Clone, Copy)]
pub struct BodySpec {
    pub shape: Shape,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub mass: Option<f64>,
}

/// A link's inertial frame, relative to its body frame. Tensor order is xx,yy,zz,xy,xz,yz.
#[derive(Debug, Clone, Copy)]
pub struct Inertia {
    pub center: [f64; 3],
    pub orientation: [f64; 4],
    pub tensor: [f64; 6],
}

/// Rotational state. Quaternions use [x,y,z,w]; angular velocity is in world axes.
#[derive(Debug, Clone, Copy)]
pub struct RotationSpec {
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

impl Default for RotationSpec {
    fn default() -> Self {
        Self {
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0; 3],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    /// Body origin in world axes.
    pub position: [f64; 3],
    /// Center-of-mass velocity in world axes; differs from origin velocity for offset COM.
    pub velocity: [f64; 3],
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

/// Extension identity shared by simulator physics installers.
/// Claimed outside mutable ECS resources so removing a resource cannot reinstall
/// a second stepping system in the same simulation.
pub struct PhysicsInstallation;
