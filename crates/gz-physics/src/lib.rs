// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared f64 construction and state values, and the dynamic backend contract.

mod backend;
pub use backend::{
    BodyId, DynamicBackend, InitialPhysicsState, JointId, JointState, PhysicsBackend, PhysicsError,
    PhysicsReset, PreparedPhysicsReset, RestoredPhysics, install,
};

#[derive(Debug, Clone, Copy)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    /// Radius in X/Y and half-height along Gazebo Z.
    Cylinder {
        radius: f64,
        half_height: f64,
    },
    /// Spherical caps around a Z-axis segment of length twice `half_length`.
    Capsule {
        radius: f64,
        half_length: f64,
    },
    Box {
        half_extents: [f64; 3],
    },
}

/// Fixed local Z-up plane. `size` is the SDF XY metadata; collision extent is
/// backend-specific. `position` and XYZW `orientation` locate the link frame.
#[derive(Debug, Clone, Copy)]
pub struct PlaneSpec {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub size: [f64; 2],
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
