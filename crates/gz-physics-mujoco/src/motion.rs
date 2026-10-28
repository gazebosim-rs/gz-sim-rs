// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-form conversions between world velocities and MuJoCo coordinates.
//!
//! Each body is an independent world child with at most one joint, so these need
//! no native call: edits can convert and check velocities before anything changes.

use crate::native::Inertial;
use crate::validate::{cross, quaternion_rotate, quaternion_rotate_inverse};

/// MuJoCo's `mjMAXVAL`: a larger qpos or qvel entry makes MuJoCo reset its data.
pub(crate) const MAX_VALUE: f64 = 1e10;

/// Whether MuJoCo keeps these generalized values instead of resetting.
pub(crate) fn representable(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite() && v.abs() <= MAX_VALUE)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn difference(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Body-axis inertia about the COM applied to `vector` (body axes).
fn inertia_times(inertial: &Inertial, vector: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = inertial.quaternion;
    let frame = [x, y, z, w];
    let local = quaternion_rotate_inverse(frame, vector);
    quaternion_rotate(frame, [0, 1, 2].map(|i| inertial.principal[i] * local[i]))
}

/// Free-joint qvel for a world COM velocity and world angular velocity of a body
/// oriented by `rotation` ([x,y,z,w]): MuJoCo stores the body-origin velocity
/// `v - w x (R c)` in world axes and `R^T w` in body axes.
pub(crate) fn free_coordinates(
    rotation: [f64; 4],
    center: [f64; 3],
    linear: [f64; 3],
    angular: [f64; 3],
) -> [f64; 6] {
    let spin = cross(angular, quaternion_rotate(rotation, center));
    let [ax, ay, az] = quaternion_rotate_inverse(rotation, angular);
    [
        linear[0] - spin[0],
        linear[1] - spin[1],
        linear[2] - spin[2],
        ax,
        ay,
        az,
    ]
}

/// A hinge in its body's frame: anchor point and unit axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Hinge {
    pub anchor: [f64; 3],
    pub axis: [f64; 3],
}

impl Hinge {
    /// Hinge rate for a world COM velocity and world angular velocity or, with
    /// `impulse`, its change for a world COM impulse and angular impulse:
    /// J^T h / (J^T M J), the projection a constraint impulse produces. With the
    /// COM offset r = c - a, J maps a rate to angular u and linear u x r.
    pub(crate) fn rate(
        &self,
        inertial: &Inertial,
        rotation: [f64; 4],
        linear: [f64; 3],
        angular: [f64; 3],
        impulse: bool,
    ) -> f64 {
        let linear = quaternion_rotate_inverse(rotation, linear);
        let angular = quaternion_rotate_inverse(rotation, angular);
        let lever = cross(self.axis, difference(inertial.center, self.anchor));
        let effective =
            dot(self.axis, inertia_times(inertial, self.axis)) + inertial.mass * dot(lever, lever);
        let momentum = if impulse {
            dot(self.axis, angular) + dot(lever, linear)
        } else {
            dot(self.axis, inertia_times(inertial, angular)) + inertial.mass * dot(lever, linear)
        };
        momentum / effective
    }

    /// World [angular, linear] COM velocity when turning at `rate`.
    pub(crate) fn world_velocity(
        &self,
        inertial: &Inertial,
        rotation: [f64; 4],
        rate: f64,
    ) -> [f64; 6] {
        let angular = quaternion_rotate(rotation, self.axis.map(|c| c * rate));
        let offset = quaternion_rotate(rotation, difference(inertial.center, self.anchor));
        let [vx, vy, vz] = cross(angular, offset);
        [angular[0], angular[1], angular[2], vx, vy, vz]
    }
}

/// World angular velocity change I_world^-1 L for a free body oriented by `rotation`.
pub(crate) fn angular_impulse_response(
    inertial: &Inertial,
    rotation: [f64; 4],
    impulse: [f64; 3],
) -> [f64; 3] {
    let [w, x, y, z] = inertial.quaternion;
    let frame = [x, y, z, w];
    let principal = quaternion_rotate_inverse(frame, quaternion_rotate_inverse(rotation, impulse));
    let scaled = [0, 1, 2].map(|i| principal[i] / inertial.principal[i]);
    quaternion_rotate(rotation, quaternion_rotate(frame, scaled))
}
