// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Differential wheel geometry and encoder odometry; physics and IO stay external.
use std::{collections::VecDeque, time::Duration};

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(super) enum InvalidDrive {
    #[error("wheel radius and separation must have finite representable positive geometry")]
    Geometry,
    #[error("drive command and wheel target bounds must be finite and representable")]
    Command,
    #[error("wheel encoder state or odometry result is not finite and representable")]
    State,
    #[error("encoder observation time must advance monotonically")]
    Time,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Geometry {
    radius: f64,
    separation: f64,
}

impl Geometry {
    pub(super) fn new(radius: f64, separation: f64) -> Result<Self, InvalidDrive> {
        if !radius.is_finite()
            || radius <= 0.0
            || !(1.0 / radius).is_finite()
            || !separation.is_finite()
            || separation <= 0.0
            || separation * 0.5 == 0.0
            || !(1.0 / separation).is_finite()
        {
            return Err(InvalidDrive::Geometry);
        }
        Ok(Self { radius, separation })
    }

    /// +X forward, +Z yaw, left at +Y. Both declared wheel axes are +Y.
    pub(super) fn targets(self, linear: f64, angular: f64) -> Result<[f64; 2], InvalidDrive> {
        let turn = angular * (self.separation * 0.5);
        let result = [(linear - turn) / self.radius, (linear + turn) / self.radius];
        if ![linear, angular, turn]
            .into_iter()
            .chain(result)
            .all(f64::is_finite)
        {
            return Err(InvalidDrive::Command);
        }
        Ok(result)
    }

    /// Uniformly scale wheel targets when necessary, preserving their commanded
    /// curvature. This bounds targets, not physical speeds under external impulses.
    pub(super) fn bounded_targets(
        self,
        linear: f64,
        angular: f64,
        limits: [f64; 2],
    ) -> Result<[f64; 2], InvalidDrive> {
        if !limits
            .into_iter()
            .all(|limit| limit.is_finite() && limit > 0.0)
        {
            return Err(InvalidDrive::Command);
        }
        let targets = self.targets(linear, angular)?;
        let ratio = (targets[0].abs() / limits[0]).max(targets[1].abs() / limits[1]);
        if !ratio.is_finite() {
            return Err(InvalidDrive::Command);
        }
        let scale = ratio.max(1.0);
        // Final clamps cover a one-ULP rounding overshoot in the ratio/division.
        Ok([
            (targets[0] / scale).clamp(-limits[0], limits[0]),
            (targets[1] / scale).clamp(-limits[1], limits[1]),
        ])
    }

    fn displacement(self, angle_delta: [f64; 2]) -> Result<[f64; 2], InvalidDrive> {
        let distances = angle_delta.map(|angle| angle * self.radius);
        // Avoid overflowing a sum that has a representable mean.
        let travel = distances[0] * 0.5 + distances[1] * 0.5;
        let rotation = (distances[1] - distances[0]) / self.separation;
        if !angle_delta
            .into_iter()
            .chain(distances)
            .chain([travel, rotation])
            .all(f64::is_finite)
        {
            return Err(InvalidDrive::State);
        }
        Ok([travel, rotation])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Reading {
    /// Encoder-integrated pose in the initial odometry frame, metres/metres/radians.
    pub pose: [f64; 3],
    /// Mean of up to ten measured interval velocities, matching the admitted window.
    pub twist: [f64; 2],
}

#[derive(Clone, Debug)]
pub(super) struct Odometry {
    geometry: Geometry,
    angles: [f64; 2],
    time: Duration,
    pose: [f64; 3],
    velocities: VecDeque<[f64; 2]>,
}

impl Odometry {
    pub(super) fn new(
        geometry: Geometry,
        angles: [f64; 2],
        time: Duration,
    ) -> Result<Self, InvalidDrive> {
        if !angles.into_iter().all(f64::is_finite) {
            return Err(InvalidDrive::State);
        }
        Ok(Self {
            geometry,
            angles,
            time,
            pose: [0.0; 3],
            velocities: VecDeque::with_capacity(10),
        })
    }

    /// Seed from actual restored wheel coordinates; reset owns a new local epoch.
    pub(super) fn reset(&mut self, angles: [f64; 2], time: Duration) -> Result<(), InvalidDrive> {
        let restored = Self::new(self.geometry, angles, time)?;
        *self = restored;
        Ok(())
    }

    pub(super) fn observe(
        &mut self,
        angles: [f64; 2],
        now: Duration,
    ) -> Result<Reading, InvalidDrive> {
        let dt = now
            .checked_sub(self.time)
            .filter(|dt| !dt.is_zero())
            .ok_or(InvalidDrive::Time)?
            .as_secs_f64();
        let delta = [angles[0] - self.angles[0], angles[1] - self.angles[1]];
        let [travel, rotation] = self.geometry.displacement(delta)?;
        let half = rotation * 0.5;
        let sinc = if half == 0.0 { 1.0 } else { half.sin() / half };
        let bearing = self.pose[2] + half;
        let yaw = self.pose[2] + rotation;
        let pose = [
            self.pose[0] + travel * sinc * bearing.cos(),
            self.pose[1] + travel * sinc * bearing.sin(),
            yaw.sin().atan2(yaw.cos()),
        ];
        let twist = [travel / dt, rotation / dt];
        if !angles
            .into_iter()
            .chain(pose)
            .chain(twist)
            .all(f64::is_finite)
        {
            return Err(InvalidDrive::State);
        }
        // Compute before changing state: even an unrepresentable mean leaves
        // coordinates, time, pose and history untouched.
        let count = (self.velocities.len() + 1).min(10);
        let skip = usize::from(self.velocities.len() == 10);
        let mut mean = twist.map(|value| value / count as f64);
        for previous in self.velocities.iter().skip(skip) {
            for axis in 0..2 {
                mean[axis] += previous[axis] / count as f64;
            }
        }
        if !mean.into_iter().all(f64::is_finite) {
            return Err(InvalidDrive::State);
        }
        if self.velocities.len() == 10 {
            self.velocities.pop_front();
        }
        self.velocities.push_back(twist);
        self.angles = angles;
        self.time = now;
        self.pose = pose;
        Ok(Reading { pose, twist: mean })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wheel_targets_preserve_forward_turn_signs_and_bounded_curvature() {
        let geometry = Geometry::new(0.2, 0.5).unwrap();
        assert_eq!(geometry.targets(1.0, 0.0).unwrap(), [5.0, 5.0]);
        assert_eq!(geometry.targets(0.0, 2.0).unwrap(), [-2.5, 2.5]);
        assert_eq!(geometry.targets(1.0, 2.0).unwrap(), [2.5, 7.5]);
        assert_eq!(
            geometry.bounded_targets(1.0, 2.0, [5.0, 3.0]).unwrap(),
            [1.0, 3.0]
        );
        assert_eq!(
            geometry.bounded_targets(-1.0, -2.0, [5.0, 3.0]).unwrap(),
            [-1.0, -3.0]
        );
        assert_eq!(
            geometry.bounded_targets(0.0, 0.0, [1.0, 1.0]).unwrap(),
            [0.0, 0.0]
        );
        assert_eq!(
            geometry.bounded_targets(0.2, 0.0, [2.0, 2.0]).unwrap(),
            [1.0, 1.0]
        );
    }
    #[test]
    fn continuous_encoder_integration_has_exact_straight_circle_and_reset_frames() {
        let geometry = Geometry::new(1.0, 2.0).unwrap();
        let mut odom = Odometry::new(geometry, [12.0, -4.0], Duration::ZERO).unwrap();
        let reading = odom.observe([14.0, -2.0], Duration::from_secs(2)).unwrap();
        assert_eq!(
            reading,
            Reading {
                pose: [2.0, 0.0, 0.0],
                twist: [1.0, 0.0]
            }
        );
        odom.reset([12.0, -4.0], Duration::ZERO).unwrap();
        let reading = odom
            .observe([12.0, -4.0 + std::f64::consts::PI], Duration::from_secs(1))
            .unwrap();
        for (actual, expected) in
            reading
                .pose
                .into_iter()
                .zip([1.0, 1.0, std::f64::consts::FRAC_PI_2])
        {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert!((reading.twist[0] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((reading.twist[1] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        odom.reset([0.0; 2], Duration::ZERO).unwrap();
        let reading = odom.observe([-1.0, 1.0], Duration::from_secs(1)).unwrap();
        assert_eq!(reading.pose[..2], [0.0; 2]);
        assert!((reading.pose[2] - 1.0).abs() < 1e-12);
        assert_eq!(reading.twist, [0.0, 1.0]);
    }
    #[test]
    fn velocity_window_is_bounded_measured_and_clears_on_reset() {
        let geometry = Geometry::new(1.0, 2.0).unwrap();
        let mut odom = Odometry::new(geometry, [0.0; 2], Duration::ZERO).unwrap();
        assert_eq!(
            odom.observe([1.0; 2], Duration::from_secs(1))
                .unwrap()
                .twist,
            [1.0, 0.0]
        );
        assert_eq!(
            odom.observe([4.0; 2], Duration::from_secs(2))
                .unwrap()
                .twist,
            [2.0, 0.0]
        );
        for index in 3..=13 {
            odom.observe([index as f64 + 2.0; 2], Duration::from_secs(index))
                .unwrap();
        }
        assert_eq!(odom.velocities.len(), 10);
        assert!(
            (odom
                .observe([16.0; 2], Duration::from_secs(14))
                .unwrap()
                .twist[0]
                - 1.0)
                .abs()
                < 1e-12
        );
        odom.reset([0.0; 2], Duration::ZERO).unwrap();
        assert!(odom.velocities.is_empty());
        assert_eq!(
            odom.observe([3.0; 2], Duration::from_secs(1))
                .unwrap()
                .twist,
            [3.0, 0.0]
        );
    }
    #[test]
    fn malformed_geometry_commands_and_nonadvancing_or_nonfinite_encoders_are_atomic() {
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::from_bits(1)] {
            assert_eq!(Geometry::new(value, 1.0), Err(InvalidDrive::Geometry));
            assert_eq!(Geometry::new(1.0, value), Err(InvalidDrive::Geometry));
        }
        let geometry = Geometry::new(0.2, 0.5).unwrap();
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(geometry.targets(value, 0.0).is_err());
            assert!(geometry.targets(0.0, value).is_err());
            assert!(geometry.bounded_targets(1.0, 0.0, [value, 1.0]).is_err());
            assert!(Odometry::new(geometry, [value, 0.0], Duration::ZERO).is_err());
        }
        assert!(geometry.targets(f64::MAX, 0.0).is_err());
        assert!(geometry.bounded_targets(1.0, 0.0, [0.0, 1.0]).is_err());
        assert!(
            geometry
                .bounded_targets(1.0, 0.0, [f64::from_bits(1), 1.0])
                .is_err()
        );
        let mut odom = Odometry::new(geometry, [0.0; 2], Duration::from_secs(1)).unwrap();
        assert_eq!(
            odom.observe([1.0; 2], Duration::from_secs(1)),
            Err(InvalidDrive::Time)
        );
        assert_eq!(
            odom.observe([1.0; 2], Duration::ZERO),
            Err(InvalidDrive::Time)
        );
        assert_eq!(
            odom.observe([f64::NAN, 0.0], Duration::from_secs(2)),
            Err(InvalidDrive::State)
        );
        assert_eq!(
            odom.reset([f64::INFINITY, 0.0], Duration::ZERO),
            Err(InvalidDrive::State)
        );
        assert_eq!(
            odom.observe([1.0; 2], Duration::from_secs(2)).unwrap().pose,
            [0.2, 0.0, 0.0]
        );
    }
}
