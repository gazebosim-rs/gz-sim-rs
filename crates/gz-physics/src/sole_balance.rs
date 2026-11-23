// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Foot balance observables from committed world-space panel reactions.
use crate::sole_supports::PanelReaction;
use glam::{DQuat, DVec2, DVec3};

/// A six-axis contact reading on the support, in the sensor's local axes.
#[derive(Debug, Clone, Copy)]
pub struct SupportWrench {
    pub force_n: DVec3,
    pub moment_nm: DVec3,
}

impl SupportWrench {
    /// Translate the moment from the foot COM to the sensor origin and reverse
    /// action/reaction before rotating into the sensor frame.
    pub fn at_sensor(
        row: &PanelReaction,
        foot_com: DVec3,
        sensor: DVec3,
        axes: DQuat,
    ) -> Option<Self> {
        if !foot_com.is_finite()
            || !sensor.is_finite()
            || !axes.is_finite()
            || (axes.length_squared() - 1.).abs() > 1e-10
        {
            return None;
        }
        let c = row.on_sole.contact;
        let local = axes.conjugate();
        let result = Self {
            force_n: local * (-c.force),
            moment_nm: local * (-c.torque - (foot_com - sensor).cross(c.force)),
        };
        (result.force_n.is_finite() && result.moment_nm.is_finite()).then_some(result)
    }

    /// Point on sensor z=0 where the tangential components of the measured
    /// moment vanish. The residual torque about the normal is preserved.
    pub fn zero_moment_point(self, minimum_normal_n: f64) -> Option<(DVec2, f64)> {
        let normal = self.force_n.z;
        if !self.force_n.is_finite()
            || !self.moment_nm.is_finite()
            || !minimum_normal_n.is_finite()
            || minimum_normal_n < 0.
            || normal.abs() <= minimum_normal_n
        {
            return None;
        }
        let p = DVec2::new(-self.moment_nm.y, self.moment_nm.x) / normal;
        let twist = self.moment_nm.z - p.perp_dot(self.force_n.truncate());
        (p.is_finite() && twist.is_finite()).then_some((p, twist))
    }
}

/// The convex support polygon expressed on a plane perpendicular to the common
/// panel normal. Vertices and margins are in metres in that plane.
#[derive(Debug)]
pub struct SupportFootprint {
    origin: DVec3,
    axes: DQuat,
    hull: Vec<DVec2>,
    pub pressure_center: DVec2,
    pub normal_force_n: f64,
}

fn append(chain: &mut Vec<DVec2>, p: DVec2) {
    while chain.len() >= 2 {
        let a = chain[chain.len() - 2];
        let b = chain[chain.len() - 1];
        if (b - a).perp_dot(p - a) > 0. {
            break;
        }
        chain.pop();
    }
    chain.push(p);
}

impl SupportFootprint {
    /// Retain only panels carrying more than the specified normal load. An
    /// empty/degenerate polygon has no finite-area balance margin.
    pub fn from_panels(
        rows: &[PanelReaction],
        origin: DVec3,
        axes: DQuat,
        minimum_n: f64,
    ) -> Option<Self> {
        if !origin.is_finite()
            || !axes.is_finite()
            || (axes.length_squared() - 1.).abs() > 1e-10
            || !minimum_n.is_finite()
            || minimum_n < 0.
        {
            return None;
        }
        let project = |p: DVec3| (axes.conjugate() * (p - origin)).truncate();
        let mut points = vec![];
        let mut force = 0.;
        let mut center = DVec2::ZERO;
        for row in rows {
            let c = &row.on_sole.contact;
            if !c.normal_force.is_finite() || c.normal_force < 0. {
                return None;
            }
            if c.normal_force <= minimum_n {
                continue;
            }
            let point = project(c.center_of_pressure?);
            force += c.normal_force;
            center += point * c.normal_force;
            points.extend(c.polygon().iter().copied().map(project));
        }
        if force == 0.
            || !force.is_finite()
            || !center.is_finite()
            || points.iter().any(|p| !p.is_finite())
        {
            return None;
        }
        points.sort_by(|a, b| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)));
        points.dedup();
        let mut lower = vec![];
        let mut upper = vec![];
        for &p in &points {
            append(&mut lower, p)
        }
        for &p in points.iter().rev() {
            append(&mut upper, p)
        }
        lower.pop();
        upper.pop();
        lower.extend(upper);
        (lower.len() >= 3).then_some(Self {
            origin,
            axes,
            hull: lower,
            pressure_center: center / force,
            normal_force_n: force,
        })
    }
    pub fn vertices(&self) -> &[DVec2] {
        &self.hull
    }
    /// Positive inside the convex hull. Outside, this is the most violated
    /// edge half-space distance, not the Euclidean distance to a corner.
    pub fn margin_m(&self, point: DVec2) -> Option<f64> {
        let mut min = f64::INFINITY;
        for (&a, &b) in self.hull.iter().zip(self.hull.iter().cycle().skip(1)) {
            min = min.min((b - a).normalize().perp_dot(point - a));
        }
        (point.is_finite() && min.is_finite()).then_some(min)
    }
    /// Intersect a world-space load line with the support plane.
    pub fn load_line(&self, point: DVec3, direction: DVec3) -> Option<DVec2> {
        let local = self.axes.conjugate() * (point - self.origin);
        let dir = self.axes.conjugate() * direction;
        if dir.z == 0. {
            return None;
        }
        let p = local.truncate() - dir.truncate() * (local.z / dir.z);
        p.is_finite().then_some(p)
    }
}
