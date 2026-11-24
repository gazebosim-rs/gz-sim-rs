// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local geometry of rounded cuboid contacts.
use glam::DVec3;

/// Signed distance and outward witness, including deterministic ties inside.
pub(super) fn witness(p: DVec3, half: DVec3) -> (f64, DVec3, DVec3) {
    let q = p.clamp(-half, half);
    let outside = p - q;
    let distance = outside.length();
    if distance > 0. {
        return (distance, outside / distance, q);
    }
    let d = p.abs() - half;
    let axis = if d.x >= d.y && d.x >= d.z {
        0
    } else if d.y >= d.z {
        1
    } else {
        2
    };
    let mut normal = DVec3::ZERO;
    normal[axis] = if p[axis] < 0. { -1. } else { 1. };
    (d[axis], normal, p - d[axis] * normal)
}

/// Squared distance to a box is piecewise quadratic along a segment. Break
/// at its six slabs. On each interval test the exact face, edge or corner arc.
pub(super) fn linear_entry(a: DVec3, b: DVec3, half: DVec3, radius: f64) -> Option<f64> {
    let delta = b - a;
    let mut knots = [0.; 8];
    knots[1] = 1.;
    let mut count = 2;
    for axis in 0..3 {
        if delta[axis] != 0. {
            for face in [-half[axis], half[axis]] {
                let t = (face - a[axis]) / delta[axis];
                if t > 0. && t < 1. {
                    knots[count] = t;
                    count += 1;
                }
            }
        }
    }
    knots[..count].sort_unstable_by(f64::total_cmp);
    for range in knots[..count].windows(2) {
        let (lo, hi) = (range[0], range[1]);
        let p = a + lo * delta;
        let midpoint = a + (0.5 * (lo + hi)) * delta;
        let mut offset = DVec3::ZERO;
        let mut velocity = DVec3::ZERO;
        for axis in 0..3 {
            if midpoint[axis].abs() > half[axis] {
                offset[axis] = p[axis] - midpoint[axis].signum() * half[axis];
                velocity[axis] = delta[axis];
            }
        }
        if offset.length() <= radius {
            return Some(lo);
        }
        let speed = velocity.length();
        if speed == 0. {
            continue;
        }
        let direction = velocity / speed;
        let along = -offset.dot(direction);
        if along < 0. {
            continue;
        }
        let perp = (offset + along * direction).length();
        let rounding = 16. * f64::EPSILON * radius.max(offset.length());
        if perp <= radius + rounding {
            let chord = if (radius - perp).abs() <= rounding {
                0.
            } else {
                ((radius - perp) * (radius + perp)).sqrt()
            };
            let t = lo + (along - chord).max(0.) / speed;
            if t <= hi {
                return Some(t);
            }
        }
    }
    None
}
