// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Analytic ballistic drop, constrained spherical slide and unilateral release.
use glam::DVec3;
pub const G: f64 = 9.81;
pub const R: f64 = 0.112;
pub const HEIGHT: f64 = 0.020;
pub const SPEED: f64 = 0.52;
pub const CENTRES: [DVec3; 2] = [DVec3::new(0.32, -0.29, 0.58), DVec3::new(0.32, 0.29, 0.58)];
pub const REST: &str = include_str!("../../../../examples/articulated_supported_cable.sdf");
pub const RELEASE: &str = include_str!("../../../../examples/articulated_releasing_cable.sdf");
pub fn onset() -> f64 {
    (2. * HEIGHT / G).sqrt()
}
pub fn release_angle(v: f64) -> f64 {
    ((2. + v * v / (G * R)) / 3.).min(1.).acos()
}
fn speed(a: f64, v: f64) -> f64 {
    (v * v + 2. * G * R * (1. - a.cos())).sqrt()
}
pub fn travel(a: f64, v: f64) -> f64 {
    // Simpson integration of dt/dθ = R/sqrt(v0² + 2gR(1−cosθ)).
    let n = 128;
    (0..=n)
        .map(|i| {
            let k = if i == 0 || i == n {
                1.
            } else if i % 2 == 0 {
                2.
            } else {
                4.
            };
            k / speed(a * i as f64 / n as f64, v)
        })
        .sum::<f64>()
        * a
        * R
        / (3. * n as f64)
}
pub fn detaches(v: f64) -> f64 {
    onset() + travel(release_angle(v), v)
}
pub struct Exact {
    pub position: DVec3,
    pub velocity: DVec3,
    pub normal_force: DVec3,
}
pub fn exact(t: f64, v: f64, site: usize, mass: f64) -> Exact {
    let t0 = onset();
    let c = CENTRES[site];
    if t < t0 {
        return Exact {
            position: c + DVec3::new(v * (t - t0), 0., R + HEIGHT - 0.5 * G * t * t),
            velocity: DVec3::new(v, 0., -G * t),
            normal_force: DVec3::ZERO,
        };
    }
    if v == 0. {
        return Exact {
            position: c + R * DVec3::Z,
            velocity: DVec3::ZERO,
            normal_force: mass * G * DVec3::Z,
        };
    }
    let angle = release_angle(v);
    let end = t0 + travel(angle, v);
    let theta = if t >= end {
        angle
    } else {
        let (mut lo, mut hi) = (0., angle);
        for _ in 0..50 {
            let mid = (lo + hi) / 2.;
            if t - t0 > travel(mid, v) {
                lo = mid
            } else {
                hi = mid
            }
        }
        (hi + lo) / 2.
    };
    let (s, k) = theta.sin_cos();
    let n = DVec3::new(s, 0., k);
    let speed = speed(theta, v);
    let tangent = DVec3::new(k, 0., -s) * speed;
    let h = (t - end).max(0.);
    Exact {
        position: c + R * n + h * tangent - 0.5 * G * h * h * DVec3::Z,
        velocity: tangent - G * h * DVec3::Z,
        normal_force: if t < end {
            mass * (G * k - speed * speed / R) * n
        } else {
            DVec3::ZERO
        },
    }
}
pub fn launched(v: f64) -> String {
    let old = 0.32 - SPEED * onset();
    let new = 0.32 - v * onset();
    // Preserve every authored property other than approach translation.
    let mut s = RELEASE.to_string();
    for line in RELEASE
        .lines()
        .filter(|l| l.contains("<particle position="))
    {
        let start = line.find("position=\"").unwrap() + 10;
        let stop = start + line[start..].find(' ').unwrap();
        let actual = line[start..stop].parse::<f64>().unwrap();
        assert!((actual - old).abs() < 1e-12);
        s = s.replace(&line[start..stop], &new.to_string());
    }
    s.replace("velocity=\"0.52 0 0\"", &format!("velocity=\"{v} 0 0\""))
}
