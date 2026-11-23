// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{self, Q, V};
use gz_physics_gpu::{SolidSphere, SupportedSpherePairLane as Lane};
pub fn fixture(count: usize) -> Vec<Lane> {
    reference::fixture(count)
        .into_iter()
        .enumerate()
        .map(|(i, mut pair)| {
            let pick = |s: usize, o: usize| ((i * s + o) % 2048) as f64 / 2047.;
            let azimuth = std::f64::consts::TAU * pick(509, 445);
            let n = V::new(0.22 * azimuth.cos(), 0.30 * azimuth.sin(), 1.).normalize();
            let x = (V::X - n * n.x).normalize();
            let y = n.cross(x);
            let angle = if i < 8 {
                0.
            } else {
                std::f64::consts::TAU * pick(941, 61)
            };
            let heading = x * angle.cos() + y * angle.sin();
            let side = n.cross(heading);
            let offset = 0.16 * (pick(629, 111) - 0.5);
            let radius = pair.a.radius;
            let speed = 1.4 + 1.5 * pick(223, 791);
            let gap_ratio = match i % 8 {
                0 => 0.,
                7 => 1.10,
                _ => 0.16 + 0.70 * pick(617, 197),
            };
            let b = 2. * radius * gap_ratio;
            let approach = 0.09 + 0.19 * pick(653, 515);
            let distance = (4. * radius * radius - b * b).max(0.).sqrt() + speed * approach;
            let d = heading * distance + side * b;
            let m = [1. / pair.a.inverse_mass, 1. / pair.b.inverse_mass];
            let total = m[0] + m[1];
            let center = n * (offset + radius) + x * (0.04 * pick(503, 821)) - y * 0.02;
            let transport =
                x * (0.10 + 0.10 * pick(199, 503)) + y * (-0.08 + 0.16 * pick(499, 919));
            for (state, ratio, spin) in [
                (&mut pair.a, -m[1] / total, -5. + 12. * pick(571, 347)),
                (&mut pair.b, m[0] / total, -9. + 13. * pick(811, 661)),
            ] {
                state.position = (center + d * ratio).to_array();
                state.velocity = (transport - heading * (ratio * speed)).to_array();
                state.angular_velocity = (n * spin).to_array();
                state.radius = radius;
            }
            pair.gravity = (-n * (7.6 + 2.8 * pick(443, 911))
                + x * (-1.5 + 3. * pick(479, 337))
                + y * (-0.6 + 1.2 * pick(829, 131)))
            .to_array();
            // Preserve independent lane identities and orientation colors from the free fixture.
            for sphere in [&mut pair.a, &mut pair.b] {
                sphere.orientation = Q::from_array(sphere.orientation).normalize().to_array();
            }
            Lane {
                pair,
                plane_normal: n.to_array(),
                plane_offset: offset,
                support_normal_impulses: [0.; 2],
            }
        })
        .collect()
}
pub fn tangent_gravity(s: &Lane) -> V {
    let n = V::from(s.plane_normal);
    let g = V::from(s.pair.gravity);
    g - g.dot(n) * n
}
pub fn scaled(mut s: Lane, factor: u32) -> Lane {
    let f = f64::from(factor);
    for b in [&mut s.pair.a, &mut s.pair.b] {
        b.velocity = b.velocity.map(|x| x / f);
        b.angular_velocity = b.angular_velocity.map(|x| x / f);
    }
    s.pair.gravity = s.pair.gravity.map(|x| x / (f * f));
    s
}
pub fn physical(mut b: SolidSphere, f: u32) -> SolidSphere {
    b.velocity = b.velocity.map(|x| x * f64::from(f));
    b.angular_velocity = b.angular_velocity.map(|x| x * f64::from(f));
    b
}
pub fn cpu_step(s: &mut Lane) {
    let g = s.pair.gravity;
    let n = V::from(s.plane_normal);
    let gn = V::from(g).dot(n);
    s.support_normal_impulses = [
        -gn * 0.001 / s.pair.a.inverse_mass,
        -gn * 0.001 / s.pair.b.inverse_mass,
    ];
    s.pair.gravity = tangent_gravity(s).to_array();
    reference::analytical(&mut s.pair);
    s.pair.gravity = g;
}
