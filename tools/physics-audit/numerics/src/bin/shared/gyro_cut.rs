// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recheck archived corner roots using fixed world angular momentum.
use super::*;
pub(super) fn verify(c: &V, cut: &V, time_ns: f64, corner: usize) {
    let start = u(&cut["ns"]) as f64;
    assert!(time_ns >= start && time_ns - start < 1_001_000.);
    let duration = (time_ns - start) * 1e-9;
    let foot = &cut["foot"];
    let pad = &cut["pad"];
    let t = tensor(c);
    let inv = t.inverse();
    let mut rot = q(&foot["orientation"]);
    let w = v(&foot["angular_velocity"]);
    let momentum = rot * (t * (rot.conjugate() * w));
    let n = q(&pad["orientation"]) * DVec3::Z;
    let center =
        v(&foot["position"]) + rot * DVec3::new(0., 0., 0.07) - v(&pad["position"]) - n * 0.06
            + (v(&foot["velocity"]) - v(&pad["velocity"])) * duration;
    let rate = |q: DQuat| {
        let q = q.normalize();
        let w = q * (inv * (q.conjugate() * momentum));
        DQuat::from_xyzw(w.x, w.y, w.z, 0.) * q * 0.5
    };
    let steps = (duration / 4e-6).ceil().max(1.) as usize;
    let h = duration / steps as f64;
    for _ in 0..steps {
        let k1 = rate(rot);
        let k2 = rate(rot + k1 * (0.5 * h));
        let k3 = rate(rot + k2 * (0.5 * h));
        let k4 = rate(rot + k3 * h);
        rot = (rot + (k1 + k2 * 2. + k3 * 2. + k4) * (h / 6.)).normalize();
    }
    for k in 0..4 {
        let r = DVec3::new(
            if k >= 2 { 0.18 } else { -0.18 },
            if k % 2 == 1 { 0.1 } else { -0.1 },
            -0.13,
        );
        let z = n.dot(center + rot * r);
        if k == corner {
            if duration == 0. {
                assert!(z <= 1e-12)
            } else {
                assert!(z.abs() < 3e-12, "root height {z}");
            }
        } else {
            assert!(z >= -1e-10);
        }
    }
}
