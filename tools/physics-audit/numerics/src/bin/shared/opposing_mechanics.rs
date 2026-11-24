// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::measures::*;
use glam::{DMat3, DQuat, DVec3};
use serde_json::Value as V;
use std::{fs, path::Path};
pub(super) const OFFSET: DVec3 = DVec3::new(0., 0., 0.07);
const HALF: DVec3 = DVec3::new(0.18, 0.10, 0.06);
pub(super) fn read(p: &Path) -> V {
    serde_json::from_reader(fs::File::open(p).unwrap()).unwrap()
}
pub(super) fn inertia(d: &V) -> DMat3 {
    DMat3::from_cols_array(&serde_json::from_value(d["tensor"].clone()).unwrap())
}
pub(super) fn mom(q: DQuat, i: DMat3, w: DVec3) -> DVec3 {
    q * (i * (q.conjugate() * w))
}
#[derive(Clone, Copy)]
pub(super) struct Cut {
    pub(super) q: DQuat,
    pub(super) c: DVec3,
    pub(super) v: DVec3,
    pub(super) l: DVec3,
}
impl Cut {
    pub(super) fn read(x: &V, i: DMat3) -> Self {
        let q = quat(&x["orientation"]);
        Self {
            q,
            c: vec(&x["position"]) + q * OFFSET,
            v: vec(&x["velocity"]),
            l: mom(q, i, vec(&x["angular_velocity"])),
        }
    }
    pub(super) fn step(&mut self, dt: f64, gravity: DVec3, inv: DMat3) {
        let rhs = |q: DQuat| {
            let u = q.normalize();
            let w = mom(u, inv, self.l);
            DQuat::from_xyzw(w.x, w.y, w.z, 0.) * u * 0.5
        };
        let a = rhs(self.q);
        let b = rhs(self.q + a * (dt / 2.));
        let c = rhs(self.q + b * (dt / 2.));
        let d = rhs(self.q + c * dt);
        self.q = (self.q + (a + (b + c) * 2. + d) * (dt / 6.)).normalize();
        self.c += (self.v + 0.5 * gravity * dt) * dt;
        self.v += gravity * dt;
    }
}
pub(super) fn corner(k: usize) -> DVec3 {
    DVec3::new(
        if k & 2 == 0 { -HALF.x } else { HALF.x },
        if k & 1 == 0 { -HALF.y } else { HALF.y },
        if k & 4 == 0 { -HALF.z } else { HALF.z },
    ) - OFFSET
}
fn geometry(d: &V, c: &V, h: &V, cut: Cut, t: f64, released: bool) -> (f64, f64, f64, f64) {
    let q = quat(&d["primary"]["pad_q"]);
    let n = q * DVec3::Z;
    let floor = vec(&d["pad_velocity"]) * t;
    let gate = vec(&c["gate_center0"]) + vec(&c["gate_velocity"]) * t - 0.025 * n;
    let mut min = f64::INFINITY;
    let mut active = 0.;
    let mut previous = f64::INFINITY;
    let mut margin = f64::INFINITY;
    for k in 0..8 {
        let p = cut.c + cut.q * corner(k);
        let a = q.conjugate() * (p - floor);
        let b = q.conjugate() * (p - gate);
        let returned = h["upper_corner"].as_u64().map(|c| c as usize);
        let selected = int(&h["corner"]) as usize;
        if returned.is_some() && k == selected {
            active = a.z;
        } else {
            min = min.min(a.z);
        }
        if returned.is_none() && k == selected {
            active = -b.z;
        } else if Some(k) == returned {
            previous = -b.z;
            if !released {
                min = min.min(previous);
            }
        } else {
            min = min.min(-b.z);
        }
        margin = margin
            .min(0.33 - a.x.abs())
            .min(0.18 - a.y.abs())
            .min(0.48 - b.x.abs())
            .min(0.42 - b.y.abs());
    }
    (min, active, margin, previous)
}
pub(super) fn reaction(a: &mut Audit, group: &str, d: &V, c: &V, h: &V, base: f64) {
    let i = inertia(d);
    let inv = i.inverse();
    let gravity = vec(&d["primary"]["gravity"]);
    let pre = Cut::read(&h["incoming"], i);
    let post = Cut::read(&h["outgoing"], i);
    let j = vec(&h["impulse"]);
    let point = vec(&h["point"]);
    let n = vec(&h["normal"]);
    let normal = j.dot(n);
    let tangent = j - n * normal;
    let slip = vec(&h["after_relative_velocity"]);
    let mu = num(&h["mu"]);
    let expected = if h["upper_corner"].is_null() {
        vec(&c["gate_velocity"])
    } else {
        vec(&d["pad_velocity"])
    };
    let arm = pre.q * corner(int(&h["corner"]) as usize);
    assert!(point.distance(pre.c + arm) < 1e-12 && pre.c == post.c && pre.q == post.q);
    let before = pre.v - expected + mom(pre.q, inv, pre.l).cross(arm);
    let after = post.v - expected + mom(post.q, inv, post.l).cross(arm);
    let linear = (2. * (post.v - pre.v)).distance(j);
    let angular = (post.l - pre.l).distance(arm.cross(j));
    let loss = -0.5 * j.dot(before + after);
    let dk = (post.v.length_squared() - pre.v.length_squared())
        + 0.5 * (post.l.dot(mom(post.q, inv, post.l)) - pre.l.dot(mom(pre.q, inv, pre.l)));
    assert!(linear < 1e-10 && angular < 1e-10);
    assert!(
        before.distance(vec(&h["before_relative_velocity"])) < 1e-8 && after.distance(slip) < 1e-8
    );
    assert!(
        normal > 0.
            && before.dot(n) < 0.
            && after.dot(n).abs() < 1e-8
            && tangent.length() <= mu * normal + 1e-8
    );
    if h["sticking"] == true {
        assert!(after.length() < 1e-8)
    } else if mu > 0. {
        assert!((tangent.length() - mu * normal).abs() < 1e-8);
        assert!(tangent.normalize().dot(after.normalize()) < -1. + 1e-8);
    }
    assert!(
        (loss - num(&h["loss_j"])).abs() < 1e-8 && (dk - num(&h["delta_energy_j"])).abs() < 1e-8
    );
    let t = (num(&h["physical_ns"]) + base) * 1e-9;
    let start = (num(&h["start_ns"]) + base) * 1e-9;
    let stop = start + num(&h["duration_ns"]) * 1e-9;
    assert!(t >= start - 1e-15 && t <= stop + 1e-15);
    let mut other = f64::INFINITY;
    let mut footprint = f64::INFINITY;
    let mut min_active = f64::INFINITY;
    let mut xyz: f64 = 0.;
    let mut att: f64 = 0.;
    for (mut x, dt, refkey) in [
        (pre, start - t, "start_state"),
        (post, stop - t, "end_state"),
    ] {
        for k in 0..=32 {
            let (g, active, m, prior) =
                geometry(d, c, h, x, t + k as f64 * dt / 32., refkey == "start_state");
            if !h["upper_corner"].is_null() {
                assert!(prior >= -2e-10);
                let expected = if k == 0 {
                    Some(1)
                } else if k == 32 {
                    Some(if refkey == "start_state" { 0 } else { 2 })
                } else {
                    None
                };
                if let Some(index) = expected {
                    assert!((prior - num(&h["prior_upper_gaps_m"][index])).abs() < 2e-10);
                }
            }
            other = other.min(g);
            min_active = min_active.min(active);
            footprint = footprint.min(m);
            if k < 32 {
                x.step(dt / 32., gravity, inv);
            }
        }
        let end = Cut::read(&h[refkey], i);
        xyz = xyz.max(end.c.distance(x.c));
        let err = (end.q.conjugate() * x.q).normalize();
        att = att.max(2. * DVec3::new(err.x, err.y, err.z).length().atan2(err.w.abs()));
    }
    assert!(
        num(&h["timing_error_ns"]) <= 0.1
            && num(&h["other_gap_m"]) > 0.
            && num(&h["footprint_margin_m"]) > 0.
            && num(&h["outward_accel_m_s2"]) > 0.
    );
    assert!(
        other + 1e-11 >= num(&h["other_gap_m"])
            && other > 0.
            && footprint + 1e-11 >= num(&h["footprint_margin_m"])
    );
    assert!(
        min_active >= -2e-10 && xyz < 2e-10 && att < 2e-8,
        "sweep {group} {min_active} {xyz} {att}"
    );
    for (k, x) in [
        ("linear_closure_ns", linear),
        ("angular_closure_nms", angular),
        ("time_bound_ns", num(&h["timing_error_ns"])),
        ("sampled_other_gap_m", other),
        ("sampled_min_active_m", min_active),
        ("sampled_footprint_m", footprint),
        ("independent_endpoint_com_m", xyz),
        ("independent_endpoint_attitude_rad", att),
    ] {
        a.sample(group, k, x);
    }
}
pub(super) fn compare(a: &mut Audit, group: &str, e: &V, h: &V, base: f64) {
    let true_t = num(&e["at_ns"]);
    let t = num(&h["physical_ns"]) + base;
    let q = quat(&h["incoming"]["orientation"]);
    let com = vec(&h["incoming"]["position"]) + q * OFFSET;
    for (k, x) in [
        ("event_time_error_ns", (t - true_t).abs()),
        (
            "reaction_error_ns",
            vec(&h["impulse"]).distance(vec(&e["impulse"])),
        ),
        (
            "normal_error_ns",
            (vec(&h["impulse"]).dot(vec(&h["normal"])) - num(&e["normal_ns"])).abs(),
        ),
        (
            "loss_error_j",
            (num(&h["loss_j"]) - num(&e["loss_j"])).abs(),
        ),
        ("incoming_com_m", com.distance(vec(&e["com"]))),
        (
            "incoming_velocity_m_s",
            vec(&h["incoming"]["velocity"]).distance(vec(&e["velocity"])),
        ),
        (
            "relative_incoming_m_s",
            vec(&h["before_relative_velocity"]).distance(vec(&e["active_prior_velocity"])),
        ),
    ] {
        a.sample(group, k, x);
    }
}
pub(super) fn cost(a: &mut Audit, label: &str, data: &V) {
    assert_eq!(arr(&data["rows"]).len(), 2376);
    for r in arr(&data["rows"]) {
        let suffix = if r["enabled"] == true {
            "projected"
        } else {
            "native"
        };
        let delay = int(&r["delay_ns"]);
        let gate = int(&r["gate_tick"]) as usize;
        for clock in ["wall_ns", "cpu_ns"] {
            let v = arr(&r[clock]);
            assert_eq!(v.len(), 8);
            for key in [
                format!("{label}/all/{suffix}"),
                format!("{label}/{delay}/{suffix}"),
            ] {
                for x in v {
                    a.sample(&key, &format!("all_{clock}"), num(x));
                }
                a.sample(&key, &format!("run_{clock}"), v.iter().map(num).sum());
                a.sample(&key, &format!("landing_{clock}"), num(&v[gate - 1]));
                if gate < v.len() {
                    a.sample(&key, &format!("next_{clock}"), num(&v[gate]));
                }
            }
        }
    }
}
