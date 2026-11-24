// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent hard-contact control for one or two sliding vertices.
//! World angular momentum is integrated directly. Each sliding contact has
//! traction opposite its own point velocity and zero normal acceleration.
use super::*;

const GRAVITY: Vector = Vector::new(0., 0., -9.81);
const MU: f64 = 0.45;

#[derive(Clone, Copy)]
struct Body {
    center: Vector,
    q: Rotation,
    velocity: Vector,
    momentum: Vector,
}
impl Body {
    fn inv_i(self, torque: Vector) -> Vector {
        self.q * ((self.q.inverse() * torque) / MOMENTS)
    }
    fn arm(self, corner: usize) -> Vector {
        let x = if corner & 1 == 0 { -0.024 } else { 0.024 };
        let y = if corner & 2 == 0 { -0.012 } else { 0.012 };
        self.q * (Vector::new(x, y, -0.020) - OFFSET)
    }
    fn point_velocity(self, r: Vector) -> Vector {
        self.velocity + self.inv_i(self.momentum).cross(r)
    }
    fn gap(self, i: usize) -> f64 {
        self.center.z + self.arm(i).z - 0.014
    }
    fn wrench(self, arms: [Vector; 2], dirs: [Vector; 2], rhs: [f64; 2], n: usize) -> [Vector; 2] {
        // K[row][column] maps unit normal load, including its own Coulomb
        // traction, to normal acceleration (or speed change for an impact).
        let k = arms.map(|row| {
            std::array::from_fn::<_, 2, _>(|i| {
                (dirs[i] / MASS + self.inv_i(arms[i].cross(dirs[i])).cross(row)).z
            })
        });
        let normal = if n == 1 {
            [rhs[0] / k[0][0], 0.]
        } else {
            let det = k[0][0] * k[1][1] - k[0][1] * k[1][0];
            assert!(det.abs() > 1e-8);
            [
                (rhs[0] * k[1][1] - rhs[1] * k[0][1]) / det,
                (k[0][0] * rhs[1] - k[1][0] * rhs[0]) / det,
            ]
        };
        assert!(normal[..n].iter().all(|&p| p >= -1e-12));
        [dirs[0] * normal[0], dirs[1] * normal[1]]
    }
    fn directions(self, arms: [Vector; 2]) -> [Vector; 2] {
        arms.map(|r| {
            let v = self.point_velocity(r);
            let tangent = Vector::new(v.x, v.y, 0.);
            // This reference only covers uninterrupted sliding.
            assert!(tangent.length() > 0.02);
            Vector::Z - MU * tangent.normalize()
        })
    }
    fn forces(self, ids: [usize; 2], n: usize) -> [Vector; 2] {
        let arms = ids.map(|i| self.arm(i));
        let w = self.inv_i(self.momentum);
        let alpha = self.inv_i(-w.cross(self.momentum));
        let rhs = arms.map(|r| -(GRAVITY + alpha.cross(r) + w.cross(w.cross(r))).z);
        self.wrench(arms, self.directions(arms), rhs, n)
    }
    fn apply(self, at: Self, arms: [Vector; 2], force: [Vector; 2], dt: f64) -> Self {
        Self {
            center: self.center + dt * at.velocity,
            velocity: self.velocity + dt * (GRAVITY + (force[0] + force[1]) / MASS),
            q: (Rotation::from_scaled_axis(dt * at.inv_i(at.momentum)) * self.q).normalize(),
            momentum: self.momentum + dt * (arms[0].cross(force[0]) + arms[1].cross(force[1])),
        }
    }
    fn advance(self, ids: [usize; 2], n: usize, dt: f64) -> (Self, [Vector; 2]) {
        let mid = self.apply(
            self,
            ids.map(|i| self.arm(i)),
            self.forces(ids, n),
            0.5 * dt,
        );
        let force = mid.forces(ids, n);
        (
            self.apply(mid, ids.map(|i| mid.arm(i)), force, dt),
            force.map(|f| f * dt),
        )
    }
    fn impact(self, ids: [usize; 2]) -> (Self, [Vector; 2]) {
        let arms = ids.map(|i| self.arm(i));
        let rhs = arms.map(|r| -self.point_velocity(r).z);
        let mut out = self;
        let mut previous = [Vector::ZERO; 2];
        // Solve for the *outgoing* sliding directions of the coupled impact.
        for _ in 0..32 {
            let impulse = self.wrench(arms, out.directions(arms), rhs, 2);
            out.velocity = self.velocity + (impulse[0] + impulse[1]) / MASS;
            out.momentum = self.momentum + arms[0].cross(impulse[0]) + arms[1].cross(impulse[1]);
            if impulse[0]
                .distance(previous[0])
                .max(impulse[1].distance(previous[1]))
                < 1e-16
            {
                assert!(arms.iter().all(|&r| out.point_velocity(r).z.abs() < 1e-12));
                return (out, impulse);
            }
            previous = impulse;
        }
        panic!("sliding point directions failed to converge")
    }
}

pub(super) fn integrate(
    observed: &reload::Samples,
    ids: [usize; 2],
    steps: usize,
    anchor: usize,
    end: usize,
    mut count: usize,
) -> Value {
    let (seed, _) = observed[&anchor];
    let q = Rotation::from_array(seed.orientation);
    let mut b = Body {
        center: Vector::from_array(seed.position) + q * OFFSET,
        q,
        velocity: seed.center_velocity.into(),
        momentum: q * (MOMENTS * (q.inverse() * Vector::from_array(seed.omega))),
    };
    let incoming_gap = ids.map(|i| b.gap(i));
    let incoming_normal_speed = ids.map(|i| b.point_velocity(b.arm(i)).z);
    let dt = DT / steps as f64;
    let mut event = None;
    let mut rows = Vec::new();
    let (mut max_yaw, mut max_origin, mut max_load) = (0_f64, 0_f64, 0_f64);
    for tick in anchor + 1..=end {
        let mut loads = [Vector::ZERO; 2];
        for sub in 0..steps {
            let (next, impulse) = b.advance(ids, count, dt);
            if count == 1 && next.gap(ids[1]) <= 0. {
                let mut lo = 0.;
                let mut hi = dt;
                assert!(b.gap(ids[1]) >= -1e-12);
                for _ in 0..40 {
                    let m = 0.5 * (lo + hi);
                    if b.advance(ids, 1, m).0.gap(ids[1]) > 0. {
                        lo = m
                    } else {
                        hi = m
                    }
                }
                let (incoming, pre) = b.advance(ids, 1, hi);
                let (separating, pulse) = incoming.impact(ids);
                let (out, post) = separating.advance(ids, 2, dt - hi);
                event = Some(json!({
                    "tick": tick,
                    "offset_us": 1e6 * (sub as f64 * dt + hi),
                    "point_impulses_ns": pulse.map(|j| j.to_array()),
                    "incoming_normal_speed_m_s": ids.map(|i| incoming.point_velocity(incoming.arm(i)).z)
                }));
                for i in 0..2 {
                    loads[i] += pre[i] + pulse[i] + post[i]
                }
                b = out;
                count = 2;
            } else {
                for i in 0..2 {
                    loads[i] += impulse[i]
                }
                b = next;
            }
        }
        let (device, sensors) = observed[&tick];
        let yaw = reload::angle(b.q);
        max_yaw =
            max_yaw.max((yaw - reload::angle(Rotation::from_array(device.orientation))).abs());
        max_origin = max_origin.max((b.center - b.q * OFFSET).distance(device.position.into()));
        for (corner, impulse) in ids.into_iter().zip(loads) {
            max_load = max_load.max(impulse.distance(sensors.vertices[corner].into()));
        }
        rows.push(json!({
            "tick": tick,
            "yaw_deg": yaw,
            "omega_rad_s": b.inv_i(b.momentum).to_array(),
            "origin_m": (b.center - b.q * OFFSET).to_array(),
            "point_impulses_ns": loads.map(|j| j.to_array()),
            "gap_m": ids.map(|i| b.gap(i)),
            "normal_speed_m_s": ids.map(|i| b.point_velocity(b.arm(i)).z)
        }));
    }
    assert!(max_yaw < 0.005 && max_origin < 8e-6 && max_load < 0.75e-6);
    json!({
        "anchor": anchor,
        "steps_per_ms": steps,
        "corner_ids": ids,
        "initial_gap_m": incoming_gap,
        "initial_normal_speed_m_s": incoming_normal_speed,
        "reload": event,
        "device_max_error": {"yaw_deg": max_yaw, "link_m": max_origin, "corner_impulse_ns": max_load},
        "rows": rows
    })
}
