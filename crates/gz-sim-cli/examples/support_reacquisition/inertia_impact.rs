// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent torque-free Euler flight and isolated frictionless impact of a triaxial sole.
use super::*;
use glam::DMat3;
#[path = "inertia_cubic.rs"]
pub(crate) mod cubic;
#[path = "inertia_early.rs"]
pub(crate) mod early;
#[path = "inertia_gyro.rs"]
pub(crate) mod gyro;
#[path = "inertia_pulse.rs"]
pub(crate) mod pulse;
#[path = "inertia_release.rs"]
pub(crate) mod released;
#[path = "inertia_return.rs"]
pub(crate) mod returns;
#[path = "inertia_impact_sdf.rs"]
mod scene;
#[path = "separated_flight.rs"]
pub(crate) mod separated;
const CASES: usize = 160;
const WINDOW: f64 = 0.004;
const RK_H: f64 = 0.000008;
#[derive(Clone, Copy)]
struct ReferenceBody {
    member: DVec3,
    principal_frame: DQuat,
    tensor: DMat3,
    inverse: DMat3,
    lower: f64,
    upper: f64,
}
impl ReferenceBody {
    fn new(member: DVec3, principal_frame: DQuat) -> Self {
        let rot = DMat3::from_quat(principal_frame);
        let tensor = 2.
            * (rot * DMat3::from_diagonal(member) * rot.transpose()
                + DMat3::from_diagonal(DVec3::new(ARM.z.powi(2), ARM.z.powi(2), 0.)));
        Self {
            member,
            principal_frame,
            tensor,
            inverse: tensor.inverse(),
            lower: 2. * member.min_element(),
            upper: 2. * (member.max_element() + ARM.z.powi(2)),
        }
    }
    fn momentum(self, q: DQuat, w: DVec3) -> DVec3 {
        q * (self.tensor * (q.conjugate() * w))
    }
    fn omega(self, q: DQuat, l: DVec3) -> DVec3 {
        q * (self.inverse * (q.conjugate() * l))
    }
    fn accel(self, q: DQuat, w: DVec3) -> DVec3 {
        let b = q.conjugate() * w;
        q * (self.inverse * ((self.tensor * b).cross(b)))
    }
    fn surface(self, state: State) -> (DVec3, f64) {
        let w = DVec3::from(state.foot.angular_velocity);
        let l = self.momentum(DQuat::from_array(state.foot.orientation), w);
        (l, 0.5 * (M * state.velocity().length_squared() + w.dot(l)))
    }
}
/// Stable down to roundoff and invariant under the two quaternion signs.
fn angular_distance(a: DQuat, b: DQuat) -> f64 {
    let q = a * b.conjugate();
    2. * q.xyz().length().atan2(q.w.abs())
}
fn pure(v: DVec3) -> DQuat {
    DQuat::from_xyzw(v.x, v.y, v.z, 0.)
}
/// Integrate attitude at fixed world angular momentum, including backwards from the impact cut.
fn advance_q(body: ReferenceBody, q: DQuat, l: DVec3, h: f64) -> DQuat {
    let rate = |q: DQuat| {
        let q = q.normalize();
        (pure(body.omega(q, l)) * q) * 0.5
    };
    let a = rate(q);
    let b = rate(q + a * (0.5 * h));
    let c = rate(q + b * (0.5 * h));
    let d = rate(q + c * h);
    (q + (a + (b + c) * 2. + d) * (h / 6.)).normalize()
}
/// Independent body-frame Euler integrator with explicit spin and attitude derivatives.
fn advance_euler(body: ReferenceBody, q: DQuat, w: DVec3, h: f64) -> (DQuat, DVec3) {
    let rate = |q: DQuat, w: DVec3| {
        (
            (q.normalize() * pure(w)) * 0.5,
            body.inverse * ((body.tensor * w).cross(w)),
        )
    };
    let (a, x) = rate(q, w);
    let (b, y) = rate(q + a * (h / 2.), w + x * (h / 2.));
    let (c, z) = rate(q + b * (h / 2.), w + y * (h / 2.));
    let (d, t) = rate(q + c * h, w + z * h);
    (
        (q + (a + (b + c) * 2. + d) * (h / 6.)).normalize(),
        w + (x + (y + z) * 2. + t) * (h / 6.),
    )
}
#[derive(Clone, Copy)]
struct Branch {
    v: DVec3,
    l: DVec3,
}
struct Physical {
    gravity: DVec3,
    id: usize,
    body: ReferenceBody,
    arrival: f64,
    q: DQuat,
    pad_q: DQuat,
    pad_v: DVec3,
    n: DVec3,
    at: DVec3,
    pre: Branch,
    post: Branch,
    corner: usize,
    j: f64,
    closing: f64,
}
impl Physical {
    fn new(id: usize) -> Self {
        let h = |p| radical(851 + id, p);
        let signed = |p| 2. * h(p) - 1.;
        let member = [
            DVec3::new(0.020, 0.023, 0.026),
            DVec3::new(0.010, 0.025, 0.032),
            DVec3::new(0.003, 0.030, 0.031),
            DVec3::new(0.0014, 0.044, 0.045),
        ][id % 4];
        let frame = DQuat::from_euler(
            glam::EulerRot::ZYX,
            0.7 * signed(11),
            0.6 * signed(13),
            0.8 * signed(17),
        );
        Self::with_body(id, ReferenceBody::new(member, frame))
    }
    fn with_body(id: usize, body: ReferenceBody) -> Self {
        let h = |p| radical(851 + id, p);
        let signed = |p| 2. * h(p) - 1.;
        let sgn = |p| if h(p) < 0.5 { -1. } else { 1. };
        let pad_q = DQuat::from_euler(
            glam::EulerRot::ZYX,
            0.4 * signed(2),
            0.18 * signed(3),
            0.16 * signed(5),
        );
        let n = pad_q * DVec3::Z;
        let q = pad_q
            * DQuat::from_euler(
                glam::EulerRot::ZYX,
                0.26 * signed(7),
                (0.16 + 0.13 * h(19)) * sgn(23),
                (0.17 + 0.18 * h(29)) * sgn(31),
            );
        let pad_v = pad_q * DVec3::new(0.16 * signed(37), 0.2 * signed(41), 0.6 * signed(43));
        let speed = [0.35, 2., 6., 14., 32., 70., 115., 0.][(id / 4) % 8];
        let spin = q * (DVec3::new(signed(47), signed(53), signed(59)).normalize() * speed);
        let rs = corners().map(|r| q * (r - ARM));
        let k = (0..4)
            .min_by(|&i, &j| rs[i].dot(n).total_cmp(&rs[j].dot(n)))
            .unwrap();
        let arm = rs[k];
        let closing = 0.6 + 1.1 * h(61);
        let v = pad_v
            + pad_q
                * DVec3::new(
                    0.1 * signed(67),
                    0.11 * signed(71),
                    -closing - spin.cross(arm).dot(n),
                );
        let arrival = (1_610_000. + (h(73) * 1_120_000.).round()) * 1e-9;
        let at = pad_v * arrival - n * arm.dot(n);
        let l = body.momentum(q, spin);
        let axis = arm.cross(n);
        let j = closing / (1. / M + axis.dot(body.omega(q, axis)));
        Self {
            gravity: DVec3::ZERO,
            id,
            body,
            arrival,
            q,
            pad_q,
            pad_v,
            n,
            at,
            pre: Branch { v, l },
            post: Branch {
                v: v + n * j / M,
                l: l + axis * j,
            },
            corner: k,
            j,
            closing,
        }
    }
    fn state(&self, t: f64) -> State {
        let b = if t < self.arrival {
            self.pre
        } else {
            self.post
        };
        self.state_branch(t, b)
    }
    fn state_branch(&self, t: f64, b: Branch) -> State {
        let dt = t - self.arrival;
        let count = (dt.abs() / RK_H).ceil().max(1.) as usize;
        let mut q = self.q;
        for _ in 0..count {
            q = advance_q(self.body, q, b.l, dt / count as f64);
        }
        let w = self.body.omega(q, b.l);
        let pos = self.at + (b.v + self.gravity * (0.5 * dt)) * dt;
        let velocity = b.v + self.gravity * dt;
        let member = |r: DVec3| Body {
            position: (pos + r).into(),
            orientation: q.into(),
            velocity: (velocity + w.cross(r)).into(),
            angular_velocity: w.into(),
        };
        State {
            foot: member(-(q * ARM)),
            upper: member(q * ARM),
            pad: Body {
                position: (self.pad_v * t - self.n * PAD_HALF[2]).into(),
                orientation: self.pad_q.into(),
                velocity: self.pad_v.into(),
                angular_velocity: [0.; 3],
            },
        }
    }
    fn trace(&self) -> Vec<Snapshot> {
        (0..=8)
            .map(|i| {
                let mut c = [0.; 4];
                if i > 0
                    && (i as f64 * 0.0005) >= self.arrival
                    && ((i - 1) as f64 * 0.0005) < self.arrival
                {
                    c[self.corner] = self.j;
                }
                frame(i, self.state(i as f64 * 0.0005), 0, false, c)
            })
            .collect()
    }
    fn certify(&self) -> Certificate {
        self.certify_branches(&[(0., self.pre), (WINDOW, self.post)])
    }
    fn certify_branches(&self, branches: &[(f64, Branch)]) -> Certificate {
        let mut out = Certificate::default();
        let n = self.n;
        let pts = corners().map(|p| p - ARM);
        for &(end, branch) in branches {
            let dt = end - self.arrival;
            let steps = (dt.abs() / RK_H).ceil() as usize;
            let h = dt / steps as f64;
            let d = h.abs();
            let w0 = self.body.omega(self.q, branch.l);
            let e2 = w0.dot(branch.l);
            let omega = (e2 / self.body.lower)
                .sqrt()
                .min(branch.l.length() / self.body.lower);
            let ratio = (self.body.upper - self.body.lower) / self.body.lower;
            let alpha = 0.5 * ratio * omega * omega;
            let beta = (1. + ratio) * omega * alpha;
            let mut q = self.q;
            let mut q2 = q;
            let mut wb = q.conjugate() * w0;
            for i in 0..steps {
                let t = self.arrival + i as f64 * h;
                let since = t - self.arrival;
                let center = self.at + (branch.v + self.gravity * (0.5 * since)) * since;
                let velocity = branch.v + self.gravity * since;
                let w = self.body.omega(q, branch.l);
                let wa = self.body.accel(q, w);
                for (k, r0) in pts.iter().enumerate() {
                    let r = q * *r0;
                    let u = velocity - self.pad_v + w.cross(r);
                    let local = self.pad_q.conjugate() * (center + r - self.pad_v * t);
                    let vbound = ((velocity - self.pad_v).length()
                        + omega * r.length()
                        + 0.5 * self.gravity.length() * d)
                        * d;
                    if local.x.abs() + vbound >= PAD_HALF[0]
                        || local.y.abs() + vbound >= PAD_HALF[1]
                        || (q.conjugate() * n).z < omega * d
                    {
                        out.reason = Some("finite face");
                        return out;
                    }
                    let g = local.z;
                    let v = h.signum() * u.dot(n);
                    let accel = (self.gravity + wa.cross(r) + w.cross(w.cross(r))).dot(n);
                    let amax = self.gravity.dot(n).abs() + (alpha + omega * omega) * r.length();
                    let jerk = (beta + 3. * omega * alpha + omega.powi(3)) * r.length();
                    // Taylor lower bounds on gap, speed and acceleration over each interval.
                    // Start the zero-gap controlled corner with its exact event complementarity.
                    let rising = v - amax * d > 0. || v >= -1e-12 && accel - jerk * d > 0.;
                    if !(g >= -1e-12 && rising || g + v.min(0.) * d - 0.5 * amax * d * d > 1e-12) {
                        out.reason = Some(if k == self.corner {
                            "primary may remain or return"
                        } else {
                            "secondary not excluded"
                        });
                        return out;
                    }
                }
                q = advance_q(self.body, q, branch.l, h);
                for _ in 0..2 {
                    (q2, wb) = advance_euler(self.body, q2, wb, h / 2.);
                }
                let angular_error = angular_distance(q, q2);
                out.orientation_error = out.orientation_error.max(angular_error);
                out.momentum_error = out
                    .momentum_error
                    .max((q2 * (self.body.tensor * wb)).distance(branch.l));
                out.energy_error = out
                    .energy_error
                    .max((wb.dot(self.body.tensor * wb) - e2).abs() * 0.5);
            }
            if out.orientation_error > 1e-10 || out.momentum_error > 1e-9 || out.energy_error > 1e-8
            {
                out.reason = Some("Euler integration convergence");
                return out;
            }
        }
        out
    }
}
#[derive(Default, Serialize)]
struct Certificate {
    reason: Option<&'static str>,
    orientation_error: f64,
    momentum_error: f64,
    energy_error: f64,
}
pub(crate) fn write(path: &Path) -> Result<()> {
    scene::write(path)
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    scene::cost(path)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn independent_euler_reference_preserves_momentum_and_dissipates_the_plastic_loss() {
        let mut certified = 0;
        let mut high = 0;
        for i in 0..CASES {
            let r = Physical::new(i);
            let proof = r.certify();
            if proof.reason.is_none() {
                certified += 1;
                let energy =
                    |b: Branch| 0.5 * (M * b.v.length_squared() + b.l.dot(r.body.omega(r.q, b.l)));
                let work = r.j * r.pad_v.dot(r.n);
                assert!(
                    (energy(r.post) - energy(r.pre) - work + r.closing * r.j / 2.).abs() < 1e-10
                );
                if r.body.omega(r.q, r.pre.l).length() > 30. {
                    high += 1;
                }
            }
        }
        assert!(
            certified > 45 && high > 0,
            "{certified} certified, {high} high-spin"
        );
    }
}
