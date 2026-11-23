// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{DT, Native, Q, V, inertia};
use gz_physics_gpu::{SolidSphere as Body, SpherePairLane as Lane};
fn chord(a: [f64; 4], b: [f64; 4]) -> f64 {
    let (a, b) = (Q::from_array(a), Q::from_array(b));
    (a - b).length().min((a + b).length())
}
#[derive(Debug, Default)]
pub struct MotionErrors {
    pub p: f64,
    pub v: f64,
    pub w: f64,
    pub q: f64,
}
impl MotionErrors {
    pub fn bodies(&mut self, a: &Lane, b: &Lane) {
        assert!(
            bytemuck::cast_slice::<Lane, f64>(std::slice::from_ref(b))
                .iter()
                .all(|v| v.is_finite())
        );
        for (x, y) in [(a.a, b.a), (a.b, b.b)] {
            self.p = self
                .p
                .max(V::from(x.position).distance(V::from(y.position)));
            self.v = self
                .v
                .max(V::from(x.velocity).distance(V::from(y.velocity)));
            self.w = self
                .w
                .max(V::from(x.angular_velocity).distance(V::from(y.angular_velocity)));
            self.q = self.q.max(chord(x.orientation, y.orientation));
        }
    }
}
#[derive(Debug, Default)]
pub struct Errors {
    pub motion: MotionErrors,
    pub impulse: f64,
    pub normal: f64,
    pub toi: f64,
}
impl Errors {
    pub fn dense(&mut self, a: &Lane, b: &Lane) {
        self.motion.bodies(a, b);
        self.impulse = self
            .impulse
            .max((a.normal_impulse - b.normal_impulse).abs())
            .max(V::from(a.tangential_impulse).distance(V::from(b.tangential_impulse)));
        if a.normal_impulse > 1e-8 || b.normal_impulse > 1e-8 {
            assert!(
                a.normal_impulse > 1e-8 && b.normal_impulse > 1e-8,
                "impact clock differs"
            );
            self.normal = self
                .normal
                .max(V::from(a.contact_normal).distance(V::from(b.contact_normal)));
            self.toi = self.toi.max((a.impact_time - b.impact_time).abs());
        }
    }
}
fn spin(b: &Body) -> V {
    V::from(b.angular_velocity) * inertia(b)
}
fn internal(s: &Lane) -> (V, f64) {
    let d = V::from(s.b.position) - V::from(s.a.position);
    let u = V::from(s.b.velocity) - V::from(s.a.velocity);
    let reduced = 1. / (s.a.inverse_mass + s.b.inverse_mass);
    let angular = reduced * d.cross(u) + spin(&s.a) + spin(&s.b);
    let energy = 0.5
        * (reduced * u.length_squared()
            + spin(&s.a).dot(V::from(s.a.angular_velocity))
            + spin(&s.b).dot(V::from(s.b.angular_velocity)));
    (angular, energy)
}
#[derive(Debug, Default)]
pub struct Balance {
    pub momentum: f64,
    pub angular: f64,
    pub energy_gain: f64,
    pub impact_energy_residual: f64,
    pub slip_sticking: f64,
    pub cone_excess: f64,
    pub restitution: f64,
    pub gap_violation: f64,
    pub point_impulse: f64,
    pub free_com: f64,
    pub impacts: usize,
    pub sliding: usize,
    pub sticking: usize,
    pub misses: usize,
}
impl Balance {
    pub fn add(&mut self, p: &Lane, q: &Lane) {
        let g = V::from(p.gravity);
        let d = V::from(q.b.position) - V::from(q.a.position);
        self.gap_violation = self.gap_violation.max(q.a.radius + q.b.radius - d.length());
        let n = V::from(q.contact_normal);
        let jt = V::from(q.tangential_impulse);
        let j = q.normal_impulse * n + jt;
        let (old_l, old_e) = internal(p);
        let (new_l, new_e) = internal(q);
        self.angular = self.angular.max(old_l.distance(new_l));
        self.energy_gain = self.energy_gain.max(new_e - old_e);
        let mut momentum = V::ZERO;
        let mut com = V::ZERO;
        for (a, b, ji, r) in [
            (p.a, q.a, -j, p.a.radius * n),
            (p.b, q.b, j, -p.b.radius * n),
        ] {
            let m = 1. / a.inverse_mass;
            let delta = (V::from(b.velocity) - V::from(a.velocity) - DT * g) * m;
            momentum += delta;
            self.point_impulse = self
                .point_impulse
                .max(delta.distance(ji))
                .max((spin(&b) - spin(&a)).distance(r.cross(ji)));
            com += m
                * (V::from(b.position)
                    - V::from(a.position)
                    - DT * V::from(a.velocity)
                    - 0.5 * DT * DT * g);
        }
        self.momentum = self.momentum.max(momentum.length());
        self.free_com = self
            .free_com
            .max(com.length() / (1. / p.a.inverse_mass + 1. / p.b.inverse_mass));
        if q.normal_impulse <= 1e-8 {
            self.misses += 1;
            return;
        }
        self.impacts += 1;
        let arms = [p.a.radius * n, -p.b.radius * n];
        let surface = |s: &Lane| {
            V::from(s.b.velocity) + V::from(s.b.angular_velocity).cross(arms[1])
                - V::from(s.a.velocity)
                - V::from(s.a.angular_velocity).cross(arms[0])
        };
        let v0 = surface(p);
        let v1 = surface(q);
        let un = v0.dot(n);
        let vt = v0 - un * n;
        let cap = q.friction * q.normal_impulse;
        self.restitution = self.restitution.max((n.dot(v1) + q.restitution * un).abs());
        self.cone_excess = self
            .cone_excess
            .max(jt.length() - cap)
            .max(jt.dot(n).abs())
            .max(-q.normal_impulse);
        let inv = p.a.inverse_mass + p.b.inverse_mass;
        let predicted = q.normal_impulse * un
            + 0.5 * inv * q.normal_impulse.powi(2)
            + jt.dot(vt)
            + 0.5 * 3.5 * inv * jt.length_squared();
        self.impact_energy_residual = self
            .impact_energy_residual
            .max((new_e - old_e - predicted).abs());
        if q.friction > 0. {
            if jt.length() >= cap * (1. - 1e-9) {
                self.sliding += 1;
            } else {
                self.sticking += 1;
                self.slip_sticking = self.slip_sticking.max((v1 - n.dot(v1) * n).length());
            }
        }
    }
}
#[derive(Debug, Default)]
pub struct NativeAgreement {
    pub raw: MotionErrors,
    pub aligned: MotionErrors,
    pub equal_law: MotionErrors,
    pub max_offset: usize,
    pub peak_postimpact_accumulated_impulse_error: f64,
    pub raw_per_tick_impulse_error: f64,
    pub synchronized_per_tick_impulse_error: f64,
    pub capped_velocity_residual: f64,
    pub capped_rim_residual: f64,
    pub capped_position_residual: f64,
    pub max_cap_difference: f64,
    pub total_rapier_impulse: usize,
    pub matched_impact: usize,
    pub mixed_sliding: usize,
}
impl NativeAgreement {
    pub fn add(&mut self, k: usize, gpu: &Lane, native: &mut Native) {
        if gpu.normal_impulse > 1e-8 && native.device_impact.is_none() {
            native.device_impact = Some(k);
            // Rapier applies restitution after the friction iterations. A sliding
            // lane may therefore use the compression impulse as its Coulomb cap.
            let compression = gpu.normal_impulse / (1. + gpu.restitution);
            native.cap_difference =
                (V::from(gpu.tangential_impulse).length() - gpu.friction * compression).max(0.);
            self.max_cap_difference = self.max_cap_difference.max(native.cap_difference);
            if native.cap_difference > 1e-8 {
                self.mixed_sliding += 1;
            }
        }
        if native.impulse_b.length() > 1e-5 && native.native_impact.is_none() {
            native.native_impact = Some(k);
            self.total_rapier_impulse += 1;
        }
        self.raw.bodies(gpu, &native.state);
        let measured =
            V::from(gpu.contact_normal) * gpu.normal_impulse + V::from(gpu.tangential_impulse);
        let error = measured.distance(native.impulse_b);
        self.raw_per_tick_impulse_error = self.raw_per_tick_impulse_error.max(error);
        if let (Some(a), Some(b)) = (native.device_impact, native.native_impact) {
            self.max_offset = self.max_offset.max(a.abs_diff(b));
            if k == a.max(b) {
                self.matched_impact += 1;
            }
        }
        let aligned = match (native.device_impact, native.native_impact) {
            (Some(a), Some(b)) => k >= a.max(b) + 3,
            (None, None) => true,
            _ => false,
        };
        if !aligned {
            return;
        }
        self.aligned.bodies(gpu, &native.state);
        self.synchronized_per_tick_impulse_error =
            self.synchronized_per_tick_impulse_error.max(error);
        if native.cap_difference < 1e-8 {
            self.equal_law.bodies(gpu, &native.state);
        }
        let elapsed = native.device_impact.map_or(0., |t| (k - t + 1) as f64 * DT);
        for (a, b) in [(gpu.a, native.state.a), (gpu.b, native.state.b)] {
            let allowed = a.inverse_mass * native.cap_difference;
            let dv = V::from(a.velocity).distance(V::from(b.velocity));
            self.capped_velocity_residual = self.capped_velocity_residual.max(dv - allowed);
            self.capped_rim_residual = self.capped_rim_residual.max(
                a.radius * V::from(a.angular_velocity).distance(V::from(b.angular_velocity))
                    - 2.5 * allowed,
            );
            self.capped_position_residual = self
                .capped_position_residual
                .max(V::from(a.position).distance(V::from(b.position)) - elapsed * allowed);
            self.peak_postimpact_accumulated_impulse_error = self
                .peak_postimpact_accumulated_impulse_error
                .max(dv / a.inverse_mass);
        }
    }
}
