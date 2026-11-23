// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{Native, Q, V};
use gz_physics_gpu::{STEP, SpherePlaneLane as Lane};

#[derive(Debug, Default)]
pub struct Errors {
    pub p: f64,
    pub v: f64,
    pub w: f64,
    pub q: f64,
    pub jn: f64,
    pub jt: f64,
}
fn chord(a: [f64; 4], b: [f64; 4]) -> f64 {
    let (a, b) = (Q::from_array(a), Q::from_array(b));
    (a - b).length().min((a + b).length())
}
impl Errors {
    pub fn add(&mut self, a: &Lane, b: &Lane) {
        for observed in [a, b] {
            assert!(
                bytemuck::cast_slice::<Lane, f64>(std::slice::from_ref(observed))
                    .iter()
                    .all(|v| v.is_finite()),
                "nonfinite qualification reference or observation"
            );
        }
        self.p = self
            .p
            .max(V::from(a.position).distance(V::from(b.position)));
        self.v = self
            .v
            .max(V::from(a.velocity).distance(V::from(b.velocity)));
        self.w = self
            .w
            .max(V::from(a.angular_velocity).distance(V::from(b.angular_velocity)));
        self.q = self.q.max(chord(a.orientation, b.orientation));
        self.jn = self.jn.max((a.normal_impulse - b.normal_impulse).abs());
        self.jt = self
            .jt
            .max(V::from(a.tangential_impulse).distance(V::from(b.tangential_impulse)));
    }
}
#[derive(Default, Debug)]
pub struct Balance {
    pub linear: f64,
    pub angular: f64,
    pub energy_gain: f64,
    pub penetration: f64,
    pub cone_excess: f64,
    pub normal_speed: f64,
    pub normal_leak: f64,
    pub sticking_slip_m_s: f64,
    pub friction_work_gain_j: f64,
    pub normal_spin_change_rad_s: f64,
    pub sliding: u64,
    pub sticking: u64,
    pub frictionless: u64,
    pub twisting: u64,
}
impl Balance {
    pub fn add(&mut self, a: &Lane, b: &Lane) {
        let n = V::from(a.plane_normal);
        let arm = -n * a.radius;
        let m = 1. / a.inverse_mass;
        let inertia = (2. / 5.) * m * a.radius.powi(2);
        let u = V::from(a.velocity) + V::from(a.gravity) * STEP.as_secs_f64();
        let v = V::from(b.velocity);
        let w = V::from(a.angular_velocity);
        let w1 = V::from(b.angular_velocity);
        let jt = V::from(b.tangential_impulse);
        let jn = b.normal_impulse;
        let cap = a.friction * jn;
        self.linear = self.linear.max(((v - u) * m - n * jn - jt).length());
        self.angular = self
            .angular
            .max(((w1 - w) * inertia - arm.cross(jt)).length());
        self.energy_gain = self.energy_gain.max(
            (m * (v.length_squared() - u.length_squared())
                + inertia * (w1.length_squared() - w.length_squared()))
                * 0.5,
        );
        self.penetration = self
            .penetration
            .max(a.plane_offset + a.radius - n.dot(V::from(b.position)));
        self.cone_excess = self.cone_excess.max(jt.length() - cap).max(-jn);
        self.normal_leak = self.normal_leak.max(n.dot(jt).abs());
        if jn > 1e-10 {
            self.normal_speed = self.normal_speed.max(v.dot(n).abs());
            self.normal_spin_change_rad_s =
                self.normal_spin_change_rad_s.max((w1 - w).dot(n).abs());
            self.friction_work_gain_j = self.friction_work_gain_j.max(jt.dot(u + w.cross(arm)));
            if a.friction == 0. {
                self.frictionless += 1;
            } else if jt.length() >= cap * (1. - 1e-9) {
                self.sliding += 1;
            } else {
                self.sticking += 1;
                let slip = v + w1.cross(arm);
                self.sticking_slip_m_s = self
                    .sticking_slip_m_s
                    .max((slip - n * slip.dot(n)).length());
            }
            if (w1 - w).length() > 1e-4 {
                self.twisting += 1;
            }
        }
    }
}
#[derive(Default, Debug)]
pub struct NativeAgreement {
    pub raw: Errors,
    pub outside_impact: Errors,
    pub max_impact_offset_ticks: usize,
    pub synchronized_surface_omega: f64,
    pub peak_cumulative_impulse_n_s: f64,
    pub postimpact_cumulative_impulse_n_s: f64,
    pub postimpact_impulse_speed_m_s: f64,
}
impl NativeAgreement {
    pub fn add(&mut self, tick: usize, gpu: &Lane, probe: &mut Native) {
        let cpu = &probe.state;
        if gpu.normal_impulse > 1e-5 {
            probe.gpu_first.get_or_insert(tick);
        }
        if cpu.normal_impulse > 1e-5 {
            probe.native_first.get_or_insert(tick);
        }
        self.raw.add(gpu, cpu);
        if let (Some(a), Some(b)) = (probe.gpu_first, probe.native_first) {
            self.max_impact_offset_ticks = self.max_impact_offset_ticks.max(a.abs_diff(b));
            // Two integrators sample an instantaneous velocity jump at different
            // times. Keep unfiltered errors above and assess post-impact state
            // once both have passed a four-public-tick settling window.
            if tick >= a.max(b) + 4 {
                self.outside_impact.add(gpu, cpu);
                // Identical constant gravity cancels between the two momentum
                // histories. This compares the total impulse since time zero.
                self.postimpact_cumulative_impulse_n_s = self
                    .postimpact_cumulative_impulse_n_s
                    .max(V::from(gpu.velocity).distance(V::from(cpu.velocity)) / gpu.inverse_mass);
                let normal_error = gpu.normal_impulse - cpu.normal_impulse;
                let tangent_error =
                    V::from(gpu.tangential_impulse) - V::from(cpu.tangential_impulse);
                self.postimpact_impulse_speed_m_s = self.postimpact_impulse_speed_m_s.max(
                    (V::from(gpu.plane_normal) * normal_error + tangent_error).length()
                        * gpu.inverse_mass,
                );
                self.synchronized_surface_omega = self.synchronized_surface_omega.max(
                    gpu.radius
                        * V::from(gpu.angular_velocity).distance(V::from(cpu.angular_velocity)),
                );
            }
        } else if probe.gpu_first.is_none() && probe.native_first.is_none() {
            self.outside_impact.add(gpu, cpu);
        }
        let delta = V::from(gpu.velocity) - V::from(cpu.velocity);
        self.peak_cumulative_impulse_n_s = self
            .peak_cumulative_impulse_n_s
            .max(delta.length() / gpu.inverse_mass);
    }
}
