// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Serialize)]
pub(super) struct Errors {
    matched_clocks: usize,
    pub max_com_m: f64,
    max_foot_m: f64,
    max_rotation_rad: f64,
    max_velocity_m_s: f64,
    max_spin_rad_s: f64,
    max_impulse_ns: f64,
    max_sampled_force_n: f64,
    max_normal_impulse_error_ns: f64,
    max_normal_heat_error_j: f64,
    max_sliding_work_error_j: f64,
    max_boundary_work_error_j: f64,
    max_shared_pressure_center_error_m: f64,
    shared_pressure_centers: usize,
    mismatched_loaded_clocks: usize,
    peak_normal_n: f64,
    reference_peak_normal_n: f64,
    peak_error_n: f64,
    final_impulse_ns: [f64; 3],
    reference_final_impulse_ns: [f64; 3],
    final_normal_heat_j: f64,
    reference_final_normal_heat_j: f64,
    heat_error_j: f64,
    first_shared_unloaded_clock_s: f64,
    rebound_at_shared_clock_m_s: f64,
    reference_rebound_at_shared_clock_m_s: f64,
    rebound_error_m_s: f64,
    final_com_error_m: f64,
    endpoint_energy_budget_error_j: f64,
    max_energy_budget_error_j: f64,
    max_momentum_budget_error_ns: f64,
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    DVec3::from_array(a).distance(b.into())
}

fn mechanical(f: &Frame) -> f64 {
    DVec3::from(f.v_m_s).length_squared()
        + 0.025 * DVec3::from(f.spin_rad_s).length_squared()
        + 2. * 9.81 * f.com_m[2]
        + f.strain_energy_j
}

impl Errors {
    pub fn compare(live: &[Frame], reference: &History, peak: f64) -> Result<Self> {
        let mut pairs = Vec::with_capacity(live.len());
        for frame in live {
            let mut matches = reference
                .frames
                .iter()
                .filter(|r| (r.original_time_s - frame.original_time_s).abs() < 1e-12);
            let r = matches.next().ok_or("missing shared reference boundary")?;
            if matches.next().is_some() {
                return Err("ambiguous shared reference boundary".into());
            }
            pairs.push((frame, r));
        }
        let &(first, r0) = pairs.first().ok_or("empty trajectory")?;
        if distance(first.com_m, r0.com_m) > 1e-10
            || distance(first.foot_m, r0.foot_m) > 1e-10
            || distance(first.v_m_s, r0.v_m_s) > 1e-12
            || distance(first.spin_rad_s, r0.spin_rad_s) > 1e-12
            || DQuat::from_array(first.orientation_xyzw)
                .angle_between(DQuat::from_array(r0.orientation_xyzw))
                > 1e-10
        {
            return Err("initial states differ from free donor".into());
        }
        let &(last, r_last) = pairs.last().ok_or("empty trajectory")?;
        let &(rebound, r_rebound) = pairs
            .iter()
            .find(|(f, r)| {
                f.original_time_s >= reference.checks.lift_off_time_s
                    && f.normal_force_n == 0.
                    && r.normal_force_n == 0.
                    && f.v_m_s[2] > 0.
                    && r.v_m_s[2] > 0.
            })
            .ok_or("no common free-flight boundary after lift-off")?;
        let mut e = Self {
            matched_clocks: pairs.len(),
            max_com_m: 0.,
            max_foot_m: 0.,
            max_rotation_rad: 0.,
            max_velocity_m_s: 0.,
            max_spin_rad_s: 0.,
            max_impulse_ns: 0.,
            max_sampled_force_n: 0.,
            max_normal_impulse_error_ns: 0.,
            max_normal_heat_error_j: 0.,
            max_sliding_work_error_j: 0.,
            max_boundary_work_error_j: 0.,
            max_shared_pressure_center_error_m: 0.,
            shared_pressure_centers: 0,
            mismatched_loaded_clocks: 0,
            peak_normal_n: peak,
            reference_peak_normal_n: reference.checks.peak_normal_n,
            peak_error_n: (peak - reference.checks.peak_normal_n).abs(),
            final_impulse_ns: last.support_frame_j_ns,
            reference_final_impulse_ns: r_last.support_frame_j_ns,
            final_normal_heat_j: last.normal_dissipation_j,
            reference_final_normal_heat_j: r_last.normal_dissipation_j,
            heat_error_j: (last.normal_dissipation_j - r_last.normal_dissipation_j).abs(),
            first_shared_unloaded_clock_s: rebound.original_time_s,
            rebound_at_shared_clock_m_s: rebound.v_m_s[2],
            reference_rebound_at_shared_clock_m_s: r_rebound.v_m_s[2],
            rebound_error_m_s: (rebound.v_m_s[2] - r_rebound.v_m_s[2]).abs(),
            final_com_error_m: distance(last.com_m, r_last.com_m),
            endpoint_energy_budget_error_j: 0.,
            max_energy_budget_error_j: 0.,
            max_momentum_budget_error_ns: 0.,
        };
        let initial_energy = mechanical(first);
        for (f, r) in pairs {
            e.max_com_m = e.max_com_m.max(distance(f.com_m, r.com_m));
            e.max_foot_m = e.max_foot_m.max(distance(f.foot_m, r.foot_m));
            let delta = DQuat::from_array(f.orientation_xyzw)
                * DQuat::from_array(r.orientation_xyzw).inverse();
            e.max_rotation_rad = e.max_rotation_rad.max(delta.to_scaled_axis().length());
            e.max_velocity_m_s = e.max_velocity_m_s.max(distance(f.v_m_s, r.v_m_s));
            e.max_spin_rad_s = e.max_spin_rad_s.max(distance(f.spin_rad_s, r.spin_rad_s));
            e.max_impulse_ns = e
                .max_impulse_ns
                .max(distance(f.support_frame_j_ns, r.support_frame_j_ns));
            e.max_sampled_force_n = e
                .max_sampled_force_n
                .max((f.normal_force_n - r.normal_force_n).abs());
            e.max_normal_impulse_error_ns = e
                .max_normal_impulse_error_ns
                .max((f.support_frame_j_ns[2] - r.support_frame_j_ns[2]).abs());
            e.max_normal_heat_error_j = e
                .max_normal_heat_error_j
                .max((f.normal_dissipation_j - r.normal_dissipation_j).abs());
            e.max_sliding_work_error_j = e
                .max_sliding_work_error_j
                .max((f.sliding_work_j - r.sliding_work_j).abs());
            e.max_boundary_work_error_j = e
                .max_boundary_work_error_j
                .max((f.boundary_energy_flux_j - r.boundary_energy_flux_j).abs());
            match (f.center_of_pressure_m, r.center_of_pressure_m) {
                (Some(l), Some(a)) => {
                    e.shared_pressure_centers += 1;
                    e.max_shared_pressure_center_error_m =
                        e.max_shared_pressure_center_error_m.max(distance(l, a));
                }
                (None, None) => {}
                _ => e.mismatched_loaded_clocks += 1,
            }
            let defect = mechanical(f) - initial_energy + f.normal_dissipation_j
                - f.sliding_work_j
                - f.boundary_energy_flux_j;
            e.endpoint_energy_budget_error_j = defect.abs();
            e.max_energy_budget_error_j = e.max_energy_budget_error_j.max(defect.abs());
            let impulse = 2.
                * (DVec3::from(f.v_m_s) - DVec3::from(first.v_m_s)
                    + 9.81 * f.original_time_s * DVec3::Z);
            e.max_momentum_budget_error_ns = e
                .max_momentum_budget_error_ns
                .max(impulse.distance(f.support_frame_j_ns.into()));
        }
        Ok(e)
    }
    pub fn qualify(&self) -> Result<()> {
        let checks = [
            ("COM position (m)", self.max_com_m, 1e-6),
            ("foot position (m)", self.max_foot_m, 1e-6),
            ("impulse (N s)", self.max_impulse_ns, 1e-4),
            ("peak normal force (N)", self.peak_error_n, 1.),
            ("heat (J)", self.max_normal_heat_error_j, 1e-4),
            ("rebound (m/s)", self.rebound_error_m_s, 1e-4),
            ("energy budget (J)", self.max_energy_budget_error_j, 2e-4),
            (
                "momentum budget (N s)",
                self.max_momentum_budget_error_ns,
                1e-9,
            ),
        ];
        for (name, measured, limit) in checks {
            if !measured.is_finite() || measured > limit {
                return Err(
                    format!("{name} error {measured} exceeds qualification bound {limit}").into(),
                );
            }
        }
        if self.matched_clocks != 28
            || self.shared_pressure_centers == 0
            || self.mismatched_loaded_clocks != 0
        {
            return Err("unmatched contact chronology at public clocks".into());
        }
        Ok(())
    }
}

#[derive(Serialize)]
struct Distribution {
    ticks: usize,
    p50_wall_us: f64,
    p95_wall_us: f64,
    p95_thread_cpu_us: f64,
}
impl Distribution {
    fn samples(timings: &[(u64, u64)]) -> Result<Self> {
        if timings.is_empty() {
            return Err("empty timing class".into());
        }
        let mut wall = timings.iter().map(|p| p.0).collect::<Vec<_>>();
        let mut cpu = timings.iter().map(|p| p.1).collect::<Vec<_>>();
        wall.sort_unstable();
        cpu.sort_unstable();
        let p = |percent: usize| (percent * timings.len()).div_ceil(100) - 1;
        Ok(Self {
            ticks: timings.len(),
            p50_wall_us: wall[p(50)] as f64 / 1e3,
            p95_wall_us: wall[p(95)] as f64 / 1e3,
            p95_thread_cpu_us: cpu[p(95)] as f64 / 1e3,
        })
    }
}
#[derive(Serialize)]
pub(super) struct Cost {
    timed_landings: usize,
    warmup_landings: usize,
    all: Distribution,
    contact_impulse: Distribution,
    no_contact_impulse: Distribution,
}
fn cpu_time_ns() -> u64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
    t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64
}
impl Cost {
    pub fn measure(xml: &str, ticks: u32, repeats: usize) -> Result<Self> {
        let (mut all, mut contact, mut free) = (vec![], vec![], vec![]);
        for rep in 0..=repeats {
            let mut sim = load(xml)?;
            for _ in 0..ticks {
                let cpu = cpu_time_ns();
                let wall = Instant::now();
                sim.step_exact(1)?;
                let nanos = wall.elapsed().as_nanos() as u64;
                let elapsed_cpu = cpu_time_ns() - cpu;
                let active = last_step(&sim)
                    .ok_or("no timed contact report")?
                    .step
                    .normal_impulse
                    > 0.;
                if rep > 0 {
                    let sample = (nanos, elapsed_cpu);
                    all.push(sample);
                    if active {
                        contact.push(sample);
                    } else {
                        free.push(sample);
                    }
                }
            }
        }
        Ok(Self {
            timed_landings: repeats,
            warmup_landings: 1,
            all: Distribution::samples(&all)?,
            contact_impulse: Distribution::samples(&contact)?,
            no_contact_impulse: Distribution::samples(&free)?,
        })
    }
}
