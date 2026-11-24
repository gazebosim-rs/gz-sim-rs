// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
pub(super) const START: usize = 5400;
const HALF: usize = 2000;
pub(super) const END: usize = START + 2 * HALF;
const STROKE: f64 = 0.035;
const MOTION_TICKS: usize = 750;
const DWELL: usize = 400;
#[derive(Default)]
struct Window {
    n: usize,
    vertical_n: f64,
    square_n2: f64,
    bilateral: usize,
    stable_lift: usize,
}
impl Window {
    fn add(&mut self, r: &JawReadings) {
        self.n += 1;
        let f = r.force_on_payload_n[2];
        self.vertical_n += f;
        self.square_n2 += f * f;
        self.bilateral += usize::from(r.squeeze_n.iter().all(|n| *n > 0.3));
        self.stable_lift += usize::from(
            (r.spindle_rad - controller::NORMAL_LIFT).abs() < 0.015
                && r.rail_speed_m_s.abs() < 0.004,
        );
    }
    fn problem(&self) -> Option<Refusal> {
        if self.n != DWELL {
            return Some(Refusal::InsufficientProbe);
        }
        if self.bilateral != DWELL {
            return Some(Refusal::NoBilateralGrasp);
        }
        if self.stable_lift != DWELL {
            return Some(Refusal::UnsettledLift);
        }
        if self.mean() < 0.035 || self.sd() > 0.12 * self.mean() {
            return Some(Refusal::InconsistentWeight);
        }
        None
    }
    fn mean(&self) -> f64 {
        self.vertical_n / self.n as f64
    }
    fn sd(&self) -> f64 {
        (self.square_n2 / self.n as f64 - self.mean().powi(2))
            .max(0.)
            .sqrt()
    }
}
#[derive(Default)]
pub(super) struct Calibration {
    windows: [Window; 3],
    positions: [Option<f64>; 3],
    horizontal_force_moment: f64,
    loose_ticks: usize,
}
#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub(super) enum Refusal {
    NoBilateralGrasp,
    UnsettledLift,
    InconsistentWeight,
    InsufficientProbe,
    MassDisagreement,
}
#[derive(Clone, Debug, Serialize, PartialEq)]
pub(super) struct Estimate {
    pub weighted_plateaus_n: [Option<f64>; 3],
    pub plateau_std_n: [Option<f64>; 3],
    pub stable_grip_ticks: [usize; 3],
    pub stable_lift_ticks: [usize; 3],
    pub rail_endpoints_m: [Option<f64>; 3],
    pub horizontal_force_moment_ns2: f64,
    pub inertial_mass_kg: Option<f64>,
    pub apparent_weight_mass_kg: Option<f64>,
    pub mass_kg: Option<f64>,
    pub travel_ticks: Option<usize>,
    pub refused: Option<Refusal>,
}
impl Calibration {
    pub(super) fn drive(sim: &mut Simulation, robot: &Robot, tick: usize) -> Result<()> {
        if !(START + 1..=END).contains(&tick) {
            return Ok(());
        }
        let i = tick - START - 1;
        let forward = i < HALF;
        let t = ((i % HALF + 1) as f64 / MOTION_TICKS as f64).min(1.);
        let f = t.powi(3) * (10. + t * (-15. + 6. * t));
        let df = 30. * t.powi(2) * (1. - t).powi(2) / (MOTION_TICKS as f64 * DT);
        let (target, speed) = if forward {
            (STROKE * f, STROKE * df)
        } else {
            (STROKE * (1. - f), -STROKE * df)
        };
        let rail = sim
            .world()
            .get::<JointSample>(robot.rail.entity)
            .ok_or("missing calibration encoder")?
            .state;
        let effort = 100. * (target - rail.position) + 28. * (speed - rail.velocity);
        JointForceCommand::set_batch(sim, [(robot.rail.id, effort.clamp(-80., 80.))])?;
        Ok(())
    }
    pub(super) fn observe(&mut self, tick: usize, r: &JawReadings) {
        for (k, end) in [START, START + HALF, END].into_iter().enumerate() {
            if tick > end - DWELL && tick <= end {
                self.windows[k].add(r);
            }
            if tick == end {
                self.positions[k] = Some(r.rail_y_m);
            }
        }
        if (START + 1..=END).contains(&tick) {
            if r.squeeze_n.iter().all(|force| *force > 0.3) {
                self.loose_ticks = 0;
            } else {
                self.loose_ticks += 1;
            }
            let n = tick - START;
            // Centered second position difference eliminates the unknown initial
            // lateral velocity. The discrete factors match integration at tick end.
            let lever = (n - 1).min(2 * HALF - n + 1) as f64 * DT;
            self.horizontal_force_moment += lever * DT * r.force_on_payload_n[1];
        }
    }
    pub(super) fn probe_lost(&self) -> bool {
        self.loose_ticks >= 8
    }
    pub(super) fn first_problem(&self) -> Option<Refusal> {
        self.windows[0].problem()
    }
    pub(super) fn finish(&self) -> Estimate {
        let mean = self.windows.each_ref().map(|w| (w.n > 0).then(|| w.mean()));
        let sd = self.windows.each_ref().map(|w| (w.n > 0).then(|| w.sd()));
        let observed = mean.into_iter().flatten().count();
        let plateau_mass = (observed > 0)
            .then(|| mean.into_iter().flatten().sum::<f64>() / (observed as f64 * 9.81));
        let inertial = match self.positions {
            [Some(start), Some(middle), Some(end)]
                if (middle - start - STROKE).abs() < 0.002 && (end - start).abs() < 0.002 =>
            {
                Some(self.horizontal_force_moment / (end - 2. * middle + start))
            }
            _ => None,
        };
        let window_problem = self
            .windows
            .iter()
            .filter(|w| w.n > 0)
            .find_map(Window::problem);
        let rejected = self
            .probe_lost()
            .then_some(Refusal::NoBilateralGrasp)
            .or(window_problem)
            .or_else(|| {
                let (Some(weight_mass), Some(inertial_mass)) = (plateau_mass, inertial) else {
                    return Some(Refusal::InsufficientProbe);
                };
                if mean
                    .into_iter()
                    .flatten()
                    .any(|v| (v - weight_mass * 9.81).abs() > 0.08 * v)
                {
                    Some(Refusal::InconsistentWeight)
                } else if (inertial_mass - weight_mass).abs() > 0.12 * weight_mass {
                    Some(Refusal::MassDisagreement)
                } else {
                    None
                }
            });
        Estimate {
            weighted_plateaus_n: mean,
            plateau_std_n: sd,
            stable_grip_ticks: self.windows.each_ref().map(|w| w.bilateral),
            stable_lift_ticks: self.windows.each_ref().map(|w| w.stable_lift),
            rail_endpoints_m: self.positions,
            horizontal_force_moment_ns2: self.horizontal_force_moment,
            inertial_mass_kg: inertial,
            apparent_weight_mass_kg: plateau_mass,
            travel_ticks: None,
            mass_kg: plateau_mass.filter(|_| rejected.is_none()),
            refused: rejected,
        }
    }
}
